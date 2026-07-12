#include "calibrator_impl.hpp"

#include "axis.hpp"
#include "utils.hpp"

#include <FreeRTOSConfig.h> // for configTOTAL_HEAP_SIZE

constexpr uint32_t MAX_CCMRAM = (65536 - configTOTAL_HEAP_SIZE);
static_assert(CALIB_MAX_SAMPLES * 2 * sizeof(CalibratorSample) <= MAX_CCMRAM);

#if CALIB_ALLOC_SAMPLES
__attribute__((section(".ccmram")))
static CalibratorSample calib_samples_1[CALIB_MAX_SAMPLES];

__attribute__((section(".ccmram")))
static CalibratorSample calib_samples_2[CALIB_MAX_SAMPLES];

CalibratorSample* CalibratorImpl::getSamples1() { return calib_samples_1; }
CalibratorSample* CalibratorImpl::getSamples2() { return calib_samples_2; }
#else
CalibratorSample* CalibratorImpl::getSamples1() { return nullptr; }
CalibratorSample* CalibratorImpl::getSamples2() { return nullptr; }
#endif

#if CALIB_RECCORD_I
static_assert(CALIB_MAX_I_SAMPLES * sizeof(CalibratorSample) <= MAX_CCMRAM);

__attribute__((section(".ccmram")))
static CalibratorSample calib_I_samples[CALIB_MAX_I_SAMPLES];

CalibratorSample* CalibratorImpl::getISamples() { return calib_I_samples; }
#endif

CalibratorUpdateHandler CalibratorImpl::empty_update_handler_;

bool CalibratorImpl::update(uint32_t i_meas_timestamp, uint32_t ctrl_timestamp)
{
    CalibratorUpdateHandler::ProcessArgs args;

    const uint32_t prim = cpu_enter_critical();
    // encoder IRQ is disabled inside this critical section,
    // so args.update_timestamp should be after last_enc_time!
    args.update_timestamp = DWT->CYCCNT;
#if ENC_TIME_FROM_TIMER
    extern volatile uint32_t _enc_last_time[2];
    extern volatile int16_t _enc_last_count[2];
    const int axis_num = axis_->axis_num_;
    args.enc_time = _enc_last_time[axis_num];
    args.enc_count = _enc_last_count[axis_num];
#endif
#if ENC_TIME_FROM_GPIO
    const auto& encoder = axis_->encoder_;
    args.enc_time = encoder.last_enc_time_;
    args.enc_count = encoder.last_enc_count_;
#endif
    cpu_exit_critical(prim);

    args.i_meas_timestamp = i_meas_timestamp;

    args.phase_dist = *axis_->open_loop_controller_.total_distance_.any();
    // correct phase_dist for args.update_timestamp from value for ctrl_timestamp
    const float phase_vel = *axis_->open_loop_controller_.phase_vel_.any();
    args.phase_dist += phase_vel * (float(int32_t(args.update_timestamp - ctrl_timestamp)) / float(CLOCK_HZ));

    const Iph_ABC_t Iph_ofs1 = axis_->motor_.prev_ofs_current_.value_or(Iph_ABC_t{0.0f, 0.0f, 0.0f});
    const Iph_ABC_t Iph_ofs2 = axis_->motor_.last_ofs_current_.value_or(Iph_ABC_t{0.0f, 0.0f, 0.0f});
    const Iph_ABC_t Iph_raw = axis_->motor_.last_raw_current_.value_or(Iph_ABC_t{0.0f, 0.0f, 0.0f});

    const Iph_ABC_t Iph_meas = axis_->motor_.current_meas_.value_or(Iph_ABC_t{0.0f, 0.0f, 0.0f});
#if CALIB_RECCORD_I
    if (enable_reccord_I_ && reccord_I_idx_ < CALIB_MAX_I_SAMPLES) {
        auto& s = calib_I_samples[reccord_I_idx_++];
        // Ialpha_raw
        s.Id = Iph_raw.phB - 0.5f * (Iph_ofs1.phB + Iph_ofs2.phB);
        // Ialpha_ofs
        s.Iq = Iph_meas.phB;
    }
    if (enable_reccord_I_x3_ && reccord_I_idx_ < CALIB_MAX_I_SAMPLES) {
        auto& s = calib_I_samples[reccord_I_idx_++];
        // Ialpha_raw
        s.Id = Iph_raw.phC - 0.5f * (Iph_ofs1.phC + Iph_ofs2.phC);
        // Ialpha_ofs
        s.Iq = Iph_meas.phC;
    }
#endif

    // Iph_raw should be measured right in the middle between Iph_ofs1 & Iph_ofs2!
    const float I_phB = Iph_raw.phB - 0.5f * (Iph_ofs1.phB + Iph_ofs2.phB);
    const float I_phC = Iph_raw.phC - 0.5f * (Iph_ofs1.phC + Iph_ofs2.phC);

    args.Ialpha = -(I_phB + I_phC);
    args.Ibeta = one_by_sqrt3 * (I_phB - I_phC);

    last_enc_count_ = args.enc_count;
    last_enc_time_ = args.enc_time;
    last_phase_dist_ = args.phase_dist;

    if (!update_handler_->process(args)) {
        // stop motor phase
        axis_->open_loop_controller_.target_vel_ = 0.0f;
    }
    return true;
}

int32_t CalibratorImpl::getEncoderCPR() const
{
    return axis_->encoder_.config_.cpr;
}

int32_t CalibratorImpl::getMotorPolePairs() const
{
    return axis_->motor_.config_.pole_pairs;
}

void CalibratorImpl::initMotor()
{
    CRITICAL_SECTION() {
        // Reset state variables
        axis_->open_loop_controller_.Idq_setpoint_ = {0.0f, 0.0f};
        axis_->open_loop_controller_.Vdq_setpoint_ = {0.0f, 0.0f};
        axis_->open_loop_controller_.phase_ = 0.0f;
        axis_->open_loop_controller_.phase_vel_ = 0.0f;

        axis_->open_loop_controller_.max_phase_vel_ramp_ = INFINITY;

        axis_->open_loop_controller_.max_current_ramp_ = 0.0f;
        axis_->open_loop_controller_.max_voltage_ramp_ = 0.0f;
        axis_->open_loop_controller_.target_current_ = 0.0f;
        axis_->open_loop_controller_.target_voltage_ = 0.0f;
        axis_->motor_.current_control_.enable_current_control_src_ = false;

        axis_->open_loop_controller_.target_vel_ = 0.0f;
        axis_->open_loop_controller_.total_distance_ = 0.0f;
        axis_->open_loop_controller_.phase_ = axis_->open_loop_controller_.initial_phase_ = 0.0;

        axis_->motor_.current_control_.Idq_setpoint_src_.connect_to(&axis_->open_loop_controller_.Idq_setpoint_);
        axis_->motor_.current_control_.Vdq_setpoint_src_.connect_to(&axis_->open_loop_controller_.Vdq_setpoint_);
        
        axis_->motor_.current_control_.phase_src_.connect_to(&axis_->open_loop_controller_.phase_);
        axis_->acim_estimator_.rotor_phase_src_.connect_to(&axis_->open_loop_controller_.phase_);

        axis_->motor_.phase_vel_src_.connect_to(&axis_->open_loop_controller_.phase_vel_);
        axis_->motor_.current_control_.phase_vel_src_.connect_to(&axis_->open_loop_controller_.phase_vel_);
        axis_->acim_estimator_.rotor_phase_vel_src_.connect_to(&axis_->open_loop_controller_.phase_vel_);
    }
    axis_->wait_for_control_iteration();

    axis_->motor_.arm(&axis_->motor_.current_control_);
}

void CalibratorImpl::shutdownMotor()
{
    if (axis_->motor_.is_armed_) {
        axis_->motor_.disarm();
    }
}

bool CalibratorImpl::isAbortOrMotorError()
{
    if (!axis_->motor_.is_armed_) {
        return true; // TODO: return "disarmed" error code
    }
    if (axis_->requested_state_ != Axis::AXIS_STATE_UNDEFINED) {
        return true; // TODO: return "aborted" error code
    }
    return false;
}

void CalibratorImpl::setEncoderError()
{
    axis_->encoder_.set_error(Encoder::ERROR_NO_RESPONSE);
}

void CalibratorImpl::setEncoderReady(int32_t init_enc_count, float delta_enc, int32_t phase_dir)
{
    auto& encoder = axis_->encoder_;

    CRITICAL_SECTION() {
        encoder.shadow_count_ = encoder.count_in_cpr_ = last_enc_count_;
    }

    float delta_enc_i;
    float delta_enc_f = modff(delta_enc, &delta_enc_i);

    encoder.config_.phase_offset = init_enc_count + int(delta_enc_i);
    encoder.config_.phase_offset_float = delta_enc_f + 0.5f; // 0.5f for center-aligned reference at (init_enc_count + 0.5f)
    encoder.config_.direction = phase_dir;
    encoder.is_ready_ = true;
}

void CalibratorImpl::setControlParams(const ControlParams& params)
{
    auto& controller = axis_->open_loop_controller_;

    if (params.phase_dist.has_value()) {
        controller.total_distance_ = *params.phase_dist;
        controller.phase_ = controller.initial_phase_ = wrap_pm_pi(*params.phase_dist);
    }
    if (params.phase_vel.has_value()) {
        controller.target_vel_ = *params.phase_vel;
    }

    if (params.voltage_value_and_time.has_value()) {
        auto [target_voltage, voltage_change_time] = *params.voltage_value_and_time;
        controller.max_voltage_ramp_ = fabs(target_voltage - controller.target_voltage_) / voltage_change_time;
        controller.target_voltage_ = target_voltage;
    }
}
