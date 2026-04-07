#include "odrive_main.h"
#include <Drivers/STM32/stm32_system.h>
#include <FreeRTOSConfig.h>

#include <float.h>

#define DEBUG_LOG 1
#define DEBUG_CURRENT 0
#define DEBUG_DETECT_LIMIT 0


BaseUpdateHandler Calibrator::empty_update_handler_;


constexpr uint32_t MAX_CCMRAM = (65536 - configTOTAL_HEAP_SIZE);
struct RecSample { float Id, Iq; };
#if RECORD_SAMPLES
constexpr uint32_t MAX_SAMPLES = MAX_CCMRAM / (2 * sizeof(RecSample));
static_assert(MAX_SAMPLES * 2 * sizeof(RecSample) <= MAX_CCMRAM);
#else
constexpr uint32_t MAX_SAMPLES = 0;
#endif
__attribute__((section(".ccmram")))
static RecSample rec_samples_1[MAX_SAMPLES];
static RecSample rec_samples_2[MAX_SAMPLES];

#if DEBUG_DETECT_LIMIT
struct RecDetectLimit
{
    uint32_t enc_dt;
    //int32_t enc_count;
    //float Iangle;
    //float phase;
};
constexpr uint32_t MAX_ENC_TIME_IDX = MAX_CCMRAM / sizeof(RecDetectLimit);
__attribute__((section(".ccmram")))

static RecDetectLimit rec_detect_limit[MAX_ENC_TIME_IDX];
#endif


Calibrator::Calibrator()
{
}

bool Calibrator::update(uint32_t current_meas_timestamp)
{
    BaseUpdateHandler::ProcessArgs args;
    args.current_meas_timestamp = current_meas_timestamp;

    const uint32_t prim = cpu_enter_critical();
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

    last_enc_time_ = args.enc_time;
    last_enc_count_ = args.enc_count;

    std::tie(args.Ialpha, args.Ibeta) = axis_->motor_.current_control_.Ialpha_beta_measured_.value_or(float2D{0.0f, 0.0f});
    
    args.phase = *axis_->open_loop_controller_.total_distance_.any();

    args.Iph_raw = axis_->motor_.last_raw_current_.value_or(Iph_ABC_t{0.0f, 0.0f, 0.0f});

    const Iph_ABC_t Iph_ofs1 = axis_->motor_.prev_ofs_current_.value_or(Iph_ABC_t{0.0f, 0.0f, 0.0f});
    const Iph_ABC_t Iph_ofs2 = axis_->motor_.last_ofs_current_.value_or(Iph_ABC_t{0.0f, 0.0f, 0.0f});
    args.Iph_ofs = Iph_ABC_t{(Iph_ofs1.phA + Iph_ofs2.phA)*0.5f, (Iph_ofs1.phB + Iph_ofs2.phB)*0.5f, (Iph_ofs1.phC + Iph_ofs2.phC)*0.5f};

    if (!update_handler_->process(args)) {
        // stop motor phase
        axis_->open_loop_controller_.target_vel_ = 0.0f;
    }
    return true;
}


class MeasureCurrentUpdateHandler : public BaseUpdateHandler
{
public:
    MeasureCurrentUpdateHandler() = default;
    virtual ~MeasureCurrentUpdateHandler() = default;

    void restart()
    {
        Ialpha_accum_ = 0.0f;
        Ibeta_accum_ = 0.0f;
        accum_count_ = 0;
    }

    virtual bool process(const ProcessArgs& args) override
    {
        Ialpha_accum_ += args.Ialpha;
        Ibeta_accum_ += args.Ibeta;
        ++accum_count_;
        return true;
    }

    float getIalpha() const {
        return Ialpha_accum_ / float(accum_count_);
    }
    float getIbeta() const {
        return Ibeta_accum_ / float(accum_count_);
    }
    uint32_t getCount() const {
        return accum_count_;
    }

private:
    float Ialpha_accum_;
    float Ibeta_accum_;
    uint32_t accum_count_;
};

class DetectMotionUpdateHandler : public BaseUpdateHandler
{
public:
    DetectMotionUpdateHandler() = default;
    virtual ~DetectMotionUpdateHandler() = default;

    void init(float phase_dir)
    {
        phase_dir_ = phase_dir;
    }

    void restart(int32_t enc_step_dir, int32_t start_enc_count, int32_t start_enc_ofs)
    {
        enc_step_dir_ = enc_step_dir;
        start_enc_count_ = start_enc_count;
        detect_enc_count_ = start_enc_count * enc_step_dir + start_enc_ofs;

        is_detected_ = false;
    }

    virtual bool process(const ProcessArgs& args) override
    {
        if (is_detected_) {
            return false;
        }
        if (args.enc_count * enc_step_dir_ > detect_enc_count_) {
            is_detected_ = true;
            detected_enc_count_ = args.enc_count;
            detected_phase_ = args.phase;
            return false;
        }
        return true;
    }

    inline bool isDetected() const { return is_detected_; }

    inline float getPhaseDir() const { return phase_dir_; }

    float evalStartPhase(float enc2phase)
    {
        return detected_phase_ + float(start_enc_count_ - detected_enc_count_) * phase_dir_ * enc2phase;
    }

private:
    float phase_dir_;

    int32_t enc_step_dir_;
    int32_t start_enc_count_;
    int32_t detect_enc_count_;

    bool is_detected_;
    int32_t detected_enc_count_;
    float detected_phase_;
};        


class VelocityFilter_SG7
{
public:
    static constexpr uint32_t Len = 6;

    VelocityFilter_SG7() = default;

    void restart(float dist_step)
    {
        dist_step_ = dist_step;
        last_idx_ = 0;
    }

    void add(float dt)
    {
        buffer_[last_idx_] = dt;
        last_idx_ = (last_idx_ + 1) & IDX_MASK;
    }

    float eval() const
    {
        float dt_sum = (buffer_[(last_idx_ - 6) & IDX_MASK] + buffer_[(last_idx_ - 1) & IDX_MASK]) * (3.0f / 28)
                     + (buffer_[(last_idx_ - 5) & IDX_MASK] + buffer_[(last_idx_ - 2) & IDX_MASK]) * (5.0f / 28)
                     + (buffer_[(last_idx_ - 4) & IDX_MASK] + buffer_[(last_idx_ - 3) & IDX_MASK]) * (6.0f / 28);
        return dist_step_ / dt_sum;
    }

private:
    float dist_step_;
    uint32_t last_idx_;

    static constexpr int BUFF_SIZE = 8; // must be 2^n >= Len
    static constexpr int IDX_MASK = BUFF_SIZE - 1;
    float buffer_[BUFF_SIZE];
};

class GeneralFilter_SG7
{
public:
    static constexpr uint32_t Len = 7;

    GeneralFilter_SG7() = default;

    void restart()
    {
        last_idx_ = 0;
    }

    void add(float value)
    {
        buffer_[last_idx_] = value;
        last_idx_ = (last_idx_ + 1) & IDX_MASK;
    }

    float eval() const
    {
        float sum = (buffer_[(last_idx_ - 7) & IDX_MASK] + buffer_[(last_idx_ - 1) & IDX_MASK]) * (-2.0f / 21)
                  + (buffer_[(last_idx_ - 6) & IDX_MASK] + buffer_[(last_idx_ - 2) & IDX_MASK]) * ( 3.0f / 21)
                  + (buffer_[(last_idx_ - 5) & IDX_MASK] + buffer_[(last_idx_ - 3) & IDX_MASK]) * ( 6.0f / 21)
                                                         + buffer_[(last_idx_ - 4) & IDX_MASK]  * ( 7.0f / 21);
        return sum;
    }

private:
    uint32_t last_idx_;

    static constexpr int BUFF_SIZE = 8; // must be 2^n >= Len
    static constexpr int IDX_MASK = BUFF_SIZE - 1;
    float buffer_[BUFF_SIZE];
};


struct SamplingStartState
{
    int32_t enc_step_dir_;
    int32_t enc_edge_ofs_;

    int32_t start_enc_edge_pos_;
    int32_t start_sample_idx_;

    int32_t filter_start_enc_edge_pos_;
};

template <typename TLim, typename TRec>
class SamplingUpdateHandler : public BaseUpdateHandler
{
    friend class DetectLimitImpl;

public:
    static constexpr uint32_t start_change_count_ = 30;
    using Velocity_Filter_t = VelocityFilter_SG7;
    using TorqueI_Filter_t = GeneralFilter_SG7;
    static_assert(TorqueI_Filter_t::Len == Velocity_Filter_t::Len + 1);

    static constexpr float phase_speed_threshold_ = 3.0f;


    SamplingUpdateHandler() {}
    virtual ~SamplingUpdateHandler() = default;

    void init(int32_t init_enc_count, float phase_dir, float phase_speed, uint32_t sample_size, int32_t enc_cpr, int32_t pole_pairs)
    {
        init_enc_count_ = init_enc_count;
        phase_dir_ = phase_dir;
        sample_size_ = sample_size;

        enc_motor_step_ = 2.0f * M_PI / float(enc_cpr);
        enc_phase_step_ = enc_motor_step_ * float(pole_pairs);
        motor_acc_scale_ = 0.5f / (sample_size * enc_motor_step_);

        // min delta time to filter very fast encoder changes
        enc_min_delta_time_ = uint32_t(TIM_1_8_CLOCK_HZ * enc_phase_step_ / (phase_speed * phase_speed_threshold_));
    }

    inline float getPhaseDir() const { return phase_dir_; }
    inline int32_t getSampleIdx() const { return sample_idx_; }

    SamplingStartState calcStartState(int32_t start_enc_edge_pos_, int32_t start_sample_idx,
                                      int32_t enc_step_dir, int32_t limit_enc_edge_pos, int32_t limit_enc_ofs)
    {
        const int32_t min_enc_edge_pos = limit_enc_edge_pos * enc_step_dir + limit_enc_ofs;

        int32_t filter_start_enc_edge_pos_ = start_enc_edge_pos_ * enc_step_dir - (Velocity_Filter_t::Len >> 1);
        while (filter_start_enc_edge_pos_ < min_enc_edge_pos) {
            start_enc_edge_pos_ += sample_size_ * enc_step_dir;
            start_sample_idx += enc_step_dir;

            filter_start_enc_edge_pos_ += sample_size_;
        }

        const int32_t enc_edge_ofs = -((enc_step_dir - 1) >> 1); // fwd -> 0, back -> 1
        return SamplingStartState{enc_step_dir, enc_edge_ofs, start_enc_edge_pos_, start_sample_idx, filter_start_enc_edge_pos_};
    }

    void restart(const SamplingStartState& start_state, int32_t curr_enc_count, uint32_t curr_enc_time)
    {
        start_state_ = start_state;

        last_enc_edge_pos_ = curr_enc_count + start_state.enc_edge_ofs_;
        last_enc_time_ = curr_enc_time;

        sub_sample_idx_ = 0;
        sample_idx_ = start_state.start_sample_idx_;

        enc_I_phB_accum_ = 0.0f;
        enc_I_phC_accum_ = 0.0f;
        enc_accum_count_ = 0;

        vel_filter_.restart(enc_motor_step_ * start_state.enc_step_dir_);
        Iq_filter_.restart();
        Id_filter_.restart();
        filter_add_count_ = 0;

        smooth_vel_accum_ = 0.0f;
        smooth_Iq_accum_ = 0.0f;
        smooth_Id_accum_ = 0.0f;

        limit_impl_.onRestart(start_state, curr_enc_count, curr_enc_time);
    }

#if DEBUG_CURRENT
    int32_t record_idx_ = 0;
    bool record_enabled_ = false;
#endif

    virtual bool process(const ProcessArgs& args) override
    {
        if (!limit_impl_.onProcess(args, enc_motor_step_, start_state_.enc_step_dir_)) {
            return false;
        }

        auto do_enc_accum = [this](const ProcessArgs& args) {
            enc_I_phB_accum_ += (args.Iph_raw.phB - args.Iph_ofs.phB);
            enc_I_phC_accum_ += (args.Iph_raw.phC - args.Iph_ofs.phC);
            ++enc_accum_count_;
        };

        const bool do_enc_accum_before = (args.enc_time > args.current_meas_timestamp);
        if (do_enc_accum_before) {
            do_enc_accum(args);
        }

        const int32_t enc_edge_pos = args.enc_count + start_state_.enc_edge_ofs_;
        const int32_t abs_delta_count = (enc_edge_pos - last_enc_edge_pos_) * start_state_.enc_step_dir_;
        // pre-filter encoder jitter
        if (abs_delta_count > 0) {
            const uint32_t delta_time = args.enc_time - last_enc_time_;
            // pre-filter encoder bad timing
            if (delta_time >= enc_min_delta_time_ * abs_delta_count) {
                const float avg_I_phB = enc_I_phB_accum_ / float(enc_accum_count_);
                const float avg_I_phC = enc_I_phC_accum_ / float(enc_accum_count_);

                enc_I_phB_accum_ = 0.0f;
                enc_I_phC_accum_ = 0.0f;
                enc_accum_count_ = 0;

                const float avg_Ialpha = -(avg_I_phB + avg_I_phC);
                const float avg_Ibeta = one_by_sqrt3 * (avg_I_phB - avg_I_phC);
#if DEBUG_CURRENT
                if (record_enabled_ && record_idx_ < MAX_SAMPLES) {
                    rec_samples_1[record_idx_] = RecSample{avg_I_raw_phB - avg_I_bias_phB, avg_I_raw_phC - avg_I_bias_phC};
                    rec_samples_2[record_idx_] = RecSample{avg_I_meas_phB, avg_I_meas_phC};
                    ++record_idx_;
                }
#endif

                // init_enc_pos = init_enc_count + 0.5, thus -1 in this formula!
                const float avg_enc_pos = ((enc_edge_pos - init_enc_count_) + (last_enc_edge_pos_ - init_enc_count_) - 1) * 0.5f;
                const float avg_enc_phase = avg_enc_pos * phase_dir_ * enc_phase_step_;

                const float cos_phase = cosf(avg_enc_phase);//our_arm_cos_f32(avg_enc_phase);
                const float sin_phase = sinf(avg_enc_phase);//our_arm_sin_f32(avg_enc_phase);
                const float step_Iq = cos_phase * avg_Ibeta - sin_phase * avg_Ialpha;
                const float step_Id = sin_phase * avg_Ibeta + cos_phase * avg_Ialpha;

                // after pre-filters abs_delta_count could be > 1 !!!
                const float step_dt = float(delta_time) / float(abs_delta_count * TIM_1_8_CLOCK_HZ);
                for (int32_t i = 0; i < abs_delta_count; ++i) {
                    if (enc_edge_pos * start_state_.enc_step_dir_ + i < start_state_.filter_start_enc_edge_pos_) {
                        continue;
                    }

                    vel_filter_.add(step_dt);
                    Iq_filter_.add(step_Iq);
                    Id_filter_.add(step_Id);
                    ++filter_add_count_;
                    if (filter_add_count_ < Velocity_Filter_t::Len) {
                        continue;
                    }
                    const float smooth_edge_vel = vel_filter_.eval();

                    if (filter_add_count_ == Velocity_Filter_t::Len) {
                        first_smooth_edge_vel_ = last_smooth_edge_vel_ = smooth_edge_vel;
                        continue;
                    }
                    // filter_add_count_ should be >= TorqueI_Filter_t::Len

                    // velocity is measured at enc change edge - so 1-order function integral
                    smooth_vel_accum_ += 0.5f * (last_smooth_edge_vel_ + smooth_edge_vel);
                    // I is measured between enc changes - so 0-order function integral
                    smooth_Iq_accum_ += Iq_filter_.eval();
                    smooth_Id_accum_ += Id_filter_.eval();

                    last_smooth_edge_vel_ = smooth_edge_vel;

                    ++sub_sample_idx_;
                    if (sub_sample_idx_ == sample_size_) {
                        sub_sample_idx_ = 0;

                        const float v_dif = last_smooth_edge_vel_ - first_smooth_edge_vel_;
                        const float v_sum = last_smooth_edge_vel_ + first_smooth_edge_vel_;
                        const float sample_acc = std::abs(v_sum) * v_dif * motor_acc_scale_;
                        first_smooth_edge_vel_ = last_smooth_edge_vel_;

                        const float sample_vel = smooth_vel_accum_ / float(sample_size_);
                        const float sample_Iq = smooth_Iq_accum_ / float(sample_size_);
                        const float sample_Id = smooth_Id_accum_ / float(sample_size_);
                        // reset accum
                        smooth_vel_accum_ = 0.0f;
                        smooth_Iq_accum_ = 0.0f;
                        smooth_Id_accum_ = 0.0f;

                        record_impl_.onSample(sample_idx_, sample_Id, sample_Iq, sample_vel, sample_acc);

                        sample_idx_ += start_state_.enc_step_dir_;
                    }
                }
                last_enc_edge_pos_ = enc_edge_pos;
                last_enc_time_ = args.enc_time;
            }
        }
        if (!do_enc_accum_before) {
            do_enc_accum(args);
        }
        return true;
    }

    TLim& getLimitImpl() { return limit_impl_; }
    TRec& getRecordImpl() { return record_impl_; }

private:
    TLim limit_impl_;
    TRec record_impl_;

    int32_t init_enc_count_;
    float phase_dir_;
    uint32_t sample_size_;
    uint32_t enc_min_delta_time_;

    float enc_motor_step_;
    float enc_phase_step_;
    float motor_acc_scale_;

    SamplingStartState start_state_;

    int32_t last_enc_edge_pos_;
    uint32_t last_enc_time_;

    uint32_t sub_sample_idx_;
    int32_t sample_idx_;

    float enc_I_phB_accum_;
    float enc_I_phC_accum_;
    uint32_t enc_accum_count_;

    Velocity_Filter_t vel_filter_;
    TorqueI_Filter_t Iq_filter_;
    TorqueI_Filter_t Id_filter_;
    uint32_t filter_add_count_;

    float smooth_vel_accum_;
    float smooth_Iq_accum_;
    float smooth_Id_accum_;

    float first_smooth_edge_vel_; // first in a sample (doesn't change inside one sample)
    float last_smooth_edge_vel_; // last in a sample (change inside one sample)
};


class SampleRecordImpl
{
public:
    SampleRecordImpl() = default;

    void init(RecSample* rec_samples) { rec_samples_ = rec_samples; }

    void onSample(int32_t sample_idx, float Id, float Iq, float vel, float acc)
    {
        if (rec_samples_ && sample_idx >= 0 && sample_idx < MAX_SAMPLES) {
            RecSample& s = rec_samples_[sample_idx];
            s.Id += Id * 0.5f; s.Iq += Iq * 0.5f; // s.vel += vel;
        }
    }

private:
    RecSample* rec_samples_ = nullptr;
};

class DetectLimitImpl
{
public:
    DetectLimitImpl() = default;

    void init(float vel_threshold, uint32_t warm_up_count)
    {
        vel_threshold_ = vel_threshold;
        warm_up_count_ = warm_up_count;
    }

    void onRestart(const SamplingStartState& start_state, int32_t curr_enc_count, uint32_t curr_enc_time)
    {
        count_ = 0;

        limit_enc_count_ = curr_enc_count;
        limit_enc_time_ = curr_enc_time;

        is_found_ = false;
    }

#if DEBUG_DETECT_LIMIT
    uint32_t rec_enc_time_idx_ = 0;
    bool rec_enc_time_wrap_ = false;
#endif

    bool onProcess(const BaseUpdateHandler::ProcessArgs& args, float enc_motor_step, int32_t enc_step_dir)
    {
        if (is_found_) {
            return false;
        }
#if 0
        if (count_ == 0) {
            Ialpha_mean_ = args.Ialpha;
            Ibeta_mean_ = args.Ibeta;
        }
        Ialpha_mean_ += 0.2f * (args.Ialpha - Ialpha_mean_);
        Ibeta_mean_ += 0.2f * (args.Ibeta - Ibeta_mean_);

        const float Iangle = atan2f(Ibeta_mean_, Ialpha_mean_);

        rec_detect_limit[rec_enc_time_idx_] = RecDetectLimit{args.enc_count, Iangle, wrap_pm_pi(args.phase)};
        ++rec_enc_time_idx_;
        if (rec_enc_time_idx_ == MAX_ENC_TIME_IDX) {
            rec_enc_time_idx_ = 0;
            rec_enc_time_wrap_ = true;
        }
#endif

        if (args.enc_count * enc_step_dir > limit_enc_count_ * enc_step_dir) {
            const float delta_time = float(args.enc_time - limit_enc_time_);
            if (count_ == 0) {
                delta_time_mean_ = float(delta_time);
                delta_time_var_ = 0.0f;
            }
            constexpr float alpha = 0.01f;
            delta_time_mean_ += alpha * (delta_time - delta_time_mean_);
            const float diff = delta_time - delta_time_mean_;
            delta_time_var_ += alpha * (diff * diff - delta_time_var_);
            ++count_;

            limit_enc_count_ = args.enc_count;
            limit_enc_time_ = args.enc_time;
        }

        if (count_ >= warm_up_count_) {
            const float delta_time = float(DWT->CYCCNT - limit_enc_time_);
            const float delta_time_threshold = delta_time_mean_ + 5.0f * sqrtf(delta_time_var_);
#if DEBUG_DETECT_LIMIT
            rec_detect_limit[rec_enc_time_idx_] = RecDetectLimit{ enc_delta_time };
            ++rec_enc_time_idx_;
            if (rec_enc_time_idx_ == MAX_ENC_TIME_IDX) {
                rec_enc_time_idx_ = 0;
                rec_enc_time_wrap_ = true;
            }
#endif            
            if (delta_time > delta_time_threshold) {
                // motor limit detected
                is_found_ = true;
                return false;
            }
        }
        return true;
    }

    inline bool isFinished() const { return is_found_; }
    inline int32_t getFoundLimit() const {
        return limit_enc_count_;
    }

private:
    float vel_threshold_;
    uint32_t warm_up_count_;

    uint32_t count_;

    float delta_time_mean_;
    float delta_time_var_;

    int32_t limit_enc_count_;
    uint32_t limit_enc_time_;

    volatile bool is_found_;
};


class StopAtLimitImpl
{
public:
    StopAtLimitImpl() = default;

    void init(int32_t lo_enc_count, int32_t hi_enc_count)
    {
        lo_enc_count_ = lo_enc_count;
        hi_enc_count_ = hi_enc_count;
    }

    void onRestart(const SamplingStartState& start_state, int32_t curr_enc_count, uint32_t curr_enc_time)
    {
        is_stopped_ = false;
        stop_enc_count_ = std::max(lo_enc_count_ * start_state.enc_step_dir_, hi_enc_count_ * start_state.enc_step_dir_);
    }

    bool onProcess(const BaseUpdateHandler::ProcessArgs& args, float enc_motor_step, int32_t enc_step_dir)
    {
        is_stopped_ |= (args.enc_count * enc_step_dir >= stop_enc_count_);
        return !is_stopped_;
    }

    inline bool isFinished() const { return is_stopped_; }

private:
    int32_t lo_enc_count_;
    int32_t hi_enc_count_;

    bool is_stopped_;
    int32_t stop_enc_count_;
};


template <typename F>
bool Calibrator::run_motor(F&& func, float timeout_seconds, bool error_on_timeout)
{
    size_t timeout_count = size_t(timeout_seconds * 1000.0f);
    for (size_t i = 0; i < timeout_count; ++i) {
        if (!axis_->motor_.is_armed_) {
            return false; // TODO: return "disarmed" error code
        }
        if (axis_->requested_state_ != Axis::AXIS_STATE_UNDEFINED) {
            axis_->motor_.disarm();
            return false; // TODO: return "aborted" error code
        }
        if (func()) {
            return true;
        }
        osDelay(1);
    }
    if (error_on_timeout)
    {
        axis_->motor_.disarm();
#if DEBUG_LOG
        printf("timeout: %f\n", timeout_seconds);
        osDelay(10);
#endif
        return false;
    }
    return true;
}

bool Calibrator::run_motor_for_time(float duration_seconds)
{
    return run_motor([](){ return false; }, duration_seconds, /*error_on_timeout=*/false);
}

class CalibratorSafeReturn
{
public:
    CalibratorSafeReturn(Calibrator& calibrator) : calibrator_(calibrator) {}
    ~CalibratorSafeReturn() {
        CRITICAL_SECTION() {
            calibrator_.update_handler_ = &Calibrator::empty_update_handler_;
        }
        auto axis = calibrator_.axis_;
        if (axis && axis->motor_.is_armed_) {
            axis->motor_.disarm();
        }
    }

private:
    Calibrator& calibrator_;
};

template <typename T>
bool Calibrator::run_record_pass(T& update_handler, const SamplingStartState& start_state, std::optional<float> start_phase,
                                 RecSample* rec_samples, float timeout)
{
    if (start_phase.has_value()) {
        CRITICAL_SECTION() {
            axis_->open_loop_controller_.total_distance_ = *start_phase;
            axis_->open_loop_controller_.phase_ = axis_->open_loop_controller_.initial_phase_ = wrap_pm_pi(*start_phase);
        }
        // settle at start_phase
        if (!run_motor_for_time(config_.phase_settle_duration)) {
            return false;
        }
    }
#if RECORD_SAMPLES
    update_handler.getRecordImpl().init(rec_samples);
#endif
    const float vel_dir = update_handler.getPhaseDir() * start_state.enc_step_dir_;
    CRITICAL_SECTION() {
        update_handler.restart(start_state, last_enc_count_, last_enc_time_);
        update_handler_= &update_handler;

        axis_->open_loop_controller_.target_vel_ = vel_dir * config_.record_phase_speed;
    }
    // run record pass
    if (!run_motor([&](){ return update_handler.getLimitImpl().isFinished(); }, timeout)) {
        return false;
    }
    CRITICAL_SECTION() {
        // target_vel_ = 0.0 after update_handler is finished
        update_handler_ = &Calibrator::empty_update_handler_;
    }
    return true;
}


bool Calibrator::run_offset_calibration()
{
    if (axis_ == nullptr) {
        return false;
    }

#if RECORD_SAMPLES
    memset(rec_samples_1, 0, sizeof(RecSample) * MAX_SAMPLES);
    memset(rec_samples_2, 0, sizeof(RecSample) * MAX_SAMPLES);
#endif

    CalibratorSafeReturn _safe_return(*this);


    auto& encoder = axis_->encoder_;
    const int32_t enc_cpr = encoder.config_.cpr;
    const int32_t pole_pairs = axis_->motor_.config_.pole_pairs;

    const float degree2phase = float(pole_pairs) * 2.0f * M_PI / 360.0f;
    const float enc2phase = float(pole_pairs) * 2.0f * M_PI / float(enc_cpr);
    const float phase2enc = float(enc_cpr) / (float(pole_pairs) * 2.0f * M_PI);


    const float max_range_timeout = config_.max_motor_degree_range * degree2phase / config_.record_phase_speed;


    MeasureCurrentUpdateHandler measure_current_handler;


    SamplingUpdateHandler<DetectLimitImpl, SampleRecordImpl> detect_limit_with_record_handler;
    detect_limit_with_record_handler.getLimitImpl().init(config_.detect_limit_vel_threshold, config_.detect_limit_warm_up_count);

    SamplingUpdateHandler<StopAtLimitImpl, SampleRecordImpl> stop_at_limit_with_record_handler;

    DetectMotionUpdateHandler detect_motion_handler;

    auto run_detect_motion = [&](SamplingStartState start_state, int32_t start_enc_count, float& out_start_phase) {
        const float vel_dir = detect_motion_handler.getPhaseDir() * start_state.enc_step_dir_;
        CRITICAL_SECTION() {
            detect_motion_handler.restart(start_state.enc_step_dir_, start_enc_count, config_.stop_limit_enc_ofs);
            update_handler_ = &detect_motion_handler;

            axis_->open_loop_controller_.target_vel_ = vel_dir * config_.record_phase_speed;
        }
        if (!run_motor([&](){ return detect_motion_handler.isDetected(); }, config_.detect_motion_duration)) {
            return false;
        }
        CRITICAL_SECTION() {
            // target_vel_ is zero
            update_handler_ = &Calibrator::empty_update_handler_;
        }
        out_start_phase = detect_motion_handler.evalStartPhase(enc2phase);
        return true;
    };

    // We use shadow_count_ to do the calibration, but the offset is used by count_in_cpr_
    // Therefore we have to sync them for calibration
    //encoder.shadow_count_ = encoder.count_in_cpr_;

    CRITICAL_SECTION() {
        // Reset state variables
        axis_->open_loop_controller_.Idq_setpoint_ = {0.0f, 0.0f};
        axis_->open_loop_controller_.Vdq_setpoint_ = {0.0f, 0.0f};
        axis_->open_loop_controller_.phase_ = 0.0f;
        axis_->open_loop_controller_.phase_vel_ = 0.0f;

        axis_->open_loop_controller_.max_phase_vel_ramp_ = INFINITY;

        axis_->open_loop_controller_.max_current_ramp_ = 0.0f;
        axis_->open_loop_controller_.max_voltage_ramp_ = config_.start_lock_voltage / config_.start_lock_settle_duration * 2.0f;
        axis_->open_loop_controller_.target_current_ = 0.0f;
        axis_->open_loop_controller_.target_voltage_ = config_.start_lock_voltage;
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

    // go to start position for start_lock_settle_duration
    if (!run_motor_for_time(config_.start_lock_settle_duration)) {
        return false;
    }

    CRITICAL_SECTION() {
        measure_current_handler.restart();
        update_handler_ = &measure_current_handler;
    }
    // stay at start position and measure current
    if (!run_motor_for_time(config_.start_lock_measure_duration)) {
        return false;
    }
    CRITICAL_SECTION() {
        update_handler_ = &Calibrator::empty_update_handler_;
    }

    const float Ialpha = measure_current_handler.getIalpha();
    const float Ibeta = measure_current_handler.getIbeta();
    const float R = config_.start_lock_voltage / Ialpha;
#if DEBUG_LOG
    printf("C: %u, Ia: %f, Ib: %f, R: %f\n", measure_current_handler.getCount(), Ialpha, Ibeta, R);
    osDelay(5);
#endif

    // change current to start_lock_current and settle
    CRITICAL_SECTION() {
        const float new_voltage = config_.start_lock_current * R;
        axis_->open_loop_controller_.max_voltage_ramp_ = std::abs(new_voltage - axis_->open_loop_controller_.target_voltage_)
                                                            / config_.start_lock_settle_duration * 2.0f;
        axis_->open_loop_controller_.target_voltage_ = new_voltage;
    }
    if (!run_motor_for_time(config_.start_lock_settle_duration)) {
        return false;
    }

    // save current enc_count as initial
    const int32_t init_enc_count = last_enc_count_;
#if DEBUG_LOG
    printf("enc0=%d\n", init_enc_count);
    osDelay(5);
#endif

    // change current to record_current_1 and settle
    CRITICAL_SECTION() {
        const float new_voltage = config_.record_current_1 * R;
        axis_->open_loop_controller_.max_voltage_ramp_ = std::abs(new_voltage - axis_->open_loop_controller_.target_voltage_)
                                                            / config_.start_lock_settle_duration * 2.0f;
        axis_->open_loop_controller_.target_voltage_ = new_voltage;
    }
    if (!run_motor_for_time(config_.start_lock_settle_duration)) {
        return false;
    }

    // scan phase forward and detect direction
    int32_t phase_dir = 0;

    CRITICAL_SECTION() {
        axis_->open_loop_controller_.target_vel_ = config_.record_phase_speed;
    }
    if (!run_motor([&](){
        const int32_t enc_count = last_enc_count_;
        if (abs(enc_count - init_enc_count) >= config_.detect_dir_enc_dist) {
            phase_dir = enc_count > init_enc_count ? 1 : -1;
            return true; // done
        }
        return false; // continue
    }, config_.detect_dir_timeout)) {
        // Encoder response error
        encoder.set_error(Encoder::ERROR_NO_RESPONSE);
        return false;
    }
    CRITICAL_SECTION() {
        // stop phase velocity
        axis_->open_loop_controller_.target_vel_ = 0.0f;
    }
    // direction detected!
#if DEBUG_LOG
    printf("enc=%d, ph=%f, ph_dir=%d\n", int32_t(last_enc_count_), *axis_->open_loop_controller_.total_distance_.any(), phase_dir);
    osDelay(5);
#endif

#if 0
    {
        // finish!!!
        axis_->motor_.disarm();
        osDelay(100);

        CRITICAL_SECTION() {
            encoder.shadow_count_ = encoder.count_in_cpr_ = last_enc_count_;
        }
        encoder.config_.direction = phase_dir;
        encoder.config_.phase_offset = init_enc_count;
        encoder.config_.phase_offset_float = 0.5f;
        encoder.is_ready_ = true;

        center_phase_ = 0.0f;
        center_enc_pos_ = center_phase_ * phase_dir * phase2enc + (encoder.config_.phase_offset + encoder.config_.phase_offset_float);
        lo_enc_pos_ = center_enc_pos_ - 1000;
        hi_enc_pos_ = center_enc_pos_ + 1000;
        enc2phase_ = enc2phase;
        phase2enc_ = phase2enc;

        return true;
    }
#endif

    detect_motion_handler.init(phase_dir);

    detect_limit_with_record_handler.init(init_enc_count, phase_dir, config_.record_phase_speed, config_.sample_size,
                                          encoder.config_.cpr, axis_->motor_.config_.pole_pairs);

    stop_at_limit_with_record_handler.init(init_enc_count, phase_dir, config_.record_phase_speed, config_.sample_size,
                                           encoder.config_.cpr, axis_->motor_.config_.pole_pairs);

    // 0. go in direction for encoder decreasing to detect lo-limit
    {
        const int32_t start_enc_edge_pos = last_enc_count_ + 1;
        SamplingStartState start_state = detect_limit_with_record_handler.calcStartState(
            start_enc_edge_pos, 0, -1/*backward*/, start_enc_edge_pos, 0);

        if (!run_record_pass(detect_limit_with_record_handler, start_state, std::nullopt, nullptr, max_range_timeout)) {
            return false;
        }
    }
    // get lo-limit
    const int32_t lo_enc_count = detect_limit_with_record_handler.getLimitImpl().getFoundLimit();
#if DEBUG_LOG
    printf("lo_lim=%d %f\n", lo_enc_count, *axis_->open_loop_controller_.total_distance_.any());
    osDelay(5);
#endif

#if DEBUG_DETECT_LIMIT
    // finish!!!
    axis_->motor_.disarm();

    printf("idx,enc_dt\n");
    osDelay(5);

    uint32_t rec_enc_time_beg = 0;
    uint32_t rec_enc_time_end = detect_limit_with_record_handler.getLimitImpl().rec_enc_time_idx_;
    if (detect_limit_with_record_handler.getLimitImpl().rec_enc_time_wrap_) {
        rec_enc_time_beg = rec_enc_time_end;
        rec_enc_time_end += MAX_ENC_TIME_IDX;
    }
    for (uint32_t rec_enc_time_idx = rec_enc_time_beg; rec_enc_time_idx < rec_enc_time_end; ++rec_enc_time_idx) {
        uint32_t idx = rec_enc_time_idx - rec_enc_time_beg;
        uint32_t read_idx = rec_enc_time_idx;
        if (read_idx >= MAX_ENC_TIME_IDX) {
            read_idx -= MAX_ENC_TIME_IDX;
        }
        const auto& rec = rec_detect_limit[read_idx];
        //printf("%u, %d, %f, %f\n", idx, rec.enc_count, rec.Iangle, rec.phase);
        printf("%u, %u\n", idx, rec.enc_dt);
        osDelay(5);
    }
    osDelay(100);
    return false;
#endif

    // 1. forward record pass with I ~= record_current_1
    const int32_t lo_enc_edge_pos = lo_enc_count + 1;  // lo_enc_count <= lo_enc_pos < lo_enc_count + 1
    SamplingStartState lo_start_state = detect_limit_with_record_handler.calcStartState(
        lo_enc_edge_pos, 0, 1/*forward*/, lo_enc_edge_pos, config_.record_limit_enc_ofs);
#if DEBUG_LOG
    printf("lo_start=%d (s=%d), %d\n", lo_start_state.start_enc_edge_pos_, lo_start_state.start_sample_idx_, lo_start_state.filter_start_enc_edge_pos_);
    osDelay(5);
#endif

    // 1a. detect motion after hitting lo-limit
    float lo_start_phase;
    if (!run_detect_motion(lo_start_state, lo_enc_count, lo_start_phase)) {
        return false;
    }
#if DEBUG_LOG
    printf("lo_start_ph=%f\n", lo_start_phase);
    osDelay(5);
#endif

#if DEBUG_CURRENT
    detect_limit_with_record_handler.record_idx_ = 0;
    detect_limit_with_record_handler.record_enabled_ = true;
#endif
    // 1b. settle and go in direction for encoder increasing to detect hi-limit and record samples
    if (!run_record_pass(detect_limit_with_record_handler, lo_start_state, lo_start_phase, rec_samples_1, max_range_timeout)) {
        return false;
    }
    // get hi-limit
    const int32_t hi_enc_count = detect_limit_with_record_handler.getLimitImpl().getFoundLimit();
#if DEBUG_LOG    
    printf("hi_lim=%d %f\n", hi_enc_count, *axis_->open_loop_controller_.total_distance_.any());
    osDelay(5);
#endif

#if DEBUG_CURRENT
    // finish!!!
    axis_->motor_.disarm();

    printf("~~~ rec_samples_1 ~~~\n");
    osDelay(5);
    printf("idx,Ib,Ic\n");
    osDelay(5);
    for (uint32_t idx = 0; idx < detect_limit_with_record_handler.record_idx_; ++idx)
    {
        const auto& s = rec_samples_1[idx];
        printf("%d, %f, %f\n", idx, s.Id, s.Iq);
        osDelay(5);
    }
    printf("~~~ rec_samples_2 ~~~\n");
    osDelay(5);
    printf("idx,Ib,Ic\n");
    osDelay(5);
    for (uint32_t idx = 0; idx < detect_limit_with_record_handler.record_idx_; ++idx)
    {
        const auto& s = rec_samples_2[idx];
        printf("%d, %f, %f\n", idx, s.Id, s.Iq);
        osDelay(5);
    }
    osDelay(100);
    return false;
#endif

    stop_at_limit_with_record_handler.getLimitImpl().init(lo_enc_count + config_.stop_limit_enc_ofs, hi_enc_count - config_.stop_limit_enc_ofs);

    // 2. backward record pass with I ~= record_current_1
    const int32_t end_sample_idx = detect_limit_with_record_handler.getSampleIdx();
    const int32_t end_enc_edge_pos = lo_start_state.start_enc_edge_pos_ + (end_sample_idx - lo_start_state.start_sample_idx_) * config_.sample_size;
    const int32_t hi_enc_edge_pos = hi_enc_count; // hi_enc_count <= hi_enc_pos < hi_enc_count + 1
    SamplingStartState hi_start_state = stop_at_limit_with_record_handler.calcStartState(
        end_enc_edge_pos, end_sample_idx - 1, -1/*backward*/, hi_enc_edge_pos, config_.record_limit_enc_ofs);
#if DEBUG_LOG
    printf("hi_start=%d (s=%d), %d\n", hi_start_state.start_enc_edge_pos_, hi_start_state.start_sample_idx_, -hi_start_state.filter_start_enc_edge_pos_);
    osDelay(5);
#endif

    // 2a. detect motion after hitting hi-limit
    float hi_start_phase;
    if (!run_detect_motion(hi_start_state, hi_enc_count, hi_start_phase)) {
        return false;
    }
#if DEBUG_LOG
    printf("hi_start_ph=%f\n", hi_start_phase);
    osDelay(5);
#endif

    // 2b. settle and go in direction for encoder decreasing to record samples
    if (!run_record_pass(stop_at_limit_with_record_handler, hi_start_state, hi_start_phase, rec_samples_1, max_range_timeout)) {
        return false;
    }

    // change current to record_current_2
    CRITICAL_SECTION() {
        axis_->open_loop_controller_.target_voltage_ = config_.record_current_2 * R;
    }

    // 3. forward record pass with I ~= record_current_2
    if (!run_record_pass(stop_at_limit_with_record_handler, lo_start_state, lo_start_phase, rec_samples_2, max_range_timeout)) {
        return false;
    }

    // 4. backward record pass with I ~= record_current_2
    if (!run_record_pass(stop_at_limit_with_record_handler, hi_start_state, hi_start_phase, rec_samples_2, max_range_timeout)) {
        return false;
    }

    // 5. return to center
    const int32_t center_enc_count = (lo_enc_edge_pos + hi_enc_edge_pos) / 2;
    CRITICAL_SECTION() {
        axis_->open_loop_controller_.total_distance_ = lo_start_phase;
        axis_->open_loop_controller_.phase_ = axis_->open_loop_controller_.initial_phase_ = wrap_pm_pi(lo_start_phase);
        axis_->open_loop_controller_.target_vel_ = phase_dir * config_.center_phase_speed;
    }
    if (!run_motor([&](){ return (last_enc_count_ >= center_enc_count); }, max_range_timeout)) {
        return false;
    }
    // lock at center
    const float center_phase = (lo_start_phase + hi_start_phase) * 0.5f;
    CRITICAL_SECTION() {
        axis_->open_loop_controller_.total_distance_ = center_phase;
        axis_->open_loop_controller_.phase_ = axis_->open_loop_controller_.initial_phase_ = wrap_pm_pi(center_phase);
        // stop phase velocity
        axis_->open_loop_controller_.target_vel_ = 0.0f;
    }
    if (!run_motor_for_time(config_.center_lock_duration)) {
        return false;
    }

#if DEBUG_LOG
    printf("center_enc=%d\n", last_enc_count_);
    osDelay(5);
#endif


    // finish!!!
    axis_->motor_.disarm();
    osDelay(100);

#if RECORD_SAMPLES
    printf("~~~samples_I1~~~\n");
    osDelay(5);
    printf("idx,Id,Iq\n");
    osDelay(5);
    for (uint32_t idx = lo_start_state.start_sample_idx_; idx <= hi_start_state.start_sample_idx_; ++idx)
    {
        const auto& s = rec_samples_1[idx];
        printf("%d, %f, %f\n", idx, s.Id, s.Iq);
        osDelay(5);
    }
    osDelay(100);
    printf("~~~samples_I2~~~\n");
    osDelay(5);
    printf("idx,Id,Iq\n");
    osDelay(5);
    for (uint32_t idx = lo_start_state.start_sample_idx_; idx <= hi_start_state.start_sample_idx_; ++idx)
    {
        const auto& s = rec_samples_2[idx];
        printf("%d, %f, %f\n", idx, s.Id, s.Iq);
        osDelay(5);
    }
    osDelay(100);
#endif
    
    const float delta_phase = build_maps(lo_start_state, hi_start_state);

    CRITICAL_SECTION() {
        encoder.shadow_count_ = encoder.count_in_cpr_ = last_enc_count_;
    }
    encoder.config_.direction = phase_dir;

    float delta_enc_i;
    float delta_enc_f = modff(delta_phase * phase_dir * phase2enc, &delta_enc_i);

    encoder.config_.phase_offset = init_enc_count + int(delta_enc_i);
    encoder.config_.phase_offset_float = delta_enc_f + 0.5f; // 0.5f for center-aligned reference at (init_enc_count + 0.5f)
    encoder.is_ready_ = true;

    center_phase_ = center_phase;
    center_enc_pos_ = center_phase * phase_dir * phase2enc + (encoder.config_.phase_offset + encoder.config_.phase_offset_float);
    lo_enc_pos_ = lo_enc_edge_pos;
    hi_enc_pos_ = hi_enc_edge_pos;
    enc2phase_ = enc2phase;
    phase2enc_ = phase2enc;
#if DEBUG_LOG
    printf("center=%f, %f - %f\n", center_enc_pos_, lo_enc_pos_, hi_enc_pos_);
    osDelay(5);
#endif
    return true;
}


template <typename F>
void filter_savgol7(int32_t size, const float* in, float* out, F&& idx_func)
{
    static constexpr float C3 = -2.0f / 21.0f;
    static constexpr float C2 =  3.0f / 21.0f;
    static constexpr float C1 =  6.0f / 21.0f;
    static constexpr float C0 =  7.0f / 21.0f;
    for (int32_t idx = 0; idx < size; ++idx)
    {
        float sum = (in[idx_func(size, idx - 3)] + in[idx_func(size, idx + 3)]) * C3
                  + (in[idx_func(size, idx - 2)] + in[idx_func(size, idx + 2)]) * C2
                  + (in[idx_func(size, idx - 1)] + in[idx_func(size, idx + 1)]) * C1
                                                 + in[idx]                      * C0;

        out[idx] = sum;
    }
}

float Calibrator::build_maps(const SamplingStartState& lo_start_state, const SamplingStartState& hi_start_state)
{
    static float tmp_I_cog_map[I_COG_MAP_MAX_SAMPLES];
    static float tmp_Id_hd_map[Id_HD_MAP_NUM_SAMPLES];

    const int32_t enc_cpr = axis_->encoder_.config_.cpr;
    const int32_t pole_pairs = axis_->motor_.config_.pole_pairs;

    const uint32_t len_in_samples = hi_start_state.start_sample_idx_ - lo_start_state.start_sample_idx_;

    const uint32_t period_numer = enc_cpr;
    const uint32_t period_denom = pole_pairs * config_.sample_size;
    // period = enc_cpr / (pole_pairs * config_.sample_size)
    // num_periods = floor(len_in_samples / period_in_samples)
    const uint32_t num_periods = (len_in_samples * period_denom) / period_numer;

    // sample_idx_ofs = sample_idx_0 + (len_in_samples - num_periods * period) / 2
    const uint32_t sample_idx_ofs = lo_start_state.start_sample_idx_ +
        (len_in_samples * period_denom - num_periods * period_numer) / (2 * period_denom);

    const uint32_t pos_denom = period_denom * Id_HD_MAP_NUM_SAMPLES;
    const float inv_pos_denom = 1.0f / pos_denom;
    const uint32_t pos_numer_ofs = sample_idx_ofs * pos_denom;

    auto iterateWithPeriodAverage = [=](const RecSample* rec_samples, std::function<void (uint32_t, float, float)>&& callback) {
        for (uint32_t i = 0; i < Id_HD_MAP_NUM_SAMPLES; ++i) {
            float p_sum_Id = 0.0;
            float p_sum_Iq = 0.0;
            for (uint32_t p = 0; p < num_periods; ++p) {
                const uint32_t pos_numer = pos_numer_ofs + period_numer * (i + p * Id_HD_MAP_NUM_SAMPLES);
                // pos_idx = sample_idx_ofs + i * period / Id_HD_MAP_NUM_SAMPLES + p * period
                const uint32_t pos_idx = pos_numer / pos_denom;
                const float pos_frac = float(pos_numer - pos_idx * pos_denom) * inv_pos_denom;
                // lerp
                const RecSample& s0 = rec_samples[pos_idx];
                const RecSample& s1 = rec_samples[pos_idx + 1];
                p_sum_Id += s0.Id + (s1.Id - s0.Id) * pos_frac;
                p_sum_Iq += s0.Iq + (s1.Iq - s0.Iq) * pos_frac;
            }
            const float p_avg_Id = p_sum_Id / num_periods;
            const float p_avg_Iq = p_sum_Iq / num_periods;
            callback(i, p_avg_Id, p_avg_Iq);
        }
    };

    // compute delta Id, delta_Iq in rec_samples_2
    for (uint32_t idx = lo_start_state.start_sample_idx_; idx <= hi_start_state.start_sample_idx_; ++idx) {
        const RecSample& s_I1 = rec_samples_1[idx];
        RecSample& s_I2 = rec_samples_2[idx];
        s_I2.Id -= s_I1.Id;
        s_I2.Iq -= s_I1.Iq;
    }
    RecSample* samples_I1 = rec_samples_1;
    RecSample* samples_diffI = rec_samples_2;

    float sum_diff_Id = 0.0f;
    float sum_diff_Iq = 0.0f;
    iterateWithPeriodAverage(samples_diffI, [&sum_diff_Id, &sum_diff_Iq](uint32_t i, float p_avg_diff_Id, float p_avg_diff_Iq) {
        sum_diff_Id += p_avg_diff_Id;
        sum_diff_Iq += p_avg_diff_Iq;
    });

    const float mean_diff_Id = sum_diff_Id / Id_HD_MAP_NUM_SAMPLES;
    const float mean_diff_Iq = sum_diff_Iq / Id_HD_MAP_NUM_SAMPLES;

    const float delta_phase = -mean_diff_Iq / mean_diff_Id;
    const float mean_real_diff_Id = mean_diff_Id - delta_phase * mean_diff_Iq;

    // build Id harmonic distortion map
    iterateWithPeriodAverage(samples_diffI, [&tmp_Id_hd_map, delta_phase, mean_real_diff_Id](uint32_t i, float p_avg_diff_Id, float p_avg_diff_Iq) {
        const float real_diff_Iq = p_avg_diff_Iq + delta_phase * p_avg_diff_Id;
        const float Id_hd = -real_diff_Iq / mean_real_diff_Id;
        tmp_Id_hd_map[i] = Id_hd;
    });

    static constexpr uint32_t Id_HD_MAP_IDX_MASK = Id_HD_MAP_NUM_SAMPLES - 1;
    // filter Id_hd_map with idx wrap
    filter_savgol7(Id_HD_MAP_NUM_SAMPLES, tmp_Id_hd_map, Id_hd_map_, [](int32_t, int32_t idx) { return idx & Id_HD_MAP_IDX_MASK; });

    // build cogging map (using already filtered Id_hd_map_)
    const float inv_period_numer = 1.0f / period_numer;
    for (int32_t idx = lo_start_state.start_sample_idx_; idx <= hi_start_state.start_sample_idx_; ++idx) {
        const RecSample& s_I1 = samples_I1[idx];
        const float real_Iq = s_I1.Iq + delta_phase * s_I1.Id;
        const float real_Id = s_I1.Id - delta_phase * s_I1.Iq;

        // period_pos = (idx - sample_idx_ofs) * Id_HD_MAP_NUM_SAMPLES / period + Id_HD_MAP_NUM_SAMPLES
        // (idx - sample_idx_ofs) can be negative so add Id_HD_MAP_NUM_SAMPLES to avoid negative period_pos_idx!
        // pos_denom = period_denom * Id_HD_MAP_NUM_SAMPLES
        const uint32_t period_pos_numer = (idx - int32_t(sample_idx_ofs)) * int32_t(pos_denom) + int32_t(Id_HD_MAP_NUM_SAMPLES * period_numer);
        const uint32_t period_pos_idx = period_pos_numer / period_numer;
        const float period_pos_frac = float(period_pos_numer - period_pos_idx * period_numer) * inv_period_numer;
        // lerp
        const float Id_hd_0 = Id_hd_map_[period_pos_idx & Id_HD_MAP_IDX_MASK];
        const float Id_hd_1 = Id_hd_map_[(period_pos_idx + 1) & Id_HD_MAP_IDX_MASK];
        const float Id_hd = Id_hd_0 + (Id_hd_1 - Id_hd_0) * period_pos_frac;

        const float I_cog = real_Iq + real_Id * Id_hd;
        tmp_I_cog_map[idx - lo_start_state.start_sample_idx_] = I_cog;
    }

    I_cog_map_size_ = len_in_samples + 1;
    // filter I_cog_map with idx mirror
    filter_savgol7(I_cog_map_size_, tmp_I_cog_map, I_cog_map_, [](int32_t size, int32_t idx) {
        if (idx < 0) return -idx;
        if (idx >= size) return ((size - 1) << 1) - idx;
        return idx;
    });

    I_cog_map_enc_beg_ = lo_start_state.start_enc_edge_pos_ + 0.5f * config_.sample_size;
    I_cog_map_enc_end_ = hi_start_state.start_enc_edge_pos_ - 0.5f * config_.sample_size;

    Id_hd_map_enc_start_ = lo_start_state.start_enc_edge_pos_ + (sample_idx_ofs - lo_start_state.start_sample_idx_ + 0.5f) * config_.sample_size;
    Id_hd_map_enc_period_ = float(enc_cpr) / pole_pairs;

#if DEBUG_LOG
    printf("build_maps: d_phs=%f, %f, %f\n", delta_phase, Id_hd_map_enc_start_, Id_hd_map_enc_period_);
    osDelay(10);

    printf("~~~Id_hd_map~~~\n");
    osDelay(10);
    printf("idx,val\n");
    osDelay(10);
    for (int i = 0; i < Id_HD_MAP_NUM_SAMPLES; ++i) {
        printf("%d, %f\n", i, Id_hd_map_[i]);
        osDelay(5);
    }

    printf("~~~I_cog_map~~~\n");
    osDelay(10);
    printf("idx,val\n");
    osDelay(10);
    for (int i = 0; i < I_cog_map_size_; ++i) {
        printf("%d, %f\n", i, I_cog_map_[i]);
        osDelay(5);
    }
    osDelay(100);
#endif

    return delta_phase;
}

float Calibrator::sample_I_cog_map(float enc_pos) const
{
    float s = (enc_pos - I_cog_map_enc_beg_) / (I_cog_map_enc_end_ - I_cog_map_enc_beg_);
    float map_pos = std::clamp(s, 0.0f, 1.0f - FLT_EPSILON) * (I_cog_map_size_- 1);

    float i_part;
    float f_part = modff(map_pos, &i_part);
    int32_t map_idx = int32_t(i_part);

    const float map_val0 = I_cog_map_[map_idx];
    const float map_val1 = I_cog_map_[map_idx + 1];
    // lerp
    return map_val0 + (map_val1 - map_val0) * f_part;
}

float Calibrator::sample_Id_hd_map(float enc_pos) const
{
    float s = (enc_pos - Id_hd_map_enc_start_) / Id_hd_map_enc_period_;
    float map_pos = s * Id_HD_MAP_NUM_SAMPLES;

    float i_part;
    float f_part = modff(map_pos, &i_part);
    int32_t map_idx = int32_t(i_part);

    static constexpr int32_t Id_HD_MAP_IDX_MASK = Id_HD_MAP_NUM_SAMPLES - 1;
    const float map_val0 = Id_hd_map_[map_idx & Id_HD_MAP_IDX_MASK];
    const float map_val1 = Id_hd_map_[(map_idx + 1) & Id_HD_MAP_IDX_MASK];
    // lerp
    return map_val0 + (map_val1 - map_val0) * f_part;
}
