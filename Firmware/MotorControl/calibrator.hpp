#ifndef __CALIBRATOR_HPP
#define __CALIBRATOR_HPP

#include "calibrator_impl.hpp"

constexpr float _PI = 3.14159265358979323846f;


struct SamplingStartState;

class DetectMotionUpdateHandler;

class Calibrator
{
public:
    static constexpr uint32_t I_COG_MAP_MAX_SAMPLES = CALIB_MAX_SAMPLES;
    static constexpr uint32_t Id_HD_MAP_NUM_SAMPLES = 256; // should be 2^n

    struct Config_t {
        float start_lock_voltage = 1.5f;  // [volt]
        float start_lock_settle_duration = 1.0f; // [sec]
        float start_lock_measure_duration = 1.0f; // [sec]
        float start_lock_current = 15.0f;  // [A]

        float detect_motion_duration = 1.0f; // [sec]
        float phase_settle_duration = 1.0f; // [sec]

        float detect_dir_timeout = 1.0f; // [sec]
        int32_t detect_dir_enc_dist = 50;
        int32_t stop_limit_enc_ofs = 5;
        int32_t record_limit_enc_ofs = 50;

        int32_t sample_size = 5;

        float max_motor_degree_range = 130.0f;
        float record_phase_speed = _PI / 2.0f;

        float phase_speed_threshold_mult = 3.0f;

        float record_current_1 = 5.0f;
        float record_current_2 = 15.0f;

        float center_phase_speed = _PI;
        float center_lock_duration = 1.0f; // [sec]

        uint32_t detect_limit_warm_up_count = 200;
        float detect_limit_filter_alpha = 0.01f; // 2/(N + 1) ~ 199 samples
        float detect_limit_threshold_mult = 5.0f;
    };

    Calibrator() = default;

    // called from axis thread!
    bool run();

    CalibratorImpl& getImpl() { return impl_; }

    Config_t config_;

    float center_phase_;
    float center_enc_pos_;
    float hi_enc_pos_;
    float lo_enc_pos_;
    float enc2phase_;
    float phase2enc_;

    float sample_I_cog_map(float enc_pos) const;
    float sample_Id_hd_map(float enc_pos) const;

private:
    template <typename F>
    bool runMotor(F&& func, float timeout_seconds, bool error_on_timeout = true);

    bool runMotorForTime(float duration_seconds);

    bool runDetectMotion(DetectMotionUpdateHandler& update_handler, const SamplingStartState& start_state, int32_t start_enc_count);

    template <typename T>
    bool runRecordPass(T& update_handler, const SamplingStartState& start_state, CalibratorSample* rec_samples, float timeout,
                       std::optional<float> start_phase = std::nullopt, std::optional<float> set_voltage = std::nullopt);

    float buildMaps(const SamplingStartState& lo_start_state, const SamplingStartState& hi_start_state);

private:
    CalibratorImpl impl_;

    uint32_t I_cog_map_size_;
    float I_cog_map_enc_beg_;
    float I_cog_map_enc_end_;
    float I_cog_map_[I_COG_MAP_MAX_SAMPLES];

    float Id_hd_map_enc_start_;
    float Id_hd_map_enc_period_;
    float Id_hd_map_[Id_HD_MAP_NUM_SAMPLES];
};

#endif // __CALIBRATOR_HPP
