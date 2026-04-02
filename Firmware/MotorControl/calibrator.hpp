#ifndef __CALIBRATOR_HPP
#define __CALIBRATOR_HPP

#include "utils.hpp"

#define RECORD_SAMPLES 1

constexpr uint32_t I_COG_MAP_MAX_SAMPLES = 1408;
constexpr uint32_t Id_HD_MAP_NUM_SAMPLES = 256; // should be 2^n

class Axis;


class BaseUpdateHandler
{
public:
    BaseUpdateHandler() = default;
    virtual ~BaseUpdateHandler() = default;

    struct ProcessArgs
    {
        uint32_t timestamp;
        float phase;
        int32_t enc_count;
        uint32_t enc_time;

        float Ialpha, Ibeta;

        Iph_ABC_t Iph_raw;
        Iph_ABC_t Iph_ofs;
        //Iph_ABC_t Iph_meas;
        //Iph_ABC_t Iph_calib;
    };

    virtual bool process(const ProcessArgs& args) { return true; }
};

struct RecSample;
struct SamplingStartState;

class CalibratorSafeReturn;

class Calibrator
{
    friend class CalibratorSafeReturn;
public:
    struct Config_t {
        float start_lock_voltage = 1.5f;  // [volt]
        float start_lock_settle_duration = 1.0f; // [sec]
        float start_lock_measure_duration = 1.0f; // [sec]
        //float start_lock_current_duration = 1.0f; // [sec]
        //float start_lock_current = 15.0f;  // [A]

        float detect_motion_duration = 1.0f; // [sec]
        float phase_settle_duration = 1.0f; // [sec]

        int32_t detect_dir_enc_dist = 25;
        int32_t stop_limit_enc_ofs = 5;
        int32_t record_limit_enc_ofs = 50;

        int32_t sample_size = 5;

        float max_motor_degree_range = 130.0f;
        float record_phase_speed = M_PI / 2.0f;

        float record_current_1 = 5.0f;
        float record_current_2 = 15.0f;

        float center_phase_speed = M_PI;
        float center_lock_duration = 1.0f; // [sec]

        float detect_limit_vel_threshold = 0.25f;
        int32_t detect_limit_warm_up_count = 50;
    };

    Calibrator();
    // called right after encoder.update() from 8 kHz interrupt!
    bool update(uint32_t timestamp);
    // called from axis thread!
    bool run_offset_calibration();

    Axis* axis_ = nullptr; // set by Axis constructor
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
    bool run_motor(F&& func, float timeout_seconds, bool error_on_timeout = true);

    bool run_motor_for_time(float duration_seconds);

    template <typename T>
    bool run_record_pass(T& update_handler, const SamplingStartState& start_state, std::optional<float> start_phase, RecSample* rec_samples, float timeout);

    float build_maps(const SamplingStartState& lo_start_state, const SamplingStartState& hi_start_state);

private:
    static BaseUpdateHandler empty_update_handler_;
    BaseUpdateHandler* volatile update_handler_ = &empty_update_handler_;

    uint32_t I_cog_map_size_;
    float I_cog_map_enc_beg_;
    float I_cog_map_enc_end_;
    float I_cog_map_[I_COG_MAP_MAX_SAMPLES];

    float Id_hd_map_enc_start_;
    float Id_hd_map_enc_period_;
    float Id_hd_map_[Id_HD_MAP_NUM_SAMPLES];

    volatile uint32_t last_enc_time_;
    volatile int16_t last_enc_count_;
};

#endif // __CALIBRATOR_HPP
