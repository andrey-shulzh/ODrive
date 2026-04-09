#ifndef __CALIBRATOR_IMPL_HPP
#define __CALIBRATOR_IMPL_HPP

#include "calibrator_inc.hpp"

#include <optional>


class Axis;

class CalibratorImpl
{
public:
    static constexpr uint32_t CLOCK_HZ = 168000000;

    static CalibratorSample* getSamples1();
    static CalibratorSample* getSamples2();

    CalibratorImpl() = default;

    Axis* axis_ = nullptr; // set by Axis constructor

    volatile uint32_t last_enc_time_;
    volatile int16_t last_enc_count_;

    // called from ControlLoop_IRQHandler at 8 kHz!
    bool update(uint32_t current_meas_timestamp);

    int32_t getEncoderCPR() const;
    int32_t getMotorPolePairs() const;

    void setUpdateHandler(CalibratorUpdateHandler* update_handler) {
        update_handler_ = update_handler;
    }

    void clearUpdateHandler() {
        update_handler_ = &empty_update_handler_;
    }

    void initMotor();
    void shutdownMotor();

    bool isAbortOrMotorError();

    void setEncoderError();
    void setEncoderReady(int32_t init_enc_count, float delta_enc, int32_t phase_dir);

    struct ControlParams
    {
        std::optional<float> phase;
        std::optional<float> phase_vel;

        std::optional<float> voltage_change_time;
        std::optional<float> voltage;
    };
    void setControlParams(const ControlParams& params);

private:
    static CalibratorUpdateHandler empty_update_handler_;
    CalibratorUpdateHandler* volatile update_handler_ = &empty_update_handler_;
};

#endif // __CALIBRATOR_IMPL_HPP
