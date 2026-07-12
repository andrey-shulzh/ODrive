#ifndef __CALIBRATOR_INC_HPP
#define __CALIBRATOR_INC_HPP

#include <stdint.h>


#define CALIB_ALLOC_SAMPLES 0
#define CALIB_RECCORD_I 1

constexpr uint32_t CALIB_MAX_ENC_CPR = 20480;
constexpr uint32_t CALIB_MIN_SAMPLE_SIZE = 5;
constexpr uint32_t CALIB_MAX_RANGE_DEG = 130;

constexpr uint32_t CALIB_MAX_SAMPLES = (CALIB_MAX_ENC_CPR * CALIB_MAX_RANGE_DEG / (360 * CALIB_MIN_SAMPLE_SIZE)) + 1;

struct CalibratorSample { float Id, Iq; };

#if CALIB_RECCORD_I
constexpr uint32_t CALIB_MAX_I_SAMPLES = 4096;
#endif 

class CalibratorUpdateHandler
{
public:
    CalibratorUpdateHandler() = default;
    virtual ~CalibratorUpdateHandler() = default;

    struct ProcessArgs
    {
        uint32_t update_timestamp;
        uint32_t i_meas_timestamp;

        int32_t enc_count;
        uint32_t enc_time;

        float phase_dist;

        float Ialpha, Ibeta;
    };

    virtual bool process(const ProcessArgs& args) { return true; }
};

#endif // __CALIBRATOR_INC_HPP
