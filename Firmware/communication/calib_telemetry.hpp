#ifndef __CALIB_TELEMETRY_HPP
#define __CALIB_TELEMETRY_HPP

#include <stdint.h>
#include <stddef.h>

// Record type marker embedded in the stream.
enum CalibTelemetryRecordType : uint8_t {
    CALIB_TELEMETRY_DATA  = 0,
    CALIB_TELEMETRY_START = 1,
    CALIB_TELEMETRY_END   = 2,
    CALIB_TELEMETRY_ENC   = 3, // encoder count event: timestamp + enc_count (stored in raw_phC)
};

// One 8 kHz control-loop sample streamed over USB (32 bytes, little-endian on
// the wire): [u8 type][u8 pad x3][u32 i_meas_timestamp]
//            [f32 ofs_phB][f32 ofs_phC][f32 raw_phB][f32 raw_phC]
//            [f32 vbus_v_raw][f32 vbus_v_ofs]
// ofs_phX = 0.5f * (ofs1_phX + ofs2_phX): average of the two offset samples
// bracketing the raw sample (same correction as CalibratorImpl uses).
// type = CALIB_TELEMETRY_DATA for a normal sample; START/END markers bound a run.
struct CalibTelemetryRecord {
    uint8_t type;
    uint8_t _pad[3];
    uint32_t timestamp;
    float ofs_phB;
    float ofs_phC;
    float raw_phB;
    float raw_phC;
    float vbus_v_raw;
    float vbus_v_ofs;
};
static_assert(sizeof(CalibTelemetryRecord) == 32, "telemetry record must be 32 bytes");

// Ring buffer. The control-loop ISR is the data producer; START/END markers are
// pushed from the calibrator thread (guarded by a critical section). The USB
// thread is the consumer. Power-of-two size. If full, the new record is dropped
// and the overflow counter is incremented, so the host can detect lost data.
class CalibTelemetry {
public:
    static constexpr uint32_t BUF_SIZE = 1024; // records (32 B each = 32 KB)

    void reset();                                // clear buffer + overflow (call with producer stopped)
    bool push(const CalibTelemetryRecord& r);    // data push (ISR context, critical section)
    void push_marker(CalibTelemetryRecordType type); // START/END (calibrator thread)
    void push_encoder(uint32_t timestamp, int16_t enc_count); // encoder event (GPIO ISR)
    uint32_t pop(CalibTelemetryRecord* out, uint32_t max); // USB thread context, returns count
    uint32_t overflow_count() const { return overflow_; }

    volatile bool enabled = false;

private:
    CalibTelemetryRecord buf_[BUF_SIZE];
    volatile uint32_t head_ = 0; // advanced by producers (ISR / marker)
    volatile uint32_t tail_ = 0; // advanced by consumer (USB thread)
    volatile uint32_t overflow_ = 0;
};

extern CalibTelemetry calib_telemetry;

#endif // __CALIB_TELEMETRY_HPP
