#include "calib_telemetry.hpp"

#include <string.h> // memcpy
#include <Drivers/STM32/stm32_system.h> // cpu_enter_critical / cpu_exit_critical

CalibTelemetry calib_telemetry;

void CalibTelemetry::reset() {
    uint32_t prim = cpu_enter_critical();
    head_ = 0;
    tail_ = 0;
    overflow_ = 0;
    cpu_exit_critical(prim);
}

bool CalibTelemetry::push(const CalibTelemetryRecord& r) {
    // Multiple producers (control-loop ISR, GPIO encoder ISR, calibrator thread)
    // advance head_ concurrently, so guard with a critical section.
    uint32_t prim = cpu_enter_critical();
    uint32_t head = head_;
    uint32_t next = (head + 1) & (BUF_SIZE - 1);
    bool ok = true;
    if (next == tail_) {
        overflow_++; // full: drop new record, flag loss
        ok = false;
    } else {
        buf_[head] = r;
        head_ = next; // publish
    }
    cpu_exit_critical(prim);
    return ok;
}

void CalibTelemetry::push_marker(CalibTelemetryRecordType type) {
    CalibTelemetryRecord r{};
    r.type = (uint8_t)type;
    push(r);
}

void CalibTelemetry::push_encoder(uint32_t timestamp, int16_t enc_count) {
    CalibTelemetryRecord r{};
    r.type = CALIB_TELEMETRY_ENC;
    r.timestamp = timestamp;
    r.raw_phC = (float)enc_count;
    push(r);
}

uint32_t CalibTelemetry::pop(CalibTelemetryRecord* out, uint32_t max) {
    uint32_t n = 0;
    while (n < max) {
        if (head_ == tail_) {
            break; // empty
        }
        // memcpy (not struct assignment) so 'out' may be unaligned (tx_buf_ + 10)
        memcpy(&out[n], &buf_[tail_], sizeof(CalibTelemetryRecord));
        tail_ = (tail_ + 1) & (BUF_SIZE - 1);
        ++n;
    }
    return n;
}
