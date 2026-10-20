#include "custom_protocol.hpp"

#include <string.h>
#include <algorithm>

#include "calib_telemetry.hpp"
#include "interface_usb.h"
#include <odrive_main.h>
#include <cmsis_os.h>
#include <freertos_vars.h>

// Temporary debug output: written straight to the USB stdout sink (no printf /
// no formatting, to avoid the newlib printf stack cost on the USB thread).
// Set CUSTOM_PROTOCOL_DEBUG to 0 when done.
#define CUSTOM_PROTOCOL_DEBUG 0
#if CUSTOM_PROTOCOL_DEBUG
static void cp_log(const char* s) {
    fibre::cbufptr_t buf{(const uint8_t*)s, (const uint8_t*)s + strlen(s)};
    usb_cdc_stdout_sink.write(buf);
    if (!usb_cdc_stdout_pending) {
        usb_cdc_stdout_pending = true;
        osMessagePut(usb_event_queue, 7, 0);
    }
}
#define CP_LOG(s) cp_log(s)
#else
#define CP_LOG(s) do {} while (0)
#endif

// Diagnostic counters (started / rx frames / parsed cmds / tx chunks)
static volatile uint32_t dbg_started = 0;
static volatile uint32_t dbg_rx = 0;
static volatile uint32_t dbg_cmd = 0;
static volatile uint32_t dbg_tx = 0;
volatile uint32_t custom_proto_dbg_isr = 0;

#if CUSTOM_PROTOCOL_DEBUG
static char* cp_append_u32(char* p, uint32_t v) {
    char tmp[10];
    int n = 0;
    if (v == 0) { *p++ = '0'; return p; }
    while (v) { tmp[n++] = (char)('0' + (v % 10u)); v /= 10u; }
    while (n) { *p++ = tmp[--n]; }
    return p;
}

void custom_proto_debug_tick() {
    char buf[96]; // five uint32 counters plus labels and terminator
    char* p = buf;
    const char* s = "CPdbg s=";
    while (*s) *p++ = *s++;
    p = cp_append_u32(p, dbg_started);
    s = " isr="; while (*s) *p++ = *s++;
    p = cp_append_u32(p, custom_proto_dbg_isr);
    s = " rx="; while (*s) *p++ = *s++;
    p = cp_append_u32(p, dbg_rx);
    s = " cmd="; while (*s) *p++ = *s++;
    p = cp_append_u32(p, dbg_cmd);
    s = " tx="; while (*s) *p++ = *s++;
    p = cp_append_u32(p, dbg_tx);
    *p++ = '\r'; *p++ = '\n'; *p = 0;
    cp_log(buf);
}
#else
void custom_proto_debug_tick() {}
#endif

using namespace fibre;

void CustomProtocol::start() {
    dbg_started = 1;
    rx_fill_ = 0;
    TransferHandle dummy;
    rx_channel_->start_read({rx_buf_, rx_buf_ + CUSTOM_PROTOCOL_RX_CHUNK}, &dummy, MEMBER_CB(this, on_read_finished));
}

void CustomProtocol::on_read_finished(ReadResult result) {
    if (result.status != kStreamOk) {
        return;
    }

    rx_fill_ = (size_t)(result.end - rx_buf_);
    dbg_rx++;

    try_process();

    if (rx_fill_ >= sizeof(rx_buf_)) {
        // Could not consume (e.g. TX still busy): drop to avoid a stuck buffer.
        rx_fill_ = 0;
    }

    uint8_t* rx_start = rx_buf_ + rx_fill_;
    size_t rx_capacity = std::min(CUSTOM_PROTOCOL_RX_CHUNK, sizeof(rx_buf_) - rx_fill_);
    uint8_t* rx_end = rx_start + rx_capacity;
    TransferHandle dummy;
    rx_channel_->start_read({rx_start, rx_end}, &dummy, MEMBER_CB(this, on_read_finished));
}

void CustomProtocol::try_process() {
    for (;;) {
        if (tx_active_) {
            return; // wait for the current response to finish
        }
        if (rx_fill_ < 4) {
            return; // not even a header yet
        }

        uint8_t cmd = rx_buf_[0];
        uint8_t req_id = rx_buf_[1];
        uint16_t arg_len = (uint16_t)(rx_buf_[2] | (rx_buf_[3] << 8));

        if ((size_t)arg_len > sizeof(rx_buf_) - 4) {
            // Malformed / oversized length: drop one byte and resync.
            rx_fill_ -= 1;
            memmove(rx_buf_, rx_buf_ + 1, rx_fill_);
            continue;
        }

        if (rx_fill_ < 4 + (size_t)arg_len) {
            return; // wait for the rest of the command
        }

        process_command(cmd, req_id, rx_buf_ + 4, arg_len);

        size_t consumed = 4 + (size_t)arg_len;
        rx_fill_ -= consumed;
        memmove(rx_buf_, rx_buf_ + consumed, rx_fill_);
    }
}

void CustomProtocol::process_command(uint8_t cmd, uint8_t req_id, const uint8_t* args, uint16_t arg_len) {
    dbg_cmd++;
    switch (cmd) {
        case 0x00: // echo: return the argument bytes unchanged
            send_response(0, req_id, args, arg_len);
            break;

        case 0x01: { // test telemetry: args = [u16 LE count]; return count bytes pattern
            uint16_t count = 0;
            if (arg_len >= 2) {
                count = (uint16_t)(args[0] | (args[1] << 8));
            }
            uint8_t buf[256];
            uint16_t n = std::min<uint16_t>(count, (uint16_t)sizeof(buf));
            for (uint16_t i = 0; i < n; ++i) {
                buf[i] = (uint8_t)i;
            }
            send_response(0, req_id, buf, n);
            break;
        }

        case 0x02: { // read telemetry: [u32 overflow][N x 24-byte records], written straight into tx_buf_
            uint32_t overflow = calib_telemetry.overflow_count();

            uint8_t* p = begin_response(0, req_id);
            p[0] = (uint8_t)(overflow & 0xFF);
            p[1] = (uint8_t)((overflow >> 8) & 0xFF);
            p[2] = (uint8_t)((overflow >> 16) & 0xFF);
            p[3] = (uint8_t)((overflow >> 24) & 0xFF);

            uint32_t max_recs = (uint32_t)((sizeof(tx_buf_) - 6 - 4) / sizeof(CalibTelemetryRecord));
            uint32_t n = calib_telemetry.pop(reinterpret_cast<CalibTelemetryRecord*>(p + 4), max_recs);

            finish_response(4 + n * sizeof(CalibTelemetryRecord));
            break;
        }

        case 0x04: { // trigger encoder offset calibration on axis 0 (async)
            odrv.get_axis(0).requested_state_ = Axis::AXIS_STATE_ENCODER_OFFSET_CALIBRATION;
            send_response(0, req_id, nullptr, 0);
            break;
        }

        default:
            send_response(1, req_id, nullptr, 0); // status 1 = unknown command
            break;
    }
}

void CustomProtocol::send_response(uint8_t status, uint8_t req_id, const uint8_t* payload, uint32_t len) {
    if (len > sizeof(tx_buf_) - 6) {
        len = sizeof(tx_buf_) - 6;
    }
    uint8_t* p = begin_response(status, req_id);
    if (payload != nullptr && len > 0) {
        memcpy(p, payload, len);
    }
    finish_response(len);
}

uint8_t* CustomProtocol::begin_response(uint8_t status, uint8_t req_id) {
    tx_buf_[0] = status;
    tx_buf_[1] = req_id;
    return tx_buf_ + 6; // payload area
}

void CustomProtocol::finish_response(uint32_t len) {
    if (len > sizeof(tx_buf_) - 6) {
        len = sizeof(tx_buf_) - 6;
    }
    tx_buf_[2] = (uint8_t)(len & 0xFF);
    tx_buf_[3] = (uint8_t)((len >> 8) & 0xFF);
    tx_buf_[4] = (uint8_t)((len >> 16) & 0xFF);
    tx_buf_[5] = (uint8_t)((len >> 24) & 0xFF);
    tx_len_ = 6 + len;
    tx_pos_ = 0;
    start_tx_chunk();
}

void CustomProtocol::start_tx_chunk() {
    if (tx_active_) {
        return;
    }
    size_t remaining = tx_len_ - tx_pos_;
    if (remaining == 0) {
        return;
    }
    size_t chunk = std::min(remaining, CUSTOM_PROTOCOL_TX_CHUNK);
    dbg_tx++;
    tx_active_ = true;
    tx_channel_->start_write({tx_buf_ + tx_pos_, tx_buf_ + tx_pos_ + chunk}, &tx_handle_, MEMBER_CB(this, on_tx_finished));
}

void CustomProtocol::on_tx_finished(WriteResult result) {
    tx_active_ = false;
    if (result.status != kStreamOk) {
        tx_pos_ = tx_len_; // abort this response
        // start_write() may reject a transfer synchronously. In that case
        // try_process() has not consumed the current command yet. Re-entering
        // it here would retry the same command recursively until stack overflow.
        return;
    } else {
        tx_pos_ = (size_t)(result.end - tx_buf_);
    }
    if (tx_pos_ < tx_len_) {
        start_tx_chunk();
    } else {
        try_process(); // process any commands that arrived while sending
    }
}
