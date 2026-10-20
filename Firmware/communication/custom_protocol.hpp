#ifndef __CUSTOM_PROTOCOL_HPP
#define __CUSTOM_PROTOCOL_HPP

#include <fibre/async_stream.hpp>

// Binary command / telemetry protocol for the custom vendor interface (EP2).
//
// The USB endpoint is a raw byte stream (the host sees a continuous stream,
// packets are reassembled transparently). Framing:
//
//   Host -> device (CUSTOM_OUT_EP), one command:
//     [u8 cmd][u8 req_id][u16 arg_len LE][args[arg_len]]
//
//   Device -> host (CUSTOM_IN_EP), length-prefixed response. The size does not
//   have to be known to the host in advance; the device announces it in the
//   header and then streams the payload:
//     [u8 status][u8 req_id][u32 total_len LE][payload[total_len]]
//
// Responses larger than one USB packet are sent in <=63 byte chunks; the
// protocol drives the chunks itself (see start_tx_chunk/on_tx_finished).

#define CUSTOM_PROTOCOL_MAX_CMD_LEN ((size_t)512)
#define CUSTOM_PROTOCOL_MAX_RESP_LEN ((size_t)512)
#define CUSTOM_PROTOCOL_TX_CHUNK ((size_t)63) // must be < USB_TX_DATA_SIZE (64)
#define CUSTOM_PROTOCOL_RX_CHUNK ((size_t)64) // cap OUT transfer size per USB read

// Temporary diagnostic: prints protocol counters to stdout when called
// periodically (e.g. from the USB thread). Remove together with CUSTOM_PROTOCOL_DEBUG.
void custom_proto_debug_tick();
extern volatile uint32_t custom_proto_dbg_isr; // incremented in usb_rx_process_packet

class CustomProtocol {
public:
    CustomProtocol(fibre::AsyncStreamSource* rx_channel, fibre::AsyncStreamSink* tx_channel)
        : rx_channel_(rx_channel), tx_channel_(tx_channel) {}

    void start();

private:
    void on_read_finished(fibre::ReadResult result);
    void on_tx_finished(fibre::WriteResult result);
    void try_process();
    void process_command(uint8_t cmd, uint8_t req_id, const uint8_t* args, uint16_t arg_len);
    void send_response(uint8_t status, uint8_t req_id, const uint8_t* payload, uint32_t len);
    uint8_t* begin_response(uint8_t status, uint8_t req_id); // returns writable payload area of tx_buf_
    void finish_response(uint32_t len);                       // writes length header and starts TX
    void start_tx_chunk();

    fibre::AsyncStreamSource* rx_channel_ = nullptr;
    fibre::AsyncStreamSink* tx_channel_ = nullptr;

    uint8_t rx_buf_[CUSTOM_PROTOCOL_MAX_CMD_LEN];
    size_t rx_fill_ = 0;

    uint8_t tx_buf_[CUSTOM_PROTOCOL_MAX_RESP_LEN];
    size_t tx_len_ = 0;
    size_t tx_pos_ = 0;
    bool tx_active_ = false;
    fibre::TransferHandle tx_handle_ = 0;
};

#endif // __CUSTOM_PROTOCOL_HPP
