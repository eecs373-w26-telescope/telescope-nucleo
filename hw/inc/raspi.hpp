#pragma once

#include "main.h"
#include <hw/inc/protocol.h>
#include <cstdint>

namespace telescope {
    constexpr uint16_t RX_BUF_SIZE = 256;
    constexpr uint16_t TX_BUF_SIZE = 134; // max frame: 4 header + 128 payload + 2 CRC
    constexpr uint32_t CONNECTION_TIMEOUT_MS = 1000;

    class RasPi{
    public:
        RasPi(UART_HandleTypeDef* huart) :
            uart_{huart}, decoder_{Decoder{}}, tx_busy_{false}, rx_read_pos_{0},
            current_state_{0}, last_state_sync_tick_{HAL_GetTick()} {}
        void init();

        // Call from main loop to consume DMA RX bytes and dispatch packets
        void process();

        // Frame and transmit a packet via DMA. Returns false if TX is busy.
        bool send_packet(uint8_t packet_id, const uint8_t* payload, uint8_t length);

        // Convenience wrappers for typed payloads
        bool send_gps(const GPSPayload& payload);
        bool send_encoder(const EncoderPayload& payload);
        bool send_imu(const IMUPayload& payload);
        bool send_state_sync(const StateSyncPayload& payload);
        bool send_debug(const DebugPayload& payload);

        // Current mirrored state from raspi (updated by STATE_SYNC packets)
        uint8_t mirrored_state();
        bool    connection_active();

        // Called by HAL from ISR context
        void tx_complete_callback();
        void rx_event_callback(uint16_t size);

    private:
        // Lightweight stream decoder (no heap allocation)
        enum class RxState : uint8_t {
            HUNT_SYNC_HI,
            HUNT_SYNC_LO,
            READ_HEADER,
            READ_PAYLOAD,
            READ_CRC,
        };

        struct Decoder {
            RxState  state = RxState::HUNT_SYNC_HI;
            uint8_t  header_buf[2]{};
            uint8_t  payload_buf[MAX_PAYLOAD_SIZE]{};
            uint8_t  crc_buf[2]{};
            uint8_t  header_pos = 0;
            uint8_t  payload_pos = 0;
            uint8_t  crc_pos = 0;
            uint8_t  expected_len = 0;
            uint8_t  packet_id = 0;
            uint32_t crc_errors = 0;
            uint32_t sync_losses = 0;
        };

        UART_HandleTypeDef* uart_ = nullptr;
        Decoder decoder_;

        // TX state
        uint8_t tx_buf_[TX_BUF_SIZE];
        volatile bool tx_busy_ = false;

        // RX DMA circular buffer
        uint8_t rx_dma_buf_[RX_BUF_SIZE];
        uint16_t rx_read_pos_ = 0;

        // Mirrored state from raspi
        volatile uint8_t current_state_ = 0;
        volatile uint32_t last_state_sync_tick_ = 0;

        void dispatch_packet(uint8_t packet_id, const uint8_t* payload, uint8_t length);
        void process_byte(uint8_t byte);
    };


} // namespace telescope
