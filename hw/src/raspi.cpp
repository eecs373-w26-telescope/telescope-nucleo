#include <hw/inc/raspi.hpp>
#include <cstring>
#include <astro/inc/units.hpp>

namespace telescope {

    void RasPi::process_byte(uint8_t byte) {
        switch (decoder_.state) {
        case RxState::HUNT_SYNC_HI:
            if (byte == SYNC_HI) {
                decoder_.state = RxState::HUNT_SYNC_LO;
            }
            break;

        case RxState::HUNT_SYNC_LO:
            if (byte == SYNC_LO) {
                decoder_.header_pos = 0;
                decoder_.state = RxState::READ_HEADER;
            } else if (byte == SYNC_HI) {
                // stay in HUNT_SYNC_LO
            } else {
                decoder_.sync_losses++;
                decoder_.state = RxState::HUNT_SYNC_HI;
            }
            break;

        case RxState::READ_HEADER:
            decoder_.header_buf[decoder_.header_pos++] = byte;
            if (decoder_.header_pos == 2) {
                decoder_.packet_id = decoder_.header_buf[0];
                decoder_.expected_len = decoder_.header_buf[1];
                if (decoder_.expected_len > MAX_PAYLOAD_SIZE) {
                    decoder_.sync_losses++;
                    decoder_.state = RxState::HUNT_SYNC_HI;
                } else if (decoder_.expected_len == 0) {
                    decoder_.crc_pos = 0;
                    decoder_.state = RxState::READ_CRC;
                } else {
                    decoder_.payload_pos = 0;
                    decoder_.state = RxState::READ_PAYLOAD;
                }
            }
            break;

        case RxState::READ_PAYLOAD:
            decoder_.payload_buf[decoder_.payload_pos++] = byte;
            if (decoder_.payload_pos == decoder_.expected_len) {
                decoder_.crc_pos = 0;
                decoder_.state = RxState::READ_CRC;
            }
            break;

        case RxState::READ_CRC:
            decoder_.crc_buf[decoder_.crc_pos++] = byte;
            if (decoder_.crc_pos == 2) {
                // Reconstruct frame for CRC verification
                uint8_t frame[4 + MAX_PAYLOAD_SIZE];
                frame[0] = SYNC_HI;
                frame[1] = SYNC_LO;
                frame[2] = decoder_.packet_id;
                frame[3] = decoder_.expected_len;
                std::memcpy(&frame[4], decoder_.payload_buf, decoder_.expected_len);

                uint16_t computed = crc16_ccitt(frame, 4 + decoder_.expected_len);
                uint16_t received = static_cast<uint16_t>(decoder_.crc_buf[0])
                                | (static_cast<uint16_t>(decoder_.crc_buf[1]) << 8);

                if (computed == received) {
                    dispatch_packet(decoder_.packet_id, decoder_.payload_buf, decoder_.expected_len);
                } else {
                    decoder_.crc_errors++;
                }
                decoder_.state = RxState::HUNT_SYNC_HI;
            }
            break;
        }
    }

    void RasPi::dispatch_packet(uint8_t packet_id, const uint8_t* payload, uint8_t length) {
        switch (packet_id) {
        case PACKET_STATE_SYNC:
            if (length >= sizeof(StateSyncPayload)) {
                StateSyncPayload sync;
                std::memcpy(&sync, payload, sizeof(sync));
                current_state_ = sync.state;
                last_state_sync_tick_ = HAL_GetTick();
            }
            break;

        case PACKET_TIME:
            if (length >= sizeof(TimePayload)) {
                TimePayload t;
                std::memcpy(&t, payload, sizeof(t));
                raspi_time_.year   = t.year;
                raspi_time_.month  = t.month;
                raspi_time_.day    = t.day;
                raspi_time_.hour   = t.hour;
                raspi_time_.minute = t.minute;
                raspi_time_.second = t.second;
                has_raspi_time_ = true;
            }
            break;

        case PACKET_DSO_TARGET:
            // SD card read and response will be wired in when SD driver is implemented
            break;

        case PACKET_DEBUG:
            break;

        default:
            break;
        }
    }

    void RasPi::init() {
        last_state_sync_tick_ = HAL_GetTick();
        
        HAL_UARTEx_ReceiveToIdle_DMA(uart_, rx_dma_buf_, RX_BUF_SIZE);
        // Disable half-transfer interrupt, we only care about idle line and transfer complete
        __HAL_DMA_DISABLE_IT(uart_->hdmarx, DMA_IT_HT);
    }

    void RasPi::process() {
        if (uart_ == nullptr) return;

        uint16_t write_pos = (RX_BUF_SIZE - __HAL_DMA_GET_COUNTER(uart_->hdmarx)) % RX_BUF_SIZE;

        while (rx_read_pos_ != write_pos) {
            process_byte(rx_dma_buf_[rx_read_pos_]);
            rx_read_pos_ = (rx_read_pos_ + 1) % RX_BUF_SIZE;
        }
    }

    bool RasPi::send_packet(uint8_t packet_id, const uint8_t* payload, uint8_t length) {
        if (uart_ == nullptr) return false;
        if (length > MAX_PAYLOAD_SIZE) return false;

        uint32_t start_wait = HAL_GetTick();
        while (tx_busy_) {
            if (HAL_GetTick() - start_wait > 10) return false;
        }

        tx_buf_[0] = SYNC_HI;
        tx_buf_[1] = SYNC_LO;
        tx_buf_[2] = packet_id;
        tx_buf_[3] = length;
        std::memcpy(&tx_buf_[4], payload, length);

        uint16_t crc = crc16_ccitt(tx_buf_, 4 + length);
        tx_buf_[4 + length] = static_cast<uint8_t>(crc & 0xFF);
        tx_buf_[5 + length] = static_cast<uint8_t>(crc >> 8);

        tx_busy_ = true;
        HAL_UART_Transmit_DMA(uart_, tx_buf_, 6 + length);
        return true;
    }

    bool RasPi::send_gps(const GPSPayload& p) {
        return send_packet(PACKET_GPS, reinterpret_cast<const uint8_t*>(&p), sizeof(p));
    }

    bool RasPi::send_encoder(const EncoderPayload& p) {
        return send_packet(PACKET_ENCODER, reinterpret_cast<const uint8_t*>(&p), sizeof(p));
    }

    bool RasPi::send_imu(const IMUPayload& p) {
        return send_packet(PACKET_IMU, reinterpret_cast<const uint8_t*>(&p), sizeof(p));
    }

    bool RasPi::send_dso_target(const DSOTargetPayload& p) {
        return send_packet(PACKET_DSO_TARGET, reinterpret_cast<const uint8_t*>(&p), sizeof(p));
    }

    bool RasPi::send_state_sync(const StateSyncPayload& p) {
        return send_packet(PACKET_STATE_SYNC, reinterpret_cast<const uint8_t*>(&p), sizeof(p));
    }

    bool RasPi::send_debug(const DebugPayload& p) {
        return send_packet(PACKET_DEBUG, reinterpret_cast<const uint8_t*>(&p), sizeof(p));
    }

    bool RasPi::send_fov_objects(const FovObjectsPayload& p, uint8_t count) {
        const uint8_t len = static_cast<uint8_t>(sizeof(uint8_t) + count * sizeof(FovObjectEntry));
        return send_packet(PACKET_FOV_OBJECTS, reinterpret_cast<const uint8_t*>(&p), len);
    }

    uint8_t RasPi::mirrored_state() {
        return current_state_;
    }

    bool RasPi::connection_active() {
        return (HAL_GetTick() - last_state_sync_tick_) < CONNECTION_TIMEOUT_MS;
    }

    void RasPi::tx_complete_callback() {
        tx_busy_ = false;
    }

    void RasPi::rx_event_callback(uint16_t size) {
        // DMA circular mode handles continuous reception.
        // Re-enable if HAL stopped it (shouldn't happen in circular mode, but defensive).
        if (uart_->RxState == HAL_UART_STATE_READY) {
            HAL_UARTEx_ReceiveToIdle_DMA(uart_, rx_dma_buf_, RX_BUF_SIZE);
            __HAL_DMA_DISABLE_IT(uart_->hdmarx, DMA_IT_HT);
        }
    }

} // namespace telescope
