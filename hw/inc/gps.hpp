#pragma once

#include "main.h"
#include <hw/inc/protocol.h>
#include <cstdint>

#define GPS_BUFFER_SIZE 128
#define GPS_DMA_BUF_SIZE 256

// GPS on USART6: D5 (PC7) = RX, D6 (PC6) = TX
// DMA RX circular mode, no TX but thats fine

namespace GPS {

class gps {
public:
    void init(UART_HandleTypeDef* uart);
    void process();

    bool has_fix() const;
    GpsPayload payload() const;

    // Called from ISR context
    void rx_event_callback(uint16_t size);

    double latitude = 0.0;
    double longitude = 0.0;
    int utc_hours = 0;
    int utc_minutes = 0;
    float utc_seconds = 0.0f;
    bool fix = false;

    int day = 0;
    int month = 0;
    int year = 0;

    uint8_t num_satellites = 0;

private:
    UART_HandleTypeDef* huart_ = nullptr;

    uint8_t dma_buf_[GPS_DMA_BUF_SIZE]{};
    uint16_t read_pos_ = 0;

    char line_buf_[GPS_BUFFER_SIZE]{};
    uint16_t line_index_ = 0;

    void process_byte(uint8_t byte);
    void parse_rmc(const char* line);
    void parse_gga(const char* line);
    bool is_rmc(const char* line) const;
    bool is_gga(const char* line) const;
};

} // namespace GPS