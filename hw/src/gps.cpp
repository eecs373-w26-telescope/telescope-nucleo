#include <hw/inc/gps.hpp>
#include <cstring>
#include <cstdio>

namespace GPS {

void gps::init(UART_HandleTypeDef* uart) {
    huart = uart;
    read_pos = 0;
    line_index = 0;
    std::memset(dma_buf, 0, sizeof(dma_buf));
    std::memset(line_buf, 0, sizeof(line_buf));

    HAL_UARTEx_ReceiveToIdle_DMA(huart, dma_buf, GPS_DMA_BUF_SIZE);
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
}

void gps::process() {
    if (huart == nullptr) return;

    uint16_t write_pos = GPS_DMA_BUF_SIZE - __HAL_DMA_GET_COUNTER(huart->hdmarx);

    while (read_pos != write_pos) {
        process_byte(dma_buf[read_pos]);
        read_pos = (read_pos + 1) % GPS_DMA_BUF_SIZE;
    }
}

void gps::process_byte(uint8_t byte) {
    char c = static_cast<char>(byte);

    if (c == '$') {
        line_index = 0;
        line_buf[line_index++] = c;
        return;
    }

    if (c == '\r') return;

    if (c == '\n') {
        line_buf[line_index] = '\0';
        if (is_gga_sentence(line_buf)) {
            parse_gga(line_buf);
        }
        line_index = 0;
        return;
    }

    if (line_index < GPS_BUFFER_SIZE - 1) {
        line_buf[line_index++] = c;
    }
}

bool gps::is_gga_sentence(const char* line) const {
    return (std::strncmp(line, "$GNGGA", 6) == 0 ||
            std::strncmp(line, "$GPGGA", 6) == 0);
}

void gps::parse_gga(const char* line) {
    float utc_raw = 0, lat_raw = 0, lon_raw = 0, alt = 0;
    char lat_dir = 0, lon_dir = 0;
    int quality = 0, sats = 0;
    float hdop = 0;

    // $G?GGA,hhmmss.ss,lat,N/S,lon,E/W,quality,sats,hdop,alt,M,...
    int matched = sscanf(line + 7,
        "%f,%f,%c,%f,%c,%d,%d,%f,%f",
        &utc_raw, &lat_raw, &lat_dir, &lon_raw, &lon_dir,
        &quality, &sats, &hdop, &alt);

    if (matched < 6) return;

    fix_quality = static_cast<uint8_t>(quality);
    if (quality == 0) return;

    num_satellites = static_cast<uint8_t>(sats);

    int lat_deg = static_cast<int>(lat_raw / 100);
    latitude = lat_deg + (lat_raw - lat_deg * 100) / 60.0;
    if (lat_dir == 'S') latitude *= -1;

    int lon_deg = static_cast<int>(lon_raw / 100);
    longitude = lon_deg + (lon_raw - lon_deg * 100) / 60.0;
    if (lon_dir == 'W') longitude *= -1;

    if (matched >= 9) {
        altitude_mm = static_cast<int32_t>(alt * 1000.0f);
    }
}

bool gps::has_fix() const {
    return fix_quality > 0;
}

GpsPayload gps::payload() const {
    GpsPayload p{};
    p.latitude_e7 = static_cast<int32_t>(latitude * 1e7);
    p.longitude_e7 = static_cast<int32_t>(longitude * 1e7);
    p.altitude_mm = altitude_mm;
    p.fix_quality = fix_quality;
    p.num_satellites = num_satellites;
    return p;
}

void gps::rx_event_callback(uint16_t size) {
    if (huart->RxState == HAL_UART_STATE_READY) {
        HAL_UARTEx_ReceiveToIdle_DMA(huart, dma_buf, GPS_DMA_BUF_SIZE);
        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
    }
}

} // namespace GPS
