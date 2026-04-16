#include <hw/inc/gps.hpp>
#include <cstring>
#include <cstdio>
#include <cstdlib>

namespace telescope {

    void GPS::init() {
        std::memset(dma_buf_, 0, sizeof(dma_buf_));
        std::memset(line_buf_, 0, sizeof(line_buf_));

        HAL_UARTEx_ReceiveToIdle_DMA(huart_, dma_buf_, GPS_DMA_BUF_SIZE);
        __HAL_DMA_DISABLE_IT(huart_->hdmarx, DMA_IT_HT);
    }

    void GPS::process() {
        if (huart_ == nullptr) return;

        uint16_t write_pos = GPS_DMA_BUF_SIZE - __HAL_DMA_GET_COUNTER(huart_->hdmarx);

        while (read_pos_ != write_pos) {
            process_byte(dma_buf_[read_pos_]);
            read_pos_ = (read_pos_ + 1) % GPS_DMA_BUF_SIZE;
        }
    }

    void GPS::process_byte(uint8_t byte) {
        char c = static_cast<char>(byte);

        if (c == '$') {
            line_index_ = 0;
            line_buf_[line_index_++] = c;
            return;
        }

        if (c == '\r') return;

        if (c == '\n') {
            line_buf_[line_index_] = '\0';
            if (is_rmc(line_buf_)) {
                parse_rmc(line_buf_);
            } else if (is_gga(line_buf_)) {
                parse_gga(line_buf_);
            }
            line_index_ = 0;
            return;
        }

        if (line_index_ < GPS_BUFFER_SIZE - 1) {
            line_buf_[line_index_++] = c;
        }
    }

    bool GPS::is_rmc(const char* line) const {
        return (std::strncmp(line, "$GPRMC", 6) == 0 ||
                std::strncmp(line, "$GNRMC", 6) == 0);
    }

    void GPS::parse_rmc(const char* line) {
        char buffer[GPS_BUFFER_SIZE];
        strncpy(buffer, line, sizeof(buffer) - 1);
        buffer[sizeof(buffer) - 1] = '\0';

        if (strncmp(buffer, "$GNRMC", 6) != 0 &&
            strncmp(buffer, "$GPRMC", 6) != 0) {
            return;
        }

        // $GNRMC,hhmmss.ss,status,lat,N/S,lon,E/W,speed,course,ddmmyy
        char utc_str[15] = {0};
        char status_str[3] = {0};
        char lat_str[15] = {0};
        char ns_str[3] = {0};
        char lon_str[15] = {0};
        char ew_str[3] = {0};
        char speed_str[10] = {0};
        char course_str[10] = {0};
        char date_str[10] = {0};

        char* start = buffer;
        char* end;
        int field_idx = 0;

        while (start != nullptr && *start != '\0') {
            end = strchr(start, ',');

            if (end != nullptr) {
                *end = '\0';
            }

            switch (field_idx) {
                case 0: break;
                case 1: strncpy(utc_str, start, sizeof(utc_str)-1); break;
                case 2: strncpy(status_str, start, sizeof(status_str)-1); break;
                case 3: strncpy(lat_str, start, sizeof(lat_str)-1); break;
                case 4: strncpy(ns_str, start, sizeof(ns_str)-1); break;
                case 5: strncpy(lon_str, start, sizeof(lon_str)-1); break;
                case 6: strncpy(ew_str, start, sizeof(ew_str)-1); break;
                case 7: strncpy(speed_str, start, sizeof(speed_str)-1); break;
                case 8: strncpy(course_str, start, sizeof(course_str)-1); break;
                case 9: strncpy(date_str, start, sizeof(date_str)-1); break;
            }

            if (end != nullptr) {
                start = end + 1;
            } else {
                start = nullptr;
            }
            field_idx++;
        }

        if (strlen(utc_str) >= 6) {
            char h[3] = {utc_str[0], utc_str[1], '\0'};
            char m[3] = {utc_str[2], utc_str[3], '\0'};
            utc_hours = atoi(h);
            utc_minutes = atoi(m);
            utc_seconds = atof(&utc_str[4]);
        }

        fix = (status_str[0] == 'A');

        if (strlen(lat_str) > 0 && strlen(lon_str) > 0) {
            double raw_lat = atof(lat_str);
            int lat_deg = static_cast<int>(raw_lat / 100);
            double lat_min = raw_lat - (lat_deg * 100);
            latitude = lat_deg + (lat_min / 60.0);
            if (ns_str[0] == 'S') latitude = -latitude;

            double raw_lon = atof(lon_str);
            int lon_deg = static_cast<int>(raw_lon / 100);
            double lon_min = raw_lon - (lon_deg * 100);
            longitude = lon_deg + (lon_min / 60.0);
            if (ew_str[0] == 'W') longitude = -longitude;
        }

        if (strlen(date_str) >= 6) {
            char d[3] = {date_str[0], date_str[1], '\0'};
            char mo[3] = {date_str[2], date_str[3], '\0'};
            char y[3] = {date_str[4], date_str[5], '\0'};
            day = atoi(d);
            month = atoi(mo);
            year = 2000 + atoi(y);
        }
    }

    bool GPS::is_gga(const char* line) const {
        return (std::strncmp(line, "$GPGGA", 6) == 0 ||
                std::strncmp(line, "$GNGGA", 6) == 0);
    }

    void GPS::parse_gga(const char* line) {
        char buffer[GPS_BUFFER_SIZE];
        strncpy(buffer, line, sizeof(buffer) - 1);
        buffer[sizeof(buffer) - 1] = '\0';

        // $GNGGA,hhmmss.ss,lat,N/S,lon,E/W,fix,sats,hdop,alt,M,geoid,M,...
        char* start = buffer;
        char* end;
        int field_idx = 0;

        while (start != nullptr && *start != '\0') {
            end = strchr(start, ',');
            if (end != nullptr) {
                *end = '\0';
            }

            if (field_idx == 7) {
                num_satellites = static_cast<uint8_t>(atoi(start));
            }

            if (end != nullptr) {
                start = end + 1;
            } else {
                start = nullptr;
            }
            field_idx++;
        }
    }

    bool GPS::has_fix() const {
        return fix;
    }

    GpsPayload GPS::payload() const {
        GpsPayload p{};
        p.latitude_e7 = static_cast<int32_t>(latitude * 1e7);
        p.longitude_e7 = static_cast<int32_t>(longitude * 1e7);
        p.num_satellites = num_satellites;
        return p;
    }

    void GPS::rx_event_callback(uint16_t size) {
        if (huart_->RxState == HAL_UART_STATE_READY) {
            HAL_UARTEx_ReceiveToIdle_DMA(huart_, dma_buf_, GPS_DMA_BUF_SIZE);
            __HAL_DMA_DISABLE_IT(huart_->hdmarx, DMA_IT_HT);
        }
    }

} // namespace telescope
