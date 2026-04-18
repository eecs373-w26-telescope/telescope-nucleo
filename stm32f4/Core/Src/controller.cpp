/*

State machine implementation
- INIT
- READY
- SEARCH
- FOUND

*/

#include <hw/inc/encoder.hpp>
#include <hw/inc/filter.hpp>
#include <hw/inc/gps.hpp>
#include <hw/inc/imu.hpp>
#include <hw/inc/raspi.hpp>
#include <hw/inc/sd_card.hpp>
#include <hw/inc/touch.hpp>
#include <hw/inc/touchscreen.hpp>
#include <astro/inc/state_machine.h>
#include <astro/inc/astronomy.hpp>

#include "main.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal_uart.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <ctime>

// __DATE__ = "Mmm DD YYYY", __TIME__ = "HH:MM:SS"
namespace build_time {
    constexpr int d(char c) { return c - '0'; }
    constexpr int month() {
        const char* s = __DATE__;
        if (s[0]=='J' && s[1]=='a') return 1;
        if (s[0]=='F') return 2;
        if (s[0]=='M' && s[2]=='r') return 3;
        if (s[0]=='A' && s[1]=='p') return 4;
        if (s[0]=='M') return 5;
        if (s[0]=='J' && s[2]=='n') return 6;
        if (s[0]=='J') return 7;
        if (s[0]=='A') return 8;
        if (s[0]=='S') return 9;
        if (s[0]=='O') return 10;
        if (s[0]=='N') return 11;
        return 12;
    }
    constexpr int day()    { return (__DATE__[4]==' ' ? 0 : d(__DATE__[4]))*10 + d(__DATE__[5]); }
    constexpr int year()   { return d(__DATE__[7])*1000 + d(__DATE__[8])*100 + d(__DATE__[9])*10 + d(__DATE__[10]); }
    constexpr int hour()   { return d(__TIME__[0])*10 + d(__TIME__[1]); }
    constexpr int minute() { return d(__TIME__[3])*10 + d(__TIME__[4]); }
    constexpr int second() { return d(__TIME__[6])*10 + d(__TIME__[7]); }
}

extern "C" UART_HandleTypeDef huart1;
extern "C" UART_HandleTypeDef huart3;
extern "C" UART_HandleTypeDef huart6;
extern "C" I2C_HandleTypeDef hi2c1;
extern "C" SPI_HandleTypeDef hspi1;

namespace telescope {

    /***********************
    PORTS & PINS
    ************************/
    #define ENCODER_YAW_CS_PORT   GPIOC
    #define ENCODER_YAW_CS_PIN    GPIO_PIN_2  // D12 (PC2)
    #define ENCODER_PITCH_CS_PORT GPIOC
    #define ENCODER_PITCH_CS_PIN  GPIO_PIN_1  // D13 (PC1)

    #define LCD_CS_PORT   GPIOA
    #define LCD_CS_PIN    GPIO_PIN_4   // A0
    #define LCD_RST_PORT  GPIOC
    #define LCD_RST_PIN   GPIO_PIN_5   // A5
    #define LCD_DC_PORT   GPIOB
    #define LCD_DC_PIN    GPIO_PIN_13  // SCK
    #define LCD_LED_PORT  GPIOB
    #define LCD_LED_PIN   GPIO_PIN_15  // MOSI

    #define TOUCH_CS_PORT  GPIOC
    #define TOUCH_CS_PIN   GPIO_PIN_3   // D11
    #define TOUCH_IRQ_PORT GPIOB
    #define TOUCH_IRQ_PIN  GPIO_PIN_14  // MISO

    #define DEBUG_BUTTON_PORT GPIOC
    #define DEBUG_BUTTON_PIN  GPIO_PIN_4

    /***********************
    OFFSETS & FILTERS
    ************************/
    constexpr uint16_t ENCODER_FULL = 16384;
    constexpr uint16_t deg_to_raw(float deg) {
        return static_cast<uint16_t>((deg / 360.0f) * ENCODER_FULL) % ENCODER_FULL;
    }

    // OFFSET HEADING HERE
    constexpr uint16_t YAW_OFFSET   = deg_to_raw(30.5f);
    constexpr uint16_t PITCH_OFFSET = deg_to_raw(116.6f);

    // BNO055 heading: int16_t units of 1/16 deg, full circle = 360 * 16 = 5760
    // AS5048A raw: uint16_t 14-bit, full circle = 16384
    constexpr std::size_t FILTER_WINDOW = 8;
    filter::Filter<int16_t,  FILTER_WINDOW> imu_filter{5760};
    filter::Filter<uint16_t, FILTER_WINDOW> yaw_filter{16384};
    filter::Filter<uint16_t, FILTER_WINDOW> pitch_filter{16384};

    /***********************
    HARDWARE INITIALIZATION
    ************************/
    Encoder yaw_encoder(&hspi1, ENCODER_YAW_CS_PORT, ENCODER_YAW_CS_PIN, YAW_OFFSET);
    Encoder pitch_encoder(&hspi1, ENCODER_PITCH_CS_PORT, ENCODER_PITCH_CS_PIN, PITCH_OFFSET);

    GPS gps(&huart6);
    IMU imu(&hi2c1);
    RasPi raspi(&huart1);
    SDCard sd_card{};

    Touchscreen touchscreen(&hspi1,
        LCD_CS_PORT,  LCD_CS_PIN,
        LCD_RST_PORT, LCD_RST_PIN,
        LCD_LED_PORT, LCD_LED_PIN,
        LCD_DC_PORT,  LCD_DC_PIN);  // cs, rst, led, dc
    Touch xpt(&hspi1, TOUCH_CS_PORT, TOUCH_CS_PIN, TOUCH_IRQ_PORT, TOUCH_IRQ_PIN);


    /***********************
    TIMING
    ************************/
    constexpr uint32_t hz_to_ms(uint32_t hz) { return 1000u / hz; }

    constexpr uint32_t IMU_HZ           = 10;
    constexpr uint32_t GPS_HZ           = 1;
    constexpr uint32_t ENCODER_HZ       = 10;
    constexpr uint32_t SERIAL_DEBUG_HZ  = 5;
    constexpr uint32_t STATE_MACHINE_HZ = 10;
    constexpr uint32_t AZ_DEBUG_HZ      = 5;
    constexpr uint32_t ASTRO_DEBUG_HZ   = 5;
    constexpr uint32_t SG_DEBUG_HZ      = 5;

    constexpr uint32_t IMU_INTERVAL_MS           = hz_to_ms(IMU_HZ);
    constexpr uint32_t GPS_INTERVAL_MS           = hz_to_ms(GPS_HZ);
    constexpr uint32_t ENCODER_INTERVAL_MS       = hz_to_ms(ENCODER_HZ);
    constexpr uint32_t SERIAL_INTERVAL_MS        = hz_to_ms(SERIAL_DEBUG_HZ);
    constexpr uint32_t TOUCH_DEBOUNCE_MS         = 500;
    constexpr uint32_t STATE_MACHINE_INTERVAL_MS = hz_to_ms(STATE_MACHINE_HZ);
    constexpr uint32_t AZ_DEBUG_INTERVAL_MS      = hz_to_ms(AZ_DEBUG_HZ);
    constexpr uint32_t ASTRO_DEBUG_INTERVAL_MS   = hz_to_ms(ASTRO_DEBUG_HZ);
    constexpr uint32_t SG_DEBUG_INTERVAL_MS      = hz_to_ms(SG_DEBUG_HZ);

    float filtered_imu_heading_deg = 0.0f;
    uint16_t filtered_yaw_raw   = 0;
    uint16_t filtered_pitch_raw = 0;
    uint32_t last_imu_tick = 0;
    uint32_t last_gps_tick = 0;
    uint32_t last_ping_tick = 0;
    uint32_t last_encoder_tick = 0;
    uint32_t last_serial_tick = 0;
    uint32_t last_touch_tick = 0;
    uint32_t last_sm_tick = 0;
    uint32_t last_az_debug_tick = 0;
    uint32_t last_astro_debug_tick = 0;
    uint32_t last_sg_debug_tick = 0;

    /***********************
    STATE MACHINE
    ************************/
    StateMachine state_machine(touchscreen, sd_card, raspi);

    auto init() -> void {
        const char* msg = "telescope init\r\n";
        HAL_UART_Transmit(&huart3, reinterpret_cast<const uint8_t*>(msg), 16, 100);

        raspi.init();

        imu.init();
        {
            char imu_buf[48];
            int imu_len = snprintf(imu_buf, sizeof(imu_buf), "IMU calib: %s\r\n", imu.calibration_loaded() ? "loaded from flash" : "none (fresh boot)");
            HAL_UART_Transmit(&huart3, reinterpret_cast<uint8_t*>(imu_buf), static_cast<uint16_t>(imu_len), 100);
        }
        gps.init();

        yaw_encoder.clear_error();
        pitch_encoder.clear_error();

        touchscreen.init();
        touchscreen.draw_main();

        xpt.init(); 
        //TODO: ADD ANYTHING ELSE TO INITIALIZE THE TOUCH SCREEN HERE

        int mount_res = sd_card.mount();
        int open_res  = sd_card.open_catalogue("0:catalogue.bin");
        char sd_buf[64];        int sd_len = snprintf(sd_buf, sizeof(sd_buf), "SD mount:%d open:%d fopen_err:%d\r\n",
                              mount_res, open_res, sd_card.last_open_error());
        HAL_UART_Transmit(&huart3, reinterpret_cast<uint8_t*>(sd_buf), static_cast<uint16_t>(sd_len), 100);

        char time_buf[64];
        int time_len = snprintf(time_buf, sizeof(time_buf),
                                "Fallback time: %04d-%02d-%02d %02d:%02d:%02d\r\n",
                                build_time::year(), build_time::month(), build_time::day(),
                                build_time::hour(), build_time::minute(), build_time::second());
        HAL_UART_Transmit(&huart3, reinterpret_cast<uint8_t*>(time_buf), static_cast<uint16_t>(time_len), 100);

        if (open_res == 0) {
            std::vector<DSO> dump;
            // Search whole sky but limit to enough objects to find 10 of each
            sd_card.search_objects_in_bounds(0.0f, 359.99f, -90.0f, 90.0f, dump, 200);

            const char* hdr = "Catalogue Dump (First 10 M and 10 NGC):\r\n";
            HAL_UART_Transmit(&huart3, reinterpret_cast<const uint8_t*>(hdr), 40, 100);

            int m_count = 0, n_count = 0;
            for (const auto& obj : dump) {
                bool is_ngc = (obj.name.find("N") != std::string::npos);
                if (is_ngc && n_count < 10) {
                    n_count++;
                } else if (!is_ngc && m_count < 10) {
                    m_count++;
                } else {
                    continue;
                }

                char obj_buf[80];
                int obj_len = snprintf(obj_buf, sizeof(obj_buf), "  %s RA=%.2fh DEC=%.2fd mag=%.1f\r\n",
                                       obj.name.c_str(),
                                       static_cast<double>(obj.pos.right_ascension),
                                       static_cast<double>(obj.pos.declination),
                                       static_cast<double>(obj.brightness));
                HAL_UART_Transmit(&huart3, reinterpret_cast<uint8_t*>(obj_buf), static_cast<uint16_t>(obj_len), 100);

                if (m_count >= 10 && n_count >= 10) break;
            }
        }

        state_machine.init();
    }

    //TODO: MOVE THIS ELSEWHERE
    bool prev_button_pressed = false;

    [[noreturn]] auto loop() -> void {
        for (;;) {
            // Send periodic sensor updates to the raspi
            raspi.process();
            gps.process();

            uint32_t now = HAL_GetTick();

            if (now - last_imu_tick >= IMU_INTERVAL_MS) {
                last_imu_tick = now;
                if (imu.update()) {
                    IMUPayload payload{};
                    payload.heading = imu_filter.update(imu.get_heading());
                    payload.calibration = imu.get_calibration();
                    filtered_imu_heading_deg = static_cast<float>(payload.heading) / 16.0f;
                    raspi.send_imu(payload);

                    static bool calib_saved = false;
                    static uint8_t mag_stable_count = 0;
                    if (!calib_saved) {
                        if ((payload.calibration & 0x03) == 0x03) {
                            if (++mag_stable_count >= 10) {
                                calib_saved = imu.save_calibration();
                            }
                        } else {
                            mag_stable_count = 0;
                        }
                    }
                }
            }

            if (now - last_gps_tick >= GPS_INTERVAL_MS) {
                last_gps_tick = now;
                if (gps.has_fix()) {
                    raspi.send_gps(gps.payload());
                }
            }

            if (now - last_encoder_tick >= ENCODER_INTERVAL_MS) {
                last_encoder_tick = now;
                EncoderPayload payload{};
                uint16_t raw = 0;
                bool yaw_ok = (yaw_encoder.read_raw_angle(raw, true) == HAL_OK);
                if (yaw_ok) {
                    filtered_yaw_raw = yaw_filter.update(raw);
                    payload.yaw_raw = filtered_yaw_raw;
                }
                bool pitch_ok = (pitch_encoder.read_raw_angle(raw) == HAL_OK);
                if (pitch_ok) {
                    filtered_pitch_raw = pitch_filter.update(raw);
                    payload.pitch_raw = filtered_pitch_raw;
                }
                if (yaw_ok || pitch_ok) raspi.send_encoder(payload);
            }

            if (now - last_serial_tick >= SERIAL_INTERVAL_MS) {
                last_serial_tick = now;
                bool imu_ok = imu.update();
                uint8_t cal = imu.get_calibration();
                constexpr float RAW_TO_DEG_DBG = 360.0f / 16384.0f;
                float yaw_deg = filtered_yaw_raw   * RAW_TO_DEG_DBG;
                float pitch_deg = filtered_pitch_raw * RAW_TO_DEG_DBG;
                char buf[120];
                int len = snprintf(buf, sizeof(buf),
                                   "IMU: %s  HDG: %.1f  CAL: S%d G%d A%d M%d  GPS: %s  SAT: %d  YAW: %.1f  PIT: %.1f\r\n",
                                   imu_ok ? "OK" : "FAIL",
                                   filtered_imu_heading_deg,
                                   (cal >> 6) & 3, (cal >> 4) & 3,
                                   (cal >> 2) & 3, cal & 3,
                                   gps.has_fix() ? "FIX" : "---",
                                   gps.num_satellites,
                                   static_cast<double>(yaw_deg),
                                   static_cast<double>(pitch_deg));
                HAL_UART_Transmit(&huart3, reinterpret_cast<uint8_t*>(buf),
                                  static_cast<uint16_t>(len), 100);

                EquatorialCoordinates eqc = state_machine.current_eqc();
                char sm_buf[120];
                int sm_len = snprintf(sm_buf, sizeof(sm_buf),
                                      "SM: state=%d target=M%d found=%d fov_objs=%d search_res=%d RA=%.2f DEC=%.2f%s\r\n",
                                      static_cast<int>(state_machine.current_state()),
                                      state_machine.selected_messier_id(),
                                      state_machine.has_selected_object() ? 1 : 0,
                                      state_machine.fov_object_count(),
                                      state_machine.last_search_result(),
                                      static_cast<double>(eqc.right_ascension),
                                      static_cast<double>(eqc.declination),
                                      gps.has_fix() ? "" : (raspi.has_raspi_time() ? " [RASPI-TIME]" : " [BUILD-TIME]"));
                HAL_UART_Transmit(&huart3, reinterpret_cast<uint8_t*>(sm_buf),
                                  static_cast<uint16_t>(sm_len), 100);

                const FOV& fov = state_machine.current_fov();
                char fov_buf[80];
                int fov_len = snprintf(fov_buf, sizeof(fov_buf),
                                       "FOV: center=%.2fh/%.2fd r=%.2fd objects=%d\r\n",
                                       static_cast<double>(fov.center_pos.right_ascension),
                                       static_cast<double>(fov.center_pos.declination),
                                       static_cast<double>(fov.radius),
                                       static_cast<int>(fov.objects.size()));
                HAL_UART_Transmit(&huart3, reinterpret_cast<uint8_t*>(fov_buf), static_cast<uint16_t>(fov_len), 100);

                for (const auto& obj : fov.objects) {
                    char obj_buf[64];
                    int obj_len = snprintf(obj_buf, sizeof(obj_buf), "  %s RA=%.2f DEC=%.2f mag=%.1f\r\n",
                                          obj.name.c_str(),
                                          static_cast<double>(obj.pos.right_ascension),
                                          static_cast<double>(obj.pos.declination),
                                          static_cast<double>(obj.brightness));
                    HAL_UART_Transmit(&huart3, reinterpret_cast<uint8_t*>(obj_buf), static_cast<uint16_t>(obj_len), 100);
                }

                StateSyncPayload sync{};
                sync.state    = static_cast<uint8_t>(state_machine.current_state());
                sync.flags    = (touchscreen.get_selected_catalogue() == CatalogueType::NGC) ? 0x01 : 0x00;
                sync.sequence = 0;
                raspi.send_state_sync(sync);
            }

            // Touchscreen input processing
            if(xpt.is_pressed() && (now - last_touch_tick >= TOUCH_DEBOUNCE_MS)){
                last_touch_tick = now;
                if(xpt.process()){
                    char btn = xpt.get_button();
                    char action = touchscreen.update_display_string(btn);
                    if(touchscreen.get_search_status()){
                        if(action == 'C'){
                            touchscreen.gocancel();
                        }
                    }
                    else{
                        if(action == 'V'){
                            touchscreen.view_change();
                        }
                        else if(action == 'T'){
                            imu.set_tare();
                            touchscreen.tare_feedback();
                        }
                        else{
                            touchscreen.normal_process(action, btn);
                        }
                    }
                }
            }

            touchscreen.tick(now);

            // State machine tick
            //TODO: I DONT THINK THIS STATE_MACHINE_INTERVAL IS NECESSARY
            if (now - last_sm_tick >= STATE_MACHINE_INTERVAL_MS) {
                last_sm_tick = now;
                constexpr float RAW_TO_DEG = 360.0f / 16384.0f;
                float yaw_deg_sm   = filtered_yaw_raw   * RAW_TO_DEG;
                float pitch_deg_sm = filtered_pitch_raw * RAW_TO_DEG;

                // Map angles > 180 to negative altitude values (e.g. 270 deg -> -90 deg)
                if (pitch_deg_sm > 180.0f) {
                    pitch_deg_sm -= 360.0f;
                }

                float AZ_OFFSET_DEG = -48.2;
                float azimuth_deg = std::fmod(filtered_imu_heading_deg + yaw_deg_sm + 360.0f + AZ_OFFSET_DEG, 360.0f);

                constexpr float FALLBACK_LAT = 42.29243326092064f;
                constexpr float FALLBACK_LON = -83.71498310538729f;
                float lat = gps.has_fix() ? static_cast<float>(gps.latitude)  : FALLBACK_LAT;
                float lon = gps.has_fix() ? static_cast<float>(gps.longitude) : FALLBACK_LON;
                UTC time{};
                if (gps.has_fix()) {
                    time.year   = gps.year;
                    time.month  = gps.month;
                    time.day    = gps.day;
                    time.hour   = gps.utc_hours;
                    time.minute = gps.utc_minutes;
                    time.second = static_cast<int>(gps.utc_seconds);
                } else if (raspi.has_raspi_time()) {
                    time = raspi.raspi_time();
                } else {
                    time.year   = build_time::year();
                    time.month  = build_time::month();
                    time.day    = build_time::day();
                    time.hour   = build_time::hour();
                    time.minute = build_time::minute();
                    time.second = build_time::second();
                }

                {
                    TimeModePayload tm{};
                    if (gps.has_fix())               tm.mode = TIME_MODE_SATELLITE;
                    else if (raspi.has_raspi_time()) tm.mode = TIME_MODE_RASPI;
                    else                             tm.mode = TIME_MODE_COMPILE;
                    raspi.send_time_mode(tm);
                }

                state_machine.update_sensors(pitch_deg_sm, azimuth_deg, lat, lon, time);
                state_machine.tick();

                if (state_machine.selected_messier_id() >= 0) {
                    state_machine.send_dso_target_packet();
                }

                {
                    const FOV& fov = state_machine.current_fov();
                    FovObjectsPayload pkt{};
                    pkt.count = 0;
                    for (const auto& obj : fov.objects) {
                        if (pkt.count >= FOV_OBJECTS_MAX) break;
                        float x = 0.0f, y = 0.0f;
                        state_machine.project_gnomonic_local_for_current_fov(obj.pos, x, y);
                        
                        // Parse name for ID and mode
                        int id = 0;
                        uint8_t mode = 0; // 0=M, 1=N
                        if (obj.name.find("N") != std::string::npos) {
                            id = std::stoi(obj.name.substr(1));
                            mode = 1;
                        } else if (obj.name.find("M") != std::string::npos) {
                            id = std::stoi(obj.name.substr(1));
                            mode = 0;
                        }

                        auto& e = pkt.objects[pkt.count++];
                        e.catalog_id   = static_cast<uint16_t>(id);
                        e.catalog_mode = mode;
                        e.x_e4 = static_cast<int16_t>(std::clamp(x, -1.0f, 1.0f) * 10000.0f);
                        e.y_e4 = static_cast<int16_t>(std::clamp(y, -1.0f, 1.0f) * 10000.0f);
                    }
                    raspi.send_fov_objects(pkt, pkt.count);
                }
                
                // Only show guidance arrow if we are actively searching for a target
                {
                    SearchGuidancePayload sg{};
                    if (state_machine.has_selected_object() && 
                        state_machine.current_state() == StateMachine::TelescopeState::SEARCH) {
                        sg.has_target = 1;

                        HorizontalCoordinates target_hc = state_machine.get_target_horizontal(state_machine.selected_object().pos);
                        HorizontalCoordinates current_hc = state_machine.current_hc();

                        // Convert to radians for trig functions
                        constexpr float DEG_TO_RAD = 3.14159265f / 180.0f;
                        float tgt_alt_rad = target_hc.altitude * DEG_TO_RAD;
                        float tgt_az_rad  = target_hc.azimuth * DEG_TO_RAD;
                        float cur_alt_rad = current_hc.altitude * DEG_TO_RAD;
                        float cur_az_rad  = current_hc.azimuth * DEG_TO_RAD;

                        // Difference in azimuth
                        float dAz_rad = tgt_az_rad - cur_az_rad;

                        // Calculate singularity-free viewplane projection
                        float dx = cosf(tgt_alt_rad) * sinf(dAz_rad);
                        float dy = sinf(tgt_alt_rad) * cosf(cur_alt_rad) - 
                                cosf(tgt_alt_rad) * sinf(cur_alt_rad) * cosf(dAz_rad);

                        float mag = sqrtf(dx * dx + dy * dy);
                        if (mag > 1e-6f) {
                            sg.dx_e4 = static_cast<int16_t>((dx / mag) * 10000.0f);
                            sg.dy_e4 = static_cast<int16_t>((-dy / mag) * 10000.0f); // Negate for screen Y-down
                        }
                        
                        float dist_deg = state_machine.get_distance_to_selected_object();
                        sg.distance_e2 = static_cast<int16_t>(
                            std::min(dist_deg * 100.0f, 32767.0f));
                    }
                    raspi.send_search_guidance(sg);

                    if (now - last_sg_debug_tick >= SG_DEBUG_INTERVAL_MS) {
                        last_sg_debug_tick = now;
                        char sg_buf[96];
                        int sg_len = snprintf(sg_buf, sizeof(sg_buf),
                            "SG: has_tgt=%d dx=%.4f dy=%.4f dist=%.1fdeg\r\n",
                            sg.has_target,
                            sg.dx_e4 / 10000.0f,
                            sg.dy_e4 / 10000.0f,
                            sg.distance_e2 / 100.0f);
                        HAL_UART_Transmit(&huart3, reinterpret_cast<uint8_t*>(sg_buf),
                                          static_cast<uint16_t>(sg_len), 100);
                    }
                }

                EquatorialCoordinates eqc = state_machine.current_eqc();
                HorizontalCoordinates hc = state_machine.current_hc();
                const FOV& fov = state_machine.current_fov();

                if (now - last_astro_debug_tick >= ASTRO_DEBUG_INTERVAL_MS) {
                    last_astro_debug_tick = now;
                    char astro_buf[256];
                    int astro_len = snprintf(astro_buf, sizeof(astro_buf),
                        "ASTRO:\r\n"
                        "  ALT: %.2f deg\r\n"
                        "  AZ : %.2f deg\r\n"
                        "  LAT: %.5f  LON: %.5f\r\n"
                        "  UTC: %04d-%02d-%02d %02d:%02d:%02d\r\n"
                        "  RA : %.2f hr\r\n"
                        "  DEC: %.2f deg\r\n"
                        "  FOV r: %.2f deg  objs:%d%s\r\n",
                        static_cast<double>(hc.altitude),
                        static_cast<double>(hc.azimuth),
                        static_cast<double>(lat),
                        static_cast<double>(lon),
                        time.year, time.month, time.day,
                        time.hour, time.minute, time.second,
                        static_cast<double>(eqc.right_ascension),
                        static_cast<double>(eqc.declination),
                        static_cast<double>(fov.radius),
                        static_cast<int>(fov.objects.size()),
                        gps.has_fix() ? "" : " [FALLBACK]"
                    );
                    HAL_UART_Transmit(&huart3,
                        reinterpret_cast<uint8_t*>(astro_buf),
                        static_cast<uint16_t>(astro_len),
                        100);

                    for (const auto& obj : fov.objects) {
                        char obj_buf[96];
                        int obj_len = snprintf(obj_buf, sizeof(obj_buf),
                            "  %s RA=%.2f DEC=%.2f mag=%.1f\r\n",
                            obj.name.c_str(),
                            static_cast<double>(obj.pos.right_ascension),
                            static_cast<double>(obj.pos.declination),
                            static_cast<double>(obj.brightness)
                        );
                        HAL_UART_Transmit(&huart3,
                            reinterpret_cast<uint8_t*>(obj_buf),
                            static_cast<uint16_t>(obj_len),
                            100);
                    }
                }
            }

            // Button press sends debug packet
            bool button_pressed = (HAL_GPIO_ReadPin(DEBUG_BUTTON_PORT, DEBUG_BUTTON_PIN) == GPIO_PIN_RESET);
            if (button_pressed && !prev_button_pressed) {
                DebugPayload dbg{};
                const char* msg = "PING";
                __builtin_memcpy(dbg.data, msg, 4);
                raspi.send_debug(dbg);
            }
            prev_button_pressed = button_pressed;
        }
    }

} // namespace telescope

extern "C" {

void PostInit() {
    telescope::init();
}

void Loop() {
    telescope::loop();
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart) {
    if (huart->Instance == USART1) {
        telescope::raspi.tx_complete_callback();
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t Size) {
    if (huart->Instance == USART1) {
        telescope::raspi.rx_event_callback(Size);
    }
    if (huart->Instance == USART6) {
        telescope::gps.rx_event_callback(Size);
    }
}

}
