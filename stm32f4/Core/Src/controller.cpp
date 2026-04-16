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

#include "main.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal_uart.h"
#include <cstdio>

extern "C" UART_HandleTypeDef huart1;
extern "C" UART_HandleTypeDef huart3;
extern "C" UART_HandleTypeDef huart6;
extern "C" I2C_HandleTypeDef hi2c1;
extern "C" SPI_HandleTypeDef hspi1;

namespace telescope {

    /*
        I2C
    */

    /*
        SPI
    */
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

    constexpr uint16_t ENCODER_FULL = 16384;
    constexpr uint16_t deg_to_raw(float deg) {
        return static_cast<uint16_t>((deg / 360.0f) * ENCODER_FULL) % ENCODER_FULL;
    }

    // OFFSET HEADING HERE
    constexpr uint16_t YAW_OFFSET   = deg_to_raw(27.0f);
    constexpr uint16_t PITCH_OFFSET = deg_to_raw(116.6f);

    encoder::Encoder* yaw_encoder = nullptr;
    encoder::Encoder* pitch_encoder = nullptr;

    telescope::Touchscreen touchscreen(&hspi1,
        LCD_CS_PORT,  LCD_CS_PIN,
        LCD_RST_PORT, LCD_RST_PIN,
        LCD_LED_PORT, LCD_LED_PIN,
        LCD_DC_PORT,  LCD_DC_PIN);  // cs, rst, led, dc
    Touch xpt;

    // BNO055 heading: int16_t units of 1/16 deg, full circle = 360 * 16 = 5760
    // AS5048A raw: uint16_t 14-bit, full circle = 16384
    constexpr std::size_t FILTER_WINDOW = 8;
    filter::Filter<int16_t,  FILTER_WINDOW> imu_filter{5760};
    filter::Filter<uint16_t, FILTER_WINDOW> yaw_filter{16384};
    filter::Filter<uint16_t, FILTER_WINDOW> pitch_filter{16384};

    /*
      Initialization sequence
    */
    GPS::gps gps_sensor;

    constexpr uint32_t IMU_INTERVAL_MS = 100; // 10 Hz
    constexpr uint32_t GPS_INTERVAL_MS = 1000; // 1 Hz
    constexpr uint32_t ENCODER_INTERVAL_MS = 100; // 10 Hz
    constexpr uint32_t SERIAL_INTERVAL_MS = 1000; // 1 Hz
    constexpr uint32_t TOUCH_DEBOUNCE_MS  = 500;
    uint32_t last_imu_tick = 0;
    uint32_t last_gps_tick = 0;
    uint32_t last_ping_tick = 0;
    uint32_t last_encoder_tick = 0;
    uint32_t last_serial_tick = 0;
    uint32_t last_touch_tick = 0;

    auto init() -> void {
        const char* msg = "telescope init\r\n";
        HAL_UART_Transmit(&huart3, reinterpret_cast<const uint8_t*>(msg), 16, 100);

        raspi::init(&huart1);
        imu::init(&hi2c1);
        gps_sensor.init(&huart6);

        static encoder::Encoder yaw_enc(&hspi1, ENCODER_YAW_CS_PORT, ENCODER_YAW_CS_PIN, YAW_OFFSET);
        yaw_encoder = &yaw_enc;
        yaw_encoder->clear_error();

        static encoder::Encoder pitch_enc(&hspi1, ENCODER_PITCH_CS_PORT, ENCODER_PITCH_CS_PIN, PITCH_OFFSET);
        pitch_encoder = &pitch_enc;
        pitch_encoder->clear_error();

        xpt.init();
        touchscreen.init();
        touchscreen.draw_main();

        static SDCard::SDCard sd;
        int sd_result = sd.mount();
        char sd_msg[48];
        int sd_len = snprintf(sd_msg, sizeof(sd_msg), "SD mount: %s\r\n", sd_result == 0 ? "OK" : "FAIL");
        HAL_UART_Transmit(&huart3, reinterpret_cast<uint8_t*>(sd_msg), static_cast<uint16_t>(sd_len), 100);
    }

    bool prev_button_pressed = false;

    #define DEBUG_BUTTON_PORT GPIOC
    #define DEBUG_BUTTON_PIN  GPIO_PIN_4

    [[noreturn]] auto loop() -> void {
        for (;;) {
            raspi::process();
            gps_sensor.process();

            uint32_t now = HAL_GetTick();

            if (now - last_imu_tick >= IMU_INTERVAL_MS) {
                last_imu_tick = now;
                if (imu::update()) {
                    ImuPayload payload{};
                    payload.heading = imu_filter.update(imu::heading());
                    payload.calibration = imu::calibration();
                    raspi::send_imu(payload);
                }
            }

            if (now - last_gps_tick >= GPS_INTERVAL_MS) {
                last_gps_tick = now;
                if (gps_sensor.has_fix()) {
                    raspi::send_gps(gps_sensor.payload());
                }
            }

            if (now - last_encoder_tick >= ENCODER_INTERVAL_MS) {
                last_encoder_tick = now;
                EncoderPayload payload{};
                uint16_t raw = 0;
                bool yaw_ok = (yaw_encoder->read_raw_angle(raw, true) == HAL_OK);
                if (yaw_ok) payload.yaw_raw = yaw_filter.update(raw);
                bool pitch_ok = (pitch_encoder->read_raw_angle(raw) == HAL_OK);
                if (pitch_ok) payload.pitch_raw = pitch_filter.update(raw);
                if (yaw_ok || pitch_ok) raspi::send_encoder(payload);
            }

            if (now - last_serial_tick >= SERIAL_INTERVAL_MS) {
                last_serial_tick = now;
                bool imu_ok = imu::update();
                uint8_t cal = imu::calibration();
                float yaw_deg = 0.0f;
                float pitch_deg = 0.0f;
                yaw_encoder->read_angle_deg(yaw_deg, true);
                pitch_encoder->read_angle_deg(pitch_deg);
                char buf[120];
                int len = snprintf(buf, sizeof(buf),
                                   "IMU: %s  HDG: %.1f  CAL: S%d G%d A%d M%d  GPS: %s  SAT: %d  YAW: %.1f  PIT: %.1f\r\n",
                                   imu_ok ? "OK" : "FAIL",
                                   static_cast<float>(imu::heading()) / 16.0f,
                                   (cal >> 6) & 3, (cal >> 4) & 3,
                                   (cal >> 2) & 3, cal & 3,
                                   gps_sensor.has_fix() ? "FIX" : "---",
                                   gps_sensor.num_satellites,
                                   static_cast<double>(yaw_deg),
                                   static_cast<double>(pitch_deg));
                HAL_UART_Transmit(&huart3, reinterpret_cast<uint8_t*>(buf),
                                  static_cast<uint16_t>(len), 100);
            }

            if(xpt.touch_pressed() && (now - last_touch_tick >= TOUCH_DEBOUNCE_MS)){
                last_touch_tick = now;
                if(xpt.touch_process()){
                    char action = touchscreen.update_display_string(xpt.button);
                    touchscreen.normal_process(action, xpt.button);
                }
            }

            // Button press sends debug packet
            bool button_pressed = (HAL_GPIO_ReadPin(DEBUG_BUTTON_PORT, DEBUG_BUTTON_PIN) == GPIO_PIN_RESET);
            if (button_pressed && !prev_button_pressed) {
                DebugPayload dbg{};
                const char* msg = "PING";
                __builtin_memcpy(dbg.data, msg, 4);
                raspi::send_debug(dbg);
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
        raspi::tx_complete_callback();
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t Size) {
    if (huart->Instance == USART1) {
        raspi::rx_event_callback(Size);
    }
    if (huart->Instance == USART6) {
        telescope::gps_sensor.rx_event_callback(Size);
    }
}

}
