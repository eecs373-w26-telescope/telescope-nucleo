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
    constexpr uint16_t YAW_OFFSET   = deg_to_raw(27.0f);
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
    Encoder yaw_encoder(&hspi1, ENCODER_YAW_CS_PORT, ENCODER_YAW_CS_PIN);
    Encoder pitch_encoder(&hspi1, ENCODER_PITCH_CS_PORT, ENCODER_PITCH_CS_PIN);

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
    constexpr uint32_t IMU_INTERVAL_MS = 100; // 10 Hz
    constexpr uint32_t GPS_INTERVAL_MS = 1000; // 1 Hz
    constexpr uint32_t ENCODER_INTERVAL_MS = 100; // 10 Hz
    constexpr uint32_t SERIAL_INTERVAL_MS = 1000; // 1 Hz
    constexpr uint32_t TOUCH_DEBOUNCE_MS  = 500;
    constexpr uint32_t STATE_MACHINE_INTERVAL_MS = 100; // 10 Hz
    uint32_t last_imu_tick = 0;
    uint32_t last_gps_tick = 0;
    uint32_t last_ping_tick = 0;
    uint32_t last_encoder_tick = 0;
    uint32_t last_serial_tick = 0;
    uint32_t last_touch_tick = 0;
    uint32_t last_sm_tick = 0;

    /***********************
    STATE MACHINE
    ************************/
    StateMachine state_machine(touchscreen, sd_card, raspi);

    auto init() -> void {
        const char* msg = "telescope init\r\n";
        HAL_UART_Transmit(&huart3, reinterpret_cast<const uint8_t*>(msg), 16, 100);

        raspi.init();

        imu.init();
        gps.init();

        yaw_encoder.clear_error();
        pitch_encoder.clear_error();

        touchscreen.init();
        touchscreen.draw_main();

        xpt.init(); 
        //TODO: ADD ANYTHING ELSE TO INITIALIZE THE TOUCH SCREEN HERE

        sd_card.mount();
        sd_card.open_catalogue("catalogue.bin");

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
                    raspi.send_imu(payload);
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
                bool yaw_ok = (yaw_encoder.read_raw_angle(raw) == HAL_OK);
                if (yaw_ok) payload.yaw_raw = yaw_filter.update(raw);
                bool pitch_ok = (pitch_encoder.read_raw_angle(raw) == HAL_OK);
                if (pitch_ok) payload.pitch_raw = pitch_filter.update(raw);
                if (yaw_ok || pitch_ok) raspi.send_encoder(payload);
            }

            if (now - last_serial_tick >= SERIAL_INTERVAL_MS) {
                last_serial_tick = now;
                bool imu_ok = imu.update();
                uint8_t cal = imu.get_calibration();
                float yaw_deg = 0.0f;
                float pitch_deg = 0.0f;
                yaw_encoder.read_angle_deg(yaw_deg); //TODO: DID THIS FUNCTION CHANGE?
                pitch_encoder.read_angle_deg(pitch_deg);
                char buf[120];
                int len = snprintf(buf, sizeof(buf),
                                   "IMU: %s  HDG: %.1f  CAL: S%d G%d A%d M%d  GPS: %s  SAT: %d  YAW: %.1f  PIT: %.1f\r\n",
                                   imu_ok ? "OK" : "FAIL",
                                   static_cast<float>(imu.get_heading()) / 16.0f,
                                   (cal >> 6) & 3, (cal >> 4) & 3,
                                   (cal >> 2) & 3, cal & 3,
                                   gps.has_fix() ? "FIX" : "---",
                                   gps.num_satellites,
                                   static_cast<double>(yaw_deg),
                                   static_cast<double>(pitch_deg));
                HAL_UART_Transmit(&huart3, reinterpret_cast<uint8_t*>(buf),
                                  static_cast<uint16_t>(len), 100);
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
                        else{
                            touchscreen.normal_process(action, btn);
                        }
                    }
                }
            }

            // State machine tick
            //TODO: I DONT THINK THIS STATE_MACHINE_INTERVAL IS NECESSARY
            if (now - last_sm_tick >= STATE_MACHINE_INTERVAL_MS) {
                last_sm_tick = now;
                float yaw_deg_sm = 0.0f;
                float pitch_deg_sm = 0.0f;
                yaw_encoder.read_angle_deg(yaw_deg_sm); //TODO: DID THIS FUNCTION CHANGE?
                pitch_encoder.read_angle_deg(pitch_deg_sm);

                float lat = static_cast<float>(gps.latitude);
                float lon = static_cast<float>(gps.longitude);
                UTC time{};
                time.year = gps.year;
                time.month = gps.month;
                time.day = gps.day;
                time.hour = gps.utc_hours;
                time.minute = gps.utc_minutes;
                time.second = static_cast<int>(gps.utc_seconds);

                state_machine.update_sensors(pitch_deg_sm, yaw_deg_sm, lat, lon, time);
                state_machine.tick();
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
