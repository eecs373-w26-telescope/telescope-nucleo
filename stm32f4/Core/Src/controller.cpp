/*

State machine implementation
- INIT
- READY
- SEARCH
- FOUND

*/

#include <hw/inc/encoder.hpp>
#include <hw/inc/gps.hpp>
#include <hw/inc/imu.hpp>
#include <hw/inc/raspi.hpp>
#include <hw/inc/sd_card.hpp>
#include <hw/inc/touchscreen.hpp>

#include "main.h"
#include "stm32f4xx.h"

extern "C" UART_HandleTypeDef huart1;
extern "C" UART_HandleTypeDef huart6;
extern "C" I2C_HandleTypeDef hi2c1;

namespace telescope {

    /*
        I2C
    */

    /*
        SPI
    */

    /*
      Initialization sequence
    */
    GPS::gps gps_sensor;

    constexpr uint32_t IMU_INTERVAL_MS = 100; // 10 Hz
    constexpr uint32_t GPS_INTERVAL_MS = 1000; // 1 Hz
    constexpr uint32_t PING_INTERVAL_MS = 1000; // 1 Hz
    uint32_t last_imu_tick = 0;
    uint32_t last_gps_tick = 0;
    uint32_t last_ping_tick = 0;

    auto init() -> void {
        raspi::init(&huart1);
        imu::init(&hi2c1);
        gps_sensor.init(&huart6);
    }

    bool prev_button_pressed = false;

    #define DEBUG_BUTTON_PORT GPIOC
    #define DEBUG_BUTTON_PIN  GPIO_PIN_5

    [[noreturn]] auto loop() -> void {
        for (;;) {
            raspi::process();
            gps_sensor.process();

            uint32_t now = HAL_GetTick();

            // Periodic ping pong
            // if (now - last_ping_tick >= PING_INTERVAL_MS) {
            //     last_ping_tick = now;
            //     DebugPayload dbg{};
            //     raspi::send_debug(dbg);
            // }

            if (now - last_imu_tick >= IMU_INTERVAL_MS) {
                last_imu_tick = now;
                if (imu::update()) {
                    ImuPayload payload{};
                    payload.heading = imu::heading();
                    raspi::send_imu(payload);
                }
            }

            if (now - last_gps_tick >= GPS_INTERVAL_MS) {
                last_gps_tick = now;
                if (gps_sensor.has_fix()) {
                    raspi::send_gps(gps_sensor.payload());
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
