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
    constexpr uint32_t IMU_INTERVAL_MS = 100; // 10 Hz
    uint32_t last_imu_tick = 0;

    auto init() -> void {
        raspi::init(&huart1);
        imu::init(&hi2c1);
    }

    bool prev_button_pressed = false;

    // TODO: assign debug button pin once soldered
    #define DEBUG_BUTTON_PORT GPIOA
    #define DEBUG_BUTTON_PIN  GPIO_PIN_0

    [[noreturn]] auto loop() -> void {
        for (;;) {
            raspi::process();

            // LED reflects raspi connection status
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1,
                raspi::connection_active() ? GPIO_PIN_SET : GPIO_PIN_RESET);

            uint32_t now = HAL_GetTick();
            if (now - last_imu_tick >= IMU_INTERVAL_MS) {
                last_imu_tick = now;
                if (imu::update()) {
                    ImuPayload payload{};
                    payload.heading = imu::heading();
                    raspi::send_imu(payload);
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
}

}
