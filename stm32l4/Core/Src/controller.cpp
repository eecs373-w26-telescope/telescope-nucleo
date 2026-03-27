/*

State machine implementation 
- INIT
- READY
- SEARCH
- FOUND 

*/

#include <hw/inc/encoder.hpp>
#include <hw/inc/gps.hpp>
#include <hw/inc/raspi.hpp>
#include <hw/inc/sd_card.hpp>
#include <hw/inc/touchscreen.hpp>

#include "main.h"
#include "stm32l4xx.h"

extern "C" UART_HandleTypeDef hlpuart1;

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
    auto init() -> void {
        raspi::init(&hlpuart1);
    }

    bool prev_button_pressed = false;

    [[noreturn]] auto loop() -> void {
        for (;;) {
            raspi::process();

            // LED reflects raspi connection status
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7,
                raspi::connection_active() ? GPIO_PIN_SET : GPIO_PIN_RESET);

            // Button press (PC13 is active low on Nucleo) sends debug packet
            bool button_pressed = (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET);
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
    if (huart->Instance == LPUART1) {
        raspi::tx_complete_callback();
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t Size) {
    if (huart->Instance == LPUART1) {
        raspi::rx_event_callback(Size);
    }
}

}
