/*

State machine implementation 
- INIT
- READY
- SEARCH
- FOUND 

*/

#include <hw/encoder.hpp>
#include <hw/gps.hpp>
#include <hw/raspi.hpp>
#include <hw/sd_card.hpp>
#include <hw/touchscreen.hpp>

#include "main.h"
#include "stm32l4xx.h"

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
        // TO-DO: Disable interrupts

        // initialize peripherals, uart, i2c

        // setup timers

        // TO-DO: Enable interrupts
    }

    [[noreturn]] auto loop() -> void {
        for (;;) {
            GPIO_PinState pin_state = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, pin_state);
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

// add any overwritten interrupt or HAL functions here -- most should trigger a state change 


}
