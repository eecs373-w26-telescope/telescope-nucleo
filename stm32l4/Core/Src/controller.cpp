/*

State machine implementation 
- INIT
- READY
- SEARCH
- FOUND 

*/

#include <state_machine.hpp>
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
            switch (currentState) {
                case TelescopeState::INIT:
                    handle_init();
                    break;
                case TelescopeState::READY:
                    handle_ready();
                    break;
                case TelescopeState::SEARCH:
                    handle_search();
                    break;
                case TelescopeState::FOUND:
                    handle_found();
                    break;
                default:
                    // error handling
                    break;
            }
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
