#pragma once

#include "main.h"
#include <stdint.h>

extern "C" SPI_HandleTypeDef hspi1;
#define TOUCH_SPI_HANDLE hspi1

#define TOUCH_CS_GPIO_Port   GPIOC
#define TOUCH_CS_Pin         GPIO_PIN_3   // D11

#define TOUCH_IRQ_GPIO_Port  GPIOB
#define TOUCH_IRQ_Pin        GPIO_PIN_14  // MISO

#define LCD_W 480
#define LCD_H 320

#define RAW_X_MIN 270
#define RAW_X_MAX 3800
#define RAW_Y_MIN 300
#define RAW_Y_MAX 3900

namespace touch{
    class Touch{
        public:
            uint16_t raw_x, raw_y, real_x, real_y;
            uint8_t pressed;
            char button;
            void init();
            bool touch_pressed();
            uint16_t touch_read_12bit(uint8_t cmd);
            bool touch_read_raw(uint16_t* x, uint16_t* y);
            bool touch_read_average_raw(uint16_t* x, uint16_t* y);
            bool touch_convert(uint16_t raw_x, uint16_t raw_y, uint16_t* x, uint16_t* y);
            char keypad_conversion(uint16_t px, uint16_t py);
            bool touch_process();
            void touch_calibration();
        private:
    };
}
