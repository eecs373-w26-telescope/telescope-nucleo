#pragma once

#include "main.h"
#include <stdint.h>

namespace telescope{
    struct touch_reg{
        static constexpr uint8_t READ_X = 0xD0;
        static constexpr uint8_t READ_Y = 0x90;
    };

    struct touch_calibration{
        static constexpr uint16_t raw_x_min = 270;
        static constexpr uint16_t raw_x_max = 3800;
        static constexpr uint16_t raw_y_min = 300;
        static constexpr uint16_t raw_y_max = 3900;
    };

    class Touch{
        public:
            Touch(SPI_HandleTypeDef* hspi, GPIO_TypeDef* cs_port, uint16_t cs_pin,
                  GPIO_TypeDef* irq_port, uint16_t irq_pin);
            void init();
            bool is_pressed();
            bool process(bool config_mode);
            char get_button();
            void calibration();

        private:
            SPI_HandleTypeDef* hspi_ = nullptr;
            GPIO_TypeDef* cs_port_ = nullptr;
            uint16_t cs_pin_;
            GPIO_TypeDef* irq_port_ = nullptr;
            uint16_t irq_pin_;

            touch_calibration calib_{};

            char button_ = '\0';

            void cs_low();
            void cs_high();

            uint16_t read_12_bit(uint8_t cmd);
            bool read_raw(uint16_t* x, uint16_t* y);
            bool read_average_raw(uint16_t* x, uint16_t* y);
            bool convert_px(uint16_t raw_x, uint16_t raw_y, uint16_t* x, uint16_t* y);
            char keypad_conversion(uint16_t px, uint16_t py, bool config_mode);
    };
}
