#pragma once

#include "main.h"
#include <stdint.h>
#include "stm32f4xx_hal.h"

namespace encoder
{
    class Encoder{
        public:
            Encoder(SPI_HandleTypeDef* hspi, GPIO_TypeDef* cs_port, uint16_t cs_pin, uint16_t offset = 0);

            HAL_StatusTypeDef read_raw_angle(uint16_t& raw_angle);
            HAL_StatusTypeDef read_angle_deg(float& angle_deg);
            HAL_StatusTypeDef clear_error();

            void set_offset(uint16_t offset);

        private:
            SPI_HandleTypeDef* hspi_;
            GPIO_TypeDef* cs_port_;
            uint16_t cs_pin_;
            uint16_t offset_;

            void cs_low();
            void cs_high();

            uint8_t even_parity_15(uint16_t value);
            uint16_t build_read_command(uint16_t addr);
            uint16_t build_frame(uint16_t value);

            HAL_StatusTypeDef transfer_16(uint16_t tx_word, uint16_t& rx_word);
            HAL_StatusTypeDef read_register(uint16_t addr, uint16_t& data_out);

    };
} // namespace encoder
