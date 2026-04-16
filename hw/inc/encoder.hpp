#pragma once

#include "main.h"
#include <stdint.h>
#include "stm32f4xx_hal.h"


namespace telescope {
    struct encoder_reg {
        static constexpr uint16_t AS5048_NOP       = 0x0000;
        static constexpr uint16_t AS5048_CLRERR    = 0x0001;
        static constexpr uint16_t AS5048_DIAG_AGC  = 0x3FFD;
        static constexpr uint16_t AS5048_MAG       = 0x3FFE;
        static constexpr uint16_t AS5048_ANGLE     = 0x3FFF;
    };

    class Encoder{
        public:
            Encoder(SPI_HandleTypeDef* hspi, GPIO_TypeDef* csPort,  uint16_t csPin);
            auto init() = default; // TODO: add init function for DMA 
    
            HAL_StatusTypeDef read_raw_angle(uint16_t& rawAngle);
            HAL_StatusTypeDef read_angle_deg(float& angleDeg);
            HAL_StatusTypeDef clear_error();
    
        private:
            SPI_HandleTypeDef* hspi_ = nullptr;
            GPIO_TypeDef* csPort_ = nullptr;
            uint16_t csPin_;
    
            void cs_low();
            void cs_high();
    
            uint8_t even_parity_15(uint16_t value);
            uint16_t build_read_command(uint16_t addr);
            uint16_t build_frame(uint16_t value);
    
            HAL_StatusTypeDef transfer_16(uint16_t txWord, uint16_t& rxWord);
            HAL_StatusTypeDef read_register(uint16_t addr, uint16_t& dataOut);
    };
} //namespace telescope

