#pragma once

#include "main.h"
#include <stdint.h>
#include "stm32f4xx_hal.h"

namespace encoder
{
    class Encoder{
        public:
            Encoder(SPI_HandleTypeDef* hspi, GPIO_TypeDef* csPort,  uint16_t csPin, uint16_t offset = 0);

            HAL_StatusTypeDef readRawAngle(uint16_t& rawAngle);
            HAL_StatusTypeDef readAngleDeg(float& angleDeg);
            HAL_StatusTypeDef clearError();

            void setOffset(uint16_t offset);

        private:
            SPI_HandleTypeDef* hspi;
            GPIO_TypeDef* csPort;
            uint16_t csPin;
            uint16_t offset;

            void csLow();
            void csHigh();

            uint8_t evenParity15(uint16_t value);
            uint16_t buildReadCommand(uint16_t addr);
            uint16_t buildFrame(uint16_t value);

            HAL_StatusTypeDef transfer16(uint16_t txWord, uint16_t& rxWord);
            HAL_StatusTypeDef readRegister(uint16_t addr, uint16_t& dataOut);

    };
} // namespace encoder
