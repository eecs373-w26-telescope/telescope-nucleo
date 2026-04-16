#pragma once

#include "main.h"

namespace spi_mode {

inline void set_mode0(SPI_HandleTypeDef* hspi) {
    __HAL_SPI_DISABLE(hspi);
    hspi->Instance->CR1 &= ~SPI_CR1_CPHA;
    hspi->Init.CLKPhase = SPI_PHASE_1EDGE;
    __HAL_SPI_ENABLE(hspi);
}

inline void set_mode1(SPI_HandleTypeDef* hspi) {
    __HAL_SPI_DISABLE(hspi);
    hspi->Instance->CR1 |= SPI_CR1_CPHA;
    hspi->Init.CLKPhase = SPI_PHASE_2EDGE;
    __HAL_SPI_ENABLE(hspi);
}

} // namespace spi_mode
