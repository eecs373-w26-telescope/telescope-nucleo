#pragma once

#include "main.h"
#include <cstdint>

namespace imu {

void init(I2C_HandleTypeDef* hi2c);

bool update();

int16_t heading();

} // namespace imu
