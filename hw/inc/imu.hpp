#pragma once

#include "main.h"
#include <cstdint>

// BNO055 9-DoF IMU on I2C1

namespace imu {

void init(I2C_HandleTypeDef* hi2c);

bool update();

int16_t heading();

uint8_t calibration();

} // namespace imu
