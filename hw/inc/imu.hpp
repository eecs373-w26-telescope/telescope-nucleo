#pragma once

#include "main.h"
#include <cstdint>

// BNO055 9-DoF IMU on I2C1

namespace telescope {
    constexpr uint32_t TIMEOUT_MS = 100;

    struct IMU_reg {
        static constexpr uint8_t BNO055_ADDR = 0x28 << 1;
        static constexpr uint8_t OPR_MODE    = 0x3D;
        static constexpr uint8_t EUL_HEADING = 0x1A;
        static constexpr uint8_t CALIB_STAT  = 0x35;
    };

    class IMU {
        public:
            IMU(I2C_HandleTypeDef* hi2c) : i2c_{hi2c} {}
            IMU() = default;
            void init();

            bool update();

            int16_t get_heading();

            uint8_t get_calibration();

        private:
            I2C_HandleTypeDef* i2c_ = nullptr;
            int16_t current_heading_ = 0;
            uint8_t current_calib_ = 0;

            bool write_reg(uint8_t reg, uint8_t val);        
            bool read_regs(uint8_t reg, uint8_t* buf, uint16_t len);
    }; 


} // namespace telescope 
