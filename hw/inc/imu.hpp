#pragma once

#include "main.h"
#include <cstdint>

// BNO055 9-DoF IMU on I2C1

namespace telescope {
    constexpr uint32_t TIMEOUT_MS = 100;

    struct IMU_reg {
        static constexpr uint8_t BNO055_ADDR      = 0x28 << 1;
        static constexpr uint8_t OPR_MODE         = 0x3D;
        static constexpr uint8_t EUL_HEADING      = 0x1A;
        static constexpr uint8_t CALIB_STAT       = 0x35;
        static constexpr uint8_t ACC_OFFSET_X_LSB = 0x55;
        static constexpr uint8_t CALIB_OFFSET_LEN = 22;
    };

    // STM32F405 sector 11: 0x080E0000-0x080FFFFF (128KB, last sector)
    static constexpr uint32_t CALIB_FLASH_ADDR = 0x080E0000;
    static constexpr uint32_t CALIB_MAGIC      = 0xCA1B0055;

    struct CalibData {
        uint32_t magic;
        uint8_t  offsets[IMU_reg::CALIB_OFFSET_LEN];
    };

    class IMU {
        public:
            IMU(I2C_HandleTypeDef* hi2c) : i2c_{hi2c} {}
            IMU() = default;
            void init();

            bool update();

            int16_t get_heading();
            uint8_t get_calibration();
            bool    calibration_loaded() const { return calib_loaded_; }
            bool    save_calibration();

        private:
            I2C_HandleTypeDef* i2c_ = nullptr;
            int16_t current_heading_ = 0;
            uint8_t current_calib_ = 0;
            bool    calib_loaded_ = false;

            bool write_reg(uint8_t reg, uint8_t val);
            bool write_regs(uint8_t reg, const uint8_t* buf, uint16_t len);
            bool read_regs(uint8_t reg, uint8_t* buf, uint16_t len);
            bool flash_load(CalibData& out) const;
            bool flash_save(const CalibData& data) const;
    };


} // namespace telescope 
