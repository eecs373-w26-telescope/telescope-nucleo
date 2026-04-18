#include <hw/inc/imu.hpp>
#include "stm32f4xx_hal.h"

namespace telescope {

    void IMU::init() {
        HAL_Delay(700); // BNO055 boot time

        write_reg(IMU_reg::OPR_MODE, 0x00);
        HAL_Delay(30);

        CalibData data;
        calib_loaded_ = flash_load(data);
        if (calib_loaded_) {
            write_regs(IMU_reg::ACC_OFFSET_X_LSB, data.offsets, IMU_reg::CALIB_OFFSET_LEN);
        }

        write_reg(IMU_reg::OPR_MODE, 0x0C);
        HAL_Delay(30);
    }

    bool IMU::update() {
        if (i2c_ == nullptr) return false;

        // NDOF mode Euler: heading(2), roll(2), pitch(2) at 0x1A, LSB first
        // heading is in degrees * 16 (0 to 5759)
        uint8_t buf[2];
        if (!read_regs(IMU_reg::EUL_HEADING, buf, 2)) return false;

        current_heading_ = static_cast<int16_t>(buf[0] | (buf[1] << 8));

        uint8_t calib;
        if (read_regs(IMU_reg::CALIB_STAT, &calib, 1)) {
            current_calib_ = calib;
        }

        return true;
    }

    int16_t IMU::get_heading() {
        int16_t tared = current_heading_ - tare_offset_;
        if (tared < 0) tared += 5760;
        return tared;
    }

    void IMU::set_tare() {
        tare_offset_ = current_heading_;
    }

    uint8_t IMU::get_calibration() {
        return current_calib_;
    }

    bool IMU::save_calibration() {
        write_reg(IMU_reg::OPR_MODE, 0x00);
        HAL_Delay(30);

        CalibData data{};
        if (!read_regs(IMU_reg::ACC_OFFSET_X_LSB, data.offsets, IMU_reg::CALIB_OFFSET_LEN)) {
            write_reg(IMU_reg::OPR_MODE, 0x0C);
            HAL_Delay(30);
            return false;
        }
        data.magic = CALIB_MAGIC;

        bool ok = flash_save(data);

        write_reg(IMU_reg::OPR_MODE, 0x0C);
        HAL_Delay(30);
        return ok;
    }

    bool IMU::flash_load(CalibData& out) const {
        const CalibData* stored = reinterpret_cast<const CalibData*>(CALIB_FLASH_ADDR);
        if (stored->magic != CALIB_MAGIC) return false;
        out = *stored;
        return true;
    }

    bool IMU::flash_save(const CalibData& data) const {
        if (HAL_FLASH_Unlock() != HAL_OK) return false;

        FLASH_EraseInitTypeDef erase{};
        erase.TypeErase    = FLASH_TYPEERASE_SECTORS;
        erase.Sector       = FLASH_SECTOR_11;
        erase.NbSectors    = 1;
        erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;

        uint32_t sector_error = 0;
        if (HAL_FLASHEx_Erase(&erase, &sector_error) != HAL_OK) {
            HAL_FLASH_Lock();
            return false;
        }

        // write CalibData word by word (4 bytes at a time)
        const uint32_t* src = reinterpret_cast<const uint32_t*>(&data);
        constexpr uint32_t word_count = (sizeof(CalibData) + 3) / 4;
        for (uint32_t i = 0; i < word_count; i++) {
            if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                                  CALIB_FLASH_ADDR + i * 4,
                                  src[i]) != HAL_OK) {
                HAL_FLASH_Lock();
                return false;
            }
        }

        HAL_FLASH_Lock();
        return true;
    }

    bool IMU::write_reg(uint8_t reg, uint8_t val) {
        return HAL_I2C_Mem_Write(i2c_, IMU_reg::BNO055_ADDR, reg, 1, &val, 1, TIMEOUT_MS) == HAL_OK;
    }

    bool IMU::write_regs(uint8_t reg, const uint8_t* buf, uint16_t len) {
        return HAL_I2C_Mem_Write(i2c_, IMU_reg::BNO055_ADDR, reg, 1,
                                 const_cast<uint8_t*>(buf), len, TIMEOUT_MS) == HAL_OK;
    }

    bool IMU::read_regs(uint8_t reg, uint8_t* buf, uint16_t len) {
        return HAL_I2C_Mem_Read(i2c_, IMU_reg::BNO055_ADDR, reg, 1, buf, len, TIMEOUT_MS) == HAL_OK;
    }

} // namespace telescope
