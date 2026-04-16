#include <hw/inc/imu.hpp>

namespace telescope {

    void IMU::init() {
        HAL_Delay(700); // BNO055 boot time

        // ensure CONFIG mode first
        write_reg(IMU_reg.OPR_MODE, 0x00);
        HAL_Delay(30);

        // switch to NDOF
        write_reg(IMU_reg.OPR_MODE, 0x0C);
        HAL_Delay(30);
    }

    bool IMU::update() {
        if (i2c_ == nullptr) return false;

        // NDOF mode Euler: heading(2), roll(2), pitch(2) at 0x1A, LSB first
        // heading is in degrees * 16 (0 to 5759)
        uint8_t buf[2];
        if (!read_regs(EUL_HEADING, buf, 2)) return false;

        current_heading_ = static_cast<int16_t>(buf[0] | (buf[1] << 8));

        uint8_t calib;
        if (read_regs(CALIB_STAT, &calib, 1)) {
            current_calib_ = calib;
        }

        return true;
    }

    int16_t IMU::get_heading() {
        return current_heading_;
    }

    uint8_t IMU::get_calibration() {
        return current_calib_;
    }

    bool IMU::write_reg(uint8_t reg, uint8_t val) {
        return HAL_I2C_Mem_Write(i2c_, BNO055_ADDR, reg, 1, &val, 1, TIMEOUT_MS) == HAL_OK;
    }

    bool IMU::read_regs(uint8_t reg, uint8_t* buf, uint16_t len) {
        return HAL_I2C_Mem_Read(i2c_, BNO055_ADDR, reg, 1, buf, len, TIMEOUT_MS) == HAL_OK;
    }

} // namespace telescope
