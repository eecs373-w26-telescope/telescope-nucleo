#include <hw/inc/imu.hpp>

namespace imu {
namespace {

constexpr uint8_t BNO055_ADDR = 0x28 << 1;

// registers
constexpr uint8_t OPR_MODE    = 0x3D;
constexpr uint8_t EUL_HEADING = 0x1A;
constexpr uint8_t CALIB_STAT  = 0x35;

constexpr uint32_t TIMEOUT_MS = 100;

I2C_HandleTypeDef* i2c = nullptr;
int16_t current_heading = 0;
uint8_t current_calib = 0;

bool write_reg(uint8_t reg, uint8_t val) {
    return HAL_I2C_Mem_Write(i2c, BNO055_ADDR, reg, 1, &val, 1, TIMEOUT_MS) == HAL_OK;
}

bool read_regs(uint8_t reg, uint8_t* buf, uint16_t len) {
    return HAL_I2C_Mem_Read(i2c, BNO055_ADDR, reg, 1, buf, len, TIMEOUT_MS) == HAL_OK;
}

} // anonymous namespace

void init(I2C_HandleTypeDef* hi2c) {
    i2c = hi2c;

    // NDOF mode
    write_reg(OPR_MODE, 0x0C);
    HAL_Delay(50);
}

bool update() {
    if (i2c == nullptr) return false;

    // NDOF mode Euler: heading(2), roll(2), pitch(2) at 0x1A, LSB first
    // heading is in degrees * 16 (0 to 5759)
    uint8_t buf[2];
    if (!read_regs(EUL_HEADING, buf, 2)) return false;

    current_heading = static_cast<int16_t>(buf[0] | (buf[1] << 8));

    uint8_t calib;
    if (read_regs(CALIB_STAT, &calib, 1)) {
        current_calib = calib;
    }

    return true;
}

int16_t heading() {
    return current_heading;
}

uint8_t calibration() {
    return current_calib;
}

} // namespace imu
