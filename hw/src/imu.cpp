#include <hw/inc/imu.hpp>
#include <cmath>

namespace imu {
namespace {

constexpr uint8_t MAG_ADDR  = 0x1E << 1;
constexpr uint8_t ACC_ADDR  = 0x19 << 1; // in case we want to use this

// mag regs
constexpr uint8_t CRA_REG_M = 0x00;
constexpr uint8_t CRB_REG_M = 0x01;
constexpr uint8_t MR_REG_M  = 0x02;
constexpr uint8_t OUT_X_H_M = 0x03;

// acc regs
constexpr uint8_t CTRL_REG1_A = 0x20;
constexpr uint8_t OUT_X_L_A   = 0x28 | 0x80; // auto increment

constexpr uint32_t TIMEOUT_MS = 50;

I2C_HandleTypeDef* i2c = nullptr;
int16_t current_heading = 0;

bool write_reg(uint8_t dev_addr, uint8_t reg, uint8_t val) {
	return HAL_I2C_Mem_Write(i2c, dev_addr, reg, 1, &val, 1, TIMEOUT_MS) == HAL_OK;
}

bool read_regs(uint8_t dev_addr, uint8_t reg, uint8_t* buf, uint16_t len) {
	return HAL_I2C_Mem_Read(i2c, dev_addr, reg, 1, buf, len, TIMEOUT_MS) == HAL_OK;
}

} // namespace imu

void init(I2C_HandleTypeDef* hi2c) {
	i2c = hi2c;

	// mag: 15hz, continuous
	write_reg(MAG_ADDR, CRA_REG_M, 0x10); // 15 Hz
	write_reg(MAG_ADDR, CRB_REG_M, 0x20); // +/- 1.3 gauss, most accuracy
	write_reg(MAG_ADDR, MR_REG_M,  0x00); // continuous

	// acc: all axes @ 10hz
	write_reg(ACC_ADDR, CTRL_REG1_A, 0x27);
}

bool update() {
	if (i2c == nullptr) return false;

	// read mag
	uint8_t mag_buf[6];
	if (!read_regs(MAG_ADDR, OUT_X_H_M, mag_buf, 6)) return false;

	int16_t mx = static_cast<int16_t>((mag_buf[0] << 8) | mag_buf[1]);
	int16_t mz = static_cast<int16_t>((mag_buf[2] << 8) | mag_buf[3]);
	int16_t my = static_cast<int16_t>((mag_buf[4] << 8) | mag_buf[5]);

	// read acc 
	uint8_t acc_buf[6];
	if (!read_regs(ACC_ADDR, OUT_X_L_A, acc_buf, 6)) return false;

	int16_t ax = static_cast<int16_t>((acc_buf[1] << 8) | acc_buf[0]);
	int16_t ay = static_cast<int16_t>((acc_buf[3] << 8) | acc_buf[2]);
	int16_t az = static_cast<int16_t>((acc_buf[5] << 8) | acc_buf[4]);

	// tilt compensation
	float ax_f = static_cast<float>(ax);
	float ay_f = static_cast<float>(ay);
	float az_f = static_cast<float>(az);

	float roll  = atan2f(ay_f, az_f);
	float pitch = atan2f(-ax_f, sqrtf(ay_f * ay_f + az_f * az_f));

	float mx_f = static_cast<float>(mx);
	float my_f = static_cast<float>(my);
	float mz_f = static_cast<float>(mz);

	float mx_comp = mx_f * cosf(pitch) + mz_f * sinf(pitch);
	float my_comp = mx_f * sinf(roll) * sinf(pitch)
	              + my_f * cosf(roll)
	              - mz_f * sinf(roll) * cosf(pitch);

	float heading_rad = atan2f(my_comp, mx_comp);
	if (heading_rad < 0.0f) heading_rad += 2.0f * 3.14159265f;

	// convert to degrees * 16, down to 1/16 degree accuracy with 16 bits of int
	float heading_deg = heading_rad * (180.0f / 3.14159265f);
	current_heading = static_cast<int16_t>(heading_deg * 16.0f);

	return true;
}

int16_t heading() {
	return current_heading;
}

} // namespace imu
