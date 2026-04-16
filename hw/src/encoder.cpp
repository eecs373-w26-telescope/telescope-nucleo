#include <hw/inc/encoder.hpp>
#include <cstdio>
#include <cstdint>
#include <cstring>

namespace encoder
{
    static constexpr uint16_t AS5048_NOP      = 0x0000;
    static constexpr uint16_t AS5048_CLRERR   = 0x0001;
    static constexpr uint16_t AS5048_DIAG_AGC = 0x3FFD;
    static constexpr uint16_t AS5048_MAG      = 0x3FFE;
    static constexpr uint16_t AS5048_ANGLE    = 0x3FFF;

    static constexpr uint16_t AS5048_FULL = 16384;

    Encoder::Encoder(SPI_HandleTypeDef* hspi, GPIO_TypeDef* cs_port, uint16_t cs_pin, uint16_t offset)
    : hspi_(hspi), cs_port_(cs_port), cs_pin_(cs_pin), offset_(offset){}

    void Encoder::set_offset(uint16_t new_offset){
        offset_ = new_offset % AS5048_FULL;
    }

    void Encoder::cs_low(){
        HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_RESET);
    }

    void Encoder::cs_high(){
        HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_SET);
    }

    uint8_t Encoder::even_parity_15(uint16_t value){
        uint8_t count = 0;
        for(int i = 0; i < 15; i++){
            if(value & (1 << i)){
                count++;
            }
        }
        return (count % 2) ? 1 : 0;
    }

    uint16_t Encoder::build_read_command(uint16_t addr){
        uint16_t cmd = 0;
        cmd |= (1u << 14);
        cmd |= (addr & 0x3FFF);
        uint8_t parity = even_parity_15(cmd);
        cmd |= (static_cast<uint16_t>(parity) << 15);
        return cmd;
    }

    uint16_t Encoder::build_frame(uint16_t value){
        uint16_t frame = value & 0x7FFF;
        uint8_t parity = even_parity_15(frame);
        frame |= (static_cast<uint16_t>(parity) << 15);
        return frame;
    }

    HAL_StatusTypeDef Encoder::transfer_16(uint16_t tx_word, uint16_t& rx_word){
        uint8_t tx[2];
        uint8_t rx[2];

        tx[0] = static_cast<uint8_t>((tx_word >> 8) & 0xFF);
        tx[1] = static_cast<uint8_t>(tx_word & 0xFF);

        cs_low();
        HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(hspi_, tx, rx, 2, HAL_MAX_DELAY);
        cs_high();

        if(status != HAL_OK) return status;

        rx_word = (static_cast<uint16_t>(rx[0] << 8)) | rx[1];
        return HAL_OK;
    }

    HAL_StatusTypeDef Encoder::read_register(uint16_t addr, uint16_t& data_out){
        uint16_t rx1 = 0;
        uint16_t rx2 = 0;

        uint16_t read_cmd  = build_read_command(addr);
        uint16_t empty_cmd = build_frame(AS5048_NOP);

        HAL_StatusTypeDef status = transfer_16(read_cmd, rx1);
        if(status != HAL_OK) return status;

        status = transfer_16(empty_cmd, rx2);
        if(status != HAL_OK) return status;

        if(rx2 & 0x4000) return HAL_ERROR;

        data_out = rx2 & 0x3FFF;
        return HAL_OK;
    }

    HAL_StatusTypeDef Encoder::read_raw_angle(uint16_t& raw_angle, bool invert){
        uint16_t raw = 0;
        HAL_StatusTypeDef status = read_register(AS5048_ANGLE, raw);
        if(status != HAL_OK) return status;
        if (invert) raw = (AS5048_FULL - raw) % AS5048_FULL;
        raw_angle = (raw + AS5048_FULL - offset_) % AS5048_FULL;
        return HAL_OK;
    }

    HAL_StatusTypeDef Encoder::read_angle_deg(float& angle_deg, bool invert){
        uint16_t raw = 0;
        HAL_StatusTypeDef status = read_raw_angle(raw, invert);
        if(status != HAL_OK) return status;
        angle_deg = (360.0f * static_cast<float>(raw)) / 16384.0f;
        return HAL_OK;
    }

    HAL_StatusTypeDef Encoder::clear_error(){
        uint16_t dummy = 0;
        return read_register(AS5048_CLRERR, dummy);
    }

} // namespace encoder
