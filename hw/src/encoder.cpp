#include <hw/inc/encoder.hpp>
#include <cstdio>
#include <cstdint>
#include <cstring>

namespace encoder
{
    static constexpr uint16_t AS5048_NOP       = 0x0000;
    static constexpr uint16_t AS5048_CLRERR    = 0x0001;
    static constexpr uint16_t AS5048_DIAG_AGC  = 0x3FFD;
    static constexpr uint16_t AS5048_MAG       = 0x3FFE;
    static constexpr uint16_t AS5048_ANGLE     = 0x3FFF;

    static constexpr uint16_t AS5048_FULL = 16384;

    //Constructor
    Encoder::Encoder(SPI_HandleTypeDef* hspi, GPIO_TypeDef* csPort, uint16_t csPin, uint16_t offset)
    : hspi(hspi), csPort(csPort), csPin(csPin), offset(offset){}

    void Encoder::setOffset(uint16_t newOffset){
        offset = newOffset % AS5048_FULL;
    }

    // Drive CS Low
    void Encoder::csLow(){
        HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_RESET);
    }

    //Drive CS High
    void Encoder::csHigh(){
        HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_SET);
    }
    //Generate the Parity bit
    uint8_t Encoder::evenParity15(uint16_t value){
        uint8_t count = 0;
        for(int i = 0; i < 15; i++){
            if(value & (1 << i)){
                count ++;
            }
        }
        return (count % 2) ? 1:0;
    }
    // Generate the read command
    uint16_t Encoder::buildReadCommand(uint16_t addr){
        uint16_t cmd = 0;
        cmd |= (1u << 14); //Read bit set to 1
        cmd |= (addr & 0x3FFF);
        uint8_t paritybit = evenParity15(cmd);
        cmd |= (static_cast<uint16_t> (paritybit) << 15);
        return cmd;
    }

    // Generate a random frame for recieving data A dumy write
    uint16_t Encoder::buildFrame(uint16_t value){
        uint16_t frame = value & 0x7FFF;
        uint8_t parity = evenParity15(frame);
        frame |= (static_cast<uint16_t> (parity) << 15);
        return frame;
    }
    //Transfer data and receive at the same time
    HAL_StatusTypeDef Encoder::transfer16(uint16_t txword, uint16_t& rxword){
        uint8_t tx[2]; // Sending out two bytes of data
        uint8_t rx[2]; // Receiving two bytes of data

        tx[0] = static_cast<uint8_t>((txword >> 8) & 0xFF);
        tx[1] = static_cast<uint8_t>(txword & 0xFF);

        csLow();
        HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(hspi, tx, rx, 2, HAL_MAX_DELAY);
        csHigh();

        if (status != HAL_OK){
            return status;
        }
        rxword = (static_cast<uint16_t> (rx[0] << 8)) | rx[1];
        return HAL_OK;

    }
    HAL_StatusTypeDef Encoder::readRegister(uint16_t addr, uint16_t& data_Out){
        uint16_t rx1 = 0;
        uint16_t rx2 = 0;

        uint16_t readCmd = buildReadCommand(addr);
        uint16_t emptyCmd = buildFrame(AS5048_NOP);

        //Send Read Command
        HAL_StatusTypeDef status = transfer16(readCmd, rx1);
        if(status != HAL_OK){
            return status;
        }

        // Get the data
        status = transfer16(emptyCmd, rx2);
        if(status != HAL_OK){
            return status;
        }
        //Check the error flag
        if(rx2 & 0x4000){
            return HAL_ERROR;
        }
        //Process the data
        data_Out = rx2 & 0x3FFF;
        return HAL_OK;
    }
    HAL_StatusTypeDef Encoder::readRawAngle(uint16_t& rawangle){
        uint16_t raw = 0;
        HAL_StatusTypeDef status = readRegister(AS5048_ANGLE, raw);
        if (status != HAL_OK) return status;
        rawangle = (raw + AS5048_FULL - offset) % AS5048_FULL;
        return HAL_OK;
    }

    HAL_StatusTypeDef Encoder::readAngleDeg(float& angleDeg){
        uint16_t raw = 0;
        HAL_StatusTypeDef status = readRawAngle(raw);
        if(status != HAL_OK){
            return status;
        }
        angleDeg = (360.0f * static_cast<float>(raw)) / 16384.0f;

        return HAL_OK;
    }
    HAL_StatusTypeDef Encoder::clearError(){
        uint16_t dummy = 0;
        return readRegister(AS5048_CLRERR, dummy);
    }
} // namespace encoder
