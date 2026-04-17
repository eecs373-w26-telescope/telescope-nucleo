#include <hw/inc/encoder.hpp>
#include <cstdio>
#include <cstdint>
#include <cstring>


namespace telescope {

    //Constructor
    Encoder::Encoder(SPI_HandleTypeDef* hspi, GPIO_TypeDef* csPort, uint16_t csPin)
    : hspi_(hspi), csPort_(csPort), csPin_(csPin){}

    // Drive CS Low
    void Encoder::cs_low(){
        HAL_GPIO_WritePin(csPort_, csPin_, GPIO_PIN_RESET);
    }

    //Drive CS High
    void Encoder::cs_high(){
        HAL_GPIO_WritePin(csPort_, csPin_, GPIO_PIN_SET);
    }

    //Generate the Parity bit
    uint8_t Encoder::even_parity_15(uint16_t value){
        uint8_t count = 0;
        for(int i = 0; i < 15; i++){
            if(value & (1 << i)){
                count ++;
            }
        }
        return (count % 2) ? 1:0;
    }

    // Generate the read command
    uint16_t Encoder::build_read_command(uint16_t addr){
        uint16_t cmd = 0;
        cmd |= (1u << 14); //Read bit set to 1
        cmd |= (addr & 0x3FFF);
        uint8_t paritybit = even_parity_15(cmd);
        cmd |= (static_cast<uint16_t> (paritybit) << 15);
        return cmd;
    }

    // Generate a random frame for recieving data A dumy write
    uint16_t Encoder::build_frame(uint16_t value){
        uint16_t frame = value & 0x7FFF;
        uint8_t parity = even_parity_15(frame);
        frame |= (static_cast<uint16_t> (parity) << 15);
        return frame;
    }

    //Transfer data and receive at the same time
    HAL_StatusTypeDef Encoder::transfer_16(uint16_t txword, uint16_t& rxword){
        uint8_t tx[2]; // Sending out two bytes of data
        uint8_t rx[2]; // Receiving two bytes of data

        tx[0] = static_cast<uint8_t>((txword >> 8) & 0xFF);
        tx[1] = static_cast<uint8_t>(txword & 0xFF);

        cs_low();
        HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(hspi_, tx, rx, 2, HAL_MAX_DELAY);
        cs_high();

        if (status != HAL_OK){
            return status;
        }
        rxword = (static_cast<uint16_t> (rx[0] << 8)) | rx[1];
        return HAL_OK;

    }

    HAL_StatusTypeDef Encoder::read_register(uint16_t addr, uint16_t& data_Out){
        uint16_t rx1 = 0;
        uint16_t rx2 = 0;

        uint16_t readCmd = build_read_command(addr);
        uint16_t emptyCmd = build_frame(encoder_reg::AS5048_NOP);

        //Send Read Command
        HAL_StatusTypeDef status = transfer_16(readCmd, rx1);
        if(status != HAL_OK){
            return status;
        }

        // Get the data
        status = transfer_16(emptyCmd, rx2);
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

    HAL_StatusTypeDef Encoder::read_raw_angle(uint16_t& rawangle){
        return read_register(encoder_reg::AS5048_ANGLE, rawangle);
    }

    HAL_StatusTypeDef Encoder::read_angle_deg(float& angleDeg){
        uint16_t raw = 0;
        HAL_StatusTypeDef status = read_raw_angle(raw);
        if(status != HAL_OK){
            return status;
        }
        angleDeg = (360.0f * static_cast<float>(raw)) / 16384.0f;

        return HAL_OK;
    }
    
    HAL_StatusTypeDef Encoder::clear_error(){
        uint16_t dummy = 0;
        return read_register(encoder_reg::AS5048_CLRERR, dummy);
    }
} //namespace telescope
