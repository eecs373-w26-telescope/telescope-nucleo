#include <hw/inc/touch.hpp>
#include <cstdio>
#include <cstdint>
#include <cstring>

namespace touch
{
    static constexpr uint16_t TOP_BAR_H = 60;
    static constexpr uint16_t OPS_W = 140;
    static constexpr uint16_t KEYPAD_W = LCD_W - OPS_W;
    static constexpr uint16_t CONTENT_Y = TOP_BAR_H;
    static constexpr uint16_t CONTENT_H = LCD_H - TOP_BAR_H;
    static constexpr uint16_t GAP = 10;
    static constexpr uint16_t KEY_COLS = 3;
    static constexpr uint16_t KEY_ROWS = 4;
    static constexpr uint16_t KEY_W = (KEYPAD_W - GAP * (KEY_COLS+1)) / KEY_COLS;
    static constexpr uint16_t KEY_H = (CONTENT_H - GAP * (KEY_ROWS+1)) / KEY_ROWS;
    static constexpr uint16_t OPS_X = KEYPAD_W;
    static constexpr uint16_t OPS_BTN_W = OPS_W - 2 * GAP;
    static constexpr uint16_t OPS_BTN_H = (CONTENT_H - 4 * GAP)/3;

    void Touch::init(){
        HAL_GPIO_WritePin(TOUCH_CS_GPIO_Port, TOUCH_CS_Pin, GPIO_PIN_SET);
    }

    bool Touch::touch_pressed(){
        return (HAL_GPIO_ReadPin(TOUCH_IRQ_GPIO_Port, TOUCH_IRQ_Pin) == GPIO_PIN_RESET);
    }

    uint16_t Touch::touch_Read12bit(uint8_t cmd){
        uint8_t rx[3];
        uint8_t tx[3];

        tx[0] = cmd;
        tx[1] = 0x00; tx[2] = 0x00;

        HAL_GPIO_WritePin(TOUCH_CS_GPIO_Port, TOUCH_CS_Pin, GPIO_PIN_RESET);
        HAL_SPI_TransmitReceive(&TOUCH_SPI_HANDLE, tx, rx, 3, HAL_MAX_DELAY);
        HAL_GPIO_WritePin(TOUCH_CS_GPIO_Port, TOUCH_CS_Pin, GPIO_PIN_SET);

        uint16_t value = ((rx[1] << 8 | rx[2])) >> 3;
        value &= 0x0FFF;
        return value;
    }

    bool Touch::touch_ReadRaw(uint16_t* x, uint16_t* y){
        if(x == nullptr || y == nullptr) return false;
        if(!touch_Pressed()) return false;

        uint8_t Xcmd = 0b11010000;
        uint8_t Ycmd = 0b10010000;

        touch_Read12bit(Xcmd);
        touch_Read12bit(Ycmd);
        *x = touch_Read12bit(Xcmd);
        *y = touch_Read12bit(Ycmd);
        return true;
    }

    bool Touch::touch_ReadAverageRaw(uint16_t* x, uint16_t* y){
        if(x == nullptr || y == nullptr) return false;
        if(!touch_Pressed()) return false;

        const int samples = 5;
        uint32_t sumX = 0, sumY = 0;
        uint16_t valid = 0;

        for(int i = 0; i < samples; i++){
            uint16_t rx, ry;
            if(touch_ReadRaw(&rx, &ry)){
                valid++;
                sumX += rx;
                sumY += ry;
            }
            HAL_Delay(1);
        }

        if(valid == 0) return false;
        *x = (uint16_t)(sumX / valid);
        *y = (uint16_t)(sumY / valid);
        return true;
    }

    bool Touch::touch_Convert(uint16_t rawX, uint16_t rawY, uint16_t* x, uint16_t* y){
        if(x == nullptr || y == nullptr) return false;

        if(rawX < RAW_X_MIN) rawX = RAW_X_MIN;
        if(rawY < RAW_Y_MIN) rawY = RAW_Y_MIN;
        if(rawX > RAW_X_MAX) rawX = RAW_X_MAX;
        if(rawY > RAW_Y_MAX) rawY = RAW_Y_MAX;

        uint32_t py = (rawX - RAW_X_MIN) * LCD_H / (RAW_X_MAX - RAW_X_MIN);
        uint32_t px = (rawY - RAW_Y_MIN) * LCD_W / (RAW_Y_MAX - RAW_Y_MIN);

        if(px >= LCD_W) px = LCD_W - 1;
        if(py >= LCD_H) py = LCD_H - 1;

        *x = (uint16_t)px;
        *y = (uint16_t)py;
        return true;
    }

    char Touch::KeyPadConversion(uint16_t px, uint16_t py){
        char keys[4][3] = {
            {'1', '2', '3'},
            {'4', '5', '6'},
            {'7', '8', '9'},
            {' ', '0', ' '}
        };
        char labels[3] = {'B', 'C', 'E'};

        if(px < OPS_X){
            int c = (px - GAP) / (KEY_W + GAP);
            int r = (py - CONTENT_Y - GAP) / (KEY_H + GAP);
            return keys[r][c];
        }
        else{
            int option = (py - CONTENT_Y - GAP) / (OPS_BTN_H + GAP);
            return labels[option];
        }
    }

    void Touch::touchCalibration(){
        uint16_t raw_x = 0, raw_y = 0;
        touch_ReadRaw(&raw_x, &raw_y);
        printf("raw_x=%u raw_y=%u\r\n", raw_x, raw_y);
        uint16_t x = 0, y = 0;
        touch_Convert(raw_x, raw_y, &x, &y);
        printf("Cali x = %u, y = %u\r\n", x, y);
    }

    bool Touch::touchprocess(){
        uint16_t raw_x = 0, raw_y = 0;
        uint16_t px = 0, py = 0;
        if(touch_ReadAverageRaw(&raw_x, &raw_y)){
            touch_Convert(raw_x, raw_y, &px, &py);
            button = KeyPadConversion(px, py);
            return true;
        }
        return false;
    }

} // namespace TOUCH
