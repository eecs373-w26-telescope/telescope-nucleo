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

    uint16_t Touch::touch_read_12bit(uint8_t cmd){
        uint8_t rx[3];
        uint8_t tx[3] = {cmd, 0x00, 0x00};

        HAL_GPIO_WritePin(TOUCH_CS_GPIO_Port, TOUCH_CS_Pin, GPIO_PIN_RESET);
        HAL_SPI_TransmitReceive(&TOUCH_SPI_HANDLE, tx, rx, 3, HAL_MAX_DELAY);
        HAL_GPIO_WritePin(TOUCH_CS_GPIO_Port, TOUCH_CS_Pin, GPIO_PIN_SET);

        uint16_t value = ((rx[1] << 8) | rx[2]) >> 3;
        return value & 0x0FFF;
    }

    bool Touch::touch_read_raw(uint16_t* x, uint16_t* y){
        if(x == nullptr || y == nullptr) return false;
        if(!touch_pressed()) return false;

        uint8_t x_cmd = 0b11010000;
        uint8_t y_cmd = 0b10010000;

        touch_read_12bit(x_cmd);
        touch_read_12bit(y_cmd);
        *x = touch_read_12bit(x_cmd);
        *y = touch_read_12bit(y_cmd);
        return true;
    }

    bool Touch::touch_read_average_raw(uint16_t* x, uint16_t* y){
        if(x == nullptr || y == nullptr) return false;
        if(!touch_pressed()) return false;

        const int samples = 5;
        uint32_t sum_x = 0, sum_y = 0;
        uint16_t valid = 0;

        for(int i = 0; i < samples; i++){
            uint16_t rx, ry;
            if(touch_read_raw(&rx, &ry)){
                valid++;
                sum_x += rx;
                sum_y += ry;
            }
            HAL_Delay(1);
        }

        if(valid == 0) return false;
        *x = (uint16_t)(sum_x / valid);
        *y = (uint16_t)(sum_y / valid);
        return true;
    }

    bool Touch::touch_convert(uint16_t raw_x, uint16_t raw_y, uint16_t* x, uint16_t* y){
        if(x == nullptr || y == nullptr) return false;

        if(raw_x < RAW_X_MIN) raw_x = RAW_X_MIN;
        if(raw_y < RAW_Y_MIN) raw_y = RAW_Y_MIN;
        if(raw_x > RAW_X_MAX) raw_x = RAW_X_MAX;
        if(raw_y > RAW_Y_MAX) raw_y = RAW_Y_MAX;

        uint32_t py = (raw_x - RAW_X_MIN) * LCD_H / (RAW_X_MAX - RAW_X_MIN);
        uint32_t px = (raw_y - RAW_Y_MIN) * LCD_W / (RAW_Y_MAX - RAW_Y_MIN);

        if(px >= LCD_W) px = LCD_W - 1;
        if(py >= LCD_H) py = LCD_H - 1;

        *x = (uint16_t)px;
        *y = (uint16_t)py;
        return true;
    }

    char Touch::keypad_conversion(uint16_t px, uint16_t py){
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

    void Touch::touch_calibration(){
        uint16_t raw_x = 0, raw_y = 0;
        touch_read_raw(&raw_x, &raw_y);
        printf("raw_x=%u raw_y=%u\r\n", raw_x, raw_y);
        uint16_t x = 0, y = 0;
        touch_convert(raw_x, raw_y, &x, &y);
        printf("Cali x = %u, y = %u\r\n", x, y);
    }

    bool Touch::touch_process(){
        uint16_t raw_x = 0, raw_y = 0;
        uint16_t px = 0, py = 0;
        if(touch_read_average_raw(&raw_x, &raw_y)){
            touch_convert(raw_x, raw_y, &px, &py);
            button = keypad_conversion(px, py);
            return true;
        }
        return false;
    }

} // namespace touch
