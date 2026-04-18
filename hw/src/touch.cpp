#include <hw/inc/touch.hpp>
#include <hw/inc/display.hpp>
#include <hw/inc/spi_mode.hpp>
#include <cstdio>
#include <cstdint>
#include <cstring>


namespace telescope{
    Touch::Touch(SPI_HandleTypeDef* hspi, GPIO_TypeDef* cs_port, uint16_t cs_pin,
                 GPIO_TypeDef* irq_port, uint16_t irq_pin):
        hspi_(hspi), cs_port_(cs_port), cs_pin_(cs_pin),
        irq_port_(irq_port), irq_pin_(irq_pin){}

    void Touch::cs_low(){
        HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_RESET);
    }
    void Touch::cs_high(){
        HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_SET);
    }

    void Touch::init(){
        HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_SET);
    }

    bool Touch::is_pressed(){
        return (HAL_GPIO_ReadPin(irq_port_, irq_pin_) == GPIO_PIN_RESET);
    }

    uint16_t Touch::read_12_bit(uint8_t cmd){
        uint8_t rx[3];
        uint8_t tx[3] = {cmd, 0x00, 0x00};

        spi_mode::set_mode0(hspi_);
        cs_low();
        HAL_SPI_TransmitReceive(hspi_, tx, rx, 3, HAL_MAX_DELAY);
        cs_high();
        spi_mode::set_mode1(hspi_);

        uint16_t value = ((rx[1] << 8) | rx[2]) >> 3;
        return value & 0x0FFF;
    }

    bool Touch::read_raw(uint16_t* x, uint16_t* y){
        if(x == nullptr || y == nullptr) return false;

        read_12_bit(touch_reg::READ_X);
        read_12_bit(touch_reg::READ_Y);
        *x = read_12_bit(touch_reg::READ_X);
        *y = read_12_bit(touch_reg::READ_Y);
        return true;
    }

    bool Touch::read_average_raw(uint16_t* x, uint16_t* y){
        if(x == nullptr || y == nullptr) return false;

        const int samples = 5;
        uint32_t sum_x = 0, sum_y = 0;
        uint16_t valid = 0;

        for(int i = 0; i < samples; i++){
            uint16_t rx, ry;
            if(read_raw(&rx, &ry)){
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

    bool Touch::convert_px(uint16_t raw_x, uint16_t raw_y, uint16_t* x, uint16_t* y){
        if(x == nullptr || y == nullptr) return false;

        if(raw_x < touch_calibration::raw_x_min) raw_x = touch_calibration::raw_x_min;
        if(raw_y < touch_calibration::raw_y_min) raw_y = touch_calibration::raw_y_min;
        if(raw_x > touch_calibration::raw_x_max) raw_x = touch_calibration::raw_x_max;
        if(raw_y > touch_calibration::raw_y_max) raw_y = touch_calibration::raw_y_max;

        uint32_t py = (raw_x - touch_calibration::raw_x_min) * LCD_H /
                      (touch_calibration::raw_x_max - touch_calibration::raw_x_min);
        uint32_t px = (raw_y - touch_calibration::raw_y_min) * LCD_W /
                      (touch_calibration::raw_y_max - touch_calibration::raw_y_min);

        if(px >= LCD_W) px = LCD_W - 1;
        if(py >= LCD_H) py = LCD_H - 1;

        *x = (uint16_t)px;
        *y = (uint16_t)py;
        return true;
    }

    char Touch::keypad_conversion(uint16_t px, uint16_t py){
        static const char keys[4][3] = {
            {'1', '2', '3'},
            {'4', '5', '6'},
            {'7', '8', '9'},
            {' ', '0', ' '}
        };

        //determines type 
        if(px >= M_BOX_X && px < M_BOX_X + M_BTN_W && py >= M_BOX_Y && py < M_BOX_Y + CAT_BTN_H){
            return 'M';
        }
        if(px >= NGC_BOX_X && px < NGC_BOX_X + NGC_BTN_W && py >= M_BOX_Y && py < M_BOX_Y + CAT_BTN_H){
            return 'G';
        }
        for(int r = 0; r < KEY_ROWS; r++){
            for(int c = 0; c < KEY_COLS; c++){
                uint16_t x = GAP + c * (KEY_W + GAP);
                uint16_t y = CONTENT_Y + GAP + r * (KEY_H + GAP);
                if(px >= x && px < x + KEY_W && py >= y && py < y + KEY_H){
                    return keys[r][c];
                }
            }
        }

        if(px >= OPS_X + GAP && px < OPS_X + GAP + OPS_BTN_W &&
           py >= DSO_VIEW_BOX_Y && py < DSO_VIEW_BOX_Y + DSO_VIEW_BOX_H){
            return 'V';
        }

        for(int i = 0; i < 3; i++){
            uint16_t x = OPS_X + GAP;
            uint16_t y = CONTENT_Y + GAP + i * (OPS_BTN_H + GAP);
            if(px >= x && px < x + OPS_BTN_W && py >= y && py < y + OPS_BTN_H){
                return (i == 0) ? 'B' : (i == 1) ? 'C' : 'E';
            }
        }

        return '\0';
    }

    void Touch::calibration(){
        uint16_t raw_x = 0, raw_y = 0;
        read_raw(&raw_x, &raw_y);
        printf("raw_x=%u raw_y=%u\r\n", raw_x, raw_y);
        uint16_t x = 0, y = 0;
        convert_px(raw_x, raw_y, &x, &y);
        printf("Cali x = %u, y = %u\r\n", x, y);
    }

    bool Touch::process(){
        uint16_t raw_x = 0, raw_y = 0;
        uint16_t px = 0, py = 0;
        if(read_average_raw(&raw_x, &raw_y)){
            convert_px(raw_x, raw_y, &px, &py);
            char c = keypad_conversion(px, py);
            if(c == '\0') return false;
            button_ = c;
            return true;
        }
        return false;
    }

    char Touch::get_button(){
        return button_;
    }

} // namespace telescope
