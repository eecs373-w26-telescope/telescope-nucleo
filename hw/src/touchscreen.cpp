#include <hw/inc/touchscreen.hpp>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <iostream>

static constexpr uint16_t LCD_W = 480;
static constexpr uint16_t LCD_H = 320;

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

static constexpr uint16_t M_BOX_X = 10;
static constexpr uint16_t M_BOX_Y = 10;
static constexpr uint16_t M_BOX_W = 50;
static constexpr uint16_t M_BOX_H = 40;

static constexpr uint16_t DSO_VIEW_BOX_W = 160;
static constexpr uint16_t DSO_VIEW_BOX_X = 360;
static constexpr uint16_t DSO_VIEW_BOX_H = 40;
static constexpr uint16_t DSO_VIEW_BOX_Y = 10;

static constexpr uint16_t DISPLAY_BOX_X = 70;
static constexpr uint16_t DISPLAY_BOX_Y = 10;
static constexpr uint16_t DISPLAY_BOX_W = OPS_X - DISPLAY_BOX_X;
static constexpr uint16_t DISPLAY_BOX_H = 40;

static constexpr uint16_t COLOR_RED      = 0xF800;
static constexpr uint16_t COLOR_BG       = 0x0000;
static constexpr uint16_t COLOR_BAR      = 0x1800;
static constexpr uint16_t COLOR_INPUT    = 0x9000;
static constexpr uint16_t COLOR_BTNBORDER = 0x5000;
static constexpr uint16_t COLOR_BTNBG    = 0x8000;
static constexpr uint16_t COLOR_BRIGHTRED = 0xB800;

static const uint8_t Numberbitmap[10][5] = {
    {0x3E, 0x51, 0x49, 0x45, 0x3E}, // 0
    {0x00, 0x42, 0x7F, 0x40, 0x00}, // 1
    {0x42, 0x61, 0x51, 0x49, 0x46}, // 2
    {0x21, 0x41, 0x45, 0x4B, 0x31}, // 3
    {0x18, 0x14, 0x12, 0x7F, 0x10}, // 4
    {0x27, 0x45, 0x45, 0x45, 0x39}, // 5
    {0x3C, 0x4A, 0x49, 0x49, 0x30}, // 6
    {0x01, 0x71, 0x09, 0x05, 0x03}, // 7
    {0x36, 0x49, 0x49, 0x49, 0x36}, // 8
    {0x06, 0x49, 0x49, 0x29, 0x1E}  // 9
};

static constexpr int MESSIER_IDS[] = {
    45, 44, 7, 31, 42, 6, 47, 41, 24, 39,
    25, 35, 22, 34, 23, 37, 33, 5, 8, 48,
    11, 4, 13, 50, 21, 36, 16, 46, 93, 3,
    15, 38, 92, 2, 62, 29, 10, 12, 81, 52,
    18, 67, 28, 55, 17, 19, 80, 103, 30, 14,
    83, 101, 102, 53, 54, 69, 9, 79, 78, 110,
    26, 40, 27, 107, 51, 70, 94, 32, 68, 56,
    106, 104, 71, 82, 49, 1, 20, 64, 75, 63,
    87, 77, 60, 57, 66, 43, 96, 85, 86, 74,
    84, 65, 105, 72, 100, 90, 88, 61, 95, 59,
    58, 89, 109, 99, 108, 73, 98, 91, 97, 76
};


namespace telescope{
    Touchscreen::Touchscreen(
                SPI_HandleTypeDef* hspi,
                GPIO_TypeDef* cs_port,
                uint16_t cs_pin,
                GPIO_TypeDef* rst_port,
                uint16_t rst_pin,
                GPIO_TypeDef* led_port,
                uint16_t led_pin,
                GPIO_TypeDef* dc_port,
                uint16_t dc_pin):
                hspi_(hspi),
                cs_port_(cs_port),
                cs_pin_(cs_pin),
                rst_port_(rst_port),
                rst_pin_(rst_pin),
                led_port_(led_port),
                led_pin_(led_pin),
                dc_port_(dc_port),
                dc_pin_(dc_pin){}

    void Touchscreen::led_on(){
        HAL_GPIO_WritePin(led_port_, led_pin_, GPIO_PIN_SET);
    }
    void Touchscreen::cs_low(){
        HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_RESET);
    }
    void Touchscreen::cs_high(){
        HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_SET);
    }
    void Touchscreen::write_command(uint8_t cmd){
        cs_low();
        HAL_GPIO_WritePin(dc_port_, dc_pin_, GPIO_PIN_RESET);
        HAL_SPI_Transmit(hspi_, &cmd, 1, HAL_MAX_DELAY);
        cs_high();
    }
    void Touchscreen::write_data(uint8_t* buff, size_t buff_size){
        cs_low();
        HAL_GPIO_WritePin(dc_port_, dc_pin_, GPIO_PIN_SET);
        HAL_SPI_Transmit(hspi_, buff, buff_size, HAL_MAX_DELAY);
        cs_high();
    }
    void Touchscreen::write_data_byte(uint8_t data){
        write_data(&data, 1);
    }
    void Touchscreen::write_data16(uint16_t data){
        uint8_t buf[2];
        buf[0] = data >> 8;
        buf[1] = data & 0xFF;
        write_data(buf, 2);
    }

    void Touchscreen::reset(){
        HAL_GPIO_WritePin(rst_port_, rst_pin_, GPIO_PIN_RESET);
        HAL_Delay(5);
        HAL_GPIO_WritePin(rst_port_, rst_pin_, GPIO_PIN_SET);
        HAL_Delay(5);
    }

    void Touchscreen::set_address_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1){
        write_command(0x2A);
        write_data16(x0);
        write_data16(x1);

        write_command(0x2B);
        write_data16(y0);
        write_data16(y1);

        write_command(0x2C);
    }

    void Touchscreen::display_on(){
        write_command(0x29);
    }

    void Touchscreen::display_off(){
        write_command(0x28);
    }

    void Touchscreen::init(){
        HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_SET);
        HAL_GPIO_WritePin(dc_port_, dc_pin_, GPIO_PIN_SET);
        HAL_GPIO_WritePin(rst_port_, rst_pin_, GPIO_PIN_SET);
        HAL_GPIO_WritePin(led_port_, led_pin_, GPIO_PIN_RESET);
        reset();

        write_command(0x11);
        HAL_Delay(120);

        write_command(0x36);
        write_data_byte(0xE8);

        write_command(0x3A);
        write_data_byte(0x55);

        write_command(0xF0);
        write_data_byte(0xC3);

        write_command(0xF0);
        write_data_byte(0x96);

        write_command(0xB4);
        write_data_byte(0x01);

        write_command(0xB7);
        write_data_byte(0xC6);

        write_command(0xC0);
        write_data_byte(0x80);
        write_data_byte(0x45);

        write_command(0xC1);
        write_data_byte(0x13);

        write_command(0xC2);
        write_data_byte(0xA7);

        write_command(0xC5);
        write_data_byte(0x0A);

        write_command(0xE8);
        write_data_byte(0x40);
        write_data_byte(0x8A);
        write_data_byte(0x00);
        write_data_byte(0x00);
        write_data_byte(0x29);
        write_data_byte(0x19);
        write_data_byte(0xA5);
        write_data_byte(0x33);

        write_command(0xE0);
        uint8_t gamma_pos[] = {
            0xF0, 0x06, 0x0B, 0x08, 0x07, 0x05, 0x2E, 0x33,
            0x47, 0x3A, 0x17, 0x16, 0x2E, 0x31
        };
        write_data(gamma_pos, sizeof(gamma_pos));

        write_command(0xE1);
        uint8_t gamma_neg[] = {
            0xF0, 0x09, 0x0D, 0x06, 0x04, 0x15, 0x2D, 0x43,
            0x46, 0x3B, 0x16, 0x16, 0x2D, 0x31
        };
        write_data(gamma_neg, sizeof(gamma_neg));

        write_command(0xF0);
        write_data_byte(0x3C);

        write_command(0xF0);
        write_data_byte(0x69);

        display_on();
    }

    void Touchscreen::draw_pixel(uint16_t x, uint16_t y, uint16_t color){
        if(x >= LCD_W || y >= LCD_H) return;
        set_address_window(x, y, x, y);
        write_data16(color);
    }

    void Touchscreen::fill_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color){
        if(x >= LCD_W || y >= LCD_H) return;
        if((x+w-1) >= LCD_W) w = LCD_W - x;
        if((y+h-1) >= LCD_H) h = LCD_H - y;

        set_address_window(x, y, x+w-1, y+h-1);

        uint8_t data[2048];
        for(int i = 0; i < 2048; i += 2){
            data[i]   = color >> 8;
            data[i+1] = color & 0xFF;
        }

        uint32_t total_pixels = w * h;
        HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(dc_port_, dc_pin_, GPIO_PIN_SET);

        while(total_pixels > 0){
            uint32_t chunk_pixels = (total_pixels > 1024) ? 1024 : total_pixels;
            HAL_SPI_Transmit(hspi_, data, chunk_pixels * 2, HAL_MAX_DELAY);
            total_pixels -= chunk_pixels;
        }

        HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_SET);
    }

    void Touchscreen::fill_screen(uint16_t color){
        fill_rect(0, 0, LCD_W, LCD_H, color);
    }

    void Touchscreen::draw_hline(uint16_t x, uint16_t y, uint16_t w, uint16_t color){
        fill_rect(x, y, w, 1, color);
    }
    void Touchscreen::draw_vline(uint16_t x, uint16_t y, uint16_t h, uint16_t color){
        fill_rect(x, y, 1, h, color);
    }
    void Touchscreen::draw_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color){
        draw_hline(x, y, w, color);
        draw_hline(x, y+h-1, w, color);
        draw_vline(x, y, h, color);
        draw_vline(x+w-1, y, h, color);
    }

    static bool get_bitmap(char c, uint8_t out[5]){
        switch(c){
            case '0': memcpy(out, Numberbitmap[0], 5); return true;
            case '1': memcpy(out, Numberbitmap[1], 5); return true;
            case '2': memcpy(out, Numberbitmap[2], 5); return true;
            case '3': memcpy(out, Numberbitmap[3], 5); return true;
            case '4': memcpy(out, Numberbitmap[4], 5); return true;
            case '5': memcpy(out, Numberbitmap[5], 5); return true;
            case '6': memcpy(out, Numberbitmap[6], 5); return true;
            case '7': memcpy(out, Numberbitmap[7], 5); return true;
            case '8': memcpy(out, Numberbitmap[8], 5); return true;
            case '9': memcpy(out, Numberbitmap[9], 5); return true;

            case 'M': { uint8_t t[5] = {0x7F, 0x04, 0x18, 0x04, 0x7F}; memcpy(out,t,5); return true; }
            case 'B': { uint8_t t[5] = {0x7F, 0x49, 0x49, 0x49, 0x36}; memcpy(out,t,5); return true; }
            case 'S': { uint8_t t[5] = {0x26, 0x49, 0x49, 0x49, 0x32}; memcpy(out,t,5); return true; }
            case 'C': { uint8_t t[5] = {0x3E, 0x41, 0x41, 0x41, 0x22}; memcpy(out,t,5); return true; }
            case 'L': { uint8_t t[5] = {0x7F, 0x40, 0x40, 0x40, 0x40}; memcpy(out,t,5); return true; }
            case 'R': { uint8_t t[5] = {0x7F, 0x09, 0x19, 0x29, 0x46}; memcpy(out,t,5); return true; }
            case 'E': { uint8_t t[5] = {0x7F, 0x49, 0x49, 0x49, 0x41}; memcpy(out,t,5); return true; }
            case 'N': { uint8_t t[5] = {0x7F, 0x02, 0x0C, 0x10, 0x7F}; memcpy(out,t,5); return true; }
            case 'T': { uint8_t t[5] = {0x01, 0x01, 0x7F, 0x01, 0x01}; memcpy(out,t,5); return true; }
            case 'A': { uint8_t t[5] = {0x7E, 0x09, 0x09, 0x09, 0x7E}; memcpy(out,t,5); return true; }
            case 'K': { uint8_t t[5] = {0x7F, 0x08, 0x14, 0x22, 0x41}; memcpy(out,t,5); return true; }
            case 'Y': { uint8_t t[5] = {0x07, 0x08, 0x70, 0x08, 0x07}; memcpy(out,t,5); return true; }
            case 'G': { uint8_t t[5] = {0x3E, 0x41, 0x49, 0x49, 0x7A}; memcpy(out,t,5); return true; }
            case 'H': { uint8_t t[5] = {0x7F, 0x08, 0x08, 0x08, 0x7F}; memcpy(out,t,5); return true; }
            case 'F': { uint8_t t[5] = {0x7F, 0x09, 0x09, 0x09, 0x01}; memcpy(out,t,5); return true; }
            case 'X': { uint8_t t[5] = {0x63, 0x14, 0x08, 0x14, 0x63}; memcpy(out,t,5); return true; }
            case 'I': { uint8_t t[5] = {0x00, 0x41, 0x7F, 0x41, 0x00}; memcpy(out,t,5); return true; }
            case 'D': { uint8_t t[5] = {0x7F, 0x41, 0x41, 0x22, 0x1C}; memcpy(out,t,5); return true; }
            case 'O': { uint8_t t[5] = {0x3E, 0x41, 0x41, 0x41, 0x3E}; memcpy(out,t,5); return true; }
            case '!': { uint8_t t[5] = {0x00, 0x00, 0x5F, 0x00, 0x00}; memcpy(out,t,5); return true; }
            case ':': { uint8_t t[5] = {0x00, 0x36, 0x36, 0x00, 0x00}; memcpy(out,t,5); return true; }

            default: return false;
        }
    }

    void Touchscreen::draw_char(uint16_t x, uint16_t y, char c, uint16_t font_color, uint16_t back_color, uint16_t scale){
        uint8_t bits[5];
        if(!get_bitmap(c, bits)) return;

        for(uint8_t col = 0; col < 5; col++){
            for(uint8_t row = 0; row < 7; row++){
                bool on = (bits[col] >> row) & 0b1;
                uint16_t color = on ? font_color : back_color;
                fill_rect(x + col*scale, y + row*scale, scale, scale, color);
            }
        }
        fill_rect(x + 5*scale, y, scale, 7 * scale, back_color);
    }

    void Touchscreen::draw_string(uint16_t x, uint16_t y, const char* str, uint16_t font_color, uint16_t back_color, uint8_t scale){
        while(*str){
            draw_char(x, y, *str, font_color, back_color, scale);
            x += 6 * scale;
            str++;
        }
    }

    void Touchscreen::draw_top_bar(){
        fill_rect(0, 0, LCD_W, TOP_BAR_H, COLOR_BAR);
        draw_string(M_BOX_X + 13, M_BOX_Y + 6, "M", COLOR_BRIGHTRED, COLOR_BAR, 4);

        fill_rect(DISPLAY_BOX_X, DISPLAY_BOX_Y, DISPLAY_BOX_W, DISPLAY_BOX_H, COLOR_BAR);
        draw_rect(DISPLAY_BOX_X, DISPLAY_BOX_Y, DISPLAY_BOX_W, DISPLAY_BOX_H, COLOR_BTNBORDER);

        view_change();
    }

    void Touchscreen::draw_number(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t fill_color, uint16_t border_color, const char* label, uint16_t txt_color, uint16_t txt_scale){
        fill_rect(x, y, w, h, fill_color);
        draw_rect(x, y, w, h, border_color);
        uint16_t text_w = strlen(label) * 6 * txt_scale;
        uint16_t text_h = 7 * txt_scale;

        uint16_t tx = x + (w - text_w) / 2;
        uint16_t ty = y + (h - text_h) / 2;

        draw_string(tx, ty, label, txt_color, fill_color, txt_scale);
    }

    void Touchscreen::draw_keypad(){
        static const char* keys[4][3] = {
            {"1", "2", "3"},
            {"4", "5", "6"},
            {"7", "8", "9"},
            {" ", "0", " "}
        };

        for(uint16_t r = 0; r < KEY_ROWS; r++){
            for(uint16_t c = 0; c < KEY_COLS; c++){
                uint16_t x = GAP + c * (KEY_W + GAP);
                uint16_t y = CONTENT_Y + GAP + r * (KEY_H + GAP);

                if(strcmp(keys[r][c], " ") == 0){
                    fill_rect(x, y, KEY_W, KEY_H, COLOR_BTNBG);
                    continue;
                }

                draw_number(x, y, KEY_W, KEY_H, COLOR_BTNBG, COLOR_BTNBORDER, keys[r][c], COLOR_BRIGHTRED, 4);
            }
        }
    }

    void Touchscreen::draw_op(){
        const char* labels[3] = {"BACK", "CLEAR", "ENTER"};
        uint16_t fills[3] = {COLOR_BTNBG, COLOR_BTNBG, COLOR_BTNBG};

        for(uint16_t i = 0; i < 3; i++){
            uint16_t x = OPS_X + GAP;
            uint16_t y = CONTENT_Y + GAP + i * (OPS_BTN_H + GAP);

            draw_number(x, y, OPS_BTN_W, OPS_BTN_H, fills[i], COLOR_BG, labels[i], COLOR_BRIGHTRED, 3);
        }
    }

    static constexpr std::size_t MESSIER_ID_COUNT =
        sizeof(MESSIER_IDS) / sizeof(MESSIER_IDS[0]);

    bool Touchscreen::messier_id_exists(int target_id){
        for(std::size_t i = 0; i < MESSIER_ID_COUNT; ++i){
            if(MESSIER_IDS[i] == target_id) return true;
        }
        return false;
    }

    void Touchscreen::input_display(const char* text){
        fill_rect(DISPLAY_BOX_X + 2, DISPLAY_BOX_Y + 2, DISPLAY_BOX_W - 4, DISPLAY_BOX_H - 4, COLOR_BAR);
        draw_rect(DISPLAY_BOX_X, DISPLAY_BOX_Y, DISPLAY_BOX_W, DISPLAY_BOX_H, COLOR_BTNBORDER);
        draw_string(DISPLAY_BOX_X + 8, DISPLAY_BOX_Y + 8, text, COLOR_BRIGHTRED, COLOR_BAR, 3);
    }

    char Touchscreen::update_display_string(char c){
        if(c == 'B'){
            if(length_ == 0) return 'B';
            length_--;
            display_[length_] = '\0';

            uint16_t x = DISPLAY_BOX_X + 8 + length_ * 6 * 3;
            uint16_t y = DISPLAY_BOX_Y + 8;
            fill_rect(x, y, 6*3, 7*3, COLOR_BAR);
            return 'B';
        }
        else if(c == 'C'){
            length_ = 0;
            display_[0] = '\0';
            fill_rect(DISPLAY_BOX_X + 2, DISPLAY_BOX_Y + 2, DISPLAY_BOX_W - 4, DISPLAY_BOX_H - 4, COLOR_BAR);
            return 'C';
        }
        else if(c == 'E'){
            enter_ = true;
            if(length_ == 0){
                error();
                return 'E';
            }
            if(!messier_id_exists(std::stoi(display_))){
                error();
            }
            else{
                gosearch();
            }
            return 'E';
        }
        else if(c == 'V'){
            view_ = !view_;
            return 'V';
        }
        else{
            if(c != ' '){
                display_[length_++] = c;
                display_[length_] = '\0';
                return 'N';
            }
        }
        return 'U';
    }

    void Touchscreen::display_new_character(char button){
        uint16_t x = DISPLAY_BOX_X + 8 + (length_ - 1) * 6 * 3;
        uint16_t y = DISPLAY_BOX_Y + 8;
        draw_char(x, y, button, COLOR_BRIGHTRED, COLOR_BAR, 3);
    }

    void Touchscreen::draw_main(){
        fill_screen(COLOR_BG);

        draw_top_bar();
        input_display("");
        draw_keypad();
        draw_op();
        HAL_GPIO_WritePin(led_port_, led_pin_, GPIO_PIN_SET);
        main_   = 1;
        enter_  = 0;
        search_ = 0;
        search_id_ = -1;
        length_ = 0;
        display_[0] = '\0';
    }

    void Touchscreen::draw_popup(const char* line1, const char* line2, const char* line3){
        const uint16_t POPUP_W = 300;
        const uint16_t POPUP_H = 140;

        const uint16_t popup_x = (LCD_W - POPUP_W) / 2;
        const uint16_t popup_y = (LCD_H - POPUP_H) / 2;

        fill_rect(popup_x, popup_y, POPUP_W, POPUP_H, COLOR_BG);
        draw_rect(popup_x, popup_y, POPUP_W, POPUP_H, COLOR_BRIGHTRED);
        draw_rect(popup_x + 2, popup_y + 2, POPUP_W - 4, POPUP_H - 4, COLOR_BRIGHTRED);

        draw_string(popup_x + 20, popup_y + 20, line1, COLOR_BRIGHTRED, COLOR_BG, 2);
        draw_string(popup_x + 20, popup_y + 50, line2, COLOR_BRIGHTRED, COLOR_BG, 2);
        draw_string(popup_x + 20, popup_y + 80, line3, COLOR_BRIGHTRED, COLOR_BG, 2);
    }

    void Touchscreen::popup(const char* line1, const char* line2, const char* line3){
        fill_screen(COLOR_BG);
        draw_string(120, 80,  line1, COLOR_BRIGHTRED, COLOR_BG, 5);
        draw_string(120, 160, line2, COLOR_BRIGHTRED, COLOR_BG, 3);
        draw_string(120, 240, line3, COLOR_BRIGHTRED, COLOR_BG, 3);
    }

    void Touchscreen::error(){
        popup("ERROR", "DSO NOT EXIST", "TRY AGAIN");
        HAL_Delay(500);
        draw_main();
    }

    void Touchscreen::gosearch(){
        search_id_ = std::stoi(display_);
        char output[8];
        snprintf(output, sizeof(output), "M%s", display_);
        enter_  = 0;
        search_ = 1;
        popup("SEARCHING:", output, "");
        draw_number(OPS_X + GAP, CONTENT_Y + GAP + 1 * (OPS_BTN_H + GAP), OPS_BTN_W, OPS_BTN_H, COLOR_BTNBG, COLOR_BG, "CANCEL", COLOR_BRIGHTRED, 3);
    }

    void Touchscreen::gocancel(){
        search_    = 0;
        display_[0] = '\0';
        length_    = 0;
        main_      = 1;
        draw_main();
    }

    void Touchscreen::view_change(){
        fill_rect(OPS_X + GAP, DSO_VIEW_BOX_Y, OPS_BTN_W, DSO_VIEW_BOX_H, COLOR_BTNBG);
        if(view_){
            draw_string(OPS_X + GAP + 15, DSO_VIEW_BOX_Y + 10, "ON ", COLOR_BRIGHTRED, COLOR_BTNBG, 3);
        }
        else{
            draw_string(OPS_X + GAP + 15, DSO_VIEW_BOX_Y + 10, "OFF", COLOR_BRIGHTRED, COLOR_BTNBG, 3);
        }
    }

    bool Touchscreen::get_search_status(){
        return search_;
    }
    bool Touchscreen::get_main(){
        return main_;
    }
    bool Touchscreen::get_view_status(){
        return view_;
    }

    void Touchscreen::normal_process(char action, char button){
        if(length_ >= 4){
            error();
        }
        if(action == 'N'){
            display_new_character(button);
        }
    }

} // namespace telescope
