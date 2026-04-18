#pragma once
#include "main.h"
#include <cstdint>
#include "stdlib.h"

namespace telescope {
    class Touchscreen{
        public:
            Touchscreen(SPI_HandleTypeDef* hspi, GPIO_TypeDef* cs_port, uint16_t cs_pin,
                GPIO_TypeDef* rst_port, uint16_t rst_pin,
                GPIO_TypeDef* led_port, uint16_t led_pin,
                GPIO_TypeDef* dc_port, uint16_t dc_pin);

            void init();
            void reset();
            void display_on();
            void display_off();
            void led_on();

            void draw_main();
            void draw_top_bar();
            void draw_keypad();
            void draw_number(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t fill_color, uint16_t border_color, const char* label, uint16_t txt_color, uint16_t txt_scale);
            void draw_op();

            void input_display(const char* text);
            char update_display_string(char c);
            void display_new_character(char button);
            void draw_popup(const char* line1, const char* line2, const char* line3);
            void popup(const char* line1, const char* line2, const char* line3);
            void error();
            void gosearch();
            void gocancel();
            void view_change();
            void normal_process(char action, char button);

            void tick(uint32_t now);
            bool get_search_status(); // returns whether in searching mode
            bool get_main(); // returns whether touchscreen is on the main screen
            bool get_view_status(); // returns whether general DSO identification is on/off
            int get_selected_messier_id();
            const char* get_display();
            void tare_feedback();

        private:
            SPI_HandleTypeDef* hspi_ = nullptr;
            GPIO_TypeDef* cs_port_ = nullptr;
            uint16_t cs_pin_;

            GPIO_TypeDef* rst_port_ = nullptr;
            uint16_t rst_pin_;

            GPIO_TypeDef* led_port_ = nullptr;
            uint16_t led_pin_;

            GPIO_TypeDef* dc_port_ = nullptr;
            uint16_t dc_pin_;

            bool enter_ = false;
            char display_[7];
            int length_ = 0;
            bool view_ = false;
            int search_id_ = -1;
            bool search_ = false;
            bool main_ = false;
            bool error_pending_ = false;
            uint32_t error_until_tick_ = 0;
            bool tare_feedback_pending_ = false;
            uint32_t tare_feedback_until_tick_ = 0;

            void cs_low();
            void cs_high();

            void write_command(uint8_t cmd);
            void write_data(uint8_t* buff, size_t buff_size);
            void write_data_byte(uint8_t data);
            void write_data16(uint16_t data);

            void set_address_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);
            void draw_pixel(uint16_t x, uint16_t y, uint16_t color);
            void fill_screen(uint16_t color);
            void fill_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
            void draw_hline(uint16_t x, uint16_t y, uint16_t w, uint16_t color);
            void draw_vline(uint16_t x, uint16_t y, uint16_t h, uint16_t color);
            void draw_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
            void draw_char(uint16_t x, uint16_t y, char c, uint16_t font_color, uint16_t back_color, uint16_t scale);
            void draw_string(uint16_t x, uint16_t y, const char* str, uint16_t font_color, uint16_t back_color, uint8_t scale);
            bool messier_id_exists(int target_id);
    };
}
