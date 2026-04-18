#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cstdlib>
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

// static constexpr uint16_t M_BOX_X = 10;
// static constexpr uint16_t M_BOX_Y = 10;
// static constexpr uint16_t M_BOX_W = 50;
// static constexpr uint16_t M_BOX_H = 40;

static constexpr uint16_t CAT_BOX_X = 10;
static constexpr uint16_t CAT_BOX_Y = 10;
static constexpr uint16_t CAT_BTN_H = 40;
static constexpr uint16_t M_BTN_W   = 50;
static constexpr uint16_t NGC_BTN_W = 80;
static constexpr uint16_t CAT_BTN_GAP = 10;

static constexpr uint16_t M_BOX_X   = CAT_BOX_X;
static constexpr uint16_t M_BOX_Y = CAT_BOX_Y;
static constexpr uint16_t M_BOX_W = M_BTN_W;
static constexpr uint16_t M_BOX_H = CAT_BTN_H;
static constexpr uint16_t NGC_BOX_X = M_BOX_X + M_BTN_W + CAT_BTN_GAP;

static constexpr uint16_t DISPLAY_BOX_X = NGC_BOX_X + NGC_BTN_W + 10;

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