# Adafruit STM32F405 Feather Express - Pinout Reference

All logic is 3.3V. Nearly all pins are 5V compliant. All pins can be interrupt inputs.

## Signal Pins

| Pin Label | GPIO # | STM32 Pin | Analog | DAC | UART | SPI | I2C | CAN | I2S | PWM | Notes |
|-----------|--------|-----------|--------|-----|------|-----|-----|-----|-----|-----|-------|
| A0 | 16 | PA4 | ADC12_IN4 | DAC1 | | | | | | - | True analog output |
| A1 | 17 | PA5 | ADC12_IN5 | DAC2 | | SPI1 SCK | | | | - | True analog output |
| A2 | 18 | PA6 | ADC12_IN6 | | | SPI1 MISO | | | | TIM3_CH1 | |
| A3 | 19 | PA7 | ADC12_IN7 | | | SPI1 MOSI | | | | TIM3_CH2 | |
| A4 | 20 | PC4 | ADC12_IN14 | | | | | | | - | |
| A5 | 21 | PC5 | ADC12_IN15 | | | | | | | - | |
| SCK | 23 | PB13 | | | | SPI2 SCK | | CAN2 TX | I2S2 CLK | TIM1_CH1N | Primary SPI clock |
| MOSI (MO) | 25 | PB15 | | | | SPI2 MOSI | | | I2S2 SD | TIM1_CH3N | Primary SPI out |
| MISO (MI) | 24 | PB14 | | | | SPI2 MISO | | | I2S2ext SD | TIM1_CH2N | Primary SPI in |
| RX | 0 | PB11 | | | USART3 RX | | I2C2 SDA | | | TIM2_CH4 | Default serial RX |
| TX | 1 | PB10 | | | USART3 TX | | I2C2 SCL | | | TIM2_CH3 | Default serial TX |
| SDA | 14 | PB7 | | | USART1 RX | | I2C1 SDA | | | TIM4_CH2 | 10K pullup to 3.3V |
| SCL | 15 | PB6 | | | USART1 TX | | I2C1 SCL | CAN2 TX | | TIM4_CH1 | 10K pullup to 3.3V |
| D5 | 5 | PC7 | | | USART6 RX | | | | I2S3 MCK | TIM3_CH2 | |
| D6 | 6 | PC6 | | | USART6 TX | | | | I2S2 MCK | TIM3_CH1 | |
| D9 | 9 | PB8 | | | | | I2C1 SCL | CAN1 RX | | TIM4_CH3 | |
| D10 | 10 | PB9 | | | | | I2C1 SDA | CAN1 TX | I2S2 WS | TIM4_CH4 | |
| D11 | 11 | PC3 | | | | SPI2 MOSI | | | I2S2 SD | - | |
| D12 | 12 | PC2 | | | | SPI2 MISO | | | I2S2ext SD | - | |
| D13 | 13 | PC1 | | | | | | | | - | Red LED |

## Power Pins

| Pin Label | Description |
|-----------|-------------|
| RST | Reset |
| 3.3V | 3.3V regulator output (500mA peak) |
| GND | Common ground |
| BAT | LiPo battery voltage (JST connector) |
| EN | 3.3V regulator enable (pull low to disable) |
| USB | USB C positive voltage |


## Pin Usage Tracker

- [x] **A0** (PA4) - SPI CS Touchscreen Display
- [x] **A1** (PA5) - SPI1_SCK
- [x] **A2** (PA6) - SPI1_MISO
- [x] **A3** (PA7) - SPI1_MOSI
- [x] **A4** (PC4) - GPIO - debug button input
- [x] **A5** (PC5) - GPIO - Touchscreen RST
- [x] **SCK** (PB13) - GPIO - Touchscreen DC
- [x] **MOSI** (PB15) - GPIO - Touchscreen LED
- [x] **MISO** (PB14) - GPIO - Touchscreen IRQ (Touch)
- [x] **RX** (PB11) - Serial Out
- [x] **TX** (PB10) - Serial Out
- [x] **SDA** (PB7) - USART1_RX - RPI TX
- [x] **SCL** (PB6) - USART1_TX - RPI RX
- [x] **D5** (PC7) - USART6_RX - GPS TX
- [x] **D6** (PC6) - USART6_TX - GPS RX
- [x] **D9** (PB8) - I2C1_SCL
- [x] **D10** (PB9) - I2C1_SDA
- [x] **D11** (PC3) - GPIO - SPI CS Touchscreen Touch
- [x] **D12** (PC2) - GPIO - SPI CS Encoder Yaw
- [x] **D13** (PC1) - GPIO - SPI CS Encoder Pitch

CHIP SELECT ARRAY HIGH TO LOW:
- Touchscreen Display
- Touchscreen Touch
- Encoder Yaw
- Encoder Pitch
