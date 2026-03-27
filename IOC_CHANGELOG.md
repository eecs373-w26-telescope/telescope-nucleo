# IOC Changelog

CubeMX configuration changes for `stm32l4/stm32l4.ioc` in case we need to regenerate and replicate



## 2026-03-26: LPUART1 + DMA for Raspi Communication - Kevin

### Connectivity > LPUART1

- Mode: Asynchronous
- Baud Rate: 115200
- Word Length: 8 bits
- Parity: None
- Stop Bits: 1
- Hardware Flow Control: None
- Data Direction: TX and RX
- One Bit Sampling: Disabled
- Clock Prescaler: DIV1
- FIFO Mode: Disabled
- TX FIFO Threshold: 1/8
- RX FIFO Threshold: 1/8

### Pin Assignment

- PC0: LPUART1_RX
- PC1: LPUART1_TX

### DMA

- LPUART1_TX: DMA1 Channel 1, Memory to Peripheral, Normal mode, Byte/Byte, Peripheral increment disabled, Memory increment enabled
- LPUART1_RX: DMA1 Channel 2, Peripheral to Memory, Circular mode, Byte/Byte, Peripheral increment disabled, Memory increment enabled

### NVIC

- LPUART1 global interrupt: Enabled (priority 0/0)
- DMA1 Channel 1 global interrupt: Enabled (priority 0/0)
- DMA1 Channel 2 global interrupt: Enabled (priority 0/0)
