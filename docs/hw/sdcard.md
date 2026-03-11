# SD Card over SDMMC + DMA Implementation Description

## Usage

## Configuration

### System Clock
For SDMMC + DMA, we need the system clock to run at a higher frequency (we chose 80MHz), not the default 4MHz. For this, we need HSE enabled, and SYSCLK sourced from PLL. 

In CubeMX:
* System Core --> RCC --> select HSE

In Clock Configuration: 
* Set System Clock Mux to PLLCLK
* Set PLL Source Mux to HSE
* Set PLLM1 to /1, *N to *20, and /R to /2 to reach a SYSCLK of 80MHz

### Enable SWD
In CubeMX:
* System Core --> SYS --> set Debug = Serial Wire

### Enable SDMMC1
In CubeMX:
* Connectivity --> enable SDMMC1
* Set 4-bit wiring 

### Enable DMA
In CubeMX:
* SDMMC1 configuration --> DMA settings --> enable RX DMA and TX DMA
* Enable related interrupts 

### Enable FATFS
FATFS is 

In CubeMX:
* Middleware --> FATFS --> select SD driver 