# Style Guide

## Naming
- Variables and functions: `snake_case`
- State machine states: `PascalCase`
- Abbreviations: all caps everywhere, including namespaces (`GPS`, `IMU`, `SPI`)
- Private member variables: trailing underscore (`hspi_`, `cs_pin_`)

## Peripheral Structure
Each peripheral is a class in its own lowercase namespace. Constructor initializes private members only; `init()` handles HAL setup, DMA, and calibration.

Registers go in a struct outside the class but within the same namespace (see `encoder.hpp`).

## Files
- `hw/inc/<peripheral>.hpp`
- `hw/src/<peripheral>.cpp`
