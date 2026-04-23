# Absolute Magnetic Rotary Encoders over SPI Implementation Description
The rotary encoder module provides absolute angular position sensing for the telescope axes. It reads 14-bit angular data from magnetic encoders (AS5048) over SPI and converts raw counts into calibrated physical angles (degrees).

## Hardware Interface
The encoder communicates over SPI using MOSI, MISO, and SCK, with a dedicated CHIP SELECT (CS) line per encoder. The encoder outputs 14-bit angular position data per read.

There is one encoder mounted on the rotating yaw shift, and another mounted on the rotating pitch shaft, each using custom 3D printed mounts. 

## Software Architecture
The encoder system is implemented as a hardware abstraction layer (encoder.cpp) that wraps STM32 HAL SPI APIs.

Core components:

* SPI Communication Layer: Handles 16-bit SPI transactions with manual CS control and simultaneous transmit/receive.
* Protocol Handling: Builds valid encoder frames including:
    * Read commands (with address + parity)
    * Dummy frames for data retrieval
    * Even parity generation for 15-bit payloads
* Data Processing Layer: Extracts valid 14-bit angle data, handles error flags, and applies:
    * Optional inversion (axis direction correction)
    * Offset compensation (mechanical alignment calibration)

## Encoder Data Processing Pipeline
* Send read command to encoder register
* Send dummy frame to receive response
* Validate error flag in returned frame
* Extract 14-bit raw angle
* Apply inversion and offset correction
* Convert to degrees (0–360°)

## Features Implemented
* High-resolution (14-bit) absolute angle sensing
* Parity-protected SPI communication
* Error detection and clearing
* Configurable inversion and zero-offset calibration
* Conversion to physical angle units (degrees)