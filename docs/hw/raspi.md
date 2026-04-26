# Raspberry Pi Interface Implementation Description
The Raspberry Pi interface provides a high-speed telemetry link to the external HUD. It handles the packaging and transmission of sensor data, system states, and astronomical object positions using a robust packet-based protocol.

## Hardware Interface
The link uses USART1 at 115200 baud. To minimize CPU overhead, the interface uses DMA for both transmission (normal mode) and reception (circular mode with IDLE line detection). 

The Nucleo connects via PB6 (TX) and PB7 (RX) to the Raspberry Pi's GPIO 14 and 15.

## Software Architecture
The interface is managed by the `RasPi` class in `raspi.cpp`, which acts as a hardware abstraction layer for the UART protocol.

Core Components:
* **Packet Encoder**: Wraps telemetry data into frames with sync words and CRC-16 checksums.
* **State Machine Decoder**: Processes the incoming byte stream to reconstruct packets and verify integrity.
* **Dispatcher**: Forwards valid incoming data (like external time sync) to the relevant system modules.
* **DMA Manager**: Handles non-blocking transfers and provides callbacks for transmission completion.

## Communication Pipeline
* Receive incoming bytes via DMA circular buffer
* Reconstruct frames using sync word hunting
* Validate packets using CRC-16/CCITT
* Extract and dispatch payload data
* Periodically transmit telemetry:
    * GPS coordinates and time
    * IMU heading and calibration
    * Encoder raw angles
    * Current system state and guidance
    * Visible FOV object list

## Features Implemented
* DMA-backed non-blocking UART communication
* CRC-16 protected packet protocol
* Circular buffer reception with idle-line detection
* Bi-directional time and state synchronization
* Heartbeat monitoring for connection health
