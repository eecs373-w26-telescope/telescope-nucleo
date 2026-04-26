# UART Communication Protocol Implementation Description
The communication protocol defines the structured exchange of data between the Nucleo controller and the Raspberry Pi HUD. It is designed to be lightweight, robust against noise, and efficient for high-frequency telemetry updates.

## Hardware Interface
Physical communication occurs over USART1 at 115200 baud (8N1). The implementation utilizes DMA to ensure that high-frequency updates for encoders and IMU do not block the main processing loop.

## Software Architecture
The protocol is implemented as a frame-based system with a fixed-size header and variable-size payload.

**Frame Structure:**
* **Sync Word (2 bytes)**: 0xABCD, used to align the decoder.
* **Packet ID (1 byte)**: Identifies the type of data being sent.
* **Length (1 byte)**: Number of bytes in the payload (0-128).
* **Payload (N bytes)**: The actual data, packed with 1-byte alignment.
* **CRC-16 (2 bytes)**: CCITT-FALSE checksum for error detection.

## Communication Pipeline
* **Packing**: Structs are packed using `#pragma pack(push, 1)` to ensure binary compatibility between the Nucleo (ARM) and RasPi.
* **Checksumming**: A 16-bit CRC is calculated over the entire header and payload.
* **Transmission**: Frames are sent via DMA; the system waits for the previous transfer to complete before starting a new one.
* **Reception**: The receiver hunts for the sync word, parses the header, and validates the checksum before dispatching the data to the application.

## Packet Definitions
* **0x01 GPS**: Latitude, longitude, satellite count, and UTC time.
* **0x02 Encoder**: 14-bit raw yaw and pitch angles.
* **0x03 IMU**: Filtered heading and 9DoF calibration status.
* **0x04 State Sync**: Current system state (IDLE/SEARCH/FOUND) and UI flags.
* **0x05 DSO Target**: Details of the selected search target (ID, RA/Dec, Magnitude).
* **0x06 FOV Objects**: List of up to 21 objects currently visible in the eyepiece.
* **0x08 Search Guidance**: Unit vectors and angular distance to the active target.
* **0x09 Pointing**: High-precision altitude and azimuth coordinates.

## Features Implemented
* Little-endian binary serialization
* CRC-16/CCITT error validation
* Multi-packet telemetry stream (1Hz to 10Hz)
* Bidirectional state and time synchronization
* Monotonic sequence tracking for gap detection
