# UART Communication Protocol

USART1 between STM32F405 Feather and Raspberry Pi at 115200 baud (8N1).

- **Feather pins**: PB7 (RX), PB6 (TX)
- **Raspi pins**: GPIO 14 - pin 8 (TXD), GPIO 15 - pin 10 (RXD)
- **DMA**: TX normal mode, RX circular mode (both sides)

All packets flow **Nucleo -> Raspi** only. The Raspi is a display-only consumer.

## Frame Format

```
Offset  Size  Field
0       1     SYNC_HI   (0xAB)
1       1     SYNC_LO   (0xCD)
2       1     PACKET_ID
3       1     LENGTH    (payload size in bytes, 0-128)
4       N     PAYLOAD
4+N     1     CRC_LO
5+N     1     CRC_HI
```

- CRC-16/CCITT-FALSE: polynomial 0x1021, init 0xFFFF
- CRC computed over bytes 0 through 3+N (sync + header + payload)
- CRC transmitted little-endian
- Min frame: 6 bytes (empty payload), max frame: 134 bytes

## Packet ID Assignments

| ID   | Name              | Payload | Rate        |
|------|-------------------|---------|-------------|
| 0x01 | PACKET_GPS        | 16B     | 1 Hz        |
| 0x02 | PACKET_ENCODER    | 4B      | 10 Hz       |
| 0x03 | PACKET_IMU        | 3B      | 10 Hz       |
| 0x04 | PACKET_STATE_SYNC | 4B      | On change   |
| 0xFF | PACKET_DEBUG      | 16B     | Ad hoc      |

## Payload Definitions

All structs use `#pragma pack(push, 1)`.

### GpsPayload (0x01, 16 bytes)

Adafruit PA1616S GPS via UART6. Position and time from RMC sentence, satellite count from GGA.

```
Offset  Type      Field           Notes
0       int32_t   latitude_e7     degrees * 10^7, negative = South
4       int32_t   longitude_e7    degrees * 10^7, negative = West
8       uint8_t   num_satellites  from GGA sentence
9       uint8_t   utc_hour        0-23
10      uint8_t   utc_minute      0-59
11      uint8_t   utc_second      0-59
12      uint8_t   utc_day         1-31
13      uint8_t   utc_month       1-12
14      uint16_t  utc_year        e.g. 2025
```

### EncoderPayload (0x02, 4 bytes)

Two AS5048A magnetic encoders via SPI1. Yaw CS: PC2 (D12), Pitch CS: PC1 (D13).
Values are offset-corrected and moving-average filtered (window=8, circular).

```
Offset  Type      Field      Notes
0       uint16_t  yaw_raw    14-bit (0-16383 = 0-360 deg)
2       uint16_t  pitch_raw  14-bit (0-16383 = 0-360 deg)
```

### ImuPayload (0x03, 3 bytes)

BNO055 9DoF IMU via I2C1 in NDOF fusion mode. Heading from Euler register 0x1A.

```
Offset  Type     Field         Notes
0       int16_t  heading       degrees * 16 (0 to 5759 = 0.0 to 359.9375 deg)
2       uint8_t  calibration   CALIB_STAT register (0x35)
                               bits [7:6]=sys, [5:4]=gyro, [3:2]=accel, [1:0]=mag
                               each field 0-3, 3 = fully calibrated
```

### StateSyncPayload (0x04, 4 bytes)

Sent by Nucleo on every state transition. Raspi mirrors this as its authoritative state.

```
Offset  Type      Field      Notes
0       uint8_t   state      TelescopeState: INIT=0, SETUP=1, READY=2, SEARCH=3, FOUND=4
1       uint8_t   flags      reserved bitfield
2       uint16_t  sequence   monotonic counter for gap detection
```

### DebugPayload (0xFF, 16 bytes)

```
Offset  Type      Field    Notes
0       uint8_t   data[16] arbitrary debug string or binary data
```

## Bandwidth Budget

115200 baud = 11,520 bytes/sec (8N1).

| Stream          | Frame size | Rate   | Bytes/sec |
|-----------------|------------|--------|-----------|
| GPS             | 22B        | 1 Hz   | 22        |
| Encoder         | 10B        | 10 Hz  | 100       |
| IMU             | 9B         | 10 Hz  | 90        |
| State sync      | 10B        | <1 Hz  | <10       |
| **Total**       |            |        | **~222**  |
