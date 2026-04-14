#pragma once

#include "main.h"
#include <hw/inc/protocol.h>
#include <cstdint>

namespace raspi {

void init(UART_HandleTypeDef* huart);

// Call from main loop to consume DMA RX bytes and dispatch packets
void process();

// Frame and transmit a packet via DMA. Returns false if TX is busy.
bool send_packet(uint8_t packet_id, const uint8_t* payload, uint8_t length);

// Convenience wrappers for typed payloads
bool send_gps(const GpsPayload& payload);
bool send_encoder(const EncoderPayload& payload);
bool send_imu(const ImuPayload& payload);
bool send_state_sync(const StateSyncPayload& payload);
bool send_dso_target(const DsoTargetPayload& payload);
bool send_debug(const DebugPayload& payload);

// Called by HAL from ISR context
void tx_complete_callback();
void rx_event_callback(uint16_t size);

} // namespace raspi
