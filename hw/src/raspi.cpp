#include <hw/inc/raspi.hpp>
#include <cstring>

namespace raspi {
namespace {

constexpr uint16_t RX_BUF_SIZE = 256;
constexpr uint16_t TX_BUF_SIZE = 134; // max frame: 4 header + 128 payload + 2 CRC

UART_HandleTypeDef* uart_handle = nullptr;

// TX state
uint8_t tx_buf[TX_BUF_SIZE];
volatile bool tx_busy = false;

// RX DMA circular buffer
uint8_t rx_dma_buf[RX_BUF_SIZE];
uint16_t rx_read_pos = 0;

// Lightweight stream decoder (no heap allocation)
enum class RxState : uint8_t {
	HUNT_SYNC_HI,
	HUNT_SYNC_LO,
	READ_HEADER,
	READ_PAYLOAD,
	READ_CRC,
};

struct Decoder {
	RxState  state = RxState::HUNT_SYNC_HI;
	uint8_t  header_buf[2]{};
	uint8_t  payload_buf[MAX_PAYLOAD_SIZE]{};
	uint8_t  crc_buf[2]{};
	uint8_t  header_pos = 0;
	uint8_t  payload_pos = 0;
	uint8_t  crc_pos = 0;
	uint8_t  expected_len = 0;
	uint8_t  packet_id = 0;
	uint32_t crc_errors = 0;
	uint32_t sync_losses = 0;
};

Decoder decoder;

void dispatch_packet(uint8_t packet_id, const uint8_t* payload, uint8_t length);

void process_byte(uint8_t byte) {
	switch (decoder.state) {
	case RxState::HUNT_SYNC_HI:
		if (byte == SYNC_HI) {
			decoder.state = RxState::HUNT_SYNC_LO;
		}
		break;

	case RxState::HUNT_SYNC_LO:
		if (byte == SYNC_LO) {
			decoder.header_pos = 0;
			decoder.state = RxState::READ_HEADER;
		} else if (byte == SYNC_HI) {
			// stay in HUNT_SYNC_LO
		} else {
			decoder.sync_losses++;
			decoder.state = RxState::HUNT_SYNC_HI;
		}
		break;

	case RxState::READ_HEADER:
		decoder.header_buf[decoder.header_pos++] = byte;
		if (decoder.header_pos == 2) {
			decoder.packet_id = decoder.header_buf[0];
			decoder.expected_len = decoder.header_buf[1];
			if (decoder.expected_len > MAX_PAYLOAD_SIZE) {
				decoder.sync_losses++;
				decoder.state = RxState::HUNT_SYNC_HI;
			} else if (decoder.expected_len == 0) {
				decoder.crc_pos = 0;
				decoder.state = RxState::READ_CRC;
			} else {
				decoder.payload_pos = 0;
				decoder.state = RxState::READ_PAYLOAD;
			}
		}
		break;

	case RxState::READ_PAYLOAD:
		decoder.payload_buf[decoder.payload_pos++] = byte;
		if (decoder.payload_pos == decoder.expected_len) {
			decoder.crc_pos = 0;
			decoder.state = RxState::READ_CRC;
		}
		break;

	case RxState::READ_CRC:
		decoder.crc_buf[decoder.crc_pos++] = byte;
		if (decoder.crc_pos == 2) {
			// Reconstruct frame for CRC verification
			uint8_t frame[4 + MAX_PAYLOAD_SIZE];
			frame[0] = SYNC_HI;
			frame[1] = SYNC_LO;
			frame[2] = decoder.packet_id;
			frame[3] = decoder.expected_len;
			std::memcpy(&frame[4], decoder.payload_buf, decoder.expected_len);

			uint16_t computed = crc16_ccitt(frame, 4 + decoder.expected_len);
			uint16_t received = static_cast<uint16_t>(decoder.crc_buf[0])
			                  | (static_cast<uint16_t>(decoder.crc_buf[1]) << 8);

			if (computed == received) {
				dispatch_packet(decoder.packet_id, decoder.payload_buf, decoder.expected_len);
			} else {
				decoder.crc_errors++;
			}
			decoder.state = RxState::HUNT_SYNC_HI;
		}
		break;
	}
}

void dispatch_packet(uint8_t packet_id, const uint8_t* payload, uint8_t length) {
	(void)packet_id;
	(void)payload;
	(void)length;
}

} // anonymous namespace

void init(UART_HandleTypeDef* huart) {
	uart_handle = huart;
	tx_busy = false;
	rx_read_pos = 0;
	decoder = Decoder{};

	HAL_UARTEx_ReceiveToIdle_DMA(uart_handle, rx_dma_buf, RX_BUF_SIZE);
	// Disable half-transfer interrupt, we only care about idle line and transfer complete
	__HAL_DMA_DISABLE_IT(uart_handle->hdmarx, DMA_IT_HT);
}

void process() {
	if (uart_handle == nullptr) return;

	uint16_t write_pos = RX_BUF_SIZE - __HAL_DMA_GET_COUNTER(uart_handle->hdmarx);

	while (rx_read_pos != write_pos) {
		process_byte(rx_dma_buf[rx_read_pos]);
		rx_read_pos = (rx_read_pos + 1) % RX_BUF_SIZE;
	}
}

bool send_packet(uint8_t packet_id, const uint8_t* payload, uint8_t length) {
	if (uart_handle == nullptr) return false;
	if (length > MAX_PAYLOAD_SIZE) return false;

	uint32_t start_wait = HAL_GetTick();
	while (tx_busy) {
		if (HAL_GetTick() - start_wait > 10) return false;
	}

	tx_buf[0] = SYNC_HI;
	tx_buf[1] = SYNC_LO;
	tx_buf[2] = packet_id;
	tx_buf[3] = length;
	std::memcpy(&tx_buf[4], payload, length);

	uint16_t crc = crc16_ccitt(tx_buf, 4 + length);
	tx_buf[4 + length] = static_cast<uint8_t>(crc & 0xFF);
	tx_buf[5 + length] = static_cast<uint8_t>(crc >> 8);

	tx_busy = true;
	HAL_UART_Transmit_DMA(uart_handle, tx_buf, 6 + length);
	return true;
}

bool send_gps(const GpsPayload& p) {
	return send_packet(PACKET_GPS, reinterpret_cast<const uint8_t*>(&p), sizeof(p));
}

bool send_encoder(const EncoderPayload& p) {
	return send_packet(PACKET_ENCODER, reinterpret_cast<const uint8_t*>(&p), sizeof(p));
}

bool send_imu(const ImuPayload& p) {
	return send_packet(PACKET_IMU, reinterpret_cast<const uint8_t*>(&p), sizeof(p));
}

bool send_state_sync(const StateSyncPayload& p) {
	return send_packet(PACKET_STATE_SYNC, reinterpret_cast<const uint8_t*>(&p), sizeof(p));
}

bool send_dso_target(const DsoTargetPayload& p) {
	return send_packet(PACKET_DSO_TARGET, reinterpret_cast<const uint8_t*>(&p), sizeof(p));
}

bool send_debug(const DebugPayload& p) {
	return send_packet(PACKET_DEBUG, reinterpret_cast<const uint8_t*>(&p), sizeof(p));
}

void tx_complete_callback() {
	tx_busy = false;
}

void rx_event_callback(uint16_t size) {
	// DMA circular mode handles continuous reception.
	// Re-enable if HAL stopped it (shouldn't happen in circular mode, but defensive).
	if (uart_handle->RxState == HAL_UART_STATE_READY) {
		HAL_UARTEx_ReceiveToIdle_DMA(uart_handle, rx_dma_buf, RX_BUF_SIZE);
		__HAL_DMA_DISABLE_IT(uart_handle->hdmarx, DMA_IT_HT);
	}
}

} // namespace raspi
