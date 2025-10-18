/* Includes ------------------------------------------------------------------*/
#include "fiber_protocol.h"

#include <string.h>

/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Calculate CRC16-CCITT checksum
 * @param  data: Pointer to data buffer
 * @param  length: Data length
 * @retval CRC16 checksum
 */
uint16_t FiberProto_CalculateCRC16(const uint8_t* data, uint16_t length)
{
    uint16_t crc = 0xFFFF;         // Initial value
    uint16_t polynomial = 0x1021;  // CRC16-CCITT polynomial

    for (uint16_t i = 0; i < length; i++) {
        crc ^= ((uint16_t)data[i] << 8);
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ polynomial;
            }
            else {
                crc = crc << 1;
            }
        }
    }

    return crc;
}

/* Exported functions --------------------------------------------------------*/

/**
 * @brief  Initialize protocol instance
 */
bool FiberProto_Init(fiber_proto_instance_t* proto, fiber_optic_instance_t* fiber, fiber_channel_t channel)
{
    if (!proto || !fiber || !fiber->initialized) {
        return false;
    }

    proto->fiber_instance = fiber;
    proto->channel = channel;
    proto->tx_seq = 0;
    proto->rx_seq = 0;
    proto->initialized = true;

    // Reset statistics
    FiberProto_ResetStats(proto);

    return true;
}

/**
 * @brief  Send packet with uint32 payload
 */
fiber_proto_error_t FiberProto_SendPacket(
    fiber_proto_instance_t* proto, uint32_t command, const uint32_t* payload, uint8_t length)
{
    uint8_t tx_buffer[FIBER_PROTO_MAX_FRAME_SIZE];
    uint16_t frame_size;
    uint16_t crc;

    if (!proto || !proto->initialized) {
        return FIBER_PROTO_ERR_PARAM;
    }

    if (length > FIBER_PROTO_MAX_PAYLOAD_WORDS) {
        proto->stats.length_errors++;
        return FIBER_PROTO_ERR_LENGTH;
    }

    // Build frame header
    // [START:1B] [COMMAND:4B] [LENGTH:1B] [PAYLOAD:N×4B] [CRC:2B]
    tx_buffer[0] = FIBER_PROTO_START_BYTE;

    // Command (little-endian)
    tx_buffer[1] = (uint8_t)(command & 0xFF);
    tx_buffer[2] = (uint8_t)((command >> 8) & 0xFF);
    tx_buffer[3] = (uint8_t)((command >> 16) & 0xFF);
    tx_buffer[4] = (uint8_t)((command >> 24) & 0xFF);

    // Length (number of uint32 words)
    tx_buffer[5] = length;

    // Copy payload (uint32 array to bytes, little-endian)
    frame_size = FIBER_PROTO_HEADER_SIZE;
    if (length > 0 && payload != NULL) {
        for (uint8_t i = 0; i < length; i++) {
            uint32_t word = payload[i];
            tx_buffer[frame_size++] = (uint8_t)(word & 0xFF);
            tx_buffer[frame_size++] = (uint8_t)((word >> 8) & 0xFF);
            tx_buffer[frame_size++] = (uint8_t)((word >> 16) & 0xFF);
            tx_buffer[frame_size++] = (uint8_t)((word >> 24) & 0xFF);
        }
    }

    // Calculate CRC over header + payload
    crc = FiberProto_CalculateCRC16(tx_buffer, frame_size);

    // Append CRC (big-endian)
    tx_buffer[frame_size] = (uint8_t)(crc >> 8);
    tx_buffer[frame_size + 1] = (uint8_t)(crc & 0xFF);

    frame_size += FIBER_PROTO_CRC_SIZE;

    // Transmit frame
    if (!FiberOptic_Transmit(proto->fiber_instance, proto->channel, tx_buffer, frame_size)) {
        return FIBER_PROTO_ERR_BUFFER;
    }

    // Update statistics
    proto->tx_seq++;  // Auto-wrap at 65535
    proto->stats.last_tx_seq = proto->tx_seq;
    proto->stats.tx_count++;

    return FIBER_PROTO_OK;
}

/**
 * @brief  Receive and parse packet
 */
fiber_proto_error_t FiberProto_ReceivePacket(
    fiber_proto_instance_t* proto, uint32_t* command, uint32_t* payload, uint8_t* length)
{
    uint8_t rx_buffer[FIBER_PROTO_MAX_FRAME_SIZE];
    uint16_t rx_length;
    uint16_t expected_frame_size;
    uint16_t calculated_crc, received_crc;
    uint8_t payload_words;

    if (!proto || !proto->initialized || !command || !payload || !length) {
        return FIBER_PROTO_ERR_PARAM;
    }

    // Check if data available
    if (!FiberOptic_DataAvailable(proto->fiber_instance, proto->channel)) {
        proto->stats.timeout_count++;
        return FIBER_PROTO_ERR_TIMEOUT;
    }

    // Receive data
    rx_length = FiberOptic_Receive(proto->fiber_instance, proto->channel, rx_buffer, sizeof(rx_buffer));

    if (rx_length < (FIBER_PROTO_HEADER_SIZE + FIBER_PROTO_CRC_SIZE)) {
        proto->stats.length_errors++;
        return FIBER_PROTO_ERR_LENGTH;
    }

    // Check start byte
    if (rx_buffer[0] != FIBER_PROTO_START_BYTE) {
        proto->stats.length_errors++;
        return FIBER_PROTO_ERR_LENGTH;
    }

    // Extract header
    // Command (little-endian)
    *command = ((uint32_t)rx_buffer[1]) | ((uint32_t)rx_buffer[2] << 8) | ((uint32_t)rx_buffer[3] << 16) |
               ((uint32_t)rx_buffer[4] << 24);

    // Length (number of uint32 words)
    payload_words = rx_buffer[5];
    *length = payload_words;

    // Validate length
    if (payload_words > FIBER_PROTO_MAX_PAYLOAD_WORDS) {
        proto->stats.length_errors++;
        return FIBER_PROTO_ERR_LENGTH;
    }

    expected_frame_size = FIBER_PROTO_HEADER_SIZE + (payload_words * 4) + FIBER_PROTO_CRC_SIZE;
    if (rx_length != expected_frame_size) {
        proto->stats.length_errors++;
        return FIBER_PROTO_ERR_LENGTH;
    }

    // Extract payload (bytes to uint32 array, little-endian)
    uint16_t byte_index = FIBER_PROTO_HEADER_SIZE;
    for (uint8_t i = 0; i < payload_words; i++) {
        payload[i] = ((uint32_t)rx_buffer[byte_index]) | ((uint32_t)rx_buffer[byte_index + 1] << 8) |
                     ((uint32_t)rx_buffer[byte_index + 2] << 16) | ((uint32_t)rx_buffer[byte_index + 3] << 24);
        byte_index += 4;
    }

    // Extract CRC (big-endian)
    received_crc = ((uint16_t)rx_buffer[byte_index] << 8) | rx_buffer[byte_index + 1];

    // Calculate CRC
    calculated_crc = FiberProto_CalculateCRC16(rx_buffer, FIBER_PROTO_HEADER_SIZE + (payload_words * 4));

    // Verify CRC
    if (calculated_crc != received_crc) {
        proto->stats.crc_errors++;
        return FIBER_PROTO_ERR_CRC;
    }

    // Update statistics
    proto->rx_seq++;
    proto->stats.last_rx_seq = proto->rx_seq;
    proto->stats.rx_count++;

    return FIBER_PROTO_OK;
}

/**
 * @brief  Send floats (convenience function)
 */
fiber_proto_error_t FiberProto_SendFloats(
    fiber_proto_instance_t* proto, uint32_t command, const float* values, uint8_t count)
{
    uint32_t payload[FIBER_PROTO_MAX_PAYLOAD_WORDS];

    if (!values || count == 0 || count > FIBER_PROTO_MAX_PAYLOAD_WORDS) {
        return FIBER_PROTO_ERR_PARAM;
    }

    // Convert floats to uint32
    for (uint8_t i = 0; i < count; i++) {
        payload[i] = FiberProto_FloatToUint32(values[i]);
    }

    return FiberProto_SendPacket(proto, command, payload, count);
}

/**
 * @brief  Parse floats (convenience function)
 */
bool FiberProto_ParseFloats(const uint32_t* payload, uint8_t payload_length, float* values, uint8_t count)
{
    if (!payload || !values || count == 0 || count > payload_length) {
        return false;
    }

    // Convert uint32 to floats
    for (uint8_t i = 0; i < count; i++) {
        values[i] = FiberProto_Uint32ToFloat(payload[i]);
    }

    return true;
}

/**
 * @brief  Get protocol statistics
 */
const fiber_proto_stats_t* FiberProto_GetStats(const fiber_proto_instance_t* proto)
{
    if (!proto || !proto->initialized) {
        return NULL;
    }

    return &proto->stats;
}

/**
 * @brief  Reset protocol statistics
 */
void FiberProto_ResetStats(fiber_proto_instance_t* proto)
{
    if (!proto) {
        return;
    }

    memset(&proto->stats, 0, sizeof(fiber_proto_stats_t));
}
