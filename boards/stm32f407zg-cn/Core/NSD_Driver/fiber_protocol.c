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

    // Initialize RX buffer
    proto->rx_buffer_len = 0;
    proto->last_rx_time_ms = 0;
    memset(proto->rx_buffer, 0, sizeof(proto->rx_buffer));

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
 * @brief  Receive and parse packet with buffered reception
 * @note   This function accumulates data across multiple calls until a complete
 *         frame is received. Safe for 1ms periodic calling even if frames arrive
 *         slowly or span multiple cycles.
 */
fiber_proto_error_t FiberProto_ReceivePacket(
    fiber_proto_instance_t* proto, uint32_t* command, uint32_t* payload, uint8_t* length)
{
    uint16_t bytes_available;
    uint16_t bytes_to_read;
    uint16_t expected_frame_size;
    uint16_t calculated_crc, received_crc;
    uint8_t payload_words;
    uint32_t current_time_ms;
    extern uint32_t get_1ms_counter(void);  // From tasks.c

    if (!proto || !proto->initialized || !command || !payload || !length) {
        return FIBER_PROTO_ERR_PARAM;
    }

    // Get current timestamp for timeout detection
    current_time_ms = get_1ms_counter();

    // Check if RX buffer has stale data (timeout expired)
    if (proto->rx_buffer_len > 0) {
        uint32_t elapsed_ms = current_time_ms - proto->last_rx_time_ms;
        if (elapsed_ms > FIBER_PROTO_RX_TIMEOUT_MS) {
            // Timeout - discard stale partial frame
            proto->rx_buffer_len = 0;
            proto->stats.timeout_count++;
        }
    }

    // Try to read new data from hardware FIFO
    if (FiberOptic_DataAvailable(proto->fiber_instance, proto->channel)) {
        // Calculate remaining space in buffer
        bytes_to_read = FIBER_PROTO_MAX_FRAME_SIZE - proto->rx_buffer_len;

        // Read new data and append to buffer
        bytes_available = FiberOptic_Receive(
            proto->fiber_instance, proto->channel, &proto->rx_buffer[proto->rx_buffer_len], bytes_to_read);

        if (bytes_available > 0) {
            proto->rx_buffer_len += bytes_available;
            proto->last_rx_time_ms = current_time_ms;  // Update timestamp
        }
    }

    // Check if we have any data buffered
    if (proto->rx_buffer_len == 0) {
        proto->stats.timeout_count++;
        return FIBER_PROTO_ERR_TIMEOUT;
    }

    // Check for minimum frame size (START + CMD + LEN + CRC = 8 bytes)
    if (proto->rx_buffer_len < (FIBER_PROTO_HEADER_SIZE + FIBER_PROTO_CRC_SIZE)) {
        // Not enough data yet - wait for more in next cycle
        return FIBER_PROTO_ERR_TIMEOUT;
    }

    // Check start byte
    if (proto->rx_buffer[0] != FIBER_PROTO_START_BYTE) {
        // Invalid start byte - scan for valid start byte
        bool found_start = false;
        for (uint16_t i = 1; i < proto->rx_buffer_len; i++) {
            if (proto->rx_buffer[i] == FIBER_PROTO_START_BYTE) {
                // Found potential start - shift buffer
                uint16_t shift_amount = i;
                memmove(proto->rx_buffer, &proto->rx_buffer[i], proto->rx_buffer_len - i);
                proto->rx_buffer_len -= shift_amount;
                found_start = true;
                break;
            }
        }

        if (!found_start) {
            // No valid start byte found - clear buffer
            proto->rx_buffer_len = 0;
        }

        proto->stats.length_errors++;
        return FIBER_PROTO_ERR_LENGTH;
    }

    // Extract payload length from header (we have at least 8 bytes at this point)
    payload_words = proto->rx_buffer[5];

    // Validate length
    if (payload_words > FIBER_PROTO_MAX_PAYLOAD_WORDS) {
        // Invalid length - discard frame and resync
        proto->rx_buffer_len = 0;
        proto->stats.length_errors++;
        return FIBER_PROTO_ERR_LENGTH;
    }

    // Calculate expected frame size
    expected_frame_size = FIBER_PROTO_HEADER_SIZE + (payload_words * 4) + FIBER_PROTO_CRC_SIZE;

    // Check if we have complete frame
    if (proto->rx_buffer_len < expected_frame_size) {
        // Incomplete frame - wait for more data in next cycle
        return FIBER_PROTO_ERR_TIMEOUT;
    }

    // === Complete frame received - parse it ===

    // Extract command (little-endian)
    *command = ((uint32_t)proto->rx_buffer[1]) | ((uint32_t)proto->rx_buffer[2] << 8) |
               ((uint32_t)proto->rx_buffer[3] << 16) | ((uint32_t)proto->rx_buffer[4] << 24);

    *length = payload_words;

    // Extract payload (bytes to uint32 array, little-endian)
    uint16_t byte_index = FIBER_PROTO_HEADER_SIZE;
    for (uint8_t i = 0; i < payload_words; i++) {
        payload[i] = ((uint32_t)proto->rx_buffer[byte_index]) | ((uint32_t)proto->rx_buffer[byte_index + 1] << 8) |
                     ((uint32_t)proto->rx_buffer[byte_index + 2] << 16) |
                     ((uint32_t)proto->rx_buffer[byte_index + 3] << 24);
        byte_index += 4;
    }

    // Extract CRC (big-endian)
    received_crc = ((uint16_t)proto->rx_buffer[byte_index] << 8) | proto->rx_buffer[byte_index + 1];

    // Calculate CRC
    calculated_crc = FiberProto_CalculateCRC16(proto->rx_buffer, FIBER_PROTO_HEADER_SIZE + (payload_words * 4));

    // Verify CRC
    if (calculated_crc != received_crc) {
        // CRC error - discard frame and clear buffer
        proto->rx_buffer_len = 0;
        proto->stats.crc_errors++;
        return FIBER_PROTO_ERR_CRC;
    }

    // Success! Remove processed frame from buffer
    if (proto->rx_buffer_len > expected_frame_size) {
        // More data in buffer - shift remaining bytes to start
        uint16_t remaining = proto->rx_buffer_len - expected_frame_size;
        memmove(proto->rx_buffer, &proto->rx_buffer[expected_frame_size], remaining);
        proto->rx_buffer_len = remaining;
    }
    else {
        // Buffer exactly consumed
        proto->rx_buffer_len = 0;
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
