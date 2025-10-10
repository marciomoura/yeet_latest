/**
 * @file optical_fiber_driver.c
 * @brief FPGA Optical Fiber Communication Driver Implementation
 *
 * Real-time safe implementation with no dynamic memory allocation.
 * All operations are deterministic and suitable for use in control loops.
 */

#include "optical_fiber_driver.h"

#include <string.h>


// ============================================================================
// Private Type Definitions
// ============================================================================

// Register access macros for type safety
#define REG_WRITE16(addr, value) (*((volatile uint16_t*)(addr)) = (value))
#define REG_READ16(addr)         (*((volatile uint16_t*)(addr)))

// ============================================================================
// Private Function Prototypes
// ============================================================================

static uint16_t calculate_baud_divisor(uint32_t baud_rate);
static volatile uint16_t* get_tx_register(uint8_t channel);
static volatile uint16_t* get_rx_register(uint8_t channel);
static volatile uint16_t* get_status_register(uint8_t channel);
static void assemble_frame(uint8_t* frame, const uint8_t* payload, uint8_t payload_length, uint8_t source_node);
static bool validate_frame_header(const uint8_t* buffer, uint16_t buffer_length);
static void process_received_data(optical_fiber_t* fiber, uint8_t channel_index);

// ============================================================================
// Public Functions - Initialization
// ============================================================================

void OpticalFiber_Init(optical_fiber_t* fiber, uint32_t baud_rate, uint8_t node_id)
{
    // Clear all state
    memset(fiber, 0, sizeof(optical_fiber_t));

    // Store configuration
    fiber->node_id = node_id;
    fiber->baud_rate = baud_rate;

    // Calculate and write baud rate divisor
    uint16_t divisor = calculate_baud_divisor(baud_rate);
    REG_WRITE16(FPGA_BAUD_RATE_REG, divisor);

    // Write node ID
    REG_WRITE16(FPGA_NODE_ID_REG, node_id);

    // Initialize channel status
    for (uint8_t i = 0; i < OPTICAL_NUM_CHANNELS; i++) {
        fiber->channel[i].rx_fifo_empty = true;
        fiber->channel[i].rx_fifo_full = false;
        fiber->channel[i].tx_busy = false;
    }
}

void OpticalFiber_Reset(optical_fiber_t* fiber)
{
    uint8_t saved_node_id = fiber->node_id;
    uint32_t saved_baud_rate = fiber->baud_rate;

    // Clear all state except configuration
    memset(fiber, 0, sizeof(optical_fiber_t));

    // Restore configuration
    fiber->node_id = saved_node_id;
    fiber->baud_rate = saved_baud_rate;

    // Reset RX FIFO empty flags
    for (uint8_t i = 0; i < OPTICAL_NUM_CHANNELS; i++) {
        fiber->channel[i].rx_fifo_empty = true;
    }
}

// ============================================================================
// Public Functions - Transmit
// ============================================================================

bool OpticalFiber_Transmit(optical_fiber_t* fiber, uint8_t channel, const uint8_t* data, uint16_t length)
{
    // Validate parameters
    if (data == NULL || length == 0 || length > OPTICAL_FRAME_SIZE) {
        return false;
    }

    // Validate channel (0=broadcast, 1-7=specific)
    if (channel > OPTICAL_NUM_CHANNELS) {
        return false;
    }

    // Get TX register for this channel
    volatile uint16_t* tx_reg = get_tx_register(channel);
    if (tx_reg == NULL) {
        return false;
    }

    // Check if transmitter is busy (for non-broadcast channels)
    if (channel > 0) {
        uint8_t channel_index = CHANNEL_TO_INDEX(channel);
        volatile uint16_t* status_reg = get_status_register(channel);
        uint16_t status = REG_READ16(status_reg);

        if (status & FPGA_TX_BUSY) {
            // Transmitter is busy, cannot send
            return false;
        }
    }

    // Write data to TX FIFO (auto-incrementing register)
    // Note: Data is written as 16-bit words, padding odd-length data with zero
    for (uint16_t i = 0; i < length; i += 2) {
        uint16_t word;
        if (i + 1 < length) {
            // Two bytes available
            word = (uint16_t)data[i] | ((uint16_t)data[i + 1] << 8);
        }
        else {
            // Only one byte available, pad with zero
            word = (uint16_t)data[i];
        }
        REG_WRITE16(tx_reg, word);
    }

    // Update statistics (for non-broadcast)
    if (channel > 0) {
        uint8_t channel_index = CHANNEL_TO_INDEX(channel);
        fiber->channel[channel_index].frames_transmitted++;
    }

    return true;
}

bool OpticalFiber_TransmitFrame(optical_fiber_t* fiber, uint8_t channel, const uint8_t* payload, uint8_t payload_length)
{
    // Validate parameters
    if (payload == NULL && payload_length > 0) {
        return false;
    }

    if (payload_length > OPTICAL_MAX_PAYLOAD) {
        return false;
    }

    // Assemble frame
    uint8_t frame[OPTICAL_FRAME_SIZE];
    assemble_frame(frame, payload, payload_length, fiber->node_id);

    // Transmit complete frame
    return OpticalFiber_Transmit(fiber, channel, frame, OPTICAL_FRAME_SIZE);
}

// ============================================================================
// Public Functions - Receive
// ============================================================================

void OpticalFiber_Poll(optical_fiber_t* fiber)
{
    fiber->poll_count++;

    // Poll each channel
    for (uint8_t channel_index = 0; channel_index < OPTICAL_NUM_CHANNELS; channel_index++) {
        // Update channel status
        uint8_t channel = INDEX_TO_CHANNEL(channel_index);
        OpticalFiber_UpdateChannelStatus(fiber, channel);

        // Skip if RX FIFO is empty
        if (fiber->channel[channel_index].rx_fifo_empty) {
            continue;
        }

        // Check for FIFO overrun
        if (fiber->channel[channel_index].rx_fifo_full) {
            fiber->channel[channel_index].rx_overruns++;
        }

        // Process received data
        process_received_data(fiber, channel_index);
    }
}

bool OpticalFiber_IsFrameReady(const optical_fiber_t* fiber, uint8_t channel)
{
    if (!IS_VALID_CHANNEL(channel)) {
        return false;
    }

    uint8_t channel_index = CHANNEL_TO_INDEX(channel);
    return fiber->rx_buffer[channel_index].frame_ready;
}

uint16_t OpticalFiber_GetFrame(optical_fiber_t* fiber, uint8_t channel, uint8_t* frame_out)
{
    if (!IS_VALID_CHANNEL(channel) || frame_out == NULL) {
        return 0;
    }

    uint8_t channel_index = CHANNEL_TO_INDEX(channel);
    optical_rx_buffer_t* rx_buf = &fiber->rx_buffer[channel_index];

    // Check if frame is ready
    if (!rx_buf->frame_ready) {
        return 0;
    }

    // Copy frame to output buffer
    memcpy(frame_out, rx_buf->buffer, OPTICAL_FRAME_SIZE);

    // Mark buffer as consumed
    rx_buf->frame_ready = false;
    rx_buf->write_index = 0;

    return OPTICAL_FRAME_SIZE;
}

uint16_t OpticalFiber_ExtractPayload(
    const uint8_t* frame, uint8_t* payload_out, uint16_t max_payload_size, uint8_t* source_node_out)
{
    if (frame == NULL || payload_out == NULL) {
        return 0;
    }

    // Validate frame header
    if (frame[FRAME_OFFSET_HEADER1] != OPTICAL_FRAME_HEADER || frame[FRAME_OFFSET_HEADER2] != OPTICAL_FRAME_HEADER) {
        return 0;
    }

    // Verify CRC
    if (!OpticalFiber_VerifyFrameCRC(frame)) {
        return 0;
    }

    // Get payload length
    uint8_t payload_length = frame[FRAME_OFFSET_LENGTH];
    if (payload_length > OPTICAL_MAX_PAYLOAD) {
        return 0;
    }

    // Check output buffer size
    if (payload_length > max_payload_size) {
        return 0;
    }

    // Extract source node if requested
    if (source_node_out != NULL) {
        *source_node_out = frame[FRAME_OFFSET_SOURCE];
    }

    // Copy payload
    memcpy(payload_out, &frame[FRAME_OFFSET_PAYLOAD], payload_length);

    return payload_length;
}

// ============================================================================
// Public Functions - Status
// ============================================================================

void OpticalFiber_UpdateChannelStatus(optical_fiber_t* fiber, uint8_t channel)
{
    if (!IS_VALID_CHANNEL(channel)) {
        return;
    }

    uint8_t channel_index = CHANNEL_TO_INDEX(channel);
    volatile uint16_t* status_reg = get_status_register(channel);

    if (status_reg == NULL) {
        return;
    }

    // Read status register
    uint16_t status = REG_READ16(status_reg);
    fiber->channel[channel_index].status_reg = status;

    // Parse status bits
    fiber->channel[channel_index].tx_busy = (status & FPGA_TX_BUSY) != 0;
    fiber->channel[channel_index].rx_fifo_empty = (status & FPGA_RX_FIFO_EMPTY) != 0;
    fiber->channel[channel_index].rx_fifo_full = (status & FPGA_RX_FIFO_FULL) != 0;
}

const optical_channel_status_t* OpticalFiber_GetChannelStatus(const optical_fiber_t* fiber, uint8_t channel)
{
    if (!IS_VALID_CHANNEL(channel)) {
        return NULL;
    }

    uint8_t channel_index = CHANNEL_TO_INDEX(channel);
    return &fiber->channel[channel_index];
}

// ============================================================================
// Public Functions - Utility
// ============================================================================

uint16_t OpticalFiber_CRC16(const uint8_t* data, uint16_t length)
{
    uint16_t crc = 0xFFFF;

    for (uint16_t i = 0; i < length; i++) {
        crc ^= (uint16_t)data[i];

        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;  // Modbus polynomial
            }
            else {
                crc = crc >> 1;
            }
        }
    }

    return crc;
}

bool OpticalFiber_VerifyFrameCRC(const uint8_t* frame)
{
    if (frame == NULL) {
        return false;
    }

    // Calculate CRC for bytes [0-125] (all except last 2 bytes which contain CRC)
    uint16_t calculated_crc = OpticalFiber_CRC16(frame, FRAME_OFFSET_CRC);

    // Extract stored CRC (little-endian)
    uint16_t stored_crc = (uint16_t)frame[FRAME_OFFSET_CRC] | ((uint16_t)frame[FRAME_OFFSET_CRC + 1] << 8);

    return (calculated_crc == stored_crc);
}

// ============================================================================
// Private Functions
// ============================================================================

static uint16_t calculate_baud_divisor(uint32_t baud_rate)
{
    // Baud divisor = OPTICAL_BAUD_RATE_BASE / desired_baud_rate
    // Clamp to 16-bit range
    uint32_t divisor = OPTICAL_BAUD_RATE_BASE / baud_rate;

    if (divisor > 0xFFFF) {
        divisor = 0xFFFF;
    }
    else if (divisor < 1) {
        divisor = 1;
    }

    return (uint16_t)divisor;
}

static volatile uint16_t* get_tx_register(uint8_t channel)
{
    switch (channel) {
        case 0:
            return (volatile uint16_t*)FPGA_TX_BROADCAST;
        case 1:
            return (volatile uint16_t*)FPGA_TX_PORT1;
        case 2:
            return (volatile uint16_t*)FPGA_TX_PORT2;
        case 3:
            return (volatile uint16_t*)FPGA_TX_PORT3;
        case 4:
            return (volatile uint16_t*)FPGA_TX_PORT4;
        case 5:
            return (volatile uint16_t*)FPGA_TX_PORT5;
        case 6:
            return (volatile uint16_t*)FPGA_TX_PORT6;
        case 7:
            return (volatile uint16_t*)FPGA_TX_PORT7;
        default:
            return NULL;
    }
}

static volatile uint16_t* get_rx_register(uint8_t channel)
{
    switch (channel) {
        case 1:
            return (volatile uint16_t*)FPGA_RX_PORT1;
        case 2:
            return (volatile uint16_t*)FPGA_RX_PORT2;
        case 3:
            return (volatile uint16_t*)FPGA_RX_PORT3;
        case 4:
            return (volatile uint16_t*)FPGA_RX_PORT4;
        case 5:
            return (volatile uint16_t*)FPGA_RX_PORT5;
        case 6:
            return (volatile uint16_t*)FPGA_RX_PORT6;
        case 7:
            return (volatile uint16_t*)FPGA_RX_PORT7;
        default:
            return NULL;
    }
}

static volatile uint16_t* get_status_register(uint8_t channel)
{
    switch (channel) {
        case 1:
            return (volatile uint16_t*)FPGA_STATUS_PORT1;
        case 2:
            return (volatile uint16_t*)FPGA_STATUS_PORT2;
        case 3:
            return (volatile uint16_t*)FPGA_STATUS_PORT3;
        case 4:
            return (volatile uint16_t*)FPGA_STATUS_PORT4;
        case 5:
            return (volatile uint16_t*)FPGA_STATUS_PORT5;
        case 6:
            return (volatile uint16_t*)FPGA_STATUS_PORT6;
        case 7:
            return (volatile uint16_t*)FPGA_STATUS_PORT7;
        default:
            return NULL;
    }
}

static void assemble_frame(uint8_t* frame, const uint8_t* payload, uint8_t payload_length, uint8_t source_node)
{
    // Clear entire frame
    memset(frame, 0, OPTICAL_FRAME_SIZE);

    // Set header
    frame[FRAME_OFFSET_HEADER1] = OPTICAL_FRAME_HEADER;
    frame[FRAME_OFFSET_HEADER2] = OPTICAL_FRAME_HEADER;

    // Set length
    frame[FRAME_OFFSET_LENGTH] = payload_length;

    // Set source
    frame[FRAME_OFFSET_SOURCE] = source_node;

    // Copy payload
    if (payload != NULL && payload_length > 0) {
        memcpy(&frame[FRAME_OFFSET_PAYLOAD], payload, payload_length);
    }

    // Calculate and store CRC (for bytes 0-125)
    uint16_t crc = OpticalFiber_CRC16(frame, FRAME_OFFSET_CRC);
    frame[FRAME_OFFSET_CRC] = (uint8_t)(crc & 0xFF);             // Low byte
    frame[FRAME_OFFSET_CRC + 1] = (uint8_t)((crc >> 8) & 0xFF);  // High byte
}

static bool validate_frame_header(const uint8_t* buffer, uint16_t buffer_length)
{
    // Need at least 4 bytes for minimal validation
    if (buffer_length < 4) {
        return false;
    }

    // Check header bytes
    if (buffer[FRAME_OFFSET_HEADER1] != OPTICAL_FRAME_HEADER || buffer[FRAME_OFFSET_HEADER2] != OPTICAL_FRAME_HEADER) {
        return false;
    }

    // Check payload length
    uint8_t payload_length = buffer[FRAME_OFFSET_LENGTH];
    if (payload_length > OPTICAL_MAX_PAYLOAD) {
        return false;
    }

    return true;
}

static void process_received_data(optical_fiber_t* fiber, uint8_t channel_index)
{
    uint8_t channel = INDEX_TO_CHANNEL(channel_index);
    optical_rx_buffer_t* rx_buf = &fiber->rx_buffer[channel_index];

    // Get RX register
    volatile uint16_t* rx_reg = get_rx_register(channel);
    if (rx_reg == NULL) {
        return;
    }

    // If frame is already ready, don't overwrite it
    if (rx_buf->frame_ready) {
        return;
    }

    // Read bytes from FIFO until empty or buffer full
    // Note: Reading the register auto-increments the FIFO pointer
    const uint16_t max_reads_per_poll = 128;  // Limit reads to prevent blocking
    uint16_t reads = 0;

    while (reads < max_reads_per_poll) {
        // Check if FIFO is now empty
        OpticalFiber_UpdateChannelStatus(fiber, channel);
        if (fiber->channel[channel_index].rx_fifo_empty) {
            break;
        }

        // Read 16-bit word from FIFO
        uint16_t word = REG_READ16(rx_reg);

        // Extract two bytes
        uint8_t byte_low = (uint8_t)(word & 0xFF);
        uint8_t byte_high = (uint8_t)((word >> 8) & 0xFF);

        // Store first byte
        if (rx_buf->write_index < sizeof(rx_buf->buffer)) {
            rx_buf->buffer[rx_buf->write_index++] = byte_low;
        }

        // Store second byte
        if (rx_buf->write_index < sizeof(rx_buf->buffer)) {
            rx_buf->buffer[rx_buf->write_index++] = byte_high;
        }

        reads++;

        // Check if we have a complete frame
        if (rx_buf->write_index >= OPTICAL_FRAME_SIZE) {
            // Validate frame header
            if (validate_frame_header(rx_buf->buffer, rx_buf->write_index)) {
                // Verify CRC
                if (OpticalFiber_VerifyFrameCRC(rx_buf->buffer)) {
                    // Valid frame received
                    rx_buf->frame_ready = true;
                    rx_buf->source_node = rx_buf->buffer[FRAME_OFFSET_SOURCE];
                    fiber->channel[channel_index].frames_received++;
                    break;  // Stop reading this channel
                }
                else {
                    // CRC error
                    fiber->channel[channel_index].crc_errors++;
                    rx_buf->write_index = 0;  // Discard and restart
                }
            }
            else {
                // Invalid header - search for next valid header
                // Simple strategy: discard everything and restart
                rx_buf->write_index = 0;
            }
        }
    }

    // Check for buffer overflow (shouldn't happen with proper frame size)
    if (rx_buf->write_index >= sizeof(rx_buf->buffer)) {
        // Buffer full without valid frame - reset
        rx_buf->write_index = 0;
    }
}
