/**
 * @file optical_fiber_driver.h
 * @brief FPGA Optical Fiber Communication Driver
 *
 * This driver provides access to the FPGA-based optical fiber transceiver system
 * via FSMC memory-mapped registers. Supports 7 bidirectional channels plus broadcast.
 *
 * Hardware Interface: FSMC Bank 4 (0x6C000000) - shared with digital I/O
 * Protocol: 128-byte fixed frames with CRC-16
 * Channels: 1-7 (bidirectional), 0 (broadcast transmit only)
 *
 * Usage:
 *   1. Call OpticalFiber_Init() once during system startup
 *   2. Call OpticalFiber_Poll() periodically (e.g., 1 kHz in Task 2)
 *   3. Use OpticalFiber_TransmitFrame() to send data
 *   4. Check OpticalFiber_IsFrameReady() and OpticalFiber_GetFrame() to receive
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

// Include HAL for GPIO definitions
#include "stm32f4xx_hal.h"

// ============================================================================
// GPIO Pin Definitions for FPGA Control
// ============================================================================

// FPGA PROGRAMB - Triggers FPGA reconfiguration from flash (active low pulse)
#define FPGA_PROGRAMB_Pin       GPIO_PIN_15
#define FPGA_PROGRAMB_GPIO_Port GPIOG

// FPGA DONE - Indicates FPGA configuration complete (input, active high)
#define FPGA_DONE_Pin       GPIO_PIN_2
#define FPGA_DONE_GPIO_Port GPIOD

// FPGA RST - Internal FPGA soft reset (output, active low pulse)
#define FPGA_RST_Pin       GPIO_PIN_11
#define FPGA_RST_GPIO_Port GPIOG

// ============================================================================
// FSMC Register Definitions
// ============================================================================

// FSMC Base Address (same as FPGA digital I/O)
#define FPGA_BASE_ADDR 0x6C000000

// Configuration Registers (Write-Only)
#define FPGA_BAUD_RATE_REG (FPGA_BASE_ADDR + 0x0004)  // Baud rate divisor
#define FPGA_NODE_ID_REG   (FPGA_BASE_ADDR + 0x0010)  // This node's ID (0-5)

// Transmit Registers (Write-Only, Auto-Increment on each write)
#define FPGA_TX_BROADCAST (FPGA_BASE_ADDR + 0x00C8)  // Port 0: Broadcast
#define FPGA_TX_PORT1     (FPGA_BASE_ADDR + 0x00CA)  // Port 1
#define FPGA_TX_PORT2     (FPGA_BASE_ADDR + 0x00CC)  // Port 2
#define FPGA_TX_PORT3     (FPGA_BASE_ADDR + 0x00CE)  // Port 3
#define FPGA_TX_PORT4     (FPGA_BASE_ADDR + 0x00D0)  // Port 4
#define FPGA_TX_PORT5     (FPGA_BASE_ADDR + 0x00D2)  // Port 5
#define FPGA_TX_PORT6     (FPGA_BASE_ADDR + 0x00D4)  // Port 6
#define FPGA_TX_PORT7     (FPGA_BASE_ADDR + 0x00D6)  // Port 7

// Receive Registers (Read-Only, Auto-Increment on each read)
#define FPGA_RX_PORT1 (FPGA_BASE_ADDR + 0x0002)  // Port 1
#define FPGA_RX_PORT2 (FPGA_BASE_ADDR + 0x0004)  // Port 2
#define FPGA_RX_PORT3 (FPGA_BASE_ADDR + 0x0006)  // Port 3
#define FPGA_RX_PORT4 (FPGA_BASE_ADDR + 0x0008)  // Port 4
#define FPGA_RX_PORT5 (FPGA_BASE_ADDR + 0x000A)  // Port 5
#define FPGA_RX_PORT6 (FPGA_BASE_ADDR + 0x000C)  // Port 6
#define FPGA_RX_PORT7 (FPGA_BASE_ADDR + 0x000E)  // Port 7

// Status Registers (Read-Only)
#define FPGA_STATUS_PORT1 (FPGA_BASE_ADDR + 0x00CA)  // Port 1 status
#define FPGA_STATUS_PORT2 (FPGA_BASE_ADDR + 0x00CC)  // Port 2 status
#define FPGA_STATUS_PORT3 (FPGA_BASE_ADDR + 0x00CE)  // Port 3 status
#define FPGA_STATUS_PORT4 (FPGA_BASE_ADDR + 0x00D0)  // Port 4 status
#define FPGA_STATUS_PORT5 (FPGA_BASE_ADDR + 0x00D2)  // Port 5 status
#define FPGA_STATUS_PORT6 (FPGA_BASE_ADDR + 0x00D4)  // Port 6 status
#define FPGA_STATUS_PORT7 (FPGA_BASE_ADDR + 0x00D6)  // Port 7 status

// Status Register Bit Masks
#define FPGA_TX_BUSY       0x0001  // Bit 0: Transmitter busy
#define FPGA_RX_FIFO_EMPTY 0x0002  // Bit 1: RX FIFO empty
#define FPGA_RX_FIFO_FULL  0x0004  // Bit 2: RX FIFO full (data loss risk)

// ============================================================================
// Protocol Constants
// ============================================================================

#define OPTICAL_FRAME_SIZE   128   // Total frame size (bytes)
#define OPTICAL_MAX_PAYLOAD  120   // Maximum payload size (bytes)
#define OPTICAL_FRAME_HEADER 0x68  // Frame header byte
#define OPTICAL_NUM_CHANNELS 7     // Number of physical channels (1-7)

// Baud rate calculation: divisor = 1382400 / desired_baud_rate
#define OPTICAL_BAUD_RATE_BASE 1382400

// Frame structure offsets
#define FRAME_OFFSET_HEADER1 0                         // First header byte
#define FRAME_OFFSET_HEADER2 1                         // Second header byte
#define FRAME_OFFSET_LENGTH  2                         // Payload length byte
#define FRAME_OFFSET_SOURCE  3                         // Source node ID
#define FRAME_OFFSET_PAYLOAD 4                         // Start of payload
#define FRAME_OFFSET_CRC     (OPTICAL_FRAME_SIZE - 2)  // CRC-16 location (bytes 126-127)

// ============================================================================
// Data Structures
// ============================================================================

/**
 * @brief Per-channel status information
 */
typedef struct {
    uint16_t status_reg;          // Last read status register value
    bool rx_fifo_empty;           // True if RX FIFO has no data
    bool rx_fifo_full;            // True if RX FIFO is full (data loss warning)
    bool tx_busy;                 // True if transmitter is currently busy
    uint32_t frames_transmitted;  // Total frames sent on this channel
    uint32_t frames_received;     // Total valid frames received on this channel
    uint32_t rx_overruns;         // Number of RX FIFO overruns detected
    uint32_t crc_errors;          // Number of CRC validation failures
} optical_channel_status_t;

/**
 * @brief Receive buffer for assembling frames
 */
typedef struct {
    uint8_t buffer[OPTICAL_FRAME_SIZE * 2];  // Double buffer for safety
    uint16_t write_index;                    // Current write position
    bool frame_ready;                        // True when complete frame assembled
    uint8_t source_node;                     // Source node ID of received frame
} optical_rx_buffer_t;

/**
 * @brief Main optical fiber driver state
 */
typedef struct {
    uint8_t node_id;                                         // This controller's node ID (0-5)
    uint32_t baud_rate;                                      // Configured baud rate
    optical_channel_status_t channel[OPTICAL_NUM_CHANNELS];  // Per-channel status (index 0-6 = channel 1-7)
    optical_rx_buffer_t rx_buffer[OPTICAL_NUM_CHANNELS];     // Receive buffers (index 0-6 = channel 1-7)
    uint32_t poll_count;                                     // Total number of poll cycles executed
} optical_fiber_t;

// ============================================================================
// Initialization Functions
// ============================================================================

/**
 * @brief Initialize FPGA hardware (reconfiguration and reset)
 *
 * This function MUST be called ONCE during system startup, BEFORE OpticalFiber_Init().
 * It performs the following sequence:
 * 1. Wait for power stabilization
 * 2. Trigger FPGA reconfiguration via PROGRAMB pin (PG15)
 * 3. Wait for FPGA configuration complete via DONE pin (PD2)
 * 4. Execute internal FPGA soft reset via RST pin (PG11)
 *
 * Hardware pins used:
 * - PG15: FPGA_PROGRAMB (output) - Triggers FPGA reconfiguration from flash
 * - PD2:  FPGA_DONE (input) - Indicates FPGA configuration complete
 * - PG11: FPGA_RST (output) - Internal FPGA soft reset
 *
 * @note FSMC and GPIO must already be initialized (by MX_FSMC_Init and MX_GPIO_Init)
 * @note This function includes delays for FPGA timing requirements
 * @warning If FPGA does not configure properly, one retry is attempted
 */
void OpticalFiber_InitHardware(void);

/**
 * @brief Initialize optical fiber communication system
 *
 * Configures the FPGA optical transceiver with specified baud rate and node ID.
 * Must be called once during system initialization, after FSMC is configured.
 *
 * @param fiber Pointer to optical fiber state structure (must be persistent)
 * @param baud_rate Desired baud rate in bps (e.g., 115200, 230400, 500000)
 * @param node_id This controller's node ID (0-5, typically 0=line, 1=machine)
 *
 * @note Typical baud rates: 115200, 230400, 460800 bps
 * @note FSMC must be initialized before calling this function
 * @note OpticalFiber_InitHardware() must be called before this function
 */
void OpticalFiber_Init(optical_fiber_t* fiber, uint32_t baud_rate, uint8_t node_id);

/**
 * @brief Reset optical fiber driver to initial state
 *
 * Clears all buffers, statistics, and state. Does not reconfigure hardware.
 *
 * @param fiber Pointer to optical fiber state structure
 */
void OpticalFiber_Reset(optical_fiber_t* fiber);

// ============================================================================
// Transmit Functions
// ============================================================================

/**
 * @brief Transmit raw data on specific channel
 *
 * Low-level transmit function. Writes data directly to TX FIFO without framing.
 * Most applications should use OpticalFiber_TransmitFrame() instead.
 *
 * @param fiber Pointer to optical fiber state structure
 * @param channel Channel number (0=broadcast, 1-7=specific channel)
 * @param data Pointer to data buffer
 * @param length Number of bytes to transmit (max OPTICAL_FRAME_SIZE)
 * @return true if transmission started successfully, false if channel busy
 */
bool OpticalFiber_Transmit(optical_fiber_t* fiber, uint8_t channel, const uint8_t* data, uint16_t length);

/**
 * @brief Transmit frame on specific channel with automatic framing
 *
 * Wraps payload in standard frame format with header, length, source, and CRC.
 * This is the recommended function for sending application data.
 *
 * Frame format (128 bytes total):
 *   [0-1]:   Header (0x68, 0x68)
 *   [2]:     Payload length (0-120)
 *   [3]:     Source node ID
 *   [4-123]: Payload data (padded with zeros)
 *   [124]:   CRC-16 low byte
 *   [125]:   CRC-16 high byte
 *   [126-127]: Padding (0x00)
 *
 * @param fiber Pointer to optical fiber state structure
 * @param channel Channel number (0=broadcast, 1-7=specific channel)
 * @param payload Pointer to payload data
 * @param payload_length Payload size in bytes (max OPTICAL_MAX_PAYLOAD)
 * @return true if frame transmitted successfully, false on error
 */
bool OpticalFiber_TransmitFrame(
    optical_fiber_t* fiber, uint8_t channel, const uint8_t* payload, uint8_t payload_length);

// ============================================================================
// Receive Functions
// ============================================================================

/**
 * @brief Poll all channels for received data
 *
 * Reads RX FIFOs from all channels and assembles received frames.
 * Must be called periodically (e.g., every 1ms in Task 2).
 *
 * This function:
 *   1. Checks each channel's RX FIFO status
 *   2. Reads available bytes from non-empty FIFOs
 *   3. Assembles complete frames in RX buffers
 *   4. Validates frame headers and CRC
 *   5. Sets frame_ready flag when valid frame received
 *
 * @param fiber Pointer to optical fiber state structure
 *
 * @note Non-blocking: returns immediately even if no data available
 * @note Call frequency affects maximum data rate and latency
 */
void OpticalFiber_Poll(optical_fiber_t* fiber);

/**
 * @brief Check if complete frame is ready on specific channel
 *
 * @param fiber Pointer to optical fiber state structure
 * @param channel Channel number (1-7)
 * @return true if complete frame is available for reading
 */
bool OpticalFiber_IsFrameReady(const optical_fiber_t* fiber, uint8_t channel);

/**
 * @brief Get received frame from specific channel
 *
 * Copies complete frame to output buffer and marks buffer as consumed.
 *
 * @param fiber Pointer to optical fiber state structure
 * @param channel Channel number (1-7)
 * @param frame_out Buffer to store received frame (must be >= OPTICAL_FRAME_SIZE)
 * @return Number of bytes copied (OPTICAL_FRAME_SIZE), or 0 if no frame available
 *
 * @note Clears frame_ready flag after reading
 */
uint16_t OpticalFiber_GetFrame(optical_fiber_t* fiber, uint8_t channel, uint8_t* frame_out);

/**
 * @brief Extract payload from received frame
 *
 * Extracts user data from a complete frame, validates header and CRC.
 *
 * @param frame Pointer to complete frame (128 bytes)
 * @param payload_out Buffer to store extracted payload
 * @param max_payload_size Maximum payload buffer size
 * @param source_node_out Pointer to store source node ID (can be NULL)
 * @return Number of payload bytes extracted (0-120), or 0 on validation error
 */
uint16_t OpticalFiber_ExtractPayload(
    const uint8_t* frame, uint8_t* payload_out, uint16_t max_payload_size, uint8_t* source_node_out);

// ============================================================================
// Status Functions
// ============================================================================

/**
 * @brief Read and update status register for specific channel
 *
 * @param fiber Pointer to optical fiber state structure
 * @param channel Channel number (1-7)
 */
void OpticalFiber_UpdateChannelStatus(optical_fiber_t* fiber, uint8_t channel);

/**
 * @brief Get channel status information
 *
 * @param fiber Pointer to optical fiber state structure
 * @param channel Channel number (1-7)
 * @return Pointer to channel status structure, or NULL if invalid channel
 */
const optical_channel_status_t* OpticalFiber_GetChannelStatus(const optical_fiber_t* fiber, uint8_t channel);

// ============================================================================
// Utility Functions
// ============================================================================

/**
 * @brief Calculate CRC-16 (Modbus) for data buffer
 *
 * Uses standard Modbus CRC-16 polynomial: 0xA001
 *
 * @param data Pointer to data buffer
 * @param length Number of bytes to include in CRC
 * @return CRC-16 value (16-bit)
 */
uint16_t OpticalFiber_CRC16(const uint8_t* data, uint16_t length);

/**
 * @brief Verify frame CRC
 *
 * Calculates CRC for frame bytes [0-125] and compares with stored CRC at [126-127].
 *
 * @param frame Pointer to complete frame (128 bytes)
 * @return true if calculated CRC matches stored CRC
 */
bool OpticalFiber_VerifyFrameCRC(const uint8_t* frame);

// ============================================================================
// Helper Macros
// ============================================================================

// Convert channel number (1-7) to array index (0-6)
#define CHANNEL_TO_INDEX(ch) ((ch) - 1)

// Convert array index (0-6) to channel number (1-7)
#define INDEX_TO_CHANNEL(idx) ((idx) + 1)

// Validate channel number
#define IS_VALID_CHANNEL(ch) ((ch) >= 1 && (ch) <= OPTICAL_NUM_CHANNELS)
