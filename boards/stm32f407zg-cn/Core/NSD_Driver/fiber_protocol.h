/**
 * Simplified protocol optimized for 1ms control loops at 691,200 baud.
 * Features:
 * - All data as uint32 words (4 bytes each)
 * - Float ↔ uint32 conversion helpers
 * - CRC16 error detection
 * - Command-based frame format
 * - 10 uint32 payload capacity per 1ms
 *
 * Protocol Frame Format:
 * [START] [COMMAND] [LENGTH] [PAYLOAD...] [CRC16]
 *   1B      4B        1B      N×4B         2B
 *
 * Performance @ 691,200 baud:
 * - Throughput: ~55 bytes/ms (practical)
 * - Max payload: 10 uint32 words (40 bytes)
 * - Frame overhead: 8 bytes
 * - Total frame: 48 bytes → 0.69ms transmission time
 * - Margin: 0.31ms for RX + processing
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FIBER_PROTOCOL_H
#define __FIBER_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>

#include "fiber_optic.h"

/* Exported defines ----------------------------------------------------------*/

/**
 * @brief Protocol configuration for 691,200 baud @ 1ms task rate
 */
#define FIBER_PROTO_START_BYTE        0xAA                                 ///< Frame start marker
#define FIBER_PROTO_MAX_PAYLOAD_WORDS 4                                    ///< Maximum uint32 words in payload
#define FIBER_PROTO_MAX_PAYLOAD_BYTES (FIBER_PROTO_MAX_PAYLOAD_WORDS * 4)  ///< 16 bytes
#define FIBER_PROTO_HEADER_SIZE       6                                    ///< START + COMMAND + LENGTH
#define FIBER_PROTO_CRC_SIZE          2                                    ///< CRC16 checksum size
#define FIBER_PROTO_MAX_FRAME_SIZE \
    (FIBER_PROTO_HEADER_SIZE + FIBER_PROTO_MAX_PAYLOAD_BYTES + FIBER_PROTO_CRC_SIZE)  ///< 26 bytes

/**
 * @brief Recommended baud rate (from legacy code)
 */
#define FIBER_PROTO_BAUD_RATE 691200  ///< 691,200 baud for 1ms control

/**
 * @brief RX buffer timeout (ms) - discard incomplete frames after this duration
 * @note Set to 5ms to allow reception across multiple 1ms cycles while preventing
 *       stale data accumulation. Normal frames arrive within 1-2ms.
 */
#define FIBER_PROTO_RX_TIMEOUT_MS 5

/**
 * @brief Command Word Bit Layout (32 bits total)
 *
 * Bits 31-30: Heartbeat Counter (2 bits) - Increments each transmission (0-3)
 * Bits 29-28: Board Side (2 bits) - 00=Reserved, 01=Machine-Side, 10=Grid-Side, 11=Reserved
 * Bits 27-26: Trip Status (2 bits) - 00=Normal, 01=Warning, 10=Tripped, 11=Fault
 * Bits 25-22: User Command (4 bits) - 0000=None, 0001=Start, 0010=Stop, 0011=Reset, etc.
 * Bits 21-18: State Machine State (4 bits) - Current state (0-15)
 * Bits 17-0:  Reserved / Payload Type / Sequence (18 bits)
 *
 * Example:
 *   Heartbeat=1, Grid-Side, Normal, Start Cmd, State=2, Type=0x01
 *   = 0b01_10_00_0001_0010_00_0000_0000_0000_0001
 */

// Bit positions
#define CMD_HEARTBEAT_POS     30  ///< Heartbeat counter position
#define CMD_BOARD_SIDE_POS    28  ///< Board side position
#define CMD_TRIP_STATUS_POS   26  ///< Trip status position
#define CMD_USER_COMMAND_POS  22  ///< User command position
#define CMD_STATE_MACHINE_POS 18  ///< State machine position

// Bit masks
#define CMD_HEARTBEAT_MASK     0xC0000000  ///< Bits 31-30
#define CMD_BOARD_SIDE_MASK    0x30000000  ///< Bits 29-28
#define CMD_TRIP_STATUS_MASK   0x0C000000  ///< Bits 27-26
#define CMD_USER_COMMAND_MASK  0x03C00000  ///< Bits 25-22
#define CMD_STATE_MACHINE_MASK 0x003C0000  ///< Bits 21-18
#define CMD_PAYLOAD_TYPE_MASK  0x0003FFFF  ///< Bits 17-0 (reserved/type/sequence)

/**
 * @brief Board Side identifiers (2 bits)
 */
#define BOARD_SIDE_RESERVED_0 0x00  ///< Reserved
#define BOARD_SIDE_MACHINE    0x01  ///< Machine-side converter
#define BOARD_SIDE_GRID       0x02  ///< Grid-side converter
#define BOARD_SIDE_RESERVED_3 0x03  ///< Reserved

/**
 * @brief Trip Status values (2 bits)
 */
#define TRIP_STATUS_NORMAL  0x00  ///< Normal operation
#define TRIP_STATUS_WARNING 0x01  ///< Warning condition
#define TRIP_STATUS_TRIPPED 0x02  ///< Tripped/stopped
#define TRIP_STATUS_FAULT   0x03  ///< Fault condition

/**
 * @brief User Command values (4 bits)
 */
#define USER_CMD_NONE           0x00  ///< No command
#define USER_CMD_START          0x01  ///< Start operation
#define USER_CMD_STOP           0x02  ///< Stop operation
#define USER_CMD_RESET          0x03  ///< Reset system
#define USER_CMD_ENABLE         0x04  ///< Enable
#define USER_CMD_DISABLE        0x05  ///< Disable
#define USER_CMD_CALIBRATE      0x06  ///< Start calibration
#define USER_CMD_EMERGENCY_STOP 0x0F  ///< Emergency stop

/**
 * @brief State Machine States (4 bits) - Define according to your application
 */
#define STATE_IDLE      0x00  ///< Idle state
#define STATE_INIT      0x01  ///< Initialization
#define STATE_READY     0x02  ///< Ready to operate
#define STATE_STARTING  0x03  ///< Starting sequence
#define STATE_RUNNING   0x04  ///< Normal operation
#define STATE_STOPPING  0x05  ///< Stopping sequence
#define STATE_FAULT     0x0E  ///< Fault state
#define STATE_EMERGENCY 0x0F  ///< Emergency state

/**
 * @brief Payload Type identifiers (lower 18 bits)
 *
 * @note These are EXAMPLE identifiers only. The actual payload content and type
 *       definitions are application-specific and should be defined in your control
 *       code. Machine-side and Grid-side boards determine their own payload formats.
 *
 *       Common approach:
 *       - Each board knows what it sends (TX payload format)
 *       - Each board knows what it expects to receive (RX payload format)
 *       - Payload type field can be used for multi-packet protocols or left as
 *         a constant if only one packet type is used per direction
 */
#define PAYLOAD_TYPE_CONTROL_DATA     0x0001  ///< Example: Real-time control data
#define PAYLOAD_TYPE_MEASUREMENT_DATA 0x0002  ///< Example: Measurement feedback
#define PAYLOAD_TYPE_PARAMETER_SET    0x0010  ///< Example: Set parameter
#define PAYLOAD_TYPE_STATUS           0x0020  ///< Example: Status information

/**
 * @brief Command word construction and extraction macros
 */

// Build command word from individual fields
#define CMD_BUILD(heartbeat, board_side, trip_status, user_cmd, state, payload_type)                                   \
    (((uint32_t)((heartbeat) & 0x03) << CMD_HEARTBEAT_POS) | ((uint32_t)((board_side) & 0x03) << CMD_BOARD_SIDE_POS) | \
        ((uint32_t)((trip_status) & 0x03) << CMD_TRIP_STATUS_POS) |                                                    \
        ((uint32_t)((user_cmd) & 0x0F) << CMD_USER_COMMAND_POS) |                                                      \
        ((uint32_t)((state) & 0x0F) << CMD_STATE_MACHINE_POS) | ((uint32_t)((payload_type) & 0x3FFFF)))

// Extract individual fields from command word
#define CMD_GET_HEARTBEAT(cmd)     (((cmd) & CMD_HEARTBEAT_MASK) >> CMD_HEARTBEAT_POS)
#define CMD_GET_BOARD_SIDE(cmd)    (((cmd) & CMD_BOARD_SIDE_MASK) >> CMD_BOARD_SIDE_POS)
#define CMD_GET_TRIP_STATUS(cmd)   (((cmd) & CMD_TRIP_STATUS_MASK) >> CMD_TRIP_STATUS_POS)
#define CMD_GET_USER_COMMAND(cmd)  (((cmd) & CMD_USER_COMMAND_MASK) >> CMD_USER_COMMAND_POS)
#define CMD_GET_STATE_MACHINE(cmd) (((cmd) & CMD_STATE_MACHINE_MASK) >> CMD_STATE_MACHINE_POS)
#define CMD_GET_PAYLOAD_TYPE(cmd)  ((cmd) & CMD_PAYLOAD_TYPE_MASK)

// Legacy compatibility (extract payload type as "command type")
#define CMD_GET_TYPE(cmd) CMD_GET_PAYLOAD_TYPE(cmd)

/* Exported types ------------------------------------------------------------*/

/**
 * @brief Protocol error codes
 */
typedef enum {
    FIBER_PROTO_OK = 0,           ///< Success
    FIBER_PROTO_ERR_CRC = 1,      ///< CRC mismatch
    FIBER_PROTO_ERR_LENGTH = 2,   ///< Invalid length
    FIBER_PROTO_ERR_TIMEOUT = 3,  ///< No data available
    FIBER_PROTO_ERR_BUFFER = 4,   ///< Buffer error
    FIBER_PROTO_ERR_PARAM = 5     ///< Invalid parameter
} fiber_proto_error_t;

/**
 * @brief Protocol statistics
 */
typedef struct {
    uint32_t tx_count;       ///< Total frames transmitted
    uint32_t rx_count;       ///< Total frames received
    uint32_t crc_errors;     ///< CRC error count
    uint32_t length_errors;  ///< Length error count
    uint32_t timeout_count;  ///< Timeout count
    uint16_t last_tx_seq;    ///< Last transmitted sequence
    uint16_t last_rx_seq;    ///< Last received sequence
} fiber_proto_stats_t;

/**
 * @brief Protocol instance with buffered RX for reliable 1ms operation
 */
typedef struct {
    fiber_optic_instance_t* fiber_instance;  ///< Hardware fiber-optic instance
    fiber_channel_t channel;                 ///< Channel number
    uint16_t tx_seq;                         ///< TX sequence counter
    uint16_t rx_seq;                         ///< RX sequence counter
    fiber_proto_stats_t stats;               ///< Protocol statistics
    bool initialized;                        ///< Initialization flag

    // RX buffering for reliable frame reception across multiple 1ms cycles
    uint8_t rx_buffer[FIBER_PROTO_MAX_FRAME_SIZE];  ///< Persistent receive buffer
    uint16_t rx_buffer_len;                         ///< Current bytes in RX buffer
    uint32_t last_rx_time_ms;                       ///< Timestamp of last received byte (for timeout)
} fiber_proto_instance_t;

/* Exported functions --------------------------------------------------------*/

/**
 * @brief  Initialize protocol instance
 * @param  proto: Protocol instance pointer
 * @param  fiber: Fiber-optic hardware instance
 * @param  channel: Channel to use (FIBER_CH1 through FIBER_CH6)
 * @retval true if successful, false on error
 */
bool FiberProto_Init(fiber_proto_instance_t* proto, fiber_optic_instance_t* fiber, fiber_channel_t channel);

/**
 * @brief  Send packet with uint32 payload
 * @param  proto: Protocol instance
 * @param  command: 32-bit command word
 * @param  payload: Array of uint32 values (can be NULL if length=0)
 * @param  length: Number of uint32 words (0 to FIBER_PROTO_MAX_PAYLOAD_WORDS)
 * @retval FIBER_PROTO_OK or error code
 *
 * @note   Frame structure: [START][COMMAND][LENGTH][PAYLOAD][CRC16]
 *         Total frame size: 8 + (length × 4) bytes
 *         At 691,200 baud: 10 words = 48 bytes = 0.69ms
 */
fiber_proto_error_t FiberProto_SendPacket(
    fiber_proto_instance_t* proto, uint32_t command, const uint32_t* payload, uint8_t length);

/**
 * @brief  Receive packet with uint32 payload
 * @param  proto: Protocol instance
 * @param  command: Pointer to store command word
 * @param  payload: Array to store uint32 values (min size: FIBER_PROTO_MAX_PAYLOAD_WORDS)
 * @param  length: Pointer to store number of uint32 words received
 * @retval FIBER_PROTO_OK or error code
 *
 * @note   Non-blocking. Returns FIBER_PROTO_ERR_TIMEOUT if no complete frame available.
 *         Buffers partial frames across multiple calls for reliable reception.
 *         CRC is automatically verified. Invalid frames are discarded.
 *
 *         Call this every 1ms from your control task. If a frame spans multiple
 *         cycles, it will be accumulated automatically until complete.
 */
fiber_proto_error_t FiberProto_ReceivePacket(
    fiber_proto_instance_t* proto, uint32_t* command, uint32_t* payload, uint8_t* length);

/**
 * @brief  Convert float to uint32 (preserves IEEE 754 bit pattern)
 * @param  value: Float value to convert
 * @retval uint32_t representation
 *
 * @note   This is a type-punning operation. The float is reinterpreted
 *         as uint32 without conversion. Reversible with FiberProto_Uint32ToFloat().
 */
static inline uint32_t FiberProto_FloatToUint32(float value)
{
    union {
        float f;
        uint32_t u;
    } converter;
    converter.f = value;
    return converter.u;
}

/**
 * @brief  Convert uint32 to float (preserves IEEE 754 bit pattern)
 * @param  value: uint32 value to convert
 * @retval float representation
 *
 * @note   This is a type-punning operation. The uint32 is reinterpreted
 *         as float without conversion. Reversible with FiberProto_FloatToUint32().
 */
static inline float FiberProto_Uint32ToFloat(uint32_t value)
{
    union {
        uint32_t u;
        float f;
    } converter;
    converter.u = value;
    return converter.f;
}

/**
 * @brief  Send packet with float payload (convenience function)
 * @param  proto: Protocol instance
 * @param  command: 32-bit command word
 * @param  values: Array of float values
 * @param  count: Number of float values (0 to FIBER_PROTO_MAX_PAYLOAD_WORDS)
 * @retval FIBER_PROTO_OK or error code
 *
 * @note   Internally converts floats to uint32 using FiberProto_FloatToUint32()
 */
fiber_proto_error_t FiberProto_SendFloats(
    fiber_proto_instance_t* proto, uint32_t command, const float* values, uint8_t count);

/**
 * @brief  Parse uint32 payload as floats (convenience function)
 * @param  payload: uint32 payload array
 * @param  payload_length: Number of uint32 words in payload
 * @param  values: Array to store float values (min size: payload_length)
 * @param  count: Number of floats to extract (must be ≤ payload_length)
 * @retval true if successful, false on error
 *
 * @note   Internally converts uint32 to floats using FiberProto_Uint32ToFloat()
 */
bool FiberProto_ParseFloats(const uint32_t* payload, uint8_t payload_length, float* values, uint8_t count);

/**
 * @brief  Get protocol statistics
 * @param  proto: Protocol instance
 * @retval Pointer to statistics structure (NULL on error)
 */
const fiber_proto_stats_t* FiberProto_GetStats(const fiber_proto_instance_t* proto);

/**
 * @brief  Reset protocol statistics to zero
 * @param  proto: Protocol instance
 */
void FiberProto_ResetStats(fiber_proto_instance_t* proto);

/**
 * @brief  Calculate CRC16-CCITT checksum
 * @param  data: Data buffer pointer
 * @param  length: Data length in bytes
 * @retval CRC16 checksum
 *
 * @note   Polynomial: 0x1021, Initial: 0xFFFF
 */
uint16_t FiberProto_CalculateCRC16(const uint8_t* data, uint16_t length);

#ifdef __cplusplus
}
#endif

#endif /* __FIBER_PROTOCOL_H */