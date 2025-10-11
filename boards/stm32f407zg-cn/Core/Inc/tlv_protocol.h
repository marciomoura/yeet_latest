/**
 ******************************************************************************
 * @file    tlv_protocol.h
 * @brief   Tag-Length-Value (TLV) Protocol for Network Communication
 * @note    Simple, efficient protocol for telemetry and parameter exchange
 ******************************************************************************
 * @attention
 *
 * TLV Protocol Format:
 *
 * [Tag (1 byte)] [Length (2 bytes)] [Value (N bytes)]
 *
 * - Tag: Identifies the data type (e.g., ADC channel, setpoint, status)
 * - Length: Number of bytes in Value field (big-endian uint16)
 * - Value: Actual data (format depends on tag)
 *
 * Multiple TLV entries can be concatenated in a single packet.
 *
 * Example Usage:
 * - UDP Telemetry: Pack multiple sensor readings into TLV packet, send @ 1-10ms
 * - TCP Parameters: Receive parameter updates in TLV format, process @ 1s
 *
 ******************************************************************************
 */

#ifndef TLV_PROTOCOL_H
#define TLV_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/* ==================== Protocol Constants ==================== */

#define TLV_HEADER_SIZE     3     ///< Tag (1) + Length (2)
#define TLV_MAX_VALUE_SIZE  512   ///< Maximum value size per TLV entry
#define TLV_MAX_PACKET_SIZE 1400  ///< Maximum total packet size (UDP safe)

/* ==================== Standard Tag Definitions ==================== */

// System Status Tags (0x00 - 0x0F)
#define TLV_TAG_SYSTEM_TIME   0x00  ///< uint32_t: System uptime (ms)
#define TLV_TAG_SYSTEM_STATUS 0x01  ///< uint8_t: System status byte
#define TLV_TAG_ERROR_CODE    0x02  ///< uint16_t: Last error code
#define TLV_TAG_NODE_ID       0x03  ///< uint8_t: Node ID

// Measurement Tags (0x10 - 0x4F) - 64 channels total
// Format: Can be int16_t, uint16_t, float, etc. depending on application
#define TLV_TAG_MEASUREMENT_0  0x10  ///< Measurement channel 0
#define TLV_TAG_MEASUREMENT_1  0x11  ///< Measurement channel 1
#define TLV_TAG_MEASUREMENT_2  0x12  ///< Measurement channel 2
#define TLV_TAG_MEASUREMENT_3  0x13  ///< Measurement channel 3
#define TLV_TAG_MEASUREMENT_4  0x14  ///< Measurement channel 4
#define TLV_TAG_MEASUREMENT_5  0x15  ///< Measurement channel 5
#define TLV_TAG_MEASUREMENT_6  0x16  ///< Measurement channel 6
#define TLV_TAG_MEASUREMENT_7  0x17  ///< Measurement channel 7
#define TLV_TAG_MEASUREMENT_8  0x18  ///< Measurement channel 8
#define TLV_TAG_MEASUREMENT_9  0x19  ///< Measurement channel 9
#define TLV_TAG_MEASUREMENT_10 0x1A  ///< Measurement channel 10
#define TLV_TAG_MEASUREMENT_11 0x1B  ///< Measurement channel 11
#define TLV_TAG_MEASUREMENT_12 0x1C  ///< Measurement channel 12
#define TLV_TAG_MEASUREMENT_13 0x1D  ///< Measurement channel 13
#define TLV_TAG_MEASUREMENT_14 0x1E  ///< Measurement channel 14
#define TLV_TAG_MEASUREMENT_15 0x1F  ///< Measurement channel 15
#define TLV_TAG_MEASUREMENT_16 0x20  ///< Measurement channel 16
#define TLV_TAG_MEASUREMENT_17 0x21  ///< Measurement channel 17
#define TLV_TAG_MEASUREMENT_18 0x22  ///< Measurement channel 18
#define TLV_TAG_MEASUREMENT_19 0x23  ///< Measurement channel 19
#define TLV_TAG_MEASUREMENT_20 0x24  ///< Measurement channel 20
#define TLV_TAG_MEASUREMENT_21 0x25  ///< Measurement channel 21
#define TLV_TAG_MEASUREMENT_22 0x26  ///< Measurement channel 22
#define TLV_TAG_MEASUREMENT_23 0x27  ///< Measurement channel 23
#define TLV_TAG_MEASUREMENT_24 0x28  ///< Measurement channel 24
#define TLV_TAG_MEASUREMENT_25 0x29  ///< Measurement channel 25
#define TLV_TAG_MEASUREMENT_26 0x2A  ///< Measurement channel 26
#define TLV_TAG_MEASUREMENT_27 0x2B  ///< Measurement channel 27
#define TLV_TAG_MEASUREMENT_28 0x2C  ///< Measurement channel 28
#define TLV_TAG_MEASUREMENT_29 0x2D  ///< Measurement channel 29
#define TLV_TAG_MEASUREMENT_30 0x2E  ///< Measurement channel 30
#define TLV_TAG_MEASUREMENT_31 0x2F  ///< Measurement channel 31
#define TLV_TAG_MEASUREMENT_32 0x30  ///< Measurement channel 32
#define TLV_TAG_MEASUREMENT_33 0x31  ///< Measurement channel 33
#define TLV_TAG_MEASUREMENT_34 0x32  ///< Measurement channel 34
#define TLV_TAG_MEASUREMENT_35 0x33  ///< Measurement channel 35
#define TLV_TAG_MEASUREMENT_36 0x34  ///< Measurement channel 36
#define TLV_TAG_MEASUREMENT_37 0x35  ///< Measurement channel 37
#define TLV_TAG_MEASUREMENT_38 0x36  ///< Measurement channel 38
#define TLV_TAG_MEASUREMENT_39 0x37  ///< Measurement channel 39
#define TLV_TAG_MEASUREMENT_40 0x38  ///< Measurement channel 40
#define TLV_TAG_MEASUREMENT_41 0x39  ///< Measurement channel 41
#define TLV_TAG_MEASUREMENT_42 0x3A  ///< Measurement channel 42
#define TLV_TAG_MEASUREMENT_43 0x3B  ///< Measurement channel 43
#define TLV_TAG_MEASUREMENT_44 0x3C  ///< Measurement channel 44
#define TLV_TAG_MEASUREMENT_45 0x3D  ///< Measurement channel 45
#define TLV_TAG_MEASUREMENT_46 0x3E  ///< Measurement channel 46
#define TLV_TAG_MEASUREMENT_47 0x3F  ///< Measurement channel 47
#define TLV_TAG_MEASUREMENT_48 0x40  ///< Measurement channel 48
#define TLV_TAG_MEASUREMENT_49 0x41  ///< Measurement channel 49
#define TLV_TAG_MEASUREMENT_50 0x42  ///< Measurement channel 50
#define TLV_TAG_MEASUREMENT_51 0x43  ///< Measurement channel 51
#define TLV_TAG_MEASUREMENT_52 0x44  ///< Measurement channel 52
#define TLV_TAG_MEASUREMENT_53 0x45  ///< Measurement channel 53
#define TLV_TAG_MEASUREMENT_54 0x46  ///< Measurement channel 54
#define TLV_TAG_MEASUREMENT_55 0x47  ///< Measurement channel 55
#define TLV_TAG_MEASUREMENT_56 0x48  ///< Measurement channel 56
#define TLV_TAG_MEASUREMENT_57 0x49  ///< Measurement channel 57
#define TLV_TAG_MEASUREMENT_58 0x4A  ///< Measurement channel 58
#define TLV_TAG_MEASUREMENT_59 0x4B  ///< Measurement channel 59
#define TLV_TAG_MEASUREMENT_60 0x4C  ///< Measurement channel 60
#define TLV_TAG_MEASUREMENT_61 0x4D  ///< Measurement channel 61
#define TLV_TAG_MEASUREMENT_62 0x4E  ///< Measurement channel 62
#define TLV_TAG_MEASUREMENT_63 0x4F  ///< Measurement channel 63

// Digital I/O Tags (0x50 - 0x5F)
#define TLV_TAG_DI_INPUTS    0x50  ///< uint8_t[N]: Digital inputs (bit-packed)
#define TLV_TAG_DO_OUTPUTS   0x51  ///< uint8_t[N]: Digital outputs (bit-packed)
#define TLV_TAG_FPGA_INPUTS  0x52  ///< uint8_t[6]: FPGA digital inputs (48 bits)
#define TLV_TAG_FPGA_OUTPUTS 0x53  ///< uint8_t[6]: FPGA digital outputs (48 bits)
#define TLV_TAG_LED_STATUS   0x54  ///< uint8_t: LED status bits

// GPIO Tags (0x60 - 0x7F) - Application-specific GPIO definitions
#define TLV_TAG_GPIO_0  0x60  ///< GPIO channel 0 (define usage in application)
#define TLV_TAG_GPIO_1  0x61  ///< GPIO channel 1 (define usage in application)
#define TLV_TAG_GPIO_2  0x62  ///< GPIO channel 2 (define usage in application)
#define TLV_TAG_GPIO_3  0x63  ///< GPIO channel 3 (define usage in application)
#define TLV_TAG_GPIO_4  0x64  ///< GPIO channel 4 (define usage in application)
#define TLV_TAG_GPIO_5  0x65  ///< GPIO channel 5 (define usage in application)
#define TLV_TAG_GPIO_6  0x66  ///< GPIO channel 6 (define usage in application)
#define TLV_TAG_GPIO_7  0x67  ///< GPIO channel 7 (define usage in application)
#define TLV_TAG_GPIO_8  0x68  ///< GPIO channel 8 (define usage in application)
#define TLV_TAG_GPIO_9  0x69  ///< GPIO channel 9 (define usage in application)
#define TLV_TAG_GPIO_10 0x6A  ///< GPIO channel 10 (define usage in application)
#define TLV_TAG_GPIO_11 0x6B  ///< GPIO channel 11 (define usage in application)
#define TLV_TAG_GPIO_12 0x6C  ///< GPIO channel 12 (define usage in application)
#define TLV_TAG_GPIO_13 0x6D  ///< GPIO channel 13 (define usage in application)
#define TLV_TAG_GPIO_14 0x6E  ///< GPIO channel 14 (define usage in application)
#define TLV_TAG_GPIO_15 0x6F  ///< GPIO channel 15 (define usage in application)
#define TLV_TAG_GPIO_16 0x70  ///< GPIO channel 16 (define usage in application)
#define TLV_TAG_GPIO_17 0x71  ///< GPIO channel 17 (define usage in application)
#define TLV_TAG_GPIO_18 0x72  ///< GPIO channel 18 (define usage in application)
#define TLV_TAG_GPIO_19 0x73  ///< GPIO channel 19 (define usage in application)
#define TLV_TAG_GPIO_20 0x74  ///< GPIO channel 20 (define usage in application)
#define TLV_TAG_GPIO_21 0x75  ///< GPIO channel 21 (define usage in application)
#define TLV_TAG_GPIO_22 0x76  ///< GPIO channel 22 (define usage in application)
#define TLV_TAG_GPIO_23 0x77  ///< GPIO channel 23 (define usage in application)
#define TLV_TAG_GPIO_24 0x78  ///< GPIO channel 24 (define usage in application)
#define TLV_TAG_GPIO_25 0x79  ///< GPIO channel 25 (define usage in application)
#define TLV_TAG_GPIO_26 0x7A  ///< GPIO channel 26 (define usage in application)
#define TLV_TAG_GPIO_27 0x7B  ///< GPIO channel 27 (define usage in application)
#define TLV_TAG_GPIO_28 0x7C  ///< GPIO channel 28 (define usage in application)
#define TLV_TAG_GPIO_29 0x7D  ///< GPIO channel 29 (define usage in application)
#define TLV_TAG_GPIO_30 0x7E  ///< GPIO channel 30 (define usage in application)
#define TLV_TAG_GPIO_31 0x7F  ///< GPIO channel 31 (define usage in application)

// Parameter Tags (0x80 - 0xBF) - Application-specific parameters
#define TLV_TAG_PARAMETER_0  0x80  ///< Parameter 0 (define usage in application)
#define TLV_TAG_PARAMETER_1  0x81  ///< Parameter 1 (define usage in application)
#define TLV_TAG_PARAMETER_2  0x82  ///< Parameter 2 (define usage in application)
#define TLV_TAG_PARAMETER_3  0x83  ///< Parameter 3 (define usage in application)
#define TLV_TAG_PARAMETER_4  0x84  ///< Parameter 4 (define usage in application)
#define TLV_TAG_PARAMETER_5  0x85  ///< Parameter 5 (define usage in application)
#define TLV_TAG_PARAMETER_6  0x86  ///< Parameter 6 (define usage in application)
#define TLV_TAG_PARAMETER_7  0x87  ///< Parameter 7 (define usage in application)
#define TLV_TAG_PARAMETER_8  0x88  ///< Parameter 8 (define usage in application)
#define TLV_TAG_PARAMETER_9  0x89  ///< Parameter 9 (define usage in application)
#define TLV_TAG_PARAMETER_10 0x8A  ///< Parameter 10 (define usage in application)
#define TLV_TAG_PARAMETER_11 0x8B  ///< Parameter 11 (define usage in application)
#define TLV_TAG_PARAMETER_12 0x8C  ///< Parameter 12 (define usage in application)
#define TLV_TAG_PARAMETER_13 0x8D  ///< Parameter 13 (define usage in application)
#define TLV_TAG_PARAMETER_14 0x8E  ///< Parameter 14 (define usage in application)
#define TLV_TAG_PARAMETER_15 0x8F  ///< Parameter 15 (define usage in application)
#define TLV_TAG_PARAMETER_16 0x90  ///< Parameter 16 (define usage in application)
#define TLV_TAG_PARAMETER_17 0x91  ///< Parameter 17 (define usage in application)
#define TLV_TAG_PARAMETER_18 0x92  ///< Parameter 18 (define usage in application)
#define TLV_TAG_PARAMETER_19 0x93  ///< Parameter 19 (define usage in application)
#define TLV_TAG_PARAMETER_20 0x94  ///< Parameter 20 (define usage in application)
#define TLV_TAG_PARAMETER_21 0x95  ///< Parameter 21 (define usage in application)
#define TLV_TAG_PARAMETER_22 0x96  ///< Parameter 22 (define usage in application)
#define TLV_TAG_PARAMETER_23 0x97  ///< Parameter 23 (define usage in application)
#define TLV_TAG_PARAMETER_24 0x98  ///< Parameter 24 (define usage in application)
#define TLV_TAG_PARAMETER_25 0x99  ///< Parameter 25 (define usage in application)
#define TLV_TAG_PARAMETER_26 0x9A  ///< Parameter 26 (define usage in application)
#define TLV_TAG_PARAMETER_27 0x9B  ///< Parameter 27 (define usage in application)
#define TLV_TAG_PARAMETER_28 0x9C  ///< Parameter 28 (define usage in application)
#define TLV_TAG_PARAMETER_29 0x9D  ///< Parameter 29 (define usage in application)
#define TLV_TAG_PARAMETER_30 0x9E  ///< Parameter 30 (define usage in application)
#define TLV_TAG_PARAMETER_31 0x9F  ///< Parameter 31 (define usage in application)
#define TLV_TAG_PARAMETER_32 0xA0  ///< Parameter 32 (define usage in application)
#define TLV_TAG_PARAMETER_33 0xA1  ///< Parameter 33 (define usage in application)
#define TLV_TAG_PARAMETER_34 0xA2  ///< Parameter 34 (define usage in application)
#define TLV_TAG_PARAMETER_35 0xA3  ///< Parameter 35 (define usage in application)
#define TLV_TAG_PARAMETER_36 0xA4  ///< Parameter 36 (define usage in application)
#define TLV_TAG_PARAMETER_37 0xA5  ///< Parameter 37 (define usage in application)
#define TLV_TAG_PARAMETER_38 0xA6  ///< Parameter 38 (define usage in application)
#define TLV_TAG_PARAMETER_39 0xA7  ///< Parameter 39 (define usage in application)
#define TLV_TAG_PARAMETER_40 0xA8  ///< Parameter 40 (define usage in application)
#define TLV_TAG_PARAMETER_41 0xA9  ///< Parameter 41 (define usage in application)
#define TLV_TAG_PARAMETER_42 0xAA  ///< Parameter 42 (define usage in application)
#define TLV_TAG_PARAMETER_43 0xAB  ///< Parameter 43 (define usage in application)
#define TLV_TAG_PARAMETER_44 0xAC  ///< Parameter 44 (define usage in application)
#define TLV_TAG_PARAMETER_45 0xAD  ///< Parameter 45 (define usage in application)
#define TLV_TAG_PARAMETER_46 0xAE  ///< Parameter 46 (define usage in application)
#define TLV_TAG_PARAMETER_47 0xAF  ///< Parameter 47 (define usage in application)
#define TLV_TAG_PARAMETER_48 0xB0  ///< Parameter 48 (define usage in application)
#define TLV_TAG_PARAMETER_49 0xB1  ///< Parameter 49 (define usage in application)
#define TLV_TAG_PARAMETER_50 0xB2  ///< Parameter 50 (define usage in application)
#define TLV_TAG_PARAMETER_51 0xB3  ///< Parameter 51 (define usage in application)
#define TLV_TAG_PARAMETER_52 0xB4  ///< Parameter 52 (define usage in application)
#define TLV_TAG_PARAMETER_53 0xB5  ///< Parameter 53 (define usage in application)
#define TLV_TAG_PARAMETER_54 0xB6  ///< Parameter 54 (define usage in application)
#define TLV_TAG_PARAMETER_55 0xB7  ///< Parameter 55 (define usage in application)
#define TLV_TAG_PARAMETER_56 0xB8  ///< Parameter 56 (define usage in application)
#define TLV_TAG_PARAMETER_57 0xB9  ///< Parameter 57 (define usage in application)
#define TLV_TAG_PARAMETER_58 0xBA  ///< Parameter 58 (define usage in application)
#define TLV_TAG_PARAMETER_59 0xBB  ///< Parameter 59 (define usage in application)
#define TLV_TAG_PARAMETER_60 0xBC  ///< Parameter 60 (define usage in application)
#define TLV_TAG_PARAMETER_61 0xBD  ///< Parameter 61 (define usage in application)
#define TLV_TAG_PARAMETER_62 0xBE  ///< Parameter 62 (define usage in application)
#define TLV_TAG_PARAMETER_63 0xBF  ///< Parameter 63 (define usage in application)

// Reserved for Future Use (0xC0 - 0xEF)

// Command Tags (0xF0 - 0xFF)
#define TLV_TAG_CMD_RESET       0xF0  ///< No value: Reset command
#define TLV_TAG_CMD_SAVE_PARAMS 0xF1  ///< No value: Save parameters to EEPROM
#define TLV_TAG_CMD_LOAD_PARAMS 0xF2  ///< No value: Load parameters from EEPROM
#define TLV_TAG_CMD_ACK         0xFD  ///< uint8_t: Acknowledgment (tag that was processed)
#define TLV_TAG_CMD_NACK        0xFE  ///< uint8_t: Negative acknowledgment
#define TLV_TAG_CMD_ERROR       0xFF  ///< uint8_t: Error code

/* ==================== TLV Builder Structure ==================== */

typedef struct {
    uint8_t *buffer;          ///< Pointer to packet buffer
    uint16_t buffer_size;     ///< Total buffer size
    uint16_t current_offset;  ///< Current write position
    uint16_t entry_count;     ///< Number of TLV entries added
} TLV_Builder_t;

/* ==================== TLV Parser Structure ==================== */

typedef struct {
    const uint8_t *buffer;    ///< Pointer to packet buffer (read-only)
    uint16_t buffer_size;     ///< Total buffer size
    uint16_t current_offset;  ///< Current read position
    uint16_t entries_parsed;  ///< Number of entries parsed so far
} TLV_Parser_t;

/* ==================== TLV Entry Structure ==================== */

typedef struct {
    uint8_t tag;           ///< Entry tag
    uint16_t length;       ///< Value length
    const uint8_t *value;  ///< Pointer to value data
} TLV_Entry_t;

/* ==================== TLV Builder Functions ==================== */

/**
 * @brief  Initialize TLV builder for creating a new packet
 * @param  builder: Pointer to TLV builder structure
 * @param  buffer: Pointer to buffer for packet data
 * @param  buffer_size: Size of buffer in bytes
 * @retval bool: true on success
 */
bool TLV_BuilderInit(TLV_Builder_t *builder, uint8_t *buffer, uint16_t buffer_size);

/**
 * @brief  Add uint8_t value to TLV packet
 * @param  builder: Pointer to TLV builder
 * @param  tag: Tag identifier
 * @param  value: Value to add
 * @retval bool: true on success
 */
bool TLV_AddUInt8(TLV_Builder_t *builder, uint8_t tag, uint8_t value);

/**
 * @brief  Add uint16_t value to TLV packet
 * @param  builder: Pointer to TLV builder
 * @param  tag: Tag identifier
 * @param  value: Value to add
 * @retval bool: true on success
 */
bool TLV_AddUInt16(TLV_Builder_t *builder, uint8_t tag, uint16_t value);

/**
 * @brief  Add uint32_t value to TLV packet
 * @param  builder: Pointer to TLV builder
 * @param  tag: Tag identifier
 * @param  value: Value to add
 * @retval bool: true on success
 */
bool TLV_AddUInt32(TLV_Builder_t *builder, uint8_t tag, uint32_t value);

/**
 * @brief  Add int16_t value to TLV packet
 * @param  builder: Pointer to TLV builder
 * @param  tag: Tag identifier
 * @param  value: Value to add
 * @retval bool: true on success
 */
bool TLV_AddInt16(TLV_Builder_t *builder, uint8_t tag, int16_t value);

/**
 * @brief  Add float value to TLV packet
 * @param  builder: Pointer to TLV builder
 * @param  tag: Tag identifier
 * @param  value: Value to add
 * @retval bool: true on success
 */
bool TLV_AddFloat(TLV_Builder_t *builder, uint8_t tag, float value);

/**
 * @brief  Add byte array to TLV packet
 * @param  builder: Pointer to TLV builder
 * @param  tag: Tag identifier
 * @param  data: Pointer to data array
 * @param  length: Number of bytes to add
 * @retval bool: true on success
 */
bool TLV_AddBytes(TLV_Builder_t *builder, uint8_t tag, const uint8_t *data, uint16_t length);

/**
 * @brief  Add command tag (no value) to TLV packet
 * @param  builder: Pointer to TLV builder
 * @param  tag: Command tag identifier
 * @retval bool: true on success
 */
bool TLV_AddCommand(TLV_Builder_t *builder, uint8_t tag);

/**
 * @brief  Get current packet size
 * @param  builder: Pointer to TLV builder
 * @retval uint16_t: Total size of packet in bytes
 */
uint16_t TLV_GetPacketSize(const TLV_Builder_t *builder);

/**
 * @brief  Get number of entries in packet
 * @param  builder: Pointer to TLV builder
 * @retval uint16_t: Number of TLV entries
 */
uint16_t TLV_GetEntryCount(const TLV_Builder_t *builder);

/* ==================== TLV Parser Functions ==================== */

/**
 * @brief  Initialize TLV parser for reading a received packet
 * @param  parser: Pointer to TLV parser structure
 * @param  buffer: Pointer to received packet buffer
 * @param  buffer_size: Size of received data in bytes
 * @retval bool: true on success
 */
bool TLV_ParserInit(TLV_Parser_t *parser, const uint8_t *buffer, uint16_t buffer_size);

/**
 * @brief  Get next TLV entry from packet
 * @param  parser: Pointer to TLV parser
 * @param  entry: Pointer to TLV entry structure to fill
 * @retval bool: true if entry retrieved, false if end of packet
 */
bool TLV_GetNextEntry(TLV_Parser_t *parser, TLV_Entry_t *entry);

/**
 * @brief  Extract uint8_t value from TLV entry
 * @param  entry: Pointer to TLV entry
 * @param  value: Pointer to store extracted value
 * @retval bool: true on success
 */
bool TLV_ExtractUInt8(const TLV_Entry_t *entry, uint8_t *value);

/**
 * @brief  Extract uint16_t value from TLV entry
 * @param  entry: Pointer to TLV entry
 * @param  value: Pointer to store extracted value
 * @retval bool: true on success
 */
bool TLV_ExtractUInt16(const TLV_Entry_t *entry, uint16_t *value);

/**
 * @brief  Extract uint32_t value from TLV entry
 * @param  entry: Pointer to TLV entry
 * @param  value: Pointer to store extracted value
 * @retval bool: true on success
 */
bool TLV_ExtractUInt32(const TLV_Entry_t *entry, uint32_t *value);

/**
 * @brief  Extract int16_t value from TLV entry
 * @param  entry: Pointer to TLV entry
 * @param  value: Pointer to store extracted value
 * @retval bool: true on success
 */
bool TLV_ExtractInt16(const TLV_Entry_t *entry, int16_t *value);

/**
 * @brief  Extract float value from TLV entry
 * @param  entry: Pointer to TLV entry
 * @param  value: Pointer to store extracted value
 * @retval bool: true on success
 */
bool TLV_ExtractFloat(const TLV_Entry_t *entry, float *value);

/**
 * @brief  Get pointer to byte array in TLV entry
 * @param  entry: Pointer to TLV entry
 * @param  data: Pointer to store pointer to data
 * @param  length: Pointer to store data length
 * @retval bool: true on success
 */
bool TLV_ExtractBytes(const TLV_Entry_t *entry, const uint8_t **data, uint16_t *length);

/* ==================== Utility Functions ==================== */

/**
 * @brief  Get human-readable name for a tag
 * @param  tag: Tag value
 * @retval const char*: Tag name string
 */
// const char *TLV_GetTagName(uint8_t tag);

/**
 * @brief  Validate TLV packet structure
 * @param  buffer: Pointer to packet buffer
 * @param  size: Size of packet in bytes
 * @retval bool: true if packet is valid
 */
bool TLV_ValidatePacket(const uint8_t *buffer, uint16_t size);

#ifdef __cplusplus
}
#endif

#endif /* TLV_PROTOCOL_H */
