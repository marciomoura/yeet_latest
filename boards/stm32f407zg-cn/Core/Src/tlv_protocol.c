/**
 ******************************************************************************
 * @file    tlv_protocol.c
 * @brief   Tag-Length-Value (TLV) Protocol Implementation
 * @note    Simple, efficient protocol for telemetry and parameter exchange
 ******************************************************************************
 */

#include "tlv_protocol.h"

#include <string.h>

/* ==================== Private Helper Functions ==================== */

/**
 * @brief  Write 16-bit value in big-endian format
 */
static void WriteUInt16BE(uint8_t *buffer, uint16_t value)
{
    buffer[0] = (value >> 8) & 0xFF;
    buffer[1] = value & 0xFF;
}

/**
 * @brief  Read 16-bit value in big-endian format
 */
static uint16_t ReadUInt16BE(const uint8_t *buffer) { return ((uint16_t)buffer[0] << 8) | buffer[1]; }

/**
 * @brief  Check if there's enough space in builder buffer
 */
static bool CheckSpace(const TLV_Builder_t *builder, uint16_t required)
{
    return (builder->current_offset + required) <= builder->buffer_size;
}

/* ==================== TLV Builder Functions ==================== */

/**
 * @brief  Initialize TLV builder for creating a new packet
 */
bool TLV_BuilderInit(TLV_Builder_t *builder, uint8_t *buffer, uint16_t buffer_size)
{
    if (builder == NULL || buffer == NULL || buffer_size == 0) {
        return false;
    }

    builder->buffer = buffer;
    builder->buffer_size = buffer_size;
    builder->current_offset = 0;
    builder->entry_count = 0;

    // Clear buffer
    memset(buffer, 0, buffer_size);

    return true;
}

/**
 * @brief  Add raw TLV entry to packet
 */
static bool TLV_AddEntry(TLV_Builder_t *builder, uint8_t tag, const uint8_t *value, uint16_t length)
{
    if (builder == NULL || (length > 0 && value == NULL)) {
        return false;
    }

    // Check if there's enough space
    uint16_t required_space = TLV_HEADER_SIZE + length;
    if (!CheckSpace(builder, required_space)) {
        return false;
    }

    // Write tag
    builder->buffer[builder->current_offset++] = tag;

    // Write length (big-endian)
    WriteUInt16BE(&builder->buffer[builder->current_offset], length);
    builder->current_offset += 2;

    // Write value
    if (length > 0) {
        memcpy(&builder->buffer[builder->current_offset], value, length);
        builder->current_offset += length;
    }

    builder->entry_count++;

    return true;
}

/**
 * @brief  Add uint8_t value to TLV packet
 */
bool TLV_AddUInt8(TLV_Builder_t *builder, uint8_t tag, uint8_t value)
{
    return TLV_AddEntry(builder, tag, &value, sizeof(uint8_t));
}

/**
 * @brief  Add uint16_t value to TLV packet
 */
bool TLV_AddUInt16(TLV_Builder_t *builder, uint8_t tag, uint16_t value)
{
    uint8_t bytes[2];
    WriteUInt16BE(bytes, value);
    return TLV_AddEntry(builder, tag, bytes, sizeof(uint16_t));
}

/**
 * @brief  Add uint32_t value to TLV packet
 */
bool TLV_AddUInt32(TLV_Builder_t *builder, uint8_t tag, uint32_t value)
{
    uint8_t bytes[4];
    bytes[0] = (value >> 24) & 0xFF;
    bytes[1] = (value >> 16) & 0xFF;
    bytes[2] = (value >> 8) & 0xFF;
    bytes[3] = value & 0xFF;
    return TLV_AddEntry(builder, tag, bytes, sizeof(uint32_t));
}

/**
 * @brief  Add int16_t value to TLV packet
 */
bool TLV_AddInt16(TLV_Builder_t *builder, uint8_t tag, int16_t value)
{
    uint8_t bytes[2];
    WriteUInt16BE(bytes, (uint16_t)value);
    return TLV_AddEntry(builder, tag, bytes, sizeof(int16_t));
}

/**
 * @brief  Add float value to TLV packet
 */
bool TLV_AddFloat(TLV_Builder_t *builder, uint8_t tag, float value)
{
    // Cast float to byte array (preserves binary representation)
    union {
        float f;
        uint8_t bytes[4];
    } converter;

    converter.f = value;

    // Reverse byte order for big-endian network transmission
    uint8_t bytes_be[4];
    bytes_be[0] = converter.bytes[3];
    bytes_be[1] = converter.bytes[2];
    bytes_be[2] = converter.bytes[1];
    bytes_be[3] = converter.bytes[0];

    return TLV_AddEntry(builder, tag, bytes_be, sizeof(float));
}

/**
 * @brief  Add byte array to TLV packet
 */
bool TLV_AddBytes(TLV_Builder_t *builder, uint8_t tag, const uint8_t *data, uint16_t length)
{
    return TLV_AddEntry(builder, tag, data, length);
}

/**
 * @brief  Add command tag (no value) to TLV packet
 */
bool TLV_AddCommand(TLV_Builder_t *builder, uint8_t tag) { return TLV_AddEntry(builder, tag, NULL, 0); }

/**
 * @brief  Get current packet size
 */
uint16_t TLV_GetPacketSize(const TLV_Builder_t *builder)
{
    if (builder == NULL) {
        return 0;
    }
    return builder->current_offset;
}

/**
 * @brief  Get number of entries in packet
 */
uint16_t TLV_GetEntryCount(const TLV_Builder_t *builder)
{
    if (builder == NULL) {
        return 0;
    }
    return builder->entry_count;
}

/* ==================== TLV Parser Functions ==================== */

/**
 * @brief  Initialize TLV parser for reading a received packet
 */
bool TLV_ParserInit(TLV_Parser_t *parser, const uint8_t *buffer, uint16_t buffer_size)
{
    if (parser == NULL || buffer == NULL || buffer_size == 0) {
        return false;
    }

    parser->buffer = buffer;
    parser->buffer_size = buffer_size;
    parser->current_offset = 0;
    parser->entries_parsed = 0;

    return true;
}

/**
 * @brief  Get next TLV entry from packet
 */
bool TLV_GetNextEntry(TLV_Parser_t *parser, TLV_Entry_t *entry)
{
    if (parser == NULL || entry == NULL) {
        return false;
    }

    // Check if we've reached the end of the buffer
    if (parser->current_offset >= parser->buffer_size) {
        return false;
    }

    // Check if there's enough data for a header
    if ((parser->current_offset + TLV_HEADER_SIZE) > parser->buffer_size) {
        return false;  // Incomplete header
    }

    // Read tag
    entry->tag = parser->buffer[parser->current_offset++];

    // Read length (big-endian)
    entry->length = ReadUInt16BE(&parser->buffer[parser->current_offset]);
    parser->current_offset += 2;

    // Check if there's enough data for the value
    if ((parser->current_offset + entry->length) > parser->buffer_size) {
        return false;  // Incomplete value
    }

    // Set pointer to value (or NULL if length is 0)
    if (entry->length > 0) {
        entry->value = &parser->buffer[parser->current_offset];
        parser->current_offset += entry->length;
    }
    else {
        entry->value = NULL;
    }

    parser->entries_parsed++;

    return true;
}

/**
 * @brief  Extract uint8_t value from TLV entry
 */
bool TLV_ExtractUInt8(const TLV_Entry_t *entry, uint8_t *value)
{
    if (entry == NULL || value == NULL || entry->length != sizeof(uint8_t)) {
        return false;
    }

    *value = entry->value[0];
    return true;
}

/**
 * @brief  Extract uint16_t value from TLV entry
 */
bool TLV_ExtractUInt16(const TLV_Entry_t *entry, uint16_t *value)
{
    if (entry == NULL || value == NULL || entry->length != sizeof(uint16_t)) {
        return false;
    }

    *value = ReadUInt16BE(entry->value);
    return true;
}

/**
 * @brief  Extract uint32_t value from TLV entry
 */
bool TLV_ExtractUInt32(const TLV_Entry_t *entry, uint32_t *value)
{
    if (entry == NULL || value == NULL || entry->length != sizeof(uint32_t)) {
        return false;
    }

    *value = ((uint32_t)entry->value[0] << 24) | ((uint32_t)entry->value[1] << 16) | ((uint32_t)entry->value[2] << 8) |
             entry->value[3];
    return true;
}

/**
 * @brief  Extract int16_t value from TLV entry
 */
bool TLV_ExtractInt16(const TLV_Entry_t *entry, int16_t *value)
{
    if (entry == NULL || value == NULL || entry->length != sizeof(int16_t)) {
        return false;
    }

    uint16_t temp = ReadUInt16BE(entry->value);
    *value = (int16_t)temp;
    return true;
}

/**
 * @brief  Extract float value from TLV entry
 */
bool TLV_ExtractFloat(const TLV_Entry_t *entry, float *value)
{
    if (entry == NULL || value == NULL || entry->length != sizeof(float)) {
        return false;
    }

    // Reverse byte order from big-endian network transmission
    union {
        float f;
        uint8_t bytes[4];
    } converter;

    converter.bytes[3] = entry->value[0];
    converter.bytes[2] = entry->value[1];
    converter.bytes[1] = entry->value[2];
    converter.bytes[0] = entry->value[3];

    *value = converter.f;
    return true;
}

/**
 * @brief  Get pointer to byte array in TLV entry
 */
bool TLV_ExtractBytes(const TLV_Entry_t *entry, const uint8_t **data, uint16_t *length)
{
    if (entry == NULL || data == NULL || length == NULL) {
        return false;
    }

    *data = entry->value;
    *length = entry->length;
    return true;
}

/* ==================== Utility Functions ==================== */

/**
 * @brief  Get human-readable name for a tag
 */
// const char *TLV_GetTagName(uint8_t tag)
// {
//     switch (tag) {
//         // System Status
//         case TLV_TAG_SYSTEM_TIME:
//             return "SYSTEM_TIME";
//         case TLV_TAG_SYSTEM_STATUS:
//             return "SYSTEM_STATUS";
//         case TLV_TAG_ERROR_CODE:
//             return "ERROR_CODE";
//         case TLV_TAG_NODE_ID:
//             return "NODE_ID";

//         // Internal ADC
//         case TLV_TAG_ADC_INTERNAL_0:
//             return "ADC_INT_CH0";
//         case TLV_TAG_ADC_INTERNAL_1:
//             return "ADC_INT_CH1";
//         case TLV_TAG_ADC_INTERNAL_2:
//             return "ADC_INT_CH2";
//         case TLV_TAG_ADC_INTERNAL_3:
//             return "ADC_INT_CH3";

//         // External ADC (AD7606)
//         case TLV_TAG_ADC_EXTERNAL_0:
//             return "ADC_EXT_CH0";
//         case TLV_TAG_ADC_EXTERNAL_1:
//             return "ADC_EXT_CH1";
//         case TLV_TAG_ADC_EXTERNAL_2:
//             return "ADC_EXT_CH2";
//         case TLV_TAG_ADC_EXTERNAL_3:
//             return "ADC_EXT_CH3";
//         case TLV_TAG_ADC_EXTERNAL_4:
//             return "ADC_EXT_CH4";
//         case TLV_TAG_ADC_EXTERNAL_5:
//             return "ADC_EXT_CH5";
//         case TLV_TAG_ADC_EXTERNAL_6:
//             return "ADC_EXT_CH6";
//         case TLV_TAG_ADC_EXTERNAL_7:
//             return "ADC_EXT_CH7";
//         case TLV_TAG_ADC_ALL_INTERNAL:
//             return "ADC_ALL_INT";
//         case TLV_TAG_ADC_ALL_EXTERNAL:
//             return "ADC_ALL_EXT";

//         // Digital I/O
//         case TLV_TAG_FPGA_INPUTS:
//             return "FPGA_INPUTS";
//         case TLV_TAG_FPGA_OUTPUTS:
//             return "FPGA_OUTPUTS";
//         case TLV_TAG_LED_STATUS:
//             return "LED_STATUS";

//         // Control/Parameters
//         case TLV_TAG_SETPOINT_0:
//             return "SETPOINT_0";
//         case TLV_TAG_SETPOINT_1:
//             return "SETPOINT_1";
//         case TLV_TAG_SETPOINT_2:
//             return "SETPOINT_2";
//         case TLV_TAG_SETPOINT_3:
//             return "SETPOINT_3";
//         case TLV_TAG_GAIN_P:
//             return "GAIN_P";
//         case TLV_TAG_GAIN_I:
//             return "GAIN_I";
//         case TLV_TAG_GAIN_D:
//             return "GAIN_D";
//         case TLV_TAG_LIMIT_MIN:
//             return "LIMIT_MIN";
//         case TLV_TAG_LIMIT_MAX:
//             return "LIMIT_MAX";

//         // Task Performance
//         case TLV_TAG_TASK1_COUNTER:
//             return "TASK1_COUNTER";
//         case TLV_TAG_TASK2_COUNTER:
//             return "TASK2_COUNTER";
//         case TLV_TAG_TASK3_COUNTER:
//             return "TASK3_COUNTER";
//         case TLV_TAG_TASK1_TIME_US:
//             return "TASK1_TIME_US";
//         case TLV_TAG_TASK2_TIME_US:
//             return "TASK2_TIME_US";
//         case TLV_TAG_TASK3_TIME_US:
//             return "TASK3_TIME_US";

//         // Calculated Values
//         case TLV_TAG_VOLTAGE_RMS:
//             return "VOLTAGE_RMS";
//         case TLV_TAG_CURRENT_RMS:
//             return "CURRENT_RMS";
//         case TLV_TAG_POWER_ACTIVE:
//             return "POWER_ACTIVE";
//         case TLV_TAG_POWER_REACTIVE:
//             return "POWER_REACTIVE";
//         case TLV_TAG_FREQUENCY:
//             return "FREQUENCY";

//         // Commands
//         case TLV_TAG_CMD_RESET:
//             return "CMD_RESET";
//         case TLV_TAG_CMD_SAVE_PARAMS:
//             return "CMD_SAVE";
//         case TLV_TAG_CMD_LOAD_PARAMS:
//             return "CMD_LOAD";
//         case TLV_TAG_CMD_ACK:
//             return "CMD_ACK";
//         case TLV_TAG_CMD_NACK:
//             return "CMD_NACK";
//         case TLV_TAG_CMD_ERROR:
//             return "CMD_ERROR";

//         default:
//             return "UNKNOWN";
//     }
// }

/**
 * @brief  Validate TLV packet structure
 */
bool TLV_ValidatePacket(const uint8_t *buffer, uint16_t size)
{
    if (buffer == NULL || size == 0) {
        return false;
    }

    uint16_t offset = 0;

    while (offset < size) {
        // Check if there's enough data for a header
        if ((offset + TLV_HEADER_SIZE) > size) {
            return false;  // Incomplete header
        }

        // Read length
        uint16_t length = ReadUInt16BE(&buffer[offset + 1]);

        // Skip tag + length
        offset += TLV_HEADER_SIZE;

        // Check if there's enough data for the value
        if ((offset + length) > size) {
            return false;  // Incomplete value
        }

        // Move to next entry
        offset += length;
    }

    return (offset == size);  // Must consume entire packet
}

/* ==================== End of File ==================== */
