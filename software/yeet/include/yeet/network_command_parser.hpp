/**
 * @file network_command_parser.hpp
 * @brief Text-based command parser for network interface
 *
 * This module provides a clean, extensible parser for text-based commands
 * received over TCP/IP. Commands are parsed and dispatched to appropriate handlers.
 *
 * Design Features:
 * - Simple text protocol (human-readable, easy to debug)
 * - Extensible: Easy to add new commands without modifying parser logic
 * - Type-safe: Command enumeration for compile-time checking
 * - Zero dynamic allocation: All parsing done on stack
 *
 * Supported Commands:
 * - "START\n"  - Begin streaming telemetry
 * - "STOP\n"   - Stop streaming telemetry
 *
 * Future Extensions:
 * - Parameter setting: "SET:param_name=value\n"
 * - Query commands: "GET:param_name\n"
 * - Configuration: "CONFIG:...\n"
 */

#pragma once

#include <cstdint>
#include <cstring>

namespace yeet::telemetry {

/**
 * @brief Network command enumeration
 *
 * Defines all supported commands that can be received via network interface.
 */
enum class command_type : std::uint8_t {
    none = 0,        /**< No command parsed (invalid or incomplete) */
    start_telemetry, /**< Begin telemetry streaming */
    stop_telemetry,  /**< Stop telemetry streaming */
    unknown          /**< Unknown command (not recognized) */
};

/**
 * @brief Parsed command result
 *
 * Contains the command type and any associated parameters.
 * For simple commands (START/STOP), only type is relevant.
 */
struct parsed_command {
    command_type type = command_type::none; /**< Command type */

    /**
     * @brief Check if command is valid
     * @return true if command was successfully parsed
     */
    [[nodiscard]] constexpr bool is_valid() const noexcept
    {
        return type != command_type::none && type != command_type::unknown;
    }

    /**
     * @brief Check if this is a telemetry control command
     * @return true if command controls telemetry streaming
     */
    [[nodiscard]] constexpr bool is_telemetry_command() const noexcept
    {
        return type == command_type::start_telemetry || type == command_type::stop_telemetry;
    }
};

/**
 * @brief Network command parser
 *
 * Parses text-based commands from network buffer.
 * Stateless parser - no internal state, can be used from any context.
 */
class command_parser {
public:
    /**
     * @brief Parse command from network buffer
     *
     * @param buffer Pointer to received data buffer
     * @param length Number of bytes in buffer
     * @return Parsed command structure
     *
     * Example:
     * @code
     * command_parser parser;
     * auto cmd = parser.parse(rx_buffer, rx_length);
     * if (cmd.is_valid()) {
     *     // Handle command
     * }
     * @endcode
     */
    [[nodiscard]] static parsed_command parse(const std::uint8_t* buffer, std::uint16_t length) noexcept;

private:
    /**
     * @brief Match buffer against expected command string
     * @param buffer Input buffer
     * @param length Buffer length
     * @param expected Expected command string (null-terminated)
     * @param min_length Minimum required length
     * @return true if buffer matches expected command
     */
    [[nodiscard]] static bool match_command(
        const std::uint8_t* buffer, std::uint16_t length, const char* expected, std::uint16_t min_length) noexcept;
};

}  // namespace yeet::telemetry
