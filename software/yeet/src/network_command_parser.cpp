/**
 * @file network_command_parser.cpp
 * @brief Implementation of network command parser
 */

#include "yeet/network_command_parser.hpp"

namespace yeet::telemetry {

parsed_command command_parser::parse(const std::uint8_t* buffer, std::uint16_t length) noexcept
{
    // Validate input
    if (buffer == nullptr || length == 0) {
        return {command_type::none};
    }

    // Parse START command
    if (match_command(buffer, length, "START", 5)) {
        return {command_type::start_telemetry};
    }

    // Parse STOP command
    if (match_command(buffer, length, "STOP", 4)) {
        return {command_type::stop_telemetry};
    }

    // Unknown command
    return {command_type::unknown};
}

bool command_parser::match_command(
    const std::uint8_t* buffer, std::uint16_t length, const char* expected, std::uint16_t min_length) noexcept
{
    // Check minimum length requirement
    if (length < min_length) {
        return false;
    }

    // Compare buffer with expected command string
    for (std::uint16_t i = 0; i < min_length; ++i) {
        if (buffer[i] != static_cast<std::uint8_t>(expected[i])) {
            return false;
        }
    }

    return true;
}

}  // namespace yeet::telemetry
