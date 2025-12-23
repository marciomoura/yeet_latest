/**
 * @file parameter_handler.cpp
 * @brief Implementation of parameter handler with direct registration
 */

#include "yeet/parameter_handler.hpp"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string_view>

#ifdef HOST_BUILD
#include <iostream>
#endif

#ifdef HOST_BUILD
#define YEET_PARAM_LOG(msg)            \
    if (verbose_) {                    \
        std::cout << msg << std::endl; \
    }
#else
#define YEET_PARAM_LOG(msg) ((void)0)
#endif

namespace {

constexpr float k_default_min = -1e6f;
constexpr float k_default_max = 1e6f;

// Keep these in sync with software/yeet_application/sfc_embedded/include/sfc_embedded/task_io.h
constexpr int k_task_io_num_digital_inputs = 48;
constexpr int k_task_io_num_digital_outputs = 48;
constexpr int k_task_io_num_analog_outputs = 4;

inline bool ends_with(std::string_view s, const char* suffix) noexcept
{
    const std::string_view suf{suffix};
    return s.size() >= suf.size() && s.compare(s.size() - suf.size(), suf.size(), suf) == 0;
}

inline bool starts_with(std::string_view s, const char* prefix) noexcept
{
    const std::string_view pre{prefix};
    return s.size() >= pre.size() && s.compare(0, pre.size(), pre) == 0;
}

inline bool is_index_parameter_name(std::string_view name) noexcept
{
    // IO index mapping parameters use the naming convention:
    //   di_{name}_index, do_{name}_index, ao_{name}_index
    return ends_with(name, "_index") &&
           (starts_with(name, "di_") || starts_with(name, "do_") || starts_with(name, "ao_"));
}

inline bool get_index_parameter_range(std::string_view name, float& min_value, float& max_value) noexcept
{
    if (starts_with(name, "di_") && ends_with(name, "_index")) {
        min_value = 0.0f;
        max_value = static_cast<float>(k_task_io_num_digital_inputs - 1);
        return true;
    }
    if (starts_with(name, "do_") && ends_with(name, "_index")) {
        min_value = 0.0f;
        max_value = static_cast<float>(k_task_io_num_digital_outputs - 1);
        return true;
    }
    if (starts_with(name, "ao_") && ends_with(name, "_index")) {
        min_value = 0.0f;
        max_value = static_cast<float>(k_task_io_num_analog_outputs - 1);
        return true;
    }
    return false;
}

inline bool is_effectively_integer(float v) noexcept
{
    // Parameter protocol transports float. For index parameters, accept only whole numbers.
    // Use a small epsilon to tolerate float representation.
    constexpr float eps = 1e-5f;
    return std::fabs(v - std::round(v)) <= eps;
}

inline void write_error_message(char* dst, std::size_t dst_size, const char* message) noexcept
{
    if (dst == nullptr || dst_size == 0) {
        return;
    }

    std::strncpy(dst, message != nullptr ? message : "", dst_size - 1);
    dst[dst_size - 1] = '\0';
}

}  // namespace

namespace yeet::telemetry {

parameter_cache_entry* parameter_handler::get_parameter_by_id(std::uint16_t id)
{
    if (id < parameter_cache_size_) {
        return &parameter_cache_[id];
    }
    return nullptr;
}

bool parameter_handler::is_value_in_range(const parameter_cache_entry& param, float value) const
{
    // Generate constraint values dynamically (same logic as in handle_get_by_index_request)
    float min_val = k_default_min;
    float max_val = k_default_max;
    (void)get_index_parameter_range(param.name, min_val, max_val);

    return (value >= min_val) && (value <= max_val);
}

bool parameter_handler::handle_set_request(
    const parameter_set_request_packet& request, parameter_set_response_packet& response)
{
    // Initialize response header
    std::memset(&response, 0, sizeof(response));
    response.header.packet_type = static_cast<std::uint8_t>(telemetry::packet_type::parameter_set_response);
    response.header.protocol_version = telemetry::protocol_version;
    response.header.payload_length = sizeof(response) - sizeof(response.header);
    response.header.sequence_number = sequence_counter_++;
    response.header.flags = 0;
    response.header.timestamp_ms = 0;

    // Copy parameter ID
    response.parameter_id = request.parameter_id;

    // Find parameter by ID (O(1) array indexing)
    parameter_cache_entry* param = get_parameter_by_id(request.parameter_id);

    if (!param) {
        // Parameter not found
        response.status = 1;  // NOT_FOUND
        write_error_message(response.error_message, sizeof(response.error_message), "NOT_FOUND");
        YEET_PARAM_LOG("[PARAM] SET FAILED: " << response.error_message);
        return true;
    }

    // Read current value
    response.old_value = param->accessor.read<float>();

    // Check range
    if (!is_value_in_range(*param, request.value)) {
        response.new_value = request.value;
        response.status = 2;  // OUT_OF_RANGE
        write_error_message(response.error_message, sizeof(response.error_message), "OUT_OF_RANGE");
        YEET_PARAM_LOG("[PARAM] SET FAILED: " << response.error_message);
        return true;
    }

    // Enforce integer-only writes for IO index parameters (e.g. di_*_index).
    // This is done after range check so error messaging remains consistent.
    if (is_index_parameter_name(param->name) && !is_effectively_integer(request.value)) {
        response.new_value = request.value;
        response.status = 2;  // OUT_OF_RANGE
        write_error_message(response.error_message, sizeof(response.error_message), "INVALID_INTEGER");
        YEET_PARAM_LOG("[PARAM] SET FAILED: " << response.error_message);
        return true;
    }

    // Write new value via low_level_access
    param->accessor.write<float>(request.value);
    response.new_value = request.value;
    response.status = 0;  // SUCCESS
    write_error_message(response.error_message, sizeof(response.error_message), "OK");

    YEET_PARAM_LOG("[PARAM] SET SUCCESS: ID=" << request.parameter_id << " (" << param->component << "." << param->name
                                              << ") = " << request.value << " (was " << response.old_value << ")");

    return true;
}

bool parameter_handler::handle_get_request(
    const parameter_get_request_packet& request, parameter_get_response_packet& response)
{
    // Initialize response header
    std::memset(&response, 0, sizeof(response));
    response.header.packet_type = static_cast<std::uint8_t>(telemetry::packet_type::parameter_get_response);
    response.header.protocol_version = telemetry::protocol_version;
    response.header.payload_length = sizeof(response) - sizeof(response.header);
    response.header.sequence_number = sequence_counter_++;
    response.header.flags = 0;
    response.header.timestamp_ms = 0;

    // Copy parameter ID
    response.parameter_id = request.parameter_id;

    // Find parameter by ID (O(1) array indexing)
    parameter_cache_entry* param = get_parameter_by_id(request.parameter_id);

    if (!param) {
        YEET_PARAM_LOG("[PARAM] GET FAILED: Parameter ID " << request.parameter_id << " not found");
        return false;
    }

    // Return current value via low_level_access
    response.value = param->accessor.read<float>();

    YEET_PARAM_LOG("[PARAM] GET: ID=" << request.parameter_id << " (" << param->component << "." << param->name
                                      << ") = " << response.value);

    return true;
}

bool parameter_handler::handle_count_request(parameter_count_response_packet& response)
{
    std::memset(&response, 0, sizeof(response));

    // Header
    response.header.packet_type = static_cast<std::uint8_t>(telemetry::packet_type::parameter_count_response);
    response.header.protocol_version = telemetry::protocol_version;
    response.header.payload_length = sizeof(response) - sizeof(response.header);
    response.header.sequence_number = sequence_counter_++;
    response.header.flags = 0;
    response.header.timestamp_ms = 0;

    // Count
    response.total_parameters = static_cast<std::uint16_t>(parameter_cache_size_);

    YEET_PARAM_LOG("[PARAM] COUNT: Returning " << response.total_parameters << " parameters");

    return true;
}

bool parameter_handler::handle_get_by_index_request(
    const parameter_get_by_index_request_packet& request, parameter_get_by_index_response_packet& response)
{
    std::memset(&response, 0, sizeof(response));

    // Header
    response.header.packet_type = static_cast<std::uint8_t>(telemetry::packet_type::parameter_get_by_index_response);
    response.header.protocol_version = telemetry::protocol_version;
    response.header.payload_length = sizeof(response) - sizeof(response.header);
    response.header.sequence_number = sequence_counter_++;
    response.header.flags = 0;
    response.header.timestamp_ms = 0;

    // Check index bounds
    if (request.parameter_index >= parameter_cache_size_) {
        response.status = 1;  // INDEX_OUT_OF_RANGE
        response.parameter_index = request.parameter_index;
        response.total_parameters = static_cast<std::uint16_t>(parameter_cache_size_);
        YEET_PARAM_LOG("[PARAM] GET_BY_INDEX FAILED: Index "
                       << request.parameter_index << " out of range (max: " << parameter_cache_size_ - 1 << ")");
        return true;
    }

    // Get parameter at index
    const auto& param = parameter_cache_[request.parameter_index];

    // Fill response with parameter ID (for use in GET/SET requests)
    response.parameter_id = param.id;
    response.parameter_index = request.parameter_index;
    response.total_parameters = static_cast<std::uint16_t>(parameter_cache_size_);
    response.status = 0;  // SUCCESS

    // Component and name (for display/debugging)
    std::strncpy(response.component, param.component, sizeof(response.component) - 1);
    response.component[sizeof(response.component) - 1] = '\0';
    std::strncpy(response.name, param.name, sizeof(response.name) - 1);
    response.name[sizeof(response.name) - 1] = '\0';

    // Generate display name from parameter name (dynamically)
    std::strncpy(response.display_name, param.name, sizeof(response.display_name) - 1);
    response.display_name[sizeof(response.display_name) - 1] = '\0';

    // Unit not currently used - leave empty
    response.unit[0] = '\0';

    // Generate description from parameter name (dynamically)
    char description[128];
    std::snprintf(description, sizeof(description), "Parameter %s.%s", param.component, param.name);
    std::strncpy(response.description, description, sizeof(response.description) - 1);
    response.description[sizeof(response.description) - 1] = '\0';

    response.type = static_cast<std::uint8_t>(param.type);

    // Generate constraint values dynamically
    // For IO index parameters, apply special range constraints
    float min_val = k_default_min;
    float max_val = k_default_max;
    (void)get_index_parameter_range(param.name, min_val, max_val);

    response.min_value = min_val;
    response.max_value = max_val;
    response.default_value = 0.0f;

    // Read current value via low_level_access
    response.current_value = param.accessor.read<float>();

    YEET_PARAM_LOG("[PARAM] GET_BY_INDEX: Returning parameter " << request.parameter_index << " ID=" << param.id << " ("
                                                                << param.component << "." << param.name << " = "
                                                                << response.current_value << ")");

    return true;
}

}  // namespace yeet::telemetry