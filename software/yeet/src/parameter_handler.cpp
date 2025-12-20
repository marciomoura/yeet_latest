/**
 * @file parameter_handler.cpp
 * @brief Implementation of parameter handler using model_registry
 */

#include "yeet/parameter_handler.hpp"

#include <cmath>
#include <cstring>

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

inline bool ends_with(const std::string& s, const char* suffix) noexcept
{
    const std::string_view sv{s};
    const std::string_view suf{suffix};
    return sv.size() >= suf.size() && sv.compare(sv.size() - suf.size(), suf.size(), suf) == 0;
}

inline bool starts_with(const std::string& s, const char* prefix) noexcept
{
    const std::string_view sv{s};
    const std::string_view pre{prefix};
    return sv.size() >= pre.size() && sv.compare(0, pre.size(), pre) == 0;
}

inline bool is_index_parameter_name(const std::string& name) noexcept
{
    // IO index mapping parameters use the naming convention:
    //   di_{name}_index, do_{name}_index, ao_{name}_index
    return ends_with(name, "_index") &&
           (starts_with(name, "di_") || starts_with(name, "do_") || starts_with(name, "ao_"));
}

inline bool get_index_parameter_range(const std::string& name, float& min_value, float& max_value) noexcept
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

parameter_handler::parameter_handler(const model_registry* registry)
    : registry_(registry), parameter_cache_{}, sequence_counter_(0)
{
    if (!registry_) {
        YEET_PARAM_LOG("[PARAM] ERROR: model_registry pointer is null!");
        return;
    }

    build_parameter_cache();

    // Initialization complete (parameter count logged only in verbose mode)
}

void parameter_handler::build_parameter_cache()
{
    parameter_cache_.clear();

    // Iterate through all entries in model_registry
    registry_->iterate_entries(
        [this](const std::string& component, const std::string& name, const low_level_access& accessor) {
            // Create cache entry
            parameter_cache_entry entry;
            entry.component = component;
            entry.name = name;

            // Generate display name from component.name
            entry.display_name = name;

            // TODO: Extract unit from type metadata when available
            entry.unit = "";  // Could parse from type string later

            // Currently only support float32
            entry.type = parameter_value_type::float32;

            // TODO: Extract constraints from metadata when available
            // For now, use wide defaults, with special handling for IO index parameters.
            entry.min_value = k_default_min;
            entry.max_value = k_default_max;
            entry.default_value = 0.0f;

            // Apply per-parameter constraints for IO mapping indices.
            // This enables parameter SET rejection for out-of-range mappings.
            (void)get_index_parameter_range(entry.name, entry.min_value, entry.max_value);

            // Store accessor for direct read/write
            entry.accessor = accessor;

            parameter_cache_.push_back(std::move(entry));
        });

    // Only log parameter cache in verbose mode
    // Note: Detailed parameter list logging is suppressed for cleaner output
    // Use --verbose flag to see full parameter enumeration during initialization
}

void parameter_handler::rebuild_parameter_cache()
{
    if (!registry_) {
        YEET_PARAM_LOG("[PARAM] ERROR: Cannot rebuild cache - model_registry pointer is null!");
        return;
    }

    build_parameter_cache();

    // Rebuild complete (parameter count logged only in verbose mode)
}

parameter_cache_entry* parameter_handler::find_parameter(const char* component, const char* name)
{
    for (auto& param : parameter_cache_) {
        if (param.component == component && param.name == name) {
            return &param;
        }
    }
    return nullptr;
}

bool parameter_handler::is_value_in_range(const parameter_cache_entry& param, float value) const
{
    return (value >= param.min_value) && (value <= param.max_value);
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

    // Copy component and name
    std::strncpy(response.component, request.component, sizeof(response.component) - 1);
    std::strncpy(response.name, request.name, sizeof(response.name) - 1);

    // Find parameter
    parameter_cache_entry* param = find_parameter(request.component, request.name);

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

    YEET_PARAM_LOG("[PARAM] SET SUCCESS: " << param->component << "." << param->name << " = " << request.value
                                           << " (was " << response.old_value << ")");

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

    // Copy component and name
    std::strncpy(response.component, request.component, sizeof(response.component) - 1);
    std::strncpy(response.name, request.name, sizeof(response.name) - 1);

    // Find parameter
    parameter_cache_entry* param = find_parameter(request.component, request.name);

    if (!param) {
        YEET_PARAM_LOG("[PARAM] GET FAILED: Parameter " << request.component << "." << request.name << " not found");
        return false;
    }

    // Return current value via low_level_access
    response.value = param->accessor.read<float>();

    YEET_PARAM_LOG("[PARAM] GET: " << param->component << "." << param->name << " = " << response.value);

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
    response.total_parameters = static_cast<std::uint16_t>(parameter_cache_.size());

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
    if (request.parameter_index >= parameter_cache_.size()) {
        response.status = 1;  // INDEX_OUT_OF_RANGE
        response.parameter_index = request.parameter_index;
        response.total_parameters = static_cast<std::uint16_t>(parameter_cache_.size());
        YEET_PARAM_LOG("[PARAM] GET_BY_INDEX FAILED: Index "
                       << request.parameter_index << " out of range (max: " << parameter_cache_.size() - 1 << ")");
        return true;
    }

    // Get parameter at index
    const auto& param = parameter_cache_[request.parameter_index];

    // Fill response
    std::strncpy(response.component, param.component.c_str(), sizeof(response.component) - 1);
    std::strncpy(response.name, param.name.c_str(), sizeof(response.name) - 1);
    std::strncpy(response.display_name, param.display_name.c_str(), sizeof(response.display_name) - 1);
    std::strncpy(response.unit, param.unit.c_str(), sizeof(response.unit) - 1);

    // Generate description from display name (could be enhanced later)
    std::string description = "Parameter " + param.display_name;
    std::strncpy(response.description, description.c_str(), sizeof(response.description) - 1);

    response.type = static_cast<std::uint8_t>(param.type);
    response.min_value = param.min_value;
    response.max_value = param.max_value;
    response.default_value = param.default_value;

    // Read current value via low_level_access
    response.current_value = param.accessor.read<float>();

    response.parameter_index = request.parameter_index;
    response.total_parameters = static_cast<std::uint16_t>(parameter_cache_.size());
    response.status = 0;  // SUCCESS

    YEET_PARAM_LOG("[PARAM] GET_BY_INDEX: Returning parameter " << request.parameter_index << " (" << param.component
                                                                << "." << param.name << " = " << response.current_value
                                                                << ")");

    return true;
}

}  // namespace yeet::telemetry
