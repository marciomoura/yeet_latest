/**
 * @file parameter_handler.cpp
 * @brief Implementation of parameter handler using model_registry
 */

#include "yeet/parameter_handler.hpp"

#include <iostream>

namespace yeet::telemetry {

parameter_handler::parameter_handler(const model_registry* registry)
    : registry_(registry), parameter_cache_{}, sequence_counter_(0)
{
    if (!registry_) {
#ifdef HOST_BUILD
        std::cout << "[PARAM] ERROR: model_registry pointer is null!" << std::endl;
#endif
        return;
    }

    build_parameter_cache();

#ifdef HOST_BUILD
    std::cout << "[PARAM] Initialized with " << parameter_cache_.size() << " parameters from model_registry"
              << std::endl;
#endif
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
            // For now, use wide defaults
            entry.min_value = -1e6f;
            entry.max_value = 1e6f;
            entry.default_value = 0.0f;

            // Store accessor for direct read/write
            entry.accessor = accessor;

            parameter_cache_.push_back(std::move(entry));
        });

#ifdef HOST_BUILD
    std::cout << "[PARAM] Built cache with " << parameter_cache_.size() << " parameters:" << std::endl;
    for (const auto& p : parameter_cache_) {
        std::cout << "[PARAM]   - " << p.component << "." << p.name << std::endl;
    }
#endif
}

void parameter_handler::rebuild_parameter_cache()
{
    if (!registry_) {
#ifdef HOST_BUILD
        std::cout << "[PARAM] ERROR: Cannot rebuild cache - model_registry pointer is null!" << std::endl;
#endif
        return;
    }

    build_parameter_cache();

#ifdef HOST_BUILD
    std::cout << "[PARAM] Rebuilt cache with " << parameter_cache_.size() << " parameters" << std::endl;
#endif
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
        std::snprintf(response.error_message, sizeof(response.error_message), "Parameter %s.%s not found",
            request.component, request.name);
#ifdef HOST_BUILD
        std::cout << "[PARAM] SET FAILED: " << response.error_message << std::endl;
#endif
        return true;
    }

    // Read current value
    response.old_value = param->accessor.read<float>();

    // Check range
    if (!is_value_in_range(*param, request.value)) {
        response.new_value = request.value;
        response.status = 2;  // OUT_OF_RANGE
        std::snprintf(response.error_message, sizeof(response.error_message), "Value %.4f out of range [%.4f, %.4f]",
            request.value, param->min_value, param->max_value);
#ifdef HOST_BUILD
        std::cout << "[PARAM] SET FAILED: " << response.error_message << std::endl;
#endif
        return true;
    }

    // Write new value via low_level_access
    param->accessor.write<float>(request.value);
    response.new_value = request.value;
    response.status = 0;  // SUCCESS
    std::snprintf(response.error_message, sizeof(response.error_message), "OK");

#ifdef HOST_BUILD
    std::cout << "[PARAM] SET SUCCESS: " << param->component << "." << param->name << " = " << request.value << " (was "
              << response.old_value << ")" << std::endl;
#endif

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
#ifdef HOST_BUILD
        std::cout << "[PARAM] GET FAILED: Parameter " << request.component << "." << request.name << " not found"
                  << std::endl;
#endif
        return false;
    }

    // Return current value via low_level_access
    response.value = param->accessor.read<float>();

#ifdef HOST_BUILD
    std::cout << "[PARAM] GET: " << param->component << "." << param->name << " = " << response.value << std::endl;
#endif

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

#ifdef HOST_BUILD
    std::cout << "[PARAM] COUNT: Returning " << response.total_parameters << " parameters" << std::endl;
#endif

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
#ifdef HOST_BUILD
        std::cout << "[PARAM] GET_BY_INDEX FAILED: Index " << request.parameter_index
                  << " out of range (max: " << parameter_cache_.size() - 1 << ")" << std::endl;
#endif
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

#ifdef HOST_BUILD
    std::cout << "[PARAM] GET_BY_INDEX: Returning parameter " << request.parameter_index << " (" << param.component
              << "." << param.name << " = " << response.current_value << ")" << std::endl;
#endif

    return true;
}

}  // namespace yeet::telemetry
