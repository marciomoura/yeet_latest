/**
 * @file parameter_handler.hpp
 * @brief Parameter handler for SFC control system
 *
 * Provides protocol handlers for reading/writing parameters over network.
 * Reads parameter metadata from model_registry which is populated by all
 * SFC component datamodels during initialization.
 */

#pragma once

#include <array>
#include <cstdint>
#include <cstring>

#include "yeet/model_registry.hpp"
#include "yeet/network_telemetry.hpp"
#include "yeet/utils/low_level_access.hpp"

namespace yeet::telemetry {

/**
 * @brief Parameter value type enumeration
 */
enum class parameter_value_type : std::uint8_t { float32 = 0, int32 = 1, boolean = 2, enum_value = 3 };

/**
 * @brief Parameter metadata structure
 *
 * Describes a single parameter including constraints and current value.
 */
struct parameter_metadata {
    char component[64];     // Component name (e.g., "test_component")
    char name[64];          // Parameter name (e.g., "test_gain")
    char display_name[64];  // Human-readable name
    char unit[16];          // Unit string
    parameter_value_type type;
    float min_value;
    float max_value;
    float default_value;
    float current_value;
    char description[128];
};

/**
 * @brief Parameter SET request packet
 */
struct parameter_set_request_packet {
    telemetry::telemetry_packet_header header;
    std::uint16_t parameter_id;
    std::uint16_t padding;
    float value;
};
static_assert(sizeof(parameter_set_request_packet) == 16 + 4 + 4, "Must be packed");

/**
 * @brief Parameter SET response packet
 */
struct parameter_set_response_packet {
    telemetry::telemetry_packet_header header;
    std::uint16_t parameter_id;
    std::uint16_t padding;
    float old_value;
    float new_value;
    std::uint8_t status;  // 0=success, 1=not_found, 2=out_of_range, 3=read_only
    std::uint8_t padding2[3];
    char error_message[128];
};
static_assert(sizeof(parameter_set_response_packet) == 16 + 4 + 8 + 4 + 128, "Must be packed");

/**
 * @brief Parameter GET request packet
 */
struct parameter_get_request_packet {
    telemetry::telemetry_packet_header header;
    std::uint16_t parameter_id;
    std::uint16_t padding;
};
static_assert(sizeof(parameter_get_request_packet) == 16 + 4, "Must be packed");

/**
 * @brief Parameter GET response packet
 */
struct parameter_get_response_packet {
    telemetry::telemetry_packet_header header;
    std::uint16_t parameter_id;
    std::uint16_t padding;
    float value;
};
static_assert(sizeof(parameter_get_response_packet) == 16 + 4 + 4, "Must be packed");

/**
 * @brief Parameter COUNT request packet (header only)
 * Request total number of parameters available
 */
struct parameter_count_request_packet {
    telemetry::telemetry_packet_header header;
};
static_assert(sizeof(parameter_count_request_packet) == 16, "Must be header only");

/**
 * @brief Parameter COUNT response packet
 * Returns total number of parameters
 */
struct parameter_count_response_packet {
    telemetry::telemetry_packet_header header;
    std::uint16_t total_parameters;
    std::uint16_t padding;
};
static_assert(sizeof(parameter_count_response_packet) == 16 + 4, "Must be packed");

/**
 * @brief Parameter GET_BY_INDEX request packet
 * Request parameter metadata by zero-based index
 */
struct parameter_get_by_index_request_packet {
    telemetry::telemetry_packet_header header;
    std::uint16_t parameter_index;
    std::uint16_t padding;
};
static_assert(sizeof(parameter_get_by_index_request_packet) == 16 + 4, "Must be packed");

/**
 * @brief Parameter GET_BY_INDEX response packet
 * Returns metadata for a single parameter at specified index
 */
struct parameter_get_by_index_response_packet {
    telemetry::telemetry_packet_header header;
    std::uint16_t parameter_id;     // Unique ID for this parameter (use in GET/SET)
    std::uint16_t parameter_index;  // Index in discovery sequence
    std::uint16_t total_parameters;
    std::uint8_t status;  // 0=success, 1=index_out_of_range
    std::uint8_t padding;
    char component[64];
    char name[64];
    char display_name[64];
    char unit[16];
    std::uint8_t type;
    std::uint8_t padding2[3];
    float min_value;
    float max_value;
    float default_value;
    float current_value;
    char description[128];
};
static_assert(
    sizeof(parameter_get_by_index_response_packet) == 16 + 8 + 64 + 64 + 64 + 16 + 4 + 16 + 128, "Must be packed");

/**
 * @brief Parameter metadata cache entry
 *
 * Stores minimal cached information about a parameter from model_registry.
 * Display names, descriptions, units, and constraints are generated dynamically
 * in response packets to minimize memory footprint.
 */
struct parameter_cache_entry {
    std::uint16_t id;           // Unique parameter ID (same as array index)
    char component[64];         // Component name (for metadata/display only)
    char name[64];              // Parameter name (for metadata/display only)
    parameter_value_type type;  // Currently only FLOAT32 supported
    low_level_access accessor;  // Direct access to parameter storage
};

/**
 * @brief Parameter handler using model_registry
 *
 * Reads parameters dynamically from model_registry instead of hardcoded test data.
 * Builds a cache of parameter metadata on initialization for fast lookup.
 */
class parameter_handler {
public:
    /**
     * @brief Maximum number of parameters supported in the cache
     */
    static constexpr std::size_t MAX_PARAMETERS = 256;

    /**
     * @brief Construct parameter handler from model_registry
     * @param registry Reference to model_registry containing all parameters
     */
    explicit parameter_handler(const model_registry* registry);

    /**
     * @brief Handle parameter SET request
     * @param request SET request packet
     * @param response SET response packet (output)
     * @return true if handled successfully
     */
    bool handle_set_request(const parameter_set_request_packet& request, parameter_set_response_packet& response);

    /**
     * @brief Handle parameter GET request
     * @param request GET request packet
     * @param response GET response packet (output)
     * @return true if handled successfully
     */
    bool handle_get_request(const parameter_get_request_packet& request, parameter_get_response_packet& response);

    /**
     * @brief Handle parameter COUNT request
     * @param response COUNT response packet (output)
     * @return true if handled successfully
     */
    bool handle_count_request(parameter_count_response_packet& response);

    /**
     * @brief Handle parameter GET_BY_INDEX request
     * @param request GET_BY_INDEX request packet
     * @param response GET_BY_INDEX response packet (output)
     * @return true if handled successfully
     */
    bool handle_get_by_index_request(
        const parameter_get_by_index_request_packet& request, parameter_get_by_index_response_packet& response);

    /**
     * @brief Get total parameter count
     */
    std::uint16_t get_parameter_count() const { return static_cast<std::uint16_t>(parameter_cache_size_); }

    /**
     * @brief Rebuild parameter cache from registry
     *
     * Call this after registering new datamodels to the registry.
     * Clears existing cache and rebuilds from current registry state.
     */
    void rebuild_parameter_cache();

    /**
     * @brief Set verbose logging
     * @param verbose True to enable verbose logging
     */
    void set_verbose(bool verbose) noexcept { verbose_ = verbose; }

private:
    /**
     * @brief Build parameter cache from model_registry
     *
     * Iterates through all parameters in registry and builds indexed cache
     */
    void build_parameter_cache();

    /**
     * @brief Get parameter by ID (direct array indexing)
     * @return Pointer to cache entry, or nullptr if ID out of range
     */
    parameter_cache_entry* get_parameter_by_id(std::uint16_t id);

    /**
     * @brief Validate parameter value against constraints (currently checks min/max if available)
     */
    bool is_value_in_range(const parameter_cache_entry& param, float value) const;

    /**
     * @brief Pointer to model_registry (not owned, must outlive this object)
     */
    const model_registry* registry_;

    /**
     * @brief Indexed cache of parameter metadata for fast lookup
     * Built once during construction by iterating model_registry
     */
    std::array<parameter_cache_entry, MAX_PARAMETERS> parameter_cache_;

    /**
     * @brief Current number of parameters in the cache
     */
    std::size_t parameter_cache_size_ = 0;

    /**
     * @brief Sequence counter for response packets
     */
    std::uint32_t sequence_counter_;

    /**
     * @brief Verbose logging flag
     */
    bool verbose_ = false;
};

}  // namespace yeet::telemetry