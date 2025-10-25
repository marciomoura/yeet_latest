#ifndef SFC_EMBEDDED_NETWORK_HANDLER_HPP
#define SFC_EMBEDDED_NETWORK_HANDLER_HPP

#include <cstdint>

#include "sfc_embedded/task_io.h"

// Forward declare from yeet::application::sfc namespace
namespace yeet {
namespace application {
namespace sfc {
class parameter_handler;

namespace network {
class telemetry_manager;
}  // namespace network
}  // namespace sfc
}  // namespace application
}  // namespace yeet

namespace yeet {
namespace application {
namespace sfc {
namespace network {

/**
 * @brief Network protocol handler for SFC embedded system
 *
 * Manages network communication state and coordinates between:
 * - Telemetry streaming (START/STOP state management)
 * - Parameter protocol requests/responses
 * - Command processing
 *
 * Architecture:
 * - Maintains persistent telemetry streaming state
 * - Handles packet type detection and routing
 * - Coordinates telemetry and parameter operations to avoid conflicts
 * - Validates outgoing packets before transmission
 */
class network_handler {
public:
    /**
     * @brief Construct a new network handler
     *
     * @param telemetry_mgr Reference to telemetry manager
     * @param param_handler Reference to parameter handler
     */
    network_handler(telemetry_manager& telemetry_mgr,
        yeet::application::sfc::parameter_handler&
            param_handler); /**
                             * @brief Process incoming network packets and generate responses
                             *
                             * This is the main entry point for network communication. It:
                             * 1. Detects packet type (command, parameter request, etc.)
                             * 2. Routes to appropriate handler
                             * 3. Manages telemetry streaming state
                             * 4. Generates responses when needed
                             *
                             * @param inputs Network input data (received packets, connection status)
                             * @param outputs Network output data (response packets to transmit)
                             */
    void process(const network_inputs_t* inputs, network_outputs_t* outputs);

    /**
     * @brief Check if telemetry streaming is currently active
     *
     * @return true if telemetry is started (START command received)
     * @return false if telemetry is stopped or not yet started
     */
    bool is_telemetry_started() const { return telemetry_started_; }

private:
    /**
     * @brief Process incoming commands (START, STOP)
     *
     * @param buffer Command data buffer
     * @param length Command data length
     * @return true if command was recognized and processed
     * @return false if command was not recognized
     */
    bool process_command(const std::uint8_t* buffer, std::uint16_t length);

    /**
     * @brief Process incoming parameter protocol packets
     *
     * @param inputs Network input data
     * @param outputs Network output data
     * @return true if parameter request was processed and response queued
     * @return false if no parameter request or processing failed
     */
    bool process_parameter_request(const network_inputs_t* inputs, network_outputs_t* outputs);

    /**
     * @brief Process telemetry transmission
     *
     * @param inputs Network input data (connection status)
     * @param outputs Network output data (telemetry packets)
     * @param parameter_response_pending True if parameter response already queued
     */
    void process_telemetry(const network_inputs_t* inputs, network_outputs_t* outputs, bool parameter_response_pending);

    /**
     * @brief Validate outgoing packet before transmission
     *
     * Checks packet_type and protocol_version fields to detect buffer corruption
     *
     * @param buffer Packet data buffer
     * @param length Packet length in bytes
     * @return true if packet is valid
     * @return false if packet is corrupted
     */
    bool validate_outgoing_packet(const std::uint8_t* buffer, std::uint16_t length) const;

    // References to external managers
    telemetry_manager& telemetry_manager_;
    yeet::application::sfc::parameter_handler& parameter_handler_;

    // Internal state
    bool telemetry_started_;  ///< Is telemetry streaming currently active?
};

}  // namespace network
}  // namespace sfc
}  // namespace application
}  // namespace yeet

#endif  // SFC_EMBEDDED_NETWORK_HANDLER_HPP
