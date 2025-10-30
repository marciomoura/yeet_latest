#include "yeet/network_handler.hpp"

#include <cstring>

#include "yeet/network_command_parser.hpp"

namespace yeet::telemetry {

network_handler::network_handler(telemetry_manager& telemetry_mgr, parameter_handler& param_handler)
    : telemetry_manager_(telemetry_mgr),
      parameter_handler_(param_handler),
      telemetry_started_(false),
      sync_frame_counter_(0)
{
}

void network_handler::process(const network_inputs_t* inputs, network_outputs_t* outputs)
{
    // Reset transmit request flag
    if (outputs != nullptr) {
        outputs->network_tx.transmit_request = false;
    }

    // ==================== PROCESS INCOMING PACKETS ====================
    bool parameter_response_pending = false;

    if (inputs != nullptr && inputs->network_rx.data_ready && inputs->network_rx.rx_length > 0) {
        // Try to process as parameter request first (most common)
        parameter_response_pending = process_parameter_request(inputs, outputs);

        // If not a parameter request, try command parsing
        if (!parameter_response_pending) {
            bool command_processed =
                process_command(inputs->network_rx.rx_buffer, inputs->network_rx.rx_length, outputs);
            parameter_response_pending = command_processed;  // Command may queue sync frame
        }
    }

    // ==================== PROCESS TELEMETRY TRANSMISSION ====================
    // Only transmit telemetry if:
    //   1. We're connected
    //   2. Telemetry is started
    //   3. No parameter response was queued this cycle
    if (inputs != nullptr && inputs->network_rx.tcp_connected) {
        process_telemetry(inputs, outputs, parameter_response_pending);
    }
}

bool network_handler::process_command(const std::uint8_t* buffer, std::uint16_t length, network_outputs_t* outputs)
{
    auto cmd = telemetry::command_parser::parse(buffer, length);

    if (cmd.is_valid()) {
        switch (cmd.type) {
            case telemetry::command_type::start_telemetry:
                telemetry_started_ = true;
                telemetry_manager_.start();
                sync_frame_counter_ = 0;  // Reset counter for new stream

                // Queue sync frame (0xDEADBEEF) at start of stream for client resynchronization
                if (outputs != nullptr) {
                    std::size_t sync_size = telemetry::encode_sync_frame(outputs->network_tx.tx_buffer);
                    outputs->network_tx.tx_length = sync_size;
                    outputs->network_tx.transmit_request = true;
                }
                return true;

            case telemetry::command_type::stop_telemetry:
                telemetry_started_ = false;
                telemetry_manager_.stop();
                sync_frame_counter_ = 0;  // Reset counter when stopping
                return true;

            default:
                break;
        }
    }

    return false;
}

bool network_handler::process_parameter_request(const network_inputs_t* inputs, network_outputs_t* outputs)
{
    // Check if packet is large enough to have a header
    if (inputs->network_rx.rx_length < sizeof(telemetry::telemetry_packet_header)) {
        return false;
    }

    const auto* header = reinterpret_cast<const telemetry::telemetry_packet_header*>(inputs->network_rx.rx_buffer);

    // Dispatch based on packet type
    switch (static_cast<telemetry::packet_type>(header->packet_type)) {
        case telemetry::packet_type::parameter_count_request: {
            parameter_count_response_packet response;
            if (parameter_handler_.handle_count_request(response)) {
                if (outputs != nullptr) {
                    std::memcpy(outputs->network_tx.tx_buffer, &response, sizeof(response));
                    outputs->network_tx.tx_length = sizeof(response);
                    outputs->network_tx.transmit_request = true;
                    outputs->network_tx.socket_id = inputs->network_rx.socket_id;
                    return true;
                }
            }
            break;
        }

        case telemetry::packet_type::parameter_get_by_index_request: {
            // Accept either full struct (20 bytes: header + 4) or minimal (header + 2 index bytes)
            const std::size_t min_size = sizeof(telemetry::telemetry_packet_header) + 2;
            if (inputs->network_rx.rx_length >= min_size) {
                parameter_get_by_index_request_packet request{};
                std::memset(&request, 0, sizeof(request));
                const std::size_t copy_size =
                    (inputs->network_rx.rx_length < sizeof(request)) ? inputs->network_rx.rx_length : sizeof(request);
                std::memcpy(&request, inputs->network_rx.rx_buffer, copy_size);

                parameter_get_by_index_response_packet response;
                if (parameter_handler_.handle_get_by_index_request(request, response)) {
                    if (outputs != nullptr) {
                        std::memcpy(outputs->network_tx.tx_buffer, &response, sizeof(response));
                        outputs->network_tx.tx_length = sizeof(response);
                        outputs->network_tx.transmit_request = true;
                        outputs->network_tx.socket_id = inputs->network_rx.socket_id;
                        return true;
                    }
                }
            }
            break;
        }

        case telemetry::packet_type::parameter_get_request: {
            if (inputs->network_rx.rx_length >= sizeof(parameter_get_request_packet)) {
                const auto* request =
                    reinterpret_cast<const parameter_get_request_packet*>(inputs->network_rx.rx_buffer);
                parameter_get_response_packet response;
                if (parameter_handler_.handle_get_request(*request, response)) {
                    if (outputs != nullptr) {
                        std::memcpy(outputs->network_tx.tx_buffer, &response, sizeof(response));
                        outputs->network_tx.tx_length = sizeof(response);
                        outputs->network_tx.transmit_request = true;
                        outputs->network_tx.socket_id = inputs->network_rx.socket_id;
                        return true;
                    }
                }
            }
            break;
        }

        case telemetry::packet_type::parameter_set_request: {
            if (inputs->network_rx.rx_length >= sizeof(parameter_set_request_packet)) {
                const auto* request =
                    reinterpret_cast<const parameter_set_request_packet*>(inputs->network_rx.rx_buffer);
                parameter_set_response_packet response;
                if (parameter_handler_.handle_set_request(*request, response)) {
                    if (outputs != nullptr) {
                        std::memcpy(outputs->network_tx.tx_buffer, &response, sizeof(response));
                        outputs->network_tx.tx_length = sizeof(response);
                        outputs->network_tx.transmit_request = true;
                        outputs->network_tx.socket_id = inputs->network_rx.socket_id;
                        return true;
                    }
                }
            }
            break;
        }

        default:
            // Not a parameter packet
            return false;
    }

    return false;
}

void network_handler::process_telemetry(
    const network_inputs_t* inputs, network_outputs_t* outputs, bool parameter_response_pending)
{
#ifdef HOST_BUILD
    static int call_count = 0;
    if (++call_count % 100 == 0) {  // Print every 100 calls to avoid spam
        printf("[DEBUG] process_telemetry() called %d times, telemetry_started=%d, param_pending=%d\n", call_count,
            telemetry_started_ ? 1 : 0, parameter_response_pending ? 1 : 0);
    }
#endif

    if (outputs == nullptr) {
        return;
    }

    // Only process telemetry if we didn't already queue a parameter response
    if (!parameter_response_pending) {
        // Only transmit if telemetry streaming is started
        if (telemetry_started_) {
            // Check if it's time to send periodic sync frame for stream recovery
            // (helps client recover from packet loss or buffer desynchronization)
            if (++sync_frame_counter_ >= SYNC_FRAME_INTERVAL) {
                sync_frame_counter_ = 0;

                // Queue periodic sync frame
                std::size_t sync_size = encode_sync_frame(outputs->network_tx.tx_buffer);
                outputs->network_tx.tx_length = sync_size;
                outputs->network_tx.transmit_request = true;
                outputs->network_tx.socket_id = (inputs != nullptr) ? inputs->network_rx.socket_id : 0;
#ifdef HOST_BUILD
                printf("[SYNC] Sending periodic sync frame (counter reset after %u packets)\n", SYNC_FRAME_INTERVAL);
#endif
                return;  // Send sync frame, skip telemetry this cycle
            }

            // Update telemetry manager (handles decimation and packet building)
            auto tx = telemetry_manager_.update(false);

            // Transmit packet if ready
            if (tx.is_ready()) {
                // Validate packet before transmission
                if (validate_outgoing_packet(tx.data, tx.size)) {
#ifdef HOST_BUILD
                    printf("[TELEMETRY] Transmitting telemetry packet: %zu bytes\n", tx.size);
#endif
                    // Copy encoded packet to transmission buffer
                    std::memcpy(outputs->network_tx.tx_buffer, tx.data, tx.size);
                    outputs->network_tx.tx_length = static_cast<std::uint16_t>(tx.size);
                    outputs->network_tx.transmit_request = true;
                    outputs->network_tx.socket_id = inputs->network_rx.socket_id;
                }
                // If validation fails, discard corrupted packet (silent drop)
            }
        }
        // If telemetry not started, do nothing (just listen for commands/parameters)
    }
    else {
        // Parameter response is queued - tell telemetry manager to skip this cycle
        // This prevents telemetry sequence number gaps (only if telemetry is started)
        if (telemetry_started_) {
            (void)telemetry_manager_.update(true);  // Discard return value - we're only updating state
        }
    }
}

bool network_handler::validate_outgoing_packet(const std::uint8_t* buffer, std::uint16_t length) const
{
    if (length < 2) {
        return false;  // Packet too short to have header
    }

    const std::uint8_t packet_type_byte = buffer[0];
    const std::uint8_t protocol_version = buffer[1];

    // Valid packet types (must match network_telemetry.hpp):
    // - 0x01-0x02: telemetry (grid/machine)
    // - 0x10: system_status
    // - 0x20: configuration
    // - 0x30: command
    // - 0x40-0x49: parameter protocol packets (SET, GET, COUNT, GET_BY_INDEX, etc.)
    const bool valid_type = (packet_type_byte >= 0x01 && packet_type_byte <= 0x02) ||  // telemetry
                            (packet_type_byte == 0x10) ||                              // system_status
                            (packet_type_byte == 0x20) ||                              // configuration
                            (packet_type_byte == 0x30) ||                              // command
                            (packet_type_byte >= 0x40 && packet_type_byte <= 0x49);    // parameters

    // Protocol version must be 0x01
    const bool valid_version = (protocol_version == 0x01);

    return valid_type && valid_version;
}

}  // namespace yeet::telemetry
