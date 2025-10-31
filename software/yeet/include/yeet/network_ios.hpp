#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
namespace yeet::telemetry {
#endif

#define MAX_NETWORK_TX_BUFFER_SIZE 1460

/**
 * @brief Network input structure (non-real-time communication)
 *
 * Handles TCP/IP and UDP communication for supervisory control, diagnostics,
 * and configuration. This interface is designed for slower, non-deterministic
 * network protocols and should be processed separately from real-time tasks.
 *
 * Usage Pattern:
 * - Called from main loop or low-priority task (e.g., yeet_network())
 * - Board support layer polls network controller for incoming data
 * - Populates rx_buffer when data received
 * - Application processes command/data
 * - Responds via network_outputs_t
 *
 * Typical Use Cases:
 * - HMI/SCADA communication
 * - Remote configuration updates
 * - Diagnostic data requests
 * - Parameter tuning
 * - Fault log retrieval
 */
typedef struct {
    /**
     * @brief Network receive buffer and metadata
     */
    struct {
        uint8_t rx_buffer[MAX_NETWORK_TX_BUFFER_SIZE]; /**< Received packet data */
        uint16_t rx_length;                            /**< Number of bytes received (0 = no data) */
        bool data_ready;                               /**< true = new data available */
        bool tcp_connected;                            /**< true = TCP connection established */
        uint8_t socket_id;                             /**< Socket identifier (0-7 for multi-socket controllers) */
    } network_rx;

} network_inputs_t;

/**
 * @brief Network output structure (non-real-time communication)
 *
 * Handles TCP/IP and UDP transmission for supervisory responses, status updates,
 * and diagnostic data. Processed separately from real-time control loops.
 *
 * Usage Pattern:
 * - Application prepares response data in tx_buffer
 * - Sets tx_length and transmit_request flag
 * - Board support layer transmits via network controller
 * - Clears transmit_request after successful send
 *
 * Typical Use Cases:
 * - Status reports to HMI/SCADA
 * - Diagnostic data responses
 * - Configuration acknowledgments
 * - Periodic telemetry updates
 * - Event notifications
 */
typedef struct {
    /**
     * @brief Network transmit buffer and metadata
     */
    struct {
        uint8_t tx_buffer[MAX_NETWORK_TX_BUFFER_SIZE]; /**< Data to transmit */
        uint16_t tx_length;                            /**< Number of bytes to send */
        bool transmit_request;                         /**< true = transmit this cycle */
        uint8_t socket_id;                             /**< Target socket identifier (0-7 for multi-socket) */
    } network_tx;

} network_outputs_t;

#ifdef __cplusplus
}  // namespace yeet::telemetry
#endif