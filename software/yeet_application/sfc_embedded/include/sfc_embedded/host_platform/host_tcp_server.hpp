/**
 * @file host_tcp_server.hpp
 * @brief TCP server implementation for host platform (POSIX sockets)
 *
 * This module provides a TCP server implementation using POSIX sockets
 * to replace the W5200 Ethernet controller when running on host platform.
 *
 * Key Features:
 * - Single client connection management
 * - Non-blocking I/O
 * - Compatible API with embedded network interface
 * - Thread-safe operation
 *
 * Usage:
 * @code
 * host_tcp_server server;
 * if (server.initialize(7000)) {
 *     while (running) {
 *         server.process();  // Accept connections, handle disconnects
 *
 *         if (server.is_client_connected()) {
 *             uint8_t buffer[256];
 *             int len = server.receive_data(buffer, sizeof(buffer));
 *             if (len > 0) {
 *                 // Process received data
 *             }
 *
 *             // Send response
 *             server.send_data(response, response_len);
 *         }
 *     }
 * }
 * @endcode
 */

#pragma once

#ifdef HOST_BUILD

#include <cstddef>
#include <cstdint>
#include <string>

namespace yeet::application::sfc::host {

/**
 * @brief TCP server for host platform using POSIX sockets
 *
 * Provides TCP server functionality compatible with the embedded
 * W5200 network interface. Supports single client connection.
 */
class host_tcp_server {
public:
    /**
     * @brief Construct TCP server
     */
    host_tcp_server();

    /**
     * @brief Destructor - closes all sockets
     */
    ~host_tcp_server();

    // Disable copy
    host_tcp_server(const host_tcp_server&) = delete;
    host_tcp_server& operator=(const host_tcp_server&) = delete;

    /**
     * @brief Initialize TCP server and start listening
     *
     * @param port TCP port to listen on (default 7000)
     * @param host IP address to bind to (default "0.0.0.0" = all interfaces)
     * @return true if initialization successful, false otherwise
     */
    bool initialize(std::uint16_t port = 7000, const std::string& host = "0.0.0.0");

    /**
     * @brief Check if server is initialized and listening
     *
     * @return true if server socket is listening
     */
    bool is_initialized() const noexcept { return server_socket_ >= 0; }

    /**
     * @brief Check if client is currently connected
     *
     * @return true if client connected, false otherwise
     */
    bool is_client_connected() const noexcept { return client_socket_ >= 0; }

    /**
     * @brief Process network events (accept connections, detect disconnects)
     *
     * Should be called periodically from main loop. Handles:
     * - Accepting new client connections (if no client connected)
     * - Detecting client disconnections
     *
     * Non-blocking operation.
     */
    void process();

    /**
     * @brief Receive data from connected client
     *
     * @param buffer Destination buffer for received data
     * @param max_len Maximum bytes to receive
     * @return Number of bytes received, 0 if no data available, -1 on error
     *
     * Non-blocking operation. Returns immediately if no data available.
     */
    int receive_data(std::uint8_t* buffer, std::size_t max_len);

    /**
     * @brief Send data to connected client
     *
     * @param data Pointer to data to send
     * @param len Number of bytes to send
     * @return true if all data sent successfully, false on error
     *
     * Blocking operation - waits until all data is sent or error occurs.
     */
    bool send_data(const std::uint8_t* data, std::size_t len);

    /**
     * @brief Get server port number
     *
     * @return Port number server is listening on
     */
    std::uint16_t get_port() const noexcept { return port_; }

    /**
     * @brief Close all connections and shutdown server
     */
    void shutdown();

private:
    /**
     * @brief Accept pending client connection
     *
     * @return true if client accepted, false if no pending connection or error
     */
    bool accept_client();

    /**
     * @brief Close client connection
     */
    void disconnect_client();

    /**
     * @brief Set socket to non-blocking mode
     *
     * @param socket Socket descriptor
     * @return true if successful
     */
    bool set_nonblocking(int socket);

private:
    int server_socket_;   ///< Server socket descriptor (-1 if not initialized)
    int client_socket_;   ///< Client socket descriptor (-1 if not connected)
    std::uint16_t port_;  ///< Server port number
    std::string host_;    ///< Server bind address
};

}  // namespace yeet::application::sfc::host

#endif  // HOST_BUILD
