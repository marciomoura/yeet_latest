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
#include <deque>
#include <mutex>
#include <string>
#include <vector>

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
     * @brief Set verbose logging mode
     *
     * @param verbose true to enable verbose logging, false to disable
     */
    void set_verbose(bool verbose) noexcept { verbose_ = verbose; }

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
     * @brief Send data to connected client (non-blocking with queue)
     *
     * @param data Pointer to data to send
     * @param len Number of bytes to send
     * @return true if data queued/sent successfully, false if queue full or error
     *
     * Non-blocking operation - queues data for transmission. If send queue
     * is full (client too slow), oldest data is dropped.
     */
    bool send_data(const std::uint8_t* data, std::size_t len);

    /**
     * @brief Flush pending send queue (called automatically in process())
     *
     * Attempts to send queued data without blocking. Returns number of
     * bytes actually transmitted.
     *
     * @return Number of bytes transmitted from queue
     */
    std::size_t flush_send_queue();

    /**
     * @brief Get number of bytes currently queued for transmission
     *
     * @return Bytes waiting in send queue
     */
    std::size_t get_send_queue_size() const;

    /**
     * @brief Get number of times send queue overflowed (data dropped)
     *
     * @return Count of queue overflow events
     */
    std::uint32_t get_queue_overflow_count() const { return queue_overflow_count_; }

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

    /**
     * @brief Attempt to send data immediately (non-blocking)
     *
     * @param data Pointer to data
     * @param len Number of bytes to send
     * @return Number of bytes actually sent (may be less than len)
     */
    std::size_t try_send_immediate(const std::uint8_t* data, std::size_t len);

private:
    int server_socket_;     ///< Server socket descriptor (-1 if not initialized)
    int client_socket_;     ///< Client socket descriptor (-1 if not connected)
    std::uint16_t port_;    ///< Server port number
    std::string host_;      ///< Server bind address
    bool verbose_ = false;  ///< Verbose logging mode

    // Send queue for non-blocking operation
    static constexpr std::size_t MAX_QUEUE_SIZE = 10 * 1024 * 1024;  ///< 10 MB max queue
    std::deque<std::uint8_t> send_queue_;                            ///< Pending data to send
    mutable std::mutex queue_mutex_;                                 ///< Protects send queue
    std::uint32_t queue_overflow_count_;                             ///< Number of queue overflows

    // Pre-allocated buffer for flushing queue to avoid heap churn
    std::vector<std::uint8_t> flush_buffer_;
};

}  // namespace yeet::application::sfc::host

#endif  // HOST_BUILD
