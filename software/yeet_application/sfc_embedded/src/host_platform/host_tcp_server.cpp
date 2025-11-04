/**
 * @file host_tcp_server.cpp
 * @brief TCP server implementation for host platform using POSIX sockets
 */

#ifdef HOST_BUILD

#include "sfc_embedded/host_platform/host_tcp_server.hpp"

#include <chrono>
#include <cstring>
#include <iostream>

// POSIX socket headers
#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")
using ssize_t = SSIZE_T;
#else
#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

namespace yeet::application::sfc::host {

namespace {
// Windows requires WSA initialization
#ifdef _WIN32
struct wsa_initializer {
    wsa_initializer()
    {
        WSADATA wsa_data;
        if (WSAStartup(MAKEWORD(2, 2), &wsa_data) != 0) {
            std::cerr << "WSAStartup failed\n";
        }
    }
    ~wsa_initializer() { WSACleanup(); }
};
static wsa_initializer wsa_init;
#endif

constexpr int INVALID_SOCKET_VALUE = -1;
}  // namespace

host_tcp_server::host_tcp_server()
    : server_socket_(INVALID_SOCKET_VALUE),
      client_socket_(INVALID_SOCKET_VALUE),
      port_(0),
      host_(),
      queue_overflow_count_(0)
{
}

host_tcp_server::~host_tcp_server() { shutdown(); }

bool host_tcp_server::initialize(std::uint16_t port, const std::string& host)
{
    if (is_initialized()) {
        std::cerr << "TCP server already initialized\n";
        return false;
    }

    port_ = port;
    host_ = host;

    // Create server socket
    server_socket_ = static_cast<int>(socket(AF_INET, SOCK_STREAM, 0));
    if (server_socket_ < 0) {
        std::cerr << "Failed to create server socket\n";
        return false;
    }

    // Set socket options - allow address reuse
    int opt = 1;
    if (setsockopt(server_socket_, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char*>(&opt), sizeof(opt)) < 0) {
        std::cerr << "Failed to set SO_REUSEADDR\n";
#ifdef _WIN32
        closesocket(server_socket_);
#else
        close(server_socket_);
#endif
        server_socket_ = INVALID_SOCKET_VALUE;
        return false;
    }

    // Bind to address and port
    sockaddr_in server_addr{};
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(port_);

    if (host_ == "0.0.0.0") {
        server_addr.sin_addr.s_addr = INADDR_ANY;
    }
    else {
        if (inet_pton(AF_INET, host_.c_str(), &server_addr.sin_addr) <= 0) {
            std::cerr << "Invalid bind address: " << host_ << "\n";
#ifdef _WIN32
            closesocket(server_socket_);
#else
            close(server_socket_);
#endif
            server_socket_ = INVALID_SOCKET_VALUE;
            return false;
        }
    }

    if (bind(server_socket_, reinterpret_cast<sockaddr*>(&server_addr), sizeof(server_addr)) < 0) {
        std::cerr << "Failed to bind to " << host_ << ":" << port_ << "\n";
#ifdef _WIN32
        closesocket(server_socket_);
#else
        close(server_socket_);
#endif
        server_socket_ = INVALID_SOCKET_VALUE;
        return false;
    }

    // Start listening
    if (listen(server_socket_, 1) < 0) {
        std::cerr << "Failed to listen on socket\n";
#ifdef _WIN32
        closesocket(server_socket_);
#else
        close(server_socket_);
#endif
        server_socket_ = INVALID_SOCKET_VALUE;
        return false;
    }

    // Set server socket to non-blocking
    if (!set_nonblocking(server_socket_)) {
        std::cerr << "Failed to set server socket non-blocking\n";
#ifdef _WIN32
        closesocket(server_socket_);
#else
        close(server_socket_);
#endif
        server_socket_ = INVALID_SOCKET_VALUE;
        return false;
    }

    std::cout << "TCP server listening on " << host_ << ":" << port_ << "\n";
    return true;
}

void host_tcp_server::process()
{
    if (!is_initialized()) {
        return;
    }

    // If no client connected, try to accept new connection
    if (!is_client_connected()) {
        accept_client();
    }
    else {
        // Flush pending data to client
        flush_send_queue();
    }
}

bool host_tcp_server::accept_client()
{
    if (!is_initialized() || is_client_connected()) {
        return false;
    }

    sockaddr_in client_addr{};
    socklen_t client_len = sizeof(client_addr);

    int new_socket = static_cast<int>(accept(server_socket_, reinterpret_cast<sockaddr*>(&client_addr), &client_len));

    if (new_socket < 0) {
        // No pending connection (non-blocking)
        return false;
    }

    client_socket_ = new_socket;

    // Set client socket to non-blocking
    set_nonblocking(client_socket_);

    char client_ip[INET_ADDRSTRLEN];
    inet_ntop(AF_INET, &client_addr.sin_addr, client_ip, sizeof(client_ip));
    std::cout << "Client connected from " << client_ip << ":" << ntohs(client_addr.sin_port) << "\n";

    return true;
}

int host_tcp_server::receive_data(std::uint8_t* buffer, std::size_t max_len)
{
    if (!is_client_connected()) {
        return -1;
    }

    ssize_t received = recv(client_socket_, reinterpret_cast<char*>(buffer), static_cast<int>(max_len), 0);

    if (received < 0) {
#ifdef _WIN32
        int error = WSAGetLastError();
        if (error == WSAEWOULDBLOCK) {
            return 0;  // No data available (non-blocking)
        }
#else
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            return 0;  // No data available (non-blocking)
        }
#endif

        // Error occurred
        std::cerr << "Receive error, disconnecting client\n";
        disconnect_client();
        return -1;
    }

    if (received == 0) {
        // Client disconnected gracefully
        std::cout << "Client disconnected\n";
        disconnect_client();
        return -1;
    }

    return static_cast<int>(received);
}

bool host_tcp_server::send_data(const std::uint8_t* data, std::size_t len)
{
    if (!is_client_connected()) {
        return false;
    }

    std::lock_guard<std::mutex> lock(queue_mutex_);

    // Check if adding this data would overflow queue
    if (send_queue_.size() + len > MAX_QUEUE_SIZE) {
        // Drop oldest data to make room
        std::size_t bytes_to_drop = (send_queue_.size() + len) - MAX_QUEUE_SIZE;
        send_queue_.erase(send_queue_.begin(), send_queue_.begin() + bytes_to_drop);
        queue_overflow_count_++;

        // Optionally log warning (limit frequency to avoid spam)
        static auto last_warning = std::chrono::steady_clock::now();
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::seconds>(now - last_warning).count() >= 5) {
            std::cerr << "[TCP] Send queue overflow, dropped " << bytes_to_drop << " bytes (client too slow)\n";
            last_warning = now;
        }
    }

    // Add data to queue
    send_queue_.insert(send_queue_.end(), data, data + len);

    return true;
}

std::size_t host_tcp_server::flush_send_queue()
{
    if (!is_client_connected()) {
        return 0;
    }

    std::lock_guard<std::mutex> lock(queue_mutex_);

    if (send_queue_.empty()) {
        return 0;
    }

    // Try to send as much as possible without blocking
    std::size_t total_sent = 0;
    const std::size_t chunk_size = std::min<std::size_t>(send_queue_.size(), 8192);  // Send up to 8KB per call

    // Copy chunk to temporary buffer (deque doesn't have data() method)
    std::vector<std::uint8_t> buffer(chunk_size);
    std::copy(send_queue_.begin(), send_queue_.begin() + chunk_size, buffer.begin());

    std::size_t sent = try_send_immediate(buffer.data(), chunk_size);

    if (sent > 0) {
        send_queue_.erase(send_queue_.begin(), send_queue_.begin() + sent);
        total_sent += sent;
    }

    return total_sent;
}

std::size_t host_tcp_server::try_send_immediate(const std::uint8_t* data, std::size_t len)
{
    if (!is_client_connected() || len == 0) {
        return 0;
    }

    ssize_t sent = send(client_socket_, reinterpret_cast<const char*>(data), static_cast<int>(len), 0);

    if (sent < 0) {
#ifdef _WIN32
        int error = WSAGetLastError();
        if (error == WSAEWOULDBLOCK) {
            return 0;  // Would block, try again later
        }
#else
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            return 0;  // Would block, try again later
        }
#endif

        // Error occurred
        std::cerr << "[TCP] Send error, disconnecting client\n";
        disconnect_client();
        return 0;
    }

    return static_cast<std::size_t>(sent);
}

std::size_t host_tcp_server::get_send_queue_size() const
{
    std::lock_guard<std::mutex> lock(queue_mutex_);
    return send_queue_.size();
}

void host_tcp_server::disconnect_client()
{
    if (client_socket_ >= 0) {
#ifdef _WIN32
        closesocket(client_socket_);
#else
        close(client_socket_);
#endif
        client_socket_ = INVALID_SOCKET_VALUE;

        // Clear send queue on disconnect
        std::lock_guard<std::mutex> lock(queue_mutex_);
        send_queue_.clear();
    }
}

void host_tcp_server::shutdown()
{
    disconnect_client();

    if (server_socket_ >= 0) {
#ifdef _WIN32
        closesocket(server_socket_);
#else
        close(server_socket_);
#endif
        server_socket_ = INVALID_SOCKET_VALUE;
    }
}

bool host_tcp_server::set_nonblocking(int socket)
{
#ifdef _WIN32
    u_long mode = 1;  // 1 = non-blocking
    if (ioctlsocket(socket, FIONBIO, &mode) != 0) {
        return false;
    }
#else
    int flags = fcntl(socket, F_GETFL, 0);
    if (flags < 0) {
        return false;
    }
    if (fcntl(socket, F_SETFL, flags | O_NONBLOCK) < 0) {
        return false;
    }
#endif

    return true;
}

}  // namespace yeet::application::sfc::host

#endif  // HOST_BUILD
