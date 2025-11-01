/**
 * @file host_tcp_server.cpp
 * @brief TCP server implementation for host platform using POSIX sockets
 */

#ifdef HOST_BUILD

#include "sfc_embedded/host_platform/host_tcp_server.hpp"

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
    : server_socket_(INVALID_SOCKET_VALUE), client_socket_(INVALID_SOCKET_VALUE), port_(0), host_()
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

    // Set client socket to non-blocking for receive operations
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

    // #ifdef HOST_BUILD
    //     // Debug: print first 32 bytes being sent
    //     printf("[SEND] Sending %zu bytes. First 32: ", len);
    //     for (size_t i = 0; i < 32 && i < len; i++) {
    //         printf("%02x ", data[i]);
    //     }
    //     printf("\n");
    // #endif

    std::size_t total_sent = 0;

    while (total_sent < len) {
        ssize_t sent = send(
            client_socket_, reinterpret_cast<const char*>(data + total_sent), static_cast<int>(len - total_sent), 0);

        if (sent < 0) {
            std::cerr << "Send error, disconnecting client\n";
            disconnect_client();
            return false;
        }

        total_sent += sent;
    }

    return true;
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
