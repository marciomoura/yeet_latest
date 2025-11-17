/**
 * @file telemetry_packet_buffer.hpp
 * @brief Thread-safe double-buffered telemetry packet queue for host simulator
 *
 * This buffer allows the real-time simulation loop to collect telemetry packets
 * at high frequency (e.g., 50µs) without blocking on TCP transmission.
 * A background thread periodically flushes the accumulated packets to TCP.
 *
 * Architecture:
 * - Write buffer: Active buffer where simulation writes packets (lock-free writes)
 * - Read buffer: Inactive buffer being transmitted by background thread
 * - Atomic swap: Thread-safe buffer exchange when transmission is complete
 */

#pragma once

#include <atomic>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <vector>

namespace yeet::application::sfc::host {

/**
 * @brief Single telemetry packet with metadata
 */
struct buffered_packet {
    static constexpr std::size_t max_packet_size = 1024;  // Maximum packet size in bytes
    std::uint8_t data[max_packet_size];                   // Packet data
    std::uint16_t length;                                 // Actual packet length
    std::uint8_t packet_type;                             // Packet type (for debugging)

    buffered_packet() : length(0), packet_type(0) { std::memset(data, 0, max_packet_size); }
};

/**
 * @brief Thread-safe double-buffered telemetry packet queue
 *
 * Allows real-time loop to write packets without blocking on TCP I/O.
 * Background thread periodically reads accumulated packets and transmits.
 *
 * Thread safety:
 * - Write operations (enqueue) are protected by write_mutex
 * - Buffer swap is protected by swap_mutex
 * - Read operations assume single reader thread
 */
class telemetry_packet_buffer {
public:
    /**
     * @brief Construct buffer with capacity
     * @param capacity Maximum number of packets per buffer
     */
    explicit telemetry_packet_buffer(std::size_t capacity = 1000) : capacity_(capacity)
    {
        write_buffer_.reserve(capacity);
        read_buffer_.reserve(capacity);
    }

    /**
     * @brief Enqueue a packet to the write buffer
     *
     * Called from real-time loop at high frequency (e.g., 50µs).
     * Non-blocking if buffer has space.
     *
     * @param packet_data Pointer to packet data
     * @param packet_length Length of packet in bytes
     * @param packet_type Packet type identifier
     * @return true if packet was enqueued, false if buffer is full
     */
    bool enqueue(const std::uint8_t* packet_data, std::uint16_t packet_length, std::uint8_t packet_type) noexcept
    {
        std::lock_guard<std::mutex> lock(write_mutex_);

        // Drop packet if buffer is full (prevent unbounded growth)
        if (write_buffer_.size() >= capacity_) {
            dropped_packets_.fetch_add(1, std::memory_order_relaxed);
            return false;
        }

        // Copy packet to buffer
        buffered_packet pkt;
        pkt.length = packet_length;
        pkt.packet_type = packet_type;
        std::memcpy(pkt.data, packet_data, packet_length);

        write_buffer_.push_back(pkt);
        enqueued_packets_.fetch_add(1, std::memory_order_relaxed);

        return true;
    }

    /**
     * @brief Swap buffers and get packets ready for transmission
     *
     * Called from background thread at transmission rate (e.g., 60 Hz).
     * Swaps write/read buffers atomically, returns packets to transmit.
     *
     * @return Vector of packets ready to transmit (moved from read buffer)
     */
    std::vector<buffered_packet> swap_and_get_packets() noexcept
    {
        std::vector<buffered_packet> packets_to_send;

        {
            // Lock both buffers during swap to ensure atomicity
            std::lock_guard<std::mutex> write_lock(write_mutex_);
            std::lock_guard<std::mutex> swap_lock(swap_mutex_);

            // Swap buffers
            std::swap(write_buffer_, read_buffer_);
        }

        // Move read buffer contents (now safe to access without lock)
        packets_to_send = std::move(read_buffer_);
        read_buffer_.clear();

        return packets_to_send;
    }

    /**
     * @brief Get buffer statistics
     */
    struct statistics {
        std::size_t enqueued_packets;     // Total packets enqueued
        std::size_t transmitted_packets;  // Total packets transmitted
        std::size_t dropped_packets;      // Total packets dropped due to overflow
        std::size_t current_write_size;   // Current write buffer size
    };

    statistics get_statistics() const noexcept
    {
        statistics stats;
        stats.enqueued_packets = enqueued_packets_.load(std::memory_order_relaxed);
        stats.transmitted_packets = transmitted_packets_.load(std::memory_order_relaxed);
        stats.dropped_packets = dropped_packets_.load(std::memory_order_relaxed);

        {
            std::lock_guard<std::mutex> lock(write_mutex_);
            stats.current_write_size = write_buffer_.size();
        }

        return stats;
    }

    /**
     * @brief Mark packets as transmitted
     * @param count Number of packets successfully transmitted
     */
    void mark_transmitted(std::size_t count) noexcept
    {
        transmitted_packets_.fetch_add(count, std::memory_order_relaxed);
    }

    /**
     * @brief Reset statistics
     */
    void reset_statistics() noexcept
    {
        enqueued_packets_.store(0, std::memory_order_relaxed);
        transmitted_packets_.store(0, std::memory_order_relaxed);
        dropped_packets_.store(0, std::memory_order_relaxed);
    }

private:
    const std::size_t capacity_;  // Maximum packets per buffer

    mutable std::mutex write_mutex_;  // Protects write_buffer_
    mutable std::mutex swap_mutex_;   // Protects buffer swap operation

    std::vector<buffered_packet> write_buffer_;  // Active buffer (simulation writes here)
    std::vector<buffered_packet> read_buffer_;   // Inactive buffer (background thread reads)

    // Statistics (atomic for lock-free reads)
    std::atomic<std::size_t> enqueued_packets_{0};
    std::atomic<std::size_t> transmitted_packets_{0};
    std::atomic<std::size_t> dropped_packets_{0};
};

}  // namespace yeet::application::sfc::host
