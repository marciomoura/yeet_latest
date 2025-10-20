/**
 * @file host_timing.hpp
 * @brief Deterministic timing control for host platform simulation
 *
 * Provides a 100 Hz control loop with precise timing using std::chrono
 * and std::thread. Replaces hardware timer interrupts with software timing.
 *
 * Key Features:
 * - Deterministic 10 ms cycle time (100 Hz)
 * - Jitter compensation using sleep_until
 * - Cycle time monitoring and statistics
 * - Overrun detection
 *
 * Usage:
 * @code
 * host_timing_loop loop(100.0f); // 100 Hz
 * loop.start();
 *
 * while (loop.is_running()) {
 *     loop.wait_for_next_cycle();
 *     // Execute control code here
 *     loop.mark_cycle_end();
 * }
 * @endcode
 */

#pragma once

#ifdef HOST_BUILD

#include <atomic>
#include <chrono>
#include <cstdint>
#include <thread>

namespace yeet::application::sfc::host {

/**
 * @brief Timing statistics for monitoring loop performance
 */
struct timing_statistics {
    float average_cycle_time_ms;  ///< Average cycle time [ms]
    float max_cycle_time_ms;      ///< Maximum cycle time [ms]
    float min_cycle_time_ms;      ///< Minimum cycle time [ms]
    std::uint32_t overrun_count;  ///< Number of cycle overruns
    std::uint32_t total_cycles;   ///< Total cycles executed
};

/**
 * @brief Deterministic timing loop for host platform
 *
 * Manages a periodic control loop with precise timing.
 * Uses std::chrono for high-resolution timing and std::thread::sleep_until
 * for jitter-free cycle timing.
 *
 * Thread Safety: Methods can be called from different threads
 */
class host_timing_loop {
public:
    /**
     * @brief Construct timing loop with specified frequency
     *
     * @param frequency_hz Loop frequency in Hz (default: 100 Hz)
     */
    explicit host_timing_loop(float frequency_hz = 100.0f);

    /**
     * @brief Destructor - stops loop if running
     */
    ~host_timing_loop();

    // Non-copyable, non-movable
    host_timing_loop(const host_timing_loop&) = delete;
    host_timing_loop& operator=(const host_timing_loop&) = delete;

    /**
     * @brief Start the timing loop
     *
     * Initializes timing references and enables loop execution.
     * Must be called before wait_for_next_cycle().
     */
    void start();

    /**
     * @brief Stop the timing loop
     *
     * Disables loop execution. wait_for_next_cycle() will return immediately.
     */
    void stop();

    /**
     * @brief Check if loop is running
     *
     * @return true if loop is active, false otherwise
     */
    bool is_running() const;

    /**
     * @brief Wait for next cycle
     *
     * Blocks until the next cycle start time. Uses sleep_until for
     * deterministic timing with minimal jitter.
     *
     * @return true if cycle started normally, false if loop stopped
     */
    bool wait_for_next_cycle();

    /**
     * @brief Mark end of cycle computation
     *
     * Call this after control code execution to measure actual cycle time.
     * Updates statistics and detects overruns.
     */
    void mark_cycle_end();

    /**
     * @brief Get timing statistics
     *
     * @return Current timing statistics
     */
    timing_statistics get_statistics() const;

    /**
     * @brief Reset timing statistics
     *
     * Clears all accumulated statistics (cycle times, overrun count, etc.)
     */
    void reset_statistics();

    /**
     * @brief Get current cycle number
     *
     * @return Total number of cycles executed since start()
     */
    std::uint32_t get_cycle_number() const;

    /**
     * @brief Get elapsed time since start
     *
     * @return Elapsed time in milliseconds
     */
    float get_elapsed_time_ms() const;

    /**
     * @brief Get configured cycle period
     *
     * @return Cycle period in milliseconds
     */
    float get_cycle_period_ms() const;

private:
    using clock = std::chrono::steady_clock;
    using time_point = std::chrono::time_point<clock>;
    using duration = std::chrono::duration<double>;

    float frequency_hz_;                ///< Loop frequency [Hz]
    float cycle_period_ms_;             ///< Cycle period [ms]
    std::chrono::microseconds period_;  ///< Cycle period in microseconds

    std::atomic<bool> running_;    ///< Loop running flag
    time_point start_time_;        ///< Loop start time
    time_point next_cycle_time_;   ///< Next cycle start time
    time_point cycle_start_time_;  ///< Current cycle start time

    // Statistics (protected by atomic or sequential access)
    std::uint32_t cycle_number_;   ///< Current cycle number
    float total_cycle_time_ms_;    ///< Sum of all cycle times
    float max_cycle_time_ms_;      ///< Maximum cycle time observed
    float min_cycle_time_ms_;      ///< Minimum cycle time observed
    std::uint32_t overrun_count_;  ///< Number of overruns

    /**
     * @brief Update statistics with current cycle time
     *
     * @param cycle_time_ms Measured cycle time in milliseconds
     */
    void update_statistics(float cycle_time_ms);
};

/**
 * @brief RAII helper for cycle timing
 *
 * Automatically calls mark_cycle_end() on destruction.
 * Useful for ensuring timing measurement even with early returns.
 *
 * Usage:
 * @code
 * host_timing_loop loop(100.0f);
 * while (loop.wait_for_next_cycle()) {
 *     cycle_timer timer(loop);
 *     // Control code here
 *     // timer automatically marks end on scope exit
 * }
 * @endcode
 */
class cycle_timer {
public:
    explicit cycle_timer(host_timing_loop& loop) : loop_(loop) {}
    ~cycle_timer() { loop_.mark_cycle_end(); }

    // Non-copyable, non-movable
    cycle_timer(const cycle_timer&) = delete;
    cycle_timer& operator=(const cycle_timer&) = delete;

private:
    host_timing_loop& loop_;
};

}  // namespace yeet::application::sfc::host

#endif  // HOST_BUILD
