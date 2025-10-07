#pragma once

#include "types.hpp"

namespace yeet::control {

/**
 * @brief A high-precision timer for measuring time intervals in real-time systems.
 *
 * This class implements a microsecond-precision timer designed for measuring
 * event durations in control systems. It provides deterministic timing
 * measurements suitable for real-time applications such as commutation
 * monitoring in power electronics.
 *
 * The timer operates by accumulating elapsed time based on a fixed sampling
 * period, ensuring predictable behavior in time-critical control loops.
 */
class interval_timer {
public:
    /**
     * @brief Constructs an interval timer with the specified sampling time.
     *
     * @param sampling_time The sampling time in seconds. This must match
     *                     the execution period of the calling control task.
     */
    explicit interval_timer(double sampling_time);

    /**
     * @brief Starts the timer, resetting the elapsed time to zero.
     *
     * Calling this method on an already running timer will restart it.
     */
    void start();

    /**
     * @brief Stops the timer and returns the total elapsed time.
     *
     * @return The elapsed time in microseconds since the timer was started.
     */
    real_t stop();

    /**
     * @brief Updates the timer's elapsed time.
     *
     * This method must be called once per control cycle to accumulate the
     * elapsed time. It should only be called when the timer is running.
     */
    void update();

    /**
     * @brief Checks if the timer is currently running.
     *
     * @return True if the timer is running and accumulating time.
     */
    bool is_running() const;

    /**
     * @brief Gets the current elapsed time without stopping the timer.
     *
     * @return The elapsed time in microseconds since the timer was started.
     */
    real_t get_elapsed_time() const;

    /**
     * @brief Resets the timer to its initial state.
     *
     * The timer is stopped and the elapsed time is reset to zero.
     */
    void reset();

    /**
     * @brief Configures or changes the sampling time of the timer.
     *
     * Calling this method also resets the timer to its initial state.
     *
     * @param sampling_time The new sampling time in seconds.
     */
    void configure(double sampling_time);

private:
    double _sampling_time;         ///< System sampling time in seconds.
    real_t _elapsed_time_us{0.0};  ///< Current elapsed time in microseconds.
    bool _is_running{false};       ///< Flag indicating if the timer is running.
};

}  // namespace yeet::control