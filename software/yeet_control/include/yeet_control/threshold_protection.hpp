#pragma once

#include "yeet_control/on_delay.hpp"
#include "yeet_control/types.hpp"
#include "yeet_control/units.hpp"

namespace yeet::control {

/**
 * @brief Threshold comparison direction for protection logic.
 */
enum class threshold_direction {
    above,  ///< Trips when signal exceeds threshold (overcurrent, overvoltage)
    below   ///< Trips when signal falls below threshold (undervoltage)
};

/**
 * @brief Generic threshold-based protection with latching behavior.
 *
 * This component implements a protection function that:
 * - Monitors a signal against configurable trip and warning thresholds
 * - Provides optional time delay for debouncing (prevents nuisance trips)
 * - Latches when trip threshold is exceeded (or falls below for undervoltage)
 * - Remains latched until explicitly reset
 * - Only allows reset when fault condition has cleared
 * - Provides separate non-latched warning output
 *
 * @tparam T The type of the monitored signal (e.g., current_pu_t, voltage_pu_t)
 * @tparam Direction Trip direction: 'above' for overcurrent/overvoltage, 'below' for undervoltage
 */
template <typename T, threshold_direction Direction = threshold_direction::above>
class threshold_protection {
public:
    /**
     * @brief Constructs the threshold protection with specified sampling period.
     *
     * @param sampling_period The control loop sampling period [s]
     */
    explicit threshold_protection(duration_t sampling_period)
        : _sampling_time(sampling_period.value()),
          _trip_timer(real_t{0.0}, _sampling_time),
          _warning_timer(real_t{0.0}, _sampling_time)
    {
    }

    /**
     * @brief Configures the trip threshold and optional hysteresis.
     *
     * For 'above' direction: Reset threshold = trip_threshold * (1 - hysteresis)
     * For 'below' direction: Reset threshold = trip_threshold * (1 + hysteresis)
     *
     * @param trip_threshold The threshold at which protection trips
     * @param hysteresis_factor Hysteresis band as fraction of threshold (default 0.05 = 5%)
     */
    void configure_trip_threshold(T trip_threshold, real_t hysteresis_factor = real_t{0.05})
    {
        _trip_threshold = trip_threshold;

        if constexpr (Direction == threshold_direction::above) {
            // For overcurrent/overvoltage: reset when value drops below (threshold - hysteresis)
            _trip_reset_threshold = trip_threshold * (real_t{1.0} - hysteresis_factor);
        }
        else {
            // For undervoltage: reset when value rises above (threshold + hysteresis)
            _trip_reset_threshold = trip_threshold * (real_t{1.0} + hysteresis_factor);
        }
    }

    /**
     * @brief Configures the warning threshold.
     *
     * Warning threshold should be lower than trip threshold.
     *
     * @param warning_threshold The threshold above which warning is active
     */
    void configure_warning_threshold(T warning_threshold) { _warning_threshold = warning_threshold; }

    /**
     * @brief Configures the time delay before trip activation.
     *
     * @param trip_delay Time delay for trip activation [s]
     */
    void configure_trip_delay(duration_t trip_delay) { _trip_timer.configure(trip_delay.value()); }

    /**
     * @brief Configures the time delay before warning activation.
     *
     * @param warning_delay Time delay for warning activation [s]
     */
    void configure_warning_delay(duration_t warning_delay) { _warning_timer.configure(warning_delay.value()); }

    /**
     * @brief Updates the protection with the latest measured value.
     *
     * Evaluates the input against thresholds and manages trip/warning states.
     *
     * @param measured_value The current measured value to monitor
     */
    void update(T measured_value)
    {
        _current_value = measured_value;

        // Check if value exceeds trip threshold (direction-dependent)
        bool exceeds_trip;
        if constexpr (Direction == threshold_direction::above) {
            exceeds_trip = measured_value > _trip_threshold;
        }
        else {
            exceeds_trip = measured_value < _trip_threshold;
        }

        // Update trip timer (on-delay behavior)
        _trip_timer.update(exceeds_trip);

        // Latch trip condition
        if (_trip_timer.get_output() && !_is_tripped) {
            _is_tripped = true;
        }

        // Check warning threshold (non-latched, direction-dependent)
        bool exceeds_warning;
        if constexpr (Direction == threshold_direction::above) {
            exceeds_warning = measured_value > _warning_threshold;
        }
        else {
            exceeds_warning = measured_value < _warning_threshold;
        }
        _warning_timer.update(exceeds_warning);
    }

    /**
     * @brief Resets the trip latch if fault condition has cleared.
     *
     * Reset is only allowed when measured value is below reset threshold.
     */
    void reset()
    {
        if (can_reset()) {
            _is_tripped = false;
            _trip_timer.reset();
            _warning_timer.reset();
        }
    }

    /**
     * @brief Gets the latched trip status.
     *
     * @return True if protection has tripped and remains latched
     */
    [[nodiscard]] bool is_tripped() const noexcept { return _is_tripped; }

    /**
     * @brief Gets the non-latched warning status.
     *
     * @return True if measured value currently exceeds warning threshold
     */
    [[nodiscard]] bool is_warning() const noexcept { return _warning_timer.get_output(); }

    /**
     * @brief Checks if protection can be reset.
     *
     * For 'above' direction: Reset allowed when current value < reset threshold
     * For 'below' direction: Reset allowed when current value > reset threshold
     *
     * @return True if fault condition has cleared and reset is allowed
     */
    [[nodiscard]] bool can_reset() const noexcept
    {
        if constexpr (Direction == threshold_direction::above) {
            return _current_value < _trip_reset_threshold;
        }
        else {
            return _current_value > _trip_reset_threshold;
        }
    }

    /**
     * @brief Gets the current measured value.
     *
     * @return The most recent measured value passed to update()
     */
    [[nodiscard]] T get_current_value() const noexcept { return _current_value; }

    /**
     * @brief Gets the configured trip threshold.
     *
     * @return The trip threshold value
     */
    [[nodiscard]] T get_trip_threshold() const noexcept { return _trip_threshold; }

    /**
     * @brief Gets the configured warning threshold.
     *
     * @return The warning threshold value
     */
    [[nodiscard]] T get_warning_threshold() const noexcept { return _warning_threshold; }

private:
    double _sampling_time;      ///< System sampling time in seconds
    T _trip_threshold{};        ///< Threshold for trip activation
    T _trip_reset_threshold{};  ///< Reset threshold (trip threshold - hysteresis)
    T _warning_threshold{};     ///< Threshold for warning activation
    T _current_value{};         ///< Most recent measured value
    bool _is_tripped{false};    ///< Latched trip status
    on_delay _trip_timer;       ///< Time delay for trip activation
    on_delay _warning_timer;    ///< Time delay for warning activation
};

}  // namespace yeet::control
