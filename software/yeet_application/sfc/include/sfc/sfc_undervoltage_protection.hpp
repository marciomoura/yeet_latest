#pragma once

#include "yeet_control/threshold_protection.hpp"
#include "yeet_control/units.hpp"

namespace yeet::application::sfc {

/**
 * @brief Grid-side undervoltage protection for LCI systems.
 *
 * This component monitors the grid voltage magnitude and provides latching trip
 * protection when voltage falls below configurable thresholds. The protection uses
 * time delays to ride through brief voltage sags.
 *
 * Typical LCI thresholds (grid-side):
 * - Warning: 85% of rated voltage (0.85 pu)
 * - Trip: 70% of rated voltage (0.70 pu)
 * - Trip delay: 500ms (ride through brief sags)
 *
 * The protection latches when tripped and requires explicit reset after the
 * fault condition has cleared.
 */
class sfc_undervoltage_protection {
public:
    // Default protection thresholds for LCI grid-side undervoltage
    static constexpr voltage_pu_t default_warning_threshold = voltage_pu_t{0.85f};  // 85%
    static constexpr voltage_pu_t default_trip_threshold = voltage_pu_t{0.70f};     // 70%
    static constexpr duration_t default_trip_delay = duration_t{0.5};               // 500ms
    static constexpr duration_t default_warning_delay = duration_t{0.1};            // 100ms
    static constexpr real_t default_hysteresis_factor = real_t{0.05};               // 5%

    /**
     * @brief Constructs the undervoltage protection with specified sampling period.
     *
     * @param sampling_period The control loop sampling period [s]
     */
    explicit sfc_undervoltage_protection(duration_t sampling_period);

    /**
     * @brief Configures the trip threshold and hysteresis.
     *
     * @param trip_threshold Grid voltage threshold for trip activation (trips below this) [pu]
     * @param hysteresis_factor Hysteresis band as fraction of threshold (default 5%)
     */
    void configure_trip_threshold(voltage_pu_t trip_threshold, real_t hysteresis_factor = default_hysteresis_factor);

    /**
     * @brief Configures the warning threshold.
     *
     * Warning threshold should be higher than trip threshold (less severe).
     *
     * @param warning_threshold Grid voltage threshold for warning activation (warns below this) [pu]
     */
    void configure_warning_threshold(voltage_pu_t warning_threshold);

    /**
     * @brief Configures the time delay before trip activation.
     *
     * @param trip_delay Time delay for trip activation [s]
     */
    void configure_trip_delay(duration_t trip_delay);

    /**
     * @brief Configures the time delay before warning activation.
     *
     * @param warning_delay Time delay for warning activation [s]
     */
    void configure_warning_delay(duration_t warning_delay);

    /**
     * @brief Updates the protection with the latest grid voltage measurement.
     *
     * Evaluates the voltage against thresholds and manages trip/warning states.
     *
     * @param grid_voltage_magnitude_pu Grid voltage magnitude in per-unit [pu]
     */
    void update(voltage_pu_t grid_voltage_magnitude_pu);

    /**
     * @brief Resets the trip latch if fault condition has cleared.
     *
     * Reset is only allowed when voltage is above reset threshold.
     */
    void reset();

    /**
     * @brief Gets the latched trip status.
     *
     * @return True if undervoltage protection has tripped and remains latched
     */
    [[nodiscard]] bool is_tripped() const noexcept;

    /**
     * @brief Gets the non-latched warning status.
     *
     * @return True if voltage currently below warning threshold
     */
    [[nodiscard]] bool is_warning() const noexcept;

    /**
     * @brief Checks if protection can be reset.
     *
     * Reset is only allowed when voltage is above reset threshold (with hysteresis).
     *
     * @return True if fault condition has cleared and reset is allowed
     */
    [[nodiscard]] bool can_reset() const noexcept;

    /**
     * @brief Gets the current grid voltage magnitude value.
     *
     * @return The most recent grid voltage magnitude passed to update() [pu]
     */
    [[nodiscard]] voltage_pu_t get_current_value() const noexcept;

    /**
     * @brief Gets the configured trip threshold.
     *
     * @return The trip threshold value [pu]
     */
    [[nodiscard]] voltage_pu_t get_trip_threshold() const noexcept;

    /**
     * @brief Gets the configured warning threshold.
     *
     * @return The warning threshold value [pu]
     */
    [[nodiscard]] voltage_pu_t get_warning_threshold() const noexcept;

private:
    /// Underlying generic protection configured for 'below' threshold behavior
    control::threshold_protection<voltage_pu_t, control::threshold_direction::below> _protection;
};

}  // namespace yeet::application::sfc
