#pragma once

#include "yeet_control/threshold_protection.hpp"
#include "yeet_control/units.hpp"

namespace yeet::application::sfc {

/**
 * @brief DC link overcurrent protection for LCI systems.
 *
 * This component monitors the DC link current and provides latching trip protection
 * with configurable warning and trip thresholds. The protection uses time delays
 * to prevent nuisance trips from transient overcurrents.
 *
 * Typical LCI thresholds:
 * - Warning: 120% of rated current (1.2 pu)
 * - Trip: 150% of rated current (1.5 pu)
 * - Trip delay: 50ms
 *
 * The protection latches when tripped and requires explicit reset after the
 * fault condition has cleared.
 */
class sfc_current_protection {
public:
    // Default protection thresholds for LCI systems
    static constexpr current_pu_t default_warning_threshold = current_pu_t{1.2f};  // 120%
    static constexpr current_pu_t default_trip_threshold = current_pu_t{1.5f};     // 150%
    static constexpr duration_t default_trip_delay = duration_t{0.05};             // 50ms
    static constexpr duration_t default_warning_delay = duration_t{0.01};          // 10ms
    static constexpr real_t default_hysteresis_factor = real_t{0.05};              // 5%

    /**
     * @brief Constructs the overcurrent protection with specified sampling period.
     *
     * @param sampling_period The control loop sampling period [s]
     */
    explicit sfc_current_protection(duration_t sampling_period);

    /**
     * @brief Configures the trip threshold and hysteresis.
     *
     * @param trip_threshold DC link current threshold for trip activation [pu]
     * @param hysteresis_factor Hysteresis band as fraction of threshold (default 5%)
     */
    void configure_trip_threshold(current_pu_t trip_threshold, real_t hysteresis_factor = default_hysteresis_factor);

    /**
     * @brief Configures the warning threshold.
     *
     * Warning threshold should be lower than trip threshold.
     *
     * @param warning_threshold DC link current threshold for warning activation [pu]
     */
    void configure_warning_threshold(current_pu_t warning_threshold);

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
     * @brief Updates the protection with the latest DC link current measurement.
     *
     * Evaluates the current against thresholds and manages trip/warning states.
     *
     * @param dc_link_current_pu DC link current in per-unit [pu]
     */
    void update(current_pu_t dc_link_current_pu);

    /**
     * @brief Resets the trip latch if fault condition has cleared.
     *
     * Reset is only allowed when current is below reset threshold.
     */
    void reset();

    /**
     * @brief Gets the latched trip status.
     *
     * @return True if overcurrent protection has tripped and remains latched
     */
    [[nodiscard]] bool is_tripped() const noexcept;

    /**
     * @brief Gets the non-latched warning status.
     *
     * @return True if current currently exceeds warning threshold
     */
    [[nodiscard]] bool is_warning() const noexcept;

    /**
     * @brief Checks if protection can be reset.
     *
     * Reset is only allowed when current is below reset threshold (with hysteresis).
     *
     * @return True if fault condition has cleared and reset is allowed
     */
    [[nodiscard]] bool can_reset() const noexcept;

    /**
     * @brief Gets the current DC link current value.
     *
     * @return The most recent DC link current passed to update() [pu]
     */
    [[nodiscard]] current_pu_t get_current_value() const noexcept;

    /**
     * @brief Gets the configured trip threshold.
     *
     * @return The trip threshold value [pu]
     */
    [[nodiscard]] current_pu_t get_trip_threshold() const noexcept;

    /**
     * @brief Gets the configured warning threshold.
     *
     * @return The warning threshold value [pu]
     */
    [[nodiscard]] current_pu_t get_warning_threshold() const noexcept;

private:
    control::threshold_protection<current_pu_t> _protection;
};

}  // namespace yeet::application::sfc
