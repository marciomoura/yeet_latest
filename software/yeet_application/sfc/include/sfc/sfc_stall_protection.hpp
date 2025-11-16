#pragma once

#include "yeet_control/hysteresis_limiter.hpp"
#include "yeet_control/on_delay.hpp"
#include "yeet_control/units.hpp"

namespace yeet::application::sfc {

/**
 * @brief Stall protection for LCI startup sequences.
 *
 * This component monitors for stall conditions during motor startup, where the
 * rotor fails to accelerate despite applied torque. A stall condition occurs
 * when high current is drawn but the speed remains low, indicating mechanical
 * or electrical faults that prevent normal acceleration.
 *
 * Physical phenomena:
 * - Mechanical stall: Seized bearing, blade rub, or excessive load torque
 * - Electrical stall: Shorted rotor winding or loss of excitation
 * - Consequence: Prolonged high current at stall → rapid thermal damage to
 *   motor windings and power electronics
 *
 * Stall detection logic:
 * - Stall condition = (Current > threshold) AND (Speed < threshold) AND (Startup active)
 * - Time delay filters transient conditions during normal acceleration
 * - State-dependent: Only active when startup sequence is running
 *
 * Typical LCI thresholds:
 * - Stall current threshold: 130% of rated current (1.3 pu)
 * - Stall speed threshold: 10% of rated speed (0.10 pu)
 * - Stall trip delay: 3.0 seconds
 *
 * The protection latches when tripped and requires explicit reset after the
 * stall condition has cleared.
 */
class sfc_stall_protection {
public:
    // Default protection thresholds for LCI stall protection
    static constexpr current_pu_t default_stall_current_threshold = current_pu_t{1.3};  // 130% current
    static constexpr speed_pu_t default_stall_speed_threshold = speed_pu_t{0.10};       // 10% speed
    static constexpr duration_t default_stall_trip_delay = duration_t{3.0};             // 3.0 seconds
    static constexpr duration_t default_stall_warning_delay = duration_t{1.5};          // 1.5 seconds
    static constexpr real_t default_current_hysteresis = real_t{0.05};                  // 5%
    static constexpr real_t default_speed_hysteresis = real_t{0.02};                    // 2%

    /**
     * @brief Constructs the stall protection with specified sampling period.
     *
     * @param sampling_period The control loop sampling period [s]
     */
    explicit sfc_stall_protection(duration_t sampling_period);

    /**
     * @brief Configures the current threshold for stall detection.
     *
     * Current must exceed this threshold (along with low speed and startup active)
     * for a stall condition to be detected.
     *
     * @param current_threshold Minimum current for stall condition [pu]
     * @param hysteresis_factor Hysteresis band as fraction of threshold (default 5%)
     */
    void configure_current_threshold(
        current_pu_t current_threshold, real_t hysteresis_factor = default_current_hysteresis);

    /**
     * @brief Configures the speed threshold for stall detection.
     *
     * Speed must be below this threshold (along with high current and startup active)
     * for a stall condition to be detected.
     *
     * @param speed_threshold Maximum speed for stall condition [pu]
     * @param hysteresis_factor Hysteresis band as fraction of threshold (default 2%)
     */
    void configure_speed_threshold(speed_pu_t speed_threshold, real_t hysteresis_factor = default_speed_hysteresis);

    /**
     * @brief Configures the time delay before stall trip activation.
     *
     * This delay filters out transient conditions during normal acceleration.
     *
     * @param trip_delay Time delay for stall trip [s]
     */
    void configure_trip_delay(duration_t trip_delay);

    /**
     * @brief Configures the time delay before stall warning activation.
     *
     * Warning activates at the same thresholds as trip, but with shorter delay.
     *
     * @param warning_delay Time delay for stall warning [s]
     */
    void configure_warning_delay(duration_t warning_delay);

    /**
     * @brief Updates the protection with latest measurements and startup state.
     *
     * Evaluates stall condition: (Current > threshold) AND (Speed < threshold) AND (Startup active)
     * The protection is only active when startup_active is true, preventing false
     * trips during normal low-speed operation (e.g., synchronized idle).
     *
     * @param measured_current DC link or machine current [pu]
     * @param measured_speed Rotor speed [pu]
     * @param startup_active True when startup sequence is running
     */
    void update(current_pu_t measured_current, speed_pu_t measured_speed, bool startup_active);

    /**
     * @brief Resets the stall trip latch if stall condition has cleared.
     *
     * Reset is only allowed when stall condition is no longer active.
     */
    void reset();

    /**
     * @brief Gets the latched stall trip status.
     *
     * @return True if stall protection has tripped and remains latched
     */
    [[nodiscard]] bool is_tripped() const noexcept;

    /**
     * @brief Gets the non-latched stall warning status.
     *
     * Warning uses the same thresholds as trip but with shorter delay.
     *
     * @return True if stall condition currently active (warning level)
     */
    [[nodiscard]] bool is_warning() const noexcept;

    /**
     * @brief Checks if protection can be reset.
     *
     * Reset is allowed when stall condition has cleared (current below threshold
     * OR speed above threshold OR startup not active).
     *
     * @return True if stall condition cleared and reset is allowed
     */
    [[nodiscard]] bool can_reset() const noexcept;

    /**
     * @brief Gets the current stall condition status (non-latched).
     *
     * @return True if stall condition currently active (immediate, not delayed)
     */
    [[nodiscard]] bool is_stall_condition_active() const noexcept;

    /**
     * @brief Gets the current measured current value.
     *
     * @return The most recent current value passed to update() [pu]
     */
    [[nodiscard]] current_pu_t get_current_value() const noexcept;

    /**
     * @brief Gets the current measured speed value.
     *
     * @return The most recent speed value passed to update() [pu]
     */
    [[nodiscard]] speed_pu_t get_speed_value() const noexcept;

    /**
     * @brief Gets the configured current threshold.
     *
     * @return The current threshold for stall detection [pu]
     */
    [[nodiscard]] current_pu_t get_current_threshold() const noexcept;

    /**
     * @brief Gets the configured speed threshold.
     *
     * @return The speed threshold for stall detection [pu]
     */
    [[nodiscard]] speed_pu_t get_speed_threshold() const noexcept;

private:
    // Parameters
    current_pu_t _current_threshold{default_stall_current_threshold};
    speed_pu_t _speed_threshold{default_stall_speed_threshold};

    // State
    control::on_delay _trip_timer;
    control::on_delay _warning_timer;
    control::hysteresis_limiter _current_limiter;  // High current detection with hysteresis
    control::hysteresis_limiter _speed_limiter;    // Low speed detection with hysteresis
    bool _is_tripped{false};
    bool _stall_condition_active{false};
    current_pu_t _current_value{};
    speed_pu_t _speed_value{};

    /**
     * @brief Internal helper: Checks if stall condition is active using hysteresis limiter outputs.
     */
    [[nodiscard]] bool check_stall_condition(bool startup_active) const noexcept;
};

}  // namespace yeet::application::sfc
