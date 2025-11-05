#pragma once

#include "yeet_control/elapsed_timer.hpp"
#include "yeet_control/rate_of_change_limiter.hpp"
#include "yeet_control/types.hpp"
#include "yeet_control/units.hpp"

namespace yeet::application::sfc {

/**
 * @brief Torque controller for LCI drive systems
 *
 * This controller sits between the speed controller and the current controller,
 * providing a clean separation of torque-level operations from current regulation.
 *
 * Key responsibilities:
 * - Torque reference arbitration (speed controller vs. manual override)
 * - Breakaway torque boost injection for starting
 * - Dynamic torque limiting from soft-starter profiles
 * - Torque rate limiting (slew rate)
 * - Torque-to-current conversion
 * - Saturation feedback for anti-windup coordination
 *
 * The controller operates in per-unit values and enforces both static absolute
 * limits and dynamic limits that can change during operation (e.g., from
 * soft-starter profiles).
 */
class sfc_torque_controller {
public:
    /**
     * @brief Operating mode for torque reference source
     */
    enum class control_mode {
        /// Torque reference comes from speed controller (normal operation)
        SPEED_CONTROL,
        /// Torque reference comes from manual setpoint (commissioning/testing)
        MANUAL_TORQUE
    };

    /**
     * @brief Constructs the torque controller
     * @param sampling_time The sampling time of the control loop [s]
     */
    explicit sfc_torque_controller(duration_t sampling_time);

    // ========== Configuration Methods ==========

    /**
     * @brief Sets the absolute torque reference limits
     *
     * These are the hard limits that apply regardless of operating mode or
     * dynamic limits. They represent the physical machine capabilities.
     *
     * @param min_torque_pu Minimum (most negative) torque limit [pu]
     * @param max_torque_pu Maximum (most positive) torque limit [pu]
     */
    void configure_absolute_torque_limits(torque_pu_t min_torque_pu, torque_pu_t max_torque_pu);

    /**
     * @brief Sets the torque-to-current conversion factor
     *
     * This factor converts torque in per-unit to current in per-unit.
     * For many machines, this is approximately 1.0 in a well-designed per-unit system.
     *
     * @param torque_to_current_factor Conversion factor [dimensionless]
     */
    void configure_torque_to_current_factor(real_t torque_to_current_factor);

    /**
     * @brief Sets the maximum rate of change for torque reference
     *
     * This limits how fast the torque can change, preventing mechanical shock
     * and excessive current transients.
     *
     * @param rate_limit_pu_per_second Maximum torque rate [pu/s]
     */
    void configure_torque_rate_limit(real_t rate_limit_pu_per_second);

    /**
     * @brief Enables or disables the torque rate limiter
     * @param enable True to enable rate limiting, false to disable
     */
    void enable_torque_rate_limiting(bool enable);

    /**
     * @brief Sets the operating mode (speed control vs. manual torque)
     * @param mode The desired control mode
     */
    void set_control_mode(control_mode mode);

    /**
     * @brief Sets the manual torque reference (only used in MANUAL_TORQUE mode)
     * @param torque_pu Manual torque setpoint [pu]
     */
    void set_manual_torque_reference(torque_pu_t torque_pu);

    /**
     * @brief Configures the breakaway torque boost parameters
     *
     * The breakaway boost adds extra torque at startup to overcome static friction.
     * It is typically enabled by the soft-starter speed generator.
     *
     * @param boost_torque_pu Additional torque to add during boost [pu]
     * @param boost_duration Duration of boost application [s]
     */
    void configure_breakaway_boost(torque_pu_t boost_torque_pu, duration_t boost_duration);

    // ========== Real-Time Execution ==========

    /**
     * @brief Resets all internal states to initial conditions
     *
     * This resets rate limiters, timers, and clears all dynamic states.
     * Configuration parameters are preserved.
     */
    void reset();

    /**
     * @brief Executes one update cycle of the torque controller
     *
     * Call order: speed.update() → torque.update() → current.update()
     *             → torque.set_downstream_saturation(current.is_saturated())
     *
     * @param torque_reference Torque command from outer loop [pu]
     * @param dynamic_max_torque_limit Upper limit from soft-starter or other sources [pu]
     * @param dynamic_min_torque_limit Lower limit from soft-starter or other sources [pu]
     * @param enable_breakaway_boost True to activate breakaway boost, false to disable
     */
    void update(torque_pu_t torque_reference,
        torque_pu_t dynamic_max_torque_limit,
        torque_pu_t dynamic_min_torque_limit,
        bool enable_breakaway_boost);

    /**
     * @brief Sets the downstream saturation status from the current controller
     *
     * This must be called after the current controller update in the same cycle
     * to provide anti-windup feedback. The saturation status influences the
     * composite saturation flag returned by is_saturated().
     *
     * @param saturated True if the current controller is saturated
     */
    void set_downstream_saturation(bool saturated) noexcept;

    // ========== Output Accessors ==========

    /**
     * @brief Gets the current reference output to the current controller
     * @return Current reference [pu]
     */
    current_pu_t get_current_reference() const;

    /**
     * @brief Gets the effective torque reference before boost injection
     *
     * This is the torque reference after arbitration (speed control vs. manual)
     * but before breakaway boost is added.
     *
     * @return Base torque reference [pu]
     */
    torque_pu_t get_base_torque_reference() const;

    /**
     * @brief Gets the total torque reference after boost injection
     *
     * This is the actual torque being commanded, including any breakaway boost.
     *
     * @return Total torque reference [pu]
     */
    torque_pu_t get_total_torque_reference() const;

    /**
     * @brief Gets the limited torque reference after applying all limits
     *
     * This is the torque after both absolute and dynamic limits are applied,
     * and after rate limiting.
     *
     * @return Limited torque reference [pu]
     */
    torque_pu_t get_limited_torque_reference() const;

    /**
     * @brief Gets the current operating mode
     * @return The active control mode
     */
    control_mode get_control_mode() const;

    /**
     * @brief Checks if breakaway boost is currently active
     * @return True if boost is being applied, false otherwise
     */
    bool is_breakaway_boost_active() const;

    /**
     * @brief Checks if the torque controller output is saturated
     *
     * Saturation occurs when the requested torque exceeds the dynamic limits.
     * This signal can be used for anti-windup in the speed controller.
     *
     * @return True if saturated, false otherwise
     */
    bool is_saturated() const;

    /**
     * @brief Gets the downstream saturation status
     *
     * Returns the saturation status from the current controller that was
     * set via set_downstream_saturation().
     *
     * @return True if downstream controller is saturated
     */
    bool get_downstream_saturation() const noexcept;

    /**
     * @brief Gets the composite saturation status
     *
     * Returns true if either this torque controller is saturated OR
     * the downstream current controller is saturated. This provides
     * a unified saturation signal for anti-windup coordination.
     *
     * @return True if any controller in the chain is saturated
     */
    bool get_composite_saturation() const noexcept;

private:
    // ========== Internal Helper Methods ==========

    /**
     * @brief Applies all torque limits (absolute and dynamic)
     * @param torque_pu Input torque to be limited
     * @return Limited torque [pu]
     */
    torque_pu_t apply_torque_limits(torque_pu_t torque_pu) const;

    // ========== Control Components ==========
    yeet::control::rate_of_change_limiter _torque_rate_limiter;
    yeet::control::elapsed_timer _breakaway_boost_timer;

    // ========== Configuration Parameters ==========
    duration_t _sampling_time;
    torque_pu_t _absolute_min_torque{-1.2};
    torque_pu_t _absolute_max_torque{1.2};
    real_t _torque_to_current_factor{1.0};
    control_mode _control_mode{control_mode::SPEED_CONTROL};
    torque_pu_t _manual_torque_reference{0.0};

    // Breakaway boost configuration
    torque_pu_t _breakaway_boost_torque{0.0};
    duration_t _breakaway_boost_duration{0.0};

    // ========== Dynamic State ==========
    torque_pu_t _base_torque_reference{0.0};
    torque_pu_t _total_torque_reference{0.0};
    torque_pu_t _limited_torque_reference{0.0};
    current_pu_t _current_reference{0.0};
    bool _is_breakaway_boost_active{false};
    bool _is_saturated{false};
    bool _downstream_saturation{false};

    // Dynamic limits (updated each cycle)
    torque_pu_t _dynamic_max_torque_limit{1.2};
    torque_pu_t _dynamic_min_torque_limit{-1.2};
};

}  // namespace yeet::application::sfc
