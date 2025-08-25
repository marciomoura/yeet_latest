#pragma once

#include "yeet_control/derivative.hpp"
#include "yeet_control/lookup_table_1d.hpp"
#include "yeet_control/pi_controller.hpp"
#include "yeet_control/rate_of_change_limiter.hpp"
#include "yeet_control/units.hpp"

namespace yeet::application::sfc {

/**
 * @brief Implements the outer speed control loop for an LCI drive.
 *
 * This class encapsulates the functionality of a PI speed controller with optional
 * feed-forward. It operates on per-unit values and generates a per-unit torque
 * reference, which serves as the setpoint for the inner current control loop.
 *
 * It includes processing for the speed reference, including clamping and rate
 * limiting, to ensure smooth and safe operation.
 */
class sfc_speed_controller {
public:
    // Default braking curve configuration (no braking allowed for safety)
    // These values are used during construction to initialize the braking torque limiter
    // with a safe default that prevents any regenerative braking until explicitly configured
    static constexpr std::array<real_t, 5> kDefaultBrakingSpeedAxisPu = {0.0, 0.25, 0.5, 0.75, 1.0};
    static constexpr std::array<real_t, 5> kDefaultNoBrakingTorqueLimitsPu = {0.0, 0.0, 0.0, 0.0, 0.0};
    static constexpr std::array<real_t, 5> kDefaultSoftStarterBrakingTorqueLimitsPu = {0.0, -0.1, -0.3, -0.6, -1.0};

    /**
     * @brief Constructs the speed controller.
     * @param sampling_time The sampling time of the control loop.
     */
    explicit sfc_speed_controller(duration_t sampling_time);

    /**
     * @brief Sets the gains for the internal PI controller.
     * @param kp The proportional gain (dimensionless).
     * @param ti The integral time constant.
     */
    void configure_pi_gains(real_t kp, duration_t ti);

    /**
     * @brief Sets the operational limits for the output torque reference.
     * @param min_pu The minimum allowed torque reference [pu].
     * @param max_pu The maximum allowed torque reference [pu].
     */
    void configure_torque_limits_pu(torque_pu_t min_pu, torque_pu_t max_pu);

    /**
     * @brief Configures the limits for the speed reference input.
     * @param min_pu The minimum allowed speed reference [pu].
     * @param max_pu The maximum allowed speed reference [pu].
     */
    void configure_speed_reference_limits_pu(speed_pu_t min_pu, speed_pu_t max_pu);

    /**
     * @brief Configures the rate limiter for the speed reference.
     * @param rate_limit_pu_per_second The maximum rate of change [pu/s].
     */
    void configure_speed_reference_rate_limit(real_t rate_limit_pu_per_second);

    /**
     * @brief Configures the feed-forward path based on system inertia.
     * @param inertia_pu The total system inertia [pu]. A value of zero disables feed-forward.
     */
    void configure_feedforward_pu(inertia_constant_pu_t inertia_pu);

    /**
     * @brief Configures the speed-dependent braking torque limit curve.
     *
     * The braking limit is not a constant value; it must decrease as speed decreases
     * to prevent commutation failures in the LCI. This function configures the
     * lookup table that defines the maximum safe braking torque at different speeds.
     *
     * @param speed_axis_pu Array of speed breakpoints [pu]. Must be sorted in ascending order.
     * @param torque_limit_axis_pu Array of corresponding maximum negative torque limits [pu].
     *                             These values should be negative or zero.
     */
    void configure_braking_torque_limit_curve(
        const std::array<real_t, 5>& speed_axis_pu, const std::array<real_t, 5>& torque_limit_axis_pu);

    /**
     * @brief Executes one update cycle of the speed controller.
     * @param speed_ref_pu The per-unit speed reference.
     * @param speed_measured_pu The measured per-unit speed.
     */
    void update(speed_pu_t speed_ref_pu, speed_pu_t speed_measured_pu);

    /**
     * @brief Gets the most recently calculated torque reference.
     * @return The torque reference [pu].
     */
    torque_pu_t get_torque_ref_pu() const;

    /**
     * @brief Gets the processed (clamped and rate-limited) speed reference.
     * @return The processed speed reference [pu].
     */
    speed_pu_t get_processed_speed_ref_pu() const;

    /**
     * @brief Gets the PI controller output before limiting.
     * @return The PI controller output value.
     */
    real_t get_pi_output() const;

    /**
     * @brief Checks if the speed controller is in saturation.
     * @return True if the controller is saturated, false otherwise.
     */
    bool is_saturated() const;

    /**
     * @brief Resets the internal state of the controller.
     * @param initial_speed_pu The initial speed to reset to [pu].
     */
    void reset(speed_pu_t initial_speed_pu = speed_pu_t{0.0});

    /**
     * @brief Enables or disables the integrator state.
     * @param status True to enable, false to disable.
     */
    void freeze_integrator(bool status);

private:
    const duration_t _sampling_time;

    // Control components
    yeet::control::pi_controller<real_t> _pi_controller;
    yeet::control::rate_of_change_limiter _speed_ref_rate_limiter;
    yeet::control::derivative _speed_ref_derivative;
    yeet::control::lookup_table_1d<real_t, 5> _braking_torque_limiter;

    // Configuration
    speed_pu_t _min_speed_ref_pu{0.0};
    speed_pu_t _max_speed_ref_pu{1.0};
    inertia_constant_pu_t _inertia_pu{0.0};  // Feed-forward gain

    // Torque limits (stored for use with dynamic braking limit)
    real_t _torque_min_limit{-1.0};
    real_t _torque_max_limit{1.0};

    // Outputs
    torque_pu_t _torque_ref_pu{};
    speed_pu_t _processed_speed_ref_pu{};
    real_t _pi_output{};
    torque_pu_t _torque_ref_unclamped{};
};

}  // namespace yeet::application::sfc
