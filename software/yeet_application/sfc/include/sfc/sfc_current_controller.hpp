#pragma once

#include "yeet_control/pi_controller.hpp"
#include "yeet_control/rate_of_change_limiter.hpp"
#include "yeet_control/units.hpp"

namespace yeet::application::sfc {

/**
 * @brief Implements the inner current control loop for an LCI drive.
 *
 * This class encapsulates the full functionality of a high-performance current
 * controller. It is suitable for both high-performance LCI drives and
 * soft-starter applications. The controller operates on per-unit values.
 */
class sfc_current_controller {
public:
    /**
     * @brief Constructs the current controller.
     * @param sampling_time The sampling time of the control loop [s].
     */
    explicit sfc_current_controller(duration_t sampling_time);

    /**
     * @brief Sets the gains for the internal PI controller.
     * @param proportional_gain The proportional gain (dimensionless).
     * @param integral_time The integral time constant [s].
     */
    void configure_pi_gains(double proportional_gain, duration_t integral_time);

    /**
     * @brief Sets the operational limits for the DC link current reference.
     * @param min_current_pu The minimum allowed current reference [pu].
     * @param max_current_pu The maximum allowed current reference [pu].
     */
    void configure_current_limits_pu(current_pu_t min_current_pu, current_pu_t max_current_pu);

    /**
     * @brief Sets the slew rate for the DC link current reference.
     * @param slew_rate_pu_per_second The maximum rate of change for the current reference [pu/s].
     */
    void configure_current_reference_slew_rate(double slew_rate_pu_per_second);

    /**
     * @brief Enables or disables the slew rate limiter for the current reference.
     * @param enable True to enable the slew rate limiter, false to disable it.
     */
    void enable_current_reference_slew_rate(bool enable);

    /**
     * @brief Sets the operational limits for the grid-side converter firing angle.
     * @param min_angle The minimum allowed firing angle [rad].
     * @param max_angle The maximum allowed firing angle [rad].
     */
    void configure_firing_angle_limits_radians(angle_t min_angle, angle_t max_angle);

    /**
     * @brief Resets all internal states of the controller.
     */
    void reset();

    /**
     * @brief Executes one update cycle of the current controller.
     *
     * @param dc_link_current_reference_pu The per-unit current reference from the outer loop.
     * @param dc_link_current_measured_pu The measured per-unit DC link current.
     * @param ideal_dc_voltage_pu The ideal no-load DC voltage from the rectifier [pu, grid-side base].
     * @param inverter_back_emf_pu Estimated per-unit inverter back-EMF [pu, grid-side reflected base].
     */
    void update(current_pu_t dc_link_current_reference_pu,
        current_pu_t dc_link_current_measured_pu,
        voltage_pu_t ideal_dc_voltage_pu,
        voltage_pu_t inverter_back_emf_grid_reflected_pu);

    /**
     * @brief Executes one update cycle of the current controller with automatic per-unit reflection.
     *
     * This overload automatically reflects the machine-side inverter back-EMF to grid-side per-unit
     * before performing the control calculation. This is the recommended interface when machine-side
     * per-unit quantities are available.
     *
     * @param dc_link_current_reference_pu The per-unit current reference from the outer loop.
     * @param dc_link_current_measured_pu The measured per-unit DC link current.
     * @param ideal_dc_voltage_pu The ideal no-load DC voltage from the rectifier [pu, grid-side base].
     * @param inverter_back_emf_machine_pu Estimated per-unit inverter back-EMF [pu, machine-side base].
     * @param machine_side_base_voltage The machine-side voltage base for per-unit reflection [V].
     * @param grid_side_base_voltage The grid-side voltage base for per-unit reflection [V].
     */
    void update(current_pu_t dc_link_current_reference_pu,
        current_pu_t dc_link_current_measured_pu,
        voltage_pu_t ideal_dc_voltage_pu,
        voltage_pu_t inverter_back_emf_machine_pu,
        const voltage_t& machine_side_base_voltage,
        const precomputed_divisor<voltage_t>& grid_side_base_voltage);

    /**
     * @brief Gets the most recently calculated firing angle reference.
     * @return The firing angle reference [rad].
     */
    angle_t get_firing_angle_reference_radians() const;

    /**
     * @brief Gets the limited current reference used in the last update cycle.
     * @return The limited DC link current reference [pu].
     */
    current_pu_t get_current_reference_limited_pu() const;

    /**
     * @brief Gets the clamped cosine of the firing angle.
     * @return The clamped cosine value.
     */
    double get_cos_alpha_clamped() const;

    /**
     * @brief Gets the current error in per-unit.
     * @return The current error [pu].
     */
    current_pu_t get_current_error_pu() const;

    /**
     * @brief Gets the PI controller output before clamping.
     * @return The PI controller output value.
     */
    real_t get_pi_output() const;

    /**
     * @brief Checks if the controller is in saturation.
     * @return True if the controller is saturated, false otherwise.
     */
    bool is_saturated() const;

private:
    void recompute_internal_limits();

    yeet::control::pi_controller<double> _pi_controller;
    yeet::control::rate_of_change_limiter _rate_of_change_limiter;

    // Configuration
    current_pu_t _dc_link_current_min_pu{};
    current_pu_t _dc_link_current_max_pu{};
    angle_t _grid_firing_angle_min_radians{};
    angle_t _grid_firing_angle_max_radians{};

    // Internal state for limits
    double _cos_alpha_min{};
    double _cos_alpha_max{};

    // Outputs
    double _cos_alpha_clamped{};
    double _cos_alpha_unclamped{};
    angle_t _firing_angle_reference_radians{};
    current_pu_t _current_reference_limited_pu{};
    current_pu_t _current_error_pu{};
    real_t _pi_output{};
};

}  // namespace yeet::application::sfc
