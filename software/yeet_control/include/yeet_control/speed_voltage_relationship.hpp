#pragma once

#include "first_order_low_pass_filter.hpp"
#include "types.hpp"

namespace yeet::control {

/**
 * @brief Models the linear relationship between mechanical speed and voltage magnitude.
 *
 * This class implements a simplified speed-voltage characteristic for electric machines
 * where the voltage magnitude is linearly related to mechanical speed. The model operates
 * entirely in per-unit and can be configured for either instantaneous (algebraic) or
 * dynamic (first-order) response.
 *
 * The governing equation is: |V_pu| = k_vs * ω_pu + V_offset_pu
 * With optional dynamics: τ * d|V_pu|/dt + |V_pu| = k_vs * ω_pu + V_offset_pu
 */
class speed_voltage_relationship {
public:
    /**
     * @brief Constructs the speed-voltage relationship model.
     *
     * @param sampling_time The sampling time for discrete-time integration [s].
     */
    explicit speed_voltage_relationship(real_t sampling_time);

    /**
     * @brief Configures the voltage-speed coefficient.
     *
     * This coefficient represents the slope of the voltage magnitude vs speed characteristic.
     * Typical values range from 0.8 to 1.2 pu/pu for most electric machines.
     *
     * @param coefficient The voltage-speed coefficient [pu/pu]. Must be positive.
     */
    void configure_voltage_speed_coefficient(real_t coefficient);

    /**
     * @brief Configures the voltage offset.
     *
     * This offset accounts for constant voltage components that are independent of speed,
     * such as losses or minimum excitation requirements.
     *
     * @param offset_pu The per-unit voltage offset [pu].
     */
    void configure_voltage_offset_pu(real_t offset_pu);

    /**
     * @brief Configures the time constant for dynamic response.
     *
     * When set to zero, the model provides instantaneous response (algebraic).
     * When positive, introduces first-order dynamics to model electrical time constants.
     *
     * @param time_constant_seconds The time constant [s]. Must be non-negative.
     */
    void configure_time_constant(real_t time_constant_seconds);

    /**
     * @brief Configures voltage magnitude limits.
     *
     * @param min_voltage_pu Minimum allowed voltage magnitude [pu]. Must be non-negative.
     * @param max_voltage_pu Maximum allowed voltage magnitude [pu]. Must be greater than min.
     */
    void configure_voltage_limits_pu(real_t min_voltage_pu, real_t max_voltage_pu);

    /**
     * @brief Updates the model with the current mechanical speed.
     *
     * This method should be called once per control cycle. The resulting voltage
     * magnitude can be retrieved using get_voltage_magnitude_pu().
     *
     * @param speed_pu The current per-unit mechanical speed [pu].
     */
    void update(real_t speed_pu) noexcept;

    /**
     * @brief Gets the current per-unit voltage magnitude.
     *
     * @return The computed per-unit voltage magnitude [pu].
     */
    real_t get_voltage_magnitude_pu() const noexcept;

    /**
     * @brief Gets the steady-state voltage magnitude for current speed.
     *
     * This returns the algebraic result without considering dynamics,
     * useful for feed-forward calculations.
     *
     * @return The steady-state per-unit voltage magnitude [pu].
     */
    real_t get_steady_state_voltage_pu() const noexcept;

    /**
     * @brief Resets the internal state to initial conditions.
     *
     * @param initial_voltage_pu The initial voltage magnitude [pu]. Defaults to 0.0.
     */
    void reset(real_t initial_voltage_pu = 0.0f);

    /**
     * @brief Checks if the model is operating in dynamic mode.
     *
     * @return True if time constant > 0 (dynamic), false if algebraic.
     */
    bool is_dynamic_mode() const noexcept;

private:
    // Configuration parameters
    real_t _voltage_speed_coefficient{1.0f};  ///< Voltage-speed coefficient [pu/pu]
    real_t _voltage_offset_pu{0.0f};          ///< Voltage offset [pu]
    real_t _time_constant_seconds{0.0f};      ///< Time constant [s]
    real_t _min_voltage_pu{0.0f};             ///< Minimum voltage limit [pu]
    real_t _max_voltage_pu{2.0f};             ///< Maximum voltage limit [pu]

    // Internal state
    first_order_low_pass_filter<real_t> _voltage_filter;  ///< Low-pass filter for dynamic response
    real_t _current_speed_pu{0.0f};                       ///< Current speed input [pu]
    real_t _current_voltage_pu{0.0f};                     ///< Current voltage output [pu]
    bool _is_dynamic_mode{false};                         ///< Flag for dynamic vs algebraic mode

    /**
     * @brief Calculates the steady-state voltage for given speed.
     *
     * @param speed_pu Input speed [pu].
     * @return Steady-state voltage magnitude [pu].
     */
    real_t calculate_steady_state_voltage(real_t speed_pu) const noexcept;

    /**
     * @brief Applies voltage limits to ensure physical constraints.
     *
     * @param voltage_pu Input voltage magnitude [pu].
     * @return Limited voltage magnitude [pu].
     */
    real_t apply_voltage_limits(real_t voltage_pu) const noexcept;
};

}  // namespace yeet::control