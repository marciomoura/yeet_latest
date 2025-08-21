#pragma once

#include "angle_wrapped.hpp"
#include "integrator.hpp"
#include "pi_controller.hpp"
#include "precomputed_divisor.hpp"
#include "reference_frame.hpp"
#include "types.hpp"
#include "units.hpp"
#include "yeet_control/precomputed_divisor.hpp"

namespace yeet::control {

/**
 * @brief Synchronous-Reference Frame Phase-Locked Loop (SRF-PLL)
 *
 * Implements the core functionality of an SRF-PLL for estimating the
 * frequency and phase of a sinusoidal signal in an alpha-beta reference frame.
 *
 * The PLL operates by:
 * 1. Transforming the input alpha-beta vector into a rotating d-q frame using the estimated angle.
 * 2. Using the q-axis component of the transformed vector as an error signal.
 * 3. Feeding this error into a PI controller to estimate the angular frequency.
 * 4. Integrating the angular frequency to produce the estimated phase angle for the next cycle.
 */
class srf_pll {
public:
    /**
     * @brief Constructor.
     * @param sampling_time The sampling time of the system.
     */
    explicit srf_pll(duration_t sampling_time);

    /**
     * @brief Configures the nominal frequency of the PLL.
     * The PI controller will then work to regulate the deviation from this frequency.
     * @param nominal_frequency The center frequency of the PLL.
     */
    void configure_nominal_frequency(frequency_t nominal_frequency);

    /**
     * @brief Configures the PI controller gains.
     * @param kp Proportional gain.
     * @param ti Integral time constant.
     */
    void configure_pi_controller(real_t kp, duration_t ti);

    /**
     * @brief Resets the PLL to its initial state, using the nominal frequency as the starting point.
     */
    void reset();

    /**
     * @brief Resets the PLL to a specific initial frequency.
     * @param initial_frequency The frequency to start with.
     */
    void reset(frequency_pu_t initial_frequency);

    /**
     * @brief Updates the PLL with a new input sample.
     * @param input_alphabeta The input per-unit voltages in the alpha-beta frame.
     */
    void update(const alphabeta<voltage_pu_t>& input_alphabeta);

    /**
     * @brief Gets the estimated phase angle.
     * @return The estimated angle.
     */
    angle_wrapped get_estimated_angle() const;

    /**
     * @brief Gets the estimated frequency in Hz.
     * @return The estimated frequency.
     */
    frequency_t get_estimated_frequency() const;

    /**
     * @brief Gets the estimated frequency in per-unit.
     * @return The estimated frequency in per-unit relative to nominal.
     */
    frequency_pu_t get_estimated_frequency_pu() const;

    /**
     * @brief Gets the estimated angular frequency in rad/s.
     * @return The estimated angular frequency.
     */
    angular_frequency_t get_estimated_angular_frequency() const;

    /**
     * @brief Gets the transformed d-q vector.
     * @return The d-q per-unit voltages from the last update.
     */
    const dq<voltage_pu_t>& get_dq_vector() const;

private:
    // Control components
    pi_controller<real_t> _frequency_pi_controller;
    integrator<angle_wrapped> _phase_integrator;

    // Configuration
    frequency_pu_t _initial_frequency{frequency_pu_t{1.0}};                  // Default to 1 per-unit
    precomputed_divisor<frequency_t> _nominal_frequency{frequency_t{50.0}};  // Default to 50 Hz

    // State variables
    frequency_t _estimated_frequency{frequency_t{50.0}};
    dq<voltage_pu_t> _dq_vector{};
};

}  // namespace yeet::control