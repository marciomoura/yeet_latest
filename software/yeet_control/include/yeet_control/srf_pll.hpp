#pragma once

#include "angle_wrapped.hpp"
#include "integrator.hpp"
#include "pi_controller.hpp"
#include "reference_frame.hpp"
#include "types.hpp"
#include "units.hpp"

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
     * @param input_alphabeta The input signal in the alpha-beta frame.
     */
    void update(const alphabeta<real_t>& input_alphabeta);

    /**
     * @brief Gets the estimated phase angle.
     * @return The estimated angle.
     */
    angle_wrapped get_estimated_angle() const;

    /**
     * @brief Gets the estimated angular frequency.
     * @return The estimated angular frequency in rad/s.
     */
    real_t get_estimated_angular_frequency() const;

    /**
     * @brief Gets the transformed d-q vector.
     * @return The d-q vector from the last update.
     */
    const dq<real_t>& get_dq_vector() const;

private:
    // Control components
    pi_controller<real_t> _frequency_pi_controller;
    integrator<angle_wrapped> _phase_integrator;

    // State variables
    frequency_pu_t _initial_frequency{0.0f};
    real_t _nominal_angular_frequency{0.0f};
    real_t _estimated_angular_frequency{0.0f};
    dq<real_t> _dq_vector{};
};

}  // namespace yeet::control