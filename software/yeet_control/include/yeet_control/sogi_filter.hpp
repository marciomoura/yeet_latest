#pragma once

#include "yeet_control/integrator.hpp"
#include "yeet_control/reference_frame.hpp"
#include "yeet_control/types.hpp"
#include "yeet_control/units.hpp"

namespace yeet::control {

/**
 * @brief Discretization-Compensated Second-Order Generalized Integrator (SOGI).
 *
 * This class implements a quadrature signal generator that takes a single-phase
 * input voltage and an estimated grid frequency. It outputs two signals
 * in quadrature (alpha and beta components), which are robust against discretization errors.
 *
 * The implementation uses an improved update sequence for its internal integrators
 * to minimize the phase and magnitude errors that occur in naive digital
 * implementations, making it more accurate and stable, especially at lower
 * sampling frequencies. It also includes an optional DC offset rejection loop.
 */
class sogi_filter {
public:
    static constexpr real_t default_k = 1.414;  // SOGI gain (dimensionless, typically sqrt(2))
    static constexpr duration_t default_dc_rejection_time_constant =
        duration_t{1.0 / 50.0};  //  DC rejection time constant

    /**
     * @brief Constructor.
     *
     * @param sampling_time The execution interval (sampling time) in seconds.
     */
    explicit sogi_filter(duration_t sampling_time);

    /**
     * @brief Configure the SOGI gain parameter.
     *
     * @param k SOGI gain (dimensionless, a good default is sqrt(2) ≈ 1.414).
     */
    void configure(real_t k);

    /**
     * @brief Configure the DC rejection loop's time constant.
     *
     * @param time_constant The time constant in seconds for the DC rejection filter.
     *                      A smaller value leads to faster rejection.
     */
    void configure_dc_rejection(duration_t time_constant);

    /**
     * @brief Reset all internal states and integrators to zero.
     */
    void reset();

    /**
     * @brief Update the SOGI with a new input measurement and estimated frequency.
     *
     * @param v_in The input voltage measurement for the current time step.
     * @param omega The estimated grid angular frequency in rad/s.
     */
    void update(voltage_pu_t v_in, angular_frequency_t omega);

    /**
     * @brief Get the quadrature output signals (alpha-beta frame).
     *
     * @return An alphabeta object containing the in-phase (alpha) and quadrature (beta) signals.
     */
    alphabeta<voltage_pu_t> get_output() const;

    /**
     * @brief Get the alpha component (in-phase with the AC part of the input).
     */
    voltage_pu_t get_alpha() const;

    /**
     * @brief Get the beta component (quadrature, lagging the alpha component by 90 degrees).
     */
    voltage_pu_t get_beta() const;

    /**
     * @brief Get the estimated DC offset from the input signal.
     */
    voltage_pu_t get_dc_offset() const;

private:
    duration_t _sampling_time;  ///< Sampling time in seconds.

    // Configuration parameters
    real_t _k{real_t{1.414}};   ///< SOGI gain (dimensionless).
    real_t _k_dc{real_t{0.0}};  ///< DC rejection gain, derived from the time constant.

    // Internal state is stored within the integrators
    integrator<real_t> _alpha_integrator{};  ///< Integrator for the alpha (in-phase) component.
    integrator<real_t> _beta_integrator{};   ///< Integrator for the beta (quadrature) component.
    integrator<real_t> _dc_integrator{};     ///< Integrator for the DC offset component.

    alphabeta<voltage_pu_t> _output{};  ///< Output of the SOGI filter in alpha-beta frame.
};

}  // namespace yeet::control