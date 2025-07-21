#pragma once

#include "reference_frame.hpp"
#include "sogi_filter_sequence_extractor.hpp"
#include "three_phase_pll.hpp"
#include "types.hpp"

namespace yeet::control {

/**
 * @brief Implements a robust, hardened three-phase PLL using a Dual-SOGI
 * sequence extractor as a pre-filter for the standard three-phase PLL.
 *
 * This class provides excellent performance under unbalanced and distorted
 * grid conditions by incorporating:
 * - A Dual-SOGI pre-filter for positive sequence extraction.
 * - A standard three-phase PLL for frequency and phase estimation.
 * - Robust operation under grid disturbances.
 */
class dual_sogi_srf_pll {
public:
    /**
     * @brief Constructor
     *
     * @param sampling_time Sampling time in seconds
     */
    explicit dual_sogi_srf_pll(real_t sampling_time);

    /**
     * @brief Configure the SOGI sequence extractor
     *
     * @param k SOGI gain (dimensionless, typically sqrt(2) ≈ 1.414)
     * @param time_constant_dc_offset_rejection DC rejection time-constant in seconds
     * @param scale_output Whether to scale output by 0.5 to compensate for gain of 2
     */
    void configure_sequence_extractor(real_t k, real_t time_constant_dc_offset_rejection, bool scale_output = true);

    /**
     * @brief Configure the PI controller for the internal PLL
     *
     * @param kp Proportional gain
     * @param ti Integral time constant
     */
    void configure_pi_controller(real_t kp, real_t ti);

    /**
     * @brief Configure the frequency limits for the internal PLL
     *
     * @param min_freq Minimum frequency in Hz
     * @param max_freq Maximum frequency in Hz
     */
    void configure_frequency_limits(real_t min_freq, real_t max_freq);

    /**
     * @brief Configure the nominal frequency for the internal PLL
     *
     * This method configures the nominal frequency for the internal PLL
     * and initializes the system to avoid startup issues.
     *
     * @param nominal_frequency Nominal frequency in Hz
     */
    void configure_nominal_frequency(real_t nominal_frequency);

    /**
     * @brief Reset all internal states
     */
    void reset();

    /**
     * @brief Update the PLL with new input measurements
     *
     * @param v_in The measured three-phase input voltage
     */
    void update(const alphabeta<real_t>& v_in);

    /**
     * @brief Get the estimated frequency in Hz
     */
    real_t get_estimated_frequency() const;

    /**
     * @brief Get the estimated angular frequency in rad/s
     */
    real_t get_estimated_angular_frequency() const;

    /**
     * @brief Get the estimated angle
     */
    angle<zero_to_two_pi> get_estimated_angle() const;

    /**
     * @brief Check if the PLL is locked
     */
    bool is_locked() const;

    /**
     * @brief Get the Q-axis error for diagnostics
     */
    real_t get_q_error() const;

    /**
     * @brief Get the positive sequence output from the extractor
     */
    const alphabeta<real_t>& get_positive_sequence() const;

    /**
     * @brief Get the alpha SOGI output for diagnostics
     */
    alphabeta<real_t> get_alpha_sogi_output() const;

    /**
     * @brief Get the beta SOGI output for diagnostics
     */
    alphabeta<real_t> get_beta_sogi_output() const;

    dq<real_t> get_voltage_dq() const { return _internal_pll.get_voltage_dq(); }

    /**
     * @brief Get the internal three-phase PLL for advanced diagnostics
     */
    // const three_phase_pll& get_internal_pll() const;

private:
    real_t _sampling_time;  ///< Sampling time in seconds

    // Internal components
    sogi_filter_sequence_extractor _sequence_extractor;  ///< Dual SOGI sequence extractor
    three_phase_pll _internal_pll;                       ///< Internal three-phase PLL

    // State variables
    alphabeta<real_t> _filtered_voltage_alphabeta{};  ///< Filtered positive sequence voltage
    abc<real_t> _filtered_voltage_abc{};              ///< Filtered positive sequence voltage in ABC frame
};

}  // namespace yeet::control
