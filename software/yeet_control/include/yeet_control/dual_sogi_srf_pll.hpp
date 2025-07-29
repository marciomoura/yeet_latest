#pragma once

#include "angle.hpp"
#include "reference_frame.hpp"
#include "sogi_filter_sequence_extractor.hpp"
#include "three_phase_pll.hpp"
#include "types.hpp"
#include "units.hpp"

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
    // Default configuration constants
    static constexpr real_t default_sogi_k = 1.414;
    static constexpr duration_t default_sogi_dc_rejection_time_constant = duration_t{0.1};

    /**
     * @brief Constructor
     *
     * @param sampling_time Sampling time of the control loop.
     */
    explicit dual_sogi_srf_pll(duration_t sampling_time);

    /**
     * @brief Configure the SOGI sequence extractor.
     *
     * @param k SOGI gain (dimensionless, typically sqrt(2) ≈ 1.414).
     * @param time_constant_dc_offset_rejection DC rejection time-constant.
     * @param scale_output Whether to scale output by 0.5 to compensate for gain of 2.
     */
    void configure_sequence_extractor(real_t k, duration_t time_constant_dc_offset_rejection, bool scale_output = true);

    /**
     * @brief Configure the PI controller for the internal PLL.
     *
     * @param kp Proportional gain.
     * @param ti Integral time constant.
     */
    void configure_pi_controller(real_t kp, duration_t ti);

    /**
     * @brief Configure the frequency limits for the internal PLL.
     *
     * @param min_freq Minimum frequency.
     * @param max_freq Maximum frequency.
     */
    void configure_frequency_limits(frequency_t min_freq, frequency_t max_freq);

    /**
     * @brief Configure the nominal frequency for the internal PLL.
     *
     * @param nominal_frequency Nominal frequency.
     */
    void configure_nominal_frequency(frequency_t nominal_frequency);

    /**
     * @brief Reset all internal states.
     */
    void reset();

    /**
     * @brief Update the PLL with new input measurements.
     *
     * @param v_in The measured three-phase input voltage in the stationary αβ frame.
     */
    void update(const alphabeta<voltage_pu_t>& v_in);

    /**
     * @brief Get the estimated frequency in Hz.
     */
    frequency_t get_estimated_frequency() const;

    /**
     * @brief Get the estimated frequency in per-unit.
     */
    frequency_pu_t get_estimated_frequency_pu() const;

    /**
     * @brief Get the estimated angle.
     */
    angle<zero_to_two_pi> get_estimated_angle() const;

    /**
     * @brief Check if the PLL is locked.
     */
    bool is_locked() const;

    /**
     * @brief Get the estimation error (d-axis voltage).
     */
    voltage_pu_t get_estimation_error() const;

    /**
     * @brief Get the estimated voltage magnitude in per-unit.
     */
    voltage_pu_t get_estimated_voltage_magnitude() const;

    /**
     * @brief Get the positive sequence output from the extractor.
     */
    const alphabeta<voltage_pu_t>& get_positive_sequence() const;

private:
    duration_t _sampling_time;  ///< Sampling time

    // Internal components
    sogi_filter_sequence_extractor _sequence_extractor;  ///< Dual SOGI sequence extractor
    three_phase_pll _internal_pll;                       ///< Internal three-phase PLL

    // State variables
    alphabeta<voltage_pu_t> _filtered_voltage_alphabeta{};  ///< Filtered positive sequence voltage
};

}  // namespace yeet::control
