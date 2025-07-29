#pragma once

#include "yeet_control/reference_frame.hpp"
#include "yeet_control/sogi_filter.hpp"
#include "yeet_control/types.hpp"
#include "yeet_control/units.hpp"

namespace yeet::control {

/**
 * @brief Implements a Dual SOGI-based Positive Sequence Component Extractor.
 *
 * This class uses two SOGI filter instances to decompose an unbalanced alpha-beta
 * voltage vector into its fundamental positive sequence component. It is highly
 * effective for filtering and providing a clean, balanced signal to a PLL,
 * especially under unbalanced grid conditions.
 *
 * The algorithm works by:
 * 1. Processing the alpha component through the first SOGI filter
 * 2. Processing the beta component through the second SOGI filter
 * 3. Cross-coupling the outputs to isolate the positive sequence component
 * 4. Optionally scaling the output to compensate for the inherent gain of 2
 */
class sogi_filter_sequence_extractor {
public:
    /**
     * @brief Constructor
     *
     * @param sampling_time Sampling time in seconds
     */
    explicit sogi_filter_sequence_extractor(duration_t sampling_time);

    /**
     * @brief Configure both internal SOGI filters with the same parameters
     *
     * @param k SOGI gain (dimensionless, typically sqrt(2) ≈ 1.414)
     * @param time_constant_dc_rejection_seconds DC rejection time-constant in seconds
     * @param scale_output Whether to scale output by 0.5 to compensate for gain of 2
     */
    void configure(real_t k, duration_t time_constant_dc_rejection_seconds, bool scale_output = true);

    /**
     * @brief Reset the state of both internal SOGI filters
     */
    void reset();

    /**
     * @brief Execute one time-step of the sequence extraction algorithm
     *
     * @param v_in The unbalanced input voltage vector in the alpha-beta frame
     * @param omega The externally estimated grid angular frequency (rad/s)
     */
    void update(const alphabeta<voltage_pu_t>& v_in, angular_frequency_t omega);

    /**
     * @brief Get the filtered positive sequence vector
     */
    const alphabeta<voltage_pu_t>& get_positive_sequence() const;

    /**
     * @brief Get the alpha component of the positive sequence
     */
    voltage_pu_t get_positive_sequence_alpha() const;

    /**
     * @brief Get the beta component of the positive sequence
     */
    voltage_pu_t get_positive_sequence_beta() const;

    /**
     * @brief Get the output of the alpha SOGI filter (for diagnostics)
     */
    alphabeta<voltage_pu_t> get_alpha_sogi_output() const;

    /**
     * @brief Get the output of the beta SOGI filter (for diagnostics)
     */
    alphabeta<voltage_pu_t> get_beta_sogi_output() const;

    /**
     * @brief Get the DC offset estimated for the alphabeta components
     */
    alphabeta<voltage_pu_t> get_dc_offset() const;

private:
    duration_t _sampling_time;  ///< Sampling time in seconds

    // Configuration parameters
    bool _scale_output{true};  ///< Whether to scale output by 0.5

    // Internal SOGI filters
    sogi_filter _sogi_alpha;  ///< SOGI filter for alpha component
    sogi_filter _sogi_beta;   ///< SOGI filter for beta component

    // Output state
    alphabeta<voltage_pu_t> _positive_sequence{};  ///< Filtered positive sequence vector
};

}  // namespace yeet::control
