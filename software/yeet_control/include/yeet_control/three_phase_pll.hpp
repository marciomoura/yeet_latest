#pragma once

#include "adaptive_band_reject_filter.hpp"
#include "angle_wrapped.hpp"
#include "integrator.hpp"
#include "on_delay.hpp"
#include "pi_controller.hpp"
#include "precomputed_divisor.hpp"
#include "reference_frame.hpp"
#include "second_order_filter.hpp"
#include "types.hpp"
#include "units.hpp"
#include "yeet_control/precomputed_divisor.hpp"

namespace yeet::control {

/**
 * @brief Three-phase phase locked loop (PLL) for frequency and phase estimation
 *
 * This class implements a synchronous reference frame (SRF) PLL for estimating
 * the angular frequency and phase of a three-phase sinusoidal signal. The PLL
 * uses the following structure:
 *
 * 1. Clarke transformation (abc → αβ)
 * 2. Park transformation (αβ → dq) using estimated angle
 * 3. PI controller on q-axis component (frequency estimation)
 * 4. Integration of frequency to obtain phase angle
 *
 * The PLL is designed for 1.0 per-unit reference signals and provides
 * direct frequency tracking without additional filtering.
 */
class three_phase_pll {
public:
    // Default configuration constants
    static constexpr real_t default_pi_kp = 0.2;
    static constexpr duration_t default_pi_ti = duration_t{0.05};

    // Default frequency limits
    static constexpr frequency_t default_nominal_frequency = frequency_t{50.0};
    static constexpr frequency_t default_min_frequency = frequency_t{30.0};
    static constexpr frequency_t default_max_frequency = frequency_t{70.0};

    // Lock detection configuration
    static constexpr voltage_pu_t default_lock_q_error_threshold = voltage_pu_t{0.05};
    static constexpr duration_t default_lock_duration = duration_t{ms(100.0)};

    // Default filter configuration
    static constexpr frequency_t default_estimated_frequency_filter_cutoff = frequency_t{10.0};
    static constexpr real_t default_estimated_frequency_filter_damping = 0.707;

    // Notch filter configuration
    static constexpr frequency_t default_notch_filter_bandwidth = frequency_t{10.0};
    static constexpr frequency_t default_notch_filter_freq_threshold = frequency_t{2.0};
    static constexpr duration_t default_notch_filter_settling_time = duration_t{0.5};

    /**
     * @brief Constructor
     *
     * @param sampling_time Sampling time in seconds
     */
    explicit three_phase_pll(duration_t sampling_time);

    /**
     * @brief Configure the PI controller for frequency estimation
     *
     * @param kp Proportional gain
     * @param ti Integral time constant
     */
    void configure_pi_controller(real_t kp, duration_t ti);

    /**
     * @brief Configure the frequency limits for the PLL
     *
     * @param min_freq Minimum frequency in Hz
     * @param max_freq Maximum frequency in Hz
     */
    void configure_frequency_limits(frequency_t min_freq, frequency_t max_freq);

    /**
     * @brief Configure the nominal frequency for feed-forward
     *
     * @param nominal_frequency Nominal frequency in Hz
     */
    void configure_nominal_frequency(frequency_t nominal_frequency);

    /**
     * @brief Configure the lock detection logic
     *
     * @param lock_duration_s Time in seconds the q-error must be below threshold for lock
     * @param q_error_threshold The threshold for the q-axis error
     */
    void configure_lock_detection(duration_t lock_duration_s, voltage_pu_t q_error_threshold);

    /**
     * @brief Reset the PLL to initial conditions
     */
    void reset();

    /**
     * @brief Update the PLL with new input measurements
     *
     * @param voltage_ab Input alpha-beta voltages in per-unit
     */
    void update(const alphabeta<voltage_pu_t>& voltage_ab);

    /**
     * @brief Get the current estimated frequency in Hz
     */
    frequency_t get_estimated_frequency() const;

    /**
     * @brief Get the current estimated frequency in per-unit
     */
    frequency_pu_t get_estimated_frequency_pu() const;

    /**
     * @brief Get the current estimated phase angle
     */
    angle_wrapped get_estimated_angle() const;

    /**
     * @brief Get the transformed dq voltages
     */
    const dq<voltage_pu_t>& get_voltage_dq() const;

    /**
     * @brief Check if the PLL is locked
     *
     * The PLL is considered locked if the q-axis error is below the configured threshold
     * for the specified duration.
     */
    real_t get_estimation_error() const { return get_voltage_dq().d(); }

    /**
     * @brief Get the estimated voltage magnitude in per-unit
     *
     * This is the magnitude of the dq voltage vector.
     */
    voltage_pu_t get_estimated_voltage_mangitude() const { return get_voltage_dq().magnitude(); }

    /**
     * @brief Check if the PLL is locked (q-axis error below threshold for a configured duration)
     *
     * @return True if the lock condition is met
     */
    bool is_locked() const;

    /**
     * @brief Get the PI controller for external monitoring/tuning
     */
    const pi_controller<real_t>& get_frequency_controller() const;

    alphabeta<voltage_pu_t> get_estimated_alphabeta() const;

    auto get_uncompensated_angle() const { return _frequency_integrator.get_output(); }

private:
    duration_t _sampling_time;  ///< Sampling time in seconds

    // Configuration
    precomputed_divisor<frequency_t> _nominal_frequency{frequency_t{50.0f}};  ///< Nominal frequency in Hz
    voltage_pu_t _lock_q_error_threshold{real_t{0.05}};                       ///< Q-axis error threshold for lock

    // Control components
    pi_controller<real_t> _frequency_pi_controller{};   ///< PI controller for frequency estimation
    integrator<angle_wrapped> _frequency_integrator{};  ///< Integrator for phase angle
    on_delay _lock_timer;                               ///< On-delay timer for lock detection logic

    second_order_low_pass_filter<real_t> _filter_frequency_estimation;  ///< Filter for frequency estimation
    adaptive_band_reject_filter<real_t> _omega_reject_filter;  ///< Adaptive notch filter for frequency rejection

    frequency_t _estimated_frequency{real_t{50.0}};  ///< Current frequency estimate (Hz)
    dq<voltage_pu_t> _voltage_dq{};                  ///< Transformed dq voltages
};

}  // namespace yeet::control
