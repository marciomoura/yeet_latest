#ifndef YEET_CONTROL_THREE_PHASE_PLL_HPP
#define YEET_CONTROL_THREE_PHASE_PLL_HPP

#include "adaptive_band_reject_filter.hpp"
#include "angle.hpp"
#include "integrator.hpp"
#include "on_delay.hpp"  // <-- New include
#include "pi_controller.hpp"
#include "reference_frame.hpp"
#include "second_order_filter.hpp"
#include "types.hpp"
#include "yeet_control/angle.hpp"

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
    /**
     * @brief Constructor
     *
     * @param sampling_time Sampling time in seconds
     */
    explicit three_phase_pll(real_t sampling_time);

    /**
     * @brief Configure the PI controller for frequency estimation
     *
     * @param kp Proportional gain
     * @param ti Integral time constant
     */
    void configure_pi_controller(real_t kp, real_t ti);

    /**
     * @brief Configure the frequency limits for the PLL
     *
     * @param min_freq Minimum frequency in Hz
     * @param max_freq Maximum frequency in Hz
     */
    void configure_frequency_limits(real_t min_freq, real_t max_freq);

    /**
     * @brief Configure the nominal frequency for feed-forward
     *
     * @param nominal_frequency Nominal frequency in Hz
     */
    void configure_nominal_frequency(real_t nominal_frequency);

    /**
     * @brief Configure the lock detection logic
     *
     * @param lock_duration_s Time in seconds the q-error must be below threshold for lock
     * @param q_error_threshold The threshold for the q-axis error
     */
    void configure_lock_detection(real_t lock_duration_s, real_t q_error_threshold);

    /**
     * @brief Reset the PLL to initial conditions
     */
    void reset();

    /**
     * @brief Update the PLL with new input measurements
     *
     * @param voltage_abc Input three-phase voltages
     */
    void update(const alphabeta<real_t>& voltage_ab);

    /**
     * @brief Get the current estimated frequency in Hz
     */
    real_t get_estimated_frequency() const;

    /**
     * @brief Get the current estimated phase angle
     */
    angle<zero_to_two_pi> get_estimated_angle() const;

    /**
     * @brief Get the transformed dq voltages
     */
    const dq<real_t>& get_voltage_dq() const;

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

    alphabeta<real_t> get_estimated_alphabeta() const {
        angle<zero_to_two_pi> const estimated_angle = get_estimated_angle();

        return _voltage_dq.to_alphabeta(estimated_angle);
    }

    auto get_uncompensated_angle() const { return _frequency_integrator.get_output(); }

private:
    real_t _sampling_time;  ///< Sampling time in seconds

    // Configuration
    real_t _nominal_frequency{50.0};               ///< Nominal frequency in Hz
    real_t _lock_q_error_threshold{real_t{0.05}};  ///< Q-axis error threshold for lock

    // Control components
    pi_controller<real_t> _frequency_pi_controller{};           ///< PI controller for frequency estimation
    integrator<angle<zero_to_two_pi>> _frequency_integrator{};  ///< Integrator for phase angle
    on_delay _lock_timer;                                       ///< On-delay timer for lock detection logic

    second_order_low_pass_filter<real_t> _filter_frequency_estimation;
    adaptive_band_reject_filter<real_t> _omega_reject_filter;

    // State variables
    real_t _estimated_frequency{real_t{50}};  ///< Current frequency estimate (Hz)
    dq<real_t> _voltage_dq{};                 ///< Transformed dq voltages
};

}  // namespace yeet::control

#endif  // YEET_CONTROL_THREE_PHASE_PLL_HPP