#pragma once

#include "sfc/sfc_commutation_mask.hpp"
#include "sfc/sfc_commutation_monitor.hpp"
#include "yeet_control/angle.hpp"
#include "yeet_control/dual_sogi_srf_pll.hpp"
#include "yeet_control/reference_frame.hpp"
#include "yeet_control/types.hpp"
#include "yeet_control/units.hpp"

namespace yeet::application::sfc {

/**
 * @brief Observes the machine-side electrical state for an LCI.
 *
 * This class integrates a Phase-Locked Loop (PLL) to estimate the grid angle
 * and frequency, and a commutation monitor to detect the commutation process.
 * It provides the necessary grid information for the firing angle
 * controller and protection systems.
 */
class sfc_machine_side_observer {
public:
    static constexpr auto default_nominal_frequency = frequency_t{50.0};
    static constexpr real_t default_pll_kp = 0.2;
    static constexpr duration_t default_pll_ti = duration_t{ms(50.0)};
    // static constexpr auto default_commutation_threshold = voltage_pu_t{0.02};

    /**
     * @brief Constructs the grid-side observer.
     * @param sampling_period The sampling period of the control loop.
     * @param nominal_frequency The nominal grid frequency.
     */
    explicit sfc_machine_side_observer(duration_t sampling_period);

    /**
     * @brief Configures the gains for the internal Phase-Locked Loop.
     * @param gains The proportional and time constant for the PLL.
     */
    void configure_pll(real_t kp, duration_t ti, frequency_t nominal_frequency);

    /**
     * @brief Configures the voltage threshold for commutation failure detection.
     * @param threshold_pu The per-unit voltage threshold.
     */
    void configure_commutation_failure_threshold(voltage_pu_t threshold_pu);

    /**
     * @brief Executes one update cycle of the grid observer.
     *
     * All inputs required for the estimation must be passed in this call.
     * @param grid_voltage_pu The three-phase measured grid voltage [pu].
     */
    void update(const abc<voltage_pu_t>& grid_voltage_pu, const control::six_pulse_generator::pulses& grid_side_pulses);

    /**
     * @brief Resets all internal states of the observer.
     *
     * This will reset the PLL and the commutation monitor to their initial states.
     */
    void reset();

    /**
     * @brief Gets the estimated grid angle.
     * @return The estimated electrical angle of the grid [rad].
     */
    const angle<zero_to_two_pi>& get_estimated_angle() const;

    /**
     * @brief Gets the estimated grid frequency in Hz.
     * @return The estimated frequency of the grid [Hz].
     */
    frequency_t get_estimated_frequency() const;

    /**
     * @brief Gets the estimated grid frequency in per-unit.
     * @return The estimated frequency of the grid [pu].
     */
    frequency_pu_t get_estimated_frequency_pu() const;

    /**
     * @brief Gets the estimated magnitude of the grid voltage.
     * @return The estimated positive-sequence voltage magnitude [pu].
     */
    voltage_pu_t get_estimated_voltage_magnitude() const;

    /**
     * @brief Gets the filtered grid voltage in the alpha-beta frame.
     * @return The filtered stationary frame voltage components [pu].
     */
    alphabeta<voltage_pu_t> get_filtered_voltage_alphabeta() const;

    /**
     * @brief Gets the status of the commutation detection logic.
     * @return True if a commutation is currently detected in any of the thyristors, false otherwise.
     */
    bool is_commuting() const;

    /**
     * @brief Gets the PLL lock status.
     * @return True if the PLL is locked, false otherwise.
     */
    bool is_locked() const { return _pll.is_locked(); }

    bool is_masking() const { return _commutation_mask.is_masking(); }

private:
    control::dual_sogi_srf_pll _pll;
    sfc_commutation_monitor _commutation_monitor;
    sfc_commutation_mask _commutation_mask;

    // Configuration
    frequency_t _nominal_frequency{};

    // Outputs
    angle<zero_to_two_pi> _estimated_angle{};
    frequency_t _estimated_frequency{};
    frequency_pu_t _estimated_frequency_pu{};
    voltage_pu_t _estimated_voltage_magnitude{};
    alphabeta<voltage_pu_t> _filtered_voltage_alphabeta{};
    bool _is_commuting{false};
};

}  // namespace yeet::application::sfc