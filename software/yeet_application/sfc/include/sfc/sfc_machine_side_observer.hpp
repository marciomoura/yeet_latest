#pragma once

#include "yeet_control/angle_wrapped.hpp"
#include "yeet_control/reference_frame.hpp"
#include "yeet_control/second_order_filter.hpp"
#include "yeet_control/srf_pll.hpp"
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
    static constexpr frequency_t default_voltage_estimation_filter_cutoff{frequency_t{10.0}};
    static constexpr frequency_t default_frequency_estimation_filter_cutoff{frequency_t{10.0}};

    /**
     * @brief Constructs the grid-side observer.
     * @param sampling_period The sampling period of the control loop.
     * @param nominal_frequency The nominal grid frequency.
     */
    explicit sfc_machine_side_observer(duration_t sampling_period);

    /**
     * @brief Configures the gains for the internal Phase-Locked Loop.
     * @param kp The proportional gain for the PLL.
     * @param ti The integral time constant for the PLL.
     * @param nominal_frequency The nominal frequency.
     */
    void configure_pll(real_t kp, duration_t ti, frequency_t nominal_frequency);

    /**
     * @brief Configures the voltage filter.
     * @param cutoff_frequency The cutoff frequency for the filter.
     * @param damping_ratio The damping ratio for the filter.
     */
    void configure_voltage_filter(frequency_t cutoff_frequency, real_t damping_ratio = 0.707);

    /**
     * @brief Configures the frequency estimation filter.
     * @param cutoff_frequency The cutoff frequency for the filter.
     * @param damping_ratio The damping ratio for the filter.
     */
    void configure_frequency_filter(frequency_t cutoff_frequency, real_t damping_ratio = 0.707);

    /**
     * @brief Executes one update cycle of the grid observer.
     *
     * All inputs required for the estimation must be passed in this call.
     * @param grid_voltage_abc The three-phase measured grid voltage [pu].
     */
    void update(const abc<voltage_pu_t>& grid_voltage_abc);

    /**
     * @brief Resets all internal states of the observer.
     *
     * This will reset the PLL and the commutation monitor to their initial states.
     * @param initial_frequency The initial frequency to reset the observer.
     */
    void reset(frequency_pu_t initial_frequency = frequency_pu_t{1.0});

    /**
     * @brief Presets the observer angle integrator and frequency to specific values.
     *
     * This method allows external synchronization by directly setting the
     * internal PLL phase integrator to a specific angle and frequency. Used for
     * bumpless transfer from virtual reference frame during startup.
     *
     * @param preset_angle The angle to preset the integrator to [rad].
     * @param preset_frequency The frequency to preset the estimate to [pu].
     */
    void preset(angle_wrapped preset_angle, frequency_pu_t preset_frequency = frequency_pu_t{1.0});

    /**
     * @brief Presets the observer and internal PLL for bumpless transfer.
     *
     * Uses the current measured voltage to compute the PLL phase error at the preset angle and
     * preloads the PLL PI controller integrator to avoid a transient when switching from a virtual
     * reference frame to the PLL-based estimate.
     *
     * @param preset_angle The phase-a aligned angle used by the observer outputs [rad].
     * @param preset_frequency The frequency to preset the estimate to [pu].
     * @param grid_voltage_abc The three-phase measured machine voltage [pu].
     */
    void preset_for_bumpless_transfer(
        angle_wrapped preset_angle, frequency_pu_t preset_frequency, const abc<voltage_pu_t>& grid_voltage_abc);

    /**
     * @brief Gets the estimated grid angle.
     * @return The estimated electrical angle of the grid [rad].
     */
    const angle_wrapped& get_estimated_angle() const;

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
     * @brief Gets the observer tracking error (q-axis component from PLL).
     * @return The q-axis voltage component representing phase tracking error [pu].
     */
    voltage_pu_t get_observer_error() const;

    /**
     * @brief Gets the status of the commutation detection logic.
     * @return True if a commutation is currently detected in any of the thyristors, false otherwise.
     */
    bool is_commuting() const;

private:
    control::srf_pll _pll;
    control::second_order_low_pass_filter<real_t> _filtered_voltage_filter;
    control::second_order_low_pass_filter<real_t> _filtered_frequency;

    // Configuration
    frequency_t _nominal_frequency{};

    // Outputs
    angle_wrapped _estimated_angle{};
    frequency_t _estimated_frequency{};
    frequency_pu_t _estimated_frequency_pu{};
    voltage_pu_t _estimated_voltage_magnitude{};
    alphabeta<voltage_pu_t> _filtered_voltage_alphabeta{};
};

}  // namespace yeet::application::sfc