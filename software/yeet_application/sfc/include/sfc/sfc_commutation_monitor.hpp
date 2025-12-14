#pragma once

#include "yeet_control/edge_detector.hpp"
#include "yeet_control/elapsed_timer.hpp"
#include "yeet_control/first_order_low_pass_filter.hpp"
#include "yeet_control/reference_frame.hpp"
#include "yeet_control/six_pulse_generator.hpp"
#include "yeet_control/units.hpp"

namespace yeet {

/**
 * @brief Monitors the commutation notches of a 3-phase thyristor bridge.
 *
 * This class implements an algorithm to detect commutation periods. It uses
 * thyristor firing pulses to start the detection and monitors line-to-line
 * voltages to determine the end of commutation.
 */
class sfc_commutation_monitor {
public:
    /**
     * @brief Configuration parameters for the SfcCommutationMonitor.
     */
    struct Config {
        /**
         * @brief The per-unit voltage level below which commutation is considered to have started,
         * but only after a firing pulse has armed the detection.
         */
        voltage_pu_t commutation_start_threshold{0.02F};

        /**
         * @brief The per-unit voltage level that the line voltage must cross for
         * the commutation to be considered complete.
         */
        voltage_pu_t commutation_end_threshold{0.03F};
    };

    /**
     * @brief Constructs an SfcCommutationMonitor with the given configuration.
     *
     * @param config The configuration parameters for the monitor.
     * @param sampling_time Control execution period used to measure notch durations.
     */
    explicit sfc_commutation_monitor(const Config& config, duration_t sampling_time = duration_t{200e-6F}) noexcept;

    /**
     * @brief Configure the filter time constant used to smooth the measured notch duration.
     *
     * Internally uses yeet_control::first_order_low_pass_filter.
     *
     * The time constant $\tau$ is mapped to a cutoff frequency using:
     * $$ f_c = \frac{1}{2\pi\tau} $$
     */
    void configure_notch_duration_filter_time_constant(duration_t time_constant) noexcept;

    /**
     * @brief Sets the configuration parameters for the monitor.
     *
     * @param config The new configuration parameters.
     */
    void set_parameters(const Config& config);

    /**
     * @brief Updates the commutation status based on the latest phase voltage measurements and firing pulses.
     *
     * This function should be called at each time step of the control loop.
     *
     * @param phase_voltages The instantaneous per-unit measurements of the three phase voltages.
     * @param pulses The latest firing pulses from the six-pulse generator.
     * @return true if any phase is currently commuting, false otherwise.
     */
    bool update(const abc<voltage_pu_t>& phase_voltages, const control::six_pulse_generator::pulses& pulses);

    /**
     * @brief Returns the current commutation status.
     *
     * @return true if any phase is currently commuting, false otherwise.
     */
    [[nodiscard]] bool is_commuting() const;

    /**
     * @brief Last completed commutation notch duration [s].
     */
    [[nodiscard]] duration_t get_notch_duration_last() const noexcept;

    /**
     * @brief Filtered commutation notch duration [s].
     */
    [[nodiscard]] duration_t get_notch_duration_filtered() const noexcept;

private:
    /**
     * @brief Updates the commutation state for a single line voltage.
     *
     * @param is_line_commuting The current commutation state for the line.
     * @param line_voltage The current absolute line voltage.
     * @param has_fired True if a relevant thyristor has just been fired.
     */
    void update_line_commutation_state(
        bool& is_line_commuting, bool& is_line_armed, voltage_pu_t line_voltage, bool has_fired);

    Config _config;

    // Internal state flags for each line voltage's commutation status.
    bool _is_commuting_ab{false};
    bool _is_commuting_bc{false};
    bool _is_commuting_ca{false};

    // Armed state flags for each line voltage.
    bool _is_armed_ab{false};
    bool _is_armed_bc{false};
    bool _is_armed_ca{false};

    control::edge_detector _p1_detector;
    control::edge_detector _p2_detector;
    control::edge_detector _p3_detector;
    control::edge_detector _p4_detector;
    control::edge_detector _p5_detector;
    control::edge_detector _p6_detector;

    duration_t _sampling_time{200e-6F};
    duration_t _notch_duration_last{0.0F};
    duration_t _notch_duration_filter_time_constant{10e-3F};

    control::elapsed_timer _notch_duration_timer;
    control::first_order_low_pass_filter<duration_t> _notch_duration_filter;
};

}  // namespace yeet