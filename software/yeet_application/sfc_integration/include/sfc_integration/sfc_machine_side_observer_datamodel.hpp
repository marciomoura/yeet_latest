#pragma once

#include "yeet/model_registry.hpp"
#include "yeet/parameter.hpp"
#include "yeet/signal.hpp"
#include "yeet_control/reference_frame.hpp"
#include "yeet_control/six_pulse_generator.hpp"
#include "yeet_control/units.hpp"

/**
 * @file sfc_machine_side_observer_datamodel.hpp
 * @brief Datamodel specification for SFC Machine-Side Observer component
 */

namespace yeet::application::sfc::integration {

class sfc_machine_side_observer_datamodel {
public:
    /**
     * @brief Input signals for the machine-side observer component
     */
    struct inputs {
        /// Three-phase machine voltage measurements in per-unit (machine-side base)
        /// Range: [0.0, 1.5] pu (wide range for variable speed operation)
        yeet::signal_input<yeet::abc<yeet::voltage_pu_t>> machine_voltage_pu;

        /// Machine-side converter firing pulses for commutation detection
        /// Type: six thyristor gate signals (true = firing)
        yeet::signal_input<yeet::control::six_pulse_generator::pulses> machine_side_pulses;
    };

    /**
     * @brief Output signals from the machine-side observer component
     */
    struct outputs {
        /// Estimated machine voltage magnitude
        /// Range: [0.0, 1.5] pu
        yeet::signal_output<yeet::voltage_pu_t> estimated_voltage_magnitude;

        /// Estimated machine frequency (electrical)
        /// Range: [0.0, 200.0] Hz for gas turbine applications
        yeet::signal_output<yeet::frequency_t> estimated_frequency;

        /// Estimated machine frequency in per-unit
        /// Range: [0.0, 4.0] pu for variable speed operation
        yeet::signal_output<yeet::frequency_pu_t> estimated_frequency_pu;

        /// Filtered machine voltage in alpha-beta stationary frame
        yeet::signal_output<yeet::alphabeta<yeet::voltage_pu_t>> filtered_voltage_alphabeta;

        /// Machine-side commutation detection status
        yeet::signal_output<bool> is_commuting;

        /// Observer convergence status
        yeet::signal_output<bool> is_converged;
    };

    /**
     * @brief Configuration parameters for the machine-side observer
     */
    struct parameters {
        /// Adaptive filter convergence rate
        /// Range: [0.001, 0.1]
        yeet::parameter<double> adaptive_filter_convergence_rate = 0.01;

        /// Minimum frequency for tracking
        /// Range: [0.1, 5.0] Hz
        yeet::parameter<double> minimum_tracking_frequency = 0.5;

        /// Maximum frequency for tracking
        /// Range: [100.0, 300.0] Hz
        yeet::parameter<double> maximum_tracking_frequency = 200.0;

        /// Voltage threshold for commutation detection
        /// Range: [0.05, 0.3] pu
        yeet::parameter<double> commutation_detection_threshold = 0.15;

        /// Time window for commutation event detection
        /// Range: [0.0005, 0.005] s
        yeet::parameter<double> commutation_detection_time_window = 0.001;

        /// Convergence threshold for observer
        /// Range: [0.01, 0.1] pu
        yeet::parameter<double> convergence_threshold = 0.05;

        /// Convergence time window
        /// Range: [0.1, 1.0] s
        yeet::parameter<double> convergence_time_window = 0.2;
    };

private:
    inputs* _model_inputs;
    outputs* _model_outputs;
    parameters* _model_parameters;

public:
    sfc_machine_side_observer_datamodel(inputs* model_inputs, outputs* model_outputs, parameters* model_parameters)
        : _model_inputs(model_inputs), _model_outputs(model_outputs), _model_parameters(model_parameters) {
        assert(_model_inputs != nullptr);
        assert(_model_outputs != nullptr);
        assert(_model_parameters != nullptr);
    }

    void register_data_model(yeet::model_registry& registry) {
        yeet::model_registry_entry entry{yeet::signal_registry{}, yeet::parameter_registry{}};

        // Register outputs in signal registry
        entry.signal_registry.insert("estimated_voltage_magnitude", _model_outputs->estimated_voltage_magnitude);
        entry.signal_registry.insert("estimated_frequency", _model_outputs->estimated_frequency);
        entry.signal_registry.insert("estimated_frequency_pu", _model_outputs->estimated_frequency_pu);
        entry.signal_registry.insert("filtered_voltage_alphabeta", _model_outputs->filtered_voltage_alphabeta);
        entry.signal_registry.insert("is_commuting", _model_outputs->is_commuting);
        entry.signal_registry.insert("is_converged", _model_outputs->is_converged);

        // Register parameters in parameter registry
        entry.parameter_registry.insert("adaptive_filter_convergence_rate",
                                        _model_parameters->adaptive_filter_convergence_rate);
        entry.parameter_registry.insert("minimum_tracking_frequency", _model_parameters->minimum_tracking_frequency);
        entry.parameter_registry.insert("maximum_tracking_frequency", _model_parameters->maximum_tracking_frequency);
        entry.parameter_registry.insert("commutation_detection_threshold",
                                        _model_parameters->commutation_detection_threshold);
        entry.parameter_registry.insert("commutation_detection_time_window",
                                        _model_parameters->commutation_detection_time_window);
        entry.parameter_registry.insert("convergence_threshold", _model_parameters->convergence_threshold);
        entry.parameter_registry.insert("convergence_time_window", _model_parameters->convergence_time_window);

        registry.insert("sfc_machine_side_observer", std::move(entry));
    }

    inputs* get_inputs() { return _model_inputs; }
    outputs* get_outputs() { return _model_outputs; }
    parameters* get_parameters() { return _model_parameters; }
};

/**
 * @brief OUTPUT INTERFACE
 *
 * Output signals provided by the machine-side observer component.
 * These match the expected getter method signatures:
 */
struct output_interface {
    /// Estimated machine voltage angle (electrical)
    /// Type: angle_t
    /// Range: [0, 2π] radians
    /// Usage: Machine electrical angle for synchronization and control
    /// Method: angle_t get_estimated_angle() const;
    using estimated_angle = void;  // angle_t

    /// Estimated machine frequency (absolute)
    /// Type: frequency_t
    /// Range: [0, 200] Hz for gas turbine applications
    /// Accuracy: ±0.5 Hz under normal conditions
    /// Method: frequency_t get_estimated_frequency() const;
    using estimated_frequency = void;  // frequency_t

    /// Estimated machine frequency in per-unit relative to machine base
    /// Type: frequency_pu_t
    /// Range: [0.0, 4.0] pu (0% to 400% of rated speed)
    /// Method: frequency_pu_t get_estimated_frequency_pu() const;
    using estimated_frequency_pu = void;  // frequency_pu_t

    /// Estimated machine voltage magnitude (back-EMF)
    /// Type: voltage_pu_t
    /// Range: [0.0, 1.2] pu
    /// Usage: Critical for inverter firing angle calculation
    /// Method: voltage_pu_t get_estimated_voltage_magnitude() const;
    using estimated_voltage_magnitude = void;  // voltage_pu_t

    /// Filtered machine voltage in alpha-beta stationary frame
    /// Type: alphabeta<voltage_pu_t>
    /// Usage: Clean voltage signals with commutation notches filtered out
    /// Method: alphabeta<voltage_pu_t> get_filtered_voltage_alphabeta() const;
    using filtered_voltage_alphabeta = void;  // alphabeta<voltage_pu_t>

    /// Machine-side commutation detection status
    /// Type: bool
    /// Usage: Indicates when machine-side converter commutation is occurring
    /// Method: bool is_commuting() const;
    using is_commuting = void;  // bool

    /// Observer lock status indication (for sensorless operation)
    /// Type: bool
    /// Usage: Indicates whether observer has reliable speed/position estimate
    /// Method: bool is_locked() const;
    using is_locked = void;  // bool
};

/**
 * @brief PARAMETER INTERFACE
 *
 * Configuration parameters for the machine-side observer component.
 */
struct parameter_interface {
    /// Observer proportional gain for frequency tracking
    /// Type: frequency_t
    /// Default: 100.0 Hz (lower than grid-side due to slower dynamics)
    /// Range: [20, 200] Hz
    /// Method: void configure_observer_gains(frequency_t kp, frequency_t ki);
    using observer_proportional_gain = void;  // frequency_t, default 100.0

    /// Observer integral gain for steady-state accuracy
    /// Type: frequency_t
    /// Default: 10000.0 Hz (lower than grid-side)
    /// Range: [5000, 50000] Hz
    /// Method: void configure_observer_gains(frequency_t kp, frequency_t ki);
    using observer_integral_gain = void;  // frequency_t, default 10000.0

    /// Nominal machine frequency for initialization
    /// Type: frequency_t
    /// Default: 50.0 Hz (or 60.0 Hz depending on system)
    /// Range: [50, 60] Hz
    /// Method: void configure_nominal_frequency(frequency_t f_nominal);
    using machine_nominal_frequency = void;  // frequency_t, default 50.0

    /// Voltage threshold for commutation detection
    /// Type: voltage_pu_t
    /// Default: 0.15 pu (higher than grid due to larger commutation notches)
    /// Range: [0.1, 0.5] pu
    /// Method: void configure_commutation_detection(voltage_pu_t threshold, duration_t time_window);
    using commutation_detection_threshold = void;  // voltage_pu_t, default 0.15

    /// Time window for commutation event detection
    /// Type: duration_t
    /// Default: 0.002 s (2 ms, longer than grid-side)
    /// Range: [0.001, 0.01] s
    /// Method: void configure_commutation_detection(voltage_pu_t threshold, duration_t time_window);
    using commutation_detection_time_window = void;  // duration_t, default 0.002

    /// Speed error threshold for observer lock detection
    /// Type: frequency_pu_t
    /// Default: 0.1 pu
    /// Range: [0.05, 0.2] pu
    /// Method: void configure_lock_detection(frequency_pu_t threshold, duration_t time_window);
    using lock_detection_threshold = void;  // frequency_pu_t, default 0.1

    /// Time window for observer lock verification
    /// Type: duration_t
    /// Default: 0.2 s (200 ms, longer than grid-side due to machine inertia)
    /// Range: [0.1, 1.0] s
    /// Method: void configure_lock_detection(frequency_pu_t threshold, duration_t time_window);
    using lock_detection_time_window = void;  // duration_t, default 0.2

    /// Adaptive filter cutoff frequency multiplier
    /// Type: real_t
    /// Default: 5.0 (cutoff = 5 * estimated_frequency)
    /// Range: [3.0, 10.0]
    /// Method: void configure_adaptive_filtering(real_t frequency_multiplier);
    /// Usage: Enables filter adaptation to machine speed for optimal harmonic rejection
    using adaptive_filter_multiplier = void;  // real_t, default 5.0
};

/**
 * @brief COMPONENT INTERFACE CONTRACT
 *
 * The implementing sfc_machine_side_observer class must provide these methods:
 *
 * Core Update Method:
 *   void update(const abc<voltage_pu_t>& machine_voltage_pu,
 *               const six_pulse_generator::pulses& machine_side_pulses);
 *
 * Configuration Methods:
 *   void configure_observer_gains(frequency_t kp, frequency_t ki);
 *   void configure_nominal_frequency(frequency_t f_nominal);
 *   void configure_commutation_detection(voltage_pu_t threshold, duration_t time_window);
 *   void configure_lock_detection(frequency_pu_t threshold, duration_t time_window);
 *   void configure_adaptive_filtering(real_t frequency_multiplier);
 *
 * Output Getter Methods:
 *   angle_t get_estimated_angle() const;
 *   frequency_t get_estimated_frequency() const;
 *   frequency_pu_t get_estimated_frequency_pu() const;
 *   voltage_pu_t get_estimated_voltage_magnitude() const;
 *   alphabeta<voltage_pu_t> get_filtered_voltage_alphabeta() const;
 *   bool is_commuting() const;
 *   bool is_locked() const;
 *
 * State Management Methods:
 *   void reset();
 *
 * Architectural Compliance:
 * - Memory: No dynamic allocation during runtime (real-time safety)
 * - Timing: Deterministic execution time < 15 μs per update
 * - Thread Safety: Not required (single-threaded real-time context)
 * - Error Handling: Graceful degradation when machine voltage is lost
 */

}  // namespace yeet::application::sfc::integration
