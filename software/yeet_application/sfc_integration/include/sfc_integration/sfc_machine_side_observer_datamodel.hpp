#pragma once

/**
 * @file sfc_machine_side_observer_datamodel.hpp
 * @brief Datamodel specification for SFC Machine-Side Observer component
 *
 * This file defines the interface contract for the SFC Machine-Side Observer component
 * following the yeet framework datamodel pattern. The observer estimates machine-side
 * electrical state including voltage magnitude, frequency, and commutation detection.
 */

namespace yeet::application::sfc::integration {

/**
 * @brief Datamodel for the SFC machine-side observer component.
 *
 * This datamodel defines the complete interface contract for the machine-side observer
 * which estimates the machine-side electrical state for LCI control. The observer
 * performs similar functions to the grid-side observer but adapted for the machine side:
 * - Machine voltage magnitude estimation for back-EMF calculation
 * - Machine frequency tracking for speed estimation
 * - Commutation event detection for machine-side converter protection
 * - Filtered voltage signals for downstream control loops
 *
 * Component Purpose:
 * The machine-side observer provides essential machine state estimation for the LCI control system.
 * It enables sensorless operation and provides feedback for:
 * - Inverter back-EMF calculation for firing angle control
 * - Machine speed estimation when encoders are not available
 * - Machine-side commutation monitoring for protection
 *
 * Real-time Performance Requirements:
 * - Update rate: Typically 10 kHz (100 μs)
 * - Frequency tracking range: 0-200 Hz for typical gas turbine applications
 * - Voltage magnitude accuracy: ±2% of rated voltage
 * - Commutation detection latency: < 50 μs
 */
class sfc_machine_side_observer_datamodel {
public:
    /**
     * @brief INPUT INTERFACE
     *
     * Input signals required by the machine-side observer component.
     * Expected update() method signature:
     *
     * void update(const abc<voltage_pu_t>& machine_voltage_pu,
     *             const six_pulse_generator::pulses& machine_side_pulses);
     */
    struct input_interface {
        /// Three-phase machine voltage measurements in per-unit (machine-side base)
        /// Type: abc<voltage_pu_t>
        /// Range: [0.0, 1.5] pu (wider range than grid due to variable speed operation)
        /// Frequency: Variable frequency (0-200 Hz) depending on machine speed
        /// Usage: Primary input for voltage magnitude estimation and back-EMF calculation
        /// Note: Contains commutation notches and harmonics from inverter operation
        using machine_voltage_pu = void;  // abc<voltage_pu_t>

        /// Machine-side converter firing pulses for commutation detection
        /// Type: six_pulse_generator::pulses (std::array<bool, 6>)
        /// Usage: Enables machine-side commutation event detection and timing analysis
        /// Note: Each element represents one thyristor gate signal (true = firing)
        using machine_side_pulses = void;  // six_pulse_generator::pulses
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
};

}  // namespace yeet::application::sfc::integration
