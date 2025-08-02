#pragma once

/**
 * @file sfc_grid_side_observer_datamodel.hpp
 * @brief Datamodel specification for SFC Grid-Side Observer component
 *
 * This file defines the interface contract for the SFC Grid-Side Observer component
 * following the yeet framework datamodel pattern. The datamodel specifies:
 *
 * 1. Input signal types and their meanings
 * 2. Output signal types and their meanings
 * 3. Configuration parameters and their default values
 * 4. Component interface expectations
 *
 * This datamodel should be used by:
 * - Integration test frameworks for signal connection verification
 * - Auto-generation tools for creating wrapper classes
 * - Documentation systems for API specification
 * - Runtime introspection and debugging tools
 */

namespace yeet::application::sfc::integration {

/**
 * @brief Datamodel for the SFC grid-side observer component.
 *
 * This datamodel defines the complete interface contract for the grid-side observer
 * which estimates the grid angle, frequency, voltage magnitude, and detects commutation.
 * It integrates a Phase-Locked Loop (PLL) and commutation monitor.
 *
 * Component Purpose:
 * The grid-side observer provides essential grid state estimation for the LCI control system.
 * It performs the following key functions:
 * - Grid angle estimation via PLL for synchronization
 * - Grid frequency tracking for adaptive control
 * - Voltage magnitude estimation for voltage regulation
 * - Commutation event detection for protection and timing
 * - PLL lock status monitoring for system health
 *
 * Real-time Performance Requirements:
 * - Update rate: Typically 10 kHz (100 μs)
 * - PLL bandwidth: 10-50 Hz for grid frequency tracking
 * - Commutation detection latency: < 50 μs
 * - Lock detection time: < 100 ms at startup
 */
class sfc_grid_side_observer_datamodel {
public:
    /**
     * @brief INPUT INTERFACE
     *
     * Input signals required by the grid-side observer component.
     * These match the expected update() method signature:
     *
     * void update(const abc<voltage_pu_t>& grid_voltage_pu,
     *             const six_pulse_generator::pulses& grid_side_pulses);
     */
    struct input_interface {
        /// Three-phase grid voltage measurements in per-unit (grid-side base)
        /// Type: abc<voltage_pu_t>
        /// Range: [0.8, 1.2] pu under normal operating conditions
        /// Frequency: Grid fundamental frequency (50/60 Hz) plus harmonics
        /// Usage: Primary input for PLL angle estimation and voltage magnitude calculation
        using grid_voltage_pu = void;  // abc<voltage_pu_t>

        /// Grid-side converter firing pulses for commutation detection
        /// Type: six_pulse_generator::pulses (std::array<bool, 6>)
        /// Usage: Enables commutation event detection and timing analysis
        /// Note: Each element represents one thyristor gate signal (true = firing)
        using grid_side_pulses = void;  // six_pulse_generator::pulses
    };

    /**
     * @brief OUTPUT INTERFACE
     *
     * Output signals provided by the grid-side observer component.
     * These match the expected getter method signatures:
     */
    struct output_interface {
        /// Estimated grid voltage angle (electrical)
        /// Type: angle_t
        /// Range: [0, 2π] radians
        /// Accuracy: ±0.01 rad under normal grid conditions
        /// Method: angle_t get_estimated_angle() const;
        using estimated_angle = void;  // angle_t

        /// Estimated grid frequency (absolute)
        /// Type: frequency_t
        /// Range: [45, 65] Hz for 50 Hz systems, [55, 75] Hz for 60 Hz systems
        /// Accuracy: ±0.1 Hz under normal conditions
        /// Method: frequency_t get_estimated_frequency() const;
        using estimated_frequency = void;  // frequency_t

        /// Estimated grid frequency in per-unit
        /// Type: frequency_pu_t
        /// Range: [0.9, 1.3] pu
        /// Method: frequency_pu_t get_estimated_frequency_pu() const;
        using estimated_frequency_pu = void;  // frequency_pu_t

        /// Estimated positive-sequence voltage magnitude
        /// Type: voltage_pu_t
        /// Range: [0.8, 1.2] pu under normal conditions
        /// Method: voltage_pu_t get_estimated_voltage_magnitude() const;
        using estimated_voltage_magnitude = void;  // voltage_pu_t

        /// Filtered grid voltage in alpha-beta stationary frame
        /// Type: alphabeta<voltage_pu_t>
        /// Usage: Provides clean voltage signals for downstream control
        /// Method: alphabeta<voltage_pu_t> get_filtered_voltage_alphabeta() const;
        using filtered_voltage_alphabeta = void;  // alphabeta<voltage_pu_t>

        /// Commutation detection status
        /// Type: bool
        /// Usage: Indicates when grid-side converter commutation is occurring
        /// Method: bool is_commuting() const;
        using is_commuting = void;  // bool

        /// PLL lock status indication
        /// Type: bool
        /// Usage: Indicates whether PLL has achieved frequency lock
        /// Method: bool is_locked() const;
        using is_locked = void;  // bool
    };

    /**
     * @brief PARAMETER INTERFACE
     *
     * Configuration parameters for the grid-side observer component.
     * These match the expected configuration method signatures:
     */
    struct parameter_interface {
        /// PLL proportional gain for frequency tracking
        /// Type: frequency_t
        /// Default: 200.0 Hz
        /// Range: [50, 500] Hz
        /// Method: void configure_pll_gains(frequency_t kp, frequency_t ki);
        using pll_proportional_gain = void;  // frequency_t, default 200.0

        /// PLL integral gain for steady-state accuracy
        /// Type: frequency_t
        /// Default: 40000.0 Hz
        /// Range: [10000, 100000] Hz
        /// Method: void configure_pll_gains(frequency_t kp, frequency_t ki);
        using pll_integral_gain = void;  // frequency_t, default 40000.0

        /// Nominal grid frequency for initialization
        /// Type: frequency_t
        /// Default: 50.0 Hz
        /// Range: [50, 60] Hz
        /// Method: void configure_nominal_frequency(frequency_t f_nominal);
        using grid_nominal_frequency = void;  // frequency_t, default 50.0

        /// Voltage threshold for commutation detection
        /// Type: voltage_pu_t
        /// Default: 0.1 pu
        /// Range: [0.05, 0.3] pu
        /// Method: void configure_commutation_detection(voltage_pu_t threshold, duration_t time_window);
        using commutation_detection_threshold = void;  // voltage_pu_t, default 0.1

        /// Time window for commutation event detection
        /// Type: duration_t
        /// Default: 0.001 s (1 ms)
        /// Range: [0.0005, 0.005] s
        /// Method: void configure_commutation_detection(voltage_pu_t threshold, duration_t time_window);
        using commutation_detection_time_window = void;  // duration_t, default 0.001

        /// Frequency error threshold for PLL lock detection
        /// Type: voltage_pu_t
        /// Default: 0.05 pu
        /// Range: [0.01, 0.1] pu
        /// Method: void configure_lock_detection(voltage_pu_t threshold, duration_t time_window);
        using lock_detection_threshold = void;  // voltage_pu_t, default 0.05

        /// Time window for PLL lock verification
        /// Type: duration_t
        /// Default: 0.1 s (100 ms)
        /// Range: [0.05, 0.5] s
        /// Method: void configure_lock_detection(voltage_pu_t threshold, duration_t time_window);
        using lock_detection_time_window = void;  // duration_t, default 0.1
    };

    /**
     * @brief COMPONENT INTERFACE CONTRACT
     *
     * The implementing sfc_grid_side_observer class must provide these methods:
     *
     * Core Update Method:
     *   void update(const abc<voltage_pu_t>& grid_voltage_pu,
     *               const six_pulse_generator::pulses& grid_side_pulses);
     *
     * Configuration Methods:
     *   void configure_pll_gains(frequency_t kp, frequency_t ki);
     *   void configure_nominal_frequency(frequency_t f_nominal);
     *   void configure_commutation_detection(voltage_pu_t threshold, duration_t time_window);
     *   void configure_lock_detection(voltage_pu_t threshold, duration_t time_window);
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
     * - Timing: Deterministic execution time < 10 μs per update
     * - Thread Safety: Not required (single-threaded real-time context)
     * - Error Handling: Fail-safe operation under input signal loss
     */
};

}  // namespace yeet::application::sfc::integration
