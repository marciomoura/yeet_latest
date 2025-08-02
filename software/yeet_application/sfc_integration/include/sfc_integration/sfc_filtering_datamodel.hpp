#pragma once

/**
 * @file sfc_filtering_datamodel.hpp
 * @brief Datamodel specification for SFC Filtering components
 *
 * This file defines the interface contracts for all SFC filtering components
 * following the yeet framework datamodel pattern. The filtering components
 * provide signal conditioning, noise reduction, and harmonic filtering.
 */

namespace yeet::application::sfc::integration {

/**
 * @brief Datamodel for the SFC voltage filtering component.
 *
 * This datamodel defines the interface contract for voltage signal filtering
 * including harmonic filtering, noise reduction, and signal conditioning.
 */
class sfc_voltage_filtering_datamodel {
public:
    /**
     * @brief INPUT INTERFACE
     *
     * Raw and preprocessed voltage signals for filtering.
     * Expected update() method signature:
     *
     * void update(const std::array<voltage_pu_t, 2>& voltage_alphabeta_pu,
     *             voltage_pu_t voltage_magnitude_pu,
     *             frequency_t fundamental_frequency,
     *             bool enable_harmonic_filtering);
     */
    struct input_interface {
        /// Input voltage in αβ coordinates (per-unit)
        /// Type: std::array<voltage_pu_t, 2>
        /// Range: [-1.5, 1.5] pu for each component
        /// Source: Grid-side or machine-side measurements after αβ transformation
        /// Usage: Primary voltage signals for filtering
        using voltage_alphabeta_pu = void;  // std::array<voltage_pu_t, 2>

        /// Voltage magnitude for adaptive filtering
        /// Type: voltage_pu_t
        /// Range: [0.0, 1.5] pu
        /// Source: Calculated magnitude from αβ components
        /// Usage: Controls adaptive filter parameters
        using voltage_magnitude_pu = void;  // voltage_pu_t

        /// Fundamental frequency for harmonic filtering
        /// Type: frequency_t (Hz)
        /// Range: [45.0, 65.0] Hz for grid-side, [0.1, 120.0] Hz for machine-side
        /// Source: Grid-side or machine-side observer frequency estimation
        /// Usage: Adapts harmonic filter notch frequencies
        using fundamental_frequency = void;  // frequency_t

        /// Enable signal for harmonic filtering
        /// Type: bool
        /// Source: Control system configuration
        /// Usage: Switches between basic and harmonic filtering modes
        using enable_harmonic_filtering = void;  // bool
    };

    /**
     * @brief OUTPUT INTERFACE
     */
    struct output_interface {
        /// Filtered voltage in αβ coordinates
        /// Type: std::array<voltage_pu_t, 2>
        /// Range: [-1.5, 1.5] pu for each component
        /// Method: std::array<voltage_pu_t, 2> get_filtered_voltage_alphabeta_pu() const;
        using filtered_voltage_alphabeta_pu = void;  // std::array<voltage_pu_t, 2>

        /// Low-pass filtered voltage (noise reduction)
        /// Type: std::array<voltage_pu_t, 2>
        /// Range: [-1.5, 1.5] pu for each component
        /// Method: std::array<voltage_pu_t, 2> get_lowpass_voltage_alphabeta_pu() const;
        using lowpass_voltage_alphabeta_pu = void;  // std::array<voltage_pu_t, 2>

        /// Harmonic component estimate (5th, 7th, 11th, 13th)
        /// Type: std::array<voltage_pu_t, 8> (αβ for each harmonic)
        /// Range: [-0.5, 0.5] pu for each component
        /// Method: std::array<voltage_pu_t, 8> get_harmonic_components() const;
        using harmonic_components = void;  // std::array<voltage_pu_t, 8>

        /// Filtered voltage magnitude
        /// Type: voltage_pu_t
        /// Range: [0.0, 1.5] pu
        /// Method: voltage_pu_t get_filtered_voltage_magnitude_pu() const;
        using filtered_voltage_magnitude_pu = void;  // voltage_pu_t

        /// Total harmonic distortion estimate
        /// Type: real_t (percentage)
        /// Range: [0.0, 50.0] %
        /// Method: real_t get_total_harmonic_distortion() const;
        using total_harmonic_distortion = void;  // real_t

        /// Filter convergence status
        /// Type: bool
        /// Method: bool is_converged() const;
        using is_converged = void;  // bool
    };

    /**
     * @brief PARAMETER INTERFACE
     */
    struct parameter_interface {
        /// Low-pass filter cutoff frequency
        /// Type: frequency_t (Hz)
        /// Default: 1000.0 Hz
        /// Range: [100.0, 5000.0] Hz
        using lowpass_cutoff_frequency = void;  // frequency_t, default 1000.0

        /// Harmonic filter quality factor
        /// Type: real_t
        /// Default: 10.0
        /// Range: [5.0, 50.0]
        using harmonic_filter_q_factor = void;  // real_t, default 10.0

        /// Adaptive filter convergence rate
        /// Type: real_t
        /// Default: 0.01
        /// Range: [0.001, 0.1]
        using adaptive_convergence_rate = void;  // real_t, default 0.01

        /// Enable 5th harmonic filtering
        /// Type: bool
        /// Default: true
        using enable_5th_harmonic_filter = void;  // bool, default true

        /// Enable 7th harmonic filtering
        /// Type: bool
        /// Default: true
        using enable_7th_harmonic_filter = void;  // bool, default true

        /// Enable 11th harmonic filtering
        /// Type: bool
        /// Default: false
        using enable_11th_harmonic_filter = void;  // bool, default false

        /// Enable 13th harmonic filtering
        /// Type: bool
        /// Default: false
        using enable_13th_harmonic_filter = void;  // bool, default false
    };
};

/**
 * @brief Datamodel for the SFC current filtering component.
 */
class sfc_current_filtering_datamodel {
public:
    /**
     * @brief INPUT INTERFACE
     */
    struct input_interface {
        /// Input current in αβ coordinates (per-unit)
        /// Type: std::array<current_pu_t, 2>
        /// Range: [-2.0, 2.0] pu for each component
        /// Source: Grid-side or machine-side current measurements
        using current_alphabeta_pu = void;  // std::array<current_pu_t, 2>

        /// Current magnitude for adaptive filtering
        /// Type: current_pu_t
        /// Range: [0.0, 2.0] pu
        /// Source: Calculated magnitude from αβ components
        using current_magnitude_pu = void;  // current_pu_t

        /// Fundamental frequency for harmonic filtering
        /// Type: frequency_t (Hz)
        /// Range: [45.0, 65.0] Hz for grid-side, [0.1, 120.0] Hz for machine-side
        /// Source: Observer frequency estimation
        using fundamental_frequency = void;  // frequency_t

        /// Enable signal for harmonic filtering
        /// Type: bool
        /// Source: Control system configuration
        using enable_harmonic_filtering = void;  // bool
    };

    /**
     * @brief OUTPUT INTERFACE
     */
    struct output_interface {
        /// Filtered current in αβ coordinates
        /// Type: std::array<current_pu_t, 2>
        /// Method: std::array<current_pu_t, 2> get_filtered_current_alphabeta_pu() const;
        using filtered_current_alphabeta_pu = void;  // std::array<current_pu_t, 2>

        /// Low-pass filtered current (noise reduction)
        /// Type: std::array<current_pu_t, 2>
        /// Method: std::array<current_pu_t, 2> get_lowpass_current_alphabeta_pu() const;
        using lowpass_current_alphabeta_pu = void;  // std::array<current_pu_t, 2>

        /// Harmonic current components
        /// Type: std::array<current_pu_t, 8> (αβ for each harmonic)
        /// Method: std::array<current_pu_t, 8> get_harmonic_components() const;
        using harmonic_components = void;  // std::array<current_pu_t, 8>

        /// Filtered current magnitude
        /// Type: current_pu_t
        /// Method: current_pu_t get_filtered_current_magnitude_pu() const;
        using filtered_current_magnitude_pu = void;  // current_pu_t

        /// Current THD estimate
        /// Type: real_t (percentage)
        /// Method: real_t get_total_harmonic_distortion() const;
        using total_harmonic_distortion = void;  // real_t

        /// Filter convergence status
        /// Type: bool
        /// Method: bool is_converged() const;
        using is_converged = void;  // bool
    };

    /**
     * @brief PARAMETER INTERFACE
     */
    struct parameter_interface {
        /// Low-pass filter cutoff frequency
        /// Type: frequency_t (Hz)
        /// Default: 2000.0 Hz
        /// Range: [500.0, 10000.0] Hz
        using lowpass_cutoff_frequency = void;  // frequency_t, default 2000.0

        /// Harmonic filter quality factor
        /// Type: real_t
        /// Default: 15.0
        /// Range: [5.0, 50.0]
        using harmonic_filter_q_factor = void;  // real_t, default 15.0

        /// Adaptive filter convergence rate
        /// Type: real_t
        /// Default: 0.02
        /// Range: [0.001, 0.1]
        using adaptive_convergence_rate = void;  // real_t, default 0.02

        /// Enable 5th harmonic filtering
        /// Type: bool
        /// Default: true
        using enable_5th_harmonic_filter = void;  // bool, default true

        /// Enable 7th harmonic filtering
        /// Type: bool
        /// Default: true
        using enable_7th_harmonic_filter = void;  // bool, default true

        /// Enable 11th harmonic filtering
        /// Type: bool
        /// Default: true
        using enable_11th_harmonic_filter = void;  // bool, default true

        /// Enable 13th harmonic filtering
        /// Type: bool
        /// Default: true
        using enable_13th_harmonic_filter = void;  // bool, default true
    };
};

/**
 * @brief Datamodel for the SFC notch filtering component.
 *
 * Provides targeted filtering for specific frequency components that may
 * interfere with control system operation.
 */
class sfc_notch_filtering_datamodel {
public:
    /**
     * @brief INPUT INTERFACE
     */
    struct input_interface {
        /// Input signal for notch filtering
        /// Type: real_t
        /// Range: [-10.0, 10.0] (general purpose)
        /// Source: Any signal requiring notch filtering
        using input_signal = void;  // real_t

        /// Target notch frequency
        /// Type: frequency_t (Hz)
        /// Range: [1.0, 5000.0] Hz
        /// Source: Frequency identification or configuration
        using notch_frequency = void;  // frequency_t

        /// Enable signal for notch filtering
        /// Type: bool
        /// Source: Control system configuration
        using enable_notch_filter = void;  // bool
    };

    /**
     * @brief OUTPUT INTERFACE
     */
    struct output_interface {
        /// Notch-filtered output signal
        /// Type: real_t
        /// Method: real_t get_filtered_signal() const;
        using filtered_signal = void;  // real_t

        /// Removed frequency component
        /// Type: real_t
        /// Method: real_t get_removed_component() const;
        using removed_component = void;  // real_t

        /// Filter magnitude response at notch frequency
        /// Type: real_t
        /// Method: real_t get_notch_attenuation() const;
        using notch_attenuation = void;  // real_t
    };

    /**
     * @brief PARAMETER INTERFACE
     */
    struct parameter_interface {
        /// Notch filter quality factor
        /// Type: real_t
        /// Default: 20.0
        /// Range: [5.0, 100.0]
        using notch_q_factor = void;  // real_t, default 20.0

        /// Notch depth (attenuation)
        /// Type: real_t (dB)
        /// Default: -40.0 dB
        /// Range: [-60.0, -20.0] dB
        using notch_depth_db = void;  // real_t, default -40.0

        /// Adaptive frequency tracking rate
        /// Type: real_t
        /// Default: 0.001
        /// Range: [0.0001, 0.01]
        using frequency_tracking_rate = void;  // real_t, default 0.001
    };
};

}  // namespace yeet::application::sfc::integration
