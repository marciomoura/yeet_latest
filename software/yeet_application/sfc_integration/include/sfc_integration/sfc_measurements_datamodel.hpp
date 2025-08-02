#pragma once

/**
 * @file sfc_measurements_datamodel.hpp
 * @brief Datamodel specification for SFC Measurement components
 *
 * This file defines the interface contracts for all SFC measurement components
 * following the yeet framework datamodel pattern. The measurement components
 * provide signal acquisition, conditioning, and filtering for the control system.
 */

namespace yeet::application::sfc::integration {

/**
 * @brief Datamodel for the SFC grid-side measurements component.
 *
 * This datamodel defines the interface contract for grid-side voltage and current
 * measurements with signal conditioning and per-unit conversion.
 */
class sfc_grid_side_measurements_datamodel {
public:
    /**
     * @brief INPUT INTERFACE
     *
     * Raw measurement signals from grid-side sensors.
     * Expected update() method signature:
     *
     * void update(const std::array<voltage_t, 3>& line_voltages,
     *             const std::array<current_t, 3>& line_currents,
     *             voltage_t dc_link_voltage,
     *             bool sensor_health_ok);
     */
    struct input_interface {
        /// Three-phase line-to-line voltages (raw sensor values)
        /// Type: std::array<voltage_t, 3>
        /// Range: [0, 20000] V typical for medium voltage applications
        /// Source: Grid voltage transformers (VTs)
        /// Usage: Primary voltage measurements for control and protection
        using line_voltages = void;  // std::array<voltage_t, 3>

        /// Three-phase line currents (raw sensor values)
        /// Type: std::array<current_t, 3>
        /// Range: [0, 5000] A typical for high power applications
        /// Source: Current transformers (CTs) on grid side
        /// Usage: Grid current measurements for control and protection
        using line_currents = void;  // std::array<current_t, 3>

        /// DC link voltage measurement
        /// Type: voltage_t
        /// Range: [0, 15000] V typical
        /// Source: DC link voltage sensor
        /// Usage: DC voltage monitoring and control feedback
        using dc_link_voltage = void;  // voltage_t

        /// Sensor health status
        /// Type: bool
        /// Source: Sensor diagnostic system
        /// Usage: Indicates validity of measurement data
        using sensor_health_ok = void;  // bool
    };

    /**
     * @brief OUTPUT INTERFACE
     */
    struct output_interface {
        /// Grid voltages in per-unit (αβ frame)
        /// Type: std::array<voltage_pu_t, 2>
        /// Range: [0.0, 1.5] pu
        /// Method: std::array<voltage_pu_t, 2> get_grid_voltage_alphabeta_pu() const;
        using grid_voltage_alphabeta_pu = void;  // std::array<voltage_pu_t, 2>

        /// Grid currents in per-unit (αβ frame)
        /// Type: std::array<current_pu_t, 2>
        /// Range: [-2.0, 2.0] pu
        /// Method: std::array<current_pu_t, 2> get_grid_current_alphabeta_pu() const;
        using grid_current_alphabeta_pu = void;  // std::array<current_pu_t, 2>

        /// Grid voltage magnitude in per-unit
        /// Type: voltage_pu_t
        /// Range: [0.0, 1.5] pu
        /// Method: voltage_pu_t get_grid_voltage_magnitude_pu() const;
        using grid_voltage_magnitude_pu = void;  // voltage_pu_t

        /// Grid current magnitude in per-unit
        /// Type: current_pu_t
        /// Range: [0.0, 2.0] pu
        /// Method: current_pu_t get_grid_current_magnitude_pu() const;
        using grid_current_magnitude_pu = void;  // current_pu_t

        /// DC link voltage in per-unit
        /// Type: voltage_pu_t
        /// Range: [0.0, 1.5] pu
        /// Method: voltage_pu_t get_dc_voltage_pu() const;
        using dc_voltage_pu = void;  // voltage_pu_t

        /// Measurement validity status
        /// Type: bool
        /// Method: bool is_valid() const;
        using is_valid = void;  // bool
    };

    /**
     * @brief PARAMETER INTERFACE
     */
    struct parameter_interface {
        /// Voltage sensor scaling factor
        /// Type: real_t
        /// Default: 1.0
        /// Range: [0.5, 2.0]
        using voltage_scale_factor = void;  // real_t, default 1.0

        /// Current sensor scaling factor
        /// Type: real_t
        /// Default: 1.0
        /// Range: [0.5, 2.0]
        using current_scale_factor = void;  // real_t, default 1.0

        /// Low-pass filter cutoff frequency
        /// Type: frequency_t (Hz)
        /// Default: 1000.0 Hz
        /// Range: [100.0, 5000.0] Hz
        using filter_cutoff_frequency = void;  // frequency_t, default 1000.0
    };
};

/**
 * @brief Datamodel for the SFC machine-side measurements component.
 */
class sfc_machine_side_measurements_datamodel {
public:
    /**
     * @brief INPUT INTERFACE
     */
    struct input_interface {
        /// Machine terminal voltages (line-to-line)
        /// Type: std::array<voltage_t, 3>
        /// Range: [0, 10000] V typical
        /// Source: Machine voltage sensors
        using machine_voltages = void;  // std::array<voltage_t, 3>

        /// Machine phase currents
        /// Type: std::array<current_t, 3>
        /// Range: [-5000, 5000] A typical
        /// Source: Machine current sensors
        using machine_currents = void;  // std::array<current_t, 3>

        /// Machine speed measurement
        /// Type: speed_t (rad/s)
        /// Range: [-1000, 1000] rad/s typical
        /// Source: Speed encoder or resolver
        using machine_speed = void;  // speed_t

        /// Rotor position
        /// Type: angle_t (radians)
        /// Range: [0.0, 2π] radians
        /// Source: Position encoder or resolver
        using rotor_position = void;  // angle_t

        /// Sensor health status
        /// Type: bool
        /// Source: Sensor diagnostic system
        using sensor_health_ok = void;  // bool
    };

    /**
     * @brief OUTPUT INTERFACE
     */
    struct output_interface {
        /// Machine voltages in per-unit (αβ frame)
        /// Type: std::array<voltage_pu_t, 2>
        /// Method: std::array<voltage_pu_t, 2> get_machine_voltage_alphabeta_pu() const;
        using machine_voltage_alphabeta_pu = void;  // std::array<voltage_pu_t, 2>

        /// Machine currents in per-unit (αβ frame)
        /// Type: std::array<current_pu_t, 2>
        /// Method: std::array<current_pu_t, 2> get_machine_current_alphabeta_pu() const;
        using machine_current_alphabeta_pu = void;  // std::array<current_pu_t, 2>

        /// Machine speed in per-unit
        /// Type: speed_pu_t
        /// Method: speed_pu_t get_machine_speed_pu() const;
        using machine_speed_pu = void;  // speed_pu_t

        /// Rotor position electrical
        /// Type: angle_t
        /// Method: angle_t get_rotor_position_electrical() const;
        using rotor_position_electrical = void;  // angle_t

        /// Machine voltage magnitude in per-unit
        /// Type: voltage_pu_t
        /// Method: voltage_pu_t get_machine_voltage_magnitude_pu() const;
        using machine_voltage_magnitude_pu = void;  // voltage_pu_t

        /// Machine current magnitude in per-unit
        /// Type: current_pu_t
        /// Method: current_pu_t get_machine_current_magnitude_pu() const;
        using machine_current_magnitude_pu = void;  // current_pu_t

        /// Measurement validity status
        /// Type: bool
        /// Method: bool is_valid() const;
        using is_valid = void;  // bool
    };

    /**
     * @brief PARAMETER INTERFACE
     */
    struct parameter_interface {
        /// Machine voltage scaling factor
        /// Type: real_t
        /// Default: 1.0
        /// Range: [0.5, 2.0]
        using voltage_scale_factor = void;  // real_t, default 1.0

        /// Machine current scaling factor
        /// Type: real_t
        /// Default: 1.0
        /// Range: [0.5, 2.0]
        using current_scale_factor = void;  // real_t, default 1.0

        /// Speed measurement scaling factor
        /// Type: real_t
        /// Default: 1.0
        /// Range: [0.5, 2.0]
        using speed_scale_factor = void;  // real_t, default 1.0

        /// Position sensor pole pairs
        /// Type: int
        /// Default: 1
        /// Range: [1, 10]
        using pole_pairs = void;  // int, default 1

        /// Anti-aliasing filter cutoff frequency
        /// Type: frequency_t (Hz)
        /// Default: 2000.0 Hz
        /// Range: [500.0, 5000.0] Hz
        using filter_cutoff_frequency = void;  // frequency_t, default 2000.0
    };
};

/**
 * @brief Datamodel for the SFC DC link measurements component.
 */
class sfc_dc_link_measurements_datamodel {
public:
    /**
     * @brief INPUT INTERFACE
     */
    struct input_interface {
        /// DC link voltage measurement
        /// Type: voltage_t
        /// Range: [0, 15000] V typical
        /// Source: DC voltage sensor
        using dc_voltage = void;  // voltage_t

        /// DC link current measurement
        /// Type: current_t
        /// Range: [-5000, 5000] A typical
        /// Source: DC current sensor (shunt or hall effect)
        using dc_current = void;  // current_t

        /// Sensor health status
        /// Type: bool
        /// Source: Sensor diagnostic system
        using sensor_health_ok = void;  // bool
    };

    /**
     * @brief OUTPUT INTERFACE
     */
    struct output_interface {
        /// DC voltage in per-unit
        /// Type: voltage_pu_t
        /// Range: [0.0, 1.5] pu
        /// Method: voltage_pu_t get_dc_voltage_pu() const;
        using dc_voltage_pu = void;  // voltage_pu_t

        /// DC current in per-unit
        /// Type: current_pu_t
        /// Range: [-2.0, 2.0] pu
        /// Method: current_pu_t get_dc_current_pu() const;
        using dc_current_pu = void;  // current_pu_t

        /// Filtered DC voltage (low noise)
        /// Type: voltage_pu_t
        /// Range: [0.0, 1.5] pu
        /// Method: voltage_pu_t get_dc_voltage_filtered_pu() const;
        using dc_voltage_filtered_pu = void;  // voltage_pu_t

        /// Filtered DC current (low noise)
        /// Type: current_pu_t
        /// Range: [-2.0, 2.0] pu
        /// Method: current_pu_t get_dc_current_filtered_pu() const;
        using dc_current_filtered_pu = void;  // current_pu_t

        /// Instantaneous DC power
        /// Type: power_pu_t
        /// Range: [-3.0, 3.0] pu
        /// Method: power_pu_t get_dc_power_pu() const;
        using dc_power_pu = void;  // power_pu_t

        /// Measurement validity status
        /// Type: bool
        /// Method: bool is_valid() const;
        using is_valid = void;  // bool
    };

    /**
     * @brief PARAMETER INTERFACE
     */
    struct parameter_interface {
        /// DC voltage scaling factor
        /// Type: real_t
        /// Default: 1.0
        /// Range: [0.5, 2.0]
        using voltage_scale_factor = void;  // real_t, default 1.0

        /// DC current scaling factor
        /// Type: real_t
        /// Default: 1.0
        /// Range: [0.5, 2.0]
        using current_scale_factor = void;  // real_t, default 1.0

        /// Voltage filter time constant
        /// Type: time_t (seconds)
        /// Default: 0.001 s
        /// Range: [0.0001, 0.01] s
        using voltage_filter_time_constant = void;  // time_t, default 0.001

        /// Current filter time constant
        /// Type: time_t (seconds)
        /// Default: 0.0005 s
        /// Range: [0.0001, 0.005] s
        using current_filter_time_constant = void;  // time_t, default 0.0005
    };
};

}  // namespace yeet::application::sfc::integration
