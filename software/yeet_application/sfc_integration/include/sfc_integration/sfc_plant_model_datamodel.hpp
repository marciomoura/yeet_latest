#pragma once

/**
 * @file sfc_plant_model_datamodel.hpp
 * @brief Datamodel specification for SFC Plant Model component
 *
 * This file defines the interface contract for the SFC Plant Model component
 * following the yeet framework datamodel pattern. The plant model implements
 * the complete electrical and mechanical system dynamics.
 */

namespace yeet::application::sfc::integration {

/**
 * @brief Datamodel for the SFC 6-pulse to 6-pulse plant model component.
 *
 * This datamodel defines the complete interface contract for the SFC plant model
 * which simulates the full LCI drive system including grid interface, converters,
 * DC link, machine, and mechanical load dynamics.
 *
 * Component Purpose:
 * The SFC plant model provides:
 * - Complete electrical system simulation (grid-side rectifier, DC link, machine-side inverter)
 * - Machine electrical dynamics (synchronous machine model with damper windings)
 * - Mechanical system dynamics (inertia, load torque, friction)
 * - Grid interface modeling (impedance, distortion, voltage regulation)
 *
 * Model Complexity:
 * - Grid-side: 6-pulse thyristor rectifier with commutation modeling
 * - DC link: Inductance and resistance with current ripple dynamics
 * - Machine-side: 6-pulse thyristor inverter with commutation overlap
 * - Machine: Detailed synchronous machine with field, damper windings
 * - Mechanical: Multi-mass system with flexible coupling (optional)
 *
 * Real-time Performance Requirements:
 * - Update rate: 10 kHz (100 μs) for control system testing
 * - Integration method: Fixed-step explicit or semi-implicit
 * - Accuracy: Sufficient for control system validation and tuning
 * - Stability: Numerically stable across full operating range
 */
class sfc_6p_6p_plant_model_datamodel {
public:
    /**
     * @brief INPUT INTERFACE
     *
     * Input signals required by the plant model component.
     * Expected update() method signature:
     *
     * void update(const std::array<bool, 6>& grid_side_firing_pulses,
     *             const std::array<bool, 6>& machine_side_firing_pulses,
     *             voltage_t grid_voltage_magnitude,
     *             angle_t grid_voltage_angle,
     *             frequency_t grid_frequency,
     *             torque_t external_load_torque,
     *             time_t delta_time);
     */
    struct input_interface {
        /// Grid-side thyristor firing pulses (T1-T6)
        /// Type: std::array<bool, 6>
        /// Source: Grid-side modulator firing pulse generation
        /// Usage: Controls grid-side rectifier thyristor switching
        using grid_side_firing_pulses = void;  // std::array<bool, 6>

        /// Machine-side thyristor firing pulses (T1-T6)
        /// Type: std::array<bool, 6>
        /// Source: Machine-side modulator firing pulse generation
        /// Usage: Controls machine-side inverter thyristor switching
        using machine_side_firing_pulses = void;  // std::array<bool, 6>

        /// Grid voltage magnitude (line-to-line RMS)
        /// Type: voltage_t
        /// Range: [1000, 15000] V typical
        /// Source: Grid voltage source model or external disturbance
        /// Usage: Defines grid voltage amplitude for rectifier input
        using grid_voltage_magnitude = void;  // voltage_t

        /// Grid voltage phase angle
        /// Type: angle_t
        /// Range: [0.0, 2π] radians
        /// Source: Grid voltage source model
        /// Usage: Defines grid voltage phase reference
        using grid_voltage_angle = void;  // angle_t

        /// Grid frequency
        /// Type: frequency_t (Hz)
        /// Range: [45.0, 65.0] Hz typical
        /// Source: Grid frequency variation model
        /// Usage: Defines fundamental frequency for all grid-side calculations
        using grid_frequency = void;  // frequency_t

        /// External load torque applied to machine shaft
        /// Type: torque_t
        /// Range: [-10000, 10000] Nm typical (depends on application)
        /// Source: Load torque profile or external mechanical system
        /// Usage: Mechanical load on the drive system
        using external_load_torque = void;  // torque_t

        /// Integration time step
        /// Type: time_t (seconds)
        /// Range: [10e-6, 200e-6] s (10-200 μs typical)
        /// Source: Real-time control system time base
        /// Usage: Numerical integration step size
        using delta_time = void;  // time_t
    };

    /**
     * @brief OUTPUT INTERFACE
     *
     * Output signals provided by the plant model component.
     * These represent the complete system state and measurements.
     */
    struct output_interface {
        /// Grid-side line currents (three-phase)
        /// Type: std::array<current_t, 3>
        /// Range: [-5000, 5000] A typical
        /// Method: std::array<current_t, 3> get_grid_side_currents() const;
        using grid_side_currents = void;  // std::array<current_t, 3>

        /// Grid-side line voltages (three-phase, line-to-line)
        /// Type: std::array<voltage_t, 3>
        /// Range: [0, 15000] V typical
        /// Method: std::array<voltage_t, 3> get_grid_side_voltages() const;
        using grid_side_voltages = void;  // std::array<voltage_t, 3>

        /// DC link voltage
        /// Type: voltage_t
        /// Range: [0, 15000] V typical
        /// Method: voltage_t get_dc_link_voltage() const;
        using dc_link_voltage = void;  // voltage_t

        /// DC link current
        /// Type: current_t
        /// Range: [-5000, 5000] A typical
        /// Method: current_t get_dc_link_current() const;
        using dc_link_current = void;  // current_t

        /// Machine terminal currents (three-phase)
        /// Type: std::array<current_t, 3>
        /// Range: [-5000, 5000] A typical
        /// Method: std::array<current_t, 3> get_machine_currents() const;
        using machine_currents = void;  // std::array<current_t, 3>

        /// Machine terminal voltages (three-phase, line-to-line)
        /// Type: std::array<voltage_t, 3>
        /// Range: [0, 10000] V typical
        /// Method: std::array<voltage_t, 3> get_machine_voltages() const;
        using machine_voltages = void;  // std::array<voltage_t, 3>

        /// Machine rotor speed (mechanical)
        /// Type: speed_t (rad/s)
        /// Range: [-1000, 1000] rad/s typical
        /// Method: speed_t get_rotor_speed() const;
        using rotor_speed = void;  // speed_t

        /// Machine rotor position (mechanical)
        /// Type: angle_t (radians)
        /// Range: [0.0, 2π] radians
        /// Method: angle_t get_rotor_position() const;
        using rotor_position = void;  // angle_t

        /// Machine electromagnetic torque
        /// Type: torque_t
        /// Range: [-10000, 10000] Nm typical
        /// Method: torque_t get_electromagnetic_torque() const;
        using electromagnetic_torque = void;  // torque_t

        /// Machine field current
        /// Type: current_t
        /// Range: [0, 1000] A typical
        /// Method: current_t get_field_current() const;
        using field_current = void;  // current_t

        /// Machine d-axis current (rotor reference frame)
        /// Type: current_t
        /// Range: [-5000, 5000] A typical
        /// Method: current_t get_machine_current_d() const;
        using machine_current_d = void;  // current_t

        /// Machine q-axis current (rotor reference frame)
        /// Type: current_t
        /// Range: [-5000, 5000] A typical
        /// Method: current_t get_machine_current_q() const;
        using machine_current_q = void;  // current_t

        /// Machine d-axis voltage (rotor reference frame)
        /// Type: voltage_t
        /// Range: [-5000, 5000] V typical
        /// Method: voltage_t get_machine_voltage_d() const;
        using machine_voltage_d = void;  // voltage_t

        /// Machine q-axis voltage (rotor reference frame)
        /// Type: voltage_t
        /// Range: [-5000, 5000] V typical
        /// Method: voltage_t get_machine_voltage_q() const;
        using machine_voltage_q = void;  // voltage_t

        /// Grid-side commutation status
        /// Type: bool
        /// Method: bool is_grid_side_commutating() const;
        using is_grid_side_commutating = void;  // bool

        /// Machine-side commutation status
        /// Type: bool
        /// Method: bool is_machine_side_commutating() const;
        using is_machine_side_commutating = void;  // bool

        /// System power (instantaneous)
        /// Type: power_t
        /// Range: [-50e6, 50e6] W typical (50 MW)
        /// Method: power_t get_instantaneous_power() const;
        using instantaneous_power = void;  // power_t

        /// Grid power factor
        /// Type: real_t
        /// Range: [-1.0, 1.0]
        /// Method: real_t get_grid_power_factor() const;
        using grid_power_factor = void;  // real_t
    };

    /**
     * @brief PARAMETER INTERFACE
     *
     * Configuration parameters for the plant model component.
     */
    struct parameter_interface {
        // Grid-side parameters
        /// Grid impedance (source reactance)
        /// Type: impedance_t
        /// Default: 0.1 pu
        /// Range: [0.01, 0.5] pu
        using grid_source_impedance = void;  // impedance_t, default 0.1

        /// Grid commutation reactance
        /// Type: reactance_t
        /// Default: 0.15 pu
        /// Range: [0.05, 0.3] pu
        using grid_commutation_reactance = void;  // reactance_t, default 0.15

        // DC link parameters
        /// DC link inductance
        /// Type: inductance_t
        /// Default: 0.5e-3 H
        /// Range: [0.1e-3, 5.0e-3] H
        using dc_link_inductance = void;  // inductance_t, default 0.5e-3

        /// DC link resistance
        /// Type: resistance_t
        /// Default: 0.01 Ω
        /// Range: [0.001, 0.1] Ω
        using dc_link_resistance = void;  // resistance_t, default 0.01

        // Machine electrical parameters
        /// Machine stator resistance (per phase)
        /// Type: resistance_t
        /// Default: 0.02 pu
        /// Range: [0.005, 0.05] pu
        using machine_stator_resistance = void;  // resistance_t, default 0.02

        /// Machine d-axis synchronous reactance
        /// Type: reactance_t
        /// Default: 1.2 pu
        /// Range: [0.8, 2.0] pu
        using machine_xd = void;  // reactance_t, default 1.2

        /// Machine q-axis synchronous reactance
        /// Type: reactance_t
        /// Default: 1.15 pu
        /// Range: [0.8, 2.0] pu
        using machine_xq = void;  // reactance_t, default 1.15

        /// Machine d-axis transient reactance
        /// Type: reactance_t
        /// Default: 0.3 pu
        /// Range: [0.15, 0.5] pu
        using machine_xd_prime = void;  // reactance_t, default 0.3

        /// Machine q-axis transient reactance
        /// Type: reactance_t
        /// Default: 0.4 pu
        /// Range: [0.2, 0.6] pu
        using machine_xq_prime = void;  // reactance_t, default 0.4

        /// Machine d-axis subtransient reactance
        /// Type: reactance_t
        /// Default: 0.2 pu
        /// Range: [0.1, 0.3] pu
        using machine_xd_double_prime = void;  // reactance_t, default 0.2

        /// Machine q-axis subtransient reactance
        /// Type: reactance_t
        /// Default: 0.2 pu
        /// Range: [0.1, 0.3] pu
        using machine_xq_double_prime = void;  // reactance_t, default 0.2

        /// Machine d-axis transient time constant
        /// Type: time_t (seconds)
        /// Default: 1.0 s
        /// Range: [0.5, 5.0] s
        using machine_td0_prime = void;  // time_t, default 1.0

        /// Machine q-axis transient time constant
        /// Type: time_t (seconds)
        /// Default: 0.5 s
        /// Range: [0.2, 2.0] s
        using machine_tq0_prime = void;  // time_t, default 0.5

        /// Machine d-axis subtransient time constant
        /// Type: time_t (seconds)
        /// Default: 0.03 s
        /// Range: [0.01, 0.1] s
        using machine_td0_double_prime = void;  // time_t, default 0.03

        /// Machine q-axis subtransient time constant
        /// Type: time_t (seconds)
        /// Default: 0.05 s
        /// Range: [0.02, 0.1] s
        using machine_tq0_double_prime = void;  // time_t, default 0.05

        // Machine mechanical parameters
        /// Machine moment of inertia
        /// Type: inertia_t (kg⋅m²)
        /// Default: 100.0 kg⋅m²
        /// Range: [10.0, 1000.0] kg⋅m²
        using machine_inertia = void;  // inertia_t, default 100.0

        /// Machine mechanical damping
        /// Type: damping_t (Nm⋅s/rad)
        /// Default: 0.1 Nm⋅s/rad
        /// Range: [0.01, 1.0] Nm⋅s/rad
        using machine_damping = void;  // damping_t, default 0.1

        /// Machine pole pairs
        /// Type: int
        /// Default: 2
        /// Range: [1, 10]
        using machine_pole_pairs = void;  // int, default 2

        // Converter parameters
        /// Machine-side commutation reactance
        /// Type: reactance_t
        /// Default: 0.12 pu
        /// Range: [0.05, 0.25] pu
        using machine_commutation_reactance = void;  // reactance_t, default 0.12

        /// Thyristor voltage drop
        /// Type: voltage_t
        /// Default: 2.0 V
        /// Range: [1.0, 5.0] V
        using thyristor_voltage_drop = void;  // voltage_t, default 2.0

        /// Thyristor turn-on time
        /// Type: time_t (seconds)
        /// Default: 5e-6 s
        /// Range: [1e-6, 20e-6] s
        using thyristor_turn_on_time = void;  // time_t, default 5e-6

        /// Thyristor turn-off time
        /// Type: time_t (seconds)
        /// Default: 50e-6 s
        /// Range: [20e-6, 200e-6] s
        using thyristor_turn_off_time = void;  // time_t, default 50e-6

        // Simulation parameters
        /// Integration method (explicit_euler, rk4, semi_implicit)
        /// Type: integration_method_t
        /// Default: semi_implicit
        using integration_method = void;  // integration_method_t, default semi_implicit

        /// Maximum integration step size
        /// Type: time_t (seconds)
        /// Default: 100e-6 s
        /// Range: [10e-6, 1000e-6] s
        using max_integration_step = void;  // time_t, default 100e-6

        /// Numerical damping factor for stability
        /// Type: real_t
        /// Default: 0.001
        /// Range: [0.0, 0.01]
        using numerical_damping = void;  // real_t, default 0.001
    };

    /**
     * @brief COMPONENT INTERFACE CONTRACT
     *
     * The implementing sfc_6p_6p_plant_model class must provide these methods:
     *
     * Core Update Method:
     *   void update(const std::array<bool, 6>& grid_side_firing_pulses,
     *               const std::array<bool, 6>& machine_side_firing_pulses,
     *               voltage_t grid_voltage_magnitude,
     *               angle_t grid_voltage_angle,
     *               frequency_t grid_frequency,
     *               torque_t external_load_torque,
     *               time_t delta_time);
     *
     * Configuration Methods (organized by subsystem):
     *   void configure_grid_parameters(impedance_t source_impedance, reactance_t commutation_reactance);
     *   void configure_dc_link(inductance_t inductance, resistance_t resistance);
     *   void configure_machine_electrical(resistance_t rs, reactance_t xd, reactance_t xq, ...);
     *   void configure_machine_mechanical(inertia_t inertia, damping_t damping, int pole_pairs);
     *   void configure_converter_parameters(reactance_t commutation_reactance, voltage_t thyristor_drop);
     *   void configure_simulation(integration_method_t method, time_t max_step, real_t damping);
     *
     * Output Getter Methods:
     *   std::array<current_t, 3> get_grid_side_currents() const;
     *   std::array<voltage_t, 3> get_grid_side_voltages() const;
     *   voltage_t get_dc_link_voltage() const;
     *   current_t get_dc_link_current() const;
     *   std::array<current_t, 3> get_machine_currents() const;
     *   std::array<voltage_t, 3> get_machine_voltages() const;
     *   speed_t get_rotor_speed() const;
     *   angle_t get_rotor_position() const;
     *   torque_t get_electromagnetic_torque() const;
     *   [... additional getters for all outputs ...]
     *
     * State Management Methods:
     *   void reset();
     *   void initialize_steady_state(speed_t initial_speed, current_t initial_current);
     *   void set_initial_conditions(const state_vector_t& initial_state);
     *   state_vector_t get_state() const;
     *
     * Diagnostic Methods:
     *   bool is_numerically_stable() const;
     *   real_t get_energy_balance_error() const;
     *   real_t get_simulation_performance_index() const;
     *
     * Architectural Compliance:
     * - Memory: No dynamic allocation during runtime (real-time safety)
     * - Timing: Deterministic execution time < 50 μs per update
     * - Thread Safety: Not required (single-threaded real-time context)
     * - Numerical Stability: Guaranteed stability across full operating range
     * - Energy Conservation: Maintains energy balance within 0.1% error
     */
};

}  // namespace yeet::application::sfc::integration
