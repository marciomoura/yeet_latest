#pragma once

/**
 * @file sfc_machine_side_modulator_datamodel.hpp
 * @brief Datamodel specification for SFC Machine-Side Modulator component
 *
 * This file defines the interface contract for the SFC Machine-Side Modulator component
 * following the yeet framework datamodel pattern. The machine-side modulator implements
 * the thyristor firing control for the machine-side inverter.
 */

namespace yeet::application::sfc::integration {

/**
 * @brief Datamodel for the SFC machine-side modulator component.
 *
 * This datamodel defines the complete interface contract for the machine-side modulator
 * which controls the thyristor firing for the machine-side inverter in an LCI drive.
 * The modulator provides precise timing control for current commutation.
 *
 * Component Purpose:
 * The machine-side modulator is responsible for:
 * - Thyristor firing timing control for the machine-side inverter
 * - Mode-dependent firing angle selection (pulse mode, normal mode)
 * - Coordination with rotor position sensing for optimal commutation
 * - Protection against commutation failures and overcurrent
 *
 * Operating Modes:
 * - Pulse Mode: Fixed firing angle (typically 90°) for initial motor magnetization
 * - Normal Mode: Variable firing angle based on lookup table and current level
 * - Bypass Mode: Alternative firing pattern for special operating conditions
 *
 * Control Algorithm:
 * - Lookup table-based firing angle determination
 * - Rotor position-based firing pulse generation
 * - Mode-dependent firing angle offset calculation
 * - Current-dependent advance angle compensation
 *
 * Real-time Performance Requirements:
 * - Update rate: 10 kHz (100 μs) for precise timing control
 * - Firing pulse accuracy: ±1° electrical
 * - Response time: < 500 μs for mode changes
 * - Jitter: < 10 μs for firing pulse timing
 */
class sfc_machine_side_modulator_datamodel {
public:
    /**
     * @brief INPUT INTERFACE
     *
     * Input signals required by the machine-side modulator component.
     * Expected update() method signature:
     *
     * void update(bool enable_command,
     *             operating_mode_t operating_mode,
     *             angle_t rotor_position_electrical,
     *             angle_t lookup_table_firing_angle,
     *             current_pu_t dc_link_current_pu,
     *             bool commutation_detected);
     */
    struct input_interface {
        /// Enable command for the modulator
        /// Type: bool
        /// Source: High-level control system enable logic
        /// Usage: Master enable/disable for firing pulse generation
        using enable_command = void;  // bool

        /// Operating mode selection
        /// Type: operating_mode_t (enum: pulse_mode, normal_mode, bypass_mode)
        /// Source: Control system mode logic
        /// Usage: Determines firing angle calculation method
        using operating_mode = void;  // operating_mode_t

        /// Rotor electrical position
        /// Type: angle_t
        /// Range: [0.0, 2π] radians (0° to 360° electrical)
        /// Source: Rotor position sensor or observer
        /// Usage: Determines thyristor firing timing relative to rotor position
        using rotor_position_electrical = void;  // angle_t

        /// Firing angle from lookup table
        /// Type: angle_t
        /// Range: [0.0, π] radians (0° to 180° electrical)
        /// Source: Firing angle lookup table based on speed and current
        /// Usage: Base firing angle for normal operating mode
        using lookup_table_firing_angle = void;  // angle_t

        /// DC link current for advance angle calculation
        /// Type: current_pu_t
        /// Range: [-1.5, 1.5] pu
        /// Source: DC link current measurement
        /// Usage: Determines current-dependent advance angle compensation
        using dc_link_current_pu = void;  // current_pu_t

        /// Commutation detection signal
        /// Type: bool
        /// Source: Commutation detection logic
        /// Usage: Verifies successful current commutation for protection
        using commutation_detected = void;  // bool
    };

    /**
     * @brief OUTPUT INTERFACE
     *
     * Output signals provided by the machine-side modulator component.
     * These match the expected getter method signatures:
     */
    struct output_interface {
        /// Final calculated firing angle
        /// Type: angle_t
        /// Range: [0.0, π] radians (0° to 180° electrical)
        /// Usage: Primary control output for thyristor gate drive timing
        /// Method: angle_t get_firing_angle() const;
        using firing_angle = void;  // angle_t

        /// Thyristor firing pulses for each phase (6-pulse pattern)
        /// Type: std::array<bool, 6>
        /// Usage: Gate drive signals for thyristor switches T1-T6
        /// Method: std::array<bool, 6> get_firing_pulses() const;
        using firing_pulses = void;  // std::array<bool, 6>

        /// Active operating mode
        /// Type: operating_mode_t
        /// Usage: Confirmation of current operating mode
        /// Method: operating_mode_t get_operating_mode() const;
        using active_operating_mode = void;  // operating_mode_t

        /// Pulse mode firing angle (diagnostic)
        /// Type: angle_t
        /// Range: [π/4, π/2] radians (45° to 90° electrical)
        /// Usage: Shows configured pulse mode firing angle
        /// Method: angle_t get_pulse_mode_firing_angle() const;
        using pulse_mode_firing_angle = void;  // angle_t

        /// Advance angle compensation
        /// Type: angle_t
        /// Range: [0.0, π/6] radians (0° to 30° electrical)
        /// Usage: Current-dependent firing angle advancement
        /// Method: angle_t get_advance_angle() const;
        using advance_angle = void;  // angle_t

        /// Modulator enable status
        /// Type: bool
        /// Usage: Indicates whether firing pulses are enabled
        /// Method: bool is_enabled() const;
        using is_enabled = void;  // bool

        /// Commutation failure detection
        /// Type: bool
        /// Usage: Indicates detected commutation failure condition
        /// Method: bool has_commutation_failure() const;
        using has_commutation_failure = void;  // bool

        /// Next firing position (predictive)
        /// Type: angle_t
        /// Range: [0.0, 2π] radians
        /// Usage: Rotor position where next firing pulse will occur
        /// Method: angle_t get_next_firing_position() const;
        using next_firing_position = void;  // angle_t
    };

    /**
     * @brief PARAMETER INTERFACE
     *
     * Configuration parameters for the machine-side modulator component.
     */
    struct parameter_interface {
        /// Pulse mode firing angle
        /// Type: angle_t
        /// Default: π/2 radians (90° electrical)
        /// Range: [π/4, π/2] radians (45° to 90°)
        /// Method: void configure_pulse_mode_firing_angle_rad(angle_t angle);
        /// Usage: Fixed firing angle for pulse mode operation
        using pulse_mode_firing_angle_rad = void;  // angle_t, default π/2

        /// Minimum firing angle limit
        /// Type: angle_t
        /// Default: 0.0 radians (0°)
        /// Range: [0.0, π/6] radians (0° to 30°)
        /// Method: void configure_firing_angle_limits(angle_t min, angle_t max);
        /// Usage: Prevents excessive leading firing angles
        using min_firing_angle = void;  // angle_t, default 0.0

        /// Maximum firing angle limit
        /// Type: angle_t
        /// Default: π radians (180°)
        /// Range: [2π/3, π] radians (120° to 180°)
        /// Method: void configure_firing_angle_limits(angle_t min, angle_t max);
        /// Usage: Prevents excessive lagging firing angles
        using max_firing_angle = void;  // angle_t, default π

        /// Advance angle gain (current dependency)
        /// Type: real_t
        /// Default: 0.1 radians per pu current
        /// Range: [0.0, 0.5]
        /// Method: void configure_advance_angle(real_t gain, angle_t max_advance);
        /// Usage: Determines how much firing angle advances with current
        using advance_angle_gain = void;  // real_t, default 0.1

        /// Maximum advance angle
        /// Type: angle_t
        /// Default: π/6 radians (30°)
        /// Range: [0.0, π/3] radians (0° to 60°)
        /// Method: void configure_advance_angle(real_t gain, angle_t max_advance);
        /// Usage: Limits maximum advance angle compensation
        using max_advance_angle = void;  // angle_t, default π/6

        /// Firing pulse width
        /// Type: angle_t
        /// Default: π/18 radians (10° electrical)
        /// Range: [π/36, π/9] radians (5° to 20°)
        /// Method: void configure_pulse_timing(angle_t width, angle_t overlap);
        /// Usage: Duration of gate drive pulse for thyristor turn-on
        using firing_pulse_width = void;  // angle_t, default π/18

        /// Overlap angle between commutating thyristors
        /// Type: angle_t
        /// Default: π/18 radians (10° electrical)
        /// Range: [π/36, π/9] radians (5° to 20°)
        /// Method: void configure_pulse_timing(angle_t width, angle_t overlap);
        /// Usage: Ensures reliable current commutation between thyristors
        using overlap_angle = void;  // angle_t, default π/18

        /// Commutation failure timeout
        /// Type: time_t (seconds)
        /// Default: 0.001 s (1 ms)
        /// Range: [0.0005, 0.005] s
        /// Method: void configure_protection(time_t timeout, real_t current_threshold);
        /// Usage: Time limit for detecting commutation failure
        using commutation_failure_timeout = void;  // time_t, default 0.001

        /// Minimum current for firing
        /// Type: current_pu_t
        /// Default: 0.05 pu (5% of rated)
        /// Range: [0.01, 0.1] pu
        /// Method: void configure_protection(time_t timeout, real_t current_threshold);
        /// Usage: Minimum current required to enable firing pulses
        using minimum_firing_current = void;  // current_pu_t, default 0.05

        /// Bypass mode firing angle offset
        /// Type: angle_t
        /// Default: 0.0 radians
        /// Range: [-π/6, π/6] radians (-30° to +30°)
        /// Method: void configure_bypass_mode(angle_t offset);
        /// Usage: Additional firing angle offset for bypass mode operation
        using bypass_mode_angle_offset = void;  // angle_t, default 0.0
    };

    /**
     * @brief COMPONENT INTERFACE CONTRACT
     *
     * The implementing sfc_machine_side_modulator class must provide these methods:
     *
     * Core Update Method:
     *   void update(bool enable_command,
     *               operating_mode_t operating_mode,
     *               angle_t rotor_position_electrical,
     *               angle_t lookup_table_firing_angle,
     *               current_pu_t dc_link_current_pu,
     *               bool commutation_detected);
     *
     * Configuration Methods:
     *   void configure_pulse_mode_firing_angle_rad(angle_t angle);
     *   void configure_firing_angle_limits(angle_t min, angle_t max);
     *   void configure_advance_angle(real_t gain, angle_t max_advance);
     *   void configure_pulse_timing(angle_t width, angle_t overlap);
     *   void configure_protection(time_t timeout, real_t current_threshold);
     *   void configure_bypass_mode(angle_t offset);
     *
     * Output Getter Methods:
     *   angle_t get_firing_angle() const;
     *   std::array<bool, 6> get_firing_pulses() const;
     *   operating_mode_t get_operating_mode() const;
     *   angle_t get_pulse_mode_firing_angle() const;
     *   angle_t get_advance_angle() const;
     *   bool is_enabled() const;
     *   bool has_commutation_failure() const;
     *   angle_t get_next_firing_position() const;
     *
     * State Management Methods:
     *   void reset();
     *   void enable();
     *   void disable();
     *   void clear_faults();
     *
     * Mode Control Methods:
     *   void set_pulse_mode();
     *   void set_normal_mode();
     *   void set_bypass_mode();
     *   bool is_pulse_mode() const;
     *   bool is_normal_mode() const;
     *   bool is_bypass_mode() const;
     *
     * Architectural Compliance:
     * - Memory: No dynamic allocation during runtime (real-time safety)
     * - Timing: Deterministic execution time < 5 μs per update
     * - Thread Safety: Not required (single-threaded real-time context)
     * - Error Handling: Safe operation during sensor failures or commutation faults
     * - Protection: Commutation failure detection and current limiting
     */
};

}  // namespace yeet::application::sfc::integration
