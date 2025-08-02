#pragma once

/**
 * @file sfc_current_controller_datamodel.hpp
 * @brief Datamodel specification for SFC Current Controller component
 *
 * This file defines the interface contract for the SFC Current Controller component
 * following the yeet framework datamodel pattern. The current controller implements
 * the inner current control loop for the LCI drive system.
 */

namespace yeet::application::sfc::integration {

/**
 * @brief Datamodel for the SFC current controller component.
 *
 * This datamodel defines the complete interface contract for the current controller
 * which implements the inner current control loop for an LCI drive. The controller
 * regulates the DC link current by adjusting the grid-side rectifier firing angle.
 *
 * Component Purpose:
 * The current controller is the innermost and fastest control loop in the LCI system.
 * It provides:
 * - Fast DC link current regulation (typically 1-2 kHz bandwidth)
 * - Grid-side rectifier firing angle calculation
 * - Current limiting and protection functions
 * - Coordination with machine-side inverter control
 *
 * Control Algorithm:
 * - PI controller with feedforward compensation
 * - Anti-windup protection for current limiting
 * - Automatic per-unit reflection between grid and machine bases
 * - Ideal DC voltage estimation for feedforward control
 *
 * Real-time Performance Requirements:
 * - Update rate: Typically 10 kHz (100 μs)
 * - Control bandwidth: 1-2 kHz for fast current response
 * - Settling time: < 5 ms for current step response
 * - Steady-state error: < 1% of rated current
 */
class sfc_current_controller_datamodel {
public:
    /**
     * @brief INPUT INTERFACE
     *
     * Input signals required by the current controller component.
     * Expected update() method signature:
     *
     * void update(current_pu_t current_reference_pu,
     *             current_pu_t dc_link_current_measured_pu,
     *             voltage_pu_t ideal_dc_voltage_pu,
     *             voltage_pu_t inverter_back_emf_machine_pu,
     *             voltage_t machine_side_base_voltage,
     *             voltage_t grid_side_base_voltage);
     */
    struct input_interface {
        /// Current reference signal in per-unit (DC link base)
        /// Type: current_pu_t
        /// Range: [-1.2, 1.2] pu (±120% of rated current for transients)
        /// Source: Typically from outer speed controller or torque reference
        /// Usage: Setpoint for the current control loop
        using current_reference_pu = void;  // current_pu_t

        /// Measured DC link current in per-unit (DC link base)
        /// Type: current_pu_t
        /// Range: [-1.5, 1.5] pu (includes measurement range)
        /// Source: DC link current sensor (filtered)
        /// Usage: Feedback signal for current control loop
        using dc_link_current_measured_pu = void;  // current_pu_t

        /// Ideal no-load DC voltage from grid-side rectifier in per-unit (grid-side base)
        /// Type: voltage_pu_t
        /// Range: [0.8, 1.2] pu based on grid voltage
        /// Source: Grid-side observer voltage magnitude estimate
        /// Usage: Feedforward control for improved dynamic response
        using ideal_dc_voltage_pu = void;  // voltage_pu_t

        /// Estimated inverter back-EMF in per-unit (machine-side base)
        /// Type: voltage_pu_t
        /// Range: [0.0, 1.2] pu based on machine speed
        /// Source: Machine-side observer voltage magnitude estimate
        /// Usage: Feedforward control accounting for machine back-EMF
        using inverter_back_emf_machine_pu = void;  // voltage_pu_t

        /// Machine-side voltage base for per-unit reflection
        /// Type: voltage_t
        /// Range: [1000, 5000] V typical for industrial applications
        /// Source: Per-unit system configuration
        /// Usage: Enables automatic conversion between machine and grid per-unit bases
        using machine_side_base_voltage = void;  // voltage_t

        /// Grid-side voltage base for per-unit reflection
        /// Type: voltage_t
        /// Range: [1000, 15000] V typical for grid connections
        /// Source: Per-unit system configuration
        /// Usage: Enables automatic conversion between machine and grid per-unit bases
        using grid_side_base_voltage = void;  // voltage_t
    };

    /**
     * @brief OUTPUT INTERFACE
     *
     * Output signals provided by the current controller component.
     * These match the expected getter method signatures:
     */
    struct output_interface {
        /// Grid-side rectifier firing angle reference
        /// Type: angle_t
        /// Range: [0.0, π/2] radians (0° to 90° electrical)
        /// Usage: Primary control output sent to grid-side pulse generator
        /// Method: angle_t get_firing_angle_reference() const;
        using firing_angle_reference = void;  // angle_t

        /// Current control error (reference - measured)
        /// Type: current_pu_t
        /// Range: [-1.0, 1.0] pu
        /// Usage: Diagnostic signal for control performance monitoring
        /// Method: current_pu_t get_current_error() const;
        using current_error = void;  // current_pu_t

        /// PI controller output before limiting
        /// Type: angle_t
        /// Range: [0.0, π] radians (before saturation)
        /// Usage: Diagnostic signal for anti-windup monitoring
        /// Method: angle_t get_pi_output() const;
        using pi_output = void;  // angle_t

        /// Controller saturation status
        /// Type: bool
        /// Usage: Indicates when firing angle has reached limits
        /// Method: bool is_saturated() const;
        using is_saturated = void;  // bool

        /// Control loop active status
        /// Type: bool
        /// Usage: Indicates whether current control is active and stable
        /// Method: bool is_active() const;
        using is_active = void;  // bool
    };

    /**
     * @brief PARAMETER INTERFACE
     *
     * Configuration parameters for the current controller component.
     */
    struct parameter_interface {
        /// PI controller proportional gain
        /// Type: real_t
        /// Default: 5.0
        /// Range: [1.0, 20.0]
        /// Method: void configure_pi_gains(real_t kp, real_t ki);
        /// Usage: Determines speed of response and stability margins
        using pi_proportional_gain = void;  // real_t, default 5.0

        /// PI controller integral gain
        /// Type: real_t
        /// Default: 500.0
        /// Range: [100.0, 2000.0]
        /// Method: void configure_pi_gains(real_t kp, real_t ki);
        /// Usage: Eliminates steady-state error and disturbance rejection
        using pi_integral_gain = void;  // real_t, default 500.0

        /// Minimum firing angle limit
        /// Type: angle_t
        /// Default: 0.0 radians (0°)
        /// Range: [0.0, π/6] radians (0° to 30°)
        /// Method: void configure_firing_angle_limits(angle_t min, angle_t max);
        /// Usage: Prevents excessive leading firing angles
        using min_firing_angle = void;  // angle_t, default 0.0

        /// Maximum firing angle limit
        /// Type: angle_t
        /// Default: π/2 radians (90°)
        /// Range: [π/4, π/2] radians (45° to 90°)
        /// Method: void configure_firing_angle_limits(angle_t min, angle_t max);
        /// Usage: Prevents excessive lagging firing angles and maintains rectifier operation
        using max_firing_angle = void;  // angle_t, default π/2

        /// Current reference rate limit (slew rate)
        /// Type: current_pu_t per second
        /// Default: 10.0 pu/s
        /// Range: [1.0, 50.0] pu/s
        /// Method: void configure_reference_limits(current_pu_t rate_limit, current_pu_t magnitude_limit);
        /// Usage: Prevents excessive current reference changes that could destabilize the system
        using current_reference_rate_limit = void;  // current_pu_t/s, default 10.0

        /// Current reference magnitude limit
        /// Type: current_pu_t
        /// Default: 1.1 pu (110% of rated)
        /// Range: [1.0, 1.5] pu
        /// Method: void configure_reference_limits(current_pu_t rate_limit, current_pu_t magnitude_limit);
        /// Usage: Protects against overcurrent conditions
        using current_reference_magnitude_limit = void;  // current_pu_t, default 1.1

        /// Feedforward gain for ideal DC voltage compensation
        /// Type: real_t
        /// Default: 0.8
        /// Range: [0.0, 1.0]
        /// Method: void configure_feedforward_gains(real_t dc_voltage_gain, real_t back_emf_gain);
        /// Usage: Improves dynamic response to grid voltage variations
        using feedforward_dc_voltage_gain = void;  // real_t, default 0.8

        /// Feedforward gain for back-EMF compensation
        /// Type: real_t
        /// Default: 0.5
        /// Range: [0.0, 1.0]
        /// Method: void configure_feedforward_gains(real_t dc_voltage_gain, real_t back_emf_gain);
        /// Usage: Compensates for machine back-EMF effects on current control
        using feedforward_back_emf_gain = void;  // real_t, default 0.5

        /// Anti-windup gain for integral term protection
        /// Type: real_t
        /// Default: 0.1
        /// Range: [0.05, 0.5]
        /// Method: void configure_anti_windup(real_t anti_windup_gain);
        /// Usage: Prevents integrator windup during saturation
        using anti_windup_gain = void;  // real_t, default 0.1
    };

    /**
     * @brief COMPONENT INTERFACE CONTRACT
     *
     * The implementing sfc_current_controller class must provide these methods:
     *
     * Core Update Method:
     *   void update(current_pu_t current_reference_pu,
     *               current_pu_t dc_link_current_measured_pu,
     *               voltage_pu_t ideal_dc_voltage_pu,
     *               voltage_pu_t inverter_back_emf_machine_pu,
     *               voltage_t machine_side_base_voltage,
     *               voltage_t grid_side_base_voltage);
     *
     * Alternative Update Method (automatic per-unit reflection):
     *   void update(current_pu_t current_reference_pu,
     *               current_pu_t dc_link_current_measured_pu,
     *               voltage_pu_t ideal_dc_voltage_pu,
     *               voltage_pu_t inverter_back_emf_machine_pu);
     *
     * Configuration Methods:
     *   void configure_pi_gains(real_t kp, real_t ki);
     *   void configure_firing_angle_limits(angle_t min, angle_t max);
     *   void configure_reference_limits(current_pu_t rate_limit, current_pu_t magnitude_limit);
     *   void configure_feedforward_gains(real_t dc_voltage_gain, real_t back_emf_gain);
     *   void configure_anti_windup(real_t anti_windup_gain);
     *
     * Output Getter Methods:
     *   angle_t get_firing_angle_reference() const;
     *   current_pu_t get_current_error() const;
     *   angle_t get_pi_output() const;
     *   bool is_saturated() const;
     *   bool is_active() const;
     *
     * State Management Methods:
     *   void reset();
     *   void enable();
     *   void disable();
     *
     * Architectural Compliance:
     * - Memory: No dynamic allocation during runtime (real-time safety)
     * - Timing: Deterministic execution time < 5 μs per update
     * - Thread Safety: Not required (single-threaded real-time context)
     * - Error Handling: Safe operation during sensor failures or saturation
     */
};

}  // namespace yeet::application::sfc::integration
