#pragma once

/**
 * @file sfc_speed_controller_datamodel.hpp
 * @brief Datamodel specification for SFC Speed Controller component
 *
 * This file defines the interface contract for the SFC Speed Controller component
 * following the yeet framework datamodel pattern. The speed controller implements
 * the outer speed control loop for the LCI drive system.
 */

namespace yeet::application::sfc::integration {

/**
 * @brief Datamodel for the SFC speed controller component.
 *
 * This datamodel defines the complete interface contract for the speed controller
 * which implements the outer speed control loop for an LCI drive. The controller
 * regulates machine speed by generating current references for the inner current loop.
 *
 * Component Purpose:
 * The speed controller is the outer control loop in the LCI system cascade.
 * It provides:
 * - Machine speed regulation with adjustable bandwidth (typically 5-50 Hz)
 * - Torque and current reference generation
 * - Acceleration/deceleration control with configurable ramp rates
 * - Integration with torque-mode control for gas turbine starting applications
 *
 * Control Algorithm:
 * - PI controller with configurable gains and limits
 * - Speed reference rate limiting for smooth operation
 * - Anti-windup protection during current limiting
 * - Mode switching between speed control and torque control
 *
 * Real-time Performance Requirements:
 * - Update rate: Typically 1-10 kHz (100 μs - 1 ms)
 * - Control bandwidth: 5-50 Hz for speed response
 * - Settling time: < 100 ms for speed step response (depending on inertia)
 * - Steady-state error: < 0.1% of rated speed
 */
class sfc_speed_controller_datamodel {
public:
    /**
     * @brief INPUT INTERFACE
     *
     * Input signals required by the speed controller component.
     * Expected update() method signature:
     *
     * void update(speed_pu_t speed_reference_pu,
     *             speed_pu_t measured_speed_pu,
     *             current_pu_t measured_current_pu,
     *             bool enable_speed_control,
     *             current_pu_t external_current_reference_pu = 0.0);
     */
    struct input_interface {
        /// Speed reference signal in per-unit (machine base)
        /// Type: speed_pu_t
        /// Range: [-1.2, 1.2] pu for motoring/generating operation
        /// Source: High-level control system or operator setpoint
        /// Usage: Setpoint for the speed control loop
        using speed_reference_pu = void;  // speed_pu_t

        /// Measured machine speed in per-unit (machine base)
        /// Type: speed_pu_t
        /// Range: [-1.5, 1.5] pu (includes measurement range)
        /// Source: Speed encoder, resolver, or sensorless observer
        /// Usage: Feedback signal for speed control loop
        using measured_speed_pu = void;  // speed_pu_t

        /// Measured DC link current for anti-windup logic
        /// Type: current_pu_t
        /// Range: [-1.5, 1.5] pu
        /// Source: DC link current measurement
        /// Usage: Feedback for current limiting and anti-windup protection
        using measured_current_pu = void;  // current_pu_t

        /// Enable signal for speed control mode
        /// Type: bool
        /// Source: Control system mode logic
        /// Usage: Switches between speed control and external current reference mode
        using enable_speed_control = void;  // bool

        /// External current reference for torque mode operation
        /// Type: current_pu_t
        /// Range: [-1.2, 1.2] pu
        /// Source: External torque controller or manual reference
        /// Usage: Direct current reference when speed control is disabled
        using external_current_reference_pu = void;  // current_pu_t, default 0.0
    };

    /**
     * @brief OUTPUT INTERFACE
     *
     * Output signals provided by the speed controller component.
     * These match the expected getter method signatures:
     */
    struct output_interface {
        /// Current reference output to inner current loop
        /// Type: current_pu_t
        /// Range: [-1.2, 1.2] pu (limited to protect current loop)
        /// Usage: Primary control output sent to current controller
        /// Method: current_pu_t get_current_reference() const;
        using current_reference = void;  // current_pu_t

        /// Speed control error (reference - measured)
        /// Type: speed_pu_t
        /// Range: [-1.0, 1.0] pu
        /// Usage: Diagnostic signal for control performance monitoring
        /// Method: speed_pu_t get_speed_error() const;
        using speed_error = void;  // speed_pu_t

        /// PI controller output before limiting
        /// Type: current_pu_t
        /// Range: [-2.0, 2.0] pu (before saturation)
        /// Usage: Diagnostic signal for anti-windup monitoring
        /// Method: current_pu_t get_pi_output() const;
        using pi_output = void;  // current_pu_t

        /// Controller saturation status
        /// Type: bool
        /// Usage: Indicates when current reference has reached limits
        /// Method: bool is_saturated() const;
        using is_saturated = void;  // bool

        /// Speed control active status
        /// Type: bool
        /// Usage: Indicates whether speed control mode is active and stable
        /// Method: bool is_speed_control_active() const;
        using is_speed_control_active = void;  // bool

        /// Speed reference after rate limiting
        /// Type: speed_pu_t
        /// Range: [-1.2, 1.2] pu
        /// Usage: Diagnostic signal showing actual reference applied
        /// Method: speed_pu_t get_speed_reference_limited() const;
        using speed_reference_limited = void;  // speed_pu_t

        /// Estimated torque output (proportional to current reference)
        /// Type: torque_pu_t
        /// Range: [-1.2, 1.2] pu
        /// Usage: Diagnostic signal for torque monitoring
        /// Method: torque_pu_t get_estimated_torque() const;
        using estimated_torque = void;  // torque_pu_t
    };

    /**
     * @brief PARAMETER INTERFACE
     *
     * Configuration parameters for the speed controller component.
     */
    struct parameter_interface {
        /// PI controller proportional gain
        /// Type: real_t
        /// Default: 10.0
        /// Range: [1.0, 100.0]
        /// Method: void configure_pi_gains(real_t kp, real_t ki);
        /// Usage: Determines speed of response and stability margins
        using pi_proportional_gain = void;  // real_t, default 10.0

        /// PI controller integral gain
        /// Type: real_t
        /// Default: 100.0
        /// Range: [10.0, 1000.0]
        /// Method: void configure_pi_gains(real_t kp, real_t ki);
        /// Usage: Eliminates steady-state error and load disturbance rejection
        using pi_integral_gain = void;  // real_t, default 100.0

        /// Speed reference rate limit (acceleration/deceleration)
        /// Type: speed_pu_t per second
        /// Default: 2.0 pu/s
        /// Range: [0.1, 10.0] pu/s
        /// Method: void configure_reference_limits(speed_pu_t rate_limit, speed_pu_t magnitude_limit);
        /// Usage: Controls acceleration/deceleration rate for smooth operation
        using speed_reference_rate_limit = void;  // speed_pu_t/s, default 2.0

        /// Speed reference magnitude limit
        /// Type: speed_pu_t
        /// Default: 1.1 pu (110% of rated speed)
        /// Range: [1.0, 1.2] pu
        /// Method: void configure_reference_limits(speed_pu_t rate_limit, speed_pu_t magnitude_limit);
        /// Usage: Protects against overspeed conditions
        using speed_reference_magnitude_limit = void;  // speed_pu_t, default 1.1

        /// Current reference output limit (positive)
        /// Type: current_pu_t
        /// Default: 1.1 pu (110% of rated current)
        /// Range: [1.0, 1.5] pu
        /// Method: void configure_current_limits(current_pu_t max_current, current_pu_t min_current);
        /// Usage: Limits maximum torque/current output
        using max_current_reference = void;  // current_pu_t, default 1.1

        /// Current reference output limit (negative)
        /// Type: current_pu_t
        /// Default: -1.1 pu (110% of rated current, generating)
        /// Range: [-1.5, -1.0] pu
        /// Method: void configure_current_limits(current_pu_t max_current, current_pu_t min_current);
        /// Usage: Limits maximum regenerative torque/current
        using min_current_reference = void;  // current_pu_t, default -1.1

        /// Anti-windup gain for integral term protection
        /// Type: real_t
        /// Default: 0.1
        /// Range: [0.05, 0.5]
        /// Method: void configure_anti_windup(real_t anti_windup_gain);
        /// Usage: Prevents integrator windup during current limiting
        using anti_windup_gain = void;  // real_t, default 0.1

        /// Deadband for speed error (reduces noise sensitivity)
        /// Type: speed_pu_t
        /// Default: 0.001 pu (0.1% of rated speed)
        /// Range: [0.0, 0.01] pu
        /// Method: void configure_deadband(speed_pu_t deadband);
        /// Usage: Eliminates control action for small speed errors
        using speed_error_deadband = void;  // speed_pu_t, default 0.001

        /// Low-pass filter time constant for speed reference
        /// Type: time_t (seconds)
        /// Default: 0.01 s (100 Hz cutoff)
        /// Range: [0.001, 0.1] s
        /// Method: void configure_reference_filter(time_t time_constant);
        /// Usage: Smooths speed reference transitions
        using speed_reference_filter_time_constant = void;  // time_t, default 0.01

        /// Torque-to-current conversion factor
        /// Type: real_t
        /// Default: 1.0 (assumes flux at rated)
        /// Range: [0.5, 1.5]
        /// Method: void configure_torque_constant(real_t torque_constant);
        /// Usage: Converts between torque and current units for diagnostics
        using torque_to_current_factor = void;  // real_t, default 1.0
    };

    /**
     * @brief COMPONENT INTERFACE CONTRACT
     *
     * The implementing sfc_speed_controller class must provide these methods:
     *
     * Core Update Method:
     *   void update(speed_pu_t speed_reference_pu,
     *               speed_pu_t measured_speed_pu,
     *               current_pu_t measured_current_pu,
     *               bool enable_speed_control,
     *               current_pu_t external_current_reference_pu = 0.0);
     *
     * Configuration Methods:
     *   void configure_pi_gains(real_t kp, real_t ki);
     *   void configure_reference_limits(speed_pu_t rate_limit, speed_pu_t magnitude_limit);
     *   void configure_current_limits(current_pu_t max_current, current_pu_t min_current);
     *   void configure_anti_windup(real_t anti_windup_gain);
     *   void configure_deadband(speed_pu_t deadband);
     *   void configure_reference_filter(time_t time_constant);
     *   void configure_torque_constant(real_t torque_constant);
     *
     * Output Getter Methods:
     *   current_pu_t get_current_reference() const;
     *   speed_pu_t get_speed_error() const;
     *   current_pu_t get_pi_output() const;
     *   bool is_saturated() const;
     *   bool is_speed_control_active() const;
     *   speed_pu_t get_speed_reference_limited() const;
     *   torque_pu_t get_estimated_torque() const;
     *
     * State Management Methods:
     *   void reset();
     *   void enable();
     *   void disable();
     *   void set_speed_control_mode(bool enable);
     *
     * Mode Switching Methods:
     *   void switch_to_speed_mode();
     *   void switch_to_torque_mode();
     *   bool is_in_speed_mode() const;
     *   bool is_in_torque_mode() const;
     *
     * Architectural Compliance:
     * - Memory: No dynamic allocation during runtime (real-time safety)
     * - Timing: Deterministic execution time < 10 μs per update
     * - Thread Safety: Not required (single-threaded real-time context)
     * - Error Handling: Graceful degradation during sensor failures
     * - Mode Switching: Bumpless transfer between speed and torque modes
     */
};

}  // namespace yeet::application::sfc::integration
