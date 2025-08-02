#pragma once

#include "yeet/model_registry.hpp"
#include "yeet/parameter.hpp"
#include "yeet/signal.hpp"
#include "yeet_control/reference_frame.hpp"
#include "yeet_control/six_pulse_generator.hpp"
#include "yeet_control/units.hpp"


/**
 * @file sfc_current_controller_datamodel.hpp
 * @brief Datamodel specification for SFC Current Controller component
 */

namespace yeet::application::sfc::integration {

class sfc_current_controller_datamodel {
public:
    /**
     * @brief Input signals for the current controller component
     */
    struct inputs {
        /// Current reference signal in per-unit (DC link base)
        /// Range: [-1.2, 1.2] pu
        yeet::signal_input<yeet::current_pu_t> current_reference_pu;

        /// Measured DC link current in per-unit
        /// Range: [-1.5, 1.5] pu
        yeet::signal_input<yeet::current_pu_t> dc_link_current_measured_pu;

        /// Ideal no-load DC voltage from grid-side rectifier
        /// Range: [0.8, 1.2] pu
        yeet::signal_input<yeet::voltage_pu_t> ideal_dc_voltage_pu;

        /// Estimated inverter back-EMF
        /// Range: [0.0, 1.2] pu
        yeet::signal_input<yeet::voltage_pu_t> inverter_back_emf_machine_pu;
    };

    /**
     * @brief Output signals from the current controller component
     */
    struct outputs {
        /// Grid-side rectifier firing angle reference
        /// Range: [0.0, π/2] radians
        yeet::signal_output<yeet::angle_t> firing_angle_reference;

        /// Current control error (reference - measured)
        /// Range: [-1.0, 1.0] pu
        yeet::signal_output<yeet::current_pu_t> current_error;

        /// PI controller output before limiting
        yeet::signal_output<yeet::angle_t> pi_output;

        /// Controller saturation status
        yeet::signal_output<bool> is_saturated;

        /// Control loop active status
        yeet::signal_output<bool> is_active;
    };

    /**
     * @brief Configuration parameters for the current controller
     */
    struct parameters {
        /// PI controller proportional gain
        /// Range: [1.0, 20.0]
        yeet::parameter<yeet::real_t> pi_proportional_gain = yeet::real_t(5.0);

        /// PI controller integral gain
        /// Range: [100.0, 2000.0]
        yeet::parameter<yeet::real_t> pi_integral_gain = yeet::real_t(500.0);

        /// Minimum firing angle limit
        /// Range: [0.0, π/6] radians
        yeet::parameter<yeet::angle_t> min_firing_angle = yeet::angle_t(0.0);

        /// Maximum firing angle limit
        /// Range: [π/4, π/2] radians
        yeet::parameter<yeet::angle_t> max_firing_angle = yeet::angle_t(1.5708);  // π/2

        /// Current reference rate limit
        /// Range: [1.0, 50.0] pu/s
        yeet::parameter<yeet::real_t> current_reference_rate_limit = yeet::real_t(10.0);

        /// Current reference magnitude limit
        /// Range: [1.0, 1.5] pu
        yeet::parameter<yeet::current_pu_t> current_reference_magnitude_limit = yeet::current_pu_t(1.1);

        /// Feedforward gain for ideal DC voltage compensation
        /// Range: [0.0, 1.0]
        yeet::parameter<yeet::real_t> feedforward_dc_voltage_gain = yeet::real_t(0.8);

        /// Feedforward gain for back-EMF compensation
        /// Range: [0.0, 1.0]
        yeet::parameter<yeet::real_t> feedforward_back_emf_gain = yeet::real_t(0.5);

        /// Anti-windup gain for integral term protection
        /// Range: [0.05, 0.5]
        yeet::parameter<yeet::real_t> anti_windup_gain = yeet::real_t(0.1);
    };

private:
    inputs* _model_inputs;
    outputs* _model_outputs;
    parameters* _model_parameters;

public:
    sfc_current_controller_datamodel(inputs* model_inputs, outputs* model_outputs, parameters* model_parameters)
        : _model_inputs(model_inputs), _model_outputs(model_outputs), _model_parameters(model_parameters) {
        assert(_model_inputs != nullptr);
        assert(_model_outputs != nullptr);
        assert(_model_parameters != nullptr);
    }

    void register_data_model(yeet::model_registry& registry) {
        yeet::model_registry_entry entry{yeet::signal_registry{}, yeet::parameter_registry{}};

        // Register outputs in signal registry
        entry.signal_registry.insert("firing_angle_reference", _model_outputs->firing_angle_reference);
        entry.signal_registry.insert("current_error", _model_outputs->current_error);
        entry.signal_registry.insert("pi_output", _model_outputs->pi_output);
        entry.signal_registry.insert("is_saturated", _model_outputs->is_saturated);
        entry.signal_registry.insert("is_active", _model_outputs->is_active);

        // Register parameters in parameter registry
        entry.parameter_registry.insert("pi_proportional_gain", _model_parameters->pi_proportional_gain);
        entry.parameter_registry.insert("pi_integral_gain", _model_parameters->pi_integral_gain);
        entry.parameter_registry.insert("min_firing_angle", _model_parameters->min_firing_angle);
        entry.parameter_registry.insert("max_firing_angle", _model_parameters->max_firing_angle);
        entry.parameter_registry.insert("current_reference_rate_limit",
                                        _model_parameters->current_reference_rate_limit);
        entry.parameter_registry.insert("current_reference_magnitude_limit",
                                        _model_parameters->current_reference_magnitude_limit);
        entry.parameter_registry.insert("feedforward_dc_voltage_gain", _model_parameters->feedforward_dc_voltage_gain);
        entry.parameter_registry.insert("feedforward_back_emf_gain", _model_parameters->feedforward_back_emf_gain);
        entry.parameter_registry.insert("anti_windup_gain", _model_parameters->anti_windup_gain);

        registry.insert("sfc_current_controller", std::move(entry));
    }

    inputs* get_inputs() { return _model_inputs; }
    outputs* get_outputs() { return _model_outputs; }
    parameters* get_parameters() { return _model_parameters; }
};
     *             voltage_pu_t ideal_dc_voltage_pu,
     *             voltage_pu_t inverter_back_emf_machine_pu,
     *             voltage_t machine_side_base_voltage,
     *             voltage_t grid_side_base_voltage);
     */ struct input_interface {
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
