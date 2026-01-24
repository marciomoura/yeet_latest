#pragma once

#include <array>

#include "sfc/sfc_excitation_controller.hpp"
#include "yeet/model_registry.hpp"
#include "yeet/parameter.hpp"
#include "yeet/parameter_handler.hpp"
#include "yeet/signal.hpp"
#include "yeet/utils/parameter_accessors.hpp"
#include "yeet_control/units.hpp"

/**
 * @file sfc_excitation_controller_datamodel.hpp
 * @brief Datamodel specification for SFC Excitation Controller component
 */

namespace yeet::application::sfc::integration {

class sfc_excitation_controller_datamodel {
public:
    /**
     * @brief Input signals for the excitation controller component
     */
    struct inputs {
        /// Machine rotor speed [pu]
        signal_input<speed_pu_t> machine_rotor_speed_pu;

        /// Machine voltage magnitude [pu]
        signal_input<voltage_pu_t> machine_voltage_magnitude_pu;

        /// Release excitation command (enables field current output) [bool]
        signal_input<bool> release_excitation;
    };

    /**
     * @brief Output signals from the excitation controller component
     */
    struct outputs {
        /// Field current reference [pu]
        signal_output<current_pu_t> field_current_reference;

        /// Machine voltage reference used by excitation voltage loop [pu]
        signal_output<voltage_pu_t> machine_voltage_reference_pu;

        /// Machine voltage magnitude measurement used by excitation voltage loop [pu]
        signal_output<voltage_pu_t> machine_voltage_magnitude_pu;

        /// Machine voltage control error (reference - measured) [pu]
        signal_output<voltage_pu_t> machine_voltage_error_pu;

        /// Pre-fluxing elapsed time [s]
        signal_output<duration_t> prefluxing_elapsed_time;

        /// Field established flag - true when flux buildup timer has elapsed [bool]
        signal_output<bool> is_field_established;

        /// Excitation start command - true when excitation should be energized [bool]
        signal_output<bool> excitation_start_command;

        /// Excitation stop command - true when excitation should be de-energized [bool]
        signal_output<bool> excitation_stop_command;
    };

    /**
     * @brief Configuration parameters for the excitation controller
     */
    struct parameters {
        static constexpr std::size_t NUM_POINTS = sfc::sfc_excitation_controller::NUM_VOLTAGE_REFERENCE_POINTS;

        // --- Low-Voltage / Forced Mode Operation ---

        /// Forced field current reference used before release [pu]
        /// This constant value is maintained while speed and voltage are below release thresholds.
        parameter<current_pu_t> forced_field_current_pu = current_pu_t{0.8};

        /// Voltage threshold for releasing to automatic control [pu]
        /// When Vmeas exceeds this value, the PI voltage controller takes over.
        parameter<voltage_pu_t> release_voltage_threshold_pu = voltage_pu_t{0.2};

        /// Speed threshold for releasing to automatic control [pu]
        /// When speed exceeds this value, the PI voltage controller takes over.
        parameter<speed_pu_t> release_speed_threshold_pu = speed_pu_t{0.2};

        // --- Automatic Voltage Control (PI) ---

        /// Speed axis for voltage reference lookup table [pu]
        parameter<std::array<speed_pu_t, NUM_POINTS>> voltage_reference_speed_axis_pu =
            sfc::sfc_excitation_controller::kDefaultVoltageReferenceSpeedAxisPu;

        /// Voltage reference lookup table values [pu]
        parameter<std::array<voltage_pu_t, NUM_POINTS>> voltage_reference_pu =
            sfc::sfc_excitation_controller::kDefaultVoltageReferencePu;

        /// Voltage PI proportional gain
        /// Default: 1.0 provides moderate response for voltage regulation
        parameter<real_t> voltage_pi_kp = real_t{1.0};

        /// Voltage PI integral time constant [s]
        /// Default: 1.0s provides stable voltage regulation without overshoot
        parameter<duration_t> voltage_pi_ti = duration_t{1.0};

        /// Voltage PI output limits: field current reference min/max [pu]
        parameter<current_pu_t> voltage_pi_output_min_pu = current_pu_t{0.0};
        parameter<current_pu_t> voltage_pi_output_max_pu = current_pu_t{1.2};

        /// Pre-fluxing duration [s]
        /// Time required to establish the magnetic field before releasing excitation
        parameter<duration_t> prefluxing_duration = duration_t{2.0};

        /// Enable override for field current reference output
        /// When true, field_current_reference_override_value is used instead of controller output
        parameter<bool> field_current_reference_override = false;

        /// Override value for field current reference [pu]
        /// Used when field_current_reference_override is true
        parameter<current_pu_t> field_current_reference_override_value = current_pu_t{1.0};
    };

private:
    inputs* _model_inputs;
    outputs* _model_outputs;
    parameters* _model_parameters;

public:
    sfc_excitation_controller_datamodel(inputs* model_inputs, outputs* model_outputs, parameters* model_parameters)
        : _model_inputs(model_inputs), _model_outputs(model_outputs), _model_parameters(model_parameters)
    {
        assert(_model_inputs != nullptr);
        assert(_model_outputs != nullptr);
        assert(_model_parameters != nullptr);
    }

    void register_data_model(yeet::telemetry::parameter_handler& handler)
    {
        static constexpr std::string_view component_name = "excitation_controller";

        handler.register_parameter(component_name, "forced_field_current_pu",
            yeet::utils::make_units_accessor(_model_parameters->forced_field_current_pu));
        handler.register_parameter(component_name, "release_voltage_threshold_pu",
            yeet::utils::make_units_accessor(_model_parameters->release_voltage_threshold_pu));
        handler.register_parameter(component_name, "release_speed_threshold_pu",
            yeet::utils::make_units_accessor(_model_parameters->release_speed_threshold_pu));

        // Register voltage reference lookup table entries individually
        static constexpr std::string_view speed_axis_names[] = {"voltage_reference_speed_axis_pu[0]",
            "voltage_reference_speed_axis_pu[1]", "voltage_reference_speed_axis_pu[2]",
            "voltage_reference_speed_axis_pu[3]", "voltage_reference_speed_axis_pu[4]",
            "voltage_reference_speed_axis_pu[5]", "voltage_reference_speed_axis_pu[6]"};

        for (std::size_t i = 0; i < parameters::NUM_POINTS; ++i) {
            handler.register_parameter(component_name, speed_axis_names[i],
                yeet::utils::make_array_element_accessor(_model_parameters->voltage_reference_speed_axis_pu, i));
        }

        static constexpr std::string_view voltage_ref_names[] = {"voltage_reference_pu[0]", "voltage_reference_pu[1]",
            "voltage_reference_pu[2]", "voltage_reference_pu[3]", "voltage_reference_pu[4]", "voltage_reference_pu[5]",
            "voltage_reference_pu[6]"};

        for (std::size_t i = 0; i < parameters::NUM_POINTS; ++i) {
            handler.register_parameter(component_name, voltage_ref_names[i],
                yeet::utils::make_array_element_accessor(_model_parameters->voltage_reference_pu, i));
        }

        handler.register_parameter(
            component_name, "kp", yeet::utils::make_units_accessor(_model_parameters->voltage_pi_kp));
        handler.register_parameter(
            component_name, "ti", yeet::utils::make_units_accessor(_model_parameters->voltage_pi_ti));
        handler.register_parameter(component_name, "pi_output_min_pu",
            yeet::utils::make_units_accessor(_model_parameters->voltage_pi_output_min_pu));
        handler.register_parameter(component_name, "pi_output_max_pu",
            yeet::utils::make_units_accessor(_model_parameters->voltage_pi_output_max_pu));

        handler.register_parameter(component_name, "prefluxing_duration",
            yeet::utils::make_units_accessor(_model_parameters->prefluxing_duration));
        handler.register_parameter(component_name, "field_current_reference_override",
            yeet::utils::make_bool_accessor(_model_parameters->field_current_reference_override));
        handler.register_parameter(component_name, "field_current_reference_override_value",
            yeet::utils::make_units_accessor(_model_parameters->field_current_reference_override_value));
    }

    inputs* get_inputs() { return _model_inputs; }
    outputs* get_outputs() { return _model_outputs; }
    parameters* get_parameters() { return _model_parameters; }
};

}  // namespace yeet::application::sfc::integration
