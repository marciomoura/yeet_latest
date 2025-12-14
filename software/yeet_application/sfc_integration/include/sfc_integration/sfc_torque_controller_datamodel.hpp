#pragma once

#include <array>
#include <string>

#include "sfc/sfc_state_machine.hpp"
#include "sfc/sfc_torque_controller.hpp"
#include "yeet/model_registry.hpp"
#include "yeet/parameter.hpp"
#include "yeet/signal.hpp"
#include "yeet/utils/parameter_accessors.hpp"
#include "yeet_control/units.hpp"

/**
 * @file sfc_torque_controller_datamodel.hpp
 * @brief Datamodel specification for SFC Torque Controller component
 *
 * The torque controller sits between the speed controller and current controller,
 * managing torque-level operations including breakaway boost, dynamic limiting,
 * and torque-to-current conversion.
 */

namespace yeet::application::sfc::integration {

class sfc_torque_controller_datamodel {
public:
    /**
     * @brief Input signals for the torque controller component
     */
    struct inputs {
        /// Torque reference from speed controller [pu]
        signal_input<torque_pu_t> torque_reference;

        /// Speed reference used for speed-based torque limiting [pu]
        signal_input<speed_pu_t> speed_reference;

        /// Grid-side state machine current state ID (received via fiber)
        signal_input<yeet::application::sfc::sfc_state_id> current_state_id;

        /// Local machine-side protection trip indication
        signal_input<bool> machine_protection_tripped;

        /// Enable signal for breakaway boost from speed generator
        signal_input<bool> enable_breakaway_boost;

        /// Saturation feedback from downstream current controller
        signal_input<bool> is_saturated;

        /// Control mode selection (SPEED_CONTROL or MANUAL_TORQUE) from startup sequencer
        signal_input<sfc::sfc_torque_controller::control_mode> control_mode;

        /// Manual torque reference for virtual reference frame (startup phase) [pu]
        signal_input<torque_pu_t> manual_torque_reference;
    };

    /**
     * @brief Output signals from the torque controller component
     */
    struct outputs {
        /// Current reference output to current controller [pu]
        signal_output<current_pu_t> current_reference;

        /// Controller saturation status (combines local and downstream saturation)
        signal_output<bool> is_saturated;

        /// Breakaway boost active status
        signal_output<bool> is_breakaway_boost_active;

        /// Control loop active status
        signal_output<bool> is_active;

        /// Applied torque reference output (after limiting and boost) for bumpless transfer [pu]
        signal_output<torque_pu_t> torque_reference;

        /// Braking requested but clamped to minimum motoring torque
        signal_output<bool> is_braking_requested;

        /// Speed-based minimum torque limit evaluated at current speed reference [pu]
        signal_output<torque_pu_t> speed_based_min_torque_limit;

        /// Speed-based maximum torque limit evaluated at current speed reference [pu]
        signal_output<torque_pu_t> speed_based_max_torque_limit;
    };

    /**
     * @brief Configuration parameters for the torque controller
     */
    struct parameters {
        static constexpr size_t NUM_SPEED_TORQUE_POINTS = sfc::sfc_torque_controller::NUM_SPEED_TORQUE_LIMIT_POINTS;

        /// Enable override for torque reference testing
        parameter<bool> torque_reference_override = false;

        /// Forced torque reference value when override is enabled
        parameter<torque_pu_t> torque_reference_override_value = torque_pu_t{0.0};

        /// Minimum torque limit [pu]
        parameter<torque_pu_t> absolute_min_torque = torque_pu_t{-1.2};

        /// Maximum torque limit [pu]
        parameter<torque_pu_t> absolute_max_torque = torque_pu_t{1.2};

        /// Torque-to-current conversion factor
        parameter<real_t> torque_to_current_factor = real_t(1.0);

        /// Torque rate limit [pu/s]
        parameter<real_t> torque_rate_limit = real_t(10.0);

        /// Enable torque rate limiting
        parameter<bool> enable_torque_rate_limiting = true;

        /// Breakaway boost torque magnitude [pu]
        parameter<torque_pu_t> breakaway_boost_torque = torque_pu_t{0.0};

        /// Breakaway boost duration [s]
        parameter<duration_t> breakaway_boost_duration = duration_t{1.0};

        /// Minimum motoring torque for natural commutation [pu]
        parameter<torque_pu_t> minimum_motoring_torque = torque_pu_t{0.02};

        /// Enable minimum motoring torque clamp
        parameter<bool> enable_minimum_motoring_torque = true;

        /// Enable speed-based torque limiting (speed reference -> min/max torque)
        parameter<bool> enable_speed_based_torque_limits = false;

        /// Speed axis for speed-based torque limit lookup tables [pu]
        parameter<std::array<speed_pu_t, NUM_SPEED_TORQUE_POINTS>> speed_torque_limit_speed_axis_pu =
            sfc::sfc_torque_controller::kDefaultSpeedTorqueLimitSpeedAxisPu;

        /// Speed-based minimum torque limits [pu]
        parameter<std::array<torque_pu_t, NUM_SPEED_TORQUE_POINTS>> speed_torque_limit_min_torque_pu =
            sfc::sfc_torque_controller::kDefaultSpeedTorqueLimitMinTorquePu;

        /// Speed-based maximum torque limits [pu]
        parameter<std::array<torque_pu_t, NUM_SPEED_TORQUE_POINTS>> speed_torque_limit_max_torque_pu =
            sfc::sfc_torque_controller::kDefaultSpeedTorqueLimitMaxTorquePu;
    };

private:
    inputs* _model_inputs;
    outputs* _model_outputs;
    parameters* _model_parameters;

public:
    sfc_torque_controller_datamodel(inputs* model_inputs, outputs* model_outputs, parameters* model_parameters)
        : _model_inputs(model_inputs), _model_outputs(model_outputs), _model_parameters(model_parameters)
    {
        assert(_model_inputs != nullptr);
        assert(_model_outputs != nullptr);
        assert(_model_parameters != nullptr);
    }

    void register_data_model(model_registry& registry)
    {
        model_registry_entry entry{signal_registry{}, parameter_registry{}};

        entry.parameters.insert(
            "torque_reference_override", yeet::utils::make_bool_accessor(_model_parameters->torque_reference_override));
        entry.parameters.insert("torque_reference_override_value",
            yeet::utils::make_units_accessor(_model_parameters->torque_reference_override_value));
        entry.parameters.insert(
            "absolute_min_torque", yeet::utils::make_units_accessor(_model_parameters->absolute_min_torque));
        entry.parameters.insert(
            "absolute_max_torque", yeet::utils::make_units_accessor(_model_parameters->absolute_max_torque));
        entry.parameters.insert(
            "torque_to_current_factor", yeet::utils::make_units_accessor(_model_parameters->torque_to_current_factor));
        entry.parameters.insert(
            "torque_rate_limit", yeet::utils::make_units_accessor(_model_parameters->torque_rate_limit));
        entry.parameters.insert("enable_torque_rate_limiting",
            yeet::utils::make_bool_accessor(_model_parameters->enable_torque_rate_limiting));
        entry.parameters.insert(
            "breakaway_boost_torque", yeet::utils::make_units_accessor(_model_parameters->breakaway_boost_torque));
        entry.parameters.insert(
            "breakaway_boost_duration", yeet::utils::make_units_accessor(_model_parameters->breakaway_boost_duration));
        entry.parameters.insert(
            "minimum_motoring_torque", yeet::utils::make_units_accessor(_model_parameters->minimum_motoring_torque));
        entry.parameters.insert("enable_minimum_motoring_torque",
            yeet::utils::make_bool_accessor(_model_parameters->enable_minimum_motoring_torque));

        entry.parameters.insert("enable_speed_based_torque_limits",
            yeet::utils::make_bool_accessor(_model_parameters->enable_speed_based_torque_limits));

        // Per-element accessors for LUT arrays using generic array accessor
        for (std::size_t i = 0; i < parameters::NUM_SPEED_TORQUE_POINTS; ++i) {
            std::string param_name = "speed_torque_limit_speed_axis_pu[" + std::to_string(i) + "]";
            entry.parameters.insert(param_name,
                yeet::utils::make_array_element_accessor(_model_parameters->speed_torque_limit_speed_axis_pu, i));
        }

        for (std::size_t i = 0; i < parameters::NUM_SPEED_TORQUE_POINTS; ++i) {
            std::string param_name = "speed_torque_limit_min_torque_pu[" + std::to_string(i) + "]";
            entry.parameters.insert(param_name,
                yeet::utils::make_array_element_accessor(_model_parameters->speed_torque_limit_min_torque_pu, i));
        }

        for (std::size_t i = 0; i < parameters::NUM_SPEED_TORQUE_POINTS; ++i) {
            std::string param_name = "speed_torque_limit_max_torque_pu[" + std::to_string(i) + "]";
            entry.parameters.insert(param_name,
                yeet::utils::make_array_element_accessor(_model_parameters->speed_torque_limit_max_torque_pu, i));
        }

        registry.insert("torque_controller", std::move(entry));
    }

    inputs* get_inputs() { return _model_inputs; }
    outputs* get_outputs() { return _model_outputs; }
    parameters* get_parameters() { return _model_parameters; }
};

}  // namespace yeet::application::sfc::integration
