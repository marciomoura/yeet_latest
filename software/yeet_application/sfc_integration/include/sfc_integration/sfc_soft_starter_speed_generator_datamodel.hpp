#pragma once

#include "sfc/sfc_soft_starter_speed_generator.hpp"
#include "sfc/sfc_state_machine.hpp"
#include "yeet/model_registry.hpp"
#include "yeet/parameter.hpp"
#include "yeet/signal.hpp"
#include "yeet/utils/parameter_accessors.hpp"
#include "yeet_control/units.hpp"

/**
 * @file sfc_soft_starter_speed_generator_datamodel.hpp
 * @brief Datamodel specification for SFC Soft-Starter Speed Reference Generator component
 *
 * This component generates smooth speed reference trajectories during the starting sequence
 * of a gas turbine or synchronous machine. It manages multiple acceleration profiles
 * (creep speed, purge, ignition, assist) and handles transitions between them.
 */

namespace yeet::application::sfc::integration {

class sfc_soft_starter_speed_generator_datamodel {
public:
    /**
     * @brief Input signals for the speed reference generator component
     */
    struct inputs {
        /// Current state ID from state machine
        signal_input<sfc::sfc_state_id> current_state_id;

        /// Measured machine rotor speed [pu]
        signal_input<speed_pu_t> measured_speed;

        /// Command to run Profile 1 [bool]
        signal_input<bool> run_profile_1;

        /// Command to run Profile 2 [bool]
        signal_input<bool> run_profile_2;

        /// Command to run Profile 3 [bool]
        signal_input<bool> run_profile_3;

        /// Command to run Profile 4 [bool]
        signal_input<bool> run_profile_4;

        /// Command to hold current speed [bool]
        signal_input<bool> hold_speed;

        /// Command to decelerate to zero [bool]
        signal_input<bool> decelerate;

        /// Command to go to idle [bool]
        signal_input<bool> idle;
    };

    /**
     * @brief Output signals from the speed reference generator component
     */
    struct outputs {
        /// Speed reference output to speed controller [pu]
        signal_output<speed_pu_t> speed_reference;

        /// Maximum torque limit [pu]
        signal_output<torque_pu_t> torque_limit_max_pu;

        /// Minimum torque limit [pu]
        signal_output<torque_pu_t> torque_limit_min_pu;

        /// Active acceleration profile has completed [bool]
        signal_output<bool> is_profile_complete;

        /// Breakaway torque boost is active [bool]
        signal_output<bool> is_breakaway_boost_active;

        /// Currently active command
        signal_output<sfc::speed_command_t> active_command;

        /// Currently selected profile
        signal_output<sfc::acceleration_profile_t> selected_profile;
    };

    /**
     * @brief Configuration parameters for the speed reference generator
     *
     * All profiles can be field-configured. The names (creep, purge, ignition, assist)
     * are semantic labels but the actual profile behavior is fully programmable.
     */
    struct parameters {
        // === Profile 1 Configuration (Typically: Creep Speed) ===
        /// Enable Profile 1
        parameter<bool> profile_1_enabled = true;
        /// Target speed for Profile 1 [pu]
        parameter<speed_pu_t> profile_1_target_speed = speed_pu_t{0.05};
        /// Acceleration time for Profile 1 [s]
        parameter<duration_t> profile_1_acceleration_time = duration_t{30.0};
        /// Maximum torque limit during Profile 1 [pu]
        parameter<torque_pu_t> profile_1_torque_limit_max = torque_pu_t{0.40};
        /// Minimum torque limit during Profile 1 [pu]
        parameter<torque_pu_t> profile_1_torque_limit_min = torque_pu_t{-0.20};
        /// Initial torque boost for Profile 1 [pu]
        parameter<torque_pu_t> profile_1_initial_torque_boost = torque_pu_t{0.20};
        /// Boost duration for Profile 1 [s]
        parameter<duration_t> profile_1_boost_duration = duration_t{2.0};

        // === Profile 2 Configuration (Typically: Purge Acceleration) ===
        /// Enable Profile 2
        parameter<bool> profile_2_enabled = true;
        /// Target speed for Profile 2 [pu]
        parameter<speed_pu_t> profile_2_target_speed = speed_pu_t{0.20};
        /// Acceleration time for Profile 2 [s]
        parameter<duration_t> profile_2_acceleration_time = duration_t{120.0};
        /// Maximum torque limit during Profile 2 [pu]
        parameter<torque_pu_t> profile_2_torque_limit_max = torque_pu_t{0.8};
        /// Minimum torque limit during Profile 2 [pu]
        parameter<torque_pu_t> profile_2_torque_limit_min = torque_pu_t{-0.20};
        /// Initial torque boost for Profile 2 [pu]
        parameter<torque_pu_t> profile_2_initial_torque_boost = torque_pu_t{0.15};
        /// Boost duration for Profile 2 [s]
        parameter<duration_t> profile_2_boost_duration = duration_t{2.0};

        // === Profile 3 Configuration (Typically: Ignition Acceleration) ===
        /// Enable Profile 3
        parameter<bool> profile_3_enabled = true;
        /// Target speed for Profile 3 [pu]
        parameter<speed_pu_t> profile_3_target_speed = speed_pu_t{0.55};
        /// Acceleration time for Profile 3 [s]
        parameter<duration_t> profile_3_acceleration_time = duration_t{180.0};
        /// Maximum torque limit during Profile 3 [pu]
        parameter<torque_pu_t> profile_3_torque_limit_max = torque_pu_t{0.8};
        /// Minimum torque limit during Profile 3 [pu]
        parameter<torque_pu_t> profile_3_torque_limit_min = torque_pu_t{-0.30};
        /// Initial torque boost for Profile 3 [pu]
        parameter<torque_pu_t> profile_3_initial_torque_boost = torque_pu_t{0.10};
        /// Boost duration for Profile 3 [s]
        parameter<duration_t> profile_3_boost_duration = duration_t{1.5};

        // === Profile 4 Configuration (Typically: Acceleration Assist) ===
        /// Enable Profile 4
        parameter<bool> profile_4_enabled = true;
        /// Target speed for Profile 4 [pu]
        parameter<speed_pu_t> profile_4_target_speed = speed_pu_t{0.95};
        /// Acceleration time for Profile 4 [s]
        parameter<duration_t> profile_4_acceleration_time = duration_t{300.0};
        /// Maximum torque limit during Profile 4 [pu]
        parameter<torque_pu_t> profile_4_torque_limit_max = torque_pu_t{0.80};
        /// Minimum torque limit during Profile 4 [pu]
        parameter<torque_pu_t> profile_4_torque_limit_min = torque_pu_t{-0.40};
        /// Initial torque boost for Profile 4 [pu]
        parameter<torque_pu_t> profile_4_initial_torque_boost = torque_pu_t{0.05};
        /// Boost duration for Profile 4 [s]
        parameter<duration_t> profile_4_boost_duration = duration_t{1.0};

        // === Deceleration Configuration ===
        /// Deceleration time from any speed to zero [s]
        parameter<duration_t> deceleration_time = duration_t{60.0};

        // === Absolute Safety Limits ===
        /// Absolute minimum speed limit [pu]
        parameter<speed_pu_t> min_speed_limit = speed_pu_t{0.0};
        /// Absolute maximum speed limit [pu]
        parameter<speed_pu_t> max_speed_limit = speed_pu_t{1.05};
        /// Absolute maximum torque limit [pu]
        parameter<torque_pu_t> max_torque_limit = torque_pu_t{1.2};
        /// Absolute minimum torque limit [pu]
        parameter<torque_pu_t> min_torque_limit = torque_pu_t{-1.2};

        parameter<bool> override_speed_reference = false;
        parameter<torque_pu_t> override_min_torque_limit_value = torque_pu_t{-2.0};
        parameter<torque_pu_t> override_max_torque_limit_value = torque_pu_t{2.0};
        parameter<speed_pu_t> override_speed_reference_value = speed_pu_t{0.0};
    };

private:
    inputs* _model_inputs;
    outputs* _model_outputs;
    parameters* _model_parameters;

public:
    sfc_soft_starter_speed_generator_datamodel(
        inputs* model_inputs, outputs* model_outputs, parameters* model_parameters)
        : _model_inputs(model_inputs), _model_outputs(model_outputs), _model_parameters(model_parameters)
    {
    }

    void register_data_model(model_registry& registry)
    {
        model_registry_entry entry{signal_registry{}, parameter_registry{}};

        // Register profile 1 parameters
        entry.parameters.insert(
            "profile_1_enabled", yeet::utils::make_bool_accessor(_model_parameters->profile_1_enabled));
        entry.parameters.insert(
            "profile_1_target_speed", yeet::utils::make_units_accessor(_model_parameters->profile_1_target_speed));
        entry.parameters.insert("profile_1_acceleration_time",
            yeet::utils::make_units_accessor(_model_parameters->profile_1_acceleration_time));
        entry.parameters.insert("profile_1_torque_limit_max",
            yeet::utils::make_units_accessor(_model_parameters->profile_1_torque_limit_max));
        entry.parameters.insert("profile_1_torque_limit_min",
            yeet::utils::make_units_accessor(_model_parameters->profile_1_torque_limit_min));
        entry.parameters.insert("profile_1_initial_torque_boost",
            yeet::utils::make_units_accessor(_model_parameters->profile_1_initial_torque_boost));
        entry.parameters.insert(
            "profile_1_boost_duration", yeet::utils::make_units_accessor(_model_parameters->profile_1_boost_duration));

        // // Register profile 2 parameters
        // entry.parameters.insert(
        //     "profile_2_enabled", yeet::utils::make_bool_accessor(_model_parameters->profile_2_enabled));
        // entry.parameters.insert(
        //     "profile_2_target_speed", yeet::utils::make_units_accessor(_model_parameters->profile_2_target_speed));
        // entry.parameters.insert("profile_2_acceleration_time",
        //     yeet::utils::make_units_accessor(_model_parameters->profile_2_acceleration_time));
        // entry.parameters.insert("profile_2_torque_limit_max",
        //     yeet::utils::make_units_accessor(_model_parameters->profile_2_torque_limit_max));
        // entry.parameters.insert("profile_2_torque_limit_min",
        //     yeet::utils::make_units_accessor(_model_parameters->profile_2_torque_limit_min));
        // entry.parameters.insert("profile_2_initial_torque_boost",
        //     yeet::utils::make_units_accessor(_model_parameters->profile_2_initial_torque_boost));
        // entry.parameters.insert(
        //     "profile_2_boost_duration",
        //     yeet::utils::make_units_accessor(_model_parameters->profile_2_boost_duration));

        // // Register profile 3 parameters
        // entry.parameters.insert(
        //     "profile_3_enabled", yeet::utils::make_bool_accessor(_model_parameters->profile_3_enabled));
        // entry.parameters.insert(
        //     "profile_3_target_speed", yeet::utils::make_units_accessor(_model_parameters->profile_3_target_speed));
        // entry.parameters.insert("profile_3_acceleration_time",
        //     yeet::utils::make_units_accessor(_model_parameters->profile_3_acceleration_time));
        // entry.parameters.insert("profile_3_torque_limit_max",
        //     yeet::utils::make_units_accessor(_model_parameters->profile_3_torque_limit_max));
        // entry.parameters.insert("profile_3_torque_limit_min",
        //     yeet::utils::make_units_accessor(_model_parameters->profile_3_torque_limit_min));
        // entry.parameters.insert("profile_3_initial_torque_boost",
        //     yeet::utils::make_units_accessor(_model_parameters->profile_3_initial_torque_boost));
        // entry.parameters.insert(
        //     "profile_3_boost_duration",
        //     yeet::utils::make_units_accessor(_model_parameters->profile_3_boost_duration));

        // // Register profile 4 parameters
        // entry.parameters.insert(
        //     "profile_4_enabled", yeet::utils::make_bool_accessor(_model_parameters->profile_4_enabled));
        // entry.parameters.insert(
        //     "profile_4_target_speed", yeet::utils::make_units_accessor(_model_parameters->profile_4_target_speed));
        // entry.parameters.insert("profile_4_acceleration_time",
        //     yeet::utils::make_units_accessor(_model_parameters->profile_4_acceleration_time));
        // entry.parameters.insert("profile_4_torque_limit_max",
        //     yeet::utils::make_units_accessor(_model_parameters->profile_4_torque_limit_max));
        // entry.parameters.insert("profile_4_torque_limit_min",
        //     yeet::utils::make_units_accessor(_model_parameters->profile_4_torque_limit_min));
        // entry.parameters.insert("profile_4_initial_torque_boost",
        //     yeet::utils::make_units_accessor(_model_parameters->profile_4_initial_torque_boost));
        // entry.parameters.insert(
        //     "profile_4_boost_duration",
        //     yeet::utils::make_units_accessor(_model_parameters->profile_4_boost_duration));

        // Register deceleration and safety limit parameters
        entry.parameters.insert(
            "deceleration_time", yeet::utils::make_units_accessor(_model_parameters->deceleration_time));
        entry.parameters.insert(
            "min_speed_limit", yeet::utils::make_units_accessor(_model_parameters->min_speed_limit));
        entry.parameters.insert(
            "max_speed_limit", yeet::utils::make_units_accessor(_model_parameters->max_speed_limit));
        entry.parameters.insert(
            "max_torque_limit", yeet::utils::make_units_accessor(_model_parameters->max_torque_limit));
        entry.parameters.insert(
            "min_torque_limit", yeet::utils::make_units_accessor(_model_parameters->min_torque_limit));

        // Register override parameters
        entry.parameters.insert(
            "override_speed_reference", yeet::utils::make_bool_accessor(_model_parameters->override_speed_reference));
        entry.parameters.insert("override_min_torque_limit_value",
            yeet::utils::make_units_accessor(_model_parameters->override_min_torque_limit_value));
        entry.parameters.insert("override_max_torque_limit_value",
            yeet::utils::make_units_accessor(_model_parameters->override_max_torque_limit_value));
        entry.parameters.insert("override_speed_reference_value",
            yeet::utils::make_units_accessor(_model_parameters->override_speed_reference_value));

        registry.insert("speed_generator", std::move(entry));
    }

    inputs& get_inputs() { return *_model_inputs; }
    outputs& get_outputs() { return *_model_outputs; }
    parameters& get_parameters() { return *_model_parameters; }
};

}  // namespace yeet::application::sfc::integration
