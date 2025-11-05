#pragma once

#include <string>

#include "sfc/sfc_soft_starter_speed_generator.hpp"  // For speed_command_t and acceleration_profile_t
#include "sfc_integration/sfc_current_controller_runner.hpp"
#include "sfc_integration/sfc_dc_link_measurements_runner.hpp"
#include "sfc_integration/sfc_excitation_controller_runner.hpp"
#include "sfc_integration/sfc_grid_side_hardware_mock_runner.hpp"
#include "sfc_integration/sfc_grid_side_integrator_supervisor_runner.hpp"
#include "sfc_integration/sfc_grid_side_measurements_runner.hpp"
#include "sfc_integration/sfc_grid_side_modulator_runner.hpp"
#include "sfc_integration/sfc_grid_side_observer_runner.hpp"
#include "sfc_integration/sfc_grid_side_pulse_generator_runner.hpp"
#include "sfc_integration/sfc_machine_side_controller_runner.hpp"
#include "sfc_integration/sfc_machine_side_hardware_mock_runner.hpp"
#include "sfc_integration/sfc_machine_side_integrator_supervisor_runner.hpp"
#include "sfc_integration/sfc_machine_side_measurements_runner.hpp"
#include "sfc_integration/sfc_machine_side_modulator_runner.hpp"
#include "sfc_integration/sfc_machine_side_observer_runner.hpp"
#include "sfc_integration/sfc_machine_side_pulse_generator_runner.hpp"
#include "sfc_integration/sfc_operation_mode_monitor_runner.hpp"
#include "sfc_integration/sfc_plant_model_runner.hpp"
#include "sfc_integration/sfc_soft_starter_speed_generator_runner.hpp"
#include "sfc_integration/sfc_speed_controller_runner.hpp"
#include "sfc_integration/sfc_state_machine_runner.hpp"
#include "sfc_integration/sfc_torque_controller_runner.hpp"
#include "yeet_control/types.hpp"
#include "yeet_testing/data_sampler_entry.hpp"
#include "yeet_testing/simulator.hpp"

namespace yeet::application::sfc::integration {

static inline void register_minimal_current_controller_logger(
    yeet::simulator& simulator, sfc_current_controller_runner& current_controller)
{
    const std::string group{"current_controller:"};
    simulator.register_sampler_entry(yeet::make_lambda_sampler<current_pu_t>(
        group + "input current_reference", [&]() { return current_controller.get_inputs().current_reference.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<current_pu_t>(
        group + "input current_measured", [&]() { return current_controller.get_inputs().current_measured.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "input freeze_integrator", [&]() { return current_controller.get_inputs().freeze_integrator.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "input preset_request", [&]() { return current_controller.get_inputs().preset_request.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<current_pu_t>(
        group + "output current_error", [&]() { return current_controller.get_outputs().current_error.read(); }));
}

static inline void register_complete_current_controller_logger(
    yeet::simulator& simulator, sfc_current_controller_runner& current_controller)
{
    const std::string group{"current_controller:"};

    register_minimal_current_controller_logger(simulator, current_controller);

    simulator.register_sampler_entry(
        yeet::make_lambda_sampler<voltage_pu_t>(group + "input grid_side_voltage_magnitude",
            [&]() { return current_controller.get_inputs().grid_side_voltage_magnitude.read(); }));
    simulator.register_sampler_entry(
        yeet::make_lambda_sampler<voltage_pu_t>(group + "input machine_side_voltage_magnitude",
            [&]() { return current_controller.get_inputs().machine_side_voltage_magnitude.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "output is_active", [&]() { return current_controller.get_outputs().is_active.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<real_t>(group + "output firing_angle",
        [&]() { return current_controller.get_outputs().firing_angle.read().get_pu(); }));
}

static inline void register_minimal_speed_controller_logger(
    yeet::simulator& simulator, sfc_speed_controller_runner& speed_controller)
{
    const std::string group{"speed_controller:"};
    simulator.register_sampler_entry(yeet::make_lambda_sampler<speed_pu_t>(
        group + "input speed_reference_pu", [&]() { return speed_controller.get_inputs().speed_reference_pu.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<speed_pu_t>(
        group + "input measured_speed_pu", [&]() { return speed_controller.get_inputs().measured_speed_pu.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "input freeze_integrator", [&]() { return speed_controller.get_inputs().freeze_integrator.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<speed_pu_t>(group + "calculated speed_error", [&]() {
        return speed_controller.get_inputs().speed_reference_pu.read() -
               speed_controller.get_inputs().measured_speed_pu.read();
    }));
}

static inline void register_complete_speed_controller_logger(
    yeet::simulator& simulator, sfc_speed_controller_runner& speed_controller)
{
    const std::string group{"speed_controller:"};

    register_minimal_speed_controller_logger(simulator, speed_controller);

    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "input release_pulses", [&]() { return speed_controller.get_inputs().release_pulses.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "input preset_request", [&]() { return speed_controller.get_inputs().preset_request.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "input is_saturated", [&]() { return speed_controller.get_inputs().is_saturated.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<torque_pu_t>(
        group + "output torque_reference", [&]() { return speed_controller.get_outputs().torque_reference.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<real_t>(
        group + "output pi_output", [&]() { return speed_controller.get_outputs().pi_output.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "output is_saturated", [&]() { return speed_controller.get_outputs().is_saturated.read(); }));
}

static inline void register_minimal_excitation_controller_logger(
    yeet::simulator& simulator, sfc_excitation_controller_runner& excitation_controller)
{
    const std::string group{"excitation_controller:"};
    simulator.register_sampler_entry(yeet::make_lambda_sampler<speed_pu_t>(group + "input machine_rotor_speed_pu",
        [&]() { return excitation_controller.get_inputs().machine_rotor_speed_pu.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "input exciter_ready", [&]() { return excitation_controller.get_inputs().exciter_ready.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "input start_command", [&]() { return excitation_controller.get_inputs().start_command.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "input stop_command", [&]() { return excitation_controller.get_inputs().stop_command.read(); }));
    simulator.register_sampler_entry(
        yeet::make_lambda_sampler<voltage_pu_t>(group + "output excitation_voltage_setpoint_pu",
            [&]() { return excitation_controller.get_outputs().excitation_voltage_setpoint_pu.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "output enable_excitation",
        [&]() { return excitation_controller.get_outputs().enable_excitation.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "output is_field_established",
        [&]() { return excitation_controller.get_outputs().is_field_established.read(); }));
}

static inline void register_complete_excitation_controller_logger(
    yeet::simulator& simulator, sfc_excitation_controller_runner& excitation_controller)
{
    const std::string group{"excitation_controller:"};

    register_minimal_excitation_controller_logger(simulator, excitation_controller);

    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "output is_running", [&]() { return excitation_controller.get_outputs().is_running.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<duration_t>(group + "output prefluxing_elapsed_time",
        [&]() { return excitation_controller.get_outputs().prefluxing_elapsed_time.read(); }));
}

// --- Soft-starter speed generator runner loggers ---
static inline void register_minimal_sfc_soft_starter_speed_generator_logger(
    yeet::simulator& simulator, sfc_soft_starter_speed_generator_runner& speed_generator)
{
    const std::string group{"speed_generator:"};

    // Key outputs
    simulator.register_sampler_entry(yeet::make_lambda_sampler<speed_pu_t>(group + "output speed_reference_pu",
        [&]() { return speed_generator.get_outputs().speed_reference_pu.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<torque_pu_t>(group + "output torque_limit_max_pu",
        [&]() { return speed_generator.get_outputs().torque_limit_max_pu.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<torque_pu_t>(group + "output torque_limit_min_pu",
        [&]() { return speed_generator.get_outputs().torque_limit_min_pu.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "output is_breakaway_boost_active",
        [&]() { return speed_generator.get_outputs().is_breakaway_boost_active.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "output is_profile_complete",
        [&]() { return speed_generator.get_outputs().is_profile_complete.read(); }));
}

static inline void register_complete_sfc_soft_starter_speed_generator_logger(
    yeet::simulator& simulator, sfc_soft_starter_speed_generator_runner& speed_generator)
{
    const std::string group{"speed_generator:"};

    register_minimal_sfc_soft_starter_speed_generator_logger(simulator, speed_generator);

    // All inputs
    simulator.register_sampler_entry(yeet::make_lambda_sampler<sfc_state_id>(
        group + "input current_state_id", [&]() { return speed_generator.get_inputs().current_state_id.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<speed_pu_t>(
        group + "input measured_speed_pu", [&]() { return speed_generator.get_inputs().measured_speed_pu.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "input run_profile_1", [&]() { return speed_generator.get_inputs().run_profile_1.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "input run_profile_2", [&]() { return speed_generator.get_inputs().run_profile_2.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "input run_profile_3", [&]() { return speed_generator.get_inputs().run_profile_3.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "input run_profile_4", [&]() { return speed_generator.get_inputs().run_profile_4.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "input hold_speed", [&]() { return speed_generator.get_inputs().hold_speed.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "input decelerate", [&]() { return speed_generator.get_inputs().decelerate.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "input idle", [&]() { return speed_generator.get_inputs().idle.read(); }));

    // Additional outputs
    simulator.register_sampler_entry(yeet::make_lambda_sampler<speed_command_t>(
        group + "output active_command", [&]() { return speed_generator.get_outputs().active_command.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<acceleration_profile_t>(
        group + "output selected_profile", [&]() { return speed_generator.get_outputs().selected_profile.read(); }));
}

// --- Torque controller runner loggers ---
static inline void register_minimal_sfc_torque_controller_logger(
    yeet::simulator& simulator, sfc_torque_controller_runner& torque_controller)
{
    const std::string group{"torque_controller:"};

    // Key inputs
    simulator.register_sampler_entry(yeet::make_lambda_sampler<torque_pu_t>(
        group + "input torque_reference", [&]() { return torque_controller.get_inputs().torque_reference.read(); }));

    // Key outputs
    simulator.register_sampler_entry(yeet::make_lambda_sampler<current_pu_t>(group + "output current_reference",
        [&]() { return torque_controller.get_outputs().current_reference.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "output is_breakaway_boost_active",
        [&]() { return torque_controller.get_outputs().is_breakaway_boost_active.read(); }));
}

static inline void register_complete_sfc_torque_controller_logger(
    yeet::simulator& simulator, sfc_torque_controller_runner& torque_controller)
{
    const std::string group{"torque_controller:"};

    register_minimal_sfc_torque_controller_logger(simulator, torque_controller);

    // All inputs
    simulator.register_sampler_entry(yeet::make_lambda_sampler<torque_pu_t>(group + "input dynamic_max_torque_limit",
        [&]() { return torque_controller.get_inputs().dynamic_max_torque_limit.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<torque_pu_t>(group + "input dynamic_min_torque_limit",
        [&]() { return torque_controller.get_inputs().dynamic_min_torque_limit.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "input enable_breakaway_boost",
        [&]() { return torque_controller.get_inputs().enable_breakaway_boost.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "input is_saturated", [&]() { return torque_controller.get_inputs().is_saturated.read(); }));

    // Additional outputs
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "output is_saturated", [&]() { return torque_controller.get_outputs().is_saturated.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "output is_active", [&]() { return torque_controller.get_outputs().is_active.read(); }));
}

static inline void register_minimal_dc_link_measurements_logger(
    yeet::simulator& simulator, sfc_dc_link_measurements_runner& dc_link_measurements)
{
    const std::string group{"dc_link_measurements:"};
    simulator.register_sampler_entry(
        yeet::make_lambda_array_sampler<abc<current_pu_t>>(group + "input grid_side_current",
            [&]() { return dc_link_measurements.get_inputs().grid_side_current.read(); }));

    simulator.register_sampler_entry(yeet::make_lambda_sampler<current_pu_t>(
        group + "output dc_current_pu", [&]() { return dc_link_measurements.get_outputs().dc_current_pu.read(); }));
}

static inline void register_complete_dc_link_measurements_logger(
    yeet::simulator& simulator, sfc_dc_link_measurements_runner& dc_link_measurements)
{
    const std::string group{"dc_link_measurements:"};

    register_minimal_dc_link_measurements_logger(simulator, dc_link_measurements);

    simulator.register_sampler_entry(yeet::make_lambda_sampler<current_pu_t>(group + "output dc_current_filtered_pu",
        [&]() { return dc_link_measurements.get_outputs().dc_current_filtered_pu.read(); }));
    simulator.register_sampler_entry(
        yeet::make_lambda_sampler<current_pu_t>(group + "output dc_current_heavy_filtered_pu",
            [&]() { return dc_link_measurements.get_outputs().dc_current_heavy_filtered_pu.read(); }));
}

static inline void register_minimal_grid_side_measurements_logger(
    yeet::simulator& simulator, sfc_grid_side_measurements_ideal_runner& grid_side_measurements)
{
    const std::string group{"grid_side_measurements:"};
    simulator.register_sampler_entry(
        yeet::make_lambda_array_sampler<abc<voltage_pu_t>>(group + "output grid_voltage_abc",
            [&]() { return grid_side_measurements.get_outputs().grid_voltage_abc.read(); }));
    simulator.register_sampler_entry(
        yeet::make_lambda_array_sampler<abc<current_pu_t>>(group + "output grid_current_measurements",
            [&]() { return grid_side_measurements.get_outputs().grid_current_abc.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<voltage_pu_t>(group + "output grid_voltage_magnitude",
        [&]() { return grid_side_measurements.get_outputs().grid_voltage_magnitude.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "output is_commuting", [&]() { return grid_side_measurements.get_outputs().is_commuting.read(); }));
}

static inline void register_complete_grid_side_measurements_logger(
    yeet::simulator& simulator, sfc_grid_side_measurements_ideal_runner& grid_side_measurements)
{
    const std::string group{"grid_side_measurements:"};

    register_minimal_grid_side_measurements_logger(simulator, grid_side_measurements);

    simulator.register_sampler_entry(yeet::make_lambda_sampler<frequency_pu_t>(group + "output grid_side_frequency",
        [&]() { return grid_side_measurements.get_outputs().grid_side_frequency.read(); }));
    simulator.register_sampler_entry(
        yeet::make_lambda_array_sampler<alphabeta<voltage_pu_t>>(group + "output grid_voltage_alphabeta",
            [&]() { return grid_side_measurements.get_outputs().grid_voltage_alphabeta.read(); }));
    simulator.register_sampler_entry(
        yeet::make_lambda_array_sampler<alphabeta<voltage_pu_t>>(group + "output grid_voltage_alphabeta_filtered",
            [&]() { return grid_side_measurements.get_outputs().grid_voltage_alphabeta_filtered.read(); }));
    // raw grid_voltage_abc already in minimal logger
    simulator.register_sampler_entry(
        yeet::make_lambda_array_sampler<abc<voltage_pu_t>>(group + "output grid_voltage_abc_unmasked",
            [&]() { return grid_side_measurements.get_outputs().grid_voltage_abc_unmasked.read(); }));
    simulator.register_sampler_entry(
        yeet::make_lambda_array_sampler<abc<voltage_pu_t>>(group + "output grid_voltage_abc_filtered",
            [&]() { return grid_side_measurements.get_outputs().grid_voltage_abc_filtered.read(); }));
    // raw grid_current_abc already in minimal logger
    simulator.register_sampler_entry(
        yeet::make_lambda_sampler<voltage_pu_t>(group + "output grid_voltage_magnitude_filtered",
            [&]() { return grid_side_measurements.get_outputs().grid_voltage_magnitude_filtered.read(); }));
}

// --- ADC-based grid side measurements runner loggers ---
static inline void register_minimal_grid_side_measurements_logger(
    yeet::simulator& simulator, sfc_grid_side_measurements_runner& grid_side_measurements)
{
    const std::string group{"grid_side_measurements:"};

    simulator.register_sampler_entry(
        yeet::make_lambda_array_sampler<abc<voltage_pu_t>>(group + "output grid_voltage_abc",
            [&]() { return grid_side_measurements.get_outputs().grid_voltage_abc.read(); }));
    simulator.register_sampler_entry(
        yeet::make_lambda_array_sampler<abc<current_pu_t>>(group + "output grid_current_measurements",
            [&]() { return grid_side_measurements.get_outputs().grid_current_abc.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<voltage_pu_t>(group + "output grid_voltage_magnitude",
        [&]() { return grid_side_measurements.get_outputs().grid_voltage_magnitude.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "output is_commuting", [&]() { return grid_side_measurements.get_outputs().is_commuting.read(); }));
}

static inline void register_complete_grid_side_measurements_logger(
    yeet::simulator& simulator, sfc_grid_side_measurements_runner& grid_side_measurements)
{
    const std::string group{"grid_side_measurements:"};

    register_minimal_grid_side_measurements_logger(simulator, grid_side_measurements);

    simulator.register_sampler_entry(yeet::make_lambda_sampler<frequency_pu_t>(group + "output grid_side_frequency",
        [&]() { return grid_side_measurements.get_outputs().grid_side_frequency.read(); }));
    simulator.register_sampler_entry(
        yeet::make_lambda_array_sampler<alphabeta<voltage_pu_t>>(group + "output grid_voltage_alphabeta",
            [&]() { return grid_side_measurements.get_outputs().grid_voltage_alphabeta.read(); }));
    simulator.register_sampler_entry(
        yeet::make_lambda_array_sampler<alphabeta<voltage_pu_t>>(group + "output grid_voltage_alphabeta_filtered",
            [&]() { return grid_side_measurements.get_outputs().grid_voltage_alphabeta_filtered.read(); }));
    // raw grid_voltage_abc already in minimal logger
    simulator.register_sampler_entry(
        yeet::make_lambda_array_sampler<abc<voltage_pu_t>>(group + "output grid_voltage_abc_unmasked",
            [&]() { return grid_side_measurements.get_outputs().grid_voltage_abc_unmasked.read(); }));
    simulator.register_sampler_entry(
        yeet::make_lambda_array_sampler<abc<voltage_pu_t>>(group + "output grid_voltage_abc_filtered",
            [&]() { return grid_side_measurements.get_outputs().grid_voltage_abc_filtered.read(); }));
    // raw grid_current_abc already in minimal logger
    simulator.register_sampler_entry(
        yeet::make_lambda_sampler<voltage_pu_t>(group + "output grid_voltage_magnitude_filtered",
            [&]() { return grid_side_measurements.get_outputs().grid_voltage_magnitude_filtered.read(); }));
}

static inline void register_minimal_machine_side_measurements_logger(
    yeet::simulator& simulator, sfc_machine_side_measurements_ideal_runner& machine_side_measurements)
{
    const std::string group{"machine_side_measurements:"};
    simulator.register_sampler_entry(
        yeet::make_lambda_array_sampler<abc<voltage_pu_t>>(group + "output machine_voltage_abc",
            [&]() { return machine_side_measurements.get_outputs().machine_voltage_abc.read(); }));
    simulator.register_sampler_entry(
        yeet::make_lambda_array_sampler<alphabeta<voltage_pu_t>>(group + "output machine_voltage_alphabeta",
            [&]() { return machine_side_measurements.get_outputs().machine_voltage_alphabeta.read(); }));
    simulator.register_sampler_entry(
        yeet::make_lambda_array_sampler<abc<current_pu_t>>(group + "output machine_current_abc",
            [&]() { return machine_side_measurements.get_outputs().machine_current_abc.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<speed_pu_t>(group + "output machine_side_speed",
        [&]() { return machine_side_measurements.get_outputs().machine_side_speed.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<voltage_pu_t>(group + "output machine_voltage_magnitude",
        [&]() { return machine_side_measurements.get_outputs().machine_voltage_magnitude.read(); }));
}

static inline void register_complete_machine_side_measurements_logger(
    yeet::simulator& simulator, sfc_machine_side_measurements_ideal_runner& machine_side_measurements)
{
    const std::string group{"machine_side_measurements:"};

    register_minimal_machine_side_measurements_logger(simulator, machine_side_measurements);

    // raw machine_voltage_alphabeta already in minimal logger
    simulator.register_sampler_entry(
        yeet::make_lambda_array_sampler<alphabeta<voltage_pu_t>>(group + "output machine_voltage_alphabeta_filtered",
            [&]() { return machine_side_measurements.get_outputs().machine_voltage_alphabeta_filtered.read(); }));
    // raw machine_voltage_abc already in minimal logger
    simulator.register_sampler_entry(
        yeet::make_lambda_array_sampler<abc<voltage_pu_t>>(group + "output machine_voltage_abc_unmasked",
            [&]() { return machine_side_measurements.get_outputs().machine_voltage_abc_unmasked.read(); }));
    simulator.register_sampler_entry(
        yeet::make_lambda_array_sampler<abc<voltage_pu_t>>(group + "output machine_voltage_abc_filtered",
            [&]() { return machine_side_measurements.get_outputs().machine_voltage_abc_filtered.read(); }));
    // raw machine_current_abc already in minimal logger
    simulator.register_sampler_entry(
        yeet::make_lambda_sampler<voltage_pu_t>(group + "output machine_voltage_magnitude_filtered",
            [&]() { return machine_side_measurements.get_outputs().machine_voltage_magnitude_filtered.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "output is_commuting", [&]() { return machine_side_measurements.get_outputs().is_commuting.read(); }));
}

// --- ADC-based machine side measurements runner loggers ---
static inline void register_minimal_machine_side_measurements_logger(
    yeet::simulator& simulator, sfc_machine_side_measurements_runner& machine_side_measurements)
{
    const std::string group{"machine_side_measurements:"};
    simulator.register_sampler_entry(
        yeet::make_lambda_array_sampler<abc<voltage_pu_t>>(group + "output machine_voltage_abc",
            [&]() { return machine_side_measurements.get_outputs().machine_voltage_abc.read(); }));
    simulator.register_sampler_entry(
        yeet::make_lambda_array_sampler<alphabeta<voltage_pu_t>>(group + "output machine_voltage_alphabeta",
            [&]() { return machine_side_measurements.get_outputs().machine_voltage_alphabeta.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<speed_pu_t>(group + "output machine_side_speed",
        [&]() { return machine_side_measurements.get_outputs().machine_side_speed.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<voltage_pu_t>(group + "output machine_voltage_magnitude",
        [&]() { return machine_side_measurements.get_outputs().machine_voltage_magnitude.read(); }));
}

static inline void register_complete_machine_side_measurements_logger(
    yeet::simulator& simulator, sfc_machine_side_measurements_runner& machine_side_measurements)
{
    const std::string group{"machine_side_measurements:"};

    register_minimal_machine_side_measurements_logger(simulator, machine_side_measurements);

    // raw machine_voltage_alphabeta already in minimal logger
    simulator.register_sampler_entry(
        yeet::make_lambda_array_sampler<alphabeta<voltage_pu_t>>(group + "output machine_voltage_alphabeta_filtered",
            [&]() { return machine_side_measurements.get_outputs().machine_voltage_alphabeta_filtered.read(); }));
    // raw machine_voltage_abc already in minimal logger
    simulator.register_sampler_entry(
        yeet::make_lambda_array_sampler<abc<voltage_pu_t>>(group + "output machine_voltage_abc_unmasked",
            [&]() { return machine_side_measurements.get_outputs().machine_voltage_abc_unmasked.read(); }));
    simulator.register_sampler_entry(
        yeet::make_lambda_array_sampler<abc<voltage_pu_t>>(group + "output machine_voltage_abc_filtered",
            [&]() { return machine_side_measurements.get_outputs().machine_voltage_abc_filtered.read(); }));
    simulator.register_sampler_entry(
        yeet::make_lambda_sampler<voltage_pu_t>(group + "output machine_voltage_magnitude_filtered",
            [&]() { return machine_side_measurements.get_outputs().machine_voltage_magnitude_filtered.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "output is_commuting", [&]() { return machine_side_measurements.get_outputs().is_commuting.read(); }));
}

static inline void register_minimal_grid_side_observer_logger(
    yeet::simulator& simulator, sfc_grid_side_observer_runner& grid_side_observer)
{
    const std::string group{"grid_side_observer:"};
    simulator.register_sampler_entry(yeet::make_lambda_array_sampler<abc<voltage_pu_t>>(
        group + "input grid_voltage_pu", [&]() { return grid_side_observer.get_inputs().grid_voltage_pu.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<real_t>(group + "output estimated_angle",
        [&]() { return grid_side_observer.get_outputs().estimated_position.read().get_pu(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<frequency_pu_t>(group + "output estimated_frequency_pu",
        [&]() { return grid_side_observer.get_outputs().estimated_frequency_pu.read(); }));
}

static inline void register_complete_grid_side_observer_logger(
    yeet::simulator& simulator, sfc_grid_side_observer_runner& grid_side_observer)
{
    const std::string group{"grid_side_observer:"};

    register_minimal_grid_side_observer_logger(simulator, grid_side_observer);

    // estimated_frequency_pu already in minimal logger
    simulator.register_sampler_entry(
        yeet::make_lambda_sampler<voltage_pu_t>(group + "output estimated_voltage_magnitude",
            [&]() { return grid_side_observer.get_outputs().estimated_voltage_magnitude.read(); }));
    simulator.register_sampler_entry(
        yeet::make_lambda_array_sampler<alphabeta<voltage_pu_t>>(group + "output filtered_voltage_alphabeta",
            [&]() { return grid_side_observer.get_outputs().filtered_voltage_alphabeta.read(); }));
}

static inline void register_minimal_state_machine_logger(
    yeet::simulator& simulator, sfc_state_machine_runner& state_machine)
{
    const std::string group{"state_machine:"};
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "input start_command", [&]() { return state_machine.get_inputs().start_command.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "input stop_command", [&]() { return state_machine.get_inputs().stop_command.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "output release_pulses", [&]() { return state_machine.get_outputs().release_pulses.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "output preset_request", [&]() { return state_machine.get_outputs().preset_request.read(); }));
}

static inline void register_complete_state_machine_logger(
    yeet::simulator& simulator, sfc_state_machine_runner& state_machine)
{
    const std::string group{"state_machine:"};

    register_minimal_state_machine_logger(simulator, state_machine);

    // Log the combined input signals (hardware + overrides) that are actually fed to the state machine
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "combined_input auxiliaries_are_on",
        [&]() { return state_machine.get_outputs().auxiliaries_are_on.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "combined_input breakers_are_on",
        [&]() { return state_machine.get_outputs().breakers_are_on.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "combined_input start_command", [&]() { return state_machine.get_outputs().start_command.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "combined_input stop_command", [&]() { return state_machine.get_outputs().stop_command.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "combined_input shutdown_request",
        [&]() { return state_machine.get_outputs().shutdown_request.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "combined_input is_current_extinguished",
        [&]() { return state_machine.get_outputs().is_current_extinguished.read(); }));

    // State status outputs
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "output has_transitioned", [&]() { return state_machine.get_outputs().has_transitioned.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<sfc_state_id>(
        group + "output current_state_id", [&]() { return state_machine.get_outputs().current_state_id.read(); }));
}

static inline void register_minimal_operation_mode_monitor_logger(
    yeet::simulator& simulator, sfc_operation_mode_monitor_runner& operation_mode_monitor)
{
    const std::string group{"operation_mode_monitor:"};
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "input enable", [&]() { return operation_mode_monitor.get_inputs().enable.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<speed_pu_t>(group + "input measured_speed_pu",
        [&]() { return operation_mode_monitor.get_inputs().measured_speed_pu.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<current_pu_t>(group + "input dc_link_current_pu",
        [&]() { return operation_mode_monitor.get_inputs().dc_link_current_pu.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<sfc_operating_mode_t>(
        group + "output operating_mode", [&]() { return operation_mode_monitor.get_outputs().operating_mode.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "output has_mode_changed",
        [&]() { return operation_mode_monitor.get_outputs().has_mode_changed.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "output current_is_zero",
        [&]() { return operation_mode_monitor.get_outputs().current_is_zero.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "output is_discontinuous_mode",
        [&]() { return operation_mode_monitor.get_outputs().is_discontinuous_mode.read(); }));
}

static inline void register_complete_operation_mode_monitor_logger(
    yeet::simulator& simulator, sfc_operation_mode_monitor_runner& operation_mode_monitor)
{
    const std::string group{"operation_mode_monitor:"};

    register_minimal_operation_mode_monitor_logger(simulator, operation_mode_monitor);

    // Add parameter logging for debugging
    simulator.register_sampler_entry(yeet::make_lambda_sampler<speed_pu_t>(group + "param mode_handover_speed_pu",
        [&]() { return operation_mode_monitor.get_parameters().mode_handover_speed_pu.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<real_t>(group + "param mode_hysteresis_factor",
        [&]() { return operation_mode_monitor.get_parameters().mode_hysteresis_factor.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<current_pu_t>(group + "param zero_current_threshold_pu",
        [&]() { return operation_mode_monitor.get_parameters().zero_current_threshold_pu.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<duration_t>(group + "param is_zero_on_delay_s",
        [&]() { return operation_mode_monitor.get_parameters().is_zero_on_delay_s.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<duration_t>(group + "param discontinuous_on_delay_s",
        [&]() { return operation_mode_monitor.get_parameters().discontinuous_on_delay_s.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<duration_t>(group + "param discontinuous_off_delay_s",
        [&]() { return operation_mode_monitor.get_parameters().discontinuous_off_delay_s.read(); }));
}

static inline void register_minimal_machine_side_controller_logger(
    yeet::simulator& simulator, sfc_machine_side_controller_runner& machine_side_controller)
{
    const std::string group{"machine_side_controller:"};
    simulator.register_sampler_entry(yeet::make_lambda_sampler<speed_pu_t>(
        group + "input machine_speed", [&]() { return machine_side_controller.get_inputs().machine_speed.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<current_pu_t>(group + "input dc_link_current",
        [&]() { return machine_side_controller.get_inputs().dc_link_current.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<real_t>(group + "output firing_angle",
        [&]() { return machine_side_controller.get_outputs().firing_angle.read().get_pu(); }));
}

static inline void register_complete_machine_side_controller_logger(
    yeet::simulator& simulator, sfc_machine_side_controller_runner& machine_side_controller)
{
    const std::string group{"machine_side_controller:"};

    register_minimal_machine_side_controller_logger(simulator, machine_side_controller);

    // Add additional outputs if available in the complete datamodel
}

static inline void register_minimal_machine_side_integrator_supervisor_logger(
    yeet::simulator& simulator, sfc_machine_side_integrator_supervisor_runner& integrator_supervisor)
{
    const std::string group{"machine_side_integrator_supervisor:"};

    // Key inputs
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "input grid_side_freeze_integrator",
        [&]() { return integrator_supervisor.get_inputs().grid_side_freeze_integrator.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "input grid_side_is_tripped",
        [&]() { return integrator_supervisor.get_inputs().grid_side_is_tripped.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "input grid_side_release_pulses",
        [&]() { return integrator_supervisor.get_inputs().grid_side_release_pulses.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "input current_is_zero", [&]() { return integrator_supervisor.get_inputs().current_is_zero.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "input speed_saturated", [&]() { return integrator_supervisor.get_inputs().speed_saturated.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "input torque_saturated",
        [&]() { return integrator_supervisor.get_inputs().torque_saturated.read(); }));

    // Key outputs
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "output freeze_integrator",
        [&]() { return integrator_supervisor.get_outputs().freeze_integrator.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "output preset_request", [&]() { return integrator_supervisor.get_outputs().preset_request.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "output release_pulses", [&]() { return integrator_supervisor.get_outputs().release_pulses.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "output is_tripped", [&]() { return integrator_supervisor.get_outputs().is_tripped.read(); }));
}

static inline void register_complete_machine_side_integrator_supervisor_logger(
    yeet::simulator& simulator, sfc_machine_side_integrator_supervisor_runner& integrator_supervisor)
{
    const std::string group{"machine_side_integrator_supervisor:"};

    register_minimal_machine_side_integrator_supervisor_logger(simulator, integrator_supervisor);

    // Add parameter if needed
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "param override_freeze",
        [&]() { return integrator_supervisor.get_parameters().override_freeze.read(); }));
}

// Grid-side integrator supervisor loggers
static inline void register_minimal_grid_side_integrator_supervisor_logger(
    yeet::simulator& simulator, sfc_grid_side_integrator_supervisor_runner& integrator_supervisor)
{
    const std::string group{"grid_side_integrator_supervisor:"};

    // Key inputs
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "input state_machine_release_pulses",
        [&]() { return integrator_supervisor.get_inputs().state_machine_release_pulses.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "input preset_request", [&]() { return integrator_supervisor.get_inputs().preset_request.read(); }));
    simulator.register_sampler_entry(
        yeet::make_lambda_sampler<bool>(group + "input forced_commutation_freeze_integrator",
            [&]() { return integrator_supervisor.get_inputs().forced_commutation_freeze_integrator.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "input shutdown_request",
        [&]() { return integrator_supervisor.get_inputs().shutdown_request.read(); }));

    // Key outputs
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "output freeze_integrator",
        [&]() { return integrator_supervisor.get_outputs().freeze_integrator.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "output preset_request", [&]() { return integrator_supervisor.get_outputs().preset_request.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "output release_pulses", [&]() { return integrator_supervisor.get_outputs().release_pulses.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "output is_tripped", [&]() { return integrator_supervisor.get_outputs().is_tripped.read(); }));
}

static inline void register_complete_grid_side_integrator_supervisor_logger(
    yeet::simulator& simulator, sfc_grid_side_integrator_supervisor_runner& integrator_supervisor)
{
    const std::string group{"grid_side_integrator_supervisor:"};

    register_minimal_grid_side_integrator_supervisor_logger(simulator, integrator_supervisor);

    // Add parameters if needed
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "param override_freeze",
        [&]() { return integrator_supervisor.get_parameters().override_freeze.read(); }));
}

static inline void register_minimal_grid_side_modulator_logger(
    yeet::simulator& simulator, sfc_grid_side_modulator_runner& grid_side_modulator)
{
    const std::string group{"grid_side_modulator:"};
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "input release_pulses", [&]() { return grid_side_modulator.get_inputs().release_pulses.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "input request_force_commutation",
        [&]() { return grid_side_modulator.get_inputs().request_force_commutation.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "input current_is_zero", [&]() { return grid_side_modulator.get_inputs().current_is_zero.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<real_t>(group + "output firing_angle",
        [&]() { return grid_side_modulator.get_outputs().firing_angle.read().get_pu(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "output release_pulses", [&]() { return grid_side_modulator.get_outputs().release_pulses.read(); }));
}

static inline void register_complete_grid_side_modulator_logger(
    yeet::simulator& simulator, sfc_grid_side_modulator_runner& grid_side_modulator)
{
    const std::string group{"grid_side_modulator:"};

    register_minimal_grid_side_modulator_logger(simulator, grid_side_modulator);

    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "output force_commutation_acknowledged",
        [&]() { return grid_side_modulator.get_outputs().force_commutation_acknowledged.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "output freeze_integrator",
        [&]() { return grid_side_modulator.get_outputs().freeze_integrator.read(); }));
}

static inline void register_minimal_machine_side_modulator_logger(
    yeet::simulator& simulator, sfc_machine_side_modulator_runner& machine_side_modulator)
{
    const std::string group{"machine_side_modulator:"};
    simulator.register_sampler_entry(yeet::make_lambda_sampler<real_t>(group + "output firing_angle",
        [&]() { return machine_side_modulator.get_outputs().firing_angle.read().get_pu(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<real_t>(group + "output current_rotor_sector", [&]() {
        return static_cast<real_t>(machine_side_modulator.get_outputs().current_rotor_sector.read()) / 6.0f;
    }));
}

static inline void register_complete_machine_side_modulator_logger(
    yeet::simulator& simulator, sfc_machine_side_modulator_runner& machine_side_modulator)
{
    const std::string group{"machine_side_modulator:"};

    register_minimal_machine_side_modulator_logger(simulator, machine_side_modulator);

    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "input force_commutation_acknowledged",
        [&]() { return machine_side_modulator.get_inputs().force_commutation_acknowledged.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "output request_force_commutation",
        [&]() { return machine_side_modulator.get_outputs().request_force_commutation.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "output release_pulses", [&]() { return machine_side_modulator.get_outputs().release_pulses.read(); }));
}

static inline void register_minimal_machine_side_observer_logger(
    yeet::simulator& simulator, sfc_machine_side_observer_runner& machine_side_observer)
{
    const std::string group{"machine_side_observer:"};
    simulator.register_sampler_entry(
        yeet::make_lambda_array_sampler<abc<voltage_pu_t>>(group + "input machine_voltage_pu",
            [&]() { return machine_side_observer.get_inputs().machine_voltage_pu.read(); }));
    simulator.register_sampler_entry(
        yeet::make_lambda_sampler<voltage_pu_t>(group + "output estimated_voltage_magnitude",
            [&]() { return machine_side_observer.get_outputs().estimated_voltage_magnitude.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<frequency_pu_t>(group + "output estimated_frequency_pu",
        [&]() { return machine_side_observer.get_outputs().estimated_frequency_pu.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<real_t>(group + "output estimated_position",
        [&]() { return machine_side_observer.get_outputs().estimated_position.read().get_pu(); }));
}

static inline void register_complete_machine_side_observer_logger(
    yeet::simulator& simulator, sfc_machine_side_observer_runner& machine_side_observer)
{
    const std::string group{"machine_side_observer:"};

    register_minimal_machine_side_observer_logger(simulator, machine_side_observer);

    simulator.register_sampler_entry(
        yeet::make_lambda_array_sampler<alphabeta<voltage_pu_t>>(group + "output filtered_voltage_alphabeta",
            [&]() { return machine_side_observer.get_outputs().filtered_voltage_alphabeta.read(); }));
}

static inline void register_minimal_grid_side_pulse_generator_logger(
    yeet::simulator& simulator, sfc_grid_side_pulse_generator_runner& grid_side_pulse_generator)
{
    const std::string group{"grid_side_pulse_generator:"};
    simulator.register_sampler_entry(yeet::make_lambda_sampler<real_t>(group + "input reference_angle",
        [&]() { return grid_side_pulse_generator.get_inputs().reference_angle.read().get_pu(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<real_t>(group + "input firing_angle",
        [&]() { return grid_side_pulse_generator.get_inputs().firing_angle.read().get_pu(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "input release_pulses",
        [&]() { return grid_side_pulse_generator.get_inputs().release_pulses.read(); }));
    simulator.register_sampler_entry(
        yeet::make_lambda_array_sampler<std::array<bool, 6>>(group + "output firing_pulses",
            [&]() { return grid_side_pulse_generator.get_outputs().firing_pulses.read().to_array(); }));
}

static inline void register_complete_grid_side_pulse_generator_logger(
    yeet::simulator& simulator, sfc_grid_side_pulse_generator_runner& grid_side_pulse_generator)
{
    const std::string group{"grid_side_pulse_generator:"};

    register_minimal_grid_side_pulse_generator_logger(simulator, grid_side_pulse_generator);

    simulator.register_sampler_entry(yeet::make_lambda_sampler<real_t>(group + "output actual_firing_angle",
        [&]() { return grid_side_pulse_generator.get_outputs().actual_firing_angle.read().get_pu(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "output release_pulses",
        [&]() { return grid_side_pulse_generator.get_outputs().release_pulses.read(); }));
}

static inline void register_minimal_machine_side_pulse_generator_logger(
    yeet::simulator& simulator, sfc_machine_side_pulse_generator_runner& machine_side_pulse_generator)
{
    const std::string group{"machine_side_pulse_generator:"};
    simulator.register_sampler_entry(yeet::make_lambda_sampler<real_t>(group + "input reference_angle",
        [&]() { return machine_side_pulse_generator.get_inputs().reference_angle.read().get_pu(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<real_t>(group + "input firing_angle",
        [&]() { return machine_side_pulse_generator.get_inputs().firing_angle.read().get_pu(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "input release_pulses",
        [&]() { return machine_side_pulse_generator.get_inputs().release_pulses.read(); }));
    simulator.register_sampler_entry(
        yeet::make_lambda_array_sampler<std::array<bool, 6>>(group + "output firing_pulses",
            [&]() { return machine_side_pulse_generator.get_outputs().firing_pulses.read().to_array(); }));
}

static inline void register_complete_machine_side_pulse_generator_logger(
    yeet::simulator& simulator, sfc_machine_side_pulse_generator_runner& machine_side_pulse_generator)
{
    const std::string group{"machine_side_pulse_generator:"};

    register_minimal_machine_side_pulse_generator_logger(simulator, machine_side_pulse_generator);

    simulator.register_sampler_entry(yeet::make_lambda_sampler<real_t>(group + "output actual_firing_angle",
        [&]() { return machine_side_pulse_generator.get_outputs().actual_firing_angle.read().get_pu(); }));
}

static inline void register_minimal_plant_model_logger(yeet::simulator& simulator, sfc_plant_model_runner& plant_model)
{
    const std::string group{"plant_model:"};
    simulator.register_sampler_entry(
        yeet::make_lambda_array_sampler<std::array<bool, 6>>(group + "input grid_side_firing_pulses",
            [&]() { return plant_model.get_inputs().grid_side_firing_pulses.read().to_array(); }));
    simulator.register_sampler_entry(
        yeet::make_lambda_array_sampler<std::array<bool, 6>>(group + "input machine_side_firing_pulses",
            [&]() { return plant_model.get_inputs().machine_side_firing_pulses.read().to_array(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<torque_pu_t>(
        group + "input load_torque_pu", [&]() { return plant_model.get_inputs().load_torque_pu.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<current_t>(
        group + "output dc_link_current", [&]() { return plant_model.get_outputs().dc_link_current.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<voltage_t>(
        group + "output dc_link_voltage", [&]() { return plant_model.get_outputs().dc_link_voltage.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<speed_pu_t>(
        group + "output machine_speed", [&]() { return plant_model.get_outputs().machine_speed.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<frequency_t>(
        group + "output machine_frequency", [&]() { return plant_model.get_outputs().machine_frequency.read(); }));
}

static inline void register_complete_plant_model_logger(yeet::simulator& simulator, sfc_plant_model_runner& plant_model)
{
    const std::string group{"plant_model:"};

    register_minimal_plant_model_logger(simulator, plant_model);

    simulator.register_sampler_entry(yeet::make_lambda_array_sampler<abc<voltage_t>>(
        group + "output grid_voltage", [&]() { return plant_model.get_outputs().grid_voltage.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_array_sampler<abc<current_t>>(
        group + "output grid_current", [&]() { return plant_model.get_outputs().grid_current.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_array_sampler<abc<voltage_t>>(
        group + "output machine_voltage", [&]() { return plant_model.get_outputs().machine_voltage.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_array_sampler<abc<current_t>>(
        group + "output machine_current", [&]() { return plant_model.get_outputs().machine_current.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<real_t>(group + "output grid_voltage_source_angle",
        [&]() { return plant_model.get_outputs().grid_voltage_source_angle.read().get_pu(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<real_t>(group + "output machine_voltage_source_angle",
        [&]() { return plant_model.get_outputs().machine_voltage_source_angle.read().get_pu(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<frequency_t>(
        group + "output grid_frequency", [&]() { return plant_model.get_outputs().grid_frequency.read(); }));
}

// ==================== Hardware Mock Loggers ====================
// Grid-Side Hardware Mock
static inline void register_minimal_grid_side_hardware_mock_logger(
    yeet::simulator& simulator, sfc_grid_side_hardware_mock_runner& hardware_mock)
{
    const std::string group{"grid_side_hardware_mock:"};
    simulator.register_sampler_entry(yeet::make_lambda_array_sampler<abc<voltage_t>>(
        group + "input grid_voltage", [&]() { return hardware_mock.get_inputs().grid_voltage.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_array_sampler<abc<current_t>>(
        group + "input grid_current", [&]() { return hardware_mock.get_inputs().grid_current.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<int>(
        group + "output v_ab_counts", [&]() { return hardware_mock.get_outputs().v_ab_counts.read().value; }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<int>(
        group + "output v_bc_counts", [&]() { return hardware_mock.get_outputs().v_bc_counts.read().value; }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<int>(
        group + "output i_a_counts", [&]() { return hardware_mock.get_outputs().i_a_counts.read().value; }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<int>(
        group + "output i_c_counts", [&]() { return hardware_mock.get_outputs().i_c_counts.read().value; }));
}

static inline void register_complete_grid_side_hardware_mock_logger(
    yeet::simulator& simulator, sfc_grid_side_hardware_mock_runner& hardware_mock)
{
    // For now complete == minimal (no extra internal states exposed)
    register_minimal_grid_side_hardware_mock_logger(simulator, hardware_mock);
}

// Machine-Side Hardware Mock
static inline void register_minimal_machine_side_hardware_mock_logger(
    yeet::simulator& simulator, sfc_machine_side_hardware_mock_runner& hardware_mock)
{
    const std::string group{"machine_side_hardware_mock:"};
    simulator.register_sampler_entry(yeet::make_lambda_array_sampler<abc<voltage_t>>(
        group + "input machine_voltage", [&]() { return hardware_mock.get_inputs().machine_voltage.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<int>(
        group + "output v_ab_counts", [&]() { return hardware_mock.get_outputs().v_ab_counts.read().value; }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<int>(
        group + "output v_bc_counts", [&]() { return hardware_mock.get_outputs().v_bc_counts.read().value; }));
}

static inline void register_complete_machine_side_hardware_mock_logger(
    yeet::simulator& simulator, sfc_machine_side_hardware_mock_runner& hardware_mock)
{
    // For now complete == minimal
    register_minimal_machine_side_hardware_mock_logger(simulator, hardware_mock);
}

// Generic overloaded functions for easier logger registration

static inline void register_minimal_logger(yeet::simulator& simulator, sfc_current_controller_runner& runner)
{
    register_minimal_current_controller_logger(simulator, runner);
}

static inline void register_complete_logger(yeet::simulator& simulator, sfc_current_controller_runner& runner)
{
    register_complete_current_controller_logger(simulator, runner);
}

static inline void register_minimal_logger(yeet::simulator& simulator, sfc_speed_controller_runner& runner)
{
    register_minimal_speed_controller_logger(simulator, runner);
}

static inline void register_complete_logger(yeet::simulator& simulator, sfc_speed_controller_runner& runner)
{
    register_complete_speed_controller_logger(simulator, runner);
}

static inline void register_minimal_logger(yeet::simulator& simulator, sfc_excitation_controller_runner& runner)
{
    register_minimal_excitation_controller_logger(simulator, runner);
}

static inline void register_complete_logger(yeet::simulator& simulator, sfc_excitation_controller_runner& runner)
{
    register_complete_excitation_controller_logger(simulator, runner);
}

static inline void register_minimal_logger(yeet::simulator& simulator, sfc_soft_starter_speed_generator_runner& runner)
{
    register_minimal_sfc_soft_starter_speed_generator_logger(simulator, runner);
}

static inline void register_complete_logger(yeet::simulator& simulator, sfc_soft_starter_speed_generator_runner& runner)
{
    register_complete_sfc_soft_starter_speed_generator_logger(simulator, runner);
}

static inline void register_minimal_logger(yeet::simulator& simulator, sfc_torque_controller_runner& runner)
{
    register_minimal_sfc_torque_controller_logger(simulator, runner);
}

static inline void register_complete_logger(yeet::simulator& simulator, sfc_torque_controller_runner& runner)
{
    register_complete_sfc_torque_controller_logger(simulator, runner);
}

static inline void register_minimal_logger(yeet::simulator& simulator, sfc_dc_link_measurements_runner& runner)
{
    register_minimal_dc_link_measurements_logger(simulator, runner);
}

static inline void register_complete_logger(yeet::simulator& simulator, sfc_dc_link_measurements_runner& runner)
{
    register_complete_dc_link_measurements_logger(simulator, runner);
}

static inline void register_minimal_logger(yeet::simulator& simulator, sfc_grid_side_measurements_ideal_runner& runner)
{
    register_minimal_grid_side_measurements_logger(simulator, runner);
}

static inline void register_complete_logger(yeet::simulator& simulator, sfc_grid_side_measurements_ideal_runner& runner)
{
    register_complete_grid_side_measurements_logger(simulator, runner);
}

static inline void register_minimal_logger(yeet::simulator& simulator, sfc_grid_side_measurements_runner& runner)
{
    register_minimal_grid_side_measurements_logger(simulator, runner);
}

static inline void register_complete_logger(yeet::simulator& simulator, sfc_grid_side_measurements_runner& runner)
{
    register_complete_grid_side_measurements_logger(simulator, runner);
}

static inline void register_minimal_logger(
    yeet::simulator& simulator, sfc_machine_side_measurements_ideal_runner& runner)
{
    register_minimal_machine_side_measurements_logger(simulator, runner);
}

static inline void register_complete_logger(
    yeet::simulator& simulator, sfc_machine_side_measurements_ideal_runner& runner)
{
    register_complete_machine_side_measurements_logger(simulator, runner);
}

static inline void register_minimal_logger(yeet::simulator& simulator, sfc_machine_side_measurements_runner& runner)
{
    register_minimal_machine_side_measurements_logger(simulator, runner);
}

static inline void register_complete_logger(yeet::simulator& simulator, sfc_machine_side_measurements_runner& runner)
{
    register_complete_machine_side_measurements_logger(simulator, runner);
}

static inline void register_minimal_logger(yeet::simulator& simulator, sfc_grid_side_observer_runner& runner)
{
    register_minimal_grid_side_observer_logger(simulator, runner);
}

static inline void register_complete_logger(yeet::simulator& simulator, sfc_grid_side_observer_runner& runner)
{
    register_complete_grid_side_observer_logger(simulator, runner);
}

static inline void register_minimal_logger(yeet::simulator& simulator, sfc_machine_side_observer_runner& runner)
{
    register_minimal_machine_side_observer_logger(simulator, runner);
}

static inline void register_complete_logger(yeet::simulator& simulator, sfc_machine_side_observer_runner& runner)
{
    register_complete_machine_side_observer_logger(simulator, runner);
}

static inline void register_minimal_logger(yeet::simulator& simulator, sfc_state_machine_runner& runner)
{
    register_minimal_state_machine_logger(simulator, runner);
}

static inline void register_complete_logger(yeet::simulator& simulator, sfc_state_machine_runner& runner)
{
    register_complete_state_machine_logger(simulator, runner);
}

static inline void register_minimal_logger(yeet::simulator& simulator, sfc_machine_side_controller_runner& runner)
{
    register_minimal_machine_side_controller_logger(simulator, runner);
}

static inline void register_complete_logger(yeet::simulator& simulator, sfc_machine_side_controller_runner& runner)
{
    register_complete_machine_side_controller_logger(simulator, runner);
}

static inline void register_minimal_logger(yeet::simulator& simulator, sfc_grid_side_modulator_runner& runner)
{
    register_minimal_grid_side_modulator_logger(simulator, runner);
}

static inline void register_complete_logger(yeet::simulator& simulator, sfc_grid_side_modulator_runner& runner)
{
    register_complete_grid_side_modulator_logger(simulator, runner);
}

static inline void register_minimal_logger(yeet::simulator& simulator, sfc_machine_side_modulator_runner& runner)
{
    register_minimal_machine_side_modulator_logger(simulator, runner);
}

static inline void register_complete_logger(yeet::simulator& simulator, sfc_machine_side_modulator_runner& runner)
{
    register_complete_machine_side_modulator_logger(simulator, runner);
}

static inline void register_minimal_logger(yeet::simulator& simulator, sfc_grid_side_pulse_generator_runner& runner)
{
    register_minimal_grid_side_pulse_generator_logger(simulator, runner);
}

static inline void register_complete_logger(yeet::simulator& simulator, sfc_grid_side_pulse_generator_runner& runner)
{
    register_complete_grid_side_pulse_generator_logger(simulator, runner);
}

static inline void register_minimal_logger(yeet::simulator& simulator, sfc_machine_side_pulse_generator_runner& runner)
{
    register_minimal_machine_side_pulse_generator_logger(simulator, runner);
}

static inline void register_complete_logger(yeet::simulator& simulator, sfc_machine_side_pulse_generator_runner& runner)
{
    register_complete_machine_side_pulse_generator_logger(simulator, runner);
}

static inline void register_minimal_logger(yeet::simulator& simulator, sfc_plant_model_runner& runner)
{
    register_minimal_plant_model_logger(simulator, runner);
}

static inline void register_complete_logger(yeet::simulator& simulator, sfc_plant_model_runner& runner)
{
    register_complete_plant_model_logger(simulator, runner);
}

static inline void register_minimal_logger(yeet::simulator& simulator, sfc_operation_mode_monitor_runner& runner)
{
    register_minimal_operation_mode_monitor_logger(simulator, runner);
}

static inline void register_complete_logger(yeet::simulator& simulator, sfc_operation_mode_monitor_runner& runner)
{
    register_complete_operation_mode_monitor_logger(simulator, runner);
}

// Machine-side integrator supervisor overloads
static inline void register_minimal_logger(
    yeet::simulator& simulator, sfc_machine_side_integrator_supervisor_runner& runner)
{
    register_minimal_machine_side_integrator_supervisor_logger(simulator, runner);
}

static inline void register_complete_logger(
    yeet::simulator& simulator, sfc_machine_side_integrator_supervisor_runner& runner)
{
    register_complete_machine_side_integrator_supervisor_logger(simulator, runner);
}

// Hardware mock overloads
static inline void register_minimal_logger(yeet::simulator& simulator, sfc_grid_side_hardware_mock_runner& runner)
{
    register_minimal_grid_side_hardware_mock_logger(simulator, runner);
}

static inline void register_complete_logger(yeet::simulator& simulator, sfc_grid_side_hardware_mock_runner& runner)
{
    register_complete_grid_side_hardware_mock_logger(simulator, runner);
}

static inline void register_minimal_logger(yeet::simulator& simulator, sfc_machine_side_hardware_mock_runner& runner)
{
    register_minimal_machine_side_hardware_mock_logger(simulator, runner);
}

static inline void register_complete_logger(yeet::simulator& simulator, sfc_machine_side_hardware_mock_runner& runner)
{
    register_complete_machine_side_hardware_mock_logger(simulator, runner);
}

}  // namespace yeet::application::sfc::integration
