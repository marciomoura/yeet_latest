#pragma once

#include <string>

#include "sfc/sfc_soft_starter_speed_generator.hpp"  // For speed_command_t and acceleration_profile_t
#include "sfc_integration/sfc_current_controller_runner.hpp"
#include "sfc_integration/sfc_dc_link_measurements_runner.hpp"
#include "sfc_integration/sfc_excitation_controller_runner.hpp"
#include "sfc_integration/sfc_grid_io_mock_runner.hpp"
#include "sfc_integration/sfc_grid_side_hardware_mock_runner.hpp"
#include "sfc_integration/sfc_grid_side_integrator_supervisor_runner.hpp"
#include "sfc_integration/sfc_grid_side_measurements_runner.hpp"
#include "sfc_integration/sfc_grid_side_modulator_runner.hpp"
#include "sfc_integration/sfc_grid_side_observer_runner.hpp"
#include "sfc_integration/sfc_grid_side_protections_runner.hpp"
#include "sfc_integration/sfc_machine_io_mock_runner.hpp"
#include "sfc_integration/sfc_machine_side_controller_runner.hpp"
#include "sfc_integration/sfc_machine_side_hardware_mock_runner.hpp"
#include "sfc_integration/sfc_machine_side_integrator_supervisor_runner.hpp"
#include "sfc_integration/sfc_machine_side_measurements_runner.hpp"
#include "sfc_integration/sfc_machine_side_modulator_runner.hpp"
#include "sfc_integration/sfc_machine_side_observer_runner.hpp"
#include "sfc_integration/sfc_machine_side_protections_runner.hpp"
#include "sfc_integration/sfc_operation_mode_monitor_runner.hpp"
#include "sfc_integration/sfc_plant_model_runner.hpp"
#include "sfc_integration/sfc_soft_starter_speed_generator_runner.hpp"
#include "sfc_integration/sfc_speed_controller_runner.hpp"
#include "sfc_integration/sfc_startup_sequencer_runner.hpp"
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

    simulator.register_sampler_entry(yeet::make_lambda_sampler<voltage_pu_t>(group + "input grid_voltage_magnitude",
        [&]() { return current_controller.get_inputs().grid_voltage_magnitude.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<voltage_pu_t>(group + "input machine_voltage_magnitude",
        [&]() { return current_controller.get_inputs().machine_voltage_magnitude.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "output is_active", [&]() { return current_controller.get_outputs().is_active.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<real_t>(group + "output firing_angle",
        [&]() { return current_controller.get_outputs().firing_angle.read().get_pu(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<real_t>(
        group + "output pi_output", [&]() { return current_controller.get_outputs().pi_output.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "output is_saturated", [&]() { return current_controller.get_outputs().is_saturated.read(); }));
}

static inline void register_minimal_speed_controller_logger(
    yeet::simulator& simulator, sfc_speed_controller_runner& speed_controller)
{
    const std::string group{"speed_controller:"};
    simulator.register_sampler_entry(yeet::make_lambda_sampler<speed_pu_t>(
        group + "input speed_reference", [&]() { return speed_controller.get_inputs().speed_reference.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<speed_pu_t>(
        group + "input measured_speed", [&]() { return speed_controller.get_inputs().measured_speed.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "input freeze_integrator", [&]() { return speed_controller.get_inputs().freeze_integrator.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<speed_pu_t>(group + "calculated speed_error", [&]() {
        return speed_controller.get_inputs().speed_reference.read() -
               speed_controller.get_inputs().measured_speed.read();
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
    simulator.register_sampler_entry(yeet::make_lambda_sampler<torque_pu_t>(group + "input applied_torque_reference",
        [&]() { return speed_controller.get_inputs().applied_torque_reference.read(); }));
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
    simulator.register_sampler_entry(
        yeet::make_lambda_sampler<voltage_pu_t>(group + "input machine_voltage_magnitude_pu",
            [&]() { return excitation_controller.get_inputs().machine_voltage_magnitude_pu.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "input release_excitation",
        [&]() { return excitation_controller.get_inputs().release_excitation.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<current_pu_t>(group + "output field_current_reference",
        [&]() { return excitation_controller.get_outputs().field_current_reference.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<duration_t>(group + "output prefluxing_elapsed_time",
        [&]() { return excitation_controller.get_outputs().prefluxing_elapsed_time.read(); }));
}

static inline void register_complete_excitation_controller_logger(
    yeet::simulator& simulator, sfc_excitation_controller_runner& excitation_controller)
{
    const std::string group{"excitation_controller:"};

    register_minimal_excitation_controller_logger(simulator, excitation_controller);

    simulator.register_sampler_entry(
        yeet::make_lambda_sampler<voltage_pu_t>(group + "output machine_voltage_reference_pu",
            [&]() { return excitation_controller.get_outputs().machine_voltage_reference_pu.read(); }));
    simulator.register_sampler_entry(
        yeet::make_lambda_sampler<voltage_pu_t>(group + "output machine_voltage_magnitude_pu",
            [&]() { return excitation_controller.get_outputs().machine_voltage_magnitude_pu.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<voltage_pu_t>(group + "output machine_voltage_error_pu",
        [&]() { return excitation_controller.get_outputs().machine_voltage_error_pu.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "output is_field_established",
        [&]() { return excitation_controller.get_outputs().is_field_established.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "output excitation_start_command",
        [&]() { return excitation_controller.get_outputs().excitation_start_command.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "output excitation_stop_command",
        [&]() { return excitation_controller.get_outputs().excitation_stop_command.read(); }));
}

// --- Soft-starter speed generator runner loggers ---
static inline void register_minimal_sfc_soft_starter_speed_generator_logger(
    yeet::simulator& simulator, sfc_soft_starter_speed_generator_runner& speed_generator)
{
    const std::string group{"speed_generator:"};

    // Key outputs
    simulator.register_sampler_entry(yeet::make_lambda_sampler<speed_pu_t>(
        group + "output speed_reference", [&]() { return speed_generator.get_outputs().speed_reference.read(); }));
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
        group + "input measured_speed", [&]() { return speed_generator.get_inputs().measured_speed.read(); }));
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
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "input enable_breakaway_boost",
        [&]() { return torque_controller.get_inputs().enable_breakaway_boost.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "input is_saturated", [&]() { return torque_controller.get_inputs().is_saturated.read(); }));

    // Additional outputs
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "output is_saturated", [&]() { return torque_controller.get_outputs().is_saturated.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "output is_active", [&]() { return torque_controller.get_outputs().is_active.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "output is_braking_requested",
        [&]() { return torque_controller.get_outputs().is_braking_requested.read(); }));
}

static inline void register_minimal_dc_link_measurements_logger(
    yeet::simulator& simulator, sfc_dc_link_measurements_runner& dc_link_measurements)
{
    const std::string group{"dc_link_measurements:"};
    simulator.register_sampler_entry(yeet::make_lambda_array_sampler<abc<current_pu_t>>(
        group + "input grid_current_abc", [&]() { return dc_link_measurements.get_inputs().grid_current_abc.read(); }));

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

// --- Grid-side protections runner loggers ---
static inline void register_minimal_grid_side_protections_logger(
    yeet::simulator& simulator, sfc_grid_side_protections_runner& grid_protections)
{
    const std::string group{"grid_protections:"};

    // Key inputs
    simulator.register_sampler_entry(yeet::make_lambda_sampler<current_pu_t>(
        group + "input dc_link_current_pu", [&]() { return grid_protections.get_inputs().dc_link_current_pu.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<voltage_pu_t>(group + "input grid_voltage_magnitude_pu",
        [&]() { return grid_protections.get_inputs().grid_voltage_magnitude_pu.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "input release_protections",
        [&]() { return grid_protections.get_inputs().release_protections.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "input door_is_open", [&]() { return grid_protections.get_inputs().door_is_open.read(); }));

    // Key parameters
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "param confirm_trips", [&]() { return grid_protections.get_parameters().confirm_trips.read(); }));

    // Key outputs
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "output has_active_trip", [&]() { return grid_protections.get_outputs().has_active_trip.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "output is_inhibited", [&]() { return grid_protections.get_outputs().is_inhibited.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "output has_active_alarm", [&]() { return grid_protections.get_outputs().has_active_alarm.read(); }));
}

static inline void register_complete_grid_side_protections_logger(
    yeet::simulator& simulator, sfc_grid_side_protections_runner& grid_protections)
{
    const std::string group{"grid_protections:"};

    register_minimal_grid_side_protections_logger(simulator, grid_protections);

    // All trip outputs
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "output overcurrent_tripped",
        [&]() { return grid_protections.get_outputs().overcurrent_tripped.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "output overvoltage_tripped",
        [&]() { return grid_protections.get_outputs().overvoltage_tripped.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "output undervoltage_tripped",
        [&]() { return grid_protections.get_outputs().undervoltage_tripped.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "output communication_loss_tripped",
        [&]() { return grid_protections.get_outputs().communication_loss_tripped.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "output door_open_tripped", [&]() { return grid_protections.get_outputs().door_open_tripped.read(); }));

    // All alarm outputs
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "output overcurrent_alarmed",
        [&]() { return grid_protections.get_outputs().overcurrent_alarmed.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "output overvoltage_alarmed",
        [&]() { return grid_protections.get_outputs().overvoltage_alarmed.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "output undervoltage_alarmed",
        [&]() { return grid_protections.get_outputs().undervoltage_alarmed.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "output communication_loss_alarmed",
        [&]() { return grid_protections.get_outputs().communication_loss_alarmed.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "output door_open_alarmed", [&]() { return grid_protections.get_outputs().door_open_alarmed.read(); }));

    // All warning outputs
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "output overcurrent_warning",
        [&]() { return grid_protections.get_outputs().overcurrent_warning.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "output overvoltage_warning",
        [&]() { return grid_protections.get_outputs().overvoltage_warning.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "output undervoltage_warning",
        [&]() { return grid_protections.get_outputs().undervoltage_warning.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "output communication_loss_warning",
        [&]() { return grid_protections.get_outputs().communication_loss_warning.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "output undervoltage_detected",
        [&]() { return grid_protections.get_outputs().undervoltage_detected.read(); }));
}

// --- Machine-side protections runner loggers ---
static inline void register_minimal_machine_side_protections_logger(
    yeet::simulator& simulator, sfc_machine_side_protections_runner& machine_protections)
{
    const std::string group{"machine_protections:"};

    // Key inputs
    simulator.register_sampler_entry(yeet::make_lambda_sampler<current_pu_t>(group + "input dc_link_current_pu",
        [&]() { return machine_protections.get_inputs().dc_link_current_pu.read(); }));
    simulator.register_sampler_entry(
        yeet::make_lambda_sampler<voltage_pu_t>(group + "input machine_voltage_magnitude_pu",
            [&]() { return machine_protections.get_inputs().machine_voltage_magnitude_pu.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<frequency_pu_t>(group + "input machine_frequency_pu",
        [&]() { return machine_protections.get_inputs().machine_frequency_pu.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<speed_pu_t>(
        group + "input machine_speed", [&]() { return machine_protections.get_inputs().machine_speed.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "input startup_active", [&]() { return machine_protections.get_inputs().startup_active.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "input closed_loop_active",
        [&]() { return machine_protections.get_inputs().closed_loop_active.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "input excitation_is_on", [&]() { return machine_protections.get_inputs().excitation_is_on.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "input release_protections",
        [&]() { return machine_protections.get_inputs().release_protections.read(); }));

    // Key parameters
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "param confirm_trips", [&]() { return machine_protections.get_parameters().confirm_trips.read(); }));

    // Key outputs
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "output has_active_trip", [&]() { return machine_protections.get_outputs().has_active_trip.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "output is_inhibited", [&]() { return machine_protections.get_outputs().is_inhibited.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "output has_active_alarm",
        [&]() { return machine_protections.get_outputs().has_active_alarm.read(); }));
}

static inline void register_complete_machine_side_protections_logger(
    yeet::simulator& simulator, sfc_machine_side_protections_runner& machine_protections)
{
    const std::string group{"machine_protections:"};

    register_minimal_machine_side_protections_logger(simulator, machine_protections);

    // All trip/alarm outputs
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "output over_flux_tripped",
        [&]() { return machine_protections.get_outputs().over_flux_tripped.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "output under_flux_tripped",
        [&]() { return machine_protections.get_outputs().under_flux_tripped.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "output stall_tripped", [&]() { return machine_protections.get_outputs().stall_tripped.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "output overspeed_tripped",
        [&]() { return machine_protections.get_outputs().overspeed_tripped.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "output machine_overvoltage_tripped",
        [&]() { return machine_protections.get_outputs().machine_overvoltage_tripped.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "output communication_loss_tripped",
        [&]() { return machine_protections.get_outputs().communication_loss_tripped.read(); }));

    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "output over_flux_alarmed",
        [&]() { return machine_protections.get_outputs().over_flux_alarmed.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "output under_flux_alarmed",
        [&]() { return machine_protections.get_outputs().under_flux_alarmed.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "output stall_alarmed", [&]() { return machine_protections.get_outputs().stall_alarmed.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "output overspeed_alarmed",
        [&]() { return machine_protections.get_outputs().overspeed_alarmed.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "output machine_overvoltage_alarmed",
        [&]() { return machine_protections.get_outputs().machine_overvoltage_alarmed.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "output communication_loss_alarmed",
        [&]() { return machine_protections.get_outputs().communication_loss_alarmed.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "output loss_of_excitation_tripped",
        [&]() { return machine_protections.get_outputs().loss_of_excitation_tripped.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "output loss_of_excitation_alarmed",
        [&]() { return machine_protections.get_outputs().loss_of_excitation_alarmed.read(); }));

    // All warning outputs
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "output over_flux_warning",
        [&]() { return machine_protections.get_outputs().over_flux_warning.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "output under_flux_warning",
        [&]() { return machine_protections.get_outputs().under_flux_warning.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "output stall_warning", [&]() { return machine_protections.get_outputs().stall_warning.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "output communication_loss_warning",
        [&]() { return machine_protections.get_outputs().communication_loss_warning.read(); }));
}

static inline void register_minimal_grid_side_measurements_logger(
    yeet::simulator& simulator, sfc_grid_side_measurements_ideal_runner& grid_side_measurements)
{
    const std::string group{"grid_side_measurements:"};
    simulator.register_sampler_entry(
        yeet::make_lambda_array_sampler<abc<voltage_pu_t>>(group + "output grid_voltage_abc",
            [&]() { return grid_side_measurements.get_outputs().grid_voltage_abc.read(); }));
    simulator.register_sampler_entry(
        yeet::make_lambda_array_sampler<abc<current_pu_t>>(group + "output grid_current_abc",
            [&]() { return grid_side_measurements.get_outputs().grid_current_abc.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<voltage_pu_t>(group + "output grid_voltage_magnitude",
        [&]() { return grid_side_measurements.get_outputs().grid_voltage_magnitude.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "output is_commuting", [&]() { return grid_side_measurements.get_outputs().is_commuting.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "output is_masking", [&]() { return grid_side_measurements.get_outputs().is_masking.read(); }));
}

static inline void register_complete_grid_side_measurements_logger(
    yeet::simulator& simulator, sfc_grid_side_measurements_ideal_runner& grid_side_measurements)
{
    const std::string group{"grid_side_measurements:"};

    register_minimal_grid_side_measurements_logger(simulator, grid_side_measurements);

    simulator.register_sampler_entry(yeet::make_lambda_sampler<frequency_pu_t>(
        group + "output grid_frequency", [&]() { return grid_side_measurements.get_outputs().grid_frequency.read(); }));
    simulator.register_sampler_entry(
        yeet::make_lambda_array_sampler<alphabeta<voltage_pu_t>>(group + "output grid_voltage_alphabeta",
            [&]() { return grid_side_measurements.get_outputs().grid_voltage_alphabeta.read(); }));
    simulator.register_sampler_entry(
        yeet::make_lambda_array_sampler<alphabeta<voltage_pu_t>>(group + "output grid_voltage_alphabeta_filtered",
            [&]() { return grid_side_measurements.get_outputs().grid_voltage_alphabeta_filtered.read(); }));
    // raw grid_voltage_abc already in minimal logger
    simulator.register_sampler_entry(
        yeet::make_lambda_array_sampler<abc<voltage_pu_t>>(group + "output grid_voltage_abc_raw",
            [&]() { return grid_side_measurements.get_outputs().grid_voltage_abc_raw.read(); }));
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
        yeet::make_lambda_array_sampler<abc<current_pu_t>>(group + "output grid_current_abc",
            [&]() { return grid_side_measurements.get_outputs().grid_current_abc.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<voltage_pu_t>(group + "output grid_voltage_magnitude",
        [&]() { return grid_side_measurements.get_outputs().grid_voltage_magnitude.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "output is_commuting", [&]() { return grid_side_measurements.get_outputs().is_commuting.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "output is_masking", [&]() { return grid_side_measurements.get_outputs().is_masking.read(); }));
}

static inline void register_complete_grid_side_measurements_logger(
    yeet::simulator& simulator, sfc_grid_side_measurements_runner& grid_side_measurements)
{
    const std::string group{"grid_side_measurements:"};

    register_minimal_grid_side_measurements_logger(simulator, grid_side_measurements);

    simulator.register_sampler_entry(yeet::make_lambda_sampler<frequency_pu_t>(
        group + "output grid_frequency", [&]() { return grid_side_measurements.get_outputs().grid_frequency.read(); }));
    simulator.register_sampler_entry(
        yeet::make_lambda_array_sampler<alphabeta<voltage_pu_t>>(group + "output grid_voltage_alphabeta",
            [&]() { return grid_side_measurements.get_outputs().grid_voltage_alphabeta.read(); }));
    simulator.register_sampler_entry(
        yeet::make_lambda_array_sampler<alphabeta<voltage_pu_t>>(group + "output grid_voltage_alphabeta_filtered",
            [&]() { return grid_side_measurements.get_outputs().grid_voltage_alphabeta_filtered.read(); }));
    // raw grid_voltage_abc already in minimal logger
    simulator.register_sampler_entry(
        yeet::make_lambda_array_sampler<abc<voltage_pu_t>>(group + "output grid_voltage_abc_raw",
            [&]() { return grid_side_measurements.get_outputs().grid_voltage_abc_raw.read(); }));
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
    simulator.register_sampler_entry(yeet::make_lambda_sampler<speed_pu_t>(group + "output machine_speed",
        [&]() { return machine_side_measurements.get_outputs().machine_speed.read(); }));
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
        yeet::make_lambda_array_sampler<abc<voltage_pu_t>>(group + "output machine_voltage_abc_raw",
            [&]() { return machine_side_measurements.get_outputs().machine_voltage_abc_raw.read(); }));
    simulator.register_sampler_entry(
        yeet::make_lambda_array_sampler<abc<voltage_pu_t>>(group + "output machine_voltage_abc_filtered",
            [&]() { return machine_side_measurements.get_outputs().machine_voltage_abc_filtered.read(); }));
    // raw machine_current_abc already in minimal logger
    simulator.register_sampler_entry(
        yeet::make_lambda_sampler<voltage_pu_t>(group + "output machine_voltage_magnitude_filtered",
            [&]() { return machine_side_measurements.get_outputs().machine_voltage_magnitude_filtered.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "output is_commuting", [&]() { return machine_side_measurements.get_outputs().is_commuting.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "output is_masking", [&]() { return machine_side_measurements.get_outputs().is_masking.read(); }));
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
    simulator.register_sampler_entry(yeet::make_lambda_sampler<speed_pu_t>(group + "output machine_speed",
        [&]() { return machine_side_measurements.get_outputs().machine_speed.read(); }));
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
        yeet::make_lambda_array_sampler<abc<voltage_pu_t>>(group + "output machine_voltage_abc_raw",
            [&]() { return machine_side_measurements.get_outputs().machine_voltage_abc_raw.read(); }));
    simulator.register_sampler_entry(
        yeet::make_lambda_array_sampler<abc<voltage_pu_t>>(group + "output machine_voltage_abc_filtered",
            [&]() { return machine_side_measurements.get_outputs().machine_voltage_abc_filtered.read(); }));
    simulator.register_sampler_entry(
        yeet::make_lambda_sampler<voltage_pu_t>(group + "output machine_voltage_magnitude_filtered",
            [&]() { return machine_side_measurements.get_outputs().machine_voltage_magnitude_filtered.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "output is_commuting", [&]() { return machine_side_measurements.get_outputs().is_commuting.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "output is_masking", [&]() { return machine_side_measurements.get_outputs().is_masking.read(); }));
}

static inline void register_minimal_grid_side_observer_logger(
    yeet::simulator& simulator, sfc_grid_side_observer_runner& grid_side_observer)
{
    const std::string group{"grid_side_observer:"};
    simulator.register_sampler_entry(yeet::make_lambda_array_sampler<abc<voltage_pu_t>>(
        group + "input grid_voltage_abc", [&]() { return grid_side_observer.get_inputs().grid_voltage_abc.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<real_t>(group + "output estimated_angle",
        [&]() { return grid_side_observer.get_outputs().estimated_reference_angle.read().get_pu(); }));
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
        yeet::make_lambda_array_sampler<alphabeta<voltage_pu_t>>(group + "output estimated_voltage_alphabeta_filtered",
            [&]() { return grid_side_observer.get_outputs().estimated_voltage_alphabeta_filtered.read(); }));
}

static inline void register_minimal_state_machine_logger(
    yeet::simulator& simulator, sfc_state_machine_runner& state_machine)
{
    const std::string group{"state_machine:"};

    // Hardware inputs
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "input auxiliaries_are_on", [&]() { return state_machine.get_inputs().auxiliaries_are_on.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "input mv_breaker_is_closed",
        [&]() { return state_machine.get_inputs().mv_breaker_is_closed.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "input start_command", [&]() { return state_machine.get_inputs().start_command.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "input stop_command", [&]() { return state_machine.get_inputs().stop_command.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "input is_current_extinguished",
        [&]() { return state_machine.get_inputs().is_current_extinguished.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "input protection_inhibited",
        [&]() { return state_machine.get_inputs().protection_inhibited.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "input any_protection_fault_active",
        [&]() { return state_machine.get_inputs().any_protection_fault_active.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "input mv_breaker_is_tripping",
        [&]() { return state_machine.get_inputs().mv_breaker_is_tripping.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "input external_fault", [&]() { return state_machine.get_inputs().external_fault.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "input emergency_off", [&]() { return state_machine.get_inputs().emergency_off.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "input door_is_open", [&]() { return state_machine.get_inputs().door_is_open.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "input machine_side_protection_inhibited",
        [&]() { return state_machine.get_inputs().machine_side_protection_inhibited.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "input machine_side_protection_tripped",
        [&]() { return state_machine.get_inputs().machine_side_protection_tripped.read(); }));

    // Parameters
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "param turn_on_command", [&]() { return state_machine.get_parameters().turn_on_command.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "param start_command", [&]() { return state_machine.get_parameters().start_command.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "param stop_command", [&]() { return state_machine.get_parameters().stop_command.read(); }));

    // Key outputs
    simulator.register_sampler_entry(yeet::make_lambda_sampler<sfc_state_id>(
        group + "output current_state_id", [&]() { return state_machine.get_outputs().current_state_id.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "output release_pulses", [&]() { return state_machine.get_outputs().release_pulses.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "output preset_request", [&]() { return state_machine.get_outputs().preset_request.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "output request_extinguish_dc_current",
        [&]() { return state_machine.get_outputs().request_extinguish_dc_current.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "output release_grid_protections",
        [&]() { return state_machine.get_outputs().release_grid_protections.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "output release_machine_protections",
        [&]() { return state_machine.get_outputs().release_machine_protections.read(); }));
}

static inline void register_complete_state_machine_logger(
    yeet::simulator& simulator, sfc_state_machine_runner& state_machine)
{
    const std::string group{"state_machine:"};

    register_minimal_state_machine_logger(simulator, state_machine);

    // Log the combined/OR'd command signals (hardware + parameters/overrides) that are fed to the state machine
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "ored_command auxiliaries_are_on",
        [&]() { return state_machine.get_outputs().auxiliaries_are_on.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "ored_command mv_breaker_is_closed",
        [&]() { return state_machine.get_outputs().mv_breaker_is_closed.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "ored_command start_command", [&]() { return state_machine.get_outputs().start_command.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "ored_command stop_command", [&]() { return state_machine.get_outputs().stop_command.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "ored_command is_current_extinguished",
        [&]() { return state_machine.get_outputs().is_current_extinguished.read(); }));

    // Additional outputs
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "output has_transitioned", [&]() { return state_machine.get_outputs().has_transitioned.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "output mv_breaker_close", [&]() { return state_machine.get_outputs().mv_breaker_close.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "output mv_breaker_open", [&]() { return state_machine.get_outputs().mv_breaker_open.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "output auxiliaries_on_command",
        [&]() { return state_machine.get_outputs().auxiliaries_on_command.read(); }));
}

// --- Startup sequencer runner loggers ---
static inline void register_minimal_startup_sequencer_logger(
    yeet::simulator& simulator, sfc_startup_sequencer_runner& startup_sequencer)
{
    const std::string group{"startup_sequencer:"};

    // Key inputs
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "input is_field_established",
        [&]() { return startup_sequencer.get_inputs().is_field_established.read(); }));

    // State and mode outputs
    simulator.register_sampler_entry(yeet::make_lambda_sampler<startup_sequencer_state_t>(
        group + "output current_state", [&]() { return startup_sequencer.get_outputs().current_state.read(); }));
    simulator.register_sampler_entry(
        yeet::make_lambda_sampler<reference_frame_source_t>(group + "output reference_frame_source",
            [&]() { return startup_sequencer.get_outputs().reference_frame_source.read(); }));
    simulator.register_sampler_entry(
        yeet::make_lambda_sampler<sfc_torque_controller::control_mode>(group + "output torque_control_mode",
            [&]() { return startup_sequencer.get_outputs().torque_control_mode.read(); }));

    // Release signals
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "output release_machine_side_pulses",
        [&]() { return startup_sequencer.get_outputs().release_machine_side_pulses.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "output release_excitation",
        [&]() { return startup_sequencer.get_outputs().release_excitation.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "output release_machine_observer",
        [&]() { return startup_sequencer.get_outputs().release_machine_observer.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "output release_speed_controller",
        [&]() { return startup_sequencer.get_outputs().release_speed_controller.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "output release_torque_controller",
        [&]() { return startup_sequencer.get_outputs().release_torque_controller.read(); }));

    // Virtual reference outputs
    simulator.register_sampler_entry(yeet::make_lambda_sampler<real_t>(group + "output virtual_rotor_angle",
        [&]() { return startup_sequencer.get_outputs().virtual_rotor_angle.read().get_pu(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<frequency_pu_t>(group + "output virtual_rotor_frequency",
        [&]() { return startup_sequencer.get_outputs().virtual_rotor_frequency.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<torque_pu_t>(group + "output virtual_torque_reference",
        [&]() { return startup_sequencer.get_outputs().virtual_torque_reference.read(); }));

    // Speed control outputs
    simulator.register_sampler_entry(yeet::make_lambda_sampler<speed_pu_t>(
        group + "output speed_reference", [&]() { return startup_sequencer.get_outputs().speed_reference.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<speed_pu_t>(
        group + "input measured_speed", [&]() { return startup_sequencer.get_inputs().measured_speed.read(); }));

    // Status outputs
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "output is_startup_complete",
        [&]() { return startup_sequencer.get_outputs().is_startup_complete.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<duration_t>(group + "output time_in_current_state",
        [&]() { return startup_sequencer.get_outputs().time_in_current_state.read(); }));

    // Observer preset outputs
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "output preset_observer", [&]() { return startup_sequencer.get_outputs().preset_observer.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<real_t>(
        group + "output preset_angle", [&]() { return startup_sequencer.get_outputs().preset_angle.read().get_pu(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<frequency_pu_t>(
        group + "output preset_frequency", [&]() { return startup_sequencer.get_outputs().preset_frequency.read(); }));
}

static inline void register_complete_startup_sequencer_logger(
    yeet::simulator& simulator, sfc_startup_sequencer_runner& startup_sequencer)
{
    const std::string group{"startup_sequencer:"};

    register_minimal_startup_sequencer_logger(simulator, startup_sequencer);

    // Add estimated rotor angle input
    simulator.register_sampler_entry(yeet::make_lambda_sampler<real_t>(group + "input estimated_rotor_angle",
        [&]() { return startup_sequencer.get_inputs().estimated_rotor_angle.read().get_pu(); }));
}

static inline void register_minimal_operation_mode_monitor_logger(
    yeet::simulator& simulator, sfc_operation_mode_monitor_runner& operation_mode_monitor)
{
    const std::string group{"operation_mode_monitor:"};
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "input enable", [&]() { return operation_mode_monitor.get_inputs().enable.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<speed_pu_t>(
        group + "input measured_speed", [&]() { return operation_mode_monitor.get_inputs().measured_speed.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<current_pu_t>(
        group + "input dc_link_current", [&]() { return operation_mode_monitor.get_inputs().dc_link_current.read(); }));
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
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "input grid_side_release_pulses",
        [&]() { return integrator_supervisor.get_inputs().grid_side_release_pulses.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "input torque_saturated",
        [&]() { return integrator_supervisor.get_inputs().torque_saturated.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "input is_braking_requested",
        [&]() { return integrator_supervisor.get_inputs().is_braking_requested.read(); }));

    // Key outputs
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "output freeze_integrator",
        [&]() { return integrator_supervisor.get_outputs().freeze_integrator.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "output preset_request", [&]() { return integrator_supervisor.get_outputs().preset_request.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "output release_pulses", [&]() { return integrator_supervisor.get_outputs().release_pulses.read(); }));
}

static inline void register_complete_machine_side_integrator_supervisor_logger(
    yeet::simulator& simulator, sfc_machine_side_integrator_supervisor_runner& integrator_supervisor)
{
    const std::string group{"machine_side_integrator_supervisor:"};

    register_minimal_machine_side_integrator_supervisor_logger(simulator, integrator_supervisor);

    // Add parameter if needed
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "param freeze_override",
        [&]() { return integrator_supervisor.get_parameters().freeze_override.read(); }));
}

// Grid-side integrator supervisor loggers
static inline void register_minimal_grid_side_integrator_supervisor_logger(
    yeet::simulator& simulator, sfc_grid_side_integrator_supervisor_runner& integrator_supervisor)
{
    const std::string group{"grid_side_integrator_supervisor:"};

    // Key inputs
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "input state_machine_release_pulses",
        [&]() { return integrator_supervisor.get_inputs().state_machine_release_pulses.read(); }));
    simulator.register_sampler_entry(
        yeet::make_lambda_sampler<bool>(group + "input forced_commutation_freeze_integrator",
            [&]() { return integrator_supervisor.get_inputs().forced_commutation_freeze_integrator.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "input preset_complete", [&]() { return integrator_supervisor.get_inputs().preset_complete.read(); }));

    // Key outputs
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "output freeze_integrator",
        [&]() { return integrator_supervisor.get_outputs().freeze_integrator.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "output preset_controller",
        [&]() { return integrator_supervisor.get_outputs().preset_controller.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "output release_pulses", [&]() { return integrator_supervisor.get_outputs().release_pulses.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "output shutdown_request",
        [&]() { return integrator_supervisor.get_outputs().shutdown_request.read(); }));
}

static inline void register_complete_grid_side_integrator_supervisor_logger(
    yeet::simulator& simulator, sfc_grid_side_integrator_supervisor_runner& integrator_supervisor)
{
    const std::string group{"grid_side_integrator_supervisor:"};

    register_minimal_grid_side_integrator_supervisor_logger(simulator, integrator_supervisor);

    // Add parameters if needed
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "param freeze_override",
        [&]() { return integrator_supervisor.get_parameters().freeze_override.read(); }));
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
    simulator.register_sampler_entry(
        yeet::make_lambda_array_sampler<std::array<bool, 6>>(group + "output firing_pulses",
            [&]() { return grid_side_modulator.get_outputs().firing_pulses.read().to_array(); }));
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
    simulator.register_sampler_entry(
        yeet::make_lambda_sampler<bool>(group + "input undervoltage_request_extinguish_dc_current",
            [&]() { return grid_side_modulator.get_inputs().undervoltage_request_extinguish_dc_current.read(); }));
}

static inline void register_minimal_machine_side_modulator_logger(
    yeet::simulator& simulator, sfc_machine_side_modulator_runner& machine_side_modulator)
{
    const std::string group{"machine_side_modulator:"};
    simulator.register_sampler_entry(yeet::make_lambda_sampler<real_t>(group + "output firing_angle",
        [&]() { return machine_side_modulator.get_outputs().firing_angle.read().get_pu(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<real_t>(group + "output selected_firing_angle_reference",
        [&]() { return machine_side_modulator.get_outputs().selected_firing_angle_reference.read().get_pu(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<real_t>(group + "output current_rotor_sector", [&]() {
        return static_cast<real_t>(machine_side_modulator.get_outputs().current_rotor_sector.read()) / 6.0f;
    }));
    simulator.register_sampler_entry(
        yeet::make_lambda_array_sampler<std::array<bool, 6>>(group + "output firing_pulses",
            [&]() { return machine_side_modulator.get_outputs().firing_pulses.read().to_array(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "output any_thyristor_fired",
        [&]() { return machine_side_modulator.get_outputs().any_thyristor_fired.read(); }));
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
    simulator.register_sampler_entry(yeet::make_lambda_sampler<real_t>(group + "output estimated_reference_angle",
        [&]() { return machine_side_observer.get_outputs().estimated_reference_angle.read().get_pu(); }));
}

static inline void register_complete_machine_side_observer_logger(
    yeet::simulator& simulator, sfc_machine_side_observer_runner& machine_side_observer)
{
    const std::string group{"machine_side_observer:"};

    register_minimal_machine_side_observer_logger(simulator, machine_side_observer);

    simulator.register_sampler_entry(
        yeet::make_lambda_array_sampler<alphabeta<voltage_pu_t>>(group + "output estimated_voltage_alphabeta_filtered",
            [&]() { return machine_side_observer.get_outputs().estimated_voltage_alphabeta_filtered.read(); }));
}

static inline void register_minimal_plant_model_logger(yeet::simulator& simulator, sfc_plant_model_runner& plant_model)
{
    const std::string group{"plant_model:"};
    // simulator.register_sampler_entry(
    //     yeet::make_lambda_array_sampler<std::array<bool, 6>>(group + "input grid_firing_pulses",
    //         [&]() { return plant_model.get_inputs().grid_firing_pulses.read().to_array(); }));
    // simulator.register_sampler_entry(
    //     yeet::make_lambda_array_sampler<std::array<bool, 6>>(group + "input machine_firing_pulses",
    //         [&]() { return plant_model.get_inputs().machine_firing_pulses.read().to_array(); }));
    // Note: load_torque is now a parameter, not an input signal
    simulator.register_sampler_entry(yeet::make_lambda_sampler<current_t>(
        group + "output dc_link_current", [&]() { return plant_model.get_outputs().dc_link_current.read(); }));
    // simulator.register_sampler_entry(yeet::make_lambda_sampler<voltage_t>(
    //     group + "output dc_link_voltage", [&]() { return plant_model.get_outputs().dc_link_voltage.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<speed_pu_t>(
        group + "output machine_speed", [&]() { return plant_model.get_outputs().machine_speed.read(); }));
    // simulator.register_sampler_entry(yeet::make_lambda_sampler<frequency_t>(
    //     group + "output machine_frequency", [&]() { return plant_model.get_outputs().machine_frequency.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<real_t>(
        group + "output machine_flux_pu", [&]() { return plant_model.get_outputs().machine_flux_pu.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<current_pu_t>(
        group + "output field_current_pu", [&]() { return plant_model.get_outputs().field_current_pu.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_array_sampler<abc<voltage_t>>(
        group + "output grid_voltage_abc", [&]() { return plant_model.get_outputs().grid_voltage_abc.read(); }));
}

static inline void register_complete_plant_model_logger(yeet::simulator& simulator, sfc_plant_model_runner& plant_model)
{
    const std::string group{"plant_model:"};

    register_minimal_plant_model_logger(simulator, plant_model);

    simulator.register_sampler_entry(yeet::make_lambda_array_sampler<abc<voltage_t>>(
        group + "output grid_voltage_abc", [&]() { return plant_model.get_outputs().grid_voltage_abc.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_array_sampler<abc<current_t>>(
        group + "output grid_current_abc", [&]() { return plant_model.get_outputs().grid_current_abc.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_array_sampler<abc<voltage_t>>(
        group + "output machine_voltage_abc", [&]() { return plant_model.get_outputs().machine_voltage_abc.read(); }));
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
        group + "input grid_voltage_abc", [&]() { return hardware_mock.get_inputs().grid_voltage_abc.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_array_sampler<abc<current_t>>(
        group + "input grid_current_abc", [&]() { return hardware_mock.get_inputs().grid_current_abc.read(); }));
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
        group + "input machine_voltage_abc", [&]() { return hardware_mock.get_inputs().machine_voltage_abc.read(); }));
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

// ==================== Grid IO Mock Logger ====================
// Grid-side IO mock models breaker, auxiliaries, fans - log their parameters and timing
static inline void register_minimal_grid_io_mock_logger(
    yeet::simulator& simulator, sfc_grid_io_mock_runner& grid_io_mock)
{
    const std::string group{"grid_io_mock:"};

    // Inputs (commands from control system)
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "input mv_breaker_close", [&]() { return grid_io_mock.get_inputs().mv_breaker_close.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "input mv_breaker_open", [&]() { return grid_io_mock.get_inputs().mv_breaker_open.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "input auxiliaries_on_command",
        [&]() { return grid_io_mock.get_inputs().auxiliaries_on_command.read(); }));

    // Outputs (hardware status feedback)
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "output auxiliaries_are_on", [&]() { return grid_io_mock.get_outputs().auxiliaries_are_on.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "output mv_breaker_is_closed",
        [&]() { return grid_io_mock.get_outputs().mv_breaker_is_closed.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "output start_command", [&]() { return grid_io_mock.get_outputs().start_command.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "output stop_command", [&]() { return grid_io_mock.get_outputs().stop_command.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "output emergency_off", [&]() { return grid_io_mock.get_outputs().emergency_off.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "output door_is_open", [&]() { return grid_io_mock.get_outputs().door_is_open.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "output external_fault", [&]() { return grid_io_mock.get_outputs().external_fault.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "output fans_are_running", [&]() { return grid_io_mock.get_outputs().fans_are_running.read(); }));

    // Key parameters
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "param start_command", [&]() { return grid_io_mock.get_parameters().start_command.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "param stop_command", [&]() { return grid_io_mock.get_parameters().stop_command.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "param emergency_off", [&]() { return grid_io_mock.get_parameters().emergency_off.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "param door_is_open", [&]() { return grid_io_mock.get_parameters().door_is_open.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "param external_fault", [&]() { return grid_io_mock.get_parameters().external_fault.read(); }));
}

static inline void register_complete_grid_io_mock_logger(
    yeet::simulator& simulator, sfc_grid_io_mock_runner& grid_io_mock)
{
    const std::string group{"grid_io_mock:"};

    register_minimal_grid_io_mock_logger(simulator, grid_io_mock);

    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "param mv_breaker_is_tripping",
        [&]() { return grid_io_mock.get_parameters().mv_breaker_is_tripping.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<duration_t>(group + "param breaker_close_delay",
        [&]() { return grid_io_mock.get_parameters().breaker_close_delay.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<duration_t>(
        group + "param breaker_open_delay", [&]() { return grid_io_mock.get_parameters().breaker_open_delay.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<duration_t>(group + "param auxiliary_energize_delay",
        [&]() { return grid_io_mock.get_parameters().auxiliary_energize_delay.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<duration_t>(
        group + "param fan_spinup_delay", [&]() { return grid_io_mock.get_parameters().fan_spinup_delay.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "param auxiliaries_power_available",
        [&]() { return grid_io_mock.get_parameters().auxiliaries_power_available.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "param breaker_mechanism_healthy",
        [&]() { return grid_io_mock.get_parameters().breaker_mechanism_healthy.read(); }));
}

// ==================== Machine IO Mock Logger ====================
// Machine-side IO mock models excitation controller - log its parameters and timing
static inline void register_minimal_machine_io_mock_logger(
    yeet::simulator& simulator, sfc_machine_io_mock_runner& machine_io_mock)
{
    const std::string group{"machine_io_mock:"};

    // Inputs (commands from control system)
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "input excitation_on_command",
        [&]() { return machine_io_mock.get_inputs().excitation_on_command.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "input excitation_off_command",
        [&]() { return machine_io_mock.get_inputs().excitation_off_command.read(); }));

    // Outputs (hardware status feedback)
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "output excitation_status", [&]() { return machine_io_mock.get_outputs().excitation_status.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "output external_fault", [&]() { return machine_io_mock.get_outputs().external_fault.read(); }));

    // Key parameters
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "param external_fault", [&]() { return machine_io_mock.get_parameters().external_fault.read(); }));
}

static inline void register_complete_machine_io_mock_logger(
    yeet::simulator& simulator, sfc_machine_io_mock_runner& machine_io_mock)
{
    const std::string group{"machine_io_mock:"};

    register_minimal_machine_io_mock_logger(simulator, machine_io_mock);

    simulator.register_sampler_entry(yeet::make_lambda_sampler<duration_t>(group + "param excitation_startup_delay",
        [&]() { return machine_io_mock.get_parameters().excitation_startup_delay.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<duration_t>(group + "param excitation_shutdown_delay",
        [&]() { return machine_io_mock.get_parameters().excitation_shutdown_delay.read(); }));
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

static inline void register_minimal_logger(yeet::simulator& simulator, sfc_grid_side_protections_runner& runner)
{
    register_minimal_grid_side_protections_logger(simulator, runner);
}

static inline void register_complete_logger(yeet::simulator& simulator, sfc_grid_side_protections_runner& runner)
{
    register_complete_grid_side_protections_logger(simulator, runner);
}

static inline void register_minimal_logger(yeet::simulator& simulator, sfc_machine_side_protections_runner& runner)
{
    register_minimal_machine_side_protections_logger(simulator, runner);
}

static inline void register_complete_logger(yeet::simulator& simulator, sfc_machine_side_protections_runner& runner)
{
    register_complete_machine_side_protections_logger(simulator, runner);
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

static inline void register_minimal_logger(yeet::simulator& simulator, sfc_startup_sequencer_runner& runner)
{
    register_minimal_startup_sequencer_logger(simulator, runner);
}

static inline void register_complete_logger(yeet::simulator& simulator, sfc_startup_sequencer_runner& runner)
{
    register_complete_startup_sequencer_logger(simulator, runner);
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

static inline void register_minimal_logger(yeet::simulator& simulator, sfc_grid_io_mock_runner& runner)
{
    register_minimal_grid_io_mock_logger(simulator, runner);
}

static inline void register_complete_logger(yeet::simulator& simulator, sfc_grid_io_mock_runner& runner)
{
    register_complete_grid_io_mock_logger(simulator, runner);
}

static inline void register_minimal_logger(yeet::simulator& simulator, sfc_machine_io_mock_runner& runner)
{
    register_minimal_machine_io_mock_logger(simulator, runner);
}

static inline void register_complete_logger(yeet::simulator& simulator, sfc_machine_io_mock_runner& runner)
{
    register_complete_machine_io_mock_logger(simulator, runner);
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
