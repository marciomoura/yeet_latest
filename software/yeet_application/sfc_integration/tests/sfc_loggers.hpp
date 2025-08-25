#pragma once

#include <string>

#include "sfc_integration/sfc_current_controller_runner.hpp"
#include "sfc_integration/sfc_dc_link_measurements_runner.hpp"
#include "sfc_integration/sfc_grid_side_measurements_runner.hpp"
#include "sfc_integration/sfc_grid_side_modulator_runner.hpp"
#include "sfc_integration/sfc_grid_side_observer_runner.hpp"
#include "sfc_integration/sfc_grid_side_pulse_generator_runner.hpp"
#include "sfc_integration/sfc_machine_side_controller_runner.hpp"
#include "sfc_integration/sfc_machine_side_measurements_runner.hpp"
#include "sfc_integration/sfc_machine_side_modulator_runner.hpp"
#include "sfc_integration/sfc_machine_side_observer_runner.hpp"
#include "sfc_integration/sfc_machine_side_pulse_generator_runner.hpp"
#include "sfc_integration/sfc_plant_model_runner.hpp"
#include "sfc_integration/sfc_speed_controller_runner.hpp"
#include "sfc_integration/sfc_state_machine_runner.hpp"
#include "yeet_testing/data_sampler_entry.hpp"
#include "yeet_testing/simulator.hpp"

namespace yeet::application::sfc::integration {

static inline void register_minimal_current_controller_logger(
    yeet::simulator& simulator, sfc_current_controller_runner& current_controller)
{
    const std::string group{"current_controller:"};
    simulator.register_sampler_entry(yeet::make_lambda_sampler<current_pu_t>(group + "input dc_link_current_reference",
        [&]() { return current_controller.get_inputs().dc_link_current_reference.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<current_pu_t>(group + "input dc_link_current_measured",
        [&]() { return current_controller.get_inputs().dc_link_current_measured.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "input freeze_integrator", [&]() { return current_controller.get_inputs().freeze_integrator.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<current_pu_t>(group + "output current_error",
        [&]() { return current_controller.get_signal_flow_datamodel().get_outputs().current_error.read(); }));
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
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "output is_active",
        [&]() { return current_controller.get_signal_flow_datamodel().get_outputs().is_active.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<real_t>(group + "output firing_angle",
        [&]() { return current_controller.get_signal_flow_datamodel().get_outputs().firing_angle.read().get_pu(); }));
}

static inline void register_minimal_speed_controller_logger(
    yeet::simulator& simulator, sfc_speed_controller_runner& speed_controller)
{
    const std::string group{"speed_controller:"};
    simulator.register_sampler_entry(yeet::make_lambda_sampler<speed_pu_t>(
        group + "input speed_reference", [&]() { return speed_controller.get_inputs().speed_reference_pu.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<speed_pu_t>(
        group + "input measured_speed", [&]() { return speed_controller.get_inputs().measured_speed_pu.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "input freeze_integrator", [&]() { return speed_controller.get_inputs().freeze_integrator.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "input is_active", [&]() { return speed_controller.get_outputs().is_active.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<speed_pu_t>(group + "output speed_error",
        [&]() { return speed_controller.get_signal_flow_datamodel().get_outputs().speed_error.read(); }));
}

static inline void register_complete_speed_controller_logger(
    yeet::simulator& simulator, sfc_speed_controller_runner& speed_controller)
{
    const std::string group{"speed_controller:"};

    register_minimal_speed_controller_logger(simulator, speed_controller);

    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "input enable_speed_control",
        [&]() { return speed_controller.get_inputs().enable_speed_control.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<current_pu_t>(group + "output current_reference",
        [&]() { return speed_controller.get_signal_flow_datamodel().get_outputs().current_reference.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<torque_pu_t>(group + "output torque_reference",
        [&]() { return speed_controller.get_signal_flow_datamodel().get_outputs().torque_reference.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<speed_pu_t>(group + "output speed_reference_limited",
        [&]() { return speed_controller.get_signal_flow_datamodel().get_outputs().speed_reference_limited.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<real_t>(group + "output pi_output",
        [&]() { return speed_controller.get_signal_flow_datamodel().get_outputs().pi_output.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "output is_saturated",
        [&]() { return speed_controller.get_signal_flow_datamodel().get_outputs().is_saturated.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "output is_active",
        [&]() { return speed_controller.get_signal_flow_datamodel().get_outputs().is_active.read(); }));
}

static inline void register_minimal_dc_link_measurements_logger(
    yeet::simulator& simulator, sfc_dc_link_measurements_runner& dc_link_measurements)
{
    const std::string group{"dc_link_measurements:"};
    simulator.register_sampler_entry(yeet::make_lambda_sampler<current_pu_t>(group + "output dc_current_pu",
        [&]() { return dc_link_measurements.get_signal_flow_datamodel().get_outputs().dc_current_pu.read(); }));
}

static inline void register_complete_dc_link_measurements_logger(
    yeet::simulator& simulator, sfc_dc_link_measurements_runner& dc_link_measurements)
{
    const std::string group{"dc_link_measurements:"};

    register_minimal_dc_link_measurements_logger(simulator, dc_link_measurements);

    simulator.register_sampler_entry(
        yeet::make_lambda_sampler<current_pu_t>(group + "output dc_current_filtered_pu", [&]() {
            return dc_link_measurements.get_signal_flow_datamodel().get_outputs().dc_current_filtered_pu.read();
        }));
    simulator.register_sampler_entry(
        yeet::make_lambda_sampler<current_pu_t>(group + "output dc_current_heavy_filtered_pu", [&]() {
            return dc_link_measurements.get_signal_flow_datamodel().get_outputs().dc_current_heavy_filtered_pu.read();
        }));
}

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
    simulator.register_sampler_entry(
        yeet::make_lambda_sampler<voltage_pu_t>(group + "output grid_voltage_magnitude", [&]() {
            return grid_side_measurements.get_signal_flow_datamodel().get_outputs().grid_voltage_magnitude.read();
        }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "output is_commuting",
        [&]() { return grid_side_measurements.get_signal_flow_datamodel().get_outputs().is_commuting.read(); }));
}

static inline void register_complete_grid_side_measurements_logger(
    yeet::simulator& simulator, sfc_grid_side_measurements_runner& grid_side_measurements)
{
    const std::string group{"grid_side_measurements:"};

    register_minimal_grid_side_measurements_logger(simulator, grid_side_measurements);

    simulator.register_sampler_entry(yeet::make_lambda_sampler<frequency_pu_t>(group + "output grid_side_frequency",
        [&]() { return grid_side_measurements.get_signal_flow_datamodel().get_outputs().grid_side_frequency.read(); }));
    simulator.register_sampler_entry(
        yeet::make_lambda_array_sampler<alphabeta<voltage_pu_t>>(group + "output_grid_voltage_alphabeta", [&]() {
            return grid_side_measurements.get_signal_flow_datamodel().get_outputs().grid_voltage_alphabeta.read();
        }));
    simulator.register_sampler_entry(yeet::make_lambda_array_sampler<alphabeta<voltage_pu_t>>(
        group + "output_grid_voltage_alphabeta_filtered", [&]() {
            return grid_side_measurements.get_signal_flow_datamodel()
                .get_outputs()
                .grid_voltage_alphabeta_filtered.read();
        }));
    simulator.register_sampler_entry(
        yeet::make_lambda_array_sampler<abc<voltage_pu_t>>(group + "output_grid_voltage_abc", [&]() {
            return grid_side_measurements.get_signal_flow_datamodel().get_outputs().grid_voltage_abc.read();
        }));
    simulator.register_sampler_entry(
        yeet::make_lambda_array_sampler<abc<voltage_pu_t>>(group + "output_grid_voltage_abc_unmasked", [&]() {
            return grid_side_measurements.get_signal_flow_datamodel().get_outputs().grid_voltage_abc_unmasked.read();
        }));
    simulator.register_sampler_entry(
        yeet::make_lambda_array_sampler<abc<voltage_pu_t>>(group + "output_grid_voltage_abc_filtered", [&]() {
            return grid_side_measurements.get_signal_flow_datamodel().get_outputs().grid_voltage_abc_filtered.read();
        }));
    simulator.register_sampler_entry(
        yeet::make_lambda_array_sampler<abc<current_pu_t>>(group + "output_grid_current_abc", [&]() {
            return grid_side_measurements.get_signal_flow_datamodel().get_outputs().grid_current_abc.read();
        }));
    simulator.register_sampler_entry(
        yeet::make_lambda_sampler<voltage_pu_t>(group + "output grid_voltage_magnitude_filtered", [&]() {
            return grid_side_measurements.get_signal_flow_datamodel()
                .get_outputs()
                .grid_voltage_magnitude_filtered.read();
        }));
}

static inline void register_minimal_machine_side_measurements_logger(
    yeet::simulator& simulator, sfc_machine_side_measurements_runner& machine_side_measurements)
{
    const std::string group{"machine_side_measurements:"};
    simulator.register_sampler_entry(
        yeet::make_lambda_array_sampler<abc<voltage_pu_t>>(group + "output machine_voltage_abc", [&]() {
            return machine_side_measurements.get_signal_flow_datamodel().get_outputs().machine_voltage_abc.read();
        }));
    simulator.register_sampler_entry(
        yeet::make_lambda_array_sampler<abc<current_pu_t>>(group + "output machine_current_abc", [&]() {
            return machine_side_measurements.get_signal_flow_datamodel().get_outputs().machine_current_abc.read();
        }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<speed_pu_t>(group + "output machine_side_speed", [&]() {
        return machine_side_measurements.get_signal_flow_datamodel().get_outputs().machine_side_speed.read();
    }));
    simulator.register_sampler_entry(
        yeet::make_lambda_sampler<voltage_pu_t>(group + "output machine_voltage_magnitude", [&]() {
            return machine_side_measurements.get_signal_flow_datamodel().get_outputs().machine_voltage_magnitude.read();
        }));
}

static inline void register_complete_machine_side_measurements_logger(
    yeet::simulator& simulator, sfc_machine_side_measurements_runner& machine_side_measurements)
{
    const std::string group{"machine_side_measurements:"};

    register_minimal_machine_side_measurements_logger(simulator, machine_side_measurements);

    simulator.register_sampler_entry(
        yeet::make_lambda_array_sampler<alphabeta<voltage_pu_t>>(group + "output_machine_voltage_alphabeta", [&]() {
            return machine_side_measurements.get_signal_flow_datamodel().get_outputs().machine_voltage_alphabeta.read();
        }));
    simulator.register_sampler_entry(yeet::make_lambda_array_sampler<alphabeta<voltage_pu_t>>(
        group + "output_machine_voltage_alphabeta_filtered", [&]() {
            return machine_side_measurements.get_signal_flow_datamodel()
                .get_outputs()
                .machine_voltage_alphabeta_filtered.read();
        }));
    simulator.register_sampler_entry(
        yeet::make_lambda_array_sampler<abc<voltage_pu_t>>(group + "output_machine_voltage_abc", [&]() {
            return machine_side_measurements.get_signal_flow_datamodel().get_outputs().machine_voltage_abc.read();
        }));
    simulator.register_sampler_entry(
        yeet::make_lambda_array_sampler<abc<voltage_pu_t>>(group + "output_machine_voltage_abc_unmasked", [&]() {
            return machine_side_measurements.get_signal_flow_datamodel()
                .get_outputs()
                .machine_voltage_abc_unmasked.read();
        }));
    simulator.register_sampler_entry(
        yeet::make_lambda_array_sampler<abc<voltage_pu_t>>(group + "output_machine_voltage_abc_filtered", [&]() {
            return machine_side_measurements.get_signal_flow_datamodel()
                .get_outputs()
                .machine_voltage_abc_filtered.read();
        }));
    simulator.register_sampler_entry(
        yeet::make_lambda_array_sampler<abc<current_pu_t>>(group + "output_machine_current_abc", [&]() {
            return machine_side_measurements.get_signal_flow_datamodel().get_outputs().machine_current_abc.read();
        }));
    simulator.register_sampler_entry(
        yeet::make_lambda_sampler<voltage_pu_t>(group + "output machine_voltage_magnitude_filtered", [&]() {
            return machine_side_measurements.get_signal_flow_datamodel()
                .get_outputs()
                .machine_voltage_magnitude_filtered.read();
        }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "output is_commuting",
        [&]() { return machine_side_measurements.get_signal_flow_datamodel().get_outputs().is_commuting.read(); }));
}

static inline void register_minimal_grid_side_observer_logger(
    yeet::simulator& simulator, sfc_grid_side_observer_runner& grid_side_observer)
{
    const std::string group{"grid_side_observer:"};
    simulator.register_sampler_entry(yeet::make_lambda_array_sampler<abc<voltage_pu_t>>(
        group + "input_grid_voltage_pu", [&]() { return grid_side_observer.get_inputs().grid_voltage_pu.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<real_t>(group + "output estimated_angle", [&]() {
        return grid_side_observer.get_signal_flow_datamodel().get_outputs().estimated_position.read().get_pu();
    }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<frequency_pu_t>(group + "output estimated_frequency_pu",
        [&]() { return grid_side_observer.get_signal_flow_datamodel().get_outputs().estimated_frequency_pu.read(); }));
}

static inline void register_complete_grid_side_observer_logger(
    yeet::simulator& simulator, sfc_grid_side_observer_runner& grid_side_observer)
{
    const std::string group{"grid_side_observer:"};

    register_minimal_grid_side_observer_logger(simulator, grid_side_observer);

    simulator.register_sampler_entry(yeet::make_lambda_sampler<frequency_pu_t>(group + "output estimated_frequency_pu",
        [&]() { return grid_side_observer.get_signal_flow_datamodel().get_outputs().estimated_frequency_pu.read(); }));
    simulator.register_sampler_entry(
        yeet::make_lambda_sampler<voltage_pu_t>(group + "output estimated_voltage_magnitude", [&]() {
            return grid_side_observer.get_signal_flow_datamodel().get_outputs().estimated_voltage_magnitude.read();
        }));
    simulator.register_sampler_entry(
        yeet::make_lambda_array_sampler<alphabeta<voltage_pu_t>>(group + "output_filtered_voltage_alphabeta", [&]() {
            return grid_side_observer.get_signal_flow_datamodel().get_outputs().filtered_voltage_alphabeta.read();
        }));
}

static inline void register_minimal_state_machine_logger(
    yeet::simulator& simulator, sfc_state_machine_runner& state_machine)
{
    const std::string group{"state_machine:"};
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "input start_command", [&]() { return state_machine.get_inputs().start_command.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "input stop_command", [&]() { return state_machine.get_inputs().stop_command.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "output pulse_releases",
        [&]() { return state_machine.get_signal_flow_datamodel().get_outputs().pulse_releases.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "output freeze_integrator",
        [&]() { return state_machine.get_signal_flow_datamodel().get_outputs().freeze_integrator.read(); }));
}

static inline void register_complete_state_machine_logger(
    yeet::simulator& simulator, sfc_state_machine_runner& state_machine)
{
    const std::string group{"state_machine:"};

    register_minimal_state_machine_logger(simulator, state_machine);

    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "input auxiliaries_are_on", [&]() { return state_machine.get_inputs().auxiliaries_are_on.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "input breakers_are_on", [&]() { return state_machine.get_inputs().breakers_are_on.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "input shutdown_request", [&]() { return state_machine.get_inputs().shutdown_request.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "input is_current_extinguished",
        [&]() { return state_machine.get_inputs().is_current_extinguished.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "output has_transitioned",
        [&]() { return state_machine.get_signal_flow_datamodel().get_outputs().has_transitioned.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "output is_idle",
        [&]() { return state_machine.get_signal_flow_datamodel().get_outputs().is_idle.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "output is_auxiliaries_on",
        [&]() { return state_machine.get_signal_flow_datamodel().get_outputs().is_auxiliaries_on.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "output is_breakers_off",
        [&]() { return state_machine.get_signal_flow_datamodel().get_outputs().is_breakers_off.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "output is_breakers_on",
        [&]() { return state_machine.get_signal_flow_datamodel().get_outputs().is_breakers_on.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "output is_operation_off",
        [&]() { return state_machine.get_signal_flow_datamodel().get_outputs().is_operation_off.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "output is_operation_on",
        [&]() { return state_machine.get_signal_flow_datamodel().get_outputs().is_operation_on.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "output is_shutdown_requested",
        [&]() { return state_machine.get_signal_flow_datamodel().get_outputs().is_shutdown_requested.read(); }));
}

static inline void register_minimal_machine_side_controller_logger(
    yeet::simulator& simulator, sfc_machine_side_controller_runner& machine_side_controller)
{
    const std::string group{"machine_side_controller:"};
    simulator.register_sampler_entry(yeet::make_lambda_sampler<speed_pu_t>(
        group + "input machine_speed", [&]() { return machine_side_controller.get_inputs().machine_speed.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<current_pu_t>(group + "input dc_link_current",
        [&]() { return machine_side_controller.get_inputs().dc_link_current.read(); }));
    simulator.register_sampler_entry(
        yeet::make_lambda_sampler<real_t>(group + "output machine_side_firing_angle", [&]() {
            return machine_side_controller.get_signal_flow_datamodel()
                .get_outputs()
                .machine_side_firing_angle.read()
                .get_pu();
        }));
}

static inline void register_complete_machine_side_controller_logger(
    yeet::simulator& simulator, sfc_machine_side_controller_runner& machine_side_controller)
{
    const std::string group{"machine_side_controller:"};

    register_minimal_machine_side_controller_logger(simulator, machine_side_controller);

    // Add additional outputs if available in the complete datamodel
}

static inline void register_minimal_grid_side_modulator_logger(
    yeet::simulator& simulator, sfc_grid_side_modulator_runner& grid_side_modulator)
{
    const std::string group{"grid_side_modulator:"};
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "input request_force_commutation",
        [&]() { return grid_side_modulator.get_inputs().request_force_commutation.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "input dc_link_current_is_zero",
        [&]() { return grid_side_modulator.get_inputs().dc_link_current_is_zero.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "input enable", [&]() { return grid_side_modulator.get_inputs().enable.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<real_t>(group + "output firing_angle",
        [&]() { return grid_side_modulator.get_signal_flow_datamodel().get_outputs().firing_angle.read().get_pu(); }));
}

static inline void register_complete_grid_side_modulator_logger(
    yeet::simulator& simulator, sfc_grid_side_modulator_runner& grid_side_modulator)
{
    const std::string group{"grid_side_modulator:"};

    register_minimal_grid_side_modulator_logger(simulator, grid_side_modulator);

    simulator.register_sampler_entry(
        yeet::make_lambda_sampler<bool>(group + "output force_commutation_acknowledged", [&]() {
            return grid_side_modulator.get_signal_flow_datamodel().get_outputs().force_commutation_acknowledged.read();
        }));
    simulator.register_sampler_entry(
        yeet::make_lambda_sampler<bool>(group + "output is_requesting_main_pi_reset", [&]() {
            return grid_side_modulator.get_signal_flow_datamodel().get_outputs().is_requesting_main_pi_reset.read();
        }));
}

static inline void register_minimal_machine_side_modulator_logger(
    yeet::simulator& simulator, sfc_machine_side_modulator_runner& machine_side_modulator)
{
    const std::string group{"machine_side_modulator:"};
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(
        group + "input enable", [&]() { return machine_side_modulator.get_inputs().enable.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<real_t>(group + "output firing_angle", [&]() {
        return machine_side_modulator.get_signal_flow_datamodel().get_outputs().firing_angle.read().get_pu();
    }));
}

static inline void register_complete_machine_side_modulator_logger(
    yeet::simulator& simulator, sfc_machine_side_modulator_runner& machine_side_modulator)
{
    const std::string group{"machine_side_modulator:"};

    register_minimal_machine_side_modulator_logger(simulator, machine_side_modulator);

    simulator.register_sampler_entry(
        yeet::make_lambda_sampler<bool>(group + "input force_commutation_acknowledged", [&]() {
            return machine_side_modulator.get_signal_flow_datamodel()
                .get_inputs()
                .force_commutation_acknowledged.read();
        }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "output request_force_commutation", [&]() {
        return machine_side_modulator.get_signal_flow_datamodel().get_outputs().request_force_commutation.read();
    }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "output release_pulses",
        [&]() { return machine_side_modulator.get_signal_flow_datamodel().get_outputs().release_pulses.read(); }));
}

static inline void register_minimal_machine_side_observer_logger(
    yeet::simulator& simulator, sfc_machine_side_observer_runner& machine_side_observer)
{
    const std::string group{"machine_side_observer:"};
    simulator.register_sampler_entry(
        yeet::make_lambda_array_sampler<abc<voltage_pu_t>>(group + "input_machine_voltage_pu",
            [&]() { return machine_side_observer.get_inputs().machine_voltage_pu.read(); }));
    simulator.register_sampler_entry(
        yeet::make_lambda_sampler<voltage_pu_t>(group + "output estimated_voltage_magnitude", [&]() {
            return machine_side_observer.get_signal_flow_datamodel().get_outputs().estimated_voltage_magnitude.read();
        }));
    simulator.register_sampler_entry(
        yeet::make_lambda_sampler<frequency_pu_t>(group + "output estimated_frequency_pu", [&]() {
            return machine_side_observer.get_signal_flow_datamodel().get_outputs().estimated_frequency_pu.read();
        }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<speed_pu_t>(group + "output estimated_speed",
        [&]() { return machine_side_observer.get_signal_flow_datamodel().get_outputs().estimated_speed.read(); }));
}

static inline void register_complete_machine_side_observer_logger(
    yeet::simulator& simulator, sfc_machine_side_observer_runner& machine_side_observer)
{
    const std::string group{"machine_side_observer:"};

    register_minimal_machine_side_observer_logger(simulator, machine_side_observer);

    simulator.register_sampler_entry(
        yeet::make_lambda_sampler<frequency_pu_t>(group + "output estimated_frequency_pu", [&]() {
            return machine_side_observer.get_signal_flow_datamodel().get_outputs().estimated_frequency_pu.read();
        }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<real_t>(group + "output estimated_position", [&]() {
        return machine_side_observer.get_signal_flow_datamodel().get_outputs().estimated_position.read().get_pu();
    }));
    simulator.register_sampler_entry(
        yeet::make_lambda_array_sampler<alphabeta<voltage_pu_t>>(group + "output_filtered_voltage_alphabeta", [&]() {
            return machine_side_observer.get_signal_flow_datamodel().get_outputs().filtered_voltage_alphabeta.read();
        }));
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
        yeet::make_lambda_array_sampler<std::array<bool, 6>>(group + "output_firing_pulses", [&]() {
            return grid_side_pulse_generator.get_signal_flow_datamodel().get_outputs().firing_pulses.read().to_array();
        }));
}

static inline void register_complete_grid_side_pulse_generator_logger(
    yeet::simulator& simulator, sfc_grid_side_pulse_generator_runner& grid_side_pulse_generator)
{
    const std::string group{"grid_side_pulse_generator:"};

    register_minimal_grid_side_pulse_generator_logger(simulator, grid_side_pulse_generator);

    simulator.register_sampler_entry(yeet::make_lambda_sampler<real_t>(group + "output actual_firing_angle", [&]() {
        return grid_side_pulse_generator.get_signal_flow_datamodel().get_outputs().actual_firing_angle.read().get_pu();
    }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<bool>(group + "output is_active",
        [&]() { return grid_side_pulse_generator.get_signal_flow_datamodel().get_outputs().is_active.read(); }));
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
    simulator.register_sampler_entry(yeet::make_lambda_array_sampler<
        std::array<bool, 6>>(group + "output_firing_pulses", [&]() {
        return machine_side_pulse_generator.get_signal_flow_datamodel().get_outputs().firing_pulses.read().to_array();
    }));
}

static inline void register_complete_machine_side_pulse_generator_logger(
    yeet::simulator& simulator, sfc_machine_side_pulse_generator_runner& machine_side_pulse_generator)
{
    const std::string group{"machine_side_pulse_generator:"};

    register_minimal_machine_side_pulse_generator_logger(simulator, machine_side_pulse_generator);

    simulator.register_sampler_entry(yeet::make_lambda_sampler<real_t>(group + "output actual_firing_angle", [&]() {
        return machine_side_pulse_generator.get_signal_flow_datamodel()
            .get_outputs()
            .actual_firing_angle.read()
            .get_pu();
    }));
}

static inline void register_minimal_plant_model_logger(yeet::simulator& simulator, sfc_plant_model_runner& plant_model)
{
    const std::string group{"plant_model:"};
    simulator.register_sampler_entry(
        yeet::make_lambda_array_sampler<std::array<bool, 6>>(group + "input_grid_side_firing_pulses",
            [&]() { return plant_model.get_inputs().grid_side_firing_pulses.read().to_array(); }));
    simulator.register_sampler_entry(
        yeet::make_lambda_array_sampler<std::array<bool, 6>>(group + "input_machine_side_firing_pulses",
            [&]() { return plant_model.get_inputs().machine_side_firing_pulses.read().to_array(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<torque_pu_t>(
        group + "input load_torque_pu", [&]() { return plant_model.get_inputs().load_torque_pu.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<current_t>(group + "output dc_link_current",
        [&]() { return plant_model.get_signal_flow_datamodel().get_outputs().dc_link_current.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<voltage_t>(group + "output dc_link_voltage",
        [&]() { return plant_model.get_signal_flow_datamodel().get_outputs().dc_link_voltage.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<speed_pu_t>(group + "output machine_speed",
        [&]() { return plant_model.get_signal_flow_datamodel().get_outputs().machine_speed.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<frequency_t>(group + "output machine_frequency",
        [&]() { return plant_model.get_signal_flow_datamodel().get_outputs().machine_frequency.read(); }));
}

static inline void register_complete_plant_model_logger(yeet::simulator& simulator, sfc_plant_model_runner& plant_model)
{
    const std::string group{"plant_model:"};

    register_minimal_plant_model_logger(simulator, plant_model);

    simulator.register_sampler_entry(yeet::make_lambda_array_sampler<abc<voltage_t>>(group + "output_grid_voltage",
        [&]() { return plant_model.get_signal_flow_datamodel().get_outputs().grid_voltage.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_array_sampler<abc<current_t>>(group + "output_grid_current",
        [&]() { return plant_model.get_signal_flow_datamodel().get_outputs().grid_current.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_array_sampler<abc<voltage_t>>(group + "output_machine_voltage",
        [&]() { return plant_model.get_signal_flow_datamodel().get_outputs().machine_voltage.read(); }));
    simulator.register_sampler_entry(yeet::make_lambda_array_sampler<abc<current_t>>(group + "output_machine_current",
        [&]() { return plant_model.get_signal_flow_datamodel().get_outputs().machine_current.read(); }));
    simulator.register_sampler_entry(
        yeet::make_lambda_sampler<real_t>(group + "output grid_voltage_source_angle", [&]() {
            return plant_model.get_signal_flow_datamodel().get_outputs().grid_voltage_source_angle.read().get_pu();
        }));
    simulator.register_sampler_entry(
        yeet::make_lambda_sampler<real_t>(group + "output machine_voltage_source_angle", [&]() {
            return plant_model.get_signal_flow_datamodel().get_outputs().machine_voltage_source_angle.read().get_pu();
        }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<frequency_t>(group + "output grid_frequency",
        [&]() { return plant_model.get_signal_flow_datamodel().get_outputs().grid_frequency.read(); }));
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

static inline void register_minimal_logger(yeet::simulator& simulator, sfc_dc_link_measurements_runner& runner)
{
    register_minimal_dc_link_measurements_logger(simulator, runner);
}

static inline void register_complete_logger(yeet::simulator& simulator, sfc_dc_link_measurements_runner& runner)
{
    register_complete_dc_link_measurements_logger(simulator, runner);
}

static inline void register_minimal_logger(yeet::simulator& simulator, sfc_grid_side_measurements_runner& runner)
{
    register_minimal_grid_side_measurements_logger(simulator, runner);
}

static inline void register_complete_logger(yeet::simulator& simulator, sfc_grid_side_measurements_runner& runner)
{
    register_complete_grid_side_measurements_logger(simulator, runner);
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

}  // namespace yeet::application::sfc::integration