#include "sfc_integration/sfc_machine_side_control_io.hpp"

#include "sfc_integration/sfc_excitation_controller_runner.hpp"
#include "sfc_integration/sfc_machine_side_controller_runner.hpp"
#include "sfc_integration/sfc_machine_side_measurements_runner.hpp"
#include "sfc_integration/sfc_machine_side_modulator_runner.hpp"
#include "sfc_integration/sfc_machine_side_observer_runner.hpp"
#include "sfc_integration/sfc_machine_side_pulse_generator_runner.hpp"
#include "sfc_integration/sfc_soft_starter_speed_generator_runner.hpp"
#include "sfc_integration/sfc_speed_controller_runner.hpp"
#include "sfc_integration/sfc_torque_controller_runner.hpp"

namespace yeet::application::sfc::integration {

sfc_machine_side_control_io::sfc_machine_side_control_io(
    sfc_machine_side_measurements_runner& machine_measurements,
    sfc_excitation_controller_runner& excitation_controller,
    sfc_soft_starter_speed_generator_runner& speed_generator,
    sfc_speed_controller_runner& speed_controller,
    sfc_torque_controller_runner& torque_controller,
    sfc_machine_side_controller_runner& machine_side_controller,
    sfc_machine_side_observer_runner& machine_side_observer,
    sfc_machine_side_modulator_runner& machine_side_modulator,
    sfc_machine_side_pulse_generator_runner& machine_side_pulse_generator)
    : analog_inputs{
          .v_ab_counts = machine_measurements.get_inputs().v_ab_counts,
          .v_bc_counts = machine_measurements.get_inputs().v_bc_counts,
      },
      digital_inputs{
          .exciter_ready = excitation_controller.get_inputs().exciter_ready,
          .excitation_start_command = excitation_controller.get_inputs().start_command,
          .excitation_stop_command = excitation_controller.get_inputs().stop_command,
          .run_creep_speed = speed_generator.get_inputs().run_profile_1,
          .run_purge_accel = speed_generator.get_inputs().run_profile_2,
          .run_ignition_accel = speed_generator.get_inputs().run_profile_3,
          .run_assist_accel = speed_generator.get_inputs().run_profile_4,
          .hold_speed = speed_generator.get_inputs().hold_speed,
          .decelerate = speed_generator.get_inputs().decelerate,
          .idle = speed_generator.get_inputs().idle,
      },
      grid_side_inputs{
          .dc_link_current_measured = machine_side_controller.get_inputs().dc_link_current,
          .operating_mode = machine_side_modulator.get_inputs().operating_mode,
          .force_commutation_acknowledged = machine_side_modulator.get_inputs().force_commutation_acknowledged,
          .freeze_integrator = speed_controller.get_inputs().freeze_integrator,
          .current_controller_saturated = torque_controller.get_inputs().current_controller_is_saturated,
          .pulse_releases = machine_side_modulator.get_inputs().enable,
          .current_state_id = speed_generator.get_inputs().current_state_id,
      },
      pulse_outputs{
          .firing_pulses = machine_side_pulse_generator.get_outputs().firing_pulses,
      },
      grid_side_outputs{
          .current_reference = torque_controller.get_outputs().current_reference,
          .measured_speed_pu = machine_side_observer.get_outputs().estimated_speed,
          .machine_voltage_magnitude = machine_side_observer.get_outputs().estimated_voltage_magnitude,
          .request_force_commutation = machine_side_modulator.get_outputs().request_force_commutation,
      }
{
}

}  // namespace yeet::application::sfc::integration
