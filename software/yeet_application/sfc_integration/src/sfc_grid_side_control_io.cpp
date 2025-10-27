#include "sfc_integration/sfc_grid_side_control_io.hpp"

#include "sfc_integration/sfc_current_controller_runner.hpp"
#include "sfc_integration/sfc_dc_link_measurements_runner.hpp"
#include "sfc_integration/sfc_grid_side_measurements_runner.hpp"
#include "sfc_integration/sfc_grid_side_modulator_runner.hpp"
#include "sfc_integration/sfc_grid_side_observer_runner.hpp"
#include "sfc_integration/sfc_grid_side_pulse_generator_runner.hpp"
#include "sfc_integration/sfc_operation_mode_monitor_runner.hpp"
#include "sfc_integration/sfc_state_machine_runner.hpp"

namespace yeet::application::sfc::integration {

sfc_grid_side_control_io::sfc_grid_side_control_io(sfc_grid_side_measurements_runner& measurements,
    sfc_dc_link_measurements_runner& dc_link_measurements,
    sfc_state_machine_runner& state_machine,
    sfc_operation_mode_monitor_runner& operation_mode_monitor,
    sfc_current_controller_runner& current_controller,
    sfc_grid_side_modulator_runner& modulator,
    sfc_grid_side_pulse_generator_runner& pulse_generator,
    sfc_grid_side_observer_runner& observer)
    :  // Analog inputs → Grid measurements
      analog_inputs{.v_ab_counts = measurements.get_inputs().v_ab_counts,
          .v_bc_counts = measurements.get_inputs().v_bc_counts,
          .i_a_counts = measurements.get_inputs().i_a_counts,
          .i_c_counts = measurements.get_inputs().i_c_counts},

      // Digital inputs → State machine
      digital_inputs{.auxiliaries_are_on = state_machine.get_inputs().auxiliaries_are_on,
          .breakers_are_on = state_machine.get_inputs().breakers_are_on,
          .start_command = state_machine.get_inputs().start_command,
          .stop_command = state_machine.get_inputs().stop_command,
          .shutdown_request = state_machine.get_inputs().shutdown_request},

      // Machine-side inputs → Various components
      machine_side_inputs{.measured_speed_pu = operation_mode_monitor.get_inputs().measured_speed_pu,
          .dc_link_current_reference = current_controller.get_inputs().dc_link_current_reference,
          .machine_voltage_magnitude = current_controller.get_inputs().machine_side_voltage_magnitude,
          .request_force_commutation = modulator.get_inputs().request_force_commutation},

      // Pulse outputs
      pulse_outputs{.firing_pulses = pulse_generator.get_outputs().firing_pulses},

      // Digital outputs
      digital_outputs{.pulse_releases = state_machine.get_outputs().pulse_releases,
          .is_current_zero = operation_mode_monitor.get_outputs().is_current_zero,
          .current_controller_saturated = current_controller.get_outputs().current_controller_is_saturated,
          .operating_mode = operation_mode_monitor.get_outputs().operating_mode},

      // Machine-side outputs (cross-coupling)
      machine_side_outputs{.grid_voltage_magnitude = observer.get_outputs().estimated_voltage_magnitude,
          .dc_link_current_measured = dc_link_measurements.get_outputs().dc_current_pu,
          .operating_mode = operation_mode_monitor.get_outputs().operating_mode,
          .is_current_zero = operation_mode_monitor.get_outputs().is_current_zero,
          .force_commutation_acknowledged = modulator.get_outputs().force_commutation_acknowledged,
          .freeze_integrator = state_machine.get_outputs().freeze_integrator}
{
}

}  // namespace yeet::application::sfc::integration
