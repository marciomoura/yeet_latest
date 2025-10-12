// Copyright 2025 Techperts
// Implementation of SFC Slow IO Runner

#include "sfc_integration/sfc_slow_io_runner.hpp"

namespace yeet::application::sfc::integration {

sfc_slow_io_runner::sfc_slow_io_runner()
    : base_application(sampling_time),
      _digital_inputs(_digital_input_raw),
      _digital_outputs(_digital_output_raw),
      _analog_inputs(_analog_input_raw),
      _analog_outputs(_analog_output_raw)
{
    // Raw structs are zero-initialized by default
    // Wrappers reference the raw structs (no additional storage)
}

void sfc_slow_io_runner::initialize()
{
    // Set all outputs to safe state
    _digital_outputs.reset_all();
    _analog_outputs.reset_all();

    // Process initial state (should remain safe with all zeros)
    process_outputs();
}

void sfc_slow_io_runner::run()
{
    // Step 1: Read raw inputs from hardware and process
    // Note: Hardware layer must have already updated _digital_input_raw and _analog_input_raw
    // before calling this run() method
    process_inputs();

    // Step 2: Read signal inputs (commands from other components) and write to hardware
    process_outputs();
}

void sfc_slow_io_runner::stop()
{
    // Return all outputs to safe state
    _digital_outputs.reset_all();
    _analog_outputs.reset_all();

    // Write safe state to hardware
    process_outputs();
}

void sfc_slow_io_runner::process_inputs()
{
    // ===== Publish Digital Inputs =====

    // TCS Commands
    _outputs.start_command.write(_digital_inputs.get_start_command());
    _outputs.stop_command.write(_digital_inputs.get_stop_command());

    // Safety Interlocks
    _outputs.front_door_open.write(_digital_inputs.get_front_door_open());
    _outputs.lv_supervision_ok.write(_digital_inputs.get_lv_supervision_ok());
    _outputs.supervision_ok.write(_digital_inputs.get_supervision_ok());
    _outputs.mcb_open_alarm.write(_digital_inputs.get_mcb_open_alarm());

    // Excitation Controller Status
    _outputs.exciter_is_running.write(_digital_inputs.get_exciter_is_running());
    _outputs.exciter_voltage_applied.write(_digital_inputs.get_exciter_voltage_applied());

    // Circuit Breaker Status
    _outputs.icb_is_closed.write(_digital_inputs.get_icb_is_closed());
    _outputs.icb_is_open.write(_digital_inputs.get_icb_is_open());
    _outputs.icb_is_tripping.write(_digital_inputs.get_icb_is_tripping());

    // Protection System Inputs
    _outputs.transformer_pressure_alarm.write(_digital_inputs.get_transformer_pressure_alarm());
    _outputs.overheat_alarm.write(_digital_inputs.get_overheat_alarm());
    _outputs.overheat_trip.write(_digital_inputs.get_overheat_trip());
    _outputs.transformer_light_gas_alarm.write(_digital_inputs.get_transformer_light_gas_alarm());
    _outputs.transformer_heavy_gas_trip.write(_digital_inputs.get_transformer_heavy_gas_trip());
    _outputs.emergency_stop.write(_digital_inputs.get_emergency_stop());
    _outputs.common_hardware_trip.write(_digital_inputs.get_common_hardware_trip());
    _outputs.external_fault.write(_digital_inputs.get_external_fault());
    _outputs.overvoltage_fuse_blown.write(_digital_inputs.get_overvoltage_fuse_blown());
    _outputs.fan_stop_alarm.write(_digital_inputs.get_fan_stop_alarm());
    _outputs.reactor_temp_alarm.write(_digital_inputs.get_reactor_temp_alarm());
    _outputs.reactor_temp_trip.write(_digital_inputs.get_reactor_temp_trip());

    // ===== Publish Analog Inputs =====
    _outputs.excitation_current_a.write(_analog_inputs.get_excitation_current_a());
    _outputs.excitation_voltage_v.write(_analog_inputs.get_excitation_voltage_v());
}

void sfc_slow_io_runner::process_outputs()
{
    // ===== Read Signal Inputs and Write to Digital Outputs =====

    // Auxiliary Control
    _digital_outputs.set_fan1_on(_inputs.fan1_on.read());
    _digital_outputs.set_fan2_on(_inputs.fan2_on.read());
    _digital_outputs.set_fan3_on(_inputs.fan3_on.read());
    _digital_outputs.set_fan4_on(_inputs.fan4_on.read());

    // Circuit Breaker Commands
    _digital_outputs.set_icb_close_command(_inputs.icb_close_command.read());
    _digital_outputs.set_icb_open_command(_inputs.icb_open_command.read());
    _digital_outputs.set_icb_trip_command(_inputs.icb_trip_command.read());

    // TCS Feedback
    _digital_outputs.set_operation_off_feedback(_inputs.operation_off_feedback.read());
    _digital_outputs.set_operation_on_feedback(_inputs.operation_on_feedback.read());

    // Excitation Controller Commands
    _digital_outputs.set_exciter_on_command(_inputs.exciter_on_command.read());
    _digital_outputs.set_exciter_off_command(_inputs.exciter_off_command.read());
    _digital_outputs.set_exciter_control_mode(_inputs.exciter_control_mode.read());

    // ===== Read Signal Inputs and Write to Analog Outputs =====
    _analog_outputs.set_excitation_voltage_ref_v(_inputs.excitation_voltage_ref_v.read());
    _analog_outputs.set_excitation_current_ref_a(_inputs.excitation_current_ref_a.read());
}

bool sfc_slow_io_runner::compute_auxiliaries_healthy() const noexcept
{
    return _digital_inputs.get_auxiliaries_are_healthy();
}

bool sfc_slow_io_runner::compute_shutdown_requested() const noexcept
{
    return _digital_inputs.get_shutdown_requested();
}

bool sfc_slow_io_runner::compute_excitation_field_ready() const noexcept
{
    // Get configured thresholds
    const float current_threshold_pu = _parameters.field_current_threshold_pu.read();
    const float voltage_threshold_pu = _parameters.field_voltage_threshold_pu.read();

    // Get base values for per-unit conversion
    const float base_current_a = _parameters.rated_field_current_a.read();
    const float base_voltage_v = _parameters.rated_field_voltage_v.read();

    // Convert feedback to per-unit
    const float current_pu = _analog_inputs.get_excitation_current_pu(base_current_a);
    const float voltage_pu = _analog_inputs.get_excitation_voltage_pu(base_voltage_v);

    // Field is ready when BOTH current AND voltage exceed thresholds
    const bool current_ok = (current_pu > current_threshold_pu);
    const bool voltage_ok = (voltage_pu > voltage_threshold_pu);

    return current_ok && voltage_ok;
}
}  // namespace yeet::application::sfc::integration
