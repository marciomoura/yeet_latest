#include "sfc_integration/sfc_plant_model_machine_runner.hpp"

namespace yeet::application::sfc::integration {

sfc_plant_model_machine_runner::sfc_plant_model_machine_runner() : base_application(sampling_time), _plant() {}

void sfc_plant_model_machine_runner::initialize()
{
    // The plant model is already initialized in the constructor via _model->initialize()
    // Do not call update() here - the Simulink solver structures (including derivs)
    // are only fully allocated during the first proper step() call in run()

    // Set initial parameters
    set_parameters();
}

void sfc_plant_model_machine_runner::run()
{
    // Update load coefficients from parameters
    _load_coefficients[0] = _parameters.load_coeff_c0.read();
    _load_coefficients[1] = _parameters.load_coeff_c1.read();
    _load_coefficients[2] = _parameters.load_coeff_c2.read();

    // Execute one simulation step with all inputs
    _plant.update(_inputs.grid_firing_pulses.read().to_array(), _inputs.machine_firing_pulses.read().to_array(),
        _parameters.grid_voltage_magnitude_a.read(), _parameters.grid_voltage_magnitude_b.read(),
        _parameters.grid_voltage_magnitude_c.read(), _parameters.grid_frequency_param.read(),
        _parameters.field_voltage.read(), _load_coefficients, _parameters.forced_speed_pu.read(),
        _parameters.forced_speed_enabled.read(), _parameters.dc_isolator_closed.read());

    // Read DC link outputs
    _outputs.dc_link_voltage = _plant.get_dc_link_voltage();
    _outputs.dc_link_current = _plant.get_dc_link_current();

    // Read grid-side outputs
    _outputs.grid_current_abc = _plant.get_grid_side_currents_abc();
    _outputs.grid_voltage_abc = _plant.get_grid_side_voltages_line_to_line_abc();
    _outputs.grid_voltage_source_angle = _plant.get_grid_voltage_source_angle();
    _outputs.grid_voltage_source_phase_to_neutral_abc = _plant.get_grid_voltage_source_phase_to_neutral_abc();
    _outputs.grid_voltage_source_currents_abc = _plant.get_grid_voltage_source_currents_abc();

    // Read machine-side converter outputs
    _outputs.machine_current_abc = _plant.get_machine_side_currents_abc();
    _outputs.machine_voltage_abc = _plant.get_machine_side_voltages_line_to_line_abc();
    _outputs.machine_dc_voltage = _plant.get_machine_side_dc_voltage();

    // Read machine stator outputs
    _outputs.machine_stator_currents_abc = _plant.get_machine_stator_currents_abc();
    const auto& stator_state = _plant.get_machine_stator_state_dq();
    _outputs.machine_stator_current_d = stator_state.current_d;
    _outputs.machine_stator_current_q = stator_state.current_q;
    _outputs.machine_stator_voltage_d = stator_state.voltage_d;
    _outputs.machine_stator_voltage_q = stator_state.voltage_q;

    // Read machine mechanical outputs
    const auto& mechanical_state = _plant.get_machine_mechanical_state();
    _outputs.machine_speed = mechanical_state.speed;
    _outputs.machine_electrical_torque = mechanical_state.electrical_torque;
    _outputs.machine_mechanical_angle = angle_wrapped::from_degrees(mechanical_state.mechanical_angle);
    _outputs.machine_load_torque = mechanical_state.load_torque;
    _outputs.machine_electrical_power = mechanical_state.electrical_power;

    // Read machine rotor outputs
    const auto& rotor_state = _plant.get_machine_rotor_state();
    _outputs.machine_flux_d = rotor_state.flux_d;
    _outputs.machine_flux_q = rotor_state.flux_q;
    _outputs.machine_field_current_d = rotor_state.field_current_d;
    _outputs.machine_damper_current_q1 = rotor_state.damper_current_q1;
    _outputs.machine_damper_current_q2 = rotor_state.damper_current_q2;
    _outputs.machine_damper_current_d = rotor_state.damper_current_d;

    // Read source voltage phase
    _outputs.source_voltage_phase =
        angle_wrapped::from_radians(angle_t{pi / 2}) + _plant.get_machine_side_voltages_line_to_line_abc()
                                                           .to_alphabeta()
                                                           .phase();  //_plant.get_source_voltage_phase();

    // Grid frequency (from parameter, as this is fixed in the Simulink model)
    _outputs.grid_frequency = _parameters.grid_frequency_param.read();
}

void sfc_plant_model_machine_runner::stop() { _plant.terminate(); }

void sfc_plant_model_machine_runner::set_parameters()
{
    // Parameters are applied during the update() call via the convenience update() method
    // No additional configuration needed here since all parameters are passed directly
}

}  // namespace yeet::application::sfc::integration
