#include "sfc_integration/sfc_plant_model_runner.hpp"

#include <cmath>

namespace yeet::application::sfc::integration {

sfc_plant_model_runner::sfc_plant_model_runner(std::shared_ptr<sfc::sfc_per_unit_system> per_unit_system)
    : base_application(sampling_time), _per_unit_system(per_unit_system), _plant()
{
    assert(per_unit_system != nullptr);
}

void sfc_plant_model_runner::initialize()
{
    // Configure the plant's grid and machine parameters from the shared per-unit system
    const auto& grid_bases = _per_unit_system->get_grid_bases();
    const auto& machine_bases = _per_unit_system->get_machine_bases();

    // Convert peak phase voltage to RMS line voltage for grid side
    voltage_t grid_voltage_rms =
        voltage_t(grid_bases.base_voltage_peak_phase.value() / std::sqrt(2.0) * std::sqrt(3.0));

    _plant.set_grid_parameters(grid_bases.base_frequency.value(), grid_voltage_rms);

    // Convert peak phase voltage to RMS line voltage for machine side
    voltage_t machine_voltage_rms =
        voltage_t(machine_bases.base_voltage_peak_phase.value() / std::sqrt(2.0) * std::sqrt(3.0));

    _plant.set_machine_parameters(_parameters.initial_speed.read() * machine_bases.base_frequency.value(),
        _parameters.initial_speed.read() * machine_voltage_rms);

    // Initialize the plant model
    _plant.initialize();

    // Set initial parameters
    set_parameters();
    set_parameter_callback();
}

void sfc_plant_model_runner::run()
{
    // Set inputs to the plant model
    _plant.set_grid_side_pulses(_inputs.grid_side_firing_pulses.read().to_array());
    _plant.set_machine_side_pulses(_inputs.machine_side_firing_pulses.read().to_array());
    _plant.set_load_torque_pu(_inputs.load_torque_pu.read());

    // Set excitation voltage setpoint if in external control mode
    if (_parameters.voltage_control_mode.read() == datamodel::machine_voltage_control_mode::external_control) {
        _plant.set_excitation_voltage_setpoint_pu(_inputs.excitation_voltage_setpoint_pu.read());
    }

    // Execute one simulation step
    _plant.update();

    // Read outputs from the plant model
    _outputs.grid_voltage_abc = _plant.get_grid_side_voltages_abc();
    _outputs.grid_current_abc = _plant.get_grid_side_currents_abc();
    _outputs.dc_link_current = _plant.get_dc_link_current();
    _outputs.machine_voltage = _plant.get_machine_side_voltages_abc();
    _outputs.machine_current = _plant.get_machine_side_currents_abc();
    _outputs.grid_voltage_source_angle = _plant.get_grid_voltage_source_angle();
    _outputs.machine_voltage_source_angle = _plant.get_machine_voltage_source_angle();
    _outputs.machine_frequency = _plant.get_machine_frequency();
    _outputs.machine_speed =
        speed_pu_t{to_pu(_plant.get_machine_frequency(), _per_unit_system->get_machine_bases().base_frequency)};
    _outputs.grid_frequency = _plant.get_grid_frequency();
}

void sfc_plant_model_runner::stop() {}

void sfc_plant_model_runner::set_parameters()
{
    if (_parameters.operation_mode.read() == datamodel::machine_operation_mode::variable_speed) {
        _plant.set_operation_mode(sfc::plant_model_machine_operation_mode::variable_speed);
    }
    else if (_parameters.operation_mode.read() == datamodel::machine_operation_mode::fixed_speed) {
        _plant.set_operation_mode(sfc::plant_model_machine_operation_mode::fixed_speed);
    }

    // Set voltage control mode
    if (_parameters.voltage_control_mode.read() == datamodel::machine_voltage_control_mode::fixed_voltage) {
        _plant.set_voltage_control_mode(sfc::plant_model_voltage_control_mode::fixed_voltage);
    }
    else if (_parameters.voltage_control_mode.read() == datamodel::machine_voltage_control_mode::speed_dependent) {
        _plant.set_voltage_control_mode(sfc::plant_model_voltage_control_mode::speed_dependent);
    }
    else if (_parameters.voltage_control_mode.read() == datamodel::machine_voltage_control_mode::external_control) {
        _plant.set_voltage_control_mode(sfc::plant_model_voltage_control_mode::external_control);
    }

    _plant.configure_mechanical_load(
        _parameters.motor_inertia_constant.read(), _parameters.motor_damping_factor.read());
}

void sfc_plant_model_runner::set_parameter_callback()
{
    _parameters.operation_mode.register_callback([&]() { set_parameters(); });
    _parameters.voltage_control_mode.register_callback([&]() { set_parameters(); });
    _parameters.motor_inertia_constant.register_callback([&]() { set_parameters(); });
    _parameters.motor_damping_factor.register_callback([&]() { set_parameters(); });
}

}  // namespace yeet::application::sfc::integration
