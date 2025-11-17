#include "sfc_integration/sfc_plant_model_runner.hpp"

#include <cmath>

namespace yeet::application::sfc::integration {

sfc_plant_model_runner::sfc_plant_model_runner(std::shared_ptr<sfc::sfc_per_unit_system> per_unit_system)
    : base_application(sampling_time), _per_unit_system(per_unit_system), _plant()
{
    assert(per_unit_system != nullptr);
}

void sfc_plant_model_runner::register_datamodel(yeet::model_registry& registry)
{
    // Delegate to the datamodel's registration method
    sfc_plant_model_datamodel datamodel(&_inputs, &_outputs, &_parameters);
    datamodel.register_data_model(registry);
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
    _plant.set_grid_side_pulses(_inputs.grid_firing_pulses.read().to_array());
    _plant.set_machine_side_pulses(_inputs.machine_firing_pulses.read().to_array());

    // Set load torque from constant torque parameter
    _plant.set_load_torque_pu(_parameters.constant_torque.read());

    // Always set field current reference in variable speed mode
    // In fixed_voltage mode, this input is ignored by the plant
    if (_parameters.voltage_control_mode.read() == datamodel::machine_voltage_control_mode::variable_speed) {
        _plant.set_field_current_reference_pu(_inputs.field_current_reference.read());
    }

    // Execute one simulation step
    _plant.update();

    auto const speed_pu =
        speed_pu_t{to_pu(_plant.get_machine_frequency(), _per_unit_system->get_machine_bases().base_frequency)};

    if (speed_pu <= speed_pu_t{0.0}) {
        _plant.set_load_torque_pu(torque_pu_t{0.0});
    }
    else {
        _plant.set_load_torque_pu(_parameters.constant_torque.read());
    }

    // Read outputs from the plant model
    _outputs.grid_voltage_abc = _plant.get_grid_side_voltages_abc();
    _outputs.grid_current_abc = _plant.get_grid_side_currents_abc();
    _outputs.dc_link_current = _plant.get_dc_link_current();
    _outputs.machine_voltage_abc = _plant.get_machine_side_voltages_abc();
    _outputs.machine_current = _plant.get_machine_side_currents_abc();
    _outputs.grid_voltage_source_angle = _plant.get_grid_voltage_source_angle();
    _outputs.machine_voltage_source_angle = _plant.get_machine_voltage_source_angle();
    _outputs.machine_frequency = _plant.get_machine_frequency();
    _outputs.machine_speed =
        speed_pu_t{to_pu(_plant.get_machine_frequency(), _per_unit_system->get_machine_bases().base_frequency)};
    _outputs.grid_frequency = _plant.get_grid_frequency();
    _outputs.machine_flux_pu = _plant.get_flux_pu();
    _outputs.field_current_pu = _plant.get_field_current();
}
void sfc_plant_model_runner::stop() {}

void sfc_plant_model_runner::set_parameters()
{
    // Ensure parameters have been written at least once
    // (tests may have written different values before initialize() was called)
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
    else if (_parameters.voltage_control_mode.read() == datamodel::machine_voltage_control_mode::variable_speed) {
        _plant.set_voltage_control_mode(sfc::plant_model_voltage_control_mode::variable_speed);
    }

    _plant.configure_mechanical_load(_parameters.motor_inertia_constant.read(), _parameters.motor_damping_factor.read(),
        _parameters.motor_linear_load_coefficient.read(), _parameters.motor_quadratic_load_coefficient.read());
}

void sfc_plant_model_runner::set_parameter_callback()
{
    _parameters.operation_mode.register_callback([&]() { set_parameters(); });
    _parameters.voltage_control_mode.register_callback([&]() { set_parameters(); });
    _parameters.motor_inertia_constant.register_callback([&]() { set_parameters(); });
    _parameters.motor_damping_factor.register_callback([&]() { set_parameters(); });
    _parameters.motor_linear_load_coefficient.register_callback([&]() { set_parameters(); });
    _parameters.motor_quadratic_load_coefficient.register_callback([&]() { set_parameters(); });
}

}  // namespace yeet::application::sfc::integration
