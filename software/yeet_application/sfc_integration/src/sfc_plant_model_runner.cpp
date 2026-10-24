#include "sfc_integration/sfc_plant_model_runner.hpp"

#include <cmath>

namespace yeet::application::sfc::integration {

sfc_plant_model_runner::sfc_plant_model_runner(std::shared_ptr<sfc::sfc_per_unit_system> per_unit_system)
    : base_application(sampling_time),
      _per_unit_system(per_unit_system),
      _plant(std::make_unique<sfc::sfc_plant_model_voltage_behind_reactance>(sfc::plant_topology_type::simulink))
{
    assert(per_unit_system != nullptr);
}

void sfc_plant_model_runner::register_datamodel(yeet::telemetry::parameter_handler& handler)
{
    // Delegate to the datamodel's registration method
    sfc_plant_model_datamodel datamodel(&_inputs, &_outputs, &_parameters);
    datamodel.register_data_model(handler);
}

void sfc_plant_model_runner::initialize()
{
    const auto plant_topology = convert_topology(_parameters.topology.read());
    _plant = std::make_unique<sfc::sfc_plant_model_voltage_behind_reactance>(plant_topology);

    const auto initial_speed = speed_pu_t{_parameters.initial_speed.read()};
    configure_plant_bases(initial_speed);

    _plant->initialize();
    set_parameters();
    set_parameter_callback();
}

void sfc_plant_model_runner::run()
{
    // Apply grid operating point parameters (voltage and frequency) before each update
    // This ensures parameter changes take effect immediately, including fault injections
    _plant->set_grid_operating_point(_parameters.grid_voltage_pu.read(), _parameters.grid_frequency_pu.read());

    // Set inputs to the plant model
    _plant->set_grid_side_pulses(_inputs.grid_firing_pulses.read().to_array());
    _plant->set_machine_side_pulses(_inputs.machine_firing_pulses.read().to_array());

    // Set load torque from constant torque parameter
    _plant->set_load_torque_pu(_parameters.constant_torque.read());

    // Always set field current reference in variable speed mode
    // In fixed_voltage mode, this input is ignored by the plant
    if (_parameters.voltage_control_mode.read() == datamodel::machine_voltage_control_mode::variable_speed) {
        _plant->set_field_current_reference_pu(_inputs.field_current_reference.read());
    }

    // Execute one simulation step
    _plant->update();

    auto const speed_pu =
        speed_pu_t{to_pu(_plant->get_machine_frequency(), _per_unit_system->get_machine_bases().base_frequency)};

    if (speed_pu <= speed_pu_t{0.0}) {
        _plant->set_load_torque_pu(torque_pu_t{0.0});
    }
    else {
        _plant->set_load_torque_pu(_parameters.constant_torque.read());
    }

    // Read outputs from the plant model
    _outputs.grid_voltage_abc = _plant->get_grid_side_voltages_abc();
    _outputs.grid_current_abc = _plant->get_grid_side_currents_abc();
    _outputs.dc_link_current = _plant->get_dc_link_current();
    _outputs.machine_voltage_abc = _plant->get_machine_side_voltages_abc();
    _outputs.machine_current = _plant->get_machine_side_currents_abc();
    _outputs.grid_voltage_source_angle = _plant->get_grid_voltage_source_angle();
    _outputs.machine_voltage_source_angle = _plant->get_machine_voltage_source_angle();
    _outputs.machine_frequency = _plant->get_machine_frequency();
    _outputs.machine_speed =
        speed_pu_t{to_pu(_plant->get_machine_frequency(), _per_unit_system->get_machine_bases().base_frequency)};
    _outputs.grid_frequency = _plant->get_grid_frequency();
    _outputs.machine_flux_pu = _plant->get_flux_pu();
    _outputs.field_current_pu = _plant->get_field_current();
}
void sfc_plant_model_runner::stop() {}

void sfc_plant_model_runner::set_parameters()
{
    set_operation_mode();
    set_voltage_control_mode();
    set_mechanical_load();
    set_grid_operating_point();
}

void sfc_plant_model_runner::set_operation_mode()
{
    if (_parameters.operation_mode.read() == datamodel::machine_operation_mode::variable_speed) {
        _plant->set_operation_mode(sfc::plant_model_machine_operation_mode::variable_speed);
    }
    else {
        _plant->set_operation_mode(sfc::plant_model_machine_operation_mode::fixed_speed);
    }
}

void sfc_plant_model_runner::set_voltage_control_mode()
{
    if (_parameters.voltage_control_mode.read() == datamodel::machine_voltage_control_mode::fixed_voltage) {
        _plant->set_voltage_control_mode(sfc::plant_model_voltage_control_mode::fixed_voltage);
    }
    else {
        _plant->set_voltage_control_mode(sfc::plant_model_voltage_control_mode::variable_speed);
    }
}

void sfc_plant_model_runner::set_mechanical_load()
{
    _plant->configure_mechanical_load(_parameters.motor_inertia_constant.read(),
        _parameters.motor_damping_factor.read(), _parameters.motor_linear_load_coefficient.read(),
        _parameters.motor_quadratic_load_coefficient.read());
}

void sfc_plant_model_runner::set_grid_operating_point()
{
    _plant->set_grid_frequency_pu(_parameters.grid_frequency_pu.read());
    _plant->set_grid_voltage_pu(_parameters.grid_voltage_pu.read());
}

void sfc_plant_model_runner::set_parameter_callback()
{
    _parameters.operation_mode.register_callback([&]() { set_operation_mode(); });
    _parameters.voltage_control_mode.register_callback([&]() { set_voltage_control_mode(); });
    _parameters.motor_inertia_constant.register_callback([&]() { set_mechanical_load(); });
    _parameters.motor_damping_factor.register_callback([&]() { set_mechanical_load(); });
    _parameters.motor_linear_load_coefficient.register_callback([&]() { set_mechanical_load(); });
    _parameters.motor_quadratic_load_coefficient.register_callback([&]() { set_mechanical_load(); });
    _parameters.grid_frequency_pu.register_callback([&]() { set_grid_operating_point(); });
    _parameters.grid_voltage_pu.register_callback([&]() { set_grid_operating_point(); });
    _parameters.topology.register_callback([&]() { handle_topology_change(); });
    _parameters.fixed_machine_voltage.register_callback([&]() { handle_topology_change(); });
}

void sfc_plant_model_runner::handle_topology_change()
{
    const auto plant_topology = convert_topology(_parameters.topology.read());

    if (_plant->get_topology_type() == plant_topology) {
        return;
    }

    // Capture current state for seamless transfer
    const auto current_speed = _plant->get_mechanical_speed_pu();
    const auto current_operation_mode = _plant->get_operation_mode();
    const auto current_voltage_mode = _plant->get_voltage_control_mode();
    const auto current_field_current = _plant->get_field_current();

    // Recreate plant with new topology
    _plant = std::make_unique<sfc::sfc_plant_model_voltage_behind_reactance>(plant_topology);
    configure_plant_bases(current_speed);
    _plant->initialize();

    // Restore operation modes and mechanical state
    _plant->set_operation_mode(current_operation_mode);
    _plant->set_voltage_control_mode(current_voltage_mode);
    _plant->set_initial_mechanical_speed_pu(current_speed);
    _plant->set_field_current_reference_pu(current_field_current);

    if (_parameters.voltage_control_mode.read() == datamodel::machine_voltage_control_mode::fixed_voltage) {
        _plant->set_machine_voltage_pu(_parameters.fixed_machine_voltage.read());
    }

    set_parameters();
}

sfc::plant_topology_type sfc_plant_model_runner::convert_topology(datamodel::topology_type topology) const noexcept
{
    switch (topology) {
        case datamodel::topology_type::internal:
            return sfc::plant_topology_type::internal;
        case datamodel::topology_type::rectifier:
            return sfc::plant_topology_type::rectifier;
        case datamodel::topology_type::simulink:
        default:
            return sfc::plant_topology_type::simulink;
    }
}

void sfc_plant_model_runner::configure_plant_bases(speed_pu_t speed_pu)
{
    const auto& grid_bases = _per_unit_system->get_grid_bases();
    const auto& machine_bases = _per_unit_system->get_machine_bases();

    // Convert peak phase voltage to RMS line voltage
    voltage_t grid_voltage_rms =
        voltage_t(grid_bases.base_voltage_peak_phase.value() / std::sqrt(2.0) * std::sqrt(3.0));
    voltage_t machine_voltage_rms =
        voltage_t(machine_bases.base_voltage_peak_phase.value() / std::sqrt(2.0) * std::sqrt(3.0));

    _plant->set_grid_parameters(grid_bases.base_frequency.value(), grid_voltage_rms);
    _plant->set_machine_parameters(
        speed_pu.value() * machine_bases.base_frequency.value(), speed_pu.value() * machine_voltage_rms);
}

}  // namespace yeet::application::sfc::integration
