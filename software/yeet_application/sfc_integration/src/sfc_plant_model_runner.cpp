#include "sfc_integration/sfc_plant_model_runner.hpp"

namespace yeet::application::sfc::integration {

sfc_plant_model_runner::sfc_plant_model_runner(std::shared_ptr<sfc::sfc_per_unit_system> per_unit_system)
    : base_application(sampling_time)
{
    assert(per_unit_system != nullptr);
}

void sfc_plant_model_runner::initialize()
{
    // _plant.set_initial_mechanical_speed_pu(speed_pu_t initial_speed_pu)
    // _plant.set_operation_mode(machine_operation_mode mode)

    set_parameters();
    set_parameter_callback();
}

void sfc_plant_model_runner::run()
{
    sfc_plant_model_6p_6p& plant = _plant.plant();

    // plant.set_grid_parameters(frequency_t frequency_hz, voltage_t voltage_rms)
    plant.set_grid_side_pulses(_inputs.grid_side_firing_pulses.read().to_array());
    plant.set_machine_side_pulses(_inputs.machine_side_firing_pulses.read().to_array());
    _plant.set_load_torque_pu(_inputs.load_torque_pu.read());

    _plant.step();

    // _outputs.dc_link_current
    _outputs.grid_voltage = plant.get_grid_side_voltages_abc();
    _outputs.grid_current = plant.get_grid_side_currents_abc();
    // _outputs.dc_link_voltage = {};
    _outputs.dc_link_current = plant.get_dc_link_current();
    _outputs.machine_voltage = plant.get_machine_side_voltages_abc();
    _outputs.machine_current = plant.get_machine_side_currents_abc();
    _outputs.grid_voltage_source_angle = plant.get_grid_voltage_source_angle();
    _outputs.machine_voltage_source_angle = plant.get_machine_voltage_source_angle();
}

void sfc_plant_model_runner::stop() {}

void sfc_plant_model_runner::set_parameters()
{
    // _mode = _parameters.operation_mode;

    // Configure the plant's grid and machine parameters from the shared per-unit system
    // _plant.set_grid_parameters(_per_unit_system->get_grid_bases().base_frequency,
    //                            _per_unit_system->get_grid_bases().base_voltage_peak_phase);
    // _plant.set_machine_parameters(_per_unit_system->get_machine_bases().frequency_base,
    //                               _per_unit_system->get_machine_bases().voltage_base);

    // Configure the mechanical load with inertia and damping from parameters
    // _mechanical_load.configure(_parameters.motor_inertia_constant.read(), _parameters.motor_damping_factor.read());
}

void sfc_plant_model_runner::set_parameter_callback()
{
    _parameters.operation_mode.register_callback([&]() { set_parameters(); });
    _parameters.motor_inertia_constant.register_callback([&]() { set_parameters(); });
    _parameters.motor_damping_factor.register_callback([&]() { set_parameters(); });
}

}  // namespace yeet::application::sfc::integration
