#include "sfc_integration/sfc_plant_model_runner.hpp"

#include <cmath>

#include "yeet_control/constants.hpp"

namespace yeet::application::sfc::integration {

sfc_plant_model_runner::sfc_plant_model_runner(std::shared_ptr<sfc::sfc_per_unit_system> per_unit_system)
    : base_application(sampling_time), _mechanical_load(sampling_time), _per_unit_system(std::move(per_unit_system))
{
    assert(per_unit_system != nullptr);
}

void sfc_plant_model_runner::initialize()
{
    _plant.initialize();
    set_parameters();
    set_parameter_callback();
    _mechanical_load.reset();  // Reset to zero initial speed and position
}

void sfc_plant_model_runner::run()
{
    // 1. Read inputs from the datamodel and set them in the plant
    convert_inputs();

    // 2. Execute one step of the plant simulation
    _plant.step();

    // 3. If in variable speed mode, update the mechanical load model
    if (_mode == datamodel::machine_operation_mode::variable_speed) {
        update_mechanical_load();
    }

    // 4. Read results from the plant and write them to the output signals
    convert_outputs();
}

void sfc_plant_model_runner::stop() { _plant.terminate(); }

void sfc_plant_model_runner::set_parameters()
{
    _mode = _parameters.operation_mode;

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

void sfc_plant_model_runner::convert_inputs()
{
    // Set firing pulses from input signals
    _plant.set_grid_side_pulses(_inputs.grid_side_firing_pulses.read().to_array());
    _plant.set_machine_side_pulses(_inputs.machine_side_firing_pulses.read().to_array());
}

void sfc_plant_model_runner::update_mechanical_load()
{
    // Calculate electromagnetic torque from DC link current in per-unit
    const auto& dc_bases = _per_unit_system->get_dc_link_bases();
    current_pu_t i_dc_pu = to_pu(_plant.get_dc_link_current(), dc_bases.base_current);
    torque_pu_t electromagnetic_torque_pu = per_unit_cast<torque_pu_t>(i_dc_pu);

    // Get load torque from the input signal
    torque_pu_t load_torque_pu = _inputs.load_torque_pu;

    // Step the mechanical load model
    speed_pu_t new_speed_pu = _mechanical_load.update(electromagnetic_torque_pu, load_torque_pu);

    // Update the plant model's machine side frequency based on the new speed
    const auto& machine_bases = _per_unit_system->get_machine_bases();
    frequency_pu_t new_frequency_pu = per_unit_cast<frequency_pu_t>(new_speed_pu);
    frequency_t new_machine_frequency = to_si(new_frequency_pu, machine_bases.base_frequency);
    _plant.set_machine_parameters(new_machine_frequency, machine_bases.base_voltage_peak_phase.value());
}

void sfc_plant_model_runner::convert_outputs()
{
    // Set SI electrical outputs directly from the plant model
    _outputs.grid_voltage_ln = _plant.get_grid_side_voltages_abc();
    _outputs.grid_current = _plant.get_grid_side_currents_abc();
    _outputs.machine_voltage_ln = _plant.get_machine_side_voltages_abc();
    _outputs.machine_current = _plant.get_machine_side_currents_abc();
    _outputs.dc_link_current = _plant.get_dc_link_current();
    // _outputs.dc_link_voltage = _plant.get_dc_link_voltage();

    // Set mechanical outputs from the load model
    _outputs.rotor_speed_mechanical = _mechanical_load.get_speed_pu();

    // Convert rotor position from per-unit [0, 1) to a wrapped angle [0, 2*pi)
    // angle_pu_t pos_pu = _mechanical_load..get_position_pu();
    // _outputs.rotor_position_mechanical = angle<zero_to_two_pi>::from_radians(pos_pu.value() * two_pi);

    // Calculate and set torque and power outputs in per-unit
    const auto& dc_bases = _per_unit_system->get_dc_link_bases();
    current_pu_t i_dc_pu = to_pu(_plant.get_dc_link_current(), dc_bases.base_current);
    _outputs.electromagnetic_torque = per_unit_cast<torque_pu_t>(i_dc_pu);

    // voltage_pu_t v_dc_pu = to_pu(_plant.get_dc_link_voltage(), dc_bases.base_voltage);
    // _outputs.active_power_pu = power_pu_t(v_dc_pu.value() * i_dc_pu.value());

    // Set remaining outputs (placeholders, as in the simulator)
    _outputs.power_factor = real_t(0.85);
    _outputs.grid_current_thd = real_t(0.15);
}

}  // namespace yeet::application::sfc::integration
