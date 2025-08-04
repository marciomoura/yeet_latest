#include "sfc_plant_model_simulator.hpp"

namespace yeet::application::sfc {

sfc_plant_model_simulator::sfc_plant_model_simulator()
    : _mechanical_load(sfc_plant_model_6p_6p::kSamplingTime),
      _speed_voltage_relationship(sfc_plant_model_6p_6p::kSamplingTime),
      _pu_system(kNominalGridVoltageRMS,
          kNominalGridFrequency,
          kBaseDCLinkCurrent,
          kMachineRatedPowerVA,
          kNominalMachineVoltageRMS,
          kNominalMachineFrequency,
          kMachineNumPoles)
{
    // Configure with default mechanical parameters
    _plant.initialize();
    _mechanical_load.configure(kDefaultInertiaConstantH, kDefaultDampingFactorD);

    // Configure with default speed-voltage parameters
    _speed_voltage_relationship.configure_voltage_speed_coefficient(kDefaultVoltageSpeedCoefficient);
    _speed_voltage_relationship.configure_voltage_offset_pu(kDefaultVoltageOffset);
    _speed_voltage_relationship.configure_voltage_limits_pu(0.1f, 1.5f);  // Reasonable voltage limits
}

void sfc_plant_model_simulator::set_operation_mode(machine_operation_mode mode) { _mode = mode; }

void sfc_plant_model_simulator::set_voltage_control_mode(voltage_control_mode mode) { _voltage_mode = mode; }

void sfc_plant_model_simulator::configure_mechanical_load(
    inertia_constant_t inertia_constant_seconds, damping_coefficient_pu_t damping_coefficient)
{
    _mechanical_load.configure(inertia_constant_seconds, damping_coefficient);
}

void sfc_plant_model_simulator::configure_speed_voltage_relationship(
    real_t coefficient, real_t offset_pu, real_t time_constant_seconds)
{
    _speed_voltage_relationship.configure_voltage_speed_coefficient(coefficient);
    _speed_voltage_relationship.configure_voltage_offset_pu(offset_pu);
    _speed_voltage_relationship.configure_time_constant(time_constant_seconds);
}

void sfc_plant_model_simulator::configure_voltage_limits_pu(real_t min_voltage_pu, real_t max_voltage_pu)
{
    _speed_voltage_relationship.configure_voltage_limits_pu(min_voltage_pu, max_voltage_pu);
}

void sfc_plant_model_simulator::set_initial_mechanical_speed_pu(speed_pu_t initial_speed_pu)
{
    _mechanical_load.reset(initial_speed_pu);

    _speed_voltage_relationship.reset(1.0, 1.0);  // initial_speed_pu.value());
    // Initialize speed-voltage relationship with corresponding voltage
    // if (_voltage_mode == voltage_control_mode::speed_dependent) {
    // }

    // If in variable speed mode, also update the machine frequency to match
    if (_mode == machine_operation_mode::variable_speed) {
        frequency_pu_t initial_frequency_pu = per_unit_cast<frequency_pu_t>(initial_speed_pu);
        frequency_t machine_frequency_hz = yeet::to_si(initial_frequency_pu, kNominalMachineFrequency);

        voltage_t machine_voltage = kNominalMachineVoltageRMS;
        if (_voltage_mode == voltage_control_mode::speed_dependent) {
            real_t voltage_magnitude_pu = _speed_voltage_relationship.get_voltage_magnitude_pu();
            machine_voltage = voltage_t(voltage_magnitude_pu * kNominalMachineVoltageRMS.value());
        }

        _plant.set_machine_parameters(machine_frequency_hz, machine_voltage);
    }
}

void sfc_plant_model_simulator::set_load_torque_pu(torque_pu_t load_torque_pu)
{
    // Set the load torque in the mechanical load model
    _load_torque_pu = load_torque_pu;
}

control::mechanical_load& sfc_plant_model_simulator::mechanical_load() { return _mechanical_load; }

control::speed_voltage_relationship& sfc_plant_model_simulator::speed_voltage_relationship()
{
    return _speed_voltage_relationship;
}

const sfc_per_unit_system& sfc_plant_model_simulator::per_unit_system() const { return _pu_system; }

sfc_plant_model_6p_6p& sfc_plant_model_simulator::plant() { return _plant; }

void sfc_plant_model_simulator::setup_simulator(yeet::simulator& simulator)
{
    simulator.register_lambda_runner(sfc_plant_model_6p_6p::kSamplingTime, [&]() { this->step(); });
}

void sfc_plant_model_simulator::setup_all_logging(yeet::simulator& simulator)
{
    setup_grid_side_logging(simulator);
    setup_machine_side_logging(simulator);
    setup_dc_link_logging(simulator);
    setup_mechanical_load_logging(simulator);
}

void sfc_plant_model_simulator::setup_grid_side_logging(yeet::simulator& simulator)
{
    // Log grid-side voltages (abc)
    simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
        "grid_voltage_a", [&]() -> double { return _plant.get_grid_side_voltages_abc().a(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
        "grid_voltage_b", [&]() -> double { return _plant.get_grid_side_voltages_abc().b(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
        "grid_voltage_c", [&]() -> double { return _plant.get_grid_side_voltages_abc().c(); }));

    // Log grid-side currents (abc)
    simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
        "grid_current_a", [&]() -> double { return _plant.get_grid_side_currents_abc().a(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
        "grid_current_b", [&]() -> double { return _plant.get_grid_side_currents_abc().b(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
        "grid_current_c", [&]() -> double { return _plant.get_grid_side_currents_abc().c(); }));

    // Log grid-side angle
    simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
        "grid_angle_rad", [&]() -> double { return _plant.get_grid_voltage_source_angle().get_radians(); }));
}

void sfc_plant_model_simulator::setup_machine_side_logging(yeet::simulator& simulator)
{
    // Log machine-side voltages (abc)
    simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
        "machine_voltage_a", [&]() -> double { return _plant.get_machine_side_voltages_abc().a(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
        "machine_voltage_b", [&]() -> double { return _plant.get_machine_side_voltages_abc().b(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
        "machine_voltage_c", [&]() -> double { return _plant.get_machine_side_voltages_abc().c(); }));

    // Log machine-side currents (abc)
    simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
        "machine_current_a", [&]() -> double { return _plant.get_machine_side_currents_abc().a(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
        "machine_current_b", [&]() -> double { return _plant.get_machine_side_currents_abc().b(); }));
    simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
        "machine_current_c", [&]() -> double { return _plant.get_machine_side_currents_abc().c(); }));

    // Log machine-side angle
    simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
        "machine_angle_rad", [&]() -> double { return _plant.get_machine_voltage_source_angle().get_radians(); }));
}

void sfc_plant_model_simulator::setup_dc_link_logging(yeet::simulator& simulator)
{
    // Log DC link current
    simulator.register_sampler_entry(
        yeet::make_lambda_sampler<double>("dc_link_current", [&]() -> double { return _plant.get_dc_link_current(); }));
}

void sfc_plant_model_simulator::setup_mechanical_load_logging(yeet::simulator& simulator)
{
    // Log mechanical speed
    simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
        "mechanical_speed_pu", [&]() -> double { return _mechanical_load.get_speed_pu(); }));

    // Log machine frequency
    simulator.register_sampler_entry(yeet::make_lambda_sampler<double>("machine_frequency_hz", [&]() -> double {
        if (_mode == machine_operation_mode::variable_speed) {
            return _mechanical_load.get_speed_pu() * kNominalMachineFrequency.value();
        }
        else {
            return kNominalMachineFrequency.value();
        }
    }));

    // Log electromagnetic torque
    simulator.register_sampler_entry(yeet::make_lambda_sampler<double>("electromagnetic_torque_pu",
        [&]() -> double { return _plant.get_dc_link_current() / kBaseDCLinkCurrent.value(); }));
}

void sfc_plant_model_simulator::step()
{
    // Always step the plant model first
    _plant.update();

    // Step the mechanical dynamics (if enabled)
    step_mechanical_dynamics();

    // Step the voltage control (if enabled)
    step_voltage_control();

    // Update plant parameters with the computed values
    update_plant_parameters();
}

void sfc_plant_model_simulator::step_mechanical_dynamics()
{
    if (_mode != machine_operation_mode::variable_speed) {
        return;  // No mechanical dynamics in fixed speed mode
    }

    // Calculate electromagnetic torque from DC link current
    // Convert DC current to per-unit using the base DC current
    const current_pu_t dc_link_current = to_pu(_plant.get_dc_link_current(), kBaseDCLinkCurrent);

    // Calculate electromagnetic torque in per-unit
    // For LCI drives: T_e = k * I_dc where k is proportional to machine flux
    // In per-unit: T_e_pu = I_dc_pu (assuming unit flux)
    const torque_pu_t electromagnetic_torque_pu = torque_pu_t{dc_link_current.value()};

    // Step the mechanical load with electromagnetic torque
    _mechanical_load.update(electromagnetic_torque_pu, _load_torque_pu);
}

void sfc_plant_model_simulator::step_voltage_control()
{
    if (_voltage_mode != voltage_control_mode::speed_dependent) {
        return;  // No voltage control in fixed voltage mode
    }

    // Determine the speed input for voltage calculation
    const speed_pu_t speed_input_pu = _mechanical_load.get_speed_pu();

    // Update the speed-voltage relationship
    _speed_voltage_relationship.update(speed_input_pu);
}

void sfc_plant_model_simulator::update_plant_parameters()
{
    const frequency_t machine_frequency_hz =
        (_mode == machine_operation_mode::variable_speed)
            ? to_si(frequency_pu_t{_mechanical_load.get_speed_pu()}, kNominalMachineFrequency)
            : _plant.get_machine_frequency();

    const voltage_t machine_voltage =
        (_voltage_mode == voltage_control_mode::speed_dependent)
            ? voltage_t(_speed_voltage_relationship.get_voltage_magnitude_pu() * kNominalMachineVoltageRMS.value())
            : kNominalMachineVoltageRMS;

    _plant.set_machine_parameters(machine_frequency_hz, machine_voltage);
}

voltage_t sfc_plant_model_simulator::get_machine_voltage() const
{
    if (_voltage_mode == voltage_control_mode::speed_dependent) {
        // Use voltage from speed-voltage relationship
        const real_t voltage_magnitude_pu = _speed_voltage_relationship.get_voltage_magnitude_pu();
        return voltage_t(voltage_magnitude_pu * kNominalMachineVoltageRMS.value());
    }
    else {
        // Use nominal voltage for fixed voltage mode
        return kNominalMachineVoltageRMS;
    }
}

}  // namespace yeet::application::sfc