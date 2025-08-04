#include "sfc/sfc_plant_model_augmented_6p_6p.hpp"

namespace yeet::application::sfc {

namespace {
// Private namespace for 6p-6p topology default parameters

// Frequency and electrical parameters with proper units
static constexpr frequency_t kNominalGridFrequency = frequency_t(50.0);     // 50 Hz grid frequency
static constexpr frequency_t kNominalMachineFrequency = frequency_t(50.0);  // 50 Hz machine frequency
static constexpr voltage_t kNominalGridVoltageRMS = voltage_t(2090.0);      // 2090V RMS line voltage
static constexpr voltage_t kNominalMachineVoltageRMS = voltage_t(1980.0);   // 1980V RMS line voltage

// Firing angles (dimensionless angles in degrees)
static constexpr angle_t kNominalGridFiringAngleDeg = angle_t(13.0);              // Nominal firing angle for grid side
static constexpr angle_t kNominalMachineFiringAngleDeg = angle_t(180.0 - 26.84);  // Nominal firing angle for motor side

// Circuit parameters with proper units
static constexpr resistance_t kGridSideResistance = resistance_t(0.1552);        // 0.1552 Ohm grid side resistance
static constexpr inductance_t kGridSideInductance = inductance_t(0.4941e-3);     // 0.4941 mH grid side inductance
static constexpr resistance_t kMachineSideResistance = resistance_t(0.0788);     // 0.0788 Ohm machine side resistance
static constexpr inductance_t kMachineSideInductance = inductance_t(0.2508e-3);  // 0.2508 mH machine side inductance

// Power and current ratings with proper units
static constexpr current_t kBaseDCLinkCurrent = current_t(1044.0);  // 1044 A base DC link current
static constexpr power_t kMachineRatedPowerVA = power_t(2950e3);    // 2.95 MVA machine rated power
static constexpr int kMachineNumPoles = 2;                          // Number of poles (dimensionless)

// Default Mechanical Parameters with proper units
static constexpr inertia_constant_t kDefaultInertiaConstantH = inertia_constant_t(2.5);  // 2.5 s inertia constant
static constexpr damping_coefficient_pu_t kDefaultDampingFactorD =
    damping_coefficient_pu_t(0.01);  // 0.01 pu damping factor

// Default Speed-Voltage Parameters
static constexpr real_t kDefaultVoltageSpeedCoefficient = 0.9f;  // 0.9 pu/pu voltage-speed coefficient
static constexpr real_t kDefaultVoltageOffset = 0.1f;            // 0.1 pu voltage offset

}  // anonymous namespace

sfc_plant_model_augmented_6p_6p::sfc_plant_model_augmented_6p_6p(
    const sfc_plant_model_electrical_parameters& electrical_params,
    const sfc_plant_model_mechanical_parameters& mechanical_params,
    const sfc_plant_model_voltage_control_parameters& voltage_control_params)
    : _mechanical_load(duration_t(kSamplingTime)),
      _speed_voltage_relationship(duration_t(kSamplingTime)),
      _pu_system(electrical_params.grid_voltage_rms,
          electrical_params.grid_frequency,
          electrical_params.base_dc_link_current,
          electrical_params.machine_rated_power_va,
          electrical_params.machine_voltage_rms,
          electrical_params.machine_frequency,
          electrical_params.machine_num_poles),
      _plant(electrical_params),
      _electrical_params(electrical_params),
      _mechanical_params(mechanical_params),
      _voltage_control_params(voltage_control_params)
{
    configure_mechanical_parameters(mechanical_params);
    configure_voltage_control_parameters(voltage_control_params);
}

sfc_plant_model_augmented_6p_6p::sfc_plant_model_augmented_6p_6p()
    : sfc_plant_model_augmented_6p_6p(create_default_electrical_parameters(),
          create_default_mechanical_parameters(),
          create_default_voltage_control_parameters())
{
}

sfc_plant_model_augmented_6p_6p::~sfc_plant_model_augmented_6p_6p() = default;

void sfc_plant_model_augmented_6p_6p::configure_electrical_parameters(
    const sfc_plant_model_electrical_parameters& electrical_params)
{
    _electrical_params = electrical_params;
    _plant.configure_electrical_parameters(electrical_params);

    // Update per-unit system with new parameters
    _pu_system = sfc_per_unit_system(electrical_params.grid_voltage_rms, electrical_params.grid_frequency,
        electrical_params.base_dc_link_current, electrical_params.machine_rated_power_va,
        electrical_params.machine_voltage_rms, electrical_params.machine_frequency,
        electrical_params.machine_num_poles);
}

void sfc_plant_model_augmented_6p_6p::configure_mechanical_parameters(
    const sfc_plant_model_mechanical_parameters& mechanical_params)
{
    _mechanical_params = mechanical_params;
    _mechanical_load.configure(mechanical_params.inertia_constant_h, mechanical_params.damping_factor_d);
}

void sfc_plant_model_augmented_6p_6p::configure_voltage_control_parameters(
    const sfc_plant_model_voltage_control_parameters& voltage_control_params)
{
    _voltage_control_params = voltage_control_params;
    _speed_voltage_relationship.configure_voltage_speed_coefficient(voltage_control_params.voltage_speed_coefficient);
    _speed_voltage_relationship.configure_voltage_offset_pu(voltage_control_params.voltage_offset);
    _speed_voltage_relationship.configure_voltage_limits_pu(0.1f, 1.5f);  // Reasonable default limits
}

void sfc_plant_model_augmented_6p_6p::configure_mechanical_load(
    inertia_constant_t inertia_constant_seconds, damping_coefficient_pu_t damping_coefficient)
{
    _mechanical_params.inertia_constant_h = inertia_constant_seconds;
    _mechanical_params.damping_factor_d = damping_coefficient;
    _mechanical_load.configure(inertia_constant_seconds, damping_coefficient);
}

void sfc_plant_model_augmented_6p_6p::configure_speed_voltage_relationship(
    real_t coefficient, real_t offset_pu, real_t time_constant_seconds)
{
    _voltage_control_params.voltage_speed_coefficient = coefficient;
    _voltage_control_params.voltage_offset = offset_pu;
    _speed_voltage_relationship.configure_voltage_speed_coefficient(coefficient);
    _speed_voltage_relationship.configure_voltage_offset_pu(offset_pu);
    _speed_voltage_relationship.configure_time_constant(duration_t(time_constant_seconds));
}

void sfc_plant_model_augmented_6p_6p::configure_voltage_limits_pu(real_t min_voltage_pu, real_t max_voltage_pu)
{
    _speed_voltage_relationship.configure_voltage_limits_pu(min_voltage_pu, max_voltage_pu);
}

void sfc_plant_model_augmented_6p_6p::initialize()
{
    _plant.initialize();
    _mechanical_load.reset(speed_pu_t{0.0});
    _speed_voltage_relationship.reset(1.0, 1.0);
}

void sfc_plant_model_augmented_6p_6p::update()
{
    _plant.update();
    step_mechanical_dynamics();
    step_voltage_control();
    update_plant_parameters();
}

void sfc_plant_model_augmented_6p_6p::update(const std::array<bool, 6>& grid_side_pulses,
    const std::array<bool, 6>& machine_side_pulses,
    torque_pu_t load_torque_pu,
    frequency_t grid_frequency_hz,
    frequency_t machine_frequency_hz,
    voltage_t grid_voltage_rms,
    voltage_t machine_voltage_rms)
{
    set_load_torque_pu(load_torque_pu);
    set_grid_side_pulses(grid_side_pulses);
    set_machine_side_pulses(machine_side_pulses);
    set_grid_parameters(grid_frequency_hz, grid_voltage_rms);
    set_machine_parameters(machine_frequency_hz, machine_voltage_rms);
    update();
}

void sfc_plant_model_augmented_6p_6p::terminate() { _plant.terminate(); }

void sfc_plant_model_augmented_6p_6p::set_grid_side_pulses(const std::array<bool, 6>& pulses)
{
    _plant.set_grid_side_pulses(pulses);
}

void sfc_plant_model_augmented_6p_6p::set_machine_side_pulses(const std::array<bool, 6>& pulses)
{
    _plant.set_machine_side_pulses(pulses);
}

void sfc_plant_model_augmented_6p_6p::set_grid_parameters(frequency_t frequency_hz, voltage_t voltage_rms)
{
    _plant.set_grid_parameters(frequency_hz, voltage_rms);
}

void sfc_plant_model_augmented_6p_6p::set_machine_parameters(frequency_t frequency_hz, voltage_t voltage_rms)
{
    _plant.set_machine_parameters(frequency_hz, voltage_rms);
}

void sfc_plant_model_augmented_6p_6p::set_operation_mode(plant_model_machine_operation_mode mode) { _mode = mode; }

void sfc_plant_model_augmented_6p_6p::set_voltage_control_mode(plant_model_voltage_control_mode mode)
{
    _voltage_mode = mode;
}

void sfc_plant_model_augmented_6p_6p::set_initial_mechanical_speed_pu(speed_pu_t initial_speed_pu)
{
    _mechanical_load.reset(initial_speed_pu);
    _speed_voltage_relationship.reset(1.0, 1.0);

    if (_mode == plant_model_machine_operation_mode::variable_speed) {
        frequency_pu_t initial_frequency_pu = per_unit_cast<frequency_pu_t>(initial_speed_pu);
        frequency_t machine_frequency_hz = to_si(initial_frequency_pu, _electrical_params.machine_frequency);

        voltage_t machine_voltage = _electrical_params.machine_voltage_rms;
        if (_voltage_mode == plant_model_voltage_control_mode::speed_dependent) {
            real_t voltage_magnitude_pu = _speed_voltage_relationship.get_voltage_magnitude_pu();
            machine_voltage = voltage_t(voltage_magnitude_pu * _electrical_params.machine_voltage_rms.value());
        }

        _plant.set_machine_parameters(machine_frequency_hz, machine_voltage);
    }
}

void sfc_plant_model_augmented_6p_6p::set_load_torque_pu(torque_pu_t load_torque_pu)
{
    _load_torque_pu = load_torque_pu;
}

// Output getters - Current model state
frequency_t sfc_plant_model_augmented_6p_6p::get_grid_frequency() const noexcept { return _plant.get_grid_frequency(); }
frequency_t sfc_plant_model_augmented_6p_6p::get_machine_frequency() const noexcept
{
    return _plant.get_machine_frequency();
}
voltage_t sfc_plant_model_augmented_6p_6p::get_rms_grid_voltage() const noexcept
{
    return _plant.get_rms_grid_voltage();
}
voltage_t sfc_plant_model_augmented_6p_6p::get_rms_machine_voltage() const noexcept
{
    return _plant.get_rms_machine_voltage();
}
abc<voltage_t> sfc_plant_model_augmented_6p_6p::get_grid_side_voltages_abc() const noexcept
{
    return _plant.get_grid_side_voltages_abc();
}
abc<current_t> sfc_plant_model_augmented_6p_6p::get_grid_side_currents_abc() const noexcept
{
    return _plant.get_grid_side_currents_abc();
}
abc<voltage_t> sfc_plant_model_augmented_6p_6p::get_machine_side_voltages_abc() const noexcept
{
    return _plant.get_machine_side_voltages_abc();
}
abc<current_t> sfc_plant_model_augmented_6p_6p::get_machine_side_currents_abc() const noexcept
{
    return _plant.get_machine_side_currents_abc();
}
angle_wrapped sfc_plant_model_augmented_6p_6p::get_grid_voltage_source_angle() const noexcept
{
    return _plant.get_grid_voltage_source_angle();
}
angle_wrapped sfc_plant_model_augmented_6p_6p::get_machine_voltage_source_angle() const noexcept
{
    return _plant.get_machine_voltage_source_angle();
}
current_t sfc_plant_model_augmented_6p_6p::get_dc_link_current() const noexcept { return _plant.get_dc_link_current(); }

// Mechanical dynamics getters
speed_pu_t sfc_plant_model_augmented_6p_6p::get_mechanical_speed_pu() const noexcept
{
    return _mechanical_load.get_speed_pu();
}

torque_pu_t sfc_plant_model_augmented_6p_6p::get_electromagnetic_torque_pu() const noexcept
{
    const current_pu_t dc_link_current = to_pu(_plant.get_dc_link_current(), _electrical_params.base_dc_link_current);
    return torque_pu_t{dc_link_current.value()};
}

torque_pu_t sfc_plant_model_augmented_6p_6p::get_load_torque_pu() const noexcept { return _load_torque_pu; }

// Voltage control getters
voltage_t sfc_plant_model_augmented_6p_6p::get_machine_voltage() const noexcept
{
    if (_voltage_mode == plant_model_voltage_control_mode::speed_dependent) {
        const real_t voltage_magnitude_pu = _speed_voltage_relationship.get_voltage_magnitude_pu();
        return voltage_t(voltage_magnitude_pu * _electrical_params.machine_voltage_rms.value());
    }
    else {
        return _electrical_params.machine_voltage_rms;
    }
}

real_t sfc_plant_model_augmented_6p_6p::get_voltage_magnitude_pu() const noexcept
{
    return _speed_voltage_relationship.get_voltage_magnitude_pu();
}

// Parameter getters - Current configuration
const sfc_plant_model_electrical_parameters& sfc_plant_model_augmented_6p_6p::get_electrical_parameters() const noexcept
{
    return _electrical_params;
}

const sfc_plant_model_mechanical_parameters& sfc_plant_model_augmented_6p_6p::get_mechanical_parameters() const noexcept
{
    return _mechanical_params;
}

const sfc_plant_model_voltage_control_parameters& sfc_plant_model_augmented_6p_6p::get_voltage_control_parameters()
    const noexcept
{
    return _voltage_control_params;
}

// Component access getters
const control::mechanical_load& sfc_plant_model_augmented_6p_6p::get_mechanical_load() const noexcept
{
    return _mechanical_load;
}

control::mechanical_load& sfc_plant_model_augmented_6p_6p::get_mechanical_load() noexcept { return _mechanical_load; }

const control::speed_voltage_relationship& sfc_plant_model_augmented_6p_6p::get_speed_voltage_relationship()
    const noexcept
{
    return _speed_voltage_relationship;
}

control::speed_voltage_relationship& sfc_plant_model_augmented_6p_6p::get_speed_voltage_relationship() noexcept
{
    return _speed_voltage_relationship;
}

const sfc_per_unit_system& sfc_plant_model_augmented_6p_6p::get_per_unit_system() const noexcept { return _pu_system; }

// Operation mode getters
plant_model_machine_operation_mode sfc_plant_model_augmented_6p_6p::get_operation_mode() const noexcept
{
    return _mode;
}

plant_model_voltage_control_mode sfc_plant_model_augmented_6p_6p::get_voltage_control_mode() const noexcept
{
    return _voltage_mode;
}

// Private factory methods for default parameters
sfc_plant_model_electrical_parameters sfc_plant_model_augmented_6p_6p::create_default_electrical_parameters() noexcept
{
    return sfc_plant_model_electrical_parameters{.grid_frequency = kNominalGridFrequency,
        .machine_frequency = kNominalMachineFrequency,
        .grid_voltage_rms = kNominalGridVoltageRMS,
        .machine_voltage_rms = kNominalMachineVoltageRMS,
        .grid_firing_angle_deg = kNominalGridFiringAngleDeg,
        .machine_firing_angle_deg = kNominalMachineFiringAngleDeg,
        .grid_side_resistance = kGridSideResistance,
        .grid_side_inductance = kGridSideInductance,
        .machine_side_resistance = kMachineSideResistance,
        .machine_side_inductance = kMachineSideInductance,
        .base_dc_link_current = kBaseDCLinkCurrent,
        .machine_rated_power_va = kMachineRatedPowerVA,
        .machine_num_poles = kMachineNumPoles};
}

sfc_plant_model_mechanical_parameters sfc_plant_model_augmented_6p_6p::create_default_mechanical_parameters() noexcept
{
    return sfc_plant_model_mechanical_parameters{
        .inertia_constant_h = kDefaultInertiaConstantH, .damping_factor_d = kDefaultDampingFactorD};
}

sfc_plant_model_voltage_control_parameters
sfc_plant_model_augmented_6p_6p::create_default_voltage_control_parameters() noexcept
{
    return sfc_plant_model_voltage_control_parameters{
        .voltage_speed_coefficient = kDefaultVoltageSpeedCoefficient, .voltage_offset = kDefaultVoltageOffset};
}

void sfc_plant_model_augmented_6p_6p::step_mechanical_dynamics()
{
    if (_mode != plant_model_machine_operation_mode::variable_speed) {
        return;
    }

    const current_pu_t dc_link_current = to_pu(_plant.get_dc_link_current(), _electrical_params.base_dc_link_current);
    const torque_pu_t electromagnetic_torque_pu = torque_pu_t{dc_link_current.value()};
    _mechanical_load.update(electromagnetic_torque_pu, _load_torque_pu);
}

void sfc_plant_model_augmented_6p_6p::step_voltage_control()
{
    if (_voltage_mode != plant_model_voltage_control_mode::speed_dependent) {
        return;
    }

    const speed_pu_t speed_input_pu = _mechanical_load.get_speed_pu();
    _speed_voltage_relationship.update(speed_input_pu);
}

void sfc_plant_model_augmented_6p_6p::update_plant_parameters()
{
    const frequency_t machine_frequency_hz =
        (_mode == plant_model_machine_operation_mode::variable_speed)
            ? to_si(frequency_pu_t{_mechanical_load.get_speed_pu()}, _electrical_params.machine_frequency)
            : _plant.get_machine_frequency();

    const voltage_t machine_voltage = get_machine_voltage();
    _plant.set_machine_parameters(machine_frequency_hz, machine_voltage);
}

}  // namespace yeet::application::sfc