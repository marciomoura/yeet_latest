#include "sfc/sfc_plant_model_6p_6p_machine.hpp"

#include <algorithm>
#include <array>

#include "LCITopology6x6Minimal.h"  // Include the Simulink-generated header
#include "yeet_control/reference_frame.hpp"
#include "yeet_control/units.hpp"

namespace yeet::application::sfc {

namespace {
// Private namespace to hide default parameters - specific to 6p-6p machine topology

// Transformer parameters
static constexpr power_t kNominalTransformerPower = power_t(3362e3);            // 3.362 MVA
static constexpr frequency_t kNominalTransformerFrequency = frequency_t(50.0);  // 50 Hz
static constexpr voltage_t kTransformerHVVoltage = voltage_t(6600.0);           // 6.6 kV HV side
static constexpr resistance_pu_t kTransformerHVResistance = resistance_pu_t(0.0018);
static constexpr inductance_pu_t kTransformerHVInductance = inductance_pu_t(0.0629);
static constexpr voltage_t kTransformerLVVoltage = voltage_t(2200.0);  // 2.2 kV LV side
static constexpr resistance_pu_t kTransformerLVResistance = resistance_pu_t(0.0024);
static constexpr inductance_pu_t kTransformerLVInductance = inductance_pu_t(0.0839);
static constexpr resistance_pu_t kTransformerMagnetizationResistance = resistance_pu_t(500.0);
static constexpr inductance_pu_t kTransformerMagnetizationInductance = inductance_pu_t(500.0);

// DC link parameters
static constexpr resistance_t kDCLinkResistance = resistance_t(10e-3);  // 10 mΩ
static constexpr inductance_t kDCLinkInductance = inductance_t(8e-3);   // 8 mH

// Machine electrical parameters
static constexpr power_t kMachineNominalPower = power_t(2950e3);            // 2.95 MVA
static constexpr voltage_t kMachineNominalVoltage = voltage_t(1980.0);      // 1.98 kV
static constexpr frequency_t kMachineNominalFrequency = frequency_t(50.0);  // 50 Hz
static constexpr resistance_t kStatorResistance = resistance_t(0.0047);
static constexpr inductance_t kStatorLeakageInductance = inductance_t(0.00059);
static constexpr inductance_t kMagnetizingInductanceD = inductance_t(0.00808);
static constexpr inductance_t kMagnetizingInductanceQ = inductance_t(0.00808);
static constexpr inductance_t kCanayInductance = inductance_t(-0.00035);
static constexpr resistance_t kDamperResistanceD = resistance_t(0.136);
static constexpr inductance_t kDamperLeakageInductanceD = inductance_t(0.0015);
static constexpr resistance_t kDamperResistanceQ = resistance_t(0.136);
static constexpr inductance_t kDamperLeakageInductanceQ = inductance_t(0.0015);
static constexpr resistance_t kFieldResistance = resistance_t(0.0014);
static constexpr inductance_t kFieldLeakageInductance = inductance_t(0.00167);

// Machine mechanical parameters
static constexpr real_t kMachineInertiaCoefficient = 670.0;  // kg⋅m²
static constexpr int kMachinePolePairs = 1;

// Default measurement ratios
static constexpr real_t kDefaultGridPTRatio = 1.0;
static constexpr real_t kDefaultGridCTRatio = 1.0;
static constexpr real_t kDefaultMotorPTRatio = 1.0;
static constexpr real_t kDefaultMotorCTRatio = 1.0;

}  // anonymous namespace

sfc_plant_model_6p_6p_machine::sfc_plant_model_6p_6p_machine(
    const sfc_plant_model_transformer_parameters& transformer_params,
    const sfc_plant_model_dc_link_parameters& dc_link_params,
    const sfc_plant_model_machine_electrical_parameters& machine_electrical_params,
    const sfc_plant_model_machine_mechanical_parameters& machine_mechanical_params,
    const sfc_plant_model_measurement_parameters& measurement_params)
    : _model(std::make_unique<LCITopology6x6MinimalModelClass>()),
      _transformer_params(transformer_params),
      _dc_link_params(dc_link_params),
      _machine_electrical_params(machine_electrical_params),
      _machine_mechanical_params(machine_mechanical_params),
      _measurement_params(measurement_params),
      _cached_mechanical_state{},
      _cached_stator_state_dq{},
      _cached_rotor_state{}
{
    _model->initialize();

    // CRITICAL FIX: The Simulink-generated initialize() does not set the derivs pointer.
    // Manually set it to point to odeF[0] to fix the segfault in derivatives function.
    auto* rtm = _model->getRTM();
    rtm->derivs = rtm->odeF[0];

    // Initialize measurement ratios to prevent division by zero in Simulink outputs
    set_measurement_ratios(measurement_params.grid_pt_ratio, measurement_params.grid_ct_ratio,
        measurement_params.motor_pt_ratio, measurement_params.motor_ct_ratio);
}

sfc_plant_model_6p_6p_machine::sfc_plant_model_6p_6p_machine()
    : _model(std::make_unique<LCITopology6x6MinimalModelClass>()),
      _transformer_params(create_default_transformer_parameters()),
      _dc_link_params(create_default_dc_link_parameters()),
      _machine_electrical_params(create_default_machine_electrical_parameters()),
      _machine_mechanical_params(create_default_machine_mechanical_parameters()),
      _measurement_params(create_default_measurement_parameters()),
      _cached_mechanical_state{},
      _cached_stator_state_dq{},
      _cached_rotor_state{}
{
    _model->initialize();

    // CRITICAL FIX: The Simulink-generated initialize() does not set the derivs pointer.
    // Manually set it to point to odeF[0] to fix the segfault in derivatives function.
    auto* rtm = _model->getRTM();
    rtm->derivs = rtm->odeF[0];

    // Initialize measurement ratios to prevent division by zero in Simulink outputs
    set_measurement_ratios(kDefaultGridPTRatio, kDefaultGridCTRatio, kDefaultMotorPTRatio, kDefaultMotorCTRatio);
}

sfc_plant_model_6p_6p_machine::~sfc_plant_model_6p_6p_machine()
{
    if (_model) {
        _model->terminate();
    }
}

void sfc_plant_model_6p_6p_machine::configure_transformer_parameters(
    const sfc_plant_model_transformer_parameters& transformer_params)
{
    _transformer_params = transformer_params;
}

void sfc_plant_model_6p_6p_machine::configure_dc_link_parameters(
    const sfc_plant_model_dc_link_parameters& dc_link_params)
{
    _dc_link_params = dc_link_params;
}

void sfc_plant_model_6p_6p_machine::configure_machine_electrical_parameters(
    const sfc_plant_model_machine_electrical_parameters& machine_electrical_params)
{
    _machine_electrical_params = machine_electrical_params;
}

void sfc_plant_model_6p_6p_machine::configure_machine_mechanical_parameters(
    const sfc_plant_model_machine_mechanical_parameters& machine_mechanical_params)
{
    _machine_mechanical_params = machine_mechanical_params;
}

void sfc_plant_model_6p_6p_machine::configure_measurement_parameters(
    const sfc_plant_model_measurement_parameters& measurement_params)
{
    _measurement_params = measurement_params;
}

void sfc_plant_model_6p_6p_machine::initialize()
{
    _model->initialize();

    // CRITICAL FIX: The Simulink-generated initialize() does not set the derivs pointer.
    // Manually set it to point to odeF[0] to fix the segfault in derivatives function.
    auto* rtm = _model->getRTM();
    rtm->derivs = rtm->odeF[0];
}

void sfc_plant_model_6p_6p_machine::update()
{
    _model->step();
    update_cached_machine_states();
}

void sfc_plant_model_6p_6p_machine::update(const std::array<bool, 6>& grid_side_pulses,
    const std::array<bool, 6>& machine_side_pulses,
    voltage_t grid_voltage_mag_a,
    voltage_t grid_voltage_mag_b,
    voltage_t grid_voltage_mag_c,
    frequency_t grid_frequency,
    voltage_t field_voltage,
    const std::array<real_t, 3>& load_coefficients,
    real_t forced_speed_pu,
    bool enable_forced_speed,
    bool close_dc_isolator)
{
    set_grid_side_pulses(grid_side_pulses);
    set_machine_side_pulses(machine_side_pulses);
    set_grid_voltage_source(grid_voltage_mag_a, grid_voltage_mag_b, grid_voltage_mag_c, grid_frequency);
    set_field_voltage(field_voltage);
    set_load_coefficients(load_coefficients);
    set_forced_speed(forced_speed_pu, enable_forced_speed);
    set_dc_isolator(close_dc_isolator);
    _model->step();
    update_cached_machine_states();
}

void sfc_plant_model_6p_6p_machine::terminate() { _model->terminate(); }

void sfc_plant_model_6p_6p_machine::set_grid_side_pulses(const std::array<bool, 6>& pulses)
{
    for (size_t i = 0; i < 6; ++i) {
        _model->LCITopology6x6Minimal_U.pulses_in_grid[i] = static_cast<double>(pulses[i]);
    }
}

void sfc_plant_model_6p_6p_machine::set_machine_side_pulses(const std::array<bool, 6>& pulses)
{
    for (size_t i = 0; i < 6; ++i) {
        _model->LCITopology6x6Minimal_U.pulses_in_motor[i] = static_cast<double>(pulses[i]);
    }
}

void sfc_plant_model_6p_6p_machine::set_grid_voltage_source(
    voltage_t voltage_mag_a, voltage_t voltage_mag_b, voltage_t voltage_mag_c, frequency_t frequency)
{
    _model->LCITopology6x6Minimal_U.voltage_magnitude_a = voltage_mag_a.value();
    _model->LCITopology6x6Minimal_U.voltage_magnitude_b = voltage_mag_b.value();
    _model->LCITopology6x6Minimal_U.voltage_magnitude_c = voltage_mag_c.value();
    _model->LCITopology6x6Minimal_U.voltage_frequency = frequency.value();
}

void sfc_plant_model_6p_6p_machine::set_field_voltage(voltage_t field_voltage)
{
    _model->LCITopology6x6Minimal_U.voltage_excitation = field_voltage.value();
}

void sfc_plant_model_6p_6p_machine::set_load_coefficients(const std::array<real_t, 3>& load_coefficients)
{
    for (size_t i = 0; i < 3; ++i) {
        _model->LCITopology6x6Minimal_U.load_coefficients[i] = load_coefficients[i];
    }
}

void sfc_plant_model_6p_6p_machine::set_measurement_ratios(
    real_t grid_pt_ratio, real_t grid_ct_ratio, real_t motor_pt_ratio, real_t motor_ct_ratio)
{
    _model->LCITopology6x6Minimal_U.grid_pt_ratio = grid_pt_ratio;
    _model->LCITopology6x6Minimal_U.grid_ct_ratio = grid_ct_ratio;
    _model->LCITopology6x6Minimal_U.motor_pt_ratio = motor_pt_ratio;
    _model->LCITopology6x6Minimal_U.motor_ct_ratio = motor_ct_ratio;
}

void sfc_plant_model_6p_6p_machine::set_forced_speed(real_t speed_pu, bool enable)
{
    _model->LCITopology6x6Minimal_U.forced_speed_pu = speed_pu;
    _model->LCITopology6x6Minimal_U.enable_forced_speed = static_cast<double>(enable);
}

void sfc_plant_model_6p_6p_machine::set_dc_isolator(bool closed)
{
    _model->LCITopology6x6Minimal_U.close_dc_isolator = closed;
}

// Output getters - DC link
voltage_t sfc_plant_model_6p_6p_machine::get_dc_link_voltage() const noexcept
{
    const auto& outputs = _model->LCITopology6x6Minimal_Y;
    return voltage_t(outputs.Udc);
}

current_t sfc_plant_model_6p_6p_machine::get_dc_link_current() const noexcept
{
    const auto& outputs = _model->LCITopology6x6Minimal_Y;
    return current_t(outputs.Idc);
}

// Output getters - Grid side
abc<current_t> sfc_plant_model_6p_6p_machine::get_grid_side_currents_abc() const noexcept
{
    const auto& outputs = _model->LCITopology6x6Minimal_Y;
    const auto& currents = outputs.Iph_grid;
    return to_dimension_abc<current_t>(abc<real_t>(
        static_cast<real_t>(currents[0]), static_cast<real_t>(currents[1]), static_cast<real_t>(currents[2])));
}

abc<voltage_t> sfc_plant_model_6p_6p_machine::get_grid_side_voltages_line_to_line_abc() const noexcept
{
    const auto& outputs = _model->LCITopology6x6Minimal_Y;
    const auto& voltages = outputs.Uph_ph_grid;
    return to_dimension_abc<voltage_t>(abc<real_t>(
        static_cast<real_t>(voltages[0]), static_cast<real_t>(voltages[1]), static_cast<real_t>(voltages[2])));
}

angle_wrapped sfc_plant_model_6p_6p_machine::get_grid_voltage_source_angle() const noexcept
{
    const auto& outputs = _model->LCITopology6x6Minimal_Y;
    return angle_wrapped(outputs.wt);
}

abc<voltage_t> sfc_plant_model_6p_6p_machine::get_grid_voltage_source_phase_to_neutral_abc() const noexcept
{
    const auto& outputs = _model->LCITopology6x6Minimal_Y;
    const auto& voltages = outputs.Uph_n_source;
    return to_dimension_abc<voltage_t>(abc<real_t>(
        static_cast<real_t>(voltages[0]), static_cast<real_t>(voltages[1]), static_cast<real_t>(voltages[2])));
}

abc<current_t> sfc_plant_model_6p_6p_machine::get_grid_voltage_source_currents_abc() const noexcept
{
    const auto& outputs = _model->LCITopology6x6Minimal_Y;
    const auto& currents = outputs.Iph_source;
    return to_dimension_abc<current_t>(abc<real_t>(
        static_cast<real_t>(currents[0]), static_cast<real_t>(currents[1]), static_cast<real_t>(currents[2])));
}

// Output getters - Machine side converter
abc<current_t> sfc_plant_model_6p_6p_machine::get_machine_side_currents_abc() const noexcept
{
    const auto& outputs = _model->LCITopology6x6Minimal_Y;
    const auto& currents = outputs.Iph_motor;
    return to_dimension_abc<current_t>(abc<real_t>(
        static_cast<real_t>(currents[0]), static_cast<real_t>(currents[1]), static_cast<real_t>(currents[2])));
}

abc<voltage_t> sfc_plant_model_6p_6p_machine::get_machine_side_voltages_line_to_line_abc() const noexcept
{
    const auto& outputs = _model->LCITopology6x6Minimal_Y;
    const auto& voltages = outputs.Uph_ph_motor;
    return to_dimension_abc<voltage_t>(abc<real_t>(
        static_cast<real_t>(voltages[0]), static_cast<real_t>(voltages[1]), static_cast<real_t>(voltages[2])));
}

voltage_t sfc_plant_model_6p_6p_machine::get_machine_side_dc_voltage() const noexcept
{
    const auto& outputs = _model->LCITopology6x6Minimal_Y;
    return voltage_t(outputs.Udc_machine);
}

// Output getters - Machine states (grouped)
const machine_mechanical_state& sfc_plant_model_6p_6p_machine::get_machine_mechanical_state() const noexcept
{
    return _cached_mechanical_state;
}

const machine_stator_state_dq& sfc_plant_model_6p_6p_machine::get_machine_stator_state_dq() const noexcept
{
    return _cached_stator_state_dq;
}

abc<current_t> sfc_plant_model_6p_6p_machine::get_machine_stator_currents_abc() const noexcept
{
    const auto& outputs = _model->LCITopology6x6Minimal_Y;
    const auto& currents_a = outputs.machine_phase_current_a;
    const auto& currents_b = outputs.machine_phase_current_b;
    const auto& currents_c = outputs.machine_phase_current_c;
    return to_dimension_abc<current_t>(
        abc<real_t>(static_cast<real_t>(currents_a), static_cast<real_t>(currents_b), static_cast<real_t>(currents_c)));
}

const machine_rotor_state& sfc_plant_model_6p_6p_machine::get_machine_rotor_state() const noexcept
{
    return _cached_rotor_state;
}

angle_wrapped sfc_plant_model_6p_6p_machine::get_source_voltage_phase() const noexcept
{
    const auto& outputs = _model->LCITopology6x6Minimal_Y;
    return angle_wrapped(outputs.source_voltage_phase);
}

// Parameter getters - Current configuration values
const sfc_plant_model_transformer_parameters& sfc_plant_model_6p_6p_machine::get_transformer_parameters() const noexcept
{
    return _transformer_params;
}

const sfc_plant_model_dc_link_parameters& sfc_plant_model_6p_6p_machine::get_dc_link_parameters() const noexcept
{
    return _dc_link_params;
}

const sfc_plant_model_machine_electrical_parameters& sfc_plant_model_6p_6p_machine::get_machine_electrical_parameters()
    const noexcept
{
    return _machine_electrical_params;
}

const sfc_plant_model_machine_mechanical_parameters& sfc_plant_model_6p_6p_machine::get_machine_mechanical_parameters()
    const noexcept
{
    return _machine_mechanical_params;
}

const sfc_plant_model_measurement_parameters& sfc_plant_model_6p_6p_machine::get_measurement_parameters() const noexcept
{
    return _measurement_params;
}

// Private factory methods for default parameters
sfc_plant_model_transformer_parameters sfc_plant_model_6p_6p_machine::create_default_transformer_parameters() noexcept
{
    return sfc_plant_model_transformer_parameters{.nominal_power_va = kNominalTransformerPower,
        .nominal_frequency_hz = kNominalTransformerFrequency,
        .hv_phase_to_phase_rms_voltage = kTransformerHVVoltage,
        .hv_resistance_pu = kTransformerHVResistance,
        .hv_inductance_pu = kTransformerHVInductance,
        .lv_phase_to_phase_rms_voltage = kTransformerLVVoltage,
        .lv_resistance_pu = kTransformerLVResistance,
        .lv_inductance_pu = kTransformerLVInductance,
        .magnetization_resistance_pu = kTransformerMagnetizationResistance,
        .magnetization_inductance_pu = kTransformerMagnetizationInductance};
}

sfc_plant_model_dc_link_parameters sfc_plant_model_6p_6p_machine::create_default_dc_link_parameters() noexcept
{
    return sfc_plant_model_dc_link_parameters{.resistance = kDCLinkResistance, .inductance = kDCLinkInductance};
}

sfc_plant_model_machine_electrical_parameters
sfc_plant_model_6p_6p_machine::create_default_machine_electrical_parameters() noexcept
{
    return sfc_plant_model_machine_electrical_parameters{.nominal_power_va = kMachineNominalPower,
        .nominal_phase_to_phase_voltage = kMachineNominalVoltage,
        .nominal_frequency_hz = kMachineNominalFrequency,
        .stator_resistance = kStatorResistance,
        .stator_leakage_inductance = kStatorLeakageInductance,
        .magnetizing_inductance_d = kMagnetizingInductanceD,
        .magnetizing_inductance_q = kMagnetizingInductanceQ,
        .canay_inductance = kCanayInductance,
        .damper_resistance_d = kDamperResistanceD,
        .damper_leakage_inductance_d = kDamperLeakageInductanceD,
        .damper_resistance_q = kDamperResistanceQ,
        .damper_leakage_inductance_q = kDamperLeakageInductanceQ,
        .field_resistance = kFieldResistance,
        .field_leakage_inductance = kFieldLeakageInductance};
}

sfc_plant_model_machine_mechanical_parameters
sfc_plant_model_6p_6p_machine::create_default_machine_mechanical_parameters() noexcept
{
    return sfc_plant_model_machine_mechanical_parameters{
        .inertia_coefficient = kMachineInertiaCoefficient, .pole_pairs = kMachinePolePairs};
}

sfc_plant_model_measurement_parameters sfc_plant_model_6p_6p_machine::create_default_measurement_parameters() noexcept
{
    return sfc_plant_model_measurement_parameters{.grid_pt_ratio = kDefaultGridPTRatio,
        .grid_ct_ratio = kDefaultGridCTRatio,
        .motor_pt_ratio = kDefaultMotorPTRatio,
        .motor_ct_ratio = kDefaultMotorCTRatio};
}

void sfc_plant_model_6p_6p_machine::update_cached_machine_states()
{
    const auto& outputs = _model->LCITopology6x6Minimal_Y;

    // Update mechanical state
    _cached_mechanical_state.speed = speed_t(outputs.machine_speed);
    _cached_mechanical_state.electrical_torque = torque_t(outputs.machine_electrical_torque);
    _cached_mechanical_state.mechanical_angle = angle_t(outputs.machine_mechanical_angle);
    _cached_mechanical_state.load_torque = torque_t(outputs.machine_load);
    _cached_mechanical_state.electrical_power = power_t(outputs.machine_electrical_power);  // Update stator state
    _cached_stator_state_dq.current_d = current_t(outputs.machine_stator_current_d);
    _cached_stator_state_dq.current_q = current_t(outputs.machine_stator_current_q);
    _cached_stator_state_dq.voltage_d = voltage_t(outputs.machine_stator_voltage_d);
    _cached_stator_state_dq.voltage_q = voltage_t(outputs.machine_stator_voltage_q);

    // Update rotor state
    _cached_rotor_state.flux_d = magnetic_flux_t(outputs.machine_flux_d);
    _cached_rotor_state.flux_q = magnetic_flux_t(outputs.machine_flux_q);
    _cached_rotor_state.field_current_d = current_t(outputs.machine_field_current_d);
    _cached_rotor_state.damper_current_q1 = current_t(outputs.machine_damper_current_q1);
    _cached_rotor_state.damper_current_q2 = current_t(outputs.machine_damper_current_q2);
    _cached_rotor_state.damper_current_d = current_t(outputs.machine_damper_current_d);
}

}  // namespace yeet::application::sfc
