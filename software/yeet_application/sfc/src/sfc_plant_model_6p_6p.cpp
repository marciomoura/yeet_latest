#include "sfc/sfc_plant_model_6p_6p.hpp"

#include <algorithm>
#include <array>

#include "sfc_6p_6p_topology.h"  // Include the Simulink-generated header
#include "yeet_control/reference_frame.hpp"
#include "yeet_control/units.hpp"

namespace yeet::application::sfc {

namespace {
// Private namespace to hide default parameters - specific to 6p-6p topology

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
static constexpr resistance_t kDcLinkResistance = resistance_t(10e-3);           // 10 mOhm DC link resistance
static constexpr inductance_t kDcLinkInductance = inductance_t(8e-3);            // 8 mH DC link inductance

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

sfc_plant_model_6p_6p::sfc_plant_model_6p_6p(const sfc_plant_model_electrical_parameters& electrical_params)
    : _model(std::make_unique<generated::sfc::sfc_6p_6p_topology>()),
      _electrical_params(electrical_params),
      _mechanical_params(create_default_mechanical_parameters()),
      _voltage_control_params(create_default_voltage_control_parameters())
{
    _model->initialize();
}

sfc_plant_model_6p_6p::sfc_plant_model_6p_6p()
    : _model(std::make_unique<generated::sfc::sfc_6p_6p_topology>()),
      _electrical_params(create_default_6p_6p_electrical_parameters()),
      _mechanical_params(create_default_mechanical_parameters()),
      _voltage_control_params(create_default_voltage_control_parameters())
{
    _model->initialize();
}

sfc_plant_model_6p_6p::~sfc_plant_model_6p_6p()
{
    if (_model) {
        _model->terminate();
    }
}

void sfc_plant_model_6p_6p::configure_electrical_parameters(
    const sfc_plant_model_electrical_parameters& electrical_params)
{
    _electrical_params = electrical_params;
}

void sfc_plant_model_6p_6p::configure_mechanical_parameters(
    const sfc_plant_model_mechanical_parameters& mechanical_params)
{
    _mechanical_params = mechanical_params;
}

void sfc_plant_model_6p_6p::configure_voltage_control_parameters(
    const sfc_plant_model_voltage_control_parameters& voltage_control_params)
{
    _voltage_control_params = voltage_control_params;
}

void sfc_plant_model_6p_6p::initialize() { _model->initialize(); }

void sfc_plant_model_6p_6p::update() { _model->step(); }

void sfc_plant_model_6p_6p::update(const std::array<bool, 6>& grid_side_pulses,
    const std::array<bool, 6>& machine_side_pulses,
    frequency_t grid_frequency_hz,
    frequency_t machine_frequency_hz,
    voltage_t grid_voltage_rms,
    voltage_t machine_voltage_rms)
{
    set_grid_side_pulses(grid_side_pulses);
    set_machine_side_pulses(machine_side_pulses);
    set_grid_parameters(grid_frequency_hz, grid_voltage_rms);
    set_machine_parameters(machine_frequency_hz, machine_voltage_rms);
    _model->step();
}

void sfc_plant_model_6p_6p::terminate() { _model->terminate(); }

void sfc_plant_model_6p_6p::set_grid_side_pulses(const std::array<bool, 6>& pulses)
{
    std::array<double, 6> double_pulses{};
    std::transform(
        pulses.cbegin(), pulses.cend(), double_pulses.begin(), [](bool b) { return static_cast<double>(b); });
    _model->sfc_6p_6p_topology_U.grid_side_converter_pulses = double_pulses;
}

void sfc_plant_model_6p_6p::set_machine_side_pulses(const std::array<bool, 6>& pulses)
{
    std::array<double, 6> double_pulses{};
    std::transform(
        pulses.cbegin(), pulses.cend(), double_pulses.begin(), [](bool b) { return static_cast<double>(b); });
    _model->sfc_6p_6p_topology_U.machine_side_converter_pulses = double_pulses;
}

void sfc_plant_model_6p_6p::set_grid_parameters(frequency_t frequency_hz, voltage_t voltage_rms)
{
    _model->sfc_6p_6p_topology_U.grid_voltage_source_frequency_hz = frequency_hz;
    _model->sfc_6p_6p_topology_U.grid_voltage_source_line_voltage_rms = voltage_rms;
}

void sfc_plant_model_6p_6p::set_machine_parameters(frequency_t frequency_hz, voltage_t voltage_rms)
{
    _model->sfc_6p_6p_topology_U.machine_voltage_source_frequency_hz = frequency_hz;
    _model->sfc_6p_6p_topology_U.machine_voltage_source_line_voltage_rms = voltage_rms;
}

// Output getters - Current model state
frequency_t sfc_plant_model_6p_6p::get_grid_frequency() const noexcept
{
    return frequency_t(_model->sfc_6p_6p_topology_U.grid_voltage_source_frequency_hz);
}

frequency_t sfc_plant_model_6p_6p::get_machine_frequency() const noexcept
{
    return frequency_t(_model->sfc_6p_6p_topology_U.machine_voltage_source_frequency_hz);
}

voltage_t sfc_plant_model_6p_6p::get_rms_grid_voltage() const noexcept
{
    return voltage_t(_model->sfc_6p_6p_topology_U.grid_voltage_source_line_voltage_rms);
}

voltage_t sfc_plant_model_6p_6p::get_rms_machine_voltage() const noexcept
{
    return voltage_t(_model->sfc_6p_6p_topology_U.machine_voltage_source_line_voltage_rms);
}

abc<voltage_t> sfc_plant_model_6p_6p::get_grid_side_voltages_abc() const noexcept
{
    const auto& voltages = _model->sfc_6p_6p_topology_Y.grid_side_converter_voltage_phase_si;
    return to_dimension_abc<voltage_t>(abc<real_t>{
        static_cast<real_t>(voltages[0]), static_cast<real_t>(voltages[1]), static_cast<real_t>(voltages[2])});
}

abc<current_t> sfc_plant_model_6p_6p::get_grid_side_currents_abc() const noexcept
{
    const auto& currents = _model->sfc_6p_6p_topology_Y.grid_side_converter_current_si;
    return to_dimension_abc<current_t>(abc<real_t>{
        static_cast<real_t>(currents[0]), static_cast<real_t>(currents[1]), static_cast<real_t>(currents[2])});
}

abc<voltage_t> sfc_plant_model_6p_6p::get_machine_side_voltages_abc() const noexcept
{
    const auto& voltages = _model->sfc_6p_6p_topology_Y.machine_side_converter_voltage_phase_si;
    return to_dimension_abc<voltage_t>(abc<real_t>{
        static_cast<real_t>(voltages[0]), static_cast<real_t>(voltages[1]), static_cast<real_t>(voltages[2])});
}

abc<current_t> sfc_plant_model_6p_6p::get_machine_side_currents_abc() const noexcept
{
    const auto& currents = _model->sfc_6p_6p_topology_Y.machine_side_converter_current_si;
    return to_dimension_abc<current_t>(abc<real_t>{
        static_cast<real_t>(currents[0]), static_cast<real_t>(currents[1]), static_cast<real_t>(currents[2])});
}

angle_wrapped sfc_plant_model_6p_6p::get_grid_voltage_source_angle() const noexcept
{
    return angle_wrapped(_model->sfc_6p_6p_topology_Y.grid_voltage_source_angle_wt);
}

angle_wrapped sfc_plant_model_6p_6p::get_machine_voltage_source_angle() const noexcept
{
    return angle_wrapped(_model->sfc_6p_6p_topology_Y.machine_voltage_source_angle_wt);
}

current_t sfc_plant_model_6p_6p::get_dc_link_current() const noexcept
{
    return current_t(_model->sfc_6p_6p_topology_Y.dc_link_current_si);
}

// Parameter getters - Current configuration values
const sfc_plant_model_electrical_parameters& sfc_plant_model_6p_6p::get_electrical_parameters() const noexcept
{
    return _electrical_params;
}

const sfc_plant_model_mechanical_parameters& sfc_plant_model_6p_6p::get_mechanical_parameters() const noexcept
{
    return _mechanical_params;
}

const sfc_plant_model_voltage_control_parameters& sfc_plant_model_6p_6p::get_voltage_control_parameters() const noexcept
{
    return _voltage_control_params;
}

// Private factory methods for default parameters
sfc_plant_model_electrical_parameters sfc_plant_model_6p_6p::create_default_6p_6p_electrical_parameters() noexcept
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
        .dc_link_resistance = kDcLinkResistance,
        .dc_link_inductance = kDcLinkInductance,
        .base_dc_link_current = kBaseDCLinkCurrent,
        .machine_rated_power_va = kMachineRatedPowerVA,
        .machine_num_poles = kMachineNumPoles};
}

sfc_plant_model_mechanical_parameters sfc_plant_model_6p_6p::create_default_mechanical_parameters() noexcept
{
    return sfc_plant_model_mechanical_parameters{
        .inertia_constant_h = kDefaultInertiaConstantH, .damping_factor_d = kDefaultDampingFactorD};
}

sfc_plant_model_voltage_control_parameters sfc_plant_model_6p_6p::create_default_voltage_control_parameters() noexcept
{
    return sfc_plant_model_voltage_control_parameters{
        .voltage_speed_coefficient = kDefaultVoltageSpeedCoefficient, .voltage_offset = kDefaultVoltageOffset};
}

}  // namespace yeet::application::sfc
