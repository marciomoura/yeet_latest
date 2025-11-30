#include "sfc/sfc_plant_back_to_back_topology.hpp"

#include <cmath>

#include "yeet_control/constants.hpp"

namespace yeet::application::sfc {

namespace {
// Default parameters matching the Simulink model defaults

// Frequency and electrical parameters
static constexpr frequency_t kNominalGridFrequency = frequency_t(50.0);
static constexpr frequency_t kNominalMachineFrequency = frequency_t(50.0);
static constexpr voltage_t kNominalGridVoltageRMS = voltage_t(2090.0);
static constexpr voltage_t kNominalMachineVoltageRMS = voltage_t(1980.0);

// Firing angles
static constexpr angle_t kNominalGridFiringAngleDeg = angle_t(13.0);
static constexpr angle_t kNominalMachineFiringAngleDeg = angle_t(180.0 - 26.84);

// Circuit parameters - tuned for approximately 1.0 pu DC current at nominal operating point
// Target: ~1044 A at grid α=13°, machine α=153.16°
// Impedance scale factor of 0.41 achieves ~1.0 pu DC current
// These override any passed electrical_params to ensure consistent internal model behavior
static constexpr real_t kImpedanceScaleFactor = 0.41f;
static constexpr resistance_t kGridSideResistance = resistance_t(kImpedanceScaleFactor * 0.1552);
static constexpr inductance_t kGridSideInductance = inductance_t(kImpedanceScaleFactor * 0.4941e-3);
static constexpr resistance_t kMachineSideResistance = resistance_t(kImpedanceScaleFactor * 0.0788);
static constexpr inductance_t kMachineSideInductance = inductance_t(kImpedanceScaleFactor * 0.2508e-3);

// DC link parameters - inductance set higher (6 mH) to reduce current ripple for control loops
static constexpr resistance_t kDCLinkResistance = resistance_t(10e-3);
static constexpr inductance_t kDCLinkInductance = inductance_t(8e-3);

// Power and current ratings
static constexpr current_t kBaseDCLinkCurrent = current_t(1044.0);
static constexpr power_t kMachineRatedPowerVA = power_t(2950e3);
static constexpr int kMachineNumPoles = 2;

// Default Mechanical Parameters
static constexpr inertia_constant_t kDefaultInertiaConstantH = inertia_constant_t(2.5);
static constexpr damping_coefficient_pu_t kDefaultDampingFactorD = damping_coefficient_pu_t(0.01);

// Default Speed-Voltage Parameters
static constexpr real_t kDefaultVoltageSpeedCoefficient = 0.9f;
static constexpr real_t kDefaultVoltageOffset = 0.1f;

}  // anonymous namespace

sfc_plant_back_to_back_topology::sfc_plant_back_to_back_topology(
    const sfc_plant_model_electrical_parameters& electrical_params)
    : _electrical_params(electrical_params),
      _mechanical_params(create_default_mechanical_parameters()),
      _voltage_control_params(create_default_voltage_control_parameters())
{
    rebuild_converter();
}

sfc_plant_back_to_back_topology::sfc_plant_back_to_back_topology()
    : _electrical_params(create_default_electrical_parameters()),
      _mechanical_params(create_default_mechanical_parameters()),
      _voltage_control_params(create_default_voltage_control_parameters())
{
    rebuild_converter();
}

sfc_plant_back_to_back_topology::~sfc_plant_back_to_back_topology() = default;

void sfc_plant_back_to_back_topology::configure_electrical_parameters(
    const sfc_plant_model_electrical_parameters& electrical_params)
{
    _electrical_params = electrical_params;
    rebuild_converter();
}

void sfc_plant_back_to_back_topology::configure_mechanical_parameters(
    const sfc_plant_model_mechanical_parameters& mechanical_params)
{
    _mechanical_params = mechanical_params;
}

void sfc_plant_back_to_back_topology::configure_voltage_control_parameters(
    const sfc_plant_model_voltage_control_parameters& voltage_control_params)
{
    _voltage_control_params = voltage_control_params;
}

void sfc_plant_back_to_back_topology::initialize()
{
    // Reset the converter state
    if (_converter) {
        _converter->reset();
    }

    // Rebuild voltage generators to reset their state
    rebuild_converter();

    // Reset pulse states
    _grid_pulses = {};
    _machine_pulses = {};
}

void sfc_plant_back_to_back_topology::update()
{
    if (!_converter || !_grid_voltage_generator || !_machine_voltage_generator) {
        return;
    }

    // Update voltage generators
    _grid_voltage_generator->update();
    _machine_voltage_generator->update();

    // Get generated voltages (phase voltages in abc)
    auto grid_signal = _grid_voltage_generator->get_signal_abc();
    auto machine_signal = _machine_voltage_generator->get_signal_abc();

    // Convert to voltage_t abc
    _grid_voltages = abc<voltage_t>{voltage_t{static_cast<real_t>(grid_signal.a())},
        voltage_t{static_cast<real_t>(grid_signal.b())}, voltage_t{static_cast<real_t>(grid_signal.c())}};

    _machine_voltages = abc<voltage_t>{voltage_t{static_cast<real_t>(machine_signal.a())},
        voltage_t{static_cast<real_t>(machine_signal.b())}, voltage_t{static_cast<real_t>(machine_signal.c())}};

    // Convert pulses to thyristor_pulses struct
    control::thyristor_pulses grid_thyristor_pulses;
    grid_thyristor_pulses.t1 = _grid_pulses[0];
    grid_thyristor_pulses.t2 = _grid_pulses[1];
    grid_thyristor_pulses.t3 = _grid_pulses[2];
    grid_thyristor_pulses.t4 = _grid_pulses[3];
    grid_thyristor_pulses.t5 = _grid_pulses[4];
    grid_thyristor_pulses.t6 = _grid_pulses[5];

    control::thyristor_pulses machine_thyristor_pulses;
    machine_thyristor_pulses.t1 = _machine_pulses[0];
    machine_thyristor_pulses.t2 = _machine_pulses[1];
    machine_thyristor_pulses.t3 = _machine_pulses[2];
    machine_thyristor_pulses.t4 = _machine_pulses[3];
    machine_thyristor_pulses.t5 = _machine_pulses[4];
    machine_thyristor_pulses.t6 = _machine_pulses[5];

    // Update the converter
    _converter->update(_grid_voltages, _machine_voltages, grid_thyristor_pulses, machine_thyristor_pulses);
}

void sfc_plant_back_to_back_topology::update(const std::array<bool, 6>& grid_side_pulses,
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
    update();
}

void sfc_plant_back_to_back_topology::terminate()
{
    // Reset state on termination
    if (_converter) {
        _converter->reset();
    }
}

void sfc_plant_back_to_back_topology::set_grid_side_pulses(const std::array<bool, 6>& pulses) { _grid_pulses = pulses; }

void sfc_plant_back_to_back_topology::set_machine_side_pulses(const std::array<bool, 6>& pulses)
{
    _machine_pulses = pulses;
}

void sfc_plant_back_to_back_topology::set_grid_parameters(frequency_t frequency_hz, voltage_t voltage_rms)
{
    if (_grid_voltage_generator) {
        _grid_voltage_generator->set_signal_frequency(static_cast<double>(frequency_hz.value()));
        _grid_voltage_generator->set_fundamental_positive_sequence_signal_amplitude(
            yeet::line_to_phase_peak(voltage_rms));
    }
    _electrical_params.grid_frequency = frequency_hz;
    _electrical_params.grid_voltage_rms = voltage_rms;
}

void sfc_plant_back_to_back_topology::set_machine_parameters(frequency_t frequency_hz, voltage_t voltage_rms)
{
    if (_machine_voltage_generator) {
        _machine_voltage_generator->set_signal_frequency(static_cast<double>(frequency_hz.value()));
        _machine_voltage_generator->set_fundamental_positive_sequence_signal_amplitude(
            yeet::line_to_phase_peak(voltage_rms));
    }
    _electrical_params.machine_frequency = frequency_hz;
    _electrical_params.machine_voltage_rms = voltage_rms;
}

frequency_t sfc_plant_back_to_back_topology::get_grid_frequency() const noexcept
{
    return _electrical_params.grid_frequency;
}

frequency_t sfc_plant_back_to_back_topology::get_machine_frequency() const noexcept
{
    return _electrical_params.machine_frequency;
}

voltage_t sfc_plant_back_to_back_topology::get_rms_grid_voltage() const noexcept
{
    return _electrical_params.grid_voltage_rms;
}

voltage_t sfc_plant_back_to_back_topology::get_rms_machine_voltage() const noexcept
{
    return _electrical_params.machine_voltage_rms;
}

abc<voltage_t> sfc_plant_back_to_back_topology::get_grid_side_voltages_abc() const noexcept
{
    if (_converter) {
        return _converter->get_rectifier_bridge().get_ac_phase_voltages();
    }
    return _grid_voltages;
}

abc<current_t> sfc_plant_back_to_back_topology::get_grid_side_currents_abc() const noexcept
{
    if (_converter) {
        return _converter->get_grid_currents();
    }
    return abc<current_t>{current_t{0.0}, current_t{0.0}, current_t{0.0}};
}

abc<voltage_t> sfc_plant_back_to_back_topology::get_machine_side_voltages_abc() const noexcept
{
    if (_converter) {
        return _converter->get_inverter_bridge().get_ac_phase_voltages();
    }
    return _machine_voltages;
}

abc<current_t> sfc_plant_back_to_back_topology::get_machine_side_currents_abc() const noexcept
{
    if (_converter) {
        return _converter->get_machine_currents();
    }
    return abc<current_t>{current_t{0.0}, current_t{0.0}, current_t{0.0}};
}

angle_wrapped sfc_plant_back_to_back_topology::get_grid_voltage_source_angle() const noexcept
{
    if (_grid_voltage_generator) {
        return _grid_voltage_generator->get_fundamental_signal_angle().a();
    }
    return angle_wrapped{0.0};
}

angle_wrapped sfc_plant_back_to_back_topology::get_machine_voltage_source_angle() const noexcept
{
    if (_machine_voltage_generator) {
        return _machine_voltage_generator->get_fundamental_signal_angle().a();
    }
    return angle_wrapped{0.0};
}

current_t sfc_plant_back_to_back_topology::get_dc_link_current() const noexcept
{
    if (_converter) {
        return _converter->get_dc_current();
    }
    return current_t{0.0};
}

const sfc_plant_model_electrical_parameters& sfc_plant_back_to_back_topology::get_electrical_parameters() const noexcept
{
    return _electrical_params;
}

const sfc_plant_model_mechanical_parameters& sfc_plant_back_to_back_topology::get_mechanical_parameters() const noexcept
{
    return _mechanical_params;
}

const sfc_plant_model_voltage_control_parameters& sfc_plant_back_to_back_topology::get_voltage_control_parameters()
    const noexcept
{
    return _voltage_control_params;
}

sfc_plant_model_electrical_parameters sfc_plant_back_to_back_topology::create_default_electrical_parameters() noexcept
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

sfc_plant_model_mechanical_parameters sfc_plant_back_to_back_topology::create_default_mechanical_parameters() noexcept
{
    return sfc_plant_model_mechanical_parameters{
        .inertia_constant_h = kDefaultInertiaConstantH, .damping_factor_d = kDefaultDampingFactorD};
}

sfc_plant_model_voltage_control_parameters
sfc_plant_back_to_back_topology::create_default_voltage_control_parameters() noexcept
{
    return sfc_plant_model_voltage_control_parameters{
        .voltage_speed_coefficient = kDefaultVoltageSpeedCoefficient, .voltage_offset = kDefaultVoltageOffset};
}

void sfc_plant_back_to_back_topology::rebuild_converter()
{
    const double sampling_time = kSamplingTime;

    // Create voltage generators
    _grid_voltage_generator = std::make_unique<control::three_phase_waveform_generator>(sampling_time);
    _grid_voltage_generator->set_signal_frequency(static_cast<double>(_electrical_params.grid_frequency.value()));
    _grid_voltage_generator->set_fundamental_positive_sequence_signal_amplitude(
        yeet::line_to_phase_peak(_electrical_params.grid_voltage_rms));
    _grid_voltage_generator->set_fundamental_negative_sequence_signal_amplitude(0.0);
    _grid_voltage_generator->disable_noise();

    _machine_voltage_generator = std::make_unique<control::three_phase_waveform_generator>(sampling_time);
    _machine_voltage_generator->set_signal_frequency(static_cast<double>(_electrical_params.machine_frequency.value()));
    _machine_voltage_generator->set_fundamental_positive_sequence_signal_amplitude(
        yeet::line_to_phase_peak(_electrical_params.machine_voltage_rms));
    _machine_voltage_generator->set_fundamental_negative_sequence_signal_amplitude(0.0);
    _machine_voltage_generator->disable_noise();

    // Create the back-to-back converter using internal tuned constants
    // These override electrical_params to achieve consistent ~0.89 pu response
    _converter = std::make_unique<control::thyristor_back_to_back>(kGridSideResistance, kGridSideInductance,
        kDCLinkResistance, kDCLinkInductance, kMachineSideResistance, kMachineSideInductance,
        duration_t{static_cast<real_t>(sampling_time)});
}

}  // namespace yeet::application::sfc
