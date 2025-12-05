#include "sfc/sfc_plant_rectifier_topology.hpp"

#include <cmath>

#include "yeet_control/constants.hpp"

namespace yeet::application::sfc {

namespace {
// Default parameters for rectifier-only operation

// Frequency and electrical parameters
static constexpr frequency_t kNominalGridFrequency = frequency_t(50.0);
static constexpr voltage_t kNominalGridVoltageRMS = voltage_t(2090.0);

// Firing angles
static constexpr angle_t kNominalGridFiringAngleDeg = angle_t(13.0);

// Circuit parameters - tuned for approximately 1.0 pu DC current at nominal operating point
static constexpr real_t kImpedanceScaleFactor = 0.41f;
static constexpr resistance_t kGridSideResistance = resistance_t(kImpedanceScaleFactor * 0.1552);
static constexpr inductance_t kGridSideInductance = inductance_t(kImpedanceScaleFactor * 0.4941e-3);

// DC link parameters - inductance set higher to reduce current ripple for control loops
// For rectifier-only mode, load resistance + inductance replaces machine-side impedance
static constexpr resistance_t kDCLinkResistance = resistance_t(10e-3);
static constexpr inductance_t kDCLinkInductance = inductance_t(8e-3);

// Power and current ratings
static constexpr current_t kBaseDCLinkCurrent = current_t(1044.0);
static constexpr power_t kMachineRatedPowerVA = power_t(2950e3);
static constexpr int kMachineNumPoles = 2;

// Default Mechanical Parameters (not used in rectifier-only mode but required by interface)
static constexpr inertia_constant_t kDefaultInertiaConstantH = inertia_constant_t(2.5);
static constexpr damping_coefficient_pu_t kDefaultDampingFactorD = damping_coefficient_pu_t(0.01);

// Default Speed-Voltage Parameters (not used in rectifier-only mode)
static constexpr real_t kDefaultVoltageSpeedCoefficient = 0.9f;
static constexpr real_t kDefaultVoltageOffset = 0.1f;

}  // anonymous namespace

sfc_plant_rectifier_topology::sfc_plant_rectifier_topology(
    const sfc_plant_model_electrical_parameters& electrical_params)
    : _electrical_params(electrical_params),
      _mechanical_params(create_default_mechanical_parameters()),
      _voltage_control_params(create_default_voltage_control_parameters())
{
    rebuild_rectifier();
}

sfc_plant_rectifier_topology::sfc_plant_rectifier_topology()
    : _electrical_params(create_default_electrical_parameters()),
      _mechanical_params(create_default_mechanical_parameters()),
      _voltage_control_params(create_default_voltage_control_parameters())
{
    rebuild_rectifier();
}

sfc_plant_rectifier_topology::~sfc_plant_rectifier_topology() = default;

void sfc_plant_rectifier_topology::configure_electrical_parameters(
    const sfc_plant_model_electrical_parameters& electrical_params)
{
    _electrical_params = electrical_params;
    rebuild_rectifier();
}

void sfc_plant_rectifier_topology::configure_mechanical_parameters(
    const sfc_plant_model_mechanical_parameters& mechanical_params)
{
    _mechanical_params = mechanical_params;
    // No effect in rectifier-only mode
}

void sfc_plant_rectifier_topology::configure_voltage_control_parameters(
    const sfc_plant_model_voltage_control_parameters& voltage_control_params)
{
    _voltage_control_params = voltage_control_params;
    // No effect in rectifier-only mode
}

void sfc_plant_rectifier_topology::initialize()
{
    // Reset the rectifier state
    if (_rectifier) {
        _rectifier->reset();
    }

    // Rebuild to reset generator state
    rebuild_rectifier();

    // Reset pulse states
    _grid_pulses = {};

    // Reset machine angle
    _machine_angle = angle_wrapped{0.0};
}

void sfc_plant_rectifier_topology::update()
{
    if (!_rectifier || !_grid_voltage_generator) {
        return;
    }

    // Update voltage generator
    _grid_voltage_generator->update();

    // Get generated voltages (phase voltages in abc)
    auto grid_signal = _grid_voltage_generator->get_signal_abc();

    // Convert to voltage_t abc
    _grid_voltages = abc<voltage_t>{voltage_t{static_cast<real_t>(grid_signal.a())},
        voltage_t{static_cast<real_t>(grid_signal.b())}, voltage_t{static_cast<real_t>(grid_signal.c())}};

    // Apply circuit breaker via pulse masking - when open, block all gate pulses
    // This allows conducting thyristors to naturally commutate rather than
    // unrealistically forcing the voltage to zero (which would be a short-circuit to ground)
    control::thyristor_pulses effective_grid_pulses{};
    if (_grid_circuit_breaker_closed) {
        effective_grid_pulses.t1 = _grid_pulses[0];
        effective_grid_pulses.t2 = _grid_pulses[1];
        effective_grid_pulses.t3 = _grid_pulses[2];
        effective_grid_pulses.t4 = _grid_pulses[3];
        effective_grid_pulses.t5 = _grid_pulses[4];
        effective_grid_pulses.t6 = _grid_pulses[5];
    }
    // When breaker is open, effective_grid_pulses is all false (default-initialized)

    // Update the rectifier with actual voltage and effective pulses (circuit breaker applied)
    _rectifier->update(_grid_voltages, effective_grid_pulses, voltage_t{0.0});
}

void sfc_plant_rectifier_topology::update(const std::array<bool, 6>& grid_side_pulses,
    const std::array<bool, 6>& machine_side_pulses,
    frequency_t grid_frequency_hz,
    frequency_t machine_frequency_hz,
    voltage_t grid_voltage_rms,
    voltage_t machine_voltage_rms)
{
    // Machine-side pulses and voltage are ignored in rectifier-only mode
    (void)machine_side_pulses;
    (void)machine_voltage_rms;

    set_grid_side_pulses(grid_side_pulses);
    set_grid_parameters(grid_frequency_hz, grid_voltage_rms);

    // Store machine frequency for angle tracking
    _electrical_params.machine_frequency = machine_frequency_hz;

    update();
}

void sfc_plant_rectifier_topology::terminate()
{
    // Reset state on termination
    if (_rectifier) {
        _rectifier->reset();
    }
}

void sfc_plant_rectifier_topology::set_grid_side_pulses(const std::array<bool, 6>& pulses) { _grid_pulses = pulses; }

void sfc_plant_rectifier_topology::set_machine_side_pulses(const std::array<bool, 6>& pulses)
{
    // Machine-side pulses are ignored in rectifier-only mode
    (void)pulses;
}

void sfc_plant_rectifier_topology::set_grid_parameters(frequency_t frequency_hz, voltage_t voltage_rms)
{
    if (_grid_voltage_generator) {
        _grid_voltage_generator->set_signal_frequency(static_cast<double>(frequency_hz.value()));
        _grid_voltage_generator->set_fundamental_positive_sequence_signal_amplitude(
            yeet::line_to_phase_peak(voltage_rms));
    }
    _electrical_params.grid_frequency = frequency_hz;
    _electrical_params.grid_voltage_rms = voltage_rms;
}

void sfc_plant_rectifier_topology::set_machine_parameters(frequency_t frequency_hz, voltage_t voltage_rms)
{
    // Machine voltage is ignored (always zero in rectifier-only mode)
    // Store frequency for angle tracking
    _electrical_params.machine_frequency = frequency_hz;
    _electrical_params.machine_voltage_rms = voltage_rms;  // Stored but not used
}

frequency_t sfc_plant_rectifier_topology::get_grid_frequency() const noexcept
{
    return _electrical_params.grid_frequency;
}

frequency_t sfc_plant_rectifier_topology::get_machine_frequency() const noexcept
{
    return _electrical_params.machine_frequency;
}

voltage_t sfc_plant_rectifier_topology::get_rms_grid_voltage() const noexcept
{
    return _electrical_params.grid_voltage_rms;
}

voltage_t sfc_plant_rectifier_topology::get_rms_machine_voltage() const noexcept
{
    // Always return zero - machine side is short-circuited
    return voltage_t{0.0};
}

abc<voltage_t> sfc_plant_rectifier_topology::get_grid_side_voltages_abc() const noexcept
{
    if (_rectifier) {
        return _rectifier->get_ac_phase_voltages();
    }
    return _grid_voltages;
}

abc<current_t> sfc_plant_rectifier_topology::get_grid_side_currents_abc() const noexcept
{
    if (_rectifier) {
        return _rectifier->get_grid_currents();
    }
    return abc<current_t>{current_t{0.0}, current_t{0.0}, current_t{0.0}};
}

abc<voltage_t> sfc_plant_rectifier_topology::get_machine_side_voltages_abc() const noexcept
{
    // Machine side is short-circuited - return zeros
    return abc<voltage_t>{voltage_t{0.0}, voltage_t{0.0}, voltage_t{0.0}};
}

abc<current_t> sfc_plant_rectifier_topology::get_machine_side_currents_abc() const noexcept
{
    // In rectifier-only mode, machine currents equal DC current through the short circuit
    // Return the same as grid currents (DC current flows through the whole loop)
    if (_rectifier) {
        return _rectifier->get_grid_currents();
    }
    return abc<current_t>{current_t{0.0}, current_t{0.0}, current_t{0.0}};
}

angle_wrapped sfc_plant_rectifier_topology::get_grid_voltage_source_angle() const noexcept
{
    if (_grid_voltage_generator) {
        return _grid_voltage_generator->get_fundamental_signal_angle().a();
    }
    return angle_wrapped{0.0};
}

angle_wrapped sfc_plant_rectifier_topology::get_machine_voltage_source_angle() const noexcept
{
    // Return tracked machine angle for interface compatibility
    return _machine_angle;
}

current_t sfc_plant_rectifier_topology::get_dc_link_current() const noexcept
{
    if (_rectifier) {
        return _rectifier->get_dc_current();
    }
    return current_t{0.0};
}

const sfc_plant_model_electrical_parameters& sfc_plant_rectifier_topology::get_electrical_parameters() const noexcept
{
    return _electrical_params;
}

const sfc_plant_model_mechanical_parameters& sfc_plant_rectifier_topology::get_mechanical_parameters() const noexcept
{
    return _mechanical_params;
}

const sfc_plant_model_voltage_control_parameters& sfc_plant_rectifier_topology::get_voltage_control_parameters()
    const noexcept
{
    return _voltage_control_params;
}

void sfc_plant_rectifier_topology::set_grid_circuit_breaker_closed(bool closed)
{
    // When breaker transitions from closed to open, do not forcibly reset the rectifier.
    // Let the pulse masking logic in update() handle the commutation-off process naturally.
    // This more accurately models the physical behavior of thyristor rectifiers.
    _grid_circuit_breaker_closed = closed;

    if (!_grid_circuit_breaker_closed) {
        _rectifier->reset();
    }
}

bool sfc_plant_rectifier_topology::is_grid_circuit_breaker_closed() const noexcept
{
    return _grid_circuit_breaker_closed;
}

sfc_plant_model_electrical_parameters sfc_plant_rectifier_topology::create_default_electrical_parameters() noexcept
{
    return sfc_plant_model_electrical_parameters{.grid_frequency = kNominalGridFrequency,
        .machine_frequency = frequency_t{0.0},  // Not used in rectifier-only mode
        .grid_voltage_rms = kNominalGridVoltageRMS,
        .machine_voltage_rms = voltage_t{0.0},  // Always zero (short-circuited)
        .grid_firing_angle_deg = kNominalGridFiringAngleDeg,
        .machine_firing_angle_deg = angle_t{0.0},  // Not used
        .grid_side_resistance = kGridSideResistance,
        .grid_side_inductance = kGridSideInductance,
        .machine_side_resistance = resistance_t{0.0},  // Not used
        .machine_side_inductance = inductance_t{0.0},  // Not used
        .base_dc_link_current = kBaseDCLinkCurrent,
        .machine_rated_power_va = kMachineRatedPowerVA,
        .machine_num_poles = kMachineNumPoles};
}

sfc_plant_model_mechanical_parameters sfc_plant_rectifier_topology::create_default_mechanical_parameters() noexcept
{
    return sfc_plant_model_mechanical_parameters{
        .inertia_constant_h = kDefaultInertiaConstantH, .damping_factor_d = kDefaultDampingFactorD};
}

sfc_plant_model_voltage_control_parameters
sfc_plant_rectifier_topology::create_default_voltage_control_parameters() noexcept
{
    return sfc_plant_model_voltage_control_parameters{
        .voltage_speed_coefficient = kDefaultVoltageSpeedCoefficient, .voltage_offset = kDefaultVoltageOffset};
}

void sfc_plant_rectifier_topology::rebuild_rectifier()
{
    const double sampling_time = kSamplingTime;

    // Create voltage generator (grid side only)
    _grid_voltage_generator = std::make_unique<control::three_phase_waveform_generator>(sampling_time);
    _grid_voltage_generator->set_signal_frequency(static_cast<double>(_electrical_params.grid_frequency.value()));
    _grid_voltage_generator->set_fundamental_positive_sequence_signal_amplitude(
        yeet::line_to_phase_peak(_electrical_params.grid_voltage_rms));
    _grid_voltage_generator->set_fundamental_negative_sequence_signal_amplitude(0.0);
    _grid_voltage_generator->disable_noise();

    // Create the rectifier model
    // Grid-side impedance + DC link impedance as the load
    // The load "sees" the DC link resistance and inductance
    _rectifier = std::make_unique<control::thyristor_rectifier>(kGridSideResistance, kGridSideInductance,
        kDCLinkResistance, kDCLinkInductance, duration_t{static_cast<real_t>(sampling_time)});
}

}  // namespace yeet::application::sfc
