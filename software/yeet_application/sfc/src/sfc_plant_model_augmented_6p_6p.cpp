#include "sfc/sfc_plant_model_augmented_6p_6p.hpp"

#include <algorithm>  // std::min
#include <cmath>      // std::abs

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
static constexpr real_t kDefaultVoltageSpeedCoefficient = 0.9f;  // 0.9 pu/pu voltage-speed coefficient (legacy)
static constexpr real_t kDefaultVoltageOffset =
    0.1f;  // 0.1 pu minimum voltage offset at standstill (resistive component)

}  // anonymous namespace

sfc_plant_model_augmented_6p_6p::sfc_plant_model_augmented_6p_6p(
    const sfc_plant_model_electrical_parameters& electrical_params,
    const sfc_plant_model_mechanical_parameters& mechanical_params,
    const sfc_plant_model_voltage_control_parameters& voltage_control_params)
    : _mechanical_load(duration_t(kSamplingTime)),
      _field_current_regulator(duration_t(kSamplingTime)),
      _speed_filter(kSamplingTime),
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

    // Configure field current regulator with realistic parameters for large LCI machines
    _field_current_regulator.configure_field_circuit(
        resistance_pu_t{0.01},  // 1% field resistance (typical for large synchronous machines)
        duration_t{2.5}         // 2.5s field time constant (L/R)
    );
    _field_current_regulator.configure_pi_gains(1.0, 2.0);                // Moderate bandwidth for stable control
    _field_current_regulator.configure_voltage_limits(voltage_pu_t{0.0},  // Minimum exciter voltage
        voltage_pu_t{2.0}                                                 // Maximum exciter voltage (200% rated)
    );

    // Configure speed filter with 10ms time constant to smooth back-EMF oscillations
    // fc = 1/(2πτ) where τ = 10ms → fc ≈ 15.9 Hz
    constexpr double time_constant_s = 10e-3;  // 10 ms
    const auto cutoff_freq = control::first_order_low_pass_filter<speed_pu_t>::calculate_cutoff_frequency(
        speed_pu_t{static_cast<float>(time_constant_s)});
    _speed_filter.configure(cutoff_freq.value());

    // Capture nominal bases for per-unit runtime scaling
    _nominal_grid_voltage_rms = electrical_params.grid_voltage_rms;
    _nominal_machine_voltage_rms = electrical_params.machine_voltage_rms;
    _nominal_grid_frequency = electrical_params.grid_frequency;
    _nominal_machine_frequency = electrical_params.machine_frequency;
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
    // Note: voltage_speed_coefficient and voltage_offset are no longer used
    // Back-EMF is calculated directly from E = Φ × ω in step_voltage_control()
}

void sfc_plant_model_augmented_6p_6p::configure_mechanical_load(
    inertia_constant_t inertia_constant_seconds, damping_coefficient_pu_t damping_coefficient)
{
    _mechanical_params.inertia_constant_h = inertia_constant_seconds;
    _mechanical_params.damping_factor_d = damping_coefficient;
    _mechanical_load.configure(inertia_constant_seconds, damping_coefficient);
}

void sfc_plant_model_augmented_6p_6p::initialize()
{
    _plant.initialize();
    _mechanical_load.reset(speed_pu_t{0.0});
    _speed_filter.reset(0.0f);  // Reset speed filter to zero
    _machine_back_emf_pu = voltage_pu_t{0.0};
    // Reset runtime per-unit scalars to nominal
    _grid_voltage_pu = voltage_pu_t{1.0f};
    _machine_voltage_pu = voltage_pu_t{1.0f};
    _grid_frequency_pu = frequency_pu_t{1.0f};
    _machine_frequency_pu = frequency_pu_t{1.0f};
}

void sfc_plant_model_augmented_6p_6p::update()
{
    step_mechanical_dynamics();
    step_voltage_control();
    update_plant_parameters();

    _plant.update();
}

void sfc_plant_model_augmented_6p_6p::update(const std::array<bool, 6>& grid_side_pulses,
    const std::array<bool, 6>& machine_side_pulses,
    torque_pu_t load_torque,
    frequency_t grid_frequency_hz,
    frequency_t machine_frequency_hz,
    voltage_t grid_voltage_rms,
    voltage_t machine_voltage_rms)
{
    set_load_torque_pu(load_torque);
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
    // Convert given SI values into per-unit scalars for consistency
    _grid_frequency_pu = frequency_pu_t{real_t(frequency_hz.value() / _nominal_grid_frequency.value())};
    _grid_voltage_pu = voltage_pu_t{real_t(voltage_rms.value() / _nominal_grid_voltage_rms.value())};
    _plant.set_grid_parameters(frequency_hz, voltage_rms);
}

void sfc_plant_model_augmented_6p_6p::set_machine_parameters(frequency_t frequency_hz, voltage_t voltage_rms)
{
    _machine_frequency_pu = frequency_pu_t{real_t(frequency_hz.value() / _nominal_machine_frequency.value())};
    _machine_voltage_pu = voltage_pu_t{real_t(voltage_rms.value() / _nominal_machine_voltage_rms.value())};
    _plant.set_machine_parameters(frequency_hz, voltage_rms);
}

// ==================== Per-Unit Runtime Setters ====================
void sfc_plant_model_augmented_6p_6p::set_grid_voltage_pu(voltage_pu_t v_pu) noexcept { _grid_voltage_pu = v_pu; }
void sfc_plant_model_augmented_6p_6p::set_grid_frequency_pu(frequency_pu_t f_pu) noexcept { _grid_frequency_pu = f_pu; }
void sfc_plant_model_augmented_6p_6p::set_grid_operating_point(voltage_pu_t v_pu, frequency_pu_t f_pu) noexcept
{
    _grid_voltage_pu = v_pu;
    _grid_frequency_pu = f_pu;
}
void sfc_plant_model_augmented_6p_6p::set_machine_voltage_pu(voltage_pu_t v_pu) noexcept { _machine_voltage_pu = v_pu; }
void sfc_plant_model_augmented_6p_6p::set_machine_frequency_pu(frequency_pu_t f_pu) noexcept
{
    _machine_frequency_pu = f_pu;
}
void sfc_plant_model_augmented_6p_6p::set_machine_operating_point(voltage_pu_t v_pu, frequency_pu_t f_pu) noexcept
{
    _machine_voltage_pu = v_pu;
    _machine_frequency_pu = f_pu;
}

void sfc_plant_model_augmented_6p_6p::reset() { initialize(); }

void sfc_plant_model_augmented_6p_6p::set_operation_mode(plant_model_machine_operation_mode mode) { _mode = mode; }

void sfc_plant_model_augmented_6p_6p::set_voltage_control_mode(plant_model_voltage_control_mode mode)
{
    _voltage_mode = mode;
}

void sfc_plant_model_augmented_6p_6p::set_field_current_reference_pu(current_pu_t field_current_reference)
{
    _field_current_reference_pu = field_current_reference;
}

void sfc_plant_model_augmented_6p_6p::set_initial_mechanical_speed_pu(speed_pu_t initial_speed_pu)
{
    _mechanical_load.reset(initial_speed_pu);

    // Initialize speed filter to initial speed to avoid transients
    _speed_filter.reset(initial_speed_pu.value());

    // Initialize back-EMF based on initial speed (assuming nominal flux)
    // E = Φ × ω, with nominal flux ≈ 1.0 pu, so E ≈ ω
    _machine_back_emf_pu = per_unit_cast<voltage_pu_t>(initial_speed_pu);

    if (_mode == plant_model_machine_operation_mode::variable_speed) {
        frequency_pu_t initial_frequency_pu = per_unit_cast<frequency_pu_t>(initial_speed_pu);
        frequency_t machine_frequency_hz = to_si(initial_frequency_pu, _electrical_params.machine_frequency);

        voltage_t machine_voltage_abc = _electrical_params.machine_voltage_rms;
        if (_voltage_mode == plant_model_voltage_control_mode::variable_speed) {
            // Use the calculated back-EMF as machine voltage
            machine_voltage_abc = to_si(_machine_back_emf_pu, _electrical_params.machine_voltage_rms);
        }

        _plant.set_machine_parameters(machine_frequency_hz, machine_voltage_abc);
    }
}

void sfc_plant_model_augmented_6p_6p::set_load_torque_pu(torque_pu_t load_torque) { _load_torque_pu = load_torque; }

// Output getters - Current model state
frequency_t sfc_plant_model_augmented_6p_6p::get_grid_frequency() const noexcept { return resolve_grid_frequency(); }
frequency_t sfc_plant_model_augmented_6p_6p::get_machine_frequency() const noexcept
{
    return resolve_machine_frequency();
}
voltage_t sfc_plant_model_augmented_6p_6p::get_rms_grid_voltage() const noexcept { return resolve_grid_voltage(); }
voltage_t sfc_plant_model_augmented_6p_6p::get_rms_machine_voltage() const noexcept
{
    return resolve_machine_voltage();
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
voltage_t sfc_plant_model_augmented_6p_6p::get_machine_voltage() const noexcept { return resolve_machine_voltage(); }

real_t sfc_plant_model_augmented_6p_6p::get_voltage_magnitude_pu() const noexcept
{
    return _machine_back_emf_pu.value();
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
    if (_voltage_mode == plant_model_voltage_control_mode::variable_speed) {
        // Variable speed voltage control with enhanced physical model
        // At standstill: V ≈ V_offset × Φ (resistive voltage for flux establishment)
        // At high speed: V ≈ Φ × ω (back-EMF dominant)
        // Smooth blending between resistive-dominant and EMF-dominant regions

        // Step 1: Update field current regulator (PI control + field circuit dynamics)
        const current_pu_t actual_field_current = _field_current_regulator.update(_field_current_reference_pu);

        // Step 2: Calculate flux from field current
        // Assumes linear magnetization curve: Φ = k × i_f
        // TODO: Add saturation effects via lookup table for more accuracy
        const real_t flux_pu = actual_field_current.value();

        // Step 3: Filter the mechanical speed to reduce back-EMF oscillations
        // Raw speed from mechanical dynamics can have high-frequency oscillations
        const speed_pu_t raw_speed = _mechanical_load.get_speed_pu();
        const speed_pu_t filtered_speed = _speed_filter.update(raw_speed);

        // Step 4: Calculate voltage with smooth transition from resistive to EMF regions
        const real_t speed_pu_abs = std::abs(filtered_speed.value());
        constexpr real_t transition_speed = 0.05;  // Transition occurs between 0 and 0.15 pu speed

        // Blending factor: 0 at standstill, 1 at transition_speed and above
        const real_t blend = std::min(speed_pu_abs / transition_speed, 1.0f);

        // Resistive voltage component (dominant at low speed): V_r = V_offset × Φ
        const real_t resistive_voltage_pu = _voltage_control_params.voltage_offset * flux_pu;

        // Back-EMF component (dominant at high speed): E = Φ × ω
        const real_t back_emf_pu = flux_pu * filtered_speed.value();

        // Total voltage with smooth blending: V = V_r × (1-blend) + E × blend
        const real_t total_voltage_pu = resistive_voltage_pu * (1.0f - blend) + back_emf_pu * blend;

        // Step 5: Store the calculated voltage magnitude
        _machine_back_emf_pu = voltage_pu_t{total_voltage_pu};
    }
    // else: fixed_voltage mode - no voltage control needed
}

void sfc_plant_model_augmented_6p_6p::update_plant_parameters()
{
    // Push both grid and machine derived SI parameters into underlying plant.
    _plant.set_grid_parameters(resolve_grid_frequency(), resolve_grid_voltage());
    _plant.set_machine_parameters(resolve_machine_frequency(), resolve_machine_voltage());
}

// ==================== Override Resolution Helpers ====================
frequency_t sfc_plant_model_augmented_6p_6p::resolve_machine_frequency() const noexcept
{
    if (_mode == plant_model_machine_operation_mode::variable_speed) {
        return to_si(per_unit_cast<frequency_pu_t>(_mechanical_load.get_speed_pu()), _nominal_machine_frequency);
    }
    return frequency_t(_machine_frequency_pu.value() * _nominal_machine_frequency.value());
}

voltage_t sfc_plant_model_augmented_6p_6p::resolve_machine_voltage() const noexcept
{
    if (_voltage_mode == plant_model_voltage_control_mode::variable_speed) {
        // Variable speed mode: voltage calculated by field current regulator (E = Φ × ω)
        return to_si(_machine_back_emf_pu, _nominal_machine_voltage_rms);
    }
    // fixed_voltage mode: use configured voltage
    return voltage_t(_machine_voltage_pu.value() * _nominal_machine_voltage_rms.value());
}

frequency_t sfc_plant_model_augmented_6p_6p::resolve_grid_frequency() const noexcept
{
    return frequency_t(_grid_frequency_pu.value() * _nominal_grid_frequency.value());
}

voltage_t sfc_plant_model_augmented_6p_6p::resolve_grid_voltage() const noexcept
{
    return voltage_t(_grid_voltage_pu.value() * _nominal_grid_voltage_rms.value());
}

// ==================== Per-Unit Runtime Getters ====================
voltage_pu_t sfc_plant_model_augmented_6p_6p::get_grid_voltage_pu() const noexcept { return _grid_voltage_pu; }
voltage_pu_t sfc_plant_model_augmented_6p_6p::get_machine_voltage_pu() const noexcept { return _machine_voltage_pu; }
frequency_pu_t sfc_plant_model_augmented_6p_6p::get_grid_frequency_pu() const noexcept { return _grid_frequency_pu; }
frequency_pu_t sfc_plant_model_augmented_6p_6p::get_machine_frequency_pu() const noexcept
{
    if (_mode == plant_model_machine_operation_mode::variable_speed) {
        return per_unit_cast<frequency_pu_t>(_mechanical_load.get_speed_pu());
    }
    return _machine_frequency_pu;
}

current_pu_t sfc_plant_model_augmented_6p_6p::get_field_current() const noexcept
{
    return _field_current_regulator.get_field_current();
}

voltage_pu_t sfc_plant_model_augmented_6p_6p::get_exciter_voltage() const noexcept
{
    return _field_current_regulator.get_exciter_voltage();
}

real_t sfc_plant_model_augmented_6p_6p::get_flux_pu() const noexcept
{
    // Flux is proportional to field current (assuming linear magnetization)
    // Φ = k × i_f, where k is absorbed into per-unit system
    return _field_current_regulator.get_field_current().value();
}

}  // namespace yeet::application::sfc