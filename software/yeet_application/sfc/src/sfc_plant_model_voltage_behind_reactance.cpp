#include "sfc/sfc_plant_model_voltage_behind_reactance.hpp"

#include <algorithm>  // std::min
#include <cmath>      // std::abs

#include "sfc/sfc_plant_back_to_back_topology.hpp"
#include "sfc/sfc_plant_back_to_back_topology_simulink.hpp"
#include "sfc/sfc_plant_rectifier_topology.hpp"

namespace yeet::application::sfc {

namespace {

std::unique_ptr<sfc_plant_back_to_back_topology_interface> create_topology(
    plant_topology_type topology, const sfc_plant_model_electrical_parameters& electrical_params)
{
    switch (topology) {
        case plant_topology_type::simulink:
            return std::make_unique<sfc_plant_back_to_back_topology_simulink>(electrical_params);
        case plant_topology_type::internal:
            return std::make_unique<sfc_plant_back_to_back_topology>(electrical_params);
        case plant_topology_type::rectifier:
            return std::make_unique<sfc_plant_rectifier_topology>(electrical_params);
    }
    // Default to simulink if unknown
    return std::make_unique<sfc_plant_back_to_back_topology_simulink>(electrical_params);
}

std::unique_ptr<sfc_plant_back_to_back_topology_interface> create_default_topology(plant_topology_type topology)
{
    switch (topology) {
        case plant_topology_type::simulink:
            return std::make_unique<sfc_plant_back_to_back_topology_simulink>();
        case plant_topology_type::internal:
            return std::make_unique<sfc_plant_back_to_back_topology>();
        case plant_topology_type::rectifier:
            return std::make_unique<sfc_plant_rectifier_topology>();
    }
    // Default to simulink if unknown
    return std::make_unique<sfc_plant_back_to_back_topology_simulink>();
}

}  // namespace

sfc_plant_model_voltage_behind_reactance::sfc_plant_model_voltage_behind_reactance(
    const sfc_plant_model_electrical_parameters& electrical_params,
    const sfc_plant_model_mechanical_parameters& mechanical_params,
    const sfc_plant_model_voltage_control_parameters& voltage_control_params,
    plant_topology_type topology)
    : _plant(create_topology(topology, electrical_params)),
      _topology_type(topology),
      _mechanical_load(duration_t(kSamplingTime)),
      _field_current_regulator(duration_t(kSamplingTime)),
      _speed_filter(kSamplingTime),
      _pu_system(electrical_params.grid_voltage_rms,
          electrical_params.grid_frequency,
          electrical_params.base_dc_link_current,
          electrical_params.machine_rated_power_va,
          electrical_params.machine_voltage_rms,
          electrical_params.machine_frequency,
          electrical_params.machine_num_poles),
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

sfc_plant_model_voltage_behind_reactance::sfc_plant_model_voltage_behind_reactance(plant_topology_type topology)
    : sfc_plant_model_voltage_behind_reactance(create_default_electrical_parameters(),
          create_default_mechanical_parameters(),
          create_default_voltage_control_parameters(),
          topology)
{
}

sfc_plant_model_voltage_behind_reactance::~sfc_plant_model_voltage_behind_reactance() = default;

void sfc_plant_model_voltage_behind_reactance::configure_electrical_parameters(
    const sfc_plant_model_electrical_parameters& electrical_params)
{
    _electrical_params = electrical_params;
    _plant->configure_electrical_parameters(electrical_params);

    // Update per-unit system with new parameters
    _pu_system = sfc_per_unit_system(electrical_params.grid_voltage_rms, electrical_params.grid_frequency,
        electrical_params.base_dc_link_current, electrical_params.machine_rated_power_va,
        electrical_params.machine_voltage_rms, electrical_params.machine_frequency,
        electrical_params.machine_num_poles);
}

void sfc_plant_model_voltage_behind_reactance::configure_mechanical_parameters(
    const sfc_plant_model_mechanical_parameters& mechanical_params)
{
    _mechanical_params = mechanical_params;
    _mechanical_load.configure(mechanical_params.inertia_constant_h, mechanical_params.damping_factor_d,
        mechanical_params.linear_load_coefficient, mechanical_params.quadratic_load_coefficient);
}

void sfc_plant_model_voltage_behind_reactance::configure_voltage_control_parameters(
    const sfc_plant_model_voltage_control_parameters& voltage_control_params)
{
    _voltage_control_params = voltage_control_params;
    // Note: voltage_speed_coefficient and voltage_offset are no longer used
    // Back-EMF is calculated directly from E = Φ × ω in step_voltage_control()
}

void sfc_plant_model_voltage_behind_reactance::configure_mechanical_load(inertia_constant_t inertia_constant_seconds,
    damping_coefficient_pu_t damping_coefficient,
    real_t linear_load_coeff,
    real_t quadratic_load_coeff)
{
    _mechanical_params.inertia_constant_h = inertia_constant_seconds;
    _mechanical_params.damping_factor_d = damping_coefficient;
    _mechanical_params.linear_load_coefficient = linear_load_coeff;
    _mechanical_params.quadratic_load_coefficient = quadratic_load_coeff;
    _mechanical_load.configure(inertia_constant_seconds, damping_coefficient, linear_load_coeff, quadratic_load_coeff);
}

void sfc_plant_model_voltage_behind_reactance::initialize()
{
    _plant->initialize();
    _mechanical_load.reset(speed_pu_t{0.0});
    _speed_filter.reset(0.0f);  // Reset speed filter to zero
    _machine_back_emf_pu = voltage_pu_t{0.0};
    // Reset runtime per-unit scalars to nominal
    _grid_voltage_pu = voltage_pu_t{1.0f};
    _machine_voltage_pu = voltage_pu_t{1.0f};
    _grid_frequency_pu = frequency_pu_t{1.0f};
    _machine_frequency_pu = frequency_pu_t{1.0f};
}

void sfc_plant_model_voltage_behind_reactance::update()
{
    step_mechanical_dynamics();
    step_voltage_control();
    update_plant_parameters();

    _plant->update();
}

void sfc_plant_model_voltage_behind_reactance::update(const std::array<bool, 6>& grid_side_pulses,
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

void sfc_plant_model_voltage_behind_reactance::terminate() { _plant->terminate(); }

void sfc_plant_model_voltage_behind_reactance::set_grid_side_pulses(const std::array<bool, 6>& pulses)
{
    _plant->set_grid_side_pulses(pulses);
}

void sfc_plant_model_voltage_behind_reactance::set_machine_side_pulses(const std::array<bool, 6>& pulses)
{
    _plant->set_machine_side_pulses(pulses);
}

void sfc_plant_model_voltage_behind_reactance::set_grid_parameters(frequency_t frequency_hz, voltage_t voltage_rms)
{
    // Convert given SI values into per-unit scalars for consistency
    _grid_frequency_pu = frequency_pu_t{real_t(frequency_hz.value() / _nominal_grid_frequency.value())};
    _grid_voltage_pu = voltage_pu_t{real_t(voltage_rms.value() / _nominal_grid_voltage_rms.value())};
    _plant->set_grid_parameters(frequency_hz, voltage_rms);
}

void sfc_plant_model_voltage_behind_reactance::set_machine_parameters(frequency_t frequency_hz, voltage_t voltage_rms)
{
    _machine_frequency_pu = frequency_pu_t{real_t(frequency_hz.value() / _nominal_machine_frequency.value())};
    _machine_voltage_pu = voltage_pu_t{real_t(voltage_rms.value() / _nominal_machine_voltage_rms.value())};
    _plant->set_machine_parameters(frequency_hz, voltage_rms);
}

// ==================== Per-Unit Runtime Setters ====================
void sfc_plant_model_voltage_behind_reactance::set_grid_voltage_pu(voltage_pu_t v_pu) noexcept
{
    _grid_voltage_pu = v_pu;
}
void sfc_plant_model_voltage_behind_reactance::set_grid_frequency_pu(frequency_pu_t f_pu) noexcept
{
    _grid_frequency_pu = f_pu;
}
void sfc_plant_model_voltage_behind_reactance::set_grid_operating_point(voltage_pu_t v_pu, frequency_pu_t f_pu) noexcept
{
    _grid_voltage_pu = v_pu;
    _grid_frequency_pu = f_pu;
}
void sfc_plant_model_voltage_behind_reactance::set_machine_voltage_pu(voltage_pu_t v_pu) noexcept
{
    _machine_voltage_pu = v_pu;
}
void sfc_plant_model_voltage_behind_reactance::set_machine_frequency_pu(frequency_pu_t f_pu) noexcept
{
    _machine_frequency_pu = f_pu;
}
void sfc_plant_model_voltage_behind_reactance::set_machine_operating_point(
    voltage_pu_t v_pu, frequency_pu_t f_pu) noexcept
{
    _machine_voltage_pu = v_pu;
    _machine_frequency_pu = f_pu;
}

void sfc_plant_model_voltage_behind_reactance::reset() { initialize(); }

void sfc_plant_model_voltage_behind_reactance::set_operation_mode(plant_model_machine_operation_mode mode)
{
    _mode = mode;
}

void sfc_plant_model_voltage_behind_reactance::set_voltage_control_mode(plant_model_voltage_control_mode mode)
{
    _voltage_mode = mode;
}

void sfc_plant_model_voltage_behind_reactance::set_field_current_reference_pu(current_pu_t field_current_reference)
{
    _field_current_reference_pu = field_current_reference;
}

void sfc_plant_model_voltage_behind_reactance::set_initial_mechanical_speed_pu(speed_pu_t initial_speed_pu)
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

        _plant->set_machine_parameters(machine_frequency_hz, machine_voltage_abc);
    }
}

void sfc_plant_model_voltage_behind_reactance::set_load_torque_pu(torque_pu_t load_torque)
{
    _load_torque_pu = load_torque;
}

// Output getters - Current model state
frequency_t sfc_plant_model_voltage_behind_reactance::get_grid_frequency() const noexcept
{
    return resolve_grid_frequency();
}
frequency_t sfc_plant_model_voltage_behind_reactance::get_machine_frequency() const noexcept
{
    return resolve_machine_frequency();
}
voltage_t sfc_plant_model_voltage_behind_reactance::get_rms_grid_voltage() const noexcept
{
    return resolve_grid_voltage();
}
voltage_t sfc_plant_model_voltage_behind_reactance::get_rms_machine_voltage() const noexcept
{
    return resolve_machine_voltage();
}
abc<voltage_t> sfc_plant_model_voltage_behind_reactance::get_grid_side_voltages_abc() const noexcept
{
    return _plant->get_grid_side_voltages_abc();
}
abc<current_t> sfc_plant_model_voltage_behind_reactance::get_grid_side_currents_abc() const noexcept
{
    return _plant->get_grid_side_currents_abc();
}
abc<voltage_t> sfc_plant_model_voltage_behind_reactance::get_machine_side_voltages_abc() const noexcept
{
    return _plant->get_machine_side_voltages_abc();
}
abc<current_t> sfc_plant_model_voltage_behind_reactance::get_machine_side_currents_abc() const noexcept
{
    return _plant->get_machine_side_currents_abc();
}
angle_wrapped sfc_plant_model_voltage_behind_reactance::get_grid_voltage_source_angle() const noexcept
{
    return _plant->get_grid_voltage_source_angle();
}
angle_wrapped sfc_plant_model_voltage_behind_reactance::get_machine_voltage_source_angle() const noexcept
{
    return _plant->get_machine_voltage_source_angle();
}
current_t sfc_plant_model_voltage_behind_reactance::get_dc_link_current() const noexcept
{
    return _plant->get_dc_link_current();
}

// Mechanical dynamics getters
speed_pu_t sfc_plant_model_voltage_behind_reactance::get_mechanical_speed_pu() const noexcept
{
    return _mechanical_load.get_speed_pu();
}

torque_pu_t sfc_plant_model_voltage_behind_reactance::get_electromagnetic_torque_pu() const noexcept
{
    const current_pu_t dc_link_current = to_pu(_plant->get_dc_link_current(), _electrical_params.base_dc_link_current);
    return torque_pu_t{dc_link_current.value()};
}

torque_pu_t sfc_plant_model_voltage_behind_reactance::get_load_torque_pu() const noexcept { return _load_torque_pu; }

// Voltage control getters
voltage_t sfc_plant_model_voltage_behind_reactance::get_machine_voltage() const noexcept
{
    return resolve_machine_voltage();
}

real_t sfc_plant_model_voltage_behind_reactance::get_voltage_magnitude_pu() const noexcept
{
    return _machine_back_emf_pu.value();
}

// Parameter getters - Current configuration
const sfc_plant_model_electrical_parameters& sfc_plant_model_voltage_behind_reactance::get_electrical_parameters()
    const noexcept
{
    return _electrical_params;
}

const sfc_plant_model_mechanical_parameters& sfc_plant_model_voltage_behind_reactance::get_mechanical_parameters()
    const noexcept
{
    return _mechanical_params;
}

const sfc_plant_model_voltage_control_parameters&
sfc_plant_model_voltage_behind_reactance::get_voltage_control_parameters() const noexcept
{
    return _voltage_control_params;
}

// Component access getters
const control::mechanical_load& sfc_plant_model_voltage_behind_reactance::get_mechanical_load() const noexcept
{
    return _mechanical_load;
}

control::mechanical_load& sfc_plant_model_voltage_behind_reactance::get_mechanical_load() noexcept
{
    return _mechanical_load;
}

const sfc_per_unit_system& sfc_plant_model_voltage_behind_reactance::get_per_unit_system() const noexcept
{
    return _pu_system;
}

// Operation mode getters
plant_model_machine_operation_mode sfc_plant_model_voltage_behind_reactance::get_operation_mode() const noexcept
{
    return _mode;
}

plant_model_voltage_control_mode sfc_plant_model_voltage_behind_reactance::get_voltage_control_mode() const noexcept
{
    return _voltage_mode;
}

plant_topology_type sfc_plant_model_voltage_behind_reactance::get_topology_type() const noexcept
{
    return _topology_type;
}

// Private factory methods for default parameters
// Delegate to the default topology to get consistent parameters
sfc_plant_model_electrical_parameters
sfc_plant_model_voltage_behind_reactance::create_default_electrical_parameters() noexcept
{
    // Use Simulink topology as the default source of parameters
    auto default_topology = create_default_topology(plant_topology_type::simulink);
    return default_topology->get_electrical_parameters();
}

sfc_plant_model_mechanical_parameters
sfc_plant_model_voltage_behind_reactance::create_default_mechanical_parameters() noexcept
{
    auto default_topology = create_default_topology(plant_topology_type::simulink);
    return default_topology->get_mechanical_parameters();
}

sfc_plant_model_voltage_control_parameters
sfc_plant_model_voltage_behind_reactance::create_default_voltage_control_parameters() noexcept
{
    auto default_topology = create_default_topology(plant_topology_type::simulink);
    return default_topology->get_voltage_control_parameters();
}

void sfc_plant_model_voltage_behind_reactance::step_mechanical_dynamics()
{
    if (_mode != plant_model_machine_operation_mode::variable_speed) {
        return;
    }

    const current_pu_t dc_link_current = to_pu(_plant->get_dc_link_current(), _electrical_params.base_dc_link_current);
    const torque_pu_t electromagnetic_torque_pu = torque_pu_t{dc_link_current.value()};
    _mechanical_load.update(electromagnetic_torque_pu, _load_torque_pu);
}

void sfc_plant_model_voltage_behind_reactance::step_voltage_control()
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

void sfc_plant_model_voltage_behind_reactance::update_plant_parameters()
{
    // Push both grid and machine derived SI parameters into underlying plant.
    _plant->set_grid_parameters(resolve_grid_frequency(), resolve_grid_voltage());
    _plant->set_machine_parameters(resolve_machine_frequency(), resolve_machine_voltage());
}

// ==================== Override Resolution Helpers ====================
frequency_t sfc_plant_model_voltage_behind_reactance::resolve_machine_frequency() const noexcept
{
    if (_mode == plant_model_machine_operation_mode::variable_speed) {
        return to_si(per_unit_cast<frequency_pu_t>(_mechanical_load.get_speed_pu()), _nominal_machine_frequency);
    }
    return frequency_t(_machine_frequency_pu.value() * _nominal_machine_frequency.value());
}

voltage_t sfc_plant_model_voltage_behind_reactance::resolve_machine_voltage() const noexcept
{
    if (_voltage_mode == plant_model_voltage_control_mode::variable_speed) {
        // Variable speed mode: voltage calculated by field current regulator (E = Φ × ω)
        return to_si(_machine_back_emf_pu, _nominal_machine_voltage_rms);
    }
    // fixed_voltage mode: use configured voltage
    return voltage_t(_machine_voltage_pu.value() * _nominal_machine_voltage_rms.value());
}

frequency_t sfc_plant_model_voltage_behind_reactance::resolve_grid_frequency() const noexcept
{
    return frequency_t(_grid_frequency_pu.value() * _nominal_grid_frequency.value());
}

voltage_t sfc_plant_model_voltage_behind_reactance::resolve_grid_voltage() const noexcept
{
    return voltage_t(_grid_voltage_pu.value() * _nominal_grid_voltage_rms.value());
}

// ==================== Per-Unit Runtime Getters ====================
voltage_pu_t sfc_plant_model_voltage_behind_reactance::get_grid_voltage_pu() const noexcept { return _grid_voltage_pu; }
voltage_pu_t sfc_plant_model_voltage_behind_reactance::get_machine_voltage_pu() const noexcept
{
    return _machine_voltage_pu;
}
frequency_pu_t sfc_plant_model_voltage_behind_reactance::get_grid_frequency_pu() const noexcept
{
    return _grid_frequency_pu;
}
frequency_pu_t sfc_plant_model_voltage_behind_reactance::get_machine_frequency_pu() const noexcept
{
    if (_mode == plant_model_machine_operation_mode::variable_speed) {
        return per_unit_cast<frequency_pu_t>(_mechanical_load.get_speed_pu());
    }
    return _machine_frequency_pu;
}

current_pu_t sfc_plant_model_voltage_behind_reactance::get_field_current() const noexcept
{
    return _field_current_regulator.get_field_current();
}

voltage_pu_t sfc_plant_model_voltage_behind_reactance::get_exciter_voltage() const noexcept
{
    return _field_current_regulator.get_exciter_voltage();
}

real_t sfc_plant_model_voltage_behind_reactance::get_flux_pu() const noexcept
{
    // Flux is proportional to field current (assuming linear magnetization)
    // Φ = k × i_f, where k is absorbed into per-unit system
    return _field_current_regulator.get_field_current().value();
}

}  // namespace yeet::application::sfc
