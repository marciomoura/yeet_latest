#include "sfc/sfc_excitation_controller.hpp"

#include <algorithm>
#include <cassert>

namespace yeet::application::sfc {

sfc_excitation_controller::sfc_excitation_controller(duration_t sampling_time)
    : _sampling_time(sampling_time),
      _flux_buildup_timer(sampling_time),
      _voltage_reference_lookup_table(),
      _voltage_pi_controller(sampling_time.value())
{
    // Default voltage reference lookup table
    std::array<real_t, NUM_VOLTAGE_REFERENCE_POINTS> speed_axis{};
    std::array<real_t, NUM_VOLTAGE_REFERENCE_POINTS> voltage_ref{};
    for (size_t i = 0; i < NUM_VOLTAGE_REFERENCE_POINTS; ++i) {
        speed_axis[i] = static_cast<real_t>(kDefaultVoltageReferenceSpeedAxisPu[i].value());
        voltage_ref[i] = static_cast<real_t>(kDefaultVoltageReferencePu[i].value());
    }
    _voltage_reference_lookup_table.configure(speed_axis, voltage_ref);

    // Default PI output limits
    _voltage_pi_controller.set_output_limits(_voltage_pi_output_min_pu.value(), _voltage_pi_output_max_pu.value());
    _voltage_pi_controller.enable_output_limits(true);
    _voltage_pi_controller.enable_integrator(true);
    _voltage_pi_controller.enable_integrator_clamping(false);
}

void sfc_excitation_controller::configure_field_current_setpoint(current_pu_t field_current_setpoint)
{
    assert(field_current_setpoint.value() >= 0.0 && "Field current setpoint must be non-negative.");

    _field_current_setpoint = field_current_setpoint;
}

void sfc_excitation_controller::configure_low_voltage_field_current_setpoint(current_pu_t field_current_setpoint)
{
    assert(field_current_setpoint.value() >= 0.0 && "Low-voltage field current setpoint must be non-negative.");
    _low_voltage_field_current_setpoint = field_current_setpoint;
}

void sfc_excitation_controller::configure_voltage_threshold(voltage_pu_t voltage_threshold_pu)
{
    assert(voltage_threshold_pu.value() >= 0.0 && "Voltage threshold must be non-negative.");
    _voltage_threshold_pu = voltage_threshold_pu;
}

void sfc_excitation_controller::configure_voltage_reference_lookup_table(
    const std::array<speed_pu_t, NUM_VOLTAGE_REFERENCE_POINTS>& speed_axis_pu,
    const std::array<voltage_pu_t, NUM_VOLTAGE_REFERENCE_POINTS>& voltage_reference_pu)
{
    for (size_t i = 0; i < NUM_VOLTAGE_REFERENCE_POINTS; ++i) {
        assert(speed_axis_pu[i].value() >= 0.0 && "Speed axis values must be non-negative.");
        assert(voltage_reference_pu[i].value() >= 0.0 && "Voltage reference values must be non-negative.");
    }

    std::array<real_t, NUM_VOLTAGE_REFERENCE_POINTS> speed_axis{};
    std::array<real_t, NUM_VOLTAGE_REFERENCE_POINTS> voltage_ref{};
    for (size_t i = 0; i < NUM_VOLTAGE_REFERENCE_POINTS; ++i) {
        speed_axis[i] = static_cast<real_t>(speed_axis_pu[i].value());
        voltage_ref[i] = static_cast<real_t>(voltage_reference_pu[i].value());
    }
    _voltage_reference_lookup_table.configure(speed_axis, voltage_ref);
}

void sfc_excitation_controller::configure_voltage_pi_gains(real_t kp, duration_t ti)
{
    assert(kp >= 0.0F && "PI proportional gain must be non-negative.");
    assert(ti.value() > 0.0 && "PI integral time constant must be positive.");
    if (ti.value() <= 0.0) {
        return;
    }

    _voltage_pi_controller.configure_with_ti(static_cast<real_t>(kp), static_cast<real_t>(ti.value()));

    // Default anti-windup (back-calculation) gain: 1/Ti
    _voltage_pi_controller.set_antiwindup_gain(static_cast<real_t>(1.0 / ti.value()));
}

void sfc_excitation_controller::configure_voltage_pi_output_limits(
    current_pu_t min_field_current_pu, current_pu_t max_field_current_pu)
{
    assert(min_field_current_pu.value() <= max_field_current_pu.value() && "PI output min must be <= max.");

    _voltage_pi_output_min_pu = min_field_current_pu;
    _voltage_pi_output_max_pu = max_field_current_pu;

    _voltage_pi_controller.set_output_limits(min_field_current_pu.value(), max_field_current_pu.value());
    _voltage_pi_controller.enable_output_limits(true);
}

void sfc_excitation_controller::configure_flux_buildup_duration(duration_t flux_buildup_duration)
{
    assert(flux_buildup_duration.value() > 0.0 && "Flux build-up duration must be positive.");
    if (flux_buildup_duration.value() <= 0.0) {
        return;
    }

    _flux_buildup_duration = flux_buildup_duration;
}

bool sfc_excitation_controller::start_excitation()
{
    // Simplified: just enable and set constant field current
    _is_running = true;
    _is_field_established = false;
    _enable_excitation = true;

    // Set field current reference to configured constant value
    _field_current_reference_pu = _field_current_setpoint;
    _last_forced_field_current_reference_pu = _field_current_reference_pu;
    _is_voltage_pi_active = false;

    // Start flux build-up timer
    _flux_buildup_timer.reset();

    return true;
}

void sfc_excitation_controller::stop_excitation()
{
    // Simplified: immediate disable, no ramp-down
    _is_running = false;
    _is_field_established = false;
    _enable_excitation = false;
    _field_current_reference_pu = current_pu_t{0.0};
    _flux_buildup_timer.reset();
}

void sfc_excitation_controller::update(
    speed_pu_t machine_speed_pu, voltage_pu_t machine_voltage_magnitude_pu, bool enable_excitation_command)
{
    // Handle enable/disable command directly
    if (!enable_excitation_command) {
        // Excitation disabled - reset all states
        _is_running = false;
        _is_field_established = false;
        _enable_excitation = false;
        _field_current_reference_pu = current_pu_t{0.0};
        _machine_voltage_reference_pu = voltage_pu_t{0.0};
        _machine_voltage_error_pu = voltage_pu_t{0.0};
        _is_voltage_pi_active = false;
        _last_forced_field_current_reference_pu = current_pu_t{0.0};
        _flux_buildup_timer.reset();
        _voltage_pi_controller.reset(real_t{0.0});
        return;
    }

    // Excitation enabled - detect rising edge to reset timer
    if (!_is_running) {
        _is_running = true;
        _is_field_established = false;
        _flux_buildup_timer.reset();
    }

    // Flux build-up timer is maintained regardless of voltage control region
    if (!_is_field_established) {
        _flux_buildup_timer.update(true);
        if (_flux_buildup_timer.has_elapsed(_flux_buildup_duration)) {
            _is_field_established = true;
        }
    }

    // Select forced (low-voltage) current reference:
    // - If field_current_setpoint is configured (>0), keep it.
    // - Otherwise fall back to low_voltage_field_current_setpoint.
    const current_pu_t forced_field_current_reference =
        (_field_current_setpoint.value() > 0.0) ? _field_current_setpoint : _low_voltage_field_current_setpoint;

    const bool is_low_voltage_region = machine_voltage_magnitude_pu.value() < _voltage_threshold_pu.value();

    // Compute voltage reference and error for observability
    if (is_low_voltage_region) {
        _machine_voltage_reference_pu = voltage_pu_t{0.0};
        _machine_voltage_error_pu = voltage_pu_t{0.0};
    }
    else {
        const voltage_pu_t reference_voltage =
            voltage_pu_t{_voltage_reference_lookup_table.get_value(machine_speed_pu.value())};
        _machine_voltage_reference_pu = reference_voltage;
        _machine_voltage_error_pu = reference_voltage - machine_voltage_magnitude_pu;
    }

    if (is_low_voltage_region) {
        // Forced current region: pass-through forced current setpoint.
        _enable_excitation = true;
        _field_current_reference_pu = forced_field_current_reference;
        _last_forced_field_current_reference_pu = forced_field_current_reference;
        _is_voltage_pi_active = false;
        return;
    }

    // Voltage regulation region: PI on voltage error -> field current reference.
    const real_t voltage_error = _machine_voltage_error_pu.value();

    if (!_is_voltage_pi_active) {
        // Bumpless transfer: preset integral term so first PI output matches the last forced current.
        const real_t desired_output = _last_forced_field_current_reference_pu.value();
        _voltage_pi_controller.preset_for_bumpless_transfer(desired_output, voltage_error);
        _is_voltage_pi_active = true;
    }

    const real_t pi_output = _voltage_pi_controller.update(voltage_error);
    _field_current_reference_pu = current_pu_t{pi_output};
    _enable_excitation = true;
}

void sfc_excitation_controller::reset()
{
    _is_running = false;
    _is_field_established = false;
    _enable_excitation = false;
    _field_current_reference_pu = current_pu_t{0.0};

    _machine_voltage_reference_pu = voltage_pu_t{0.0};
    _machine_voltage_error_pu = voltage_pu_t{0.0};
    _is_voltage_pi_active = false;
    _last_forced_field_current_reference_pu = current_pu_t{0.0};

    _flux_buildup_timer.reset();

    _voltage_pi_controller.reset(real_t{0.0});
}

current_pu_t sfc_excitation_controller::get_field_current_reference_pu() const noexcept
{
    return _field_current_reference_pu;
}

bool sfc_excitation_controller::get_enable_excitation() const noexcept { return _enable_excitation; }

bool sfc_excitation_controller::get_is_field_established() const noexcept { return _is_field_established; }

bool sfc_excitation_controller::get_is_running() const noexcept { return _is_running; }

duration_t sfc_excitation_controller::get_flux_buildup_elapsed_time() const noexcept
{
    return _flux_buildup_timer.get_elapsed_time();
}

voltage_pu_t sfc_excitation_controller::get_machine_voltage_reference_pu() const noexcept
{
    return _machine_voltage_reference_pu;
}

voltage_pu_t sfc_excitation_controller::get_machine_voltage_error_pu() const noexcept
{
    return _machine_voltage_error_pu;
}

}  // namespace yeet::application::sfc
