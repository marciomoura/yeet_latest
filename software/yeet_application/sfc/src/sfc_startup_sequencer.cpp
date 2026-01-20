#include "sfc/sfc_startup_sequencer.hpp"

#include <algorithm>
#include <cmath>

#include "yeet_control/constants.hpp"

namespace yeet::application::sfc {

sfc_startup_sequencer::sfc_startup_sequencer(duration_t sampling_time)
    : _sampling_time(sampling_time),
      _virtual_frequency_ramp(sampling_time),
      _virtual_torque_ramp(sampling_time),
      _state_elapsed_timer(sampling_time),
      _virtual_angle_integrator(sampling_time),
      _speed_reference_limiter(sampling_time.value()),
      _observer_settling_timer(sampling_time)
{
    reset();
}

void sfc_startup_sequencer::configure_virtual_frequency(const virtual_frequency_config& config)
{
    _virtual_frequency_config = config;

    // Configure the frequency ramp (already in per-unit)
    linear_ramp<frequency_pu_t>::config ramp_cfg{
        .initial = config.initial, .final = config.final, .duration = config.duration};
    _virtual_frequency_ramp.configure(ramp_cfg);
}

void sfc_startup_sequencer::configure_virtual_torque(const virtual_torque_config& config)
{
    _virtual_torque_config = config;

    // Configure the torque ramp
    linear_ramp<torque_pu_t>::config ramp_cfg{
        .initial = config.initial, .final = config.final, .duration = config.duration};
    _virtual_torque_ramp.configure(ramp_cfg);
}

void sfc_startup_sequencer::configure_speed_reference_rate_limit(const speed_reference_rate_limit_config& config)
{
    _speed_reference_rate_limit_config = config;

    // Convert seconds/pu to pu/second for rate limiter
    // If seconds_per_pu = 10, then rate = 1/10 = 0.1 pu/s
    real_t rate_pu_per_second = 1.0f / config.seconds_per_pu;
    _speed_reference_limiter.configure(rate_pu_per_second);
    _speed_reference_limiter.configure_enable(config.enabled);
}

void sfc_startup_sequencer::configure_flying_restart(const flying_restart_config& config)
{
    _flying_restart_config = config;
}

void sfc_startup_sequencer::configure_force_virtual_mode(bool force_virtual)
{
    _force_virtual_mode_operation = force_virtual;
}

void sfc_startup_sequencer::configure_skip_flux_buildup(bool skip_flux_buildup)
{
    _skip_flux_buildup = skip_flux_buildup;
}

bool sfc_startup_sequencer::start_sequence()
{
    // If not in IDLE, reset to IDLE first to ensure a clean start
    if (_current_state != startup_sequencer_state_t::IDLE) {
        transition_to_idle();
    }

    // Mark start as requested and begin observer settling
    _start_requested = true;
    _observer_settling_timer.reset();

    // Stay in IDLE, don't transition yet
    // Observer will be enabled via update_outputs_for_current_state()
    update_outputs_for_current_state();

    return true;
}

void sfc_startup_sequencer::stop_sequence() { transition_to_idle(); }

void sfc_startup_sequencer::update(bool is_field_established,
    angle_wrapped estimated_rotor_angle,
    speed_pu_t measured_speed,
    voltage_pu_t measured_voltage_magnitude,
    sfc_state_id current_state_id,
    speed_pu_t speed_reference,
    frequency_t base_frequency)
{
    // Handle automatic start/stop coordination with state machine
    if (_flying_restart_config.auto_start_stop_enabled) {
        update_state_machine_coordination(measured_speed, current_state_id, speed_reference);
    }

    // Apply shutdown speed override if active
    // This forces speed reference to zero during SHUTDOWN_REQUESTED and OPERATION_OFF states
    speed_pu_t effective_speed_reference = speed_reference;
    if (_shutdown_speed_override_active) {
        effective_speed_reference = speed_pu_t{0.0};
    }

    // Update state elapsed timer (always enabled)
    _state_elapsed_timer.update(true);

    // Execute current state logic
    switch (_current_state) {
        case startup_sequencer_state_t::IDLE:
            update_idle(effective_speed_reference, measured_speed, measured_voltage_magnitude);
            break;

        case startup_sequencer_state_t::FLUX_BUILDUP:
            update_flux_buildup(is_field_established, measured_speed, measured_voltage_magnitude);
            break;

        case startup_sequencer_state_t::STARTING_PROCEDURE:
            update_starting_procedure(base_frequency);
            break;

        case startup_sequencer_state_t::CLOSED_LOOP_OPERATION:
            if (_force_virtual_mode_operation) {
                // Parameter changed at runtime - force back to starting procedure
                _previous_state = _current_state;
                _current_state = startup_sequencer_state_t::STARTING_PROCEDURE;

                // Initialize virtual reference frame
                _virtual_frequency = _virtual_frequency_config.initial;
                _virtual_rotor_angle = angle_t{0.0};
                _virtual_angle_integrator.reset();
                _virtual_torque = _virtual_torque_config.initial;

                // Reset ramps
                _virtual_frequency_ramp.reset();
                _virtual_torque_ramp.reset();

                // Reset state elapsed timer
                _state_elapsed_timer.reset();
            }
            else {
                update_closed_loop_operation(effective_speed_reference);
            }
            break;
    }

    // Update outputs based on current state
    update_outputs_for_current_state();

    // Handle observer preset during FLUX_BUILDUP state
    // During flux buildup, we feed the observer position back to the virtual angle integrator
    if (_current_state == startup_sequencer_state_t::FLUX_BUILDUP) {
        // During flux buildup, the observer is running but we want its angle
        // No preset needed - observer starts from zero
        _preset_requested = false;
    }
}

void sfc_startup_sequencer::reset()
{
    // Reset to IDLE state
    _current_state = startup_sequencer_state_t::IDLE;
    _previous_state = startup_sequencer_state_t::IDLE;
    _previous_state_machine_state = sfc_state_id::IDLE;
    _shutdown_speed_override_active = false;
    _start_requested = false;
    _preset_requested = false;
    _preset_angle_value = angle_wrapped{0.0};
    _preset_frequency_value = frequency_pu_t{0.0};

    // Reset all timers
    reset_timers();
    _observer_settling_timer.reset();

    // Reset virtual reference frame
    _virtual_frequency = frequency_pu_t{0.0};
    _virtual_rotor_angle = angle_wrapped{0.0};
    _virtual_torque = torque_pu_t{0.0};
    _virtual_angle_integrator.reset();

    // Reset outputs
    update_outputs_for_current_state();
}

const startup_sequencer_outputs& sfc_startup_sequencer::get_outputs() const noexcept { return _outputs; }

startup_sequencer_state_t sfc_startup_sequencer::get_current_state() const noexcept { return _current_state; }

bool sfc_startup_sequencer::is_startup_complete() const noexcept
{
    return _current_state == startup_sequencer_state_t::CLOSED_LOOP_OPERATION;
}

frequency_pu_t sfc_startup_sequencer::get_virtual_frequency() const noexcept { return _virtual_frequency; }

duration_t sfc_startup_sequencer::get_time_in_current_state() const noexcept
{
    return _state_elapsed_timer.get_elapsed_time();
}

// ========== State Transition Methods ==========

void sfc_startup_sequencer::transition_to_flux_buildup()
{
    if (_current_state != startup_sequencer_state_t::IDLE) {
        return;
    }

    _previous_state = _current_state;
    _current_state = startup_sequencer_state_t::FLUX_BUILDUP;

    // Reset state elapsed timer
    _state_elapsed_timer.reset();
}

void sfc_startup_sequencer::transition_to_starting_procedure()
{
    // Allow transition from FLUX_BUILDUP (normal path) or IDLE (if skip_flux_buildup is enabled)
    if (_current_state != startup_sequencer_state_t::FLUX_BUILDUP &&
        _current_state != startup_sequencer_state_t::IDLE) {
        return;
    }

    _previous_state = _current_state;
    _current_state = startup_sequencer_state_t::STARTING_PROCEDURE;

    // Check if we should skip the virtual flux phase
    if (_virtual_frequency_config.duration <= duration_t{0.0}) {
        // Skip directly to closed-loop
        transition_to_closed_loop_operation(speed_pu_t{_virtual_frequency.value()});
        return;
    }

    // Initialize virtual reference frame
    _virtual_frequency = _virtual_frequency_config.initial;
    _virtual_rotor_angle = angle_t{0.0};
    _virtual_angle_integrator.reset();

    // Initialize virtual torque
    _virtual_torque = _virtual_torque_config.initial;

    // Reset ramps (which also resets their internal timers)
    _virtual_frequency_ramp.reset();
    _virtual_torque_ramp.reset();

    // Reset state elapsed timer
    _state_elapsed_timer.reset();
}

void sfc_startup_sequencer::transition_to_closed_loop_operation(speed_pu_t initial_speed_reference)
{
    // Allow transition from STARTING_PROCEDURE (normal path) or FLUX_BUILDUP (flying start path)
    if (_current_state != startup_sequencer_state_t::STARTING_PROCEDURE &&
        _current_state != startup_sequencer_state_t::FLUX_BUILDUP) {
        return;
    }

    _previous_state = _current_state;
    _current_state = startup_sequencer_state_t::CLOSED_LOOP_OPERATION;

    // Preset the speed reference limiter at closed-loop entry.
    // - Normal path: seed with virtual frequency (speed reference tracks virtual during starting)
    // - Flying start path: seed with measured speed to avoid an initial speed error jump
    _speed_reference_limiter.reset(initial_speed_reference.value());

    // Clear preset request immediately upon transition
    // This ensures the preset signal doesn't get stuck high
    _preset_requested = false;

    // Reset state elapsed timer
    _state_elapsed_timer.reset();
}

void sfc_startup_sequencer::transition_to_idle()
{
    _previous_state = _current_state;
    _current_state = startup_sequencer_state_t::IDLE;

    // Clear start request and reset settling timer
    _start_requested = false;
    _observer_settling_timer.reset();

    // Stop all timers
    reset_timers();

    // Reset virtual references
    _virtual_frequency = frequency_pu_t{0.0};
    _virtual_rotor_angle = angle_t{0.0};
    _virtual_torque = torque_pu_t{0.0};
    _virtual_angle_integrator.reset();

    // Reset state elapsed timer
    _state_elapsed_timer.reset();
}

// ========== State Update Methods ==========

void sfc_startup_sequencer::update_idle(
    speed_pu_t speed_reference, speed_pu_t measured_speed, voltage_pu_t measured_voltage_magnitude)
{
    // Update speed reference limiter even in IDLE to handle shutdown scenarios
    // When shutdown override is active (OPERATION_OFF), we need to ramp speed to zero
    // _speed_reference_limiter.update(speed_reference);
    _speed_reference_limiter.reset(speed_pu_t{0.0f});

    // If start has been requested, wait for observer settling
    if (_start_requested) {
        _observer_settling_timer.update(true);

        // Check if settling period has elapsed
        if (_observer_settling_timer.get_elapsed_time() >= _flying_restart_config.observer_settling_time) {
            // Settling complete.
            // If skipping flux buildup, go directly to starting procedure.
            // Otherwise transition to flux buildup phase.
            if (_skip_flux_buildup) {
                transition_to_starting_procedure();
            }
            else {
                transition_to_flux_buildup();
            }

            _start_requested = false;
        }
        // Else: still settling, stay in IDLE
    }
}

void sfc_startup_sequencer::update_flux_buildup(
    bool is_field_established, speed_pu_t measured_speed, voltage_pu_t measured_voltage_magnitude)
{
    // Check if field is established (excitation controller determines this)
    if (!is_field_established) {
        return;
    }

    // Field is established. If machine is already generating sufficient voltage,
    // treat this as a flying start and jump directly to closed-loop with the
    // speed reference initialized to the current measured speed.
    // However, if virtual mode is forced, proceed with normal startup.
    const bool voltage_detected = measured_voltage_magnitude >= _flying_restart_config.voltage_threshold;
    if (voltage_detected && !_force_virtual_mode_operation) {
        transition_to_closed_loop_operation(measured_speed);
        return;
    }

    // Otherwise proceed with normal startup.
    transition_to_starting_procedure();
}

void sfc_startup_sequencer::update_starting_procedure(frequency_t base_frequency)
{
    bool has_elapsed = update_virtual_reference_frame(base_frequency);
    has_elapsed &= update_virtual_torque_reference();

    // Keep speed reference tracking virtual frequency during starting procedure
    // Since both are in per-unit, freq_pu directly maps to speed_pu
    _speed_reference_limiter.reset(_virtual_frequency.value());

    // New strategy: observer runs in background during virtual mode, but we do not
    // preset/initialize it for switchover. Bumpless transfer is achieved by
    // initializing controller integrators at the handover.
    _preset_requested = false;

    // Check if duration has elapsed
    if (has_elapsed && !_force_virtual_mode_operation) {
        // Virtual flux phase complete - transition to closed-loop
        transition_to_closed_loop_operation(speed_pu_t{_virtual_frequency.value()});
    }
}

void sfc_startup_sequencer::update_closed_loop_operation(speed_pu_t speed_reference)
{
    // Update speed reference limiter with external reference (only in closed-loop)
    _speed_reference_limiter.update(speed_reference.value());

    // Clear preset request - no longer needed in closed-loop
    _preset_requested = false;

    // Closed-loop operation - speed control is active
    // System is running normally under speed control
}

// ========== Virtual Reference Frame Generation ==========

bool sfc_startup_sequencer::update_virtual_reference_frame(frequency_t base_frequency)
{
    // Update frequency ramp and get current frequency (in per-unit)
    _virtual_frequency = _virtual_frequency_ramp.update(true);

    // Convert per-unit frequency to SI angular velocity (rad/s)
    // freq_si = freq_pu * base_frequency
    // omega_si = 2*pi * freq_si = 2*pi * freq_pu * base_frequency
    angular_frequency_t omega_si = hz_to_rad_per_sec(_virtual_frequency.value() * base_frequency.value());

    // Integrate to get angle (wrapping handled automatically by angle_wrapped)
    _virtual_rotor_angle = _virtual_angle_integrator.update(omega_si.value());

    // Check if ramp has completed
    return _virtual_frequency_ramp.is_finished();
}

bool sfc_startup_sequencer::update_virtual_torque_reference()
{
    // Update torque ramp and get current torque reference
    _virtual_torque = _virtual_torque_ramp.update(true);

    // Check if ramp has completed
    return _virtual_torque_ramp.is_finished();
}

void sfc_startup_sequencer::synchronize_virtual_to_estimated(angle_wrapped estimated_angle)
{
    // Synchronize the virtual angle integrator to the estimated angle
    // This prevents a discontinuity when switching from virtual to estimated
    _virtual_angle_integrator.reset(estimated_angle);
    _virtual_rotor_angle = estimated_angle;
}

// ========== Output Generation ==========

void sfc_startup_sequencer::update_outputs_for_current_state()
{
    // Update virtual reference outputs
    _outputs.virtual_reference.rotor_angle = _virtual_rotor_angle;
    _outputs.virtual_reference.rotor_frequency = _virtual_frequency;
    _outputs.virtual_reference.torque_reference = _virtual_torque;

    // Update speed control outputs (rate-limited speed reference)
    _outputs.speed_control.speed_reference = speed_pu_t{_speed_reference_limiter.get_output()};

    // Update controller command outputs based on state
    _outputs.controller_commands.current_state = _current_state;

    // Update preset outputs
    _outputs.controller_commands.preset_observer = _preset_requested;
    _outputs.controller_commands.preset_angle = _preset_angle_value;
    _outputs.controller_commands.preset_frequency = _preset_frequency_value;

    switch (_current_state) {
        case startup_sequencer_state_t::IDLE:
            _outputs.controller_commands.reference_frame_source =
                _force_virtual_mode_operation ? reference_frame_source_t::VIRTUAL : reference_frame_source_t::ESTIMATED;
            _outputs.controller_commands.torque_control_mode = sfc_torque_controller::control_mode::SPEED_CONTROL;
            _outputs.controller_commands.release_machine_side_pulses = false;
            // During observer settling (start requested), enable excitation in parallel so the
            // excitation controller's preflux timer can elapse without being delayed by the
            // additional observer settling time spent in IDLE.
            // If skip_flux_buildup is true, suppress excitation release.
            _outputs.controller_commands.release_excitation = _start_requested && !_skip_flux_buildup;
            _outputs.controller_commands.release_machine_observer =
                _start_requested;  // Enable observer when start requested for settling
            _outputs.controller_commands.release_speed_controller = false;
            _outputs.controller_commands.release_torque_controller = false;
            break;

        case startup_sequencer_state_t::FLUX_BUILDUP:
            _outputs.controller_commands.reference_frame_source = reference_frame_source_t::VIRTUAL;
            _outputs.controller_commands.torque_control_mode = sfc_torque_controller::control_mode::MANUAL_TORQUE;
            _outputs.controller_commands.release_machine_side_pulses = false;       // No pulses during flux buildup
            _outputs.controller_commands.release_excitation = !_skip_flux_buildup;  // Enable excitation controller
            _outputs.controller_commands.release_machine_observer = true;  // Enable observer when excitation is on
            _outputs.controller_commands.release_speed_controller = false;
            _outputs.controller_commands.release_torque_controller = false;
            break;

        case startup_sequencer_state_t::STARTING_PROCEDURE:
            _outputs.controller_commands.reference_frame_source = reference_frame_source_t::VIRTUAL;
            _outputs.controller_commands.torque_control_mode = sfc_torque_controller::control_mode::MANUAL_TORQUE;
            _outputs.controller_commands.release_machine_side_pulses = true;  // Release pulses for starting
            _outputs.controller_commands.release_excitation = !_skip_flux_buildup;
            _outputs.controller_commands.release_machine_observer = true;  // Enable observer
            _outputs.controller_commands.release_speed_controller = false;
            _outputs.controller_commands.release_torque_controller = true;  // Enable torque controller
            break;

        case startup_sequencer_state_t::CLOSED_LOOP_OPERATION:
            _outputs.controller_commands.reference_frame_source = reference_frame_source_t::ESTIMATED;
            _outputs.controller_commands.torque_control_mode = sfc_torque_controller::control_mode::SPEED_CONTROL;
            _outputs.controller_commands.release_machine_side_pulses = true;
            _outputs.controller_commands.release_excitation = !_skip_flux_buildup;
            _outputs.controller_commands.release_machine_observer = true;
            _outputs.controller_commands.release_speed_controller = true;  // Enable speed controller
            _outputs.controller_commands.release_torque_controller = true;
            break;
    }
}

// ========== Automatic Start/Stop Coordination ==========

void sfc_startup_sequencer::update_state_machine_coordination(
    speed_pu_t measured_speed, sfc_state_id current_state_id, speed_pu_t speed_reference)
{
    // Detect transition TO OPERATION_ON -> start sequence
    if (transitioned_to(sfc_state_id::OPERATION_ON, current_state_id)) {
        // Simply start the sequence - flying restart detection happens in update_idle()
        // after observer settling period
        start_sequence();
    }

    // Detect SHUTDOWN_REQUESTED or OPERATION_OFF state -> activate speed override to zero
    // This allows controlled deceleration while maintaining commutation during SHUTDOWN_REQUESTED
    // and keeps speed at zero during OPERATION_OFF
    if (current_state_id == sfc_state_id::SHUTDOWN_REQUESTED || current_state_id == sfc_state_id::OPERATION_OFF) {
        _shutdown_speed_override_active = true;
    }
    else {
        _shutdown_speed_override_active = false;
    }

    // When in any state other than OPERATION_ON or SHUTDOWN_REQUESTED, ensure sequencer is in IDLE.
    // This allows controlled deceleration in SHUTDOWN_REQUESTED, but ensures a clean
    // reset if we transition back to standby or aux control states.
    const bool is_active_state =
        (current_state_id == sfc_state_id::OPERATION_ON || current_state_id == sfc_state_id::SHUTDOWN_REQUESTED);

    if (!is_active_state && _current_state != startup_sequencer_state_t::IDLE) {
        stop_sequence();
    }

    // Store current state for next cycle edge detection
    _previous_state_machine_state = current_state_id;
}

// ========== Helper Methods ==========

void sfc_startup_sequencer::reset_timers()
{
    _virtual_frequency_ramp.reset();
    _virtual_torque_ramp.reset();
    _state_elapsed_timer.reset();
}

bool sfc_startup_sequencer::transitioned_from(sfc_state_id from_state, sfc_state_id current_state) const noexcept
{
    return _previous_state_machine_state == from_state && current_state != from_state;
}

bool sfc_startup_sequencer::transitioned_to(sfc_state_id to_state, sfc_state_id current_state) const noexcept
{
    return _previous_state_machine_state != to_state && current_state == to_state;
}

bool sfc_startup_sequencer::transition(
    sfc_state_id from_state, sfc_state_id to_state, sfc_state_id current_state) const noexcept
{
    return _previous_state_machine_state == from_state && current_state == to_state;
}

}  // namespace yeet::application::sfc
