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
      _speed_reference_limiter(sampling_time.value())
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

bool sfc_startup_sequencer::start_sequence()
{
    // Can only start from IDLE state
    if (_current_state != startup_sequencer_state_t::IDLE) {
        return false;
    }

    // Transition to flux buildup
    transition_to_flux_buildup();
    return true;
}

void sfc_startup_sequencer::stop_sequence() { transition_to_idle(); }

void sfc_startup_sequencer::update(bool is_field_established,
    angle_wrapped estimated_rotor_angle,
    speed_pu_t external_speed_reference,
    frequency_t base_frequency)
{
    // Update state elapsed timer (always enabled)
    _state_elapsed_timer.update(true);

    // Execute current state logic
    switch (_current_state) {
        case startup_sequencer_state_t::IDLE:
            update_idle();
            break;

        case startup_sequencer_state_t::FLUX_BUILDUP:
            update_flux_buildup(is_field_established);
            break;

        case startup_sequencer_state_t::STARTING_PROCEDURE:
            update_starting_procedure(base_frequency);
            break;

        case startup_sequencer_state_t::CLOSED_LOOP_OPERATION:
            update_closed_loop_operation(external_speed_reference);
            break;
    }

    // Update outputs based on current state
    update_outputs_for_current_state();

    // Store estimated angle for potential synchronization
    if (_current_state == startup_sequencer_state_t::STARTING_PROCEDURE) {
        // Synchronize virtual angle to estimated before transition
        const duration_t time_remaining =
            _virtual_frequency_config.duration - _virtual_frequency_ramp.get_elapsed_time();
        if (time_remaining < duration_t{0.1}) {  // 100ms before end
            synchronize_virtual_to_estimated(estimated_rotor_angle);
        }
    }
}

void sfc_startup_sequencer::reset()
{
    // Reset to IDLE state
    _current_state = startup_sequencer_state_t::IDLE;
    _previous_state = startup_sequencer_state_t::IDLE;

    // Reset all timers
    reset_timers();

    // Reset virtual reference frame
    _virtual_frequency = frequency_pu_t{0.0};
    _virtual_rotor_angle = angle_t{0.0};
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
    if (_current_state != startup_sequencer_state_t::FLUX_BUILDUP) {
        return;
    }

    _previous_state = _current_state;
    _current_state = startup_sequencer_state_t::STARTING_PROCEDURE;

    // Check if we should skip the virtual flux phase
    if (_virtual_frequency_config.duration <= duration_t{0.0}) {
        // Skip directly to closed-loop
        transition_to_closed_loop_operation();
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

void sfc_startup_sequencer::transition_to_closed_loop_operation()
{
    if (_current_state != startup_sequencer_state_t::STARTING_PROCEDURE) {
        return;
    }

    _previous_state = _current_state;
    _current_state = startup_sequencer_state_t::CLOSED_LOOP_OPERATION;

    // Reset state elapsed timer
    _state_elapsed_timer.reset();
}

void sfc_startup_sequencer::transition_to_idle()
{
    _previous_state = _current_state;
    _current_state = startup_sequencer_state_t::IDLE;

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

void sfc_startup_sequencer::update_idle()
{
    // Idle state - no action required
    // Waiting for start_sequence() to be called
}

void sfc_startup_sequencer::update_flux_buildup(bool is_field_established)
{
    // Check if field is established (excitation controller determines this)
    if (is_field_established) {
        // Field is established, proceed to starting procedure
        transition_to_starting_procedure();
    }
}

void sfc_startup_sequencer::update_starting_procedure(frequency_t base_frequency)
{
    bool has_elapsed = update_virtual_reference_frame(base_frequency);
    has_elapsed &= update_virtual_torque_reference();

    // Keep speed reference tracking virtual frequency during starting procedure
    // Since both are in per-unit, freq_pu directly maps to speed_pu
    _speed_reference_limiter.reset(_virtual_frequency.value());

    // Check if duration has elapsed
    if (has_elapsed) {
        // Virtual flux phase complete - transition to closed-loop
        transition_to_closed_loop_operation();
    }
}

void sfc_startup_sequencer::update_closed_loop_operation(speed_pu_t external_speed_reference)
{
    // Update speed reference limiter with external reference (only in closed-loop)
    _speed_reference_limiter.update(external_speed_reference.value());

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
    real_t omega_si = two_pi * _virtual_frequency.value() * base_frequency.value();

    // Integrate to get angle (wrapping handled automatically by angle_wrapped)
    angle_wrapped angle_increment = _virtual_angle_integrator.update(omega_si);
    _virtual_rotor_angle = angle_increment.get_radians();

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

    switch (_current_state) {
        case startup_sequencer_state_t::IDLE:
            _outputs.controller_commands.reference_frame_source = reference_frame_source_t::ESTIMATED;
            _outputs.controller_commands.torque_control_mode = sfc_torque_controller::control_mode::SPEED_CONTROL;
            _outputs.controller_commands.release_machine_side_pulses = false;
            _outputs.controller_commands.release_excitation = false;
            _outputs.controller_commands.release_machine_observer = false;
            _outputs.controller_commands.release_speed_controller = false;
            _outputs.controller_commands.release_torque_controller = false;
            break;

        case startup_sequencer_state_t::FLUX_BUILDUP:
            _outputs.controller_commands.reference_frame_source = reference_frame_source_t::VIRTUAL;
            _outputs.controller_commands.torque_control_mode = sfc_torque_controller::control_mode::MANUAL_TORQUE;
            _outputs.controller_commands.release_machine_side_pulses = false;  // No pulses during flux buildup
            _outputs.controller_commands.release_excitation = true;            // Enable excitation controller
            _outputs.controller_commands.release_machine_observer = true;      // Enable observer during flux buildup
            _outputs.controller_commands.release_speed_controller = false;
            _outputs.controller_commands.release_torque_controller = false;
            break;

        case startup_sequencer_state_t::STARTING_PROCEDURE:
            _outputs.controller_commands.reference_frame_source = reference_frame_source_t::VIRTUAL;
            _outputs.controller_commands.torque_control_mode = sfc_torque_controller::control_mode::MANUAL_TORQUE;
            _outputs.controller_commands.release_machine_side_pulses = true;  // Release pulses for starting
            _outputs.controller_commands.release_excitation = true;
            _outputs.controller_commands.release_machine_observer = true;  // Enable observer
            _outputs.controller_commands.release_speed_controller = false;
            _outputs.controller_commands.release_torque_controller = true;  // Enable torque controller
            break;

        case startup_sequencer_state_t::CLOSED_LOOP_OPERATION:
            _outputs.controller_commands.reference_frame_source = reference_frame_source_t::ESTIMATED;
            _outputs.controller_commands.torque_control_mode = sfc_torque_controller::control_mode::SPEED_CONTROL;
            _outputs.controller_commands.release_machine_side_pulses = true;
            _outputs.controller_commands.release_excitation = true;
            _outputs.controller_commands.release_machine_observer = true;
            _outputs.controller_commands.release_speed_controller = true;  // Enable speed controller
            _outputs.controller_commands.release_torque_controller = true;
            break;
    }
}

// ========== Helper Methods ==========

void sfc_startup_sequencer::reset_timers()
{
    _virtual_frequency_ramp.reset();
    _virtual_torque_ramp.reset();
    _state_elapsed_timer.reset();
}

}  // namespace yeet::application::sfc
