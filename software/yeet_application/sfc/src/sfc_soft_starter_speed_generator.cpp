#include "sfc/sfc_soft_starter_speed_generator.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>

namespace yeet::application::sfc {

sfc_soft_starter_speed_generator::sfc_soft_starter_speed_generator(duration_t sampling_time)
    : _sampling_time(sampling_time),
      _speed_ramp_limiter(1.0, sampling_time.value()),
      _breakaway_boost_timer(sampling_time)
{
    // Initialize all profiles as disabled by default
    acceleration_profile_config default_profile;
    default_profile.enabled = false;
    for (size_t i = 0; i < _profile_configs.size(); ++i) {
        _profile_configs[i] = default_profile;
    }

    // Configure default absolute limits
    _min_speed_limit = speed_pu_t{0.0};
    _max_speed_limit = speed_pu_t{1.2};
    _max_torque_limit = torque_pu_t{1.5};
    _min_torque_limit = torque_pu_t{-1.5};
}

// ========== Factory Methods ==========

acceleration_profile_config sfc_soft_starter_speed_generator::make_creep_speed_profile(
    speed_pu_t target_speed_pu, duration_t acceleration_time)
{
    acceleration_profile_config config;
    config.enabled = true;
    config.acceleration_time = acceleration_time;
    config.target_speed = target_speed_pu;
    config.torque_limit_max = torque_pu_t{0.30};  // Low torque for creep
    config.torque_limit_min = torque_pu_t{0.0};
    config.initial_torque_boost = torque_pu_t{0.15};  // Modest boost for breakaway
    config.boost_duration = duration_t{2.0};
    return config;
}

acceleration_profile_config sfc_soft_starter_speed_generator::make_purge_accel_profile(
    speed_pu_t target_speed_pu, duration_t acceleration_time)
{
    acceleration_profile_config config;
    config.enabled = true;
    config.acceleration_time = acceleration_time;
    config.target_speed = target_speed_pu;
    config.torque_limit_max = torque_pu_t{0.40};  // Moderate torque
    config.torque_limit_min = torque_pu_t{0.0};
    config.initial_torque_boost = torque_pu_t{0.20};  // Higher boost from standstill
    config.boost_duration = duration_t{3.0};
    return config;
}

acceleration_profile_config sfc_soft_starter_speed_generator::make_ignition_accel_profile(
    speed_pu_t target_speed_pu, duration_t acceleration_time)
{
    acceleration_profile_config config;
    config.enabled = true;
    config.acceleration_time = acceleration_time;
    config.target_speed = target_speed_pu;
    config.torque_limit_max = torque_pu_t{0.60};  // Higher torque for faster accel
    config.torque_limit_min = torque_pu_t{0.0};
    config.initial_torque_boost = torque_pu_t{0.10};  // Lower boost (already moving)
    config.boost_duration = duration_t{2.0};
    return config;
}

acceleration_profile_config sfc_soft_starter_speed_generator::make_assist_accel_profile(
    speed_pu_t target_speed_pu, duration_t acceleration_time)
{
    acceleration_profile_config config;
    config.enabled = true;
    config.acceleration_time = acceleration_time;
    config.target_speed = target_speed_pu;
    config.torque_limit_max = torque_pu_t{0.80};  // Highest torque for final push
    config.torque_limit_min = torque_pu_t{0.0};
    config.initial_torque_boost = torque_pu_t{0.05};  // Minimal boost (high speed)
    config.boost_duration = duration_t{1.5};
    return config;
}

// ========== Configuration Methods ==========

void sfc_soft_starter_speed_generator::configure_profile(
    acceleration_profile_t profile, const acceleration_profile_config& config)
{
    // Validate configuration
    assert(config.acceleration_time.value() > 0.0 && "Acceleration time must be positive.");
    assert(config.torque_limit_max.value() > 0.0 && "Maximum torque limit must be positive.");
    assert(config.boost_duration.value() >= 0.0 && "Boost duration must be non-negative.");
    assert(config.target_speed.value() >= 0.0 && "Target speed must be non-negative.");

    const size_t index = get_profile_index(profile);
    _profile_configs[index] = config;
}

void sfc_soft_starter_speed_generator::configure_deceleration(duration_t deceleration_time)
{
    assert(deceleration_time.value() > 0.0 && "Deceleration time must be positive.");

    _deceleration_time = deceleration_time;
}

void sfc_soft_starter_speed_generator::configure_absolute_limits(
    speed_pu_t min_speed_pu, speed_pu_t max_speed_pu, torque_pu_t max_torque_pu, torque_pu_t min_torque_pu)
{
    assert(min_speed_pu.value() >= 0.0 && "Minimum speed must be non-negative.");
    assert(max_speed_pu.value() > min_speed_pu.value() && "Maximum speed must be greater than minimum.");
    assert(max_torque_pu.value() > 0.0 && "Maximum torque must be positive.");
    assert(min_torque_pu.value() < 0.0 && "Minimum torque must be negative.");

    _min_speed_limit = min_speed_pu;
    _max_speed_limit = max_speed_pu;
    _max_torque_limit = max_torque_pu;
    _min_torque_limit = min_torque_pu;
}

// ========== Profile Selection and Validation ==========

bool sfc_soft_starter_speed_generator::is_profile_enabled(acceleration_profile_t profile) const
{
    const size_t index = get_profile_index(profile);
    return _profile_configs[index].enabled;
}

void sfc_soft_starter_speed_generator::disable_profile(acceleration_profile_t profile)
{
    const size_t index = get_profile_index(profile);
    _profile_configs[index].enabled = false;
}

bool sfc_soft_starter_speed_generator::select_profile(acceleration_profile_t profile)
{
    if (!is_profile_enabled(profile)) {
        return false;  // Cannot select disabled profile
    }

    _selected_profile = profile;
    return true;
}

acceleration_profile_t sfc_soft_starter_speed_generator::get_selected_profile() const { return _selected_profile; }

// ========== Command Execution ==========

bool sfc_soft_starter_speed_generator::execute_command(speed_command_t command)
{
    // Validate RUN_PROFILE command
    if (command == speed_command_t::RUN_PROFILE) {
        if (!is_profile_enabled(_selected_profile)) {
            return false;  // Cannot run disabled profile
        }
    }

    // Execute the command
    _active_command = command;

    // Initialize state based on command
    switch (command) {
        case speed_command_t::IDLE:
            _speed_reference_pu = speed_pu_t{0.0};
            _torque_limit_max_pu = torque_pu_t{0.0};
            _torque_limit_min_pu = torque_pu_t{0.0};
            _is_profile_complete = false;
            _is_breakaway_boost_active = false;
            _speed_ramp_limiter.reset();
            break;

        case speed_command_t::HOLD_SPEED:
            _hold_speed_setpoint = _speed_reference_pu;  // Hold current speed
            _is_profile_complete = false;
            _is_breakaway_boost_active = false;
            break;

        case speed_command_t::DECELERATE: {
            _profile_start_speed = _speed_reference_pu;
            _profile_target_speed = speed_pu_t{0.0};
            _is_profile_complete = false;
            _is_breakaway_boost_active = false;
            // Configure rate limiter for deceleration
            const real_t decel_rate = 1.0 / _deceleration_time.value();
            _speed_ramp_limiter.configure(decel_rate);
            _speed_ramp_limiter.reset(_speed_reference_pu.value());
            break;
        }

        case speed_command_t::RUN_PROFILE: {
            // Get selected profile configuration
            const size_t index = get_profile_index(_selected_profile);
            const auto& profile = _profile_configs[index];

            _profile_start_speed = _speed_reference_pu;
            _profile_target_speed = profile.target_speed;
            _is_profile_complete = false;

            // Configure rate limiter for this profile
            const real_t accel_rate = 1.0 / profile.acceleration_time.value();
            _speed_ramp_limiter.configure(accel_rate);
            _speed_ramp_limiter.reset(_speed_reference_pu.value());

            // Initialize breakaway boost if configured
            if (profile.initial_torque_boost.value() > 0.0) {
                _is_breakaway_boost_active = true;
                _breakaway_boost_timer.reset();
            }
            else {
                _is_breakaway_boost_active = false;
            }

            // Set torque limits from profile
            _torque_limit_max_pu = profile.torque_limit_max;
            _torque_limit_min_pu = profile.torque_limit_min;
            break;
        }
    }

    return true;
}

speed_command_t sfc_soft_starter_speed_generator::get_active_command() const { return _active_command; }

// ========== Real-Time Execution ==========

void sfc_soft_starter_speed_generator::update(speed_pu_t actual_speed_pu)
{
    // Execute command-specific logic
    switch (_active_command) {
        case speed_command_t::IDLE:
            // Do nothing, outputs remain at zero
            break;

        case speed_command_t::HOLD_SPEED:
            execute_hold_mode();
            break;

        case speed_command_t::DECELERATE:
            execute_deceleration();
            break;

        case speed_command_t::RUN_PROFILE: {
            // Get the active profile configuration
            const size_t index = get_profile_index(_selected_profile);
            const auto& profile = _profile_configs[index];
            execute_acceleration_profile(profile);
            break;
        }
    }

    // Update torque limits based on active command
    update_torque_limits();

    // Apply absolute safety limits
    _speed_reference_pu =
        speed_pu_t{std::clamp(_speed_reference_pu.value(), _min_speed_limit.value(), _max_speed_limit.value())};
    _torque_limit_max_pu = torque_pu_t{std::clamp(_torque_limit_max_pu.value(), 0.0f, _max_torque_limit.value())};
    _torque_limit_min_pu = torque_pu_t{std::clamp(_torque_limit_min_pu.value(), _min_torque_limit.value(), 0.0f)};
}

// ========== Output Accessors ==========

speed_pu_t sfc_soft_starter_speed_generator::get_speed_reference_pu() const { return _speed_reference_pu; }

torque_pu_t sfc_soft_starter_speed_generator::get_torque_limit_max_pu() const { return _torque_limit_max_pu; }

torque_pu_t sfc_soft_starter_speed_generator::get_torque_limit_min_pu() const { return _torque_limit_min_pu; }

// ========== Status Methods ==========

bool sfc_soft_starter_speed_generator::is_profile_complete() const { return _is_profile_complete; }

bool sfc_soft_starter_speed_generator::is_breakaway_boost_active() const { return _is_breakaway_boost_active; }

void sfc_soft_starter_speed_generator::reset(speed_pu_t initial_speed_pu)
{
    _active_command = speed_command_t::IDLE;
    _selected_profile = acceleration_profile_t::PROFILE_1;
    _speed_reference_pu = initial_speed_pu;
    _hold_speed_setpoint = initial_speed_pu;
    _torque_limit_max_pu = torque_pu_t{0.0};
    _torque_limit_min_pu = torque_pu_t{0.0};
    _is_profile_complete = false;
    _is_breakaway_boost_active = false;
    _profile_start_speed = initial_speed_pu;
    _profile_target_speed = initial_speed_pu;

    _speed_ramp_limiter.reset(initial_speed_pu.value());
    _breakaway_boost_timer.reset();
}

// ========== Private Helper Methods ==========

void sfc_soft_starter_speed_generator::execute_acceleration_profile(const acceleration_profile_config& profile)
{
    // Update speed ramp using rate limiter
    _speed_ramp_limiter.update(profile.target_speed.value());
    _speed_reference_pu = speed_pu_t{_speed_ramp_limiter.get_output()};

    // Check if profile is complete (within tolerance)
    const real_t speed_error = std::abs(profile.target_speed.value() - _speed_reference_pu.value());
    _is_profile_complete = (speed_error < 0.01);  // 1% tolerance

    // Update torque limits
    update_torque_limits();

    // Handle breakaway boost
    if (_is_breakaway_boost_active) {
        _breakaway_boost_timer.update(true);
        if (_breakaway_boost_timer.has_elapsed(profile.boost_duration)) {
            _is_breakaway_boost_active = false;
        }
    }

    // Set torque limits
    _torque_limit_max_pu = profile.torque_limit_max;
    _torque_limit_min_pu = profile.torque_limit_min;

    // Add breakaway boost if active
    if (_is_breakaway_boost_active) {
        _torque_limit_max_pu = torque_pu_t{_torque_limit_max_pu.value() + profile.initial_torque_boost.value()};
    }
}

void sfc_soft_starter_speed_generator::execute_hold_mode()
{
    // In hold mode, speed reference is constant
    _speed_reference_pu = _hold_speed_setpoint;
    _is_profile_complete = true;  // Always "complete" in hold mode

    // Set moderate torque limits for speed regulation
    _torque_limit_max_pu = torque_pu_t{0.8};
    _torque_limit_min_pu = torque_pu_t{-0.3};
}

void sfc_soft_starter_speed_generator::execute_deceleration()
{
    // Ramp down to zero
    _speed_ramp_limiter.update(0.0);
    _speed_reference_pu = speed_pu_t{_speed_ramp_limiter.get_output()};

    // Check completion
    _is_profile_complete = (_speed_reference_pu.value() < 0.01);

    // Set deceleration torque limits
    _torque_limit_max_pu = torque_pu_t{0.2};   // Allow small motoring to prevent overshoot
    _torque_limit_min_pu = torque_pu_t{-1.0};  // Allow braking
}

void sfc_soft_starter_speed_generator::update_torque_limits()
{
    // This method can be extended to implement speed-dependent torque limiting
    // For now, torque limits are set by the profile configuration
}

size_t sfc_soft_starter_speed_generator::get_profile_index(acceleration_profile_t profile) const
{
    // Map profile enum to array index
    return static_cast<size_t>(profile);
}

}  // namespace yeet::application::sfc
