#include "sfc/sfc_torque_controller.hpp"

#include <algorithm>
#include <cassert>
#include <limits>

namespace yeet::application::sfc {

sfc_torque_controller::sfc_torque_controller(duration_t sampling_time)
    : _torque_rate_limiter(sampling_time.value()), _breakaway_boost_timer(sampling_time), _sampling_time(sampling_time)
{
    // Initialize with safe defaults
    configure_absolute_torque_limits(torque_pu_t{-1.2}, torque_pu_t{1.2});
    configure_torque_to_current_factor(1.0);
    configure_torque_rate_limit(10.0);  // 10 pu/s default
    enable_torque_rate_limiting(true);
    configure_breakaway_boost(torque_pu_t{0.0}, duration_t{0.0});
    configure_speed_based_torque_limit_lookup_table(
        kDefaultSpeedTorqueLimitSpeedAxisPu, kDefaultSpeedTorqueLimitMinTorquePu, kDefaultSpeedTorqueLimitMaxTorquePu);
    enable_speed_based_torque_limits(false);
    set_control_mode(control_mode::SPEED_CONTROL);
}

// ========== Configuration Methods ==========

void sfc_torque_controller::configure_absolute_torque_limits(torque_pu_t min_torque_pu, torque_pu_t max_torque_pu)
{
    assert(min_torque_pu.value() < max_torque_pu.value() && "Minimum torque must be less than maximum torque");
    _absolute_min_torque = min_torque_pu;
    _absolute_max_torque = max_torque_pu;
}

void sfc_torque_controller::configure_torque_to_current_factor(real_t torque_to_current_factor)
{
    assert(torque_to_current_factor > 0.0 && "Torque-to-current factor must be positive");
    _torque_to_current_factor = torque_to_current_factor;
}

void sfc_torque_controller::configure_torque_rate_limit(real_t rate_limit_pu_per_second)
{
    assert(rate_limit_pu_per_second > 0.0 && "Torque rate limit must be positive");
    _torque_rate_limiter.configure(rate_limit_pu_per_second);
}

void sfc_torque_controller::enable_torque_rate_limiting(bool enable) { _torque_rate_limiter.configure_enable(enable); }

void sfc_torque_controller::set_control_mode(control_mode mode) { _control_mode = mode; }

void sfc_torque_controller::set_manual_torque_reference(torque_pu_t torque_pu) { _manual_torque_reference = torque_pu; }

void sfc_torque_controller::configure_breakaway_boost(torque_pu_t boost_torque_pu, duration_t boost_duration)
{
    _breakaway_boost_torque = boost_torque_pu;
    _breakaway_boost_duration = boost_duration;
}

void sfc_torque_controller::configure_minimum_motoring_torque(torque_pu_t min_motoring_torque_pu)
{
    assert(min_motoring_torque_pu.value() >= 0.0 && "Minimum motoring torque must be non-negative");
    _minimum_motoring_torque = min_motoring_torque_pu;
}

void sfc_torque_controller::enable_minimum_motoring_torque(bool enable) { _enable_minimum_motoring_torque = enable; }

void sfc_torque_controller::configure_speed_based_torque_limit_lookup_table(
    const std::array<speed_pu_t, NUM_SPEED_TORQUE_LIMIT_POINTS>& speed_axis_pu,
    const std::array<torque_pu_t, NUM_SPEED_TORQUE_LIMIT_POINTS>& min_torque_limit_pu,
    const std::array<torque_pu_t, NUM_SPEED_TORQUE_LIMIT_POINTS>& max_torque_limit_pu)
{
    for (size_t i = 0; i < NUM_SPEED_TORQUE_LIMIT_POINTS; ++i) {
        assert(speed_axis_pu[i].value() >= 0.0 && "Speed axis values must be non-negative.");
        assert(min_torque_limit_pu[i].value() >= 0.0 && "Min torque limits must be non-negative for motoring.");
        assert(max_torque_limit_pu[i].value() >= 0.0 && "Max torque limits must be non-negative for motoring.");
        assert(min_torque_limit_pu[i].value() <= max_torque_limit_pu[i].value() &&
               "Min torque limit must be <= max torque limit at each breakpoint.");
    }

    std::array<real_t, NUM_SPEED_TORQUE_LIMIT_POINTS> speed_axis{};
    std::array<real_t, NUM_SPEED_TORQUE_LIMIT_POINTS> min_torque{};
    std::array<real_t, NUM_SPEED_TORQUE_LIMIT_POINTS> max_torque{};

    for (size_t i = 0; i < NUM_SPEED_TORQUE_LIMIT_POINTS; ++i) {
        speed_axis[i] = static_cast<real_t>(speed_axis_pu[i].value());
        min_torque[i] = static_cast<real_t>(min_torque_limit_pu[i].value());
        max_torque[i] = static_cast<real_t>(max_torque_limit_pu[i].value());
    }

    _speed_torque_min_limit_lookup.configure(speed_axis, min_torque);
    _speed_torque_max_limit_lookup.configure(speed_axis, max_torque);
}

void sfc_torque_controller::enable_speed_based_torque_limits(bool enable) noexcept
{
    _enable_speed_based_torque_limits = enable;
}

// ========== Real-Time Execution ==========

void sfc_torque_controller::reset()
{
    _torque_rate_limiter.reset();
    _breakaway_boost_timer.reset();
    _base_torque_reference = torque_pu_t{0.0};
    _total_torque_reference = torque_pu_t{0.0};
    _limited_torque_reference = torque_pu_t{0.0};
    _current_reference = current_pu_t{0.0};
    _is_breakaway_boost_active = false;
    _is_saturated = false;
    _downstream_saturation = false;
    _is_braking_requested = false;
    _speed_based_min_torque_limit = torque_pu_t{0.0};
    _speed_based_max_torque_limit = torque_pu_t{0.0};
}

void sfc_torque_controller::update(torque_pu_t torque_reference,
    torque_pu_t dynamic_max_torque_limit,
    torque_pu_t dynamic_min_torque_limit,
    speed_pu_t measured_speed_pu,
    bool enable_breakaway_boost)
{
    // Step 1: Update dynamic limits
    // Evaluate speed-based schedule (always computed for observability)
    // We use measured speed for physical capability limits as it represents the machine's actual state.
    _speed_based_min_torque_limit = torque_pu_t{_speed_torque_min_limit_lookup.get_value(measured_speed_pu.value())};
    _speed_based_max_torque_limit = torque_pu_t{_speed_torque_max_limit_lookup.get_value(measured_speed_pu.value())};

    // Combine external dynamic limits with speed-based limits (if enabled)
    if (_enable_speed_based_torque_limits) {
        _dynamic_max_torque_limit =
            torque_pu_t{std::min(dynamic_max_torque_limit.value(), _speed_based_max_torque_limit.value())};
        _dynamic_min_torque_limit =
            torque_pu_t{std::max(dynamic_min_torque_limit.value(), _speed_based_min_torque_limit.value())};
    }
    else {
        _dynamic_max_torque_limit = dynamic_max_torque_limit;
        _dynamic_min_torque_limit = dynamic_min_torque_limit;
    }

    // Step 2: Arbitrate torque reference source
    if (_control_mode == control_mode::MANUAL_TORQUE) {
        _base_torque_reference = _manual_torque_reference;
    }
    else {
        _base_torque_reference = torque_reference;
    }

    // Step 3: Handle breakaway boost
    if (enable_breakaway_boost) {
        // Activate boost if not already active
        if (!_is_breakaway_boost_active) {
            _is_breakaway_boost_active = true;
            _breakaway_boost_timer.reset();
        }

        // Update boost timer
        _breakaway_boost_timer.update(true);

        // Check if boost duration has expired
        if (_breakaway_boost_timer.has_elapsed(_breakaway_boost_duration)) {
            _is_breakaway_boost_active = false;
        }
    }
    else {
        // Boost not requested, deactivate if active
        _is_breakaway_boost_active = false;
        _breakaway_boost_timer.reset();
    }

    // Step 4: Add breakaway boost to base reference
    if (_is_breakaway_boost_active) {
        _total_torque_reference = _base_torque_reference + _breakaway_boost_torque;
    }
    else {
        _total_torque_reference = _base_torque_reference;
    }

    // Step 5: Apply torque limits (both absolute and dynamic)
    // This also sets _is_braking_requested based on the user's torque request
    const torque_pu_t limited_torque = apply_torque_limits(_total_torque_reference);

    // Save the braking request flag from the user's original intent
    const bool braking_was_requested = _is_braking_requested;

    // Step 6: Apply rate limiting to prevent mechanical shock
    _torque_rate_limiter.update(limited_torque.value());
    torque_pu_t rate_limited_torque = torque_pu_t{_torque_rate_limiter.get_output()};

    // Apply limits again to rate-limited output to ensure dynamic limit changes are immediate
    // This might overwrite _is_braking_requested, so we restore it afterward
    _limited_torque_reference = apply_torque_limits(rate_limited_torque);

    // Restore the braking request flag based on user's original intent, not rate-limited value
    _is_braking_requested = braking_was_requested;

    // Step 7: Detect saturation for anti-windup feedback
    // Saturation occurs if the requested torque differs from the final limited torque
    const real_t saturation_threshold = 0.001;  // 0.1% tolerance
    _is_saturated =
        std::abs(_total_torque_reference.value() - _limited_torque_reference.value()) > saturation_threshold;

    // Step 8: Convert torque to current reference
    _current_reference = current_pu_t{_limited_torque_reference.value() * _torque_to_current_factor};
}

void sfc_torque_controller::set_downstream_saturation(bool saturated) noexcept { _downstream_saturation = saturated; }

// ========== Output Accessors ==========

current_pu_t sfc_torque_controller::get_current_reference() const { return _current_reference; }

torque_pu_t sfc_torque_controller::get_base_torque_reference() const { return _base_torque_reference; }

torque_pu_t sfc_torque_controller::get_total_torque_reference() const { return _total_torque_reference; }

torque_pu_t sfc_torque_controller::get_limited_torque_reference() const { return _limited_torque_reference; }

torque_pu_t sfc_torque_controller::get_speed_based_min_torque_limit() const { return _speed_based_min_torque_limit; }

torque_pu_t sfc_torque_controller::get_speed_based_max_torque_limit() const { return _speed_based_max_torque_limit; }

sfc_torque_controller::control_mode sfc_torque_controller::get_control_mode() const { return _control_mode; }

bool sfc_torque_controller::is_breakaway_boost_active() const { return _is_breakaway_boost_active; }

bool sfc_torque_controller::is_saturated() const { return _is_saturated; }

bool sfc_torque_controller::get_downstream_saturation() const noexcept { return _downstream_saturation; }

bool sfc_torque_controller::get_composite_saturation() const noexcept
{
    return _is_saturated || _downstream_saturation;
}

bool sfc_torque_controller::is_braking_requested() const noexcept { return _is_braking_requested; }

// ========== Internal Helper Methods ==========

torque_pu_t sfc_torque_controller::apply_torque_limits(torque_pu_t torque_pu)
{
    // Combine absolute and dynamic limits
    // The effective limit is the most restrictive (smallest range)
    real_t effective_min = std::max(_absolute_min_torque.value(), _dynamic_min_torque_limit.value());
    const real_t effective_max = std::min(_absolute_max_torque.value(), _dynamic_max_torque_limit.value());

    // Enforce minimum motoring torque for natural commutation
    // When enabled, clamp the lower limit to prevent negative (braking) torque
    if (_enable_minimum_motoring_torque) {
        // Track if braking was requested (input < minimum motoring torque)
        _is_braking_requested = torque_pu.value() < _minimum_motoring_torque.value();

        // Clamp effective minimum to minimum motoring torque
        effective_min = std::max(effective_min, _minimum_motoring_torque.value());
    }
    else {
        _is_braking_requested = false;
    }

    // Apply combined limits
    return torque_pu_t{std::clamp(torque_pu.value(), effective_min, effective_max)};
}

}  // namespace yeet::application::sfc
