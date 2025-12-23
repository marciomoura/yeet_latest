#include "sfc_integration/sfc_torque_controller_runner.hpp"

namespace yeet::application::sfc::integration {

static bool is_shutdown_state(const yeet::application::sfc::sfc_state_id state_id) noexcept
{
    using yeet::application::sfc::sfc_state_id;
    return (state_id == sfc_state_id::SHUTDOWN_REQUESTED) || (state_id == sfc_state_id::OPERATION_OFF);
}

void sfc_torque_controller_runner::register_datamodel(yeet::telemetry::parameter_handler& handler)
{
    // Delegate to the datamodel's registration method
    sfc_torque_controller_datamodel datamodel(&_inputs, &_outputs, &_parameters);
    datamodel.register_data_model(handler);
}

void sfc_torque_controller_runner::initialize()
{
    set_parameters();
    set_parameter_callback();
}

void sfc_torque_controller_runner::run()
{
    // Override torque reference if enabled
    if (_parameters.torque_reference_override.read()) {
        const torque_pu_t torque_reference = _parameters.torque_reference_override_value.read();

        // Write output signals
        _outputs.current_reference.write(current_pu_t{torque_reference});
        _outputs.is_saturated.write(false);
        _outputs.is_breakaway_boost_active.write(false);
        _outputs.is_active.write(true);
        _outputs.torque_reference.write(torque_reference);
        _outputs.is_braking_requested.write(false);
        _outputs.speed_based_min_torque_limit.write(torque_pu_t{0.0});
        _outputs.speed_based_max_torque_limit.write(torque_pu_t{0.0});
        return;
    }

    const auto current_state_id = _inputs.current_state_id.read();
    const bool machine_protection_tripped = _inputs.machine_protection_tripped.read();
    const bool shutdown_active = is_shutdown_state(current_state_id) || machine_protection_tripped;
    const bool operation_on = (current_state_id == yeet::application::sfc::sfc_state_id::OPERATION_ON);

    // On any shutdown request: force exact zero torque reference and bypass
    // minimum motoring torque + rate limiting so output can actually reach 0.
    if (shutdown_active) {
        if (!_was_shutdown_active) {
            _sfc_torque_controller.reset();
        }

        _was_operation_on = false;

        // Ensure clamping/ramps cannot prevent reaching zero.
        _sfc_torque_controller.enable_torque_rate_limiting(false);
        _sfc_torque_controller.enable_minimum_motoring_torque(false);

        // Read limits and downstream saturation for completeness.
        const torque_pu_t max_torque_limit = _parameters.absolute_max_torque.read();
        const torque_pu_t min_torque_limit = _parameters.absolute_min_torque.read();
        const bool downstream_saturated = _inputs.is_saturated.read();
        _sfc_torque_controller.set_downstream_saturation(downstream_saturated);

        // Mode doesn't matter since we force torque_reference=0 and manual torque is not used.
        _sfc_torque_controller.update(torque_pu_t{0.0}, max_torque_limit, min_torque_limit, speed_pu_t{0.0}, false);

        _outputs.current_reference.write(current_pu_t{0.0});
        _outputs.is_saturated.write(false);
        _outputs.is_breakaway_boost_active.write(false);
        _outputs.is_active.write(true);
        _outputs.torque_reference.write(torque_pu_t{0.0});
        _outputs.is_braking_requested.write(false);
        _outputs.speed_based_min_torque_limit.write(torque_pu_t{0.0});
        _outputs.speed_based_max_torque_limit.write(torque_pu_t{0.0});

        _was_shutdown_active = true;
        return;
    }

    // Leaving shutdown: restore configured limiter behaviors.
    if (_was_shutdown_active) {
        _sfc_torque_controller.reset();
        _was_shutdown_active = false;
    }

    // Outside OPERATION_ON we are not in closed-loop operation. Force exact zero torque/current
    // reference and disable minimum-motoring clamp so we don't get a misleading 0.02 pu reference
    // while not modulating.
    if (!operation_on) {
        if (_was_operation_on) {
            _sfc_torque_controller.reset();
            _was_operation_on = false;
        }

        _sfc_torque_controller.enable_torque_rate_limiting(false);
        _sfc_torque_controller.enable_minimum_motoring_torque(false);
        _sfc_torque_controller.set_downstream_saturation(_inputs.is_saturated.read());

        // Still call update() to keep internal state coherent, but force 0 output.
        const torque_pu_t max_torque_limit = _parameters.absolute_max_torque.read();
        const torque_pu_t min_torque_limit = _parameters.absolute_min_torque.read();
        _sfc_torque_controller.update(torque_pu_t{0.0}, max_torque_limit, min_torque_limit, speed_pu_t{0.0}, false);

        _outputs.current_reference.write(current_pu_t{0.0});
        _outputs.is_saturated.write(false);
        _outputs.is_breakaway_boost_active.write(false);
        _outputs.is_active.write(false);
        _outputs.torque_reference.write(torque_pu_t{0.0});
        _outputs.is_braking_requested.write(false);
        _outputs.speed_based_min_torque_limit.write(torque_pu_t{0.0});
        _outputs.speed_based_max_torque_limit.write(torque_pu_t{0.0});
        return;
    }

    if (!_was_operation_on) {
        // Entering OPERATION_ON: reset for clean behavior.
        _sfc_torque_controller.reset();
        _was_operation_on = true;
    }

    _sfc_torque_controller.enable_torque_rate_limiting(_parameters.enable_torque_rate_limiting.read());
    _sfc_torque_controller.enable_minimum_motoring_torque(_parameters.enable_minimum_motoring_torque.read());

    // Read input signals
    const torque_pu_t torque_reference = _inputs.torque_reference.read();
    const speed_pu_t speed_reference = _inputs.speed_reference.read();
    const torque_pu_t max_torque_limit = _parameters.absolute_max_torque.read();
    const torque_pu_t min_torque_limit = _parameters.absolute_min_torque.read();
    const bool enable_boost = _inputs.enable_breakaway_boost.read();
    const bool downstream_saturated = _inputs.is_saturated.read();
    const auto control_mode = _inputs.control_mode.read();
    const torque_pu_t manual_torque_ref = _inputs.manual_torque_reference.read();

    // Update control mode and manual torque reference from startup sequencer
    _sfc_torque_controller.set_control_mode(control_mode);
    _sfc_torque_controller.set_manual_torque_reference(manual_torque_ref);

    // Update downstream saturation feedback (must be done before update())
    _sfc_torque_controller.set_downstream_saturation(downstream_saturated);

    // Execute torque controller
    _sfc_torque_controller.update(torque_reference, max_torque_limit, min_torque_limit, speed_reference, enable_boost);

    // Write output signals (composite saturation becomes unified is_saturated)
    _outputs.current_reference.write(_sfc_torque_controller.get_current_reference());
    _outputs.is_saturated.write(_sfc_torque_controller.is_saturated());
    _outputs.is_breakaway_boost_active.write(_sfc_torque_controller.is_breakaway_boost_active());
    _outputs.is_active.write(true);
    _outputs.torque_reference.write(_sfc_torque_controller.get_limited_torque_reference());
    _outputs.is_braking_requested.write(_sfc_torque_controller.is_braking_requested());
    _outputs.speed_based_min_torque_limit.write(_sfc_torque_controller.get_speed_based_min_torque_limit());
    _outputs.speed_based_max_torque_limit.write(_sfc_torque_controller.get_speed_based_max_torque_limit());
}

void sfc_torque_controller_runner::stop()
{
    _sfc_torque_controller.reset();
    _was_shutdown_active = false;
}

void sfc_torque_controller_runner::set_parameters()
{
    // Configure absolute torque limits
    const auto abs_min_torque = _parameters.absolute_min_torque;
    const auto abs_max_torque = _parameters.absolute_max_torque;
    _sfc_torque_controller.configure_absolute_torque_limits(abs_min_torque, abs_max_torque);

    // Configure torque-to-current conversion
    const auto torque_to_current = _parameters.torque_to_current_factor;
    _sfc_torque_controller.configure_torque_to_current_factor(torque_to_current);

    // Configure rate limiting
    const auto rate_limit = _parameters.torque_rate_limit;
    const auto enable_rate_limit = _parameters.enable_torque_rate_limiting;
    _sfc_torque_controller.configure_torque_rate_limit(rate_limit);
    _sfc_torque_controller.enable_torque_rate_limiting(enable_rate_limit);

    // Configure breakaway boost
    const auto boost_torque = _parameters.breakaway_boost_torque;
    const auto boost_duration = _parameters.breakaway_boost_duration;
    _sfc_torque_controller.configure_breakaway_boost(boost_torque, boost_duration);

    // Configure minimum motoring torque for natural commutation
    const auto min_motoring_torque = _parameters.minimum_motoring_torque;
    const auto enable_min_motoring = _parameters.enable_minimum_motoring_torque;
    _sfc_torque_controller.configure_minimum_motoring_torque(min_motoring_torque);
    _sfc_torque_controller.enable_minimum_motoring_torque(enable_min_motoring);

    // Configure speed-based torque limits (speed reference -> min/max torque)
    const auto enable_speed_limits = _parameters.enable_speed_based_torque_limits;
    const auto speed_axis = _parameters.speed_torque_limit_speed_axis_pu;
    const auto min_torque = _parameters.speed_torque_limit_min_torque_pu;
    const auto max_torque = _parameters.speed_torque_limit_max_torque_pu;
    _sfc_torque_controller.configure_speed_based_torque_limit_lookup_table(speed_axis, min_torque, max_torque);
    _sfc_torque_controller.enable_speed_based_torque_limits(enable_speed_limits);
}

void sfc_torque_controller_runner::set_parameter_callback()
{
    _parameters.absolute_min_torque.register_callback([&]() {
        _sfc_torque_controller.configure_absolute_torque_limits(
            _parameters.absolute_min_torque, _parameters.absolute_max_torque);
    });
    _parameters.absolute_max_torque.register_callback([&]() {
        _sfc_torque_controller.configure_absolute_torque_limits(
            _parameters.absolute_min_torque, _parameters.absolute_max_torque);
    });
    _parameters.torque_to_current_factor.register_callback(
        [&]() { _sfc_torque_controller.configure_torque_to_current_factor(_parameters.torque_to_current_factor); });
    _parameters.torque_rate_limit.register_callback(
        [&]() { _sfc_torque_controller.configure_torque_rate_limit(_parameters.torque_rate_limit); });
    _parameters.enable_torque_rate_limiting.register_callback(
        [&]() { _sfc_torque_controller.enable_torque_rate_limiting(_parameters.enable_torque_rate_limiting); });
    _parameters.breakaway_boost_torque.register_callback([&]() {
        _sfc_torque_controller.configure_breakaway_boost(
            _parameters.breakaway_boost_torque, _parameters.breakaway_boost_duration);
    });
    _parameters.breakaway_boost_duration.register_callback([&]() {
        _sfc_torque_controller.configure_breakaway_boost(
            _parameters.breakaway_boost_torque, _parameters.breakaway_boost_duration);
    });
    _parameters.minimum_motoring_torque.register_callback(
        [&]() { _sfc_torque_controller.configure_minimum_motoring_torque(_parameters.minimum_motoring_torque); });
    _parameters.enable_minimum_motoring_torque.register_callback(
        [&]() { _sfc_torque_controller.enable_minimum_motoring_torque(_parameters.enable_minimum_motoring_torque); });
    _parameters.enable_speed_based_torque_limits.register_callback([&]() {
        _sfc_torque_controller.enable_speed_based_torque_limits(_parameters.enable_speed_based_torque_limits);
    });
    _parameters.speed_torque_limit_speed_axis_pu.register_callback([&]() {
        _sfc_torque_controller.configure_speed_based_torque_limit_lookup_table(
            _parameters.speed_torque_limit_speed_axis_pu, _parameters.speed_torque_limit_min_torque_pu,
            _parameters.speed_torque_limit_max_torque_pu);
    });
    _parameters.speed_torque_limit_min_torque_pu.register_callback([&]() {
        _sfc_torque_controller.configure_speed_based_torque_limit_lookup_table(
            _parameters.speed_torque_limit_speed_axis_pu, _parameters.speed_torque_limit_min_torque_pu,
            _parameters.speed_torque_limit_max_torque_pu);
    });
    _parameters.speed_torque_limit_max_torque_pu.register_callback([&]() {
        _sfc_torque_controller.configure_speed_based_torque_limit_lookup_table(
            _parameters.speed_torque_limit_speed_axis_pu, _parameters.speed_torque_limit_min_torque_pu,
            _parameters.speed_torque_limit_max_torque_pu);
    });
}

}  // namespace yeet::application::sfc::integration
