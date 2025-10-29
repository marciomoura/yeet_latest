#include "sfc_integration/sfc_torque_controller_runner.hpp"

namespace yeet::application::sfc::integration {

void sfc_torque_controller_runner::register_datamodel(yeet::model_registry& registry)
{
    // Delegate to the datamodel's registration method
    sfc_torque_controller_datamodel datamodel(&_inputs, &_outputs, &_parameters);
    datamodel.register_data_model(registry);
}

void sfc_torque_controller_runner::initialize()
{
    set_parameters();
    set_parameter_callback();
}

void sfc_torque_controller_runner::run()
{
    // Read input signals
    const auto torque_ref = _inputs.torque_reference_from_speed_controller.read();
    const auto dynamic_max_limit = _inputs.dynamic_max_torque_limit.read();
    const auto dynamic_min_limit = _inputs.dynamic_min_torque_limit.read();
    const auto enable_boost = _inputs.enable_breakaway_boost.read();
    const auto downstream_saturated = _inputs.current_controller_is_saturated.read();

    // Update downstream saturation feedback (must be done before update())
    _sfc_torque_controller.set_downstream_saturation(downstream_saturated);

    // Execute torque controller
    _sfc_torque_controller.update(torque_ref, dynamic_max_limit, dynamic_min_limit, enable_boost);

    // Write output signals
    _outputs.current_reference.write(_sfc_torque_controller.get_current_reference());
    _outputs.base_torque_reference.write(_sfc_torque_controller.get_base_torque_reference());
    _outputs.total_torque_reference.write(_sfc_torque_controller.get_total_torque_reference());
    _outputs.limited_torque_reference.write(_sfc_torque_controller.get_limited_torque_reference());
    _outputs.is_saturated.write(_sfc_torque_controller.is_saturated());
    _outputs.downstream_saturation.write(_sfc_torque_controller.get_downstream_saturation());
    _outputs.composite_saturation.write(_sfc_torque_controller.get_composite_saturation());
    _outputs.is_breakaway_boost_active.write(_sfc_torque_controller.is_breakaway_boost_active());
    _outputs.is_active.write(true);  // Always active when run() is called
}

void sfc_torque_controller_runner::stop() { _sfc_torque_controller.reset(); }

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
}

void sfc_torque_controller_runner::set_parameter_callback()
{
    _parameters.absolute_min_torque.register_callback([&]() { set_parameters(); });
    _parameters.absolute_max_torque.register_callback([&]() { set_parameters(); });
    _parameters.torque_to_current_factor.register_callback([&]() { set_parameters(); });
    _parameters.torque_rate_limit.register_callback([&]() { set_parameters(); });
    _parameters.enable_torque_rate_limiting.register_callback([&]() { set_parameters(); });
    _parameters.breakaway_boost_torque.register_callback([&]() { set_parameters(); });
    _parameters.breakaway_boost_duration.register_callback([&]() { set_parameters(); });
}

}  // namespace yeet::application::sfc::integration
