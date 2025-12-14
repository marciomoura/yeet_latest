#include "sfc_integration/sfc_io_mock_runner.hpp"

#include "yeet/utils/parameter_accessors.hpp"

namespace yeet::application::sfc::integration {

sfc_io_mock_runner::sfc_io_mock_runner(sfc_grid_side_external_ios_datamodel& external_ios) : _external_ios(external_ios)
{
}

void sfc_io_mock_runner::register_datamodel(yeet::model_registry& registry)
{
    yeet::model_registry_entry entry{yeet::signal_registry{}, yeet::parameter_registry{}};

    entry.parameters.insert("auxiliaries_are_on", yeet::utils::make_bool_accessor(_parameters.auxiliaries_are_on));
    entry.parameters.insert("mcb_is_closed", yeet::utils::make_bool_accessor(_parameters.mcb_is_closed));
    entry.parameters.insert("start_command", yeet::utils::make_bool_accessor(_parameters.start_command));
    entry.parameters.insert("stop_command", yeet::utils::make_bool_accessor(_parameters.stop_command));
    entry.parameters.insert("shutdown_request", yeet::utils::make_bool_accessor(_parameters.shutdown_request));
    entry.parameters.insert("breaker_is_closed", yeet::utils::make_bool_accessor(_parameters.breaker_is_closed));
    entry.parameters.insert("breaker_is_tripping", yeet::utils::make_bool_accessor(_parameters.breaker_is_tripping));
    entry.parameters.insert("emergency_off", yeet::utils::make_bool_accessor(_parameters.emergency_off));
    entry.parameters.insert("door_is_open", yeet::utils::make_bool_accessor(_parameters.door_is_open));

    registry.insert("io_mock", std::move(entry));
}

void sfc_io_mock_runner::initialize()
{
    // Initialize with safe defaults - all inputs off
    _parameters.auxiliaries_are_on.write(false);
    _parameters.mcb_is_closed.write(false);
    _parameters.start_command.write(false);
    _parameters.stop_command.write(false);
    _parameters.shutdown_request.write(false);
    _parameters.breaker_is_closed.write(false);
    _parameters.breaker_is_tripping.write(false);
    _parameters.emergency_off.write(false);
    _parameters.door_is_open.write(false);

    // Push initial values to external_ios
    run();
}

void sfc_io_mock_runner::run()
{
    // Idealized mock behavior: auxiliaries status follows the commanded output when auxiliaries are available.
    // This preserves the ability to simulate "auxiliaries not ready" while ensuring trips force auxiliaries off.
    const bool auxiliaries_are_on = _external_ios.get_auxiliaries_on_command() && _parameters.auxiliaries_are_on.read();

    _external_ios.set_digital_inputs(auxiliaries_are_on, _parameters.mcb_is_closed.read(),
        _parameters.start_command.read(), _parameters.stop_command.read(), _parameters.shutdown_request.read(),
        _parameters.breaker_is_closed.read(), _parameters.breaker_is_tripping.read(), _parameters.emergency_off.read(),
        _parameters.door_is_open.read());
}

void sfc_io_mock_runner::stop()
{
    // Reset to safe state
    _external_ios.set_digital_inputs(false, false, false, false, false, false, false, false, false);
}

}  // namespace yeet::application::sfc::integration
