#include "sfc_integration/sfc_soft_starter_speed_generator_runner.hpp"

namespace yeet::application::sfc::integration {

void sfc_soft_starter_speed_generator_runner::register_datamodel(yeet::model_registry& registry)
{
    // Delegate to the datamodel's registration method
    sfc_soft_starter_speed_generator_datamodel datamodel(&_inputs, &_outputs, &_parameters);
    datamodel.register_data_model(registry);
}

void sfc_soft_starter_speed_generator_runner::initialize()
{
    // Reset the speed generator to initial state
    _speed_generator.reset();

    // Configure all parameters from datamodel
    set_parameters();

    // Register callbacks for dynamic parameter updates
    set_parameter_callback();

    // Initialize command edge detection flags
    _prev_run_profile_1 = false;
    _prev_run_profile_2 = false;
    _prev_run_profile_3 = false;
    _prev_run_profile_4 = false;
    _prev_hold_speed = false;
    _prev_decelerate = false;
    _prev_idle = false;

    // Start in IDLE command
    _speed_generator.execute_command(sfc::speed_command_t::IDLE);
}

void sfc_soft_starter_speed_generator_runner::run()
{
    if (_parameters.override_speed_reference.read()) {
        _outputs.speed_reference = _parameters.override_speed_reference_value.read();
        _outputs.torque_limit_max_pu = _parameters.override_max_torque_limit_value.read();
        _outputs.torque_limit_min_pu = _parameters.override_min_torque_limit_value.read();
        _outputs.is_profile_complete = false;
        _outputs.is_breakaway_boost_active = false;
        _outputs.active_command = sfc::speed_command_t::IDLE;
        _outputs.selected_profile = sfc::acceleration_profile_t::PROFILE_1;
        return;
    }

    // Handle command inputs with edge detection
    handle_commands();

    // Update the speed generator with measured speed
    _speed_generator.update(_inputs.measured_speed.read());

    // Write all outputs
    _outputs.speed_reference.write(_speed_generator.get_speed_reference_pu());
    _outputs.torque_limit_max_pu.write(_speed_generator.get_torque_limit_max_pu());
    _outputs.torque_limit_min_pu.write(_speed_generator.get_torque_limit_min_pu());
    _outputs.is_profile_complete.write(_speed_generator.is_profile_complete());
    _outputs.is_breakaway_boost_active.write(_speed_generator.is_breakaway_boost_active());
    _outputs.active_command.write(_speed_generator.get_active_command());
    _outputs.selected_profile.write(_speed_generator.get_selected_profile());
}

void sfc_soft_starter_speed_generator_runner::stop()
{
    // Command IDLE and reset to safe state
    _speed_generator.execute_command(sfc::speed_command_t::IDLE);
    _speed_generator.reset();
}

void sfc_soft_starter_speed_generator_runner::set_parameters()
{
    // Configure Profile 1
    if (_parameters.profile_1_enabled.read()) {
        auto profile_1 = build_profile_config(sfc::acceleration_profile_t::PROFILE_1);
        _speed_generator.configure_profile(sfc::acceleration_profile_t::PROFILE_1, profile_1);
    }
    else {
        _speed_generator.disable_profile(sfc::acceleration_profile_t::PROFILE_1);
    }

    // Configure Profile 2
    if (_parameters.profile_2_enabled.read()) {
        auto profile_2 = build_profile_config(sfc::acceleration_profile_t::PROFILE_2);
        _speed_generator.configure_profile(sfc::acceleration_profile_t::PROFILE_2, profile_2);
    }
    else {
        _speed_generator.disable_profile(sfc::acceleration_profile_t::PROFILE_2);
    }

    // Configure Profile 3
    if (_parameters.profile_3_enabled.read()) {
        auto profile_3 = build_profile_config(sfc::acceleration_profile_t::PROFILE_3);
        _speed_generator.configure_profile(sfc::acceleration_profile_t::PROFILE_3, profile_3);
    }
    else {
        _speed_generator.disable_profile(sfc::acceleration_profile_t::PROFILE_3);
    }

    // Configure Profile 4
    if (_parameters.profile_4_enabled.read()) {
        auto profile_4 = build_profile_config(sfc::acceleration_profile_t::PROFILE_4);
        _speed_generator.configure_profile(sfc::acceleration_profile_t::PROFILE_4, profile_4);
    }
    else {
        _speed_generator.disable_profile(sfc::acceleration_profile_t::PROFILE_4);
    }

    // Configure deceleration time
    _speed_generator.configure_deceleration(_parameters.deceleration_time.read());

    // Configure absolute safety limits
    _speed_generator.configure_absolute_limits(_parameters.min_speed_limit.read(), _parameters.max_speed_limit.read(),
        _parameters.max_torque_limit.read(), _parameters.min_torque_limit.read());
}

void sfc_soft_starter_speed_generator_runner::set_parameter_callback()
{
    // Register callbacks for all profile 1 parameters
    _parameters.profile_1_enabled.register_callback([&]() { set_parameters(); });
    _parameters.profile_1_target_speed.register_callback([&]() { set_parameters(); });
    _parameters.profile_1_acceleration_time.register_callback([&]() { set_parameters(); });
    _parameters.profile_1_torque_limit_max.register_callback([&]() { set_parameters(); });
    _parameters.profile_1_torque_limit_min.register_callback([&]() { set_parameters(); });
    _parameters.profile_1_initial_torque_boost.register_callback([&]() { set_parameters(); });
    _parameters.profile_1_boost_duration.register_callback([&]() { set_parameters(); });

    // Register callbacks for all profile 2 parameters
    _parameters.profile_2_enabled.register_callback([&]() { set_parameters(); });
    _parameters.profile_2_target_speed.register_callback([&]() { set_parameters(); });
    _parameters.profile_2_acceleration_time.register_callback([&]() { set_parameters(); });
    _parameters.profile_2_torque_limit_max.register_callback([&]() { set_parameters(); });
    _parameters.profile_2_torque_limit_min.register_callback([&]() { set_parameters(); });
    _parameters.profile_2_initial_torque_boost.register_callback([&]() { set_parameters(); });
    _parameters.profile_2_boost_duration.register_callback([&]() { set_parameters(); });

    // Register callbacks for all profile 3 parameters
    _parameters.profile_3_enabled.register_callback([&]() { set_parameters(); });
    _parameters.profile_3_target_speed.register_callback([&]() { set_parameters(); });
    _parameters.profile_3_acceleration_time.register_callback([&]() { set_parameters(); });
    _parameters.profile_3_torque_limit_max.register_callback([&]() { set_parameters(); });
    _parameters.profile_3_torque_limit_min.register_callback([&]() { set_parameters(); });
    _parameters.profile_3_initial_torque_boost.register_callback([&]() { set_parameters(); });
    _parameters.profile_3_boost_duration.register_callback([&]() { set_parameters(); });

    // Register callbacks for all profile 4 parameters
    _parameters.profile_4_enabled.register_callback([&]() { set_parameters(); });
    _parameters.profile_4_target_speed.register_callback([&]() { set_parameters(); });
    _parameters.profile_4_acceleration_time.register_callback([&]() { set_parameters(); });
    _parameters.profile_4_torque_limit_max.register_callback([&]() { set_parameters(); });
    _parameters.profile_4_torque_limit_min.register_callback([&]() { set_parameters(); });
    _parameters.profile_4_initial_torque_boost.register_callback([&]() { set_parameters(); });
    _parameters.profile_4_boost_duration.register_callback([&]() { set_parameters(); });

    // Register callbacks for deceleration and limits
    _parameters.deceleration_time.register_callback([&]() { set_parameters(); });
    _parameters.min_speed_limit.register_callback([&]() { set_parameters(); });
    _parameters.max_speed_limit.register_callback([&]() { set_parameters(); });
    _parameters.max_torque_limit.register_callback([&]() { set_parameters(); });
    _parameters.min_torque_limit.register_callback([&]() { set_parameters(); });
}

void sfc_soft_starter_speed_generator_runner::handle_commands()
{
    // Read all command inputs
    const bool current_run_profile_1 = _inputs.run_profile_1.read();
    const bool current_run_profile_2 = _inputs.run_profile_2.read();
    const bool current_run_profile_3 = _inputs.run_profile_3.read();
    const bool current_run_profile_4 = _inputs.run_profile_4.read();
    const bool current_hold_speed = _inputs.hold_speed.read();
    const bool current_decelerate = _inputs.decelerate.read();
    const bool current_idle = _inputs.idle.read();

    // Priority: IDLE > DECELERATE > HOLD > RUN_PROFILE (highest profile number wins)
    // This ensures safety commands take precedence

    // Detect rising edge on IDLE command (highest priority)
    if (current_idle && !_prev_idle) {
        _speed_generator.execute_command(sfc::speed_command_t::IDLE);
    }
    // Detect rising edge on DECELERATE command
    else if (current_decelerate && !_prev_decelerate) {
        _speed_generator.execute_command(sfc::speed_command_t::DECELERATE);
    }
    // Detect rising edge on HOLD_SPEED command
    else if (current_hold_speed && !_prev_hold_speed) {
        _speed_generator.execute_command(sfc::speed_command_t::HOLD_SPEED);
    }
    // Detect rising edge on RUN_PROFILE_4 command
    else if (current_run_profile_4 && !_prev_run_profile_4) {
        if (_speed_generator.select_profile(sfc::acceleration_profile_t::PROFILE_4)) {
            _speed_generator.execute_command(sfc::speed_command_t::RUN_PROFILE);
        }
    }
    // Detect rising edge on RUN_PROFILE_3 command
    else if (current_run_profile_3 && !_prev_run_profile_3) {
        if (_speed_generator.select_profile(sfc::acceleration_profile_t::PROFILE_3)) {
            _speed_generator.execute_command(sfc::speed_command_t::RUN_PROFILE);
        }
    }
    // Detect rising edge on RUN_PROFILE_2 command
    else if (current_run_profile_2 && !_prev_run_profile_2) {
        if (_speed_generator.select_profile(sfc::acceleration_profile_t::PROFILE_2)) {
            _speed_generator.execute_command(sfc::speed_command_t::RUN_PROFILE);
        }
    }
    // Detect rising edge on RUN_PROFILE_1 command
    else if (current_run_profile_1 && !_prev_run_profile_1) {
        if (_speed_generator.select_profile(sfc::acceleration_profile_t::PROFILE_1)) {
            _speed_generator.execute_command(sfc::speed_command_t::RUN_PROFILE);
        }
    }

    // Update previous command states for next cycle
    _prev_run_profile_1 = current_run_profile_1;
    _prev_run_profile_2 = current_run_profile_2;
    _prev_run_profile_3 = current_run_profile_3;
    _prev_run_profile_4 = current_run_profile_4;
    _prev_hold_speed = current_hold_speed;
    _prev_decelerate = current_decelerate;
    _prev_idle = current_idle;
}

sfc::acceleration_profile_config sfc_soft_starter_speed_generator_runner::build_profile_config(
    sfc::acceleration_profile_t profile)
{
    sfc::acceleration_profile_config config;
    config.enabled = true;

    // Select parameters based on profile number
    switch (profile) {
        case sfc::acceleration_profile_t::PROFILE_1:
            config.target_speed = _parameters.profile_1_target_speed.read();
            config.acceleration_time = _parameters.profile_1_acceleration_time.read();
            config.torque_limit_max = _parameters.profile_1_torque_limit_max.read();
            config.torque_limit_min = _parameters.profile_1_torque_limit_min.read();
            config.initial_torque_boost = _parameters.profile_1_initial_torque_boost.read();
            config.boost_duration = _parameters.profile_1_boost_duration.read();
            break;

        case sfc::acceleration_profile_t::PROFILE_2:
            config.target_speed = _parameters.profile_2_target_speed.read();
            config.acceleration_time = _parameters.profile_2_acceleration_time.read();
            config.torque_limit_max = _parameters.profile_2_torque_limit_max.read();
            config.torque_limit_min = _parameters.profile_2_torque_limit_min.read();
            config.initial_torque_boost = _parameters.profile_2_initial_torque_boost.read();
            config.boost_duration = _parameters.profile_2_boost_duration.read();
            break;

        case sfc::acceleration_profile_t::PROFILE_3:
            config.target_speed = _parameters.profile_3_target_speed.read();
            config.acceleration_time = _parameters.profile_3_acceleration_time.read();
            config.torque_limit_max = _parameters.profile_3_torque_limit_max.read();
            config.torque_limit_min = _parameters.profile_3_torque_limit_min.read();
            config.initial_torque_boost = _parameters.profile_3_initial_torque_boost.read();
            config.boost_duration = _parameters.profile_3_boost_duration.read();
            break;

        case sfc::acceleration_profile_t::PROFILE_4:
            config.target_speed = _parameters.profile_4_target_speed.read();
            config.acceleration_time = _parameters.profile_4_acceleration_time.read();
            config.torque_limit_max = _parameters.profile_4_torque_limit_max.read();
            config.torque_limit_min = _parameters.profile_4_torque_limit_min.read();
            config.initial_torque_boost = _parameters.profile_4_initial_torque_boost.read();
            config.boost_duration = _parameters.profile_4_boost_duration.read();
            break;
    }

    return config;
}

}  // namespace yeet::application::sfc::integration
