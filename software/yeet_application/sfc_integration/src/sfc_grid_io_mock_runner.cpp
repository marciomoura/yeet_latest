#include "sfc_integration/sfc_grid_io_mock_runner.hpp"

#include "yeet/utils/parameter_accessors.hpp"

namespace yeet::application::sfc::integration {

void sfc_grid_io_mock_runner::register_datamodel(yeet::model_registry& registry)
{
    yeet::model_registry_entry entry{yeet::signal_registry{}, yeet::parameter_registry{}};

    // Timing parameters
    entry.parameters.insert("breaker_close_delay", yeet::utils::make_units_accessor(_parameters.breaker_close_delay));
    entry.parameters.insert("breaker_open_delay", yeet::utils::make_units_accessor(_parameters.breaker_open_delay));
    entry.parameters.insert(
        "auxiliary_energize_delay", yeet::utils::make_units_accessor(_parameters.auxiliary_energize_delay));
    entry.parameters.insert("fan_spinup_delay", yeet::utils::make_units_accessor(_parameters.fan_spinup_delay));

    // Availability flags
    entry.parameters.insert(
        "auxiliaries_power_available", yeet::utils::make_bool_accessor(_parameters.auxiliaries_power_available));
    entry.parameters.insert(
        "breaker_mechanism_healthy", yeet::utils::make_bool_accessor(_parameters.breaker_mechanism_healthy));

    // External supervisory inputs
    entry.parameters.insert("start_command", yeet::utils::make_bool_accessor(_parameters.start_command));
    entry.parameters.insert("stop_command", yeet::utils::make_bool_accessor(_parameters.stop_command));
    entry.parameters.insert("emergency_off", yeet::utils::make_bool_accessor(_parameters.emergency_off));
    entry.parameters.insert("door_is_open", yeet::utils::make_bool_accessor(_parameters.door_is_open));
    entry.parameters.insert("external_fault", yeet::utils::make_bool_accessor(_parameters.external_fault));
    entry.parameters.insert(
        "mv_breaker_is_tripping", yeet::utils::make_bool_accessor(_parameters.mv_breaker_is_tripping));

    // Output overrides
    entry.parameters.insert(
        "override_mv_breaker_is_closed", yeet::utils::make_bool_accessor(_parameters.override_mv_breaker_is_closed));
    entry.parameters.insert("override_mv_breaker_is_closed_value",
        yeet::utils::make_bool_accessor(_parameters.override_mv_breaker_is_closed_value));
    entry.parameters.insert(
        "override_auxiliaries_are_on", yeet::utils::make_bool_accessor(_parameters.override_auxiliaries_are_on));
    entry.parameters.insert("override_auxiliaries_are_on_value",
        yeet::utils::make_bool_accessor(_parameters.override_auxiliaries_are_on_value));
    entry.parameters.insert(
        "override_fans_are_running", yeet::utils::make_bool_accessor(_parameters.override_fans_are_running));
    entry.parameters.insert("override_fans_are_running_value",
        yeet::utils::make_bool_accessor(_parameters.override_fans_are_running_value));

    registry.insert("grid_io_mock", std::move(entry));
}

void sfc_grid_io_mock_runner::initialize()
{
    // Initialize with safe defaults - all outputs off
    _breaker_position = false;
    _auxiliary_status = false;
    _fans_running = false;

    // Reset timers
    _breaker_transition_timer = duration_t{0.0};
    _auxiliary_energize_timer = duration_t{0.0};
    _fan_spinup_timer = duration_t{0.0};

    // Reset last commanded states
    _last_breaker_close_cmd = false;
    _last_breaker_open_cmd = false;
    _last_auxiliary_cmd = false;
    _last_fan_cmd = false;

    // Push initial values to external_ios
    run();
}

void sfc_grid_io_mock_runner::run()
{
    // Update hardware models
    update_breaker_dynamics();
    update_auxiliary_contactors();
    update_fan_status();

    // Apply overrides and write status to external_ios
    apply_overrides_and_write_to_external_ios();
}

void sfc_grid_io_mock_runner::stop()
{
    // Reset to safe state - all outputs off
    _breaker_position = false;
    _auxiliary_status = false;
    _fans_running = false;

    // Write safe state to signal outputs
    _outputs.auxiliaries_are_on = false;
    _outputs.mv_breaker_is_closed = false;
    _outputs.start_command = false;
    _outputs.stop_command = false;
    _outputs.mv_breaker_is_tripping = false;
    _outputs.emergency_off = false;
    _outputs.door_is_open = false;
    _outputs.external_fault = false;
    _outputs.fans_are_running = false;
}

void sfc_grid_io_mock_runner::update_breaker_dynamics()
{
    // Read commands from control system via signal inputs
    const bool close_cmd = _inputs.mv_breaker_close.read();
    const bool open_cmd = _inputs.mv_breaker_open.read();
    const bool door_open = _parameters.door_is_open.read();
    const bool mechanism_healthy = _parameters.breaker_mechanism_healthy.read();

    // Detect command edges to reset timer
    if (close_cmd != _last_breaker_close_cmd || open_cmd != _last_breaker_open_cmd) {
        _breaker_transition_timer = duration_t{0.0};
    }
    _last_breaker_close_cmd = close_cmd;
    _last_breaker_open_cmd = open_cmd;

    // Safety: open command always wins
    if (open_cmd && mechanism_healthy) {
        // Model opening delay
        _breaker_transition_timer = _breaker_transition_timer + kSamplingTime;
        if (_breaker_transition_timer >= _parameters.breaker_open_delay.read()) {
            _breaker_position = false;
        }
    }
    // Close command: check door interlock
    else if (close_cmd && !open_cmd && !door_open && mechanism_healthy) {
        // Model closing delay
        _breaker_transition_timer = _breaker_transition_timer + kSamplingTime;
        if (_breaker_transition_timer >= _parameters.breaker_close_delay.read()) {
            _breaker_position = true;
        }
    }
    // No command: breaker stays in current position
    else {
        _breaker_transition_timer = duration_t{0.0};
    }
}

void sfc_grid_io_mock_runner::update_auxiliary_contactors()
{
    // Read command from control system via signal inputs
    const bool aux_cmd = _inputs.auxiliaries_on_command.read();
    const bool power_available = _parameters.auxiliaries_power_available.read();
    const bool emergency_off = _parameters.emergency_off.read();

    // Detect command edge
    if (aux_cmd != _last_auxiliary_cmd) {
        _auxiliary_energize_timer = duration_t{0.0};
    }
    _last_auxiliary_cmd = aux_cmd;

    // Auxiliary energization: requires command + power + no E-stop
    if (aux_cmd && power_available && !emergency_off) {
        _auxiliary_energize_timer = _auxiliary_energize_timer + kSamplingTime;
        if (_auxiliary_energize_timer >= _parameters.auxiliary_energize_delay.read()) {
            _auxiliary_status = true;
        }
    }
    // De-energize immediately on command off or E-stop
    else {
        _auxiliary_status = false;
        _auxiliary_energize_timer = duration_t{0.0};
    }
}

void sfc_grid_io_mock_runner::update_fan_status()
{
    // Helper to safely read optional fan command inputs (return false if not connected)
    auto safe_read = [](const signal_input<bool>& input) { return !input.empty() && input.read(); };

    // Read fan commands from control system via signal inputs
    // Any fan commanded = fans running (simplified model)
    // Note: Fan command inputs are optional - if not connected, treat as no command
    const bool any_fan_cmd = safe_read(_inputs.acu_fan_1_on_command) || safe_read(_inputs.acu_fan_2_on_command) ||
                             safe_read(_inputs.acu_fan_3_on_command) || safe_read(_inputs.acu_fan_4_on_command);

    // Detect command edge
    if (any_fan_cmd != _last_fan_cmd) {
        _fan_spinup_timer = duration_t{0.0};
    }
    _last_fan_cmd = any_fan_cmd;

    // Model fan spin-up delay
    if (any_fan_cmd) {
        _fan_spinup_timer = _fan_spinup_timer + kSamplingTime;
        if (_fan_spinup_timer >= _parameters.fan_spinup_delay.read()) {
            _fans_running = true;
        }
    }
    // Fans stop immediately when command removed
    else {
        _fans_running = false;
        _fan_spinup_timer = duration_t{0.0};
    }
}

void sfc_grid_io_mock_runner::apply_overrides_and_write_to_external_ios()
{
    // Apply overrides if enabled (for fault injection testing)
    bool breaker_status = _breaker_position;
    if (_parameters.override_mv_breaker_is_closed.read()) {
        breaker_status = _parameters.override_mv_breaker_is_closed_value.read();
    }

    bool auxiliary_feedback = _auxiliary_status;
    if (_parameters.override_auxiliaries_are_on.read()) {
        auxiliary_feedback = _parameters.override_auxiliaries_are_on_value.read();
    }

    bool fans_feedback = _fans_running;
    if (_parameters.override_fans_are_running.read()) {
        fans_feedback = _parameters.override_fans_are_running_value.read();
    }

    // Write all hardware status to signal outputs
    // Note: auxiliaries_are_on must match both command AND status (loopback for availability check)
    const bool auxiliaries_input = _inputs.auxiliaries_on_command.read() && auxiliary_feedback;

    _outputs.auxiliaries_are_on = auxiliaries_input;  // auxiliaries_are_on (command AND status)
    _outputs.mv_breaker_is_closed = breaker_status;
    _outputs.start_command = _parameters.start_command.read();
    _outputs.stop_command = _parameters.stop_command.read();
    _outputs.shutdown_request = _parameters.shutdown_request.read();
    _outputs.mv_breaker_is_tripping = _parameters.mv_breaker_is_tripping.read();
    _outputs.emergency_off = _parameters.emergency_off.read();
    _outputs.door_is_open = _parameters.door_is_open.read();
    _outputs.external_fault = _parameters.external_fault.read();
    _outputs.fans_are_running = fans_feedback;
}

}  // namespace yeet::application::sfc::integration
