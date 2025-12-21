#include "sfc_integration/sfc_machine_io_mock_runner.hpp"

#include "yeet/utils/parameter_accessors.hpp"

namespace yeet::application::sfc::integration {

void sfc_machine_io_mock_runner::register_datamodel(yeet::model_registry& registry)
{
    yeet::model_registry_entry entry{yeet::signal_registry{}, yeet::parameter_registry{}};

    // Timing parameters
    entry.parameters.insert(
        "excitation_startup_delay", yeet::utils::make_units_accessor(_parameters.excitation_startup_delay));
    entry.parameters.insert(
        "excitation_shutdown_delay", yeet::utils::make_units_accessor(_parameters.excitation_shutdown_delay));

    // External supervisory inputs
    entry.parameters.insert("external_fault", yeet::utils::make_bool_accessor(_parameters.external_fault));

    // Output overrides
    entry.parameters.insert(
        "override_excitation_status", yeet::utils::make_bool_accessor(_parameters.override_excitation_status));
    entry.parameters.insert("override_excitation_status_value",
        yeet::utils::make_bool_accessor(_parameters.override_excitation_status_value));

    registry.insert("machine_io_mock", std::move(entry));
}

void sfc_machine_io_mock_runner::initialize()
{
    // Initialize with safe defaults - excitation off
    _excitation_running = false;

    // Reset timer
    _excitation_transition_timer = duration_t{0.0};

    // Reset last commanded states
    _last_excitation_on_cmd = false;
    _last_excitation_off_cmd = false;

    // Push initial values to external_ios
    run();
}

void sfc_machine_io_mock_runner::run()
{
    // Update hardware model
    update_excitation_dynamics();

    // Apply overrides and write status to external_ios
    apply_overrides_and_write_to_external_ios();
}

void sfc_machine_io_mock_runner::stop()
{
    // Reset to safe state - excitation off
    _excitation_running = false;

    // Write safe state to signal outputs
    _outputs.excitation_status = false;
    _outputs.external_fault = false;
}

void sfc_machine_io_mock_runner::update_excitation_dynamics()
{
    // Helper to safely read optional command inputs (return false if not connected)
    auto safe_read = [](const signal_input<bool>& input) { return !input.empty() && input.read(); };

    // Read commands from control system via signal inputs
    const bool on_cmd = safe_read(_inputs.excitation_on_command);
    const bool off_cmd = safe_read(_inputs.excitation_off_command);

    // Detect command edges to reset timer
    if (on_cmd != _last_excitation_on_cmd || off_cmd != _last_excitation_off_cmd) {
        _excitation_transition_timer = duration_t{0.0};
    }
    _last_excitation_on_cmd = on_cmd;
    _last_excitation_off_cmd = off_cmd;

    // Safety: OFF command always wins
    if (off_cmd) {
        // Model shutdown delay
        _excitation_transition_timer = _excitation_transition_timer + kSamplingTime;
        if (_excitation_transition_timer >= _parameters.excitation_shutdown_delay.read()) {
            _excitation_running = false;
        }
    }
    // ON command: check power availability
    else if (on_cmd && !off_cmd) {
        // Model startup delay
        _excitation_transition_timer = _excitation_transition_timer + kSamplingTime;
        if (_excitation_transition_timer >= _parameters.excitation_startup_delay.read()) {
            _excitation_running = true;
        }
    }
}

void sfc_machine_io_mock_runner::apply_overrides_and_write_to_external_ios()
{
    // Apply override if enabled (for fault injection testing)
    bool excitation_feedback = _excitation_running;
    if (_parameters.override_excitation_status.read()) {
        excitation_feedback = _parameters.override_excitation_status_value.read();
    }

    // Write all hardware status to signal outputs
    _outputs.excitation_status = excitation_feedback;
    _outputs.external_fault = _parameters.external_fault.read();
    _outputs.start_command = _parameters.start_command.read();
}

}  // namespace yeet::application::sfc::integration
