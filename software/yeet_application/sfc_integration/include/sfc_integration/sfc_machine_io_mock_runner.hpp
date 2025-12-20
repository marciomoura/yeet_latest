#pragma once

#include "sfc_integration/sfc_machine_side_external_ios_datamodel.hpp"
#include "yeet/model_registry.hpp"
#include "yeet/parameter.hpp"
#include "yeet/signal_wiring.hpp"
#include "yeet/utils/parameter_accessors.hpp"
#include "yeet_control/units.hpp"

namespace yeet::application::sfc::integration {

/**
 * @brief Machine-side IO mock runner - models external hardware behavior
 *
 * Simulates the physical hardware connected to the machine-side control system:
 * - Excitation controller: Models startup/shutdown delays and status feedback
 * - External inputs: Fault signals from supervisory system
 *
 * **Architecture:**
 * - Reads control commands from external_ios digital outputs (get_excitation_on_command, etc.)
 * - Models hardware dynamics (delays)
 * - Writes hardware status back to external_ios digital inputs (set_digital_inputs)
 *
 * **Testing Features:**
 * - Configurable timing parameters for realistic behavior
 * - Override capability for fault injection testing
 * - Clear separation between commanded state and actual hardware state
 */
class sfc_machine_io_mock_runner {
public:
    struct inputs {
        // Commands from control system (external_ios digital outputs)
        signal_input<bool> excitation_on_command;
        signal_input<bool> excitation_off_command;
    };

    struct mock_storage {
        bool excitation_on_command;
        bool excitation_off_command;
    };

    struct outputs {
        // Hardware status feedback (external_ios digital inputs)
        signal_output<bool> excitation_status;
        signal_output<bool> external_fault;
    };

    struct parameters {
        // ==================== TIMING PARAMETERS ====================
        /// Time for excitation system to start after ON command
        parameter<duration_t> excitation_startup_delay{duration_t{500e-3}};  // 500ms

        /// Time for excitation system to stop after OFF command
        parameter<duration_t> excitation_shutdown_delay{duration_t{200e-3}};  // 200ms

        // ==================== EXTERNAL SUPERVISORY INPUTS ====================
        /// External fault from protection relay
        parameter<bool> external_fault{false};

        // ==================== OUTPUT OVERRIDES (Fault Injection) ====================
        /// Override excitation status feedback
        parameter<bool> override_excitation_status{false};
        parameter<bool> override_excitation_status_value{false};
    };

    sfc_machine_io_mock_runner()
    {
        yeet::wire(_inputs)
            .mocked(_storage.excitation_on_command)
            .to(&inputs::excitation_on_command)
            .mocked(_storage.excitation_off_command)
            .to(&inputs::excitation_off_command);
    }

    void register_datamodel(yeet::model_registry& registry);
    void initialize();
    void run();
    void stop();

    inputs& get_inputs() { return _inputs; }
    const inputs& get_inputs() const { return _inputs; }
    outputs& get_outputs() { return _outputs; }
    const outputs& get_outputs() const { return _outputs; }
    parameters& get_parameters() { return _parameters; }
    const parameters& get_parameters() const { return _parameters; }

private:
    inputs _inputs;
    outputs _outputs;
    parameters _parameters;
    mock_storage _storage{};

    // Internal state for modeling hardware dynamics
    duration_t _excitation_transition_timer{0.0};
    bool _excitation_running{false};  // Actual excitation system state

    // Last commanded states (for edge detection)
    bool _last_excitation_on_cmd{false};
    bool _last_excitation_off_cmd{false};

    // Sampling time for timer updates (assume 1ms)
    static constexpr duration_t kSamplingTime{1e-3};

    // State update methods
    void update_excitation_dynamics();
    void apply_overrides_and_write_to_external_ios();
};

}  // namespace yeet::application::sfc::integration
