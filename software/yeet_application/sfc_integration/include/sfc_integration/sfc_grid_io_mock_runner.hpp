#pragma once

#include "sfc_integration/sfc_grid_side_external_ios_datamodel.hpp"
#include "yeet/model_registry.hpp"
#include "yeet/parameter.hpp"
#include "yeet/utils/parameter_accessors.hpp"
#include "yeet_control/units.hpp"

namespace yeet::application::sfc::integration {

/**
 * @brief Grid-side IO mock runner - models external hardware behavior
 *
 * Simulates the physical hardware connected to the grid-side control system:
 * - MV breaker: Models close/open delays and position feedback
 * - Auxiliary contactors: Models energization delays
 * - ACU fans: Models spin-up delays
 * - External inputs: Start/stop commands, fault signals from supervisory system
 *
 * **Architecture:**
 * - Reads control commands from external_ios digital outputs (get_mv_breaker_close, etc.)
 * - Models hardware dynamics (delays, interlocks)
 * - Writes hardware status back to external_ios digital inputs (set_digital_inputs)
 *
 * **Testing Features:**
 * - Configurable timing parameters for realistic behavior
 * - Override capability for fault injection testing
 * - Clear separation between commanded state and actual hardware state
 */
class sfc_grid_io_mock_runner {
public:
    struct inputs {
        // Commands from control system (external_ios digital outputs)
        signal_input<bool> mv_breaker_close{mock_signal_input::create_mocked<bool>()};
        signal_input<bool> mv_breaker_open{mock_signal_input::create_mocked<bool>()};
        signal_input<bool> auxiliaries_on_command{mock_signal_input::create_mocked<bool>()};
        signal_input<bool> acu_fan_1_on_command{mock_signal_input::create_mocked<bool>()};
        signal_input<bool> acu_fan_2_on_command{mock_signal_input::create_mocked<bool>()};
        signal_input<bool> acu_fan_3_on_command{mock_signal_input::create_mocked<bool>()};
        signal_input<bool> acu_fan_4_on_command{mock_signal_input::create_mocked<bool>()};
    };

    struct outputs {
        // Hardware status feedback (external_ios digital inputs)
        signal_output<bool> auxiliaries_are_on;
        signal_output<bool> mv_breaker_is_closed;
        signal_output<bool> start_command;
        signal_output<bool> stop_command;
        signal_output<bool> shutdown_request;
        signal_output<bool> mv_breaker_is_tripping;
        signal_output<bool> emergency_off;
        signal_output<bool> door_is_open;
        signal_output<bool> external_fault;
        signal_output<bool> fans_are_running;
    };

    struct parameters {
        // ==================== TIMING PARAMETERS ====================
        /// Time for MV breaker to close after close command
        parameter<duration_t> breaker_close_delay{duration_t{50e-3}};  // 50ms

        /// Time for MV breaker to open after open command
        parameter<duration_t> breaker_open_delay{duration_t{20e-3}};  // 20ms

        /// Time for auxiliary contactors to energize
        parameter<duration_t> auxiliary_energize_delay{duration_t{100e-3}};  // 100ms

        /// Time for ACU fans to spin up and report running
        parameter<duration_t> fan_spinup_delay{duration_t{2.0}};  // 2s

        // ==================== AVAILABILITY FLAGS ====================
        /// AC power available for auxiliaries
        parameter<bool> auxiliaries_power_available{true};

        /// Breaker mechanism is healthy (not jammed)
        parameter<bool> breaker_mechanism_healthy{true};

        // ==================== EXTERNAL SUPERVISORY INPUTS ====================
        /// Start command from HMI/SCADA
        parameter<bool> start_command{false};

        /// Stop command from HMI/SCADA
        parameter<bool> stop_command{false};

        /// Emergency shutdown request from supervisory system
        parameter<bool> shutdown_request{false};

        /// Emergency off button pressed
        parameter<bool> emergency_off{false};

        /// Cabinet door is open
        parameter<bool> door_is_open{false};

        /// External fault from protection relay
        parameter<bool> external_fault{false};

        /// MV breaker is tripping (from protection relay)
        parameter<bool> mv_breaker_is_tripping{false};

        // ==================== OUTPUT OVERRIDES (Fault Injection) ====================
        /// Override MV breaker position feedback
        parameter<bool> override_mv_breaker_is_closed{false};
        parameter<bool> override_mv_breaker_is_closed_value{false};

        /// Override auxiliaries status feedback
        parameter<bool> override_auxiliaries_are_on{false};
        parameter<bool> override_auxiliaries_are_on_value{false};

        /// Override fan running status feedback
        parameter<bool> override_fans_are_running{false};
        parameter<bool> override_fans_are_running_value{false};
    };

    sfc_grid_io_mock_runner() = default;

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

    // Internal state for modeling hardware dynamics
    duration_t _breaker_transition_timer{0.0};
    duration_t _auxiliary_energize_timer{0.0};
    duration_t _fan_spinup_timer{0.0};

    bool _breaker_position{false};  // Actual physical breaker position
    bool _auxiliary_status{false};  // Actual contactor state
    bool _fans_running{false};      // Actual fan running state

    // Last commanded states (for edge detection)
    bool _last_breaker_close_cmd{false};
    bool _last_breaker_open_cmd{false};
    bool _last_auxiliary_cmd{false};
    bool _last_fan_cmd{false};

    // Sampling time for timer updates (assume 1ms)
    static constexpr duration_t kSamplingTime{1e-3};

    // State update methods
    void update_breaker_dynamics();
    void update_auxiliary_contactors();
    void update_fan_status();
    void apply_overrides_and_write_to_external_ios();
};

}  // namespace yeet::application::sfc::integration
