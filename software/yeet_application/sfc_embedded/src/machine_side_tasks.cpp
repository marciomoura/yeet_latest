#include "sfc_embedded/machine_side_tasks.hpp"

#include <cstdint>
#include <cstring>
#include <memory>

#include "sfc/sfc_types.hpp"
#include "sfc_embedded/fiber_optic_protocol.hpp"
#include "sfc_embedded/machine_side_network_telemetry_builder.hpp"
#include "sfc_integration/sfc_machine_side_control.hpp"
#include "yeet/network_handler.hpp"
#include "yeet/network_telemetry_manager.hpp"
#include "yeet/parameter_handler.hpp"
#include "yeet/signal.hpp"

// Ensure functions are compiled with Thumb interworking
#ifdef __arm__
#define ARM_FUNCTION __attribute__((target("thumb")))
#else
#define ARM_FUNCTION
#endif

extern "C" {

static inline yeet::application::sfc::integration::sfc_machine_side_control& get_machine_side_control()
{
    static yeet::application::sfc::integration::sfc_machine_side_control machine_side_control;
    return machine_side_control;
}

// ==================== MODEL REGISTRY ====================
// Global parameter registry populated from all SFC component datamodels
static inline yeet::model_registry& get_model_registry()
{
    static yeet::model_registry registry;
    static bool initialized = false;

    if (!initialized) {
        // Register all machine-side component parameters with the model_registry
        // This delegates to the control class which knows about all its components
        get_machine_side_control().register_datamodel(registry);
        initialized = true;
    }
    return registry;
}

// ==================== MACHINE-SIDE TELEMETRY BUILDER ====================
// Builder for machine-side telemetry packets
static inline yeet::application::sfc::telemetry::machine_side_telemetry_builder& get_machine_side_telemetry_builder()
{
    static yeet::application::sfc::telemetry::machine_side_telemetry_builder builder(get_machine_side_control());
    return builder;
}

// ==================== NETWORK TELEMETRY MANAGER ====================
// Telemetry streaming manager for TCP/IP interface
static inline yeet::telemetry::telemetry_manager& get_telemetry_manager()
{
    static yeet::telemetry::telemetry_manager manager(get_machine_side_telemetry_builder(), 10);
    return manager;
}

// ==================== PARAMETER HANDLER ====================
// Parameter configuration handler for TCP/IP interface
static inline yeet::telemetry::parameter_handler& get_parameter_handler()
{
    static yeet::telemetry::parameter_handler handler(&get_model_registry());
    return handler;
}

// ==================== NETWORK HANDLER ====================
// Network protocol handler that manages telemetry streaming and parameter requests
static inline yeet::telemetry::network_handler& get_network_handler()
{
    static yeet::telemetry::network_handler handler(get_telemetry_manager(), get_parameter_handler());
    return handler;
}

ARM_FUNCTION
void machine_embedded_init(yeet::application::sfc::integration::sfc_machine_side_control* control)
{
    // ==================== INITIALIZE PARAMETER HANDLER ====================
    // Force initialization of model registry and parameter handler on startup
    // This ensures parameters are registered before first network request
    (void)get_parameter_handler();

    // ==================== INITIALIZE DIGITAL INPUTS ====================
    // Set safe default values for supervisory controller inputs
    // yeet::mock_signal_input::write(control->get_io().digital_inputs.exciter_ready, false);
    // yeet::mock_signal_input::write(control->get_io().digital_inputs.excitation_start_command,
    // false);
    // yeet::mock_signal_input::write(control->get_io().digital_inputs.excitation_stop_command,
    // false); yeet::mock_signal_input::write(control->get_io().digital_inputs.run_creep_speed,
    // false); yeet::mock_signal_input::write(control->get_io().digital_inputs.run_purge_accel,
    // false); yeet::mock_signal_input::write(control->get_io().digital_inputs.run_ignition_accel,
    // false); yeet::mock_signal_input::write(control->get_io().digital_inputs.run_assist_accel,
    // false); yeet::mock_signal_input::write(control->get_io().digital_inputs.hold_speed, false);
    // yeet::mock_signal_input::write(control->get_io().digital_inputs.decelerate, false);
    // yeet::mock_signal_input::write(control->get_io().digital_inputs.idle, true);

    // ==================== INITIALIZE GRID-SIDE INPUTS ====================
    // Initialize cross-coupled inputs from grid-side controller with safe default values
    // In production, these come from fiber-optic communication interface
    // yeet::mock_signal_input::write(
    //     control->get_io().grid_side_inputs.dc_link_current_measured, yeet::current_pu_t{0.0});
    // yeet::mock_signal_input::write(control->get_io().grid_side_inputs.operating_mode,
    //     yeet::application::sfc::sfc_operating_mode_t::natural_commutation);
    // yeet::mock_signal_input::write(
    //     control->get_io().grid_side_inputs.force_commutation_acknowledged, false);
    // yeet::mock_signal_input::write(control->get_io().grid_side_inputs.freeze_integrator, false);
    // yeet::mock_signal_input::write(
    //     control->get_io().grid_side_inputs.current_controller_saturated, false);
    // yeet::mock_signal_input::write(control->get_io().grid_side_inputs.pulse_releases, false);
    // yeet::mock_signal_input::write(control->get_io().grid_side_inputs.current_state_id,
    //     yeet::application::sfc::sfc_state_id::IDLE);

    // Initialize machine-side control subsystem
    control->initialize();
}

ARM_FUNCTION
void machine_task_200us(yeet::application::sfc::integration::sfc_machine_side_control* control,
    const task_inputs_t* inputs,
    task_outputs_t* outputs)
{
    // HIGH-FREQUENCY CONTROL TASK
    // Configured rate: 200 µs (5 kHz)
    // Execution time budget: < 100 µs (50% of period)
    // Called from: TIM2_IRQHandler
    // Priority: Highest (measurements, observer, modulator, pulse generator)

    // ==================== ANALOG INPUTS → MEASUREMENTS ====================
    // Map hardware ADC channels to machine-side measurements via I/O interface
    // Based on sfc_io_definitions.md:
    //   AI1 (X1-1): Machine voltage Vab
    //   AI2 (X1-2): Machine voltage Vbc

    // Write ADC values directly to I/O interface analog inputs
    if (inputs != nullptr && inputs->analog_inputs.valid) {
        control->external_ios.update_analog_inputs(
            yeet::control::adc_counts{static_cast<std::int32_t>(inputs->analog_inputs.channels[1])},
            yeet::control::adc_counts{static_cast<std::int32_t>(inputs->analog_inputs.channels[2])});
    }

    // Execute the 5 kHz control update (measurements → observers → modulators → pulse generator)
    control->update_200us();

    // ==================== THYRISTOR PULSES → PWM OUTPUTS ====================
    // Extract firing pulses from machine-side I/O interface and map to hardware PWM channels
    if (outputs != nullptr) {
        const auto firing_pulses = control->external_ios.get_thyristor_pulse_outputs();

        outputs->thyristor_pulses.a_plus = firing_pulses.p1;   // T1 (A+) → p1
        outputs->thyristor_pulses.b_minus = firing_pulses.p2;  // T2 (B-) → p2
        outputs->thyristor_pulses.b_plus = firing_pulses.p3;   // T3 (B+) → p3
        outputs->thyristor_pulses.a_minus = firing_pulses.p4;  // T4 (A-) → p4
        outputs->thyristor_pulses.c_plus = firing_pulses.p5;   // T5 (C+) → p5
        outputs->thyristor_pulses.c_minus = firing_pulses.p6;  // T6 (C-) → p6
    }
}

ARM_FUNCTION
void machine_task_1000us(yeet::application::sfc::integration::sfc_machine_side_control* control,
    const task_inputs_t* inputs,
    task_outputs_t* outputs)
{
    // MEDIUM-FREQUENCY CONTROL TASK
    // Configured rate: 1 ms (1 kHz)
    // Execution time budget: < 500 µs (50% of period)
    // Called from: TIM3_IRQHandler
    // Priority: Medium (machine controller)

    // ==================== DIGITAL INPUTS → CONTROL COMMANDS ====================
    // Map hardware digital inputs to state machine control signals via I/O interface
    if (inputs != nullptr && inputs->digital_inputs.valid) {
        // Excitation control inputs
        bool exciter_ready = inputs->digital_inputs.signals[0];     // DI1: Exciter ready
        bool excitation_start = inputs->digital_inputs.signals[1];  // DI2: Start excitation
        bool excitation_stop = inputs->digital_inputs.signals[2];   // DI3: Stop excitation

        // Speed profile selection inputs (turbine starting sequence)
        bool run_creep = inputs->digital_inputs.signals[3];       // DI4: Run creep speed profile
        bool run_purge = inputs->digital_inputs.signals[4];       // DI5: Run purge acceleration
        bool run_ignition = inputs->digital_inputs.signals[5];    // DI6: Run ignition acceleration
        bool run_assist = inputs->digital_inputs.signals[6];      // DI7: Run assist acceleration
        bool hold_speed_cmd = inputs->digital_inputs.signals[7];  // DI8: Hold current speed
        bool decelerate_cmd = inputs->digital_inputs.signals[8];  // DI9: Decelerate to zero
        bool idle_cmd = inputs->digital_inputs.signals[9];        // DI10: Idle mode

        // Write digital input values directly to I/O interface
        // yeet::mock_signal_input::write(control->get_io().digital_inputs.exciter_ready,
        // exciter_ready); yeet::mock_signal_input::write(
        //     control->get_io().digital_inputs.excitation_start_command, excitation_start);
        // yeet::mock_signal_input::write(
        //     control->get_io().digital_inputs.excitation_stop_command, excitation_stop);
        // yeet::mock_signal_input::write(control->get_io().digital_inputs.run_creep_speed,
        // run_creep);
        // yeet::mock_signal_input::write(control->get_io().digital_inputs.run_purge_accel,
        // run_purge); yeet::mock_signal_input::write(
        //     control->get_io().digital_inputs.run_ignition_accel, run_ignition);
        // yeet::mock_signal_input::write(control->get_io().digital_inputs.run_assist_accel,
        // run_assist); yeet::mock_signal_input::write(control->get_io().digital_inputs.hold_speed,
        // hold_speed_cmd);
        // yeet::mock_signal_input::write(control->get_io().digital_inputs.decelerate,
        // decelerate_cmd); yeet::mock_signal_input::write(control->get_io().digital_inputs.idle,
        // idle_cmd);
    }

    // Process fiber-optic RX data from grid-side controller
    // Fiber-optic link provides cross-coupled signals between grid and machine sides
    if (inputs != nullptr && inputs->fiber_optic_rx.data_ready && inputs->fiber_optic_rx.link_ok) {
        // Decode Grid → Machine protocol message
        if (auto rx_msg = yeet::application::sfc::fiber_optic::decode_grid_to_machine(
                inputs->fiber_optic_rx.payload, inputs->fiber_optic_rx.payload_length)) {
            // TODO...
            // control->external_ios.update_fiber_optics_inputs(rx_msg->mode,
            //     yeet::current_pu_t{rx_msg->dc_link_current_pu}, rx_msg->force_commutation_acknowledged,
            //     rx_msg->is_current_zero, rx_msg->current_controller_is_saturated, rx_msg->pulse_releases);
        }
    }

    // Execute 1 kHz control update (machine controller)
    control->update_1ms();

    // ==================== FIBER-OPTIC TX → GRID-SIDE CONTROLLER ====================
    // Prepare fiber-optic TX data to send to grid-side board via I/O interface outputs
    // Transmit machine-side status and cross-coupled signals
    if (outputs != nullptr) {
        // Prepare Machine → Grid protocol message
        yeet::application::sfc::fiber_optic::machine_to_grid_message tx_msg{
            .measured_speed_pu =
                static_cast<float>(control->get_machine_side_observer().get_outputs().estimated_speed.read()),
            .dc_link_current_reference_pu =
                static_cast<float>(control->get_torque_controller().get_outputs().current_reference.read()),
            .machine_voltage_magnitude_pu = static_cast<float>(
                control->get_machine_side_observer().get_outputs().estimated_voltage_magnitude.read()),
            .force_commutation_request =
                control->get_machine_side_modulator().get_outputs().request_force_commutation.read()};

        // Encode and transmit
        outputs->fiber_optic_tx.payload_length =
            yeet::application::sfc::fiber_optic::encode_machine_to_grid(tx_msg, outputs->fiber_optic_tx.payload);
        outputs->fiber_optic_tx.transmit_request = true;
    }

    // ==================== DIGITAL OUTPUTS → STATUS INDICATORS ====================
    // Map control system status to hardware digital outputs for HMI/supervisory system
    if (outputs != nullptr) {
        // Fiber link status from RX (direct pass-through)
        outputs->digital_outputs.signals[0] = inputs->fiber_optic_rx.link_ok;  // DO1: Fiber link status

        // Machine-side control status outputs
        // These could be used for HMI display, diagnostics, or interlocks
        // Populated from get_io().digital_inputs/outputs as needed
        // DO3-DO16: Reserved for future expansion (speed controller status, fault conditions, etc.)
    }
}

ARM_FUNCTION
void machine_task_10000us(yeet::application::sfc::integration::sfc_machine_side_control* control,
    const task_inputs_t* inputs,
    task_outputs_t* outputs)
{
    // LOW-FREQUENCY CONTROL TASK
    // Configured rate: 10 ms (100 Hz)
    // Execution time budget: < 5 ms (50% of period)
    // Called from: TIM4_IRQHandler
    // Priority: Lowest (speed controller, supervisory logic)
    // Components: Speed controller, excitation controller, speed generator

    // ==================== SPEED CONTROLLER EXECUTION ====================
    // Execute low-frequency speed regulation control
    control->update_10ms();

    // ==================== OPTIONAL: ANALOG OUTPUTS → MONITORING/TESTING ====================
    // Map control system signals to DAC outputs for monitoring and testing
    // The AD5724 provides 4 analog output channels (±5V or ±10V range)
    if (outputs != nullptr) {
        // Example DAC output mappings (customize based on your application):
        // Channel A: Speed error or speed setpoint
        // Channel B: Current reference
        // Channel C: Excitation voltage
        // Channel D: Reserved

        // Note: These are for diagnostics/monitoring only
        // The actual control signals are routed via get_io() interface
        // outputs->analog_outputs.channels[0] = ...  // CH A
        // outputs->analog_outputs.channels[1] = ...  // CH B
        // outputs->analog_outputs.channels[2] = ...  // CH C
        // outputs->analog_outputs.channels[3] = ...  // CH D
        // outputs->analog_outputs.valid = true;
    }

    (void)inputs;   // Suppress unused parameter warning if inputs not needed
    (void)outputs;  // Suppress unused parameter warning if outputs not needed
}

ARM_FUNCTION
void machine_network(const network_inputs_t* inputs, network_outputs_t* outputs)
{
    // NON-REAL-TIME NETWORK COMMUNICATION TASK
    // Called from: Main loop (not time-critical)
    // Purpose: Handle TCP/IP and UDP communication for:
    //   - HMI/SCADA supervisory control
    //   - Real-time telemetry streaming (measurements, observers, controllers)
    //   - Configuration updates
    //   - Diagnostic data requests
    //   - Parameter tuning
    //   - Event logging
    //
    // Delegate to network_handler for all packet processing and state management
    get_network_handler().process(inputs, outputs);
}

}  // extern "C"