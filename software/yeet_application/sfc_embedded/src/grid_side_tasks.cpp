#include "sfc_embedded/grid_side_tasks.hpp"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>

#include "sfc/sfc_types.hpp"
#include "sfc_embedded/fiber_optic_protocol.hpp"
#include "sfc_embedded/grid_side_network_telemetry_builder.hpp"
#include "sfc_integration/sfc_grid_side_control.hpp"
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

#define RECTIFIER_MODE 1

extern "C" {

inline std::shared_ptr<yeet::application::sfc::sfc_per_unit_system> create_per_unit_system()
{
    // Standard per-unit system for SFC applications
    // Grid side: 2090V RMS, 50Hz, DC link: 1044A, Machine: 1980V RMS, 2950kW
    return std::make_shared<yeet::application::sfc::sfc_per_unit_system>(
        yeet::voltage_t(2090.0),  // Grid line-to-line voltage (V RMS)
        yeet::frequency_t(50.0),  // Grid frequency (Hz)
        yeet::current_t(1044.0),  // DC link current base (A)
        yeet::power_t(2950e3),    // Rated power (W)
        yeet::voltage_t(1980.0),  // Machine line-to-line voltage (V RMS)
        yeet::frequency_t(50.0),  // Machine base frequency (Hz)
        2                         // Number of pole pairs
    );
}

static inline yeet::application::sfc::integration::sfc_grid_side_control& get_grid_side_control()
{
    static auto per_unit_system = create_per_unit_system();
    static yeet::application::sfc::integration::sfc_grid_side_control grid_side_control(per_unit_system);
    return grid_side_control;
}

// ==================== MODEL REGISTRY ====================
// Global parameter registry populated from all SFC component datamodels
static inline yeet::model_registry& get_model_registry()
{
    static yeet::model_registry registry;
    static bool initialized = false;

    if (!initialized) {
        // Register all grid-side component parameters with the model_registry
        // This delegates to the control class which knows about all its components
        get_grid_side_control().register_datamodel(registry);
        initialized = true;
    }
    return registry;
}

// ==================== GRID-SIDE TELEMETRY BUILDER ====================
// Builder for grid-side telemetry packets
static inline yeet::application::sfc::telemetry::grid_side_telemetry_builder& get_grid_side_telemetry_builder()
{
    static yeet::application::sfc::telemetry::grid_side_telemetry_builder builder(get_grid_side_control());
    return builder;
}

// ==================== NETWORK TELEMETRY MANAGER ====================
// Telemetry streaming manager for TCP/IP interface
static inline yeet::telemetry::telemetry_manager& get_telemetry_manager()
{
    static yeet::telemetry::telemetry_manager manager(get_grid_side_telemetry_builder(), 10);
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
void yeet_embedded_init(void)
{
    // ==================== INITIALIZE PARAMETER HANDLER ====================
    // Force initialization of model registry and parameter handler on startup
    // This ensures parameters are registered before first network request
    (void)get_parameter_handler();

    // ==================== INITIALIZE DIGITAL INPUTS ====================
    // Set safe default values for supervisory controller inputs
    yeet::mock_signal_input::write(get_grid_side_control().get_io().digital_inputs.auxiliaries_are_on, false);
    yeet::mock_signal_input::write(get_grid_side_control().get_io().digital_inputs.breakers_are_on, false);
    yeet::mock_signal_input::write(get_grid_side_control().get_io().digital_inputs.start_command, false);
    yeet::mock_signal_input::write(get_grid_side_control().get_io().digital_inputs.stop_command, false);
    yeet::mock_signal_input::write(get_grid_side_control().get_io().digital_inputs.shutdown_request, false);

    // ==================== INITIALIZE ANALOG INPUTS ====================
    // Initialize ADC inputs with zero values (will be updated from hardware in 200µs task)
    yeet::mock_signal_input::write(
        get_grid_side_control().get_io().analog_inputs.v_ab_counts, yeet::control::adc_counts{0});
    yeet::mock_signal_input::write(
        get_grid_side_control().get_io().analog_inputs.v_bc_counts, yeet::control::adc_counts{0});
    yeet::mock_signal_input::write(
        get_grid_side_control().get_io().analog_inputs.i_a_counts, yeet::control::adc_counts{0});
    yeet::mock_signal_input::write(
        get_grid_side_control().get_io().analog_inputs.i_c_counts, yeet::control::adc_counts{0});

    // ==================== INITIALIZE MACHINE-SIDE INPUTS ====================
    // Initialize cross-coupled inputs from machine-side controller with safe default values
    // In production, these come from fiber-optic communication interface
    yeet::mock_signal_input::write(
        get_grid_side_control().get_io().machine_side_inputs.measured_speed_pu, yeet::speed_pu_t{0.0});
    yeet::mock_signal_input::write(
        get_grid_side_control().get_io().machine_side_inputs.dc_link_current_reference, yeet::current_pu_t{0.0});
    yeet::mock_signal_input::write(
        get_grid_side_control().get_io().machine_side_inputs.machine_voltage_magnitude, yeet::voltage_pu_t{0.0});
    yeet::mock_signal_input::write(
        get_grid_side_control().get_io().machine_side_inputs.request_force_commutation, false);

    // ==================== INITIALIZE INTERNAL CONTROLLER STATES ====================
    // Ensure deterministic start: explicitly force all freeze inputs false prior to state machine activity
    yeet::mock_signal_input::write(get_grid_side_control().current_controller.get_inputs().freeze_integrator, false);
    yeet::mock_signal_input::write(
        get_grid_side_control().current_controller.get_inputs().modulator_freeze_integrator, false);

    // Force initial conditions...
    get_grid_side_control().grid_side_observer.get_parameters().initial_frequency.write(yeet::frequency_pu_t{0.0f});

    // Initialize grid-side control subsystem
    // This will also connect internal signals (observer → measurements for angle/frequency/pulses)
    get_grid_side_control().initialize();

    // Parametrize the software
    get_grid_side_control().grid_measurements.get_parameters().voltage_transducer_gain.write(
        yeet::real_t{1.0f / 300.0f});
    get_grid_side_control().grid_measurements.get_parameters().current_transducer_gain.write(
        yeet::real_t{1.0f / 120.0f});

    get_grid_side_control().grid_side_pulse_generator.get_parameters().reference_type.write(
        yeet::control::six_pulse_generator::reference_type::line);
    get_grid_side_control().grid_side_pulse_generator.get_parameters().pulse_width.write(
        yeet::angle_wrapped::from_radians(yeet::angle_t{yeet::pi / 3.0f}));

    // ==================== INITIALIZE PARAMETER SYSTEM ====================
    // Force initialization of model registry and parameter handler
    // This ensures all component parameters are registered before network requests arrive
    get_model_registry();     // Trigger lazy initialization of parameter registry
    get_parameter_handler();  // Initialize parameter protocol handler
}

ARM_FUNCTION
void yeet_task_200us(const task_inputs_t* inputs, task_outputs_t* outputs)
{
    // HIGH-FREQUENCY CONTROL TASK
    // Configured rate: 200 µs (5 kHz)
    // Execution time budget: < 100 µs (50% of period)
    // Called from: TIM2_IRQHandler
    // Priority: Highest (innermost current control loop)

    // ==================== ANALOG INPUTS → MEASUREMENTS ====================
    // Map hardware ADC channels to grid-side measurements via I/O interface
    // Based on sfc_io_definitions.md:
    //   AI1 (X1-1): Grid voltage Vab (100V → 3.5V, ±200V range)
    //   AI2 (X1-2): Grid voltage Vbc
    //   AI5 (X1-5): Grid current Ia (1A → 5V or 5A → 5V, ±7.5A range)
    //   AI7 (X1-7): Grid current Ic

    // Write ADC values directly to I/O interface analog inputs
    yeet::mock_signal_input::write(get_grid_side_control().get_io().analog_inputs.v_ab_counts,
        yeet::control::adc_counts{
            static_cast<int16_t>(inputs->analog_inputs.channels[1])});  // On hil this is channel 1
    yeet::mock_signal_input::write(get_grid_side_control().get_io().analog_inputs.v_bc_counts,
        yeet::control::adc_counts{static_cast<int16_t>(inputs->analog_inputs.channels[2])});  // On hil this is 2
    yeet::mock_signal_input::write(get_grid_side_control().get_io().analog_inputs.i_a_counts,
        yeet::control::adc_counts{static_cast<int16_t>(inputs->analog_inputs.channels[5])});
    yeet::mock_signal_input::write(get_grid_side_control().get_io().analog_inputs.i_c_counts,
        yeet::control::adc_counts{static_cast<int16_t>(inputs->analog_inputs.channels[7])});

    // Execute the 5 kHz control update (measurements → observers → modulators → pulse generator)
    get_grid_side_control().update_200us();

    // ==================== ANALOG OUTPUTS (DAC) → MONITORING/TESTING ====================
    // Map control system signals to DAC outputs for monitoring and testing
    // The AD5724 provides 4 analog output channels (±5V or ±10V range)
    if (outputs != nullptr) {
        // Example DAC output mappings (customize based on your application):
        // Channel A: Test signal 1
        // Channel B: Test signal 2
        // Channel C: Test signal 3
        // Channel D: Test signal 4

        // Baseline constant output test: drive all four channels to +1.0 V (±5 V range)
        // Purpose: Hardware validation with scope / DMM after restoring legacy driver.
        outputs->analog_outputs.channels[0] =
            get_grid_side_control().grid_side_observer.get_outputs().estimated_position.read().get_pu().value();  // CH
                                                                                                                  // A
        outputs->analog_outputs.channels[1] =
            get_grid_side_control().grid_measurements.get_outputs().grid_voltage_abc.read().a();  // CH B
        outputs->analog_outputs.channels[2] =
            get_grid_side_control().grid_measurements.get_outputs().grid_voltage_abc_unmasked.read().a();  // CH C
        outputs->analog_outputs.channels[3] = get_grid_side_control()
                                                  .grid_measurements.get_outputs()
                                                  .grid_voltage_alphabeta_filtered.read()
                                                  .alpha();  // CH D
        outputs->analog_outputs.valid = true;                // Signal to writer task that values are valid
    }

    // ==================== THYRISTOR PULSES → PWM OUTPUTS ====================
    // Extract firing pulses from grid-side I/O interface and map to hardware PWM channels
    // Based on sfc_io_definitions.md PWM mapping:
    //   PWM1 (PC6) → a_plus  (Phase A positive arm, T1)
    //   PWM2 (PC7) → c_minus (Phase C negative arm, T6)
    //   PWM3 (PC8) → b_plus  (Phase B positive arm, T3)
    //   PWM4 (PC9) → a_minus (Phase A negative arm, T4)
    //   PWM5 (PB0) → c_plus  (Phase C positive arm, T5)
    //   PWM6 (PB1) → b_minus (Phase B negative arm, T2)
    if (outputs != nullptr) {
        const auto& firing_pulses = get_grid_side_control().get_io().pulse_outputs.firing_pulses.read();

        outputs->thyristor_pulses.a_plus = firing_pulses.p1;
        outputs->thyristor_pulses.b_minus = firing_pulses.p6;
        outputs->thyristor_pulses.b_plus = firing_pulses.p3;
        outputs->thyristor_pulses.a_minus = firing_pulses.p4;
        outputs->thyristor_pulses.c_plus = firing_pulses.p5;
        outputs->thyristor_pulses.c_minus = firing_pulses.p2;
    }
}

ARM_FUNCTION
void yeet_task_1000us(const task_inputs_t* inputs, task_outputs_t* outputs)
{
    // MEDIUM-FREQUENCY CONTROL TASK
    // Configured rate: 1 ms (1 kHz)
    // Execution time budget: < 500 µs (50% of period)
    // Called from: TIM3_IRQHandler
    // Priority: Medium (outer control loops)

    // ==================== DIGITAL INPUTS → CONTROL COMMANDS ====================
    // Map hardware digital inputs to state machine control signals via I/O interface
    // Based on typical SFC system supervisory control interface:
    //   DI1-DI6: State machine commands (auxiliaries, breakers, start, stop, etc.)
    //   DI7-DI16: System status and interlocks
    if (inputs != nullptr && inputs->digital_inputs.valid) {
        // State machine command inputs (active-high logic)
        bool auxiliaries_on = inputs->digital_inputs.signals[0];        // DI1: Auxiliary systems ready
        bool breakers_on = inputs->digital_inputs.signals[1];           // DI2: Grid breakers closed
        bool start_cmd = inputs->digital_inputs.signals[2];             // DI3: Start command
        bool stop_cmd = inputs->digital_inputs.signals[3];              // DI4: Stop command
        bool shutdown_req = inputs->digital_inputs.signals[4];          // DI5: Emergency shutdown
        bool current_extinguished = inputs->digital_inputs.signals[5];  // DI6: DC link current is zero

#ifndef RECTIFIER_MODE
        // Write digital input values directly to I/O interface
        yeet::mock_signal_input::write(
            get_grid_side_control().get_io().digital_inputs.auxiliaries_are_on, auxiliaries_on);
        yeet::mock_signal_input::write(get_grid_side_control().get_io().digital_inputs.breakers_are_on, breakers_on);
        yeet::mock_signal_input::write(get_grid_side_control().get_io().digital_inputs.start_command, start_cmd);
        yeet::mock_signal_input::write(get_grid_side_control().get_io().digital_inputs.stop_command, stop_cmd);
        yeet::mock_signal_input::write(get_grid_side_control().get_io().digital_inputs.shutdown_request, shutdown_req);
        yeet::mock_signal_input::write(
            get_grid_side_control().get_io().digital_inputs.is_current_extinguished, current_extinguished);
#else
        yeet::mock_signal_input::write(get_grid_side_control().get_io().digital_inputs.auxiliaries_are_on, true);
        yeet::mock_signal_input::write(get_grid_side_control().get_io().digital_inputs.breakers_are_on, true);
        yeet::mock_signal_input::write(get_grid_side_control().get_io().digital_inputs.start_command, true);
        yeet::mock_signal_input::write(get_grid_side_control().get_io().digital_inputs.stop_command, false);
        yeet::mock_signal_input::write(get_grid_side_control().get_io().digital_inputs.shutdown_request, false);
#endif
    }

    // Process fiber-optic RX data from machine-side controller
    // Fiber-optic link provides cross-coupled signals between grid and machine sides
#ifndef RECTIFIER_MODE
    if (inputs != nullptr && inputs->fiber_optic_rx.data_ready && inputs->fiber_optic_rx.link_ok) {
        // Decode Machine → Grid protocol message
        if (auto rx_msg = yeet::application::sfc::fiber_optic::decode_machine_to_grid(
                inputs->fiber_optic_rx.payload, inputs->fiber_optic_rx.payload_length)) {
            // Write machine-side cross-coupled signals directly to I/O interface
            yeet::mock_signal_input::write(get_grid_side_control().get_io().machine_side_inputs.measured_speed_pu,
                yeet::speed_pu_t{rx_msg->measured_speed_pu});
            yeet::mock_signal_input::write(
                get_grid_side_control().get_io().machine_side_inputs.dc_link_current_reference,
                yeet::current_pu_t{rx_msg->dc_link_current_reference_pu});
            yeet::mock_signal_input::write(
                get_grid_side_control().get_io().machine_side_inputs.machine_voltage_magnitude,
                yeet::voltage_pu_t{rx_msg->machine_voltage_magnitude_pu});
            yeet::mock_signal_input::write(
                get_grid_side_control().get_io().machine_side_inputs.request_force_commutation,
                rx_msg->force_commutation_request);
        }
    }
#else
    yeet::mock_signal_input::write(
        get_grid_side_control().get_io().machine_side_inputs.measured_speed_pu, yeet::speed_pu_t{1.0f});
    yeet::mock_signal_input::write(
        get_grid_side_control().get_io().machine_side_inputs.dc_link_current_reference, yeet::current_pu_t{0.0f});
    yeet::mock_signal_input::write(
        get_grid_side_control().get_io().machine_side_inputs.machine_voltage_magnitude, yeet::voltage_pu_t{1.0f});
    yeet::mock_signal_input::write(
        get_grid_side_control().get_io().machine_side_inputs.request_force_commutation, false);
#endif

    // ==================== EXECUTE CONTROL UPDATE ====================
    // Digital input overrides are automatically applied within the state machine lambdas
    // No separate apply_digital_input_overrides() call needed - the lambdas check
    // override parameters directly when state machine reads inputs
    // See sfc_state_machine_runner constructor for override logic

    // Execute 1 kHz control update (current controller)
    get_grid_side_control().update_1ms();

    // ==================== FIBER-OPTIC TX → MACHINE-SIDE CONTROLLER ====================
    // Prepare fiber-optic TX data to send to machine-side board via I/O interface outputs
    // Transmit grid-side status and cross-coupled signals
    if (outputs != nullptr) {
        // Prepare Grid → Machine protocol message
        yeet::application::sfc::fiber_optic::grid_to_machine_message tx_msg;

        // Read from I/O interface machine-side outputs
        const auto operating_mode = get_grid_side_control().get_io().machine_side_outputs.operating_mode.read();

        // Map SFC operating mode to fiber-optic protocol enum
        if (operating_mode == yeet::application::sfc::sfc_operating_mode_t::forced_commutation) {
            tx_msg.mode = yeet::application::sfc::fiber_optic::operating_mode::forced_commutation;
        }
        else if (operating_mode == yeet::application::sfc::sfc_operating_mode_t::natural_commutation) {
            tx_msg.mode = yeet::application::sfc::fiber_optic::operating_mode::natural_commutation;
        }
        else {
            tx_msg.mode = yeet::application::sfc::fiber_optic::operating_mode::forced_commutation;  // Safe default
        }

        tx_msg.is_current_zero = get_grid_side_control().get_io().machine_side_outputs.is_current_zero.read();
        tx_msg.force_commutation_acknowledged =
            get_grid_side_control().get_io().machine_side_outputs.force_commutation_acknowledged.read();
        tx_msg.pulse_releases = get_grid_side_control().get_io().digital_outputs.pulse_releases.read();

        // Encode and transmit
        outputs->fiber_optic_tx.payload_length =
            yeet::application::sfc::fiber_optic::encode_grid_to_machine(tx_msg, outputs->fiber_optic_tx.payload);
        outputs->fiber_optic_tx.transmit_request = true;
    }

    // ==================== DIGITAL OUTPUTS → STATUS INDICATORS ====================
    // Map control system status to hardware digital outputs via I/O interface
    // Based on typical HMI and system monitoring requirements
    if (outputs != nullptr) {
        // Read status signals from I/O interface digital outputs
        const bool pulse_releases = get_grid_side_control().get_io().digital_outputs.pulse_releases.read();
        const bool is_current_zero = get_grid_side_control().get_io().digital_outputs.is_current_zero.read();
        const bool current_saturated =
            get_grid_side_control().get_io().digital_outputs.current_controller_saturated.read();
        const auto operating_mode = get_grid_side_control().get_io().digital_outputs.operating_mode.read();

        // Status output mapping:
        // outputs->digital_outputs.signals[0] = pulse_releases;     // DO1: System running
        // outputs->digital_outputs.signals[1] = is_current_zero;    // DO2: Current zero status
        // outputs->digital_outputs.signals[2] = current_saturated;  // DO3: Controller saturation
        // outputs->digital_outputs.signals[3] =
        //     (operating_mode ==
        //         yeet::application::sfc::sfc_operating_mode_t::forced_commutation);  // DO4: Forced commutation mode
        // outputs->digital_outputs.signals[4] =
        //     (operating_mode ==
        //         yeet::application::sfc::sfc_operating_mode_t::natural_commutation);  // DO5: Natural commutation mode
        // outputs->digital_outputs.signals[5] = inputs->fiber_optic_rx.link_ok;        // DO6: Fiber link status

        for (auto& n : outputs->digital_outputs.signals) {
            n = true;
        }

        // outputs->digital_outputs.signals[0] = true;  // DO1: System running
        // outputs->digital_outputs.signals[1] = true;  // DO2: Current zero status
        // outputs->digital_outputs.signals[2] = true;  // DO3: Controller saturation
        // outputs->digital_outputs.signals[3] = true;  // DO4: Forced commutation mode
        // outputs->digital_outputs.signals[4] = true;
        // outputs->digital_outputs.signals[5] = true;  // DO6: Fiber link status

        // Additional outputs (DO7-DO17) available for application-specific status
    }
}

ARM_FUNCTION
void yeet_task_10000us(const task_inputs_t* inputs, task_outputs_t* outputs)
{
    // LOW-FREQUENCY MONITORING TASK
    // Configured rate: 10 ms (100 Hz)
    // Execution time budget: < 5 ms (50% of period)
    // Called from: TIM4_IRQHandler
    // Priority: Lowest (monitoring and diagnostics)

    // Optional: Use inputs for monitoring/diagnostics
    (void)inputs;  // Suppress unused parameter warning

    get_grid_side_control().update_10ms();

    // Optional: Update diagnostic outputs
    (void)outputs;  // Suppress unused parameter warning
}

ARM_FUNCTION
void yeet_network(const network_inputs_t* inputs, network_outputs_t* outputs)
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
