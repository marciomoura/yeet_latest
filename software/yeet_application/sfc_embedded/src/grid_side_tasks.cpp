#include "sfc_embedded/grid_side_tasks.hpp"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>

#include "sfc/sfc_fiber_optic_protocol.hpp"
#include "sfc/sfc_types.hpp"
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

// ==================== Network Telemetry/Parameter Infrastructure ====================
// These are instantiated per control instance to support both embedded (static) and host (dynamic) usage

grid_side_network_infrastructure::grid_side_network_infrastructure(
    yeet::application::sfc::integration::sfc_grid_side_control& control)
    : telemetry_builder(control),
      telemetry_manager(telemetry_builder, 10),
      parameter_handler(&registry),
      network_handler(telemetry_manager, parameter_handler)
{
    // Step 1: Register datamodel with registry
    // This populates the registry with all control system parameters
    control.register_datamodel(registry);

    // Step 2: Rebuild parameter handler cache after datamodel registration
    // The parameter_handler was constructed with empty registry, so we need to rebuild the cache
    // now that the registry has been populated with actual parameters
    parameter_handler.rebuild_parameter_cache();
}
ARM_FUNCTION
void grid_embedded_init(yeet::application::sfc::integration::sfc_grid_side_control* control)
{
    // ==================== INITIALIZE DIGITAL INPUTS ====================
    // Set safe default values for supervisory controller inputs via external_ios datamodel
    control->get_external_ios().get_digital_inputs().auxiliaries_are_on.write(false);
    control->get_external_ios().get_digital_inputs().breakers_are_on.write(false);
    control->get_external_ios().get_digital_inputs().start_command.write(false);
    control->get_external_ios().get_digital_inputs().stop_command.write(false);
    control->get_external_ios().get_digital_inputs().shutdown_request.write(false);

    // Initialize grid-side control subsystem
    // This will also connect internal signals (observer → measurements for angle/frequency/pulses)
    control->grid_side_observer.get_parameters().initial_frequency.write(yeet::frequency_pu_t{0.0f});
    control->initialize();

    // Parametrize the software with hardware-specific gains (commented out - should be done via plant_hardware staging)
    // control->grid_measurements.get_parameters().voltage_transducer_gain.write(
    //     yeet::real_t{1.0f / 300.0f});
    // control->grid_measurements.get_parameters().current_transducer_gain.write(
    //     yeet::real_t{1.0f / 120.0f});
    // control->grid_side_pulse_generator.get_parameters().reference_type.write(
    //     yeet::control::six_pulse_generator::reference_type::line);
    // control->grid_side_pulse_generator.get_parameters().pulse_width.write(
    //     yeet::angle_wrapped::from_radians(yeet::angle_t{yeet::pi / 3.0f}));

    // IMPORTANT: For host simulation with plant model, measurement parameters are configured via
    // plant_hardware::stage_grid_side_measurement_parameters() in the plant builder BEFORE calling this init.
    // For production embedded builds, these parameters should be set here or via network configuration.
}

ARM_FUNCTION
void grid_task_200us(yeet::application::sfc::integration::sfc_grid_side_control* control,
    const task_inputs_t* inputs,
    task_outputs_t* outputs)
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

    // Write ADC values directly to external_ios interface analog inputs
    control->get_external_ios().get_analog_inputs().v_ab_counts.write(yeet::control::adc_counts{
        static_cast<int16_t>(inputs->analog_inputs.channels[1])});  // On hil this is channel 1
    control->get_external_ios().get_analog_inputs().v_bc_counts.write(
        yeet::control::adc_counts{static_cast<int16_t>(inputs->analog_inputs.channels[2])});  // On hil this is 2
    control->get_external_ios().get_analog_inputs().i_a_counts.write(
        yeet::control::adc_counts{static_cast<int16_t>(inputs->analog_inputs.channels[5])});
    control->get_external_ios().get_analog_inputs().i_c_counts.write(
        yeet::control::adc_counts{static_cast<int16_t>(inputs->analog_inputs.channels[7])});

    // Execute the 5 kHz control update (measurements → observers → modulators → pulse generator)
    control->update_200us();

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
            control->grid_side_observer.get_outputs().estimated_position.read().get_pu().value();  // CH A
        outputs->analog_outputs.channels[1] =
            control->grid_measurements.get_outputs().grid_voltage_abc.read().a();  // CH B
        outputs->analog_outputs.channels[2] =
            control->grid_measurements.get_outputs().grid_voltage_abc_unmasked.read().a();  // CH C
        outputs->analog_outputs.channels[3] =
            control->grid_measurements.get_outputs().grid_voltage_alphabeta_filtered.read().alpha();  // CH D
        outputs->analog_outputs.valid = true;  // Signal to writer task that values are valid
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
        const auto& firing_pulses = control->get_firing_pulses().read();

        outputs->thyristor_pulses.a_plus = firing_pulses.p1;
        outputs->thyristor_pulses.b_minus = firing_pulses.p6;
        outputs->thyristor_pulses.b_plus = firing_pulses.p3;
        outputs->thyristor_pulses.a_minus = firing_pulses.p4;
        outputs->thyristor_pulses.c_plus = firing_pulses.p5;
        outputs->thyristor_pulses.c_minus = firing_pulses.p2;
    }
}

ARM_FUNCTION
void grid_task_1000us(yeet::application::sfc::integration::sfc_grid_side_control* control,
    const task_inputs_t* inputs,
    task_outputs_t* outputs)
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
        // bool auxiliaries_on = inputs->digital_inputs.signals[0];        // DI1: Auxiliary systems ready
        // bool breakers_on = inputs->digital_inputs.signals[1];           // DI2: Grid breakers closed
        // bool start_cmd = inputs->digital_inputs.signals[2];             // DI3: Start command
        // bool stop_cmd = inputs->digital_inputs.signals[3];              // DI4: Stop command
        // bool shutdown_req = inputs->digital_inputs.signals[4];          // DI5: Emergency shutdown
        // bool current_extinguished = inputs->digital_inputs.signals[5];  // DI6: DC link current is zero

        // #ifndef RECTIFIER_MODE
        // Write digital input values directly to external_ios interface
        // get_grid_side_control().get_external_ios().get_digital_inputs().auxiliaries_are_on.write(auxiliaries_on);
        // get_grid_side_control().get_external_ios().get_digital_inputs().breakers_are_on.write(breakers_on);
        // get_grid_side_control().get_external_ios().get_digital_inputs().start_command.write(start_cmd);
        // get_grid_side_control().get_external_ios().get_digital_inputs().stop_command.write(stop_cmd);
        // get_grid_side_control().get_external_ios().get_digital_inputs().shutdown_request.write(shutdown_req); #else
        // NOTE: Digital inputs are now managed through external_ios datamodel
        // In embedded, supervisory signals come from hardware and are routed via ADC/DI task
        // These mock writes are for simulation/testing only - not applicable in embedded context
        // The actual hardware flow: Hardware ADC/DI → ADC runner → measurements → control
        // #endif
    }

    // Process fiber-optic RX data from machine-side controller
    // Fiber-optic link provides cross-coupled signals between grid and machine sides
    // #ifndef RECTIFIER_MODE
    if (inputs != nullptr && inputs->fiber_optic_rx.data_ready && inputs->fiber_optic_rx.link_ok) {
        // Decode Machine → Grid protocol message
        if (auto rx_msg = yeet::application::sfc::fiber_optic::decode_machine_to_grid(
                inputs->fiber_optic_rx.payload, inputs->fiber_optic_rx.payload_length)) {
            // Update fiber optics inputs using proper datamodel method (respects override flags)
            control->external_ios.update_fiber_optics_inputs_from_machine(yeet::speed_pu_t{rx_msg->measured_speed_pu},
                yeet::current_pu_t{rx_msg->dc_link_current_reference_pu},
                yeet::voltage_pu_t{rx_msg->machine_voltage_magnitude_pu}, rx_msg->force_commutation_request,
                rx_msg->shutdown_request);
        }
    }
    //     // #else
    //     yeet::mock_signal_input::write(
    //         get_grid_side_control().get_io().machine_side_inputs.measured_speed_pu, yeet::speed_pu_t{1.0f});
    //     yeet::mock_signal_input::write(
    //         get_grid_side_control().get_io().machine_side_inputs.current_reference,
    //         yeet::current_pu_t{0.0f});
    //     yeet::mock_signal_input::write(
    //         get_grid_side_control().get_io().machine_side_inputs.machine_voltage_magnitude,
    //         yeet::voltage_pu_t{1.0f});
    //     yeet::mock_signal_input::write(
    //         get_grid_side_control().get_io().machine_side_inputs.request_force_commutation, false);
    // #endif

    // ==================== EXECUTE CONTROL UPDATE ====================
    // Digital input overrides are automatically applied within the state machine lambdas
    // No separate apply_digital_input_overrides() call needed - the lambdas check
    // Execute 1 kHz control update (current controller)
    control->update_1ms();

    // ==================== FIBER-OPTIC TX → MACHINE-SIDE CONTROLLER ====================
    // Prepare fiber-optic TX data to send to machine-side board via I/O interface outputs
    // Transmit grid-side status and cross-coupled signals
    if (outputs != nullptr) {
        // Prepare Grid → Machine protocol message
        yeet::application::sfc::fiber_optic::grid_to_machine_message tx_msg;

        // Read from control class outputs directly (these are sent via fiber-optics to machine-side)
        tx_msg.mode = control->get_operating_mode().read();
        tx_msg.current_is_zero = control->get_is_current_zero().read();
        tx_msg.force_commutation_acknowledged = control->get_force_commutation_acknowledged().read();
        tx_msg.pulse_releases = control->get_pulse_releases().read();
        tx_msg.dc_link_current_pu = control->dc_link_measurements.get_outputs().dc_current_filtered_pu.read().value();
        tx_msg.is_saturated = control->get_current_controller_is_saturated().read();

        // Encode and transmit
        outputs->fiber_optic_tx.payload_length =
            yeet::application::sfc::fiber_optic::encode_grid_to_machine(tx_msg, outputs->fiber_optic_tx.payload);
        outputs->fiber_optic_tx.transmit_request = true;
    }

    // ==================== DIGITAL OUTPUTS → STATUS INDICATORS ====================
    // Map control system status to hardware digital outputs via I/O interface
    // Based on typical HMI and system monitoring requirements
    // if (outputs != nullptr) {
    // Read status signals from I/O interface digital outputs
    // const bool pulse_releases = get_grid_side_control().get_io().digital_outputs.pulse_releases.read();
    // const bool is_current_zero = get_grid_side_control().get_io().digital_outputs.is_current_zero.read();
    // const bool current_saturated =
    //     get_grid_side_control().get_io().digital_outputs.current_controller_saturated.read();
    // const auto operating_mode = get_grid_side_control().get_io().digital_outputs.operating_mode.read();

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

    // for (auto& n : outputs->digital_outputs.signals) {
    //     n = true;
    // }

    // outputs->digital_outputs.signals[0] = true;  // DO1: System running
    // outputs->digital_outputs.signals[1] = true;  // DO2: Current zero status
    // outputs->digital_outputs.signals[2] = true;  // DO3: Controller saturation
    // outputs->digital_outputs.signals[3] = true;  // DO4: Forced commutation mode
    // outputs->digital_outputs.signals[4] = true;
    // outputs->digital_outputs.signals[5] = true;  // DO6: Fiber link status

    // Additional outputs (DO7-DO17) available for application-specific status
    // }
}

ARM_FUNCTION
void grid_task_10000us(yeet::application::sfc::integration::sfc_grid_side_control* control,
    const task_inputs_t* inputs,
    task_outputs_t* outputs)
{
    // LOW-FREQUENCY MONITORING TASK
    // Configured rate: 10 ms (100 Hz)
    // Execution time budget: < 5 ms (50% of period)
    // Called from: TIM4_IRQHandler
    // Priority: Lowest (monitoring and diagnostics)

    // Optional: Use inputs for monitoring/diagnostics
    (void)inputs;  // Suppress unused parameter warning

    control->update_10ms();

    // Optional: Update diagnostic outputs
    (void)outputs;  // Suppress unused parameter warning
}

ARM_FUNCTION
void grid_network(
    grid_side_network_infrastructure* infrastructure, const network_inputs_t* inputs, network_outputs_t* outputs)
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
    infrastructure->network_handler.process(inputs, outputs);
}

}  // extern "C"
