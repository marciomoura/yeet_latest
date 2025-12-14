#include "sfc_embedded/machine_side_tasks.hpp"

#include <cstdint>
#include <cstring>
#include <memory>

#include "sfc/sfc_fiber_optic_protocol.hpp"
#include "sfc/sfc_types.hpp"
#include "sfc_embedded/machine_side_network_telemetry_builder.hpp"
#include "sfc_integration/sfc_machine_side_control.hpp"
#include "yeet/network_handler.hpp"
#include "yeet/network_telemetry_manager.hpp"
#include "yeet/parameter_handler.hpp"
#include "yeet/signal.hpp"

namespace {

inline bool is_valid_di_index(int idx) noexcept { return idx >= 0 && idx < TASK_IO_NUM_DIGITAL_INPUTS; }

inline bool is_valid_do_index(int idx) noexcept { return idx >= 0 && idx < TASK_IO_NUM_DIGITAL_OUTPUTS; }

inline bool is_valid_ao_index(int idx) noexcept { return idx >= 0 && idx < TASK_IO_NUM_ANALOG_OUTPUTS; }

inline bool read_di(const task_inputs_t* inputs, int idx) noexcept
{
    if (inputs == nullptr || !inputs->digital_inputs.valid) {
        return false;
    }
    if (!is_valid_di_index(idx)) {
        return false;
    }
    return inputs->digital_inputs.signals[idx];
}

inline void clear_digital_outputs(task_outputs_t* outputs) noexcept
{
    if (outputs == nullptr) {
        return;
    }
    for (auto& s : outputs->digital_outputs.signals) {
        s = false;
    }
}

inline void write_do(task_outputs_t* outputs, int idx, bool value) noexcept
{
    if (outputs == nullptr) {
        return;
    }
    if (!is_valid_do_index(idx)) {
        return;
    }
    outputs->digital_outputs.signals[idx] = value;
}

inline void clear_analog_outputs(task_outputs_t* outputs) noexcept
{
    if (outputs == nullptr) {
        return;
    }
    for (auto& c : outputs->analog_outputs.channels) {
        c = 0.0f;
    }
    outputs->analog_outputs.valid = false;
}

inline void write_ao(task_outputs_t* outputs, int idx, float value) noexcept
{
    if (outputs == nullptr) {
        return;
    }
    if (!is_valid_ao_index(idx)) {
        return;
    }
    outputs->analog_outputs.channels[idx] = value;
    outputs->analog_outputs.valid = true;
}

}  // namespace

// Ensure functions are compiled with Thumb interworking
#ifdef __arm__
#define ARM_FUNCTION __attribute__((target("thumb")))
#else
#define ARM_FUNCTION
#endif

extern "C" {

// ==================== Static Instances for Embedded Single-Controller Build ====================
// For embedded builds with ONLY machine-side control (no dual-controller), provide static instances
// that can be accessed by the backward compatibility wrappers in the header file

#if defined(YEET_EMBEDDED_MACHINE_SIDE) && !defined(YEET_BUILD_DUAL_CONTROLLER)

// File-scope storage for static instances - allocated at first use
static yeet::application::sfc::integration::sfc_machine_side_control* static_machine_control_ptr = nullptr;
static machine_side_network_infrastructure* static_machine_infrastructure_ptr = nullptr;

// Initialization function to create the static instances
// Called only once during yeet_embedded_init and from the accessor functions
static void _initialize_static_instances()
{
    if (static_machine_control_ptr == nullptr) {
        // Allocate static storage for control instance
        static yeet::application::sfc::integration::sfc_machine_side_control static_control_storage;
        static_machine_control_ptr = &static_control_storage;

        // Allocate static storage for network infrastructure instance
        static machine_side_network_infrastructure static_infra_storage(*static_machine_control_ptr);
        static_machine_infrastructure_ptr = &static_infra_storage;
    }
}

// Accessor functions for backward compatibility wrappers
yeet::application::sfc::integration::sfc_machine_side_control& get_machine_side_control()
{
    _initialize_static_instances();
    return *static_machine_control_ptr;
}

machine_side_network_infrastructure& get_machine_side_network_infrastructure()
{
    _initialize_static_instances();
    return *static_machine_infrastructure_ptr;
}

#endif  // YEET_EMBEDDED_MACHINE_SIDE && !YEET_BUILD_DUAL_CONTROLLER

// ==================== Network Telemetry/Parameter Infrastructure ====================
// These are instantiated per control instance to support both embedded (static) and host (dynamic) usage

machine_side_network_infrastructure::machine_side_network_infrastructure(
    yeet::application::sfc::integration::sfc_machine_side_control& control)
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
void machine_embedded_init(yeet::application::sfc::integration::sfc_machine_side_control* control)
{
    // ==================== INITIALIZE DIGITAL INPUTS ====================

    // #if defined(YEET_EMBEDDED_MACHINE_SIDE) && !defined(YEET_BUILD_DUAL_CONTROLLER)
    //  Parametrize the software with hardware-specific gains (commented out - should be done via plant_hardware
    //  staging)
    control->machine_measurements.get_parameters().voltage_transducer_gain.write(yeet::real_t{1.0f / 300.0f});
    control->machine_side_pulse_generator.get_parameters().reference_type.write(
        yeet::control::six_pulse_generator::reference_type::line);
    control->machine_side_pulse_generator.get_parameters().pulse_width.write(
        yeet::angle_wrapped::from_radians(yeet::angle_t{yeet::pi / 3.0f}));
    // #endif

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
        control->external_ios.set_analog_inputs(
            yeet::control::adc_counts{static_cast<std::int32_t>(inputs->analog_inputs.channels[1])},
            yeet::control::adc_counts{static_cast<std::int32_t>(inputs->analog_inputs.channels[2])});
    }

    // Execute the 5 kHz control update (measurements → observers → modulators → pulse generator)
    control->update_200us();

    // ==================== ANALOG OUTPUTS (DAC) → EXCITATION COMMANDS ====================
    clear_analog_outputs(outputs);

    const auto& io_params = control->external_ios.get_parameters();
    const float excitation_current_reference =
        static_cast<float>(control->external_ios.get_excitation_current_reference());
    write_ao(outputs, io_params.ao_excitation_current_reference_index.read(), excitation_current_reference);
    // ==================== THYRISTOR PULSES → PWM OUTPUTS ====================
    // Extract firing pulses from grid-side I/O interface and map to hardware PWM channels
    // Based on sfc_io_definitions.md PWM mapping:
    //   PWM1 (PC6) → a_plus  (Phase A positive arm, T1)
    //   PWM2 (PC7) → c_minus (Phase C negative arm, T6)
    //   PWM3 (PC8) → b_plus  (Phase B positive arm, T3)
    //   PWM4 (PC9) → a_minus (Phase A negative arm, T4)
    //   PWM5 (PB0) → c_plus  (Phase C positive arm, T5)
    //   PWM6 (PB1) → b_minus (Phase B negative arm, T2)
    const auto& firing_pulses = control->get_firing_pulses().read();
    outputs->thyristor_pulses.a_plus = firing_pulses.p1;
    outputs->thyristor_pulses.c_minus = firing_pulses.p2;
    outputs->thyristor_pulses.b_plus = firing_pulses.p3;
    outputs->thyristor_pulses.a_minus = firing_pulses.p4;
    outputs->thyristor_pulses.c_plus = firing_pulses.p5;
    outputs->thyristor_pulses.b_minus = firing_pulses.p6;
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
    const auto& io_params = control->external_ios.get_parameters();
    if (inputs != nullptr && inputs->digital_inputs.valid) {
        // Excitation feedback/status is selectable via DI mapping; analog excitation feedback is hardwired/optional.
        const bool excitation_running = read_di(inputs, io_params.di_excitation_running_index.read());
        control->external_ios.set_excitation_inputs(yeet::current_pu_t{0.0f}, excitation_running);
    }

    // Process fiber-optic RX data from grid-side controller
    // Fiber-optic link provides cross-coupled signals between grid and machine sides
    bool communication_valid = false;
    if (inputs != nullptr && inputs->fiber_optic_rx.data_ready && inputs->fiber_optic_rx.link_ok) {
        // Use control class method to decode all signals (ensures consistency with host simulator)
        // This updates external_ios with: mode, state_id, force_commutation_acknowledged, current_is_zero,
        // is_saturated, pulse_releases, dc_link_current, preset_request
        communication_valid = control->decode_fiber_optic_rx_from_grid(
            inputs->fiber_optic_rx.payload, inputs->fiber_optic_rx.payload_length);
    }
    // Update protection system with communication status for watchdog monitoring
    control->machine_protections.update_communication_status(communication_valid);

    // Execute 1 kHz control update (machine controller)
    control->update_1ms();

    // ==================== FIBER-OPTIC TX → GRID-SIDE CONTROLLER ====================
    // Prepare fiber-optic TX data to send to grid-side board via I/O interface outputs
    // Transmit machine-side status and cross-coupled signals
    // Use control class method to encode all signals (ensures consistency with host simulator)
    // This includes: measured_speed, dc_link_current_reference_pu, machine_voltage_magnitude_pu,
    // force_commutation_request, shutdown_request
    uint8_t payload_words = control->encode_fiber_optic_tx_to_grid();
    outputs->fiber_optic_tx.payload_length = payload_words;

    // Copy encoded payload from control class to output buffer
    // Note: payload_length is in uint32_t words, memcpy needs bytes
    std::memcpy(
        outputs->fiber_optic_tx.payload, control->get_fiber_optic_tx_payload(), payload_words * sizeof(uint32_t));
    outputs->fiber_optic_tx.transmit_request = true;

    // ==================== DIGITAL OUTPUTS → STATUS / COMMANDS ====================
    // Drive mapped logical outputs onto the physical DO array.
    clear_digital_outputs(outputs);

    // Fiber link status from RX (direct pass-through)
    write_do(outputs, io_params.do_fiber_link_ok_index.read(), inputs->fiber_optic_rx.link_ok);

    // Excitation command outputs from control system to external excitation hardware
    write_do(
        outputs, io_params.do_excitation_release_index.read(), control->external_ios.get_excitation_release_command());

    // Excitation current reference output (analog output - would typically use analog_outputs)
    // For now, we'll map it to a digital output as a placeholder
    // In real hardware, this would go to a DAC or PWM output
    // TODO: Add analog output support and map excitation_current_reference to AO1

    // Machine-side control status outputs
    // DO3-DO16: Reserved for future expansion (speed controller status, fault conditions, etc.)
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

    (void)inputs;   // Suppress unused parameter warning if inputs not needed
    (void)outputs;  // Suppress unused parameter warning if outputs not needed
}

ARM_FUNCTION
void machine_network(
    machine_side_network_infrastructure* infrastructure, const network_inputs_t* inputs, network_outputs_t* outputs)
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

// ==================== Wrapper Functions for Embedded Builds ====================
// For embedded builds, provide the yeet_* function names that main.c expects
// These delegate to the machine_* functions with the static instances

#if defined(YEET_EMBEDDED_MACHINE_SIDE) && !defined(YEET_BUILD_DUAL_CONTROLLER)

ARM_FUNCTION
void yeet_embedded_init(void)
{
    _initialize_static_instances();
    machine_embedded_init(static_machine_control_ptr);
}

ARM_FUNCTION
void yeet_task_200us(const task_inputs_t* inputs, task_outputs_t* outputs)
{
    _initialize_static_instances();
    machine_task_200us(static_machine_control_ptr, inputs, outputs);
}

ARM_FUNCTION
void yeet_task_1000us(const task_inputs_t* inputs, task_outputs_t* outputs)
{
    _initialize_static_instances();
    machine_task_1000us(static_machine_control_ptr, inputs, outputs);
}

ARM_FUNCTION
void yeet_task_10000us(const task_inputs_t* inputs, task_outputs_t* outputs)
{
    _initialize_static_instances();
    machine_task_10000us(static_machine_control_ptr, inputs, outputs);
}

ARM_FUNCTION
void yeet_network(const network_inputs_t* inputs, network_outputs_t* outputs)
{
    _initialize_static_instances();
    machine_network(static_machine_infrastructure_ptr, inputs, outputs);
}

#endif  // YEET_EMBEDDED_MACHINE_SIDE && !YEET_BUILD_DUAL_CONTROLLER

}  // extern "C"