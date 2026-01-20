#include "sfc_embedded/machine_side_tasks.hpp"

#include <cstdint>
#include <cstring>
#include <memory>

#include "sfc/sfc_fiber_optic_protocol.hpp"
#include "sfc/sfc_types.hpp"
#include "sfc_embedded/machine_side_network_telemetry_builder.hpp"
#include "sfc_embedded/task_io_helpers.hpp"
#include "sfc_integration/sfc_machine_side_control.hpp"
#include "yeet/network_handler.hpp"
#include "yeet/network_telemetry_manager.hpp"
#include "yeet/parameter_handler.hpp"
#include "yeet/signal.hpp"

using namespace yeet::embedded::io;

namespace {
// ==================== IO CONFIGURATION (Indices and Polarity) ====================
// These constants define the physical IO mapping for the machine-side controller.
// Change these values to match your target hardware wiring.

using yeet::application::sfc::integration::io_polarity_t;

// Digital Inputs
constexpr int DI_EXCITATION_STATUS_INDEX = 33 + 1 - 1;
constexpr io_polarity_t DI_EXCITATION_STATUS_POLARITY = io_polarity_t::ACTIVE_HIGH;

constexpr int DI_EXTERNAL_FAULT_INDEX = 33 + 2 - 1;
constexpr io_polarity_t DI_EXTERNAL_FAULT_POLARITY = io_polarity_t::ACTIVE_HIGH;

constexpr int DI_START_COMMAND_INDEX = 33 + 4 - 1;
constexpr io_polarity_t DI_START_COMMAND_POLARITY = io_polarity_t::ACTIVE_HIGH;

// Digital Outputs
constexpr int DO_EXCITATION_ON_COMMAND_INDEX = 33 + 1 - 1;
constexpr io_polarity_t DO_EXCITATION_ON_COMMAND_POLARITY = io_polarity_t::ACTIVE_HIGH;

constexpr int DO_EXCITATION_OFF_COMMAND_INDEX = 33 + 2 - 1;
constexpr io_polarity_t DO_EXCITATION_OFF_COMMAND_POLARITY = io_polarity_t::ACTIVE_HIGH;

// Analog Outputs
constexpr int AO_EXCITATION_CURRENT_REFERENCE_INDEX = 0;
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
      parameter_handler(),
      network_handler(telemetry_manager, parameter_handler)
{
    control.register_datamodel(parameter_handler);
}

ARM_FUNCTION
void machine_embedded_init(yeet::application::sfc::integration::sfc_machine_side_control* control)
{
    // Set all callbacks before modifying default parameters.
    // Callbacks are registered during network handler initialization.
    control->initialize();

    control->pu_system_datamodel.get_parameters().dc_link_current_base.write(yeet::current_t{500.0f});
    control->pu_system_datamodel.get_parameters().grid_frequency_base.write(yeet::frequency_t{50.0f});
    control->pu_system_datamodel.get_parameters().grid_voltage_base_rms.write(yeet::voltage_t{36.0f});
    control->pu_system_datamodel.get_parameters().machine_frequency_base.write(yeet::frequency_t{50.0f});
    control->pu_system_datamodel.get_parameters().machine_num_poles.write(int{2});
    control->pu_system_datamodel.get_parameters().machine_power_base.write(yeet::power_t{100.0f});
    control->pu_system_datamodel.get_parameters().machine_voltage_base_rms.write(yeet::voltage_t{36.0f});

    // ==================== INITIALIZE DIGITAL INPUTS ====================
    // control->pu_system_datamodel.get_parameters().dc_link_current_base.write(yeet::current_t{642.0f});
    // control->pu_system_datamodel.get_parameters().grid_frequency_base.write(yeet::frequency_t{50.0f});
    // control->pu_system_datamodel.get_parameters().grid_voltage_base_rms.write(yeet::voltage_t{2090.0f});
    // control->pu_system_datamodel.get_parameters().machine_frequency_base.write(yeet::frequency_t{50.0f});
    // control->pu_system_datamodel.get_parameters().machine_num_poles.write(int{2});
    // control->pu_system_datamodel.get_parameters().machine_power_base.write(yeet::power_t{2000e3});
    // control->pu_system_datamodel.get_parameters().machine_voltage_base_rms.write(yeet::voltage_t{1980.0f});

    // Configure the PT
    // The ASI interface converts 2500 V LL to 100 V secondary voltage
    control->machine_measurements.get_parameters().voltage_adc_full_scale.write(yeet::real_t{10.0f});
    control->machine_measurements.get_parameters().pt_ratio.write(yeet::real_t{25.0f});
    // The ASI interface defines the loop-gain of 1.0/25.0
    control->machine_measurements.get_parameters().voltage_transducer_gain.write(yeet::real_t{0.040047f});

    // Configuration of the voltage PT and hardware filter compensation
    control->machine_measurements.get_parameters().voltage_hardware_filter_cutoff_frequency.write(
        yeet::frequency_t{160.0f});
    control->machine_measurements.get_parameters().enable_voltage_compensation.write(false);
    control->machine_measurements.get_parameters().enable_commutation_masking.write(true);
    control->machine_measurements.get_parameters().voltage_measurement_rotation.write(
        yeet::angle_wrapped{0.0f});  // 30.0f

    control->machine_side_modulator.get_parameters().reference_type.write(
        yeet::control::six_pulse_generator::reference_type::line);
    control->machine_side_modulator.get_parameters().pulse_width.write(
        yeet::angle_wrapped::from_radians(yeet::angle_t{yeet::pi / 3.0f}));

    // ==================== IO CONFIGURATION ====================
    auto& io_params = control->external_ios.get_parameters();

    // Digital Input Indices
    io_params.di_excitation_status_index.write(DI_EXCITATION_STATUS_INDEX);
    io_params.di_external_fault_index.write(DI_EXTERNAL_FAULT_INDEX);
    io_params.di_start_command_index.write(DI_START_COMMAND_INDEX);

    // Digital Input Polarity
    io_params.di_excitation_status_polarity.write(DI_EXCITATION_STATUS_POLARITY);
    io_params.di_external_fault_polarity.write(DI_EXTERNAL_FAULT_POLARITY);
    io_params.di_start_command_polarity.write(DI_START_COMMAND_POLARITY);

    // Digital Output Indices
    io_params.do_excitation_on_command_index.write(DO_EXCITATION_ON_COMMAND_INDEX);
    io_params.do_excitation_off_command_index.write(DO_EXCITATION_OFF_COMMAND_INDEX);

    // Digital Output Polarity
    io_params.do_excitation_on_command_polarity.write(DO_EXCITATION_ON_COMMAND_POLARITY);
    io_params.do_excitation_off_command_polarity.write(DO_EXCITATION_OFF_COMMAND_POLARITY);

    // Analog Output Indices
    io_params.ao_excitation_current_reference_index.write(AO_EXCITATION_CURRENT_REFERENCE_INDEX);

    // Configure protections to be enabled by default for embedded target
    control->machine_protections.get_parameters().protection_module_enabled.write(true);
    control->machine_protections.get_parameters().over_flux_severity.write(
        yeet::application::sfc::integration::protection_severity::alarm_only);
    control->machine_protections.get_parameters().under_flux_severity.write(
        yeet::application::sfc::integration::protection_severity::alarm_only);
    control->machine_protections.get_parameters().stall_severity.write(
        yeet::application::sfc::integration::protection_severity::alarm_only);
    control->machine_protections.get_parameters().overspeed_severity.write(
        yeet::application::sfc::integration::protection_severity::alarm_only);
    control->machine_protections.get_parameters().machine_overvoltage_severity.write(
        yeet::application::sfc::integration::protection_severity::alarm_only);
    control->machine_protections.get_parameters().communication_loss_severity.write(
        yeet::application::sfc::integration::protection_severity::trip);
    control->machine_protections.get_parameters().loss_of_excitation_severity.write(
        yeet::application::sfc::integration::protection_severity::alarm_only);
    // #endif

    control->startup_sequencer.get_parameters().force_virtual_mode_operation.write(true);
    control->startup_sequencer.get_parameters().virtual_frequency_duration.write(yeet::duration_t{50000.0f});
    control->startup_sequencer.get_parameters().virtual_frequency_initial.write(
        yeet::frequency_pu_t{0.02f});  // 50 * 0.02 = 1 Hz
    control->startup_sequencer.get_parameters().virtual_frequency_final.write(
        yeet::frequency_pu_t{0.02f});  // 50 * 0.02 = 1 Hz
    control->startup_sequencer.get_parameters().virtual_torque_duration.write(yeet::duration_t{0.0f});
    control->startup_sequencer.get_parameters().virtual_torque_initial.write(yeet::torque_pu_t{0.1f});  // 10% torque
    control->startup_sequencer.get_parameters().virtual_torque_final.write(yeet::torque_pu_t{0.1f});    // 10% torque
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
    if (inputs->analog_inputs.valid) {
        int32_t diff_ab = static_cast<int32_t>(inputs->analog_inputs.channels[1]) -
                          static_cast<int32_t>(inputs->analog_inputs.channels[2]);
        int32_t diff_bc = static_cast<int32_t>(inputs->analog_inputs.channels[2]) -
                          static_cast<int32_t>(inputs->analog_inputs.channels[3]);
        control->external_ios.set_analog_inputs(yeet::control::adc_counts{diff_ab}, yeet::control::adc_counts{diff_bc});
    }

    // Execute the 5 kHz control update (measurements → observers → modulators → pulse generator)
    control->update_200us();

    // Update hardware status flags for LED indicators
    outputs->status.trip = control->machine_protections.get_outputs().has_active_trip ||
                           control->machine_protections.get_outputs().is_inhibited;
    outputs->status.alarm = control->machine_protections.get_outputs().has_active_alarm;

    // ==================== ANALOG OUTPUTS (DAC) → EXCITATION COMMANDS ====================
    const auto& io_params = control->external_ios.get_parameters();
    write_ao(outputs, io_params.ao_excitation_current_reference_index.read(),
        control->external_ios.get_excitation_current_reference().value(),
        io_params.ao_excitation_current_reference_gain.read(), io_params.ao_excitation_current_reference_offset.read());

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
        // Read digital inputs from hardware
        const bool excitation_status = read_di(inputs, io_params.di_excitation_status_index.read());
        const bool external_fault = read_di(inputs, io_params.di_external_fault_index.read());
        const bool start_command = read_di(inputs, io_params.di_start_command_index.read());

        // Set digital inputs in datamodel (applies polarity inversion)
        control->external_ios.set_digital_inputs(excitation_status, external_fault, start_command);

        // Set analog excitation feedback (optional)
        control->external_ios.set_excitation_inputs(yeet::current_pu_t{0.0f});
    }

    // Process fiber-optic RX data from grid-side controller
    // Fiber-optic link provides cross-coupled signals between grid and machine sides
    bool communication_valid = false;
    if (inputs->fiber_optic_rx.data_ready && inputs->fiber_optic_rx.link_ok) {
        // Use control class method to decode all signals (ensures consistency with host simulator)
        // This updates external_ios with: mode, state_id, force_commutation_acknowledged, current_is_zero,
        // is_saturated, pulse_releases, dc_link_current, preset_request
        communication_valid = control->decode_fiber_optic_rx_from_grid(
            inputs->fiber_optic_rx.payload, inputs->fiber_optic_rx.payload_length);
    }
    // Update protection system with communication status for watchdog monitoring
    bool remote_ok = control->external_ios.get_fiber_optics_inputs_from_grid().communication_ok.read();
    control->machine_protections.update_communication_status(communication_valid, remote_ok);

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

    // Excitation command outputs from control system to external excitation hardware
    write_do(
        outputs, io_params.do_excitation_on_command_index.read(), control->external_ios.get_excitation_on_command());
    write_do(
        outputs, io_params.do_excitation_off_command_index.read(), control->external_ios.get_excitation_off_command());
    // Excitation current reference output (analog output - would typically use analog_outputs)
    write_ao(outputs, AO_EXCITATION_CURRENT_REFERENCE_INDEX,
        control->external_ios.get_excitation_current_reference().value(), 1.0f, 0.0f);
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