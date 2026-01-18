#include "sfc_embedded/grid_side_tasks.hpp"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>

#include "sfc/sfc_fiber_optic_protocol.hpp"
#include "sfc/sfc_types.hpp"
#include "sfc_embedded/grid_side_network_telemetry_builder.hpp"
#include "sfc_embedded/task_io_helpers.hpp"
#include "sfc_integration/sfc_grid_side_control.hpp"
#include "yeet/network_handler.hpp"
#include "yeet/network_telemetry_manager.hpp"
#include "yeet/parameter_handler.hpp"
#include "yeet/signal.hpp"
#include "yeet_control/reference_frame.hpp"

using namespace yeet::embedded::io;

// Ensure functions are compiled with Thumb interworking
#ifdef __arm__
#define ARM_FUNCTION __attribute__((target("thumb")))
#else
#define ARM_FUNCTION
#endif

#define RECTIFIER_MODE 1

namespace {
// ==================== IO CONFIGURATION (Indices and Polarity) ====================
// These constants define the physical IO mapping for the grid-side controller.
// Change these values to match your target hardware wiring.

using yeet::application::sfc::integration::io_polarity_t;

// Digital Inputs
// This is only connected to the reactor
constexpr int DI_AUXILIARIES_ARE_ON_INDEX = 14 - 1;  // Device A11 - its lv supervision
constexpr io_polarity_t DI_AUXILIARIES_ARE_ON_POLARITY = io_polarity_t::ACTIVE_LOW;

constexpr int DI_MV_BREAKER_IS_CLOSED_INDEX = 16 + 5 - 1;  // Device A12
constexpr io_polarity_t DI_MV_BREAKER_IS_CLOSED_POLARITY = io_polarity_t::ACTIVE_HIGH;

constexpr int DI_START_COMMAND_INDEX = 1 - 1;  // Device A11
constexpr io_polarity_t DI_START_COMMAND_POLARITY = io_polarity_t::ACTIVE_HIGH;

constexpr int DI_STOP_COMMAND_INDEX = 2 - 1;  // Device A11
constexpr io_polarity_t DI_STOP_COMMAND_POLARITY = io_polarity_t::ACTIVE_HIGH;

constexpr int DI_MV_BREAKER_IS_TRIPPING_INDEX = 16 + 7 - 1;  // Device A12
constexpr io_polarity_t DI_MV_BREAKER_IS_TRIPPING_POLARITY = io_polarity_t::ACTIVE_HIGH;

constexpr int DI_EMERGENCY_OFF_INDEX = 16 + 13 - 1;  // Device A12
constexpr io_polarity_t DI_EMERGENCY_OFF_POLARITY = io_polarity_t::ACTIVE_LOW;

constexpr int DI_DOOR_IS_OPEN_INDEX = 13 - 1;                                   // Device A11
constexpr io_polarity_t DI_DOOR_IS_OPEN_POLARITY = io_polarity_t::ACTIVE_HIGH;  // to be checked

constexpr int DI_EXTERNAL_FAULT_INDEX = 16 + 15 - 1;                              // Device A12
constexpr io_polarity_t DI_EXTERNAL_FAULT_POLARITY = io_polarity_t::ACTIVE_HIGH;  // to be checked

// To be tested...
constexpr int DI_IS_RUNNING_ACU_FAN_1_INDEX = 16 + 16 - 1;  // Device A12
constexpr io_polarity_t DI_IS_RUNNING_ACU_FAN_1_POLARITY = io_polarity_t::ACTIVE_HIGH;
constexpr int DI_IS_RUNNING_ACU_FAN_2_INDEX = 16 + 16 - 1;  // Device A12
constexpr io_polarity_t DI_IS_RUNNING_ACU_FAN_2_POLARITY = io_polarity_t::ACTIVE_HIGH;
constexpr int DI_IS_RUNNING_ACU_FAN_3_INDEX = 16 + 16 - 1;  // Device A12
constexpr io_polarity_t DI_IS_RUNNING_ACU_FAN_3_POLARITY = io_polarity_t::ACTIVE_HIGH;

// Digital Outputs
constexpr int DO_MV_BREAKER_CLOSE_INDEX = 32 + 5 - 1;
constexpr io_polarity_t DO_MV_BREAKER_CLOSE_POLARITY = io_polarity_t::ACTIVE_HIGH;

constexpr int DO_MV_BREAKER_OPEN_INDEX = 32 + 6 - 1;
constexpr io_polarity_t DO_MV_BREAKER_OPEN_POLARITY = io_polarity_t::ACTIVE_HIGH;

constexpr int DO_ON_COMMAND_ACU_FAN_1_INDEX = 32 + 1 - 1;
constexpr io_polarity_t DO_ON_COMMAND_ACU_FAN_1_POLARITY = io_polarity_t::ACTIVE_HIGH;

constexpr int DO_ON_COMMAND_ACU_FAN_2_INDEX = 32 + 2 - 1;
constexpr io_polarity_t DO_ON_COMMAND_ACU_FAN_2_POLARITY = io_polarity_t::ACTIVE_HIGH;

constexpr int DO_ON_COMMAND_ACU_FAN_3_INDEX = 32 + 3 - 1;
constexpr io_polarity_t DO_ON_COMMAND_ACU_FAN_3_POLARITY = io_polarity_t::ACTIVE_HIGH;

constexpr int DO_ON_COMMAND_ACU_FAN_4_INDEX = 32 + 4 - 1;
constexpr io_polarity_t DO_ON_COMMAND_ACU_FAN_4_POLARITY = io_polarity_t::ACTIVE_HIGH;

constexpr int DO_SOFTWARE_TRIP_1_INDEX = 10 - 1;
constexpr io_polarity_t DO_SOFTWARE_TRIP_1_POLARITY = io_polarity_t::ACTIVE_HIGH;

constexpr int DO_SOFTWARE_TRIP_2_INDEX = 11 - 1;
constexpr io_polarity_t DO_SOFTWARE_TRIP_2_POLARITY = io_polarity_t::ACTIVE_HIGH;

constexpr int DO_SOFTWARE_TRIP_3_INDEX = 15 - 1;
constexpr io_polarity_t DO_SOFTWARE_TRIP_3_POLARITY = io_polarity_t::ACTIVE_HIGH;

constexpr int DO_EMERGENCY_OFF_RESET_INDEX = 16 - 1;
constexpr io_polarity_t DO_EMERGENCY_OFF_RESET_POLARITY = io_polarity_t::ACTIVE_HIGH;

// DO32 - EMERGENCY OFF RESET is missing

// 33 DO1	ON CMD ACU REACT FAN
// 34 DO2	ON CMD ACU CONV1 FAN
// 35 DO3	ON CMD ACU CONV2 FAN
// 36 DO4	ON CMD ACU CTR FAN
// 37 DO5	ICB CLOSE CMD/ICB Control

}  // namespace

extern "C" {

// ==================== Static Instances for Embedded Single-Controller Build ====================
// For embedded builds with ONLY grid-side control (no dual-controller), provide static instances
// that can be accessed by the backward compatibility wrappers in the header file

#if defined(YEET_EMBEDDED_GRID_SIDE) && !defined(YEET_BUILD_DUAL_CONTROLLER)

// File-scope storage for static instances - allocated at first use
static yeet::application::sfc::integration::sfc_grid_side_control* static_grid_control_ptr = nullptr;
static grid_side_network_infrastructure* static_grid_infrastructure_ptr = nullptr;

// Initialization function to create the static instances
// Called only once during yeet_embedded_init and from the accessor functions
static void _initialize_static_instances()
{
    if (static_grid_control_ptr == nullptr) {
        // Allocate static storage for control instance
        static yeet::application::sfc::integration::sfc_grid_side_control static_control_storage;
        static_grid_control_ptr = &static_control_storage;

        // Allocate static storage for network infrastructure instance
        static grid_side_network_infrastructure static_infra_storage(*static_grid_control_ptr);
        static_grid_infrastructure_ptr = &static_infra_storage;
    }
}

// Accessor functions for backward compatibility wrappers
yeet::application::sfc::integration::sfc_grid_side_control& get_grid_side_control()
{
    _initialize_static_instances();
    return *static_grid_control_ptr;
}

grid_side_network_infrastructure& get_grid_side_network_infrastructure()
{
    _initialize_static_instances();
    return *static_grid_infrastructure_ptr;
}

#endif  // YEET_EMBEDDED_GRID_SIDE && !YEET_BUILD_DUAL_CONTROLLER

// ==================== Network Telemetry/Parameter Infrastructure ====================
// These are instantiated per control instance to support both embedded (static) and host (dynamic) usage

grid_side_network_infrastructure::grid_side_network_infrastructure(
    yeet::application::sfc::integration::sfc_grid_side_control& control)
    : telemetry_builder(control),
      telemetry_manager(telemetry_builder, 10),
      parameter_handler(),
      network_handler(telemetry_manager, parameter_handler)
{
    control.register_datamodel(parameter_handler);
}
ARM_FUNCTION
void grid_embedded_init(yeet::application::sfc::integration::sfc_grid_side_control* control)
{
    // Bypass all IOs (for testing)
    // control->external_ios.get_parameters().di_auxiliaries_are_on_override.write(true);
    // control->external_ios.get_parameters().di_auxiliaries_are_on_override_value.write(true);
    // control->external_ios.get_parameters().di_auxiliaries_are_on_override.write(true);
    // control->external_ios.get_parameters().di_auxiliaries_are_on_override_value.write(true);
    // control->external_ios.get_parameters().di_mv_breaker_is_closed_override.write(true);
    // control->external_ios.get_parameters().di_mv_breaker_is_closed_override_value.write(true);
    // control->external_ios.get_parameters().di_start_command_override.write(true);
    // control->external_ios.get_parameters().di_start_command_override_value.write(false);  // Start command not active
    // control->external_ios.get_parameters().di_stop_command_override.write(true);
    // control->external_ios.get_parameters().di_stop_command_override_value.write(false);  // Stop command not active
    // control->external_ios.get_parameters().di_mv_breaker_is_tripping_override.write(true);
    // control->external_ios.get_parameters().di_mv_breaker_is_tripping_override_value.write(false);
    // control->external_ios.get_parameters().di_external_fault_override.write(true);
    // control->external_ios.get_parameters().di_external_fault_override_value.write(false);
    // control->external_ios.get_parameters().di_emergency_off_override.write(true);
    // control->external_ios.get_parameters().di_emergency_off_override_value.write(false);
    // control->external_ios.get_parameters().di_door_is_open_override.write(true);
    // control->external_ios.get_parameters().di_door_is_open_override_value.write(false);
    // control->external_ios.get_parameters().di_fans_are_running_override.write(true);
    // control->external_ios.get_parameters().di_fans_are_running_override_value.write(true);

    // control->state_machine.get_parameters().auxiliaries_are_on_override.write(true);
    // control->state_machine.get_parameters().auxiliaries_are_on_override_value.write(true);
    // control->state_machine.get_parameters().mv_breaker_is_closed_override.write(true);
    // control->state_machine.get_parameters().mv_breaker_is_closed_override_value.write(true);
    // control->state_machine.get_parameters().is_current_extinguished_override.write(true);
    // control->state_machine.get_parameters().is_current_extinguished_override_value.write(true);

    // control->state_machine.get_parameters().turn_on_command.write(false);
    // control->state_machine.get_parameters().start_command.write(false);
    // control->state_machine.get_parameters().stop_command.write(false);

    // // Force natural commutation
    // control->operation_mode_monitor.get_parameters().force_natural_commutation.write(true);
    // control->current_controller.get_parameters().current_reference_magnitude_limit.write(yeet::current_pu_t{0.1f});
    // control->current_controller.get_parameters().forced_current_reference.write(yeet::current_pu_t{0.1f});

    //
    //
    // Initialize grid-side control subsystem
    // This will also connect internal signals (observer → measurements for angle/frequency/pulses)
    // Digital inputs are already default-initialized to safe values (false) by the datamodel
    control->grid_side_observer.get_parameters().initial_frequency.write(yeet::frequency_pu_t{0.0f});
    control->initialize();

    // Configure the per-unit system:
    control->pu_system_datamodel.get_parameters().dc_link_current_base.write(yeet::current_t{642.0f});
    control->pu_system_datamodel.get_parameters().grid_frequency_base.write(yeet::frequency_t{50.0f});
    control->pu_system_datamodel.get_parameters().grid_voltage_base_rms.write(yeet::voltage_t{2090.0f});
    control->pu_system_datamodel.get_parameters().machine_frequency_base.write(yeet::frequency_t{50.0f});
    control->pu_system_datamodel.get_parameters().machine_num_poles.write(int{2});
    control->pu_system_datamodel.get_parameters().machine_power_base.write(yeet::power_t{2000e3});
    control->pu_system_datamodel.get_parameters().machine_voltage_base_rms.write(yeet::voltage_t{1980.0f});

    // Configure the PT
    // The ASI interface converts 2500 V LL to 100 V secondary voltage
    control->grid_measurements.get_parameters().voltage_adc_full_scale.write(yeet::real_t{10.0f});
    control->grid_measurements.get_parameters().pt_ratio.write(yeet::real_t{25.0f});
    // The ASI interface defines the loop-gain of 1.0/25.0
    control->grid_measurements.get_parameters().voltage_transducer_gain.write(yeet::real_t{0.040047f});

    // Configuration of the voltage PT and hardware filter compensation
    control->grid_measurements.get_parameters().voltage_hardware_filter_cutoff_frequency.write(
        yeet::frequency_t{160.0f});
    control->grid_measurements.get_parameters().enable_voltage_compensation.write(false);
    control->grid_measurements.get_parameters().enable_commutation_masking.write(true);
    control->grid_measurements.get_parameters().voltage_measurement_rotation.write(yeet::angle_wrapped{0.0f});  // 30.0f

    // Configuration of the CT
    // Adaptation of the adc full-scale, ct ratio and transducer gain
    control->grid_measurements.get_parameters().current_adc_full_scale.write(yeet::real_t{10.0f});
    control->grid_measurements.get_parameters().ct_ratio.write(yeet::real_t{1000.0f});
    control->grid_measurements.get_parameters().current_transducer_gain.write(yeet::real_t{2.538252f});  // 1 A -> 4 V

    control->grid_side_pulse_generator.get_parameters().reference_type.write(
        yeet::control::six_pulse_generator::reference_type::line);
    control->grid_side_pulse_generator.get_parameters().pulse_width.write(
        yeet::angle_wrapped::from_radians(yeet::angle_t{yeet::pi / 3.0f}));

    // ==================== IO CONFIGURATION ====================
    auto& io_params = control->get_external_ios().get_parameters();

    // Digital Input Indices
    io_params.di_auxiliaries_are_on_index.write(DI_AUXILIARIES_ARE_ON_INDEX);
    io_params.di_mv_breaker_is_closed_index.write(DI_MV_BREAKER_IS_CLOSED_INDEX);
    io_params.di_start_command_index.write(DI_START_COMMAND_INDEX);
    io_params.di_stop_command_index.write(DI_STOP_COMMAND_INDEX);
    io_params.di_mv_breaker_is_tripping_index.write(DI_MV_BREAKER_IS_TRIPPING_INDEX);
    io_params.di_emergency_off_index.write(DI_EMERGENCY_OFF_INDEX);
    io_params.di_door_is_open_index.write(DI_DOOR_IS_OPEN_INDEX);
    io_params.di_external_fault_index.write(DI_EXTERNAL_FAULT_INDEX);

    // Digital Input Polarity
    io_params.di_auxiliaries_are_on_polarity.write(DI_AUXILIARIES_ARE_ON_POLARITY);
    io_params.di_mv_breaker_is_closed_polarity.write(DI_MV_BREAKER_IS_CLOSED_POLARITY);
    io_params.di_start_command_polarity.write(DI_START_COMMAND_POLARITY);
    io_params.di_stop_command_polarity.write(DI_STOP_COMMAND_POLARITY);
    io_params.di_mv_breaker_is_tripping_polarity.write(DI_MV_BREAKER_IS_TRIPPING_POLARITY);
    io_params.di_emergency_off_polarity.write(DI_EMERGENCY_OFF_POLARITY);
    io_params.di_door_is_open_polarity.write(DI_DOOR_IS_OPEN_POLARITY);
    io_params.di_external_fault_polarity.write(DI_EXTERNAL_FAULT_POLARITY);
    // Digital Output Indices
    io_params.do_mv_breaker_close_index.write(DO_MV_BREAKER_CLOSE_INDEX);
    io_params.do_mv_breaker_open_index.write(DO_MV_BREAKER_OPEN_INDEX);
    io_params.do_on_command_acu_fan_1_index.write(DO_ON_COMMAND_ACU_FAN_1_INDEX);
    io_params.do_on_command_acu_fan_2_index.write(DO_ON_COMMAND_ACU_FAN_2_INDEX);
    io_params.do_on_command_acu_fan_3_index.write(DO_ON_COMMAND_ACU_FAN_3_INDEX);
    io_params.do_on_command_acu_fan_4_index.write(DO_ON_COMMAND_ACU_FAN_4_INDEX);
    io_params.do_software_trip_1_index.write(DO_SOFTWARE_TRIP_1_INDEX);
    io_params.do_software_trip_2_index.write(DO_SOFTWARE_TRIP_2_INDEX);
    io_params.do_software_trip_3_index.write(DO_SOFTWARE_TRIP_3_INDEX);
    io_params.do_emergency_off_reset_index.write(DO_EMERGENCY_OFF_RESET_INDEX);

    // Digital Output Polarity
    io_params.do_mv_breaker_close_polarity.write(DO_MV_BREAKER_CLOSE_POLARITY);
    io_params.do_mv_breaker_open_polarity.write(DO_MV_BREAKER_OPEN_POLARITY);
    io_params.do_on_command_acu_fan_1_polarity.write(DO_ON_COMMAND_ACU_FAN_1_POLARITY);
    io_params.do_on_command_acu_fan_2_polarity.write(DO_ON_COMMAND_ACU_FAN_2_POLARITY);
    io_params.do_on_command_acu_fan_3_polarity.write(DO_ON_COMMAND_ACU_FAN_3_POLARITY);
    io_params.do_on_command_acu_fan_4_polarity.write(DO_ON_COMMAND_ACU_FAN_4_POLARITY);
    io_params.do_software_trip_1_polarity.write(DO_SOFTWARE_TRIP_1_POLARITY);
    io_params.do_software_trip_2_polarity.write(DO_SOFTWARE_TRIP_2_POLARITY);
    io_params.do_software_trip_3_polarity.write(DO_SOFTWARE_TRIP_3_POLARITY);
    io_params.do_emergency_off_reset_polarity.write(DO_EMERGENCY_OFF_RESET_POLARITY);
    control->grid_protections.get_parameters().protection_module_enabled.write(false);

    // Configure protections to be enabled by default for embedded target
    // control->grid_protections.get_parameters().protection_module_enabled.write(true);
    control->grid_protections.get_parameters().overcurrent_severity.write(
        yeet::application::sfc::integration::protection_severity::trip);
    control->grid_protections.get_parameters().overvoltage_severity.write(
        yeet::application::sfc::integration::protection_severity::trip);
    control->grid_protections.get_parameters().undervoltage_severity.write(
        yeet::application::sfc::integration::protection_severity::trip);
    control->grid_protections.get_parameters().communication_loss_severity.write(
        yeet::application::sfc::integration::protection_severity::trip);
    control->grid_protections.get_parameters().door_open_severity.write(
        yeet::application::sfc::integration::protection_severity::trip);
    // #endif
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
        static_cast<int16_t>(inputs->analog_inputs.channels[1] - inputs->analog_inputs.channels[2])});
    control->get_external_ios().get_analog_inputs().v_bc_counts.write(yeet::control::adc_counts{
        static_cast<int16_t>(inputs->analog_inputs.channels[2] - inputs->analog_inputs.channels[3])});
    control->get_external_ios().get_analog_inputs().i_a_counts.write(
        yeet::control::adc_counts{static_cast<int16_t>(inputs->analog_inputs.channels[5])});
    control->get_external_ios().get_analog_inputs().i_c_counts.write(
        yeet::control::adc_counts{static_cast<int16_t>(inputs->analog_inputs.channels[7])});

    // Execute the 5 kHz control update (measurements → observers → modulators → pulse generator)
    control->update_200us();

    // Update hardware status flags for LED indicators
    outputs->status.trip =
        control->grid_protections.get_outputs().has_active_trip || control->grid_protections.get_outputs().is_inhibited;
    outputs->status.alarm = control->grid_protections.get_outputs().has_active_alarm;

    // ==================== ANALOG OUTPUTS (DAC) → MONITORING/TESTING ====================
    // Map control system signals to DAC outputs for monitoring and testing
    // The AD5724 provides 4 analog output channels (±5V or ±10V range)
    // Example DAC output mappings (customize based on your application):
    // Channel A: Test signal 1
    // Channel B: Test signal 2
    // Channel C: Test signal 3
    // Channel D: Test signal 4
    auto& analog_outputs = outputs->analog_outputs.channels;
    // analog_outputs[0] = control->get_grid_side_observer()
    //                         .get_outputs()
    //                         .estimated_reference_angle.read()
    //                         .get_pu();  //
    //                         control->get_grid_measurements().get_outputs().grid_voltage_line.read().ab();
    analog_outputs[0] = control->get_grid_measurements().get_outputs().grid_voltage_abc_unmasked.read().a();
    // yeet::to_line_voltage(control->get_grid_measurements().get_outputs().grid_voltage_abc_unmasked.read().a()).ab();
    analog_outputs[1] = control->get_grid_measurements().get_outputs().grid_current_abc.read().a();
    // control->get_grid_measurements().get_outputs().grid_voltage_abc_unmasked.read().a();

    // control->get_grid_side_observer().get_outputs().estimated_reference_angle.read().get_pu();
    analog_outputs[2] = control->get_grid_side_observer().get_outputs().estimated_reference_angle.read().get_pu();
    analog_outputs[3] = control->get_grid_side_observer().get_outputs().estimated_reference_angle.read().get_pu();
    outputs->analog_outputs.valid = true;
    // analog_outputs[0] = control->get_grid_measurements().get_outputs().grid_voltage_abc_unmasked.read().a();
    // // control->get_grid_side_observer().get_outputs().estimated_reference_angle.read().get_pu();
    // analog_outputs[1] =
    //     control->get_grid_measurements()
    //         .get_outputs()
    //         .grid_voltage_abc_unmasked.read()
    //         .a();  //  5.0f;  // control->get_grid_measurements().get_outputs().grid_voltage_abc_unmasked.read().a();
    // analog_outputs[2] =
    //     control->get_grid_measurements()
    //         .get_outputs()
    //         .grid_voltage_abc_unmasked.read()
    //         .a();  // 5.0f;  // control->get_grid_measurements().get_outputs().grid_voltage_alphabeta.read().alpha();
    // analog_outputs[3] =
    //     control->get_grid_measurements()
    //         .get_outputs()
    //         .grid_voltage_abc_unmasked.read()
    //         .a();  // 5.0f;  // control->get_grid_measurements().get_outputs().grid_voltage_alphabeta.read().alpha();

    // Baseline constant output test: drive all four channels to +1.0 V (±5 V range)
    // Purpose: Hardware validation with scope / DMM after restoring legacy driver.
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
    outputs->thyristor_pulses.b_minus = firing_pulses.p6;
    outputs->thyristor_pulses.b_plus = firing_pulses.p3;
    outputs->thyristor_pulses.a_minus = firing_pulses.p4;
    outputs->thyristor_pulses.c_plus = firing_pulses.p5;
    outputs->thyristor_pulses.c_minus = firing_pulses.p2;
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
    const auto& io_params = control->get_external_ios().get_parameters();
    if (inputs != nullptr && inputs->digital_inputs.valid) {
        // OR-ing of 3 ACU fan running signals
        const bool di_fans_are_running =
            read_di(inputs, DI_IS_RUNNING_ACU_FAN_1_INDEX, DI_IS_RUNNING_ACU_FAN_1_POLARITY) &&
            read_di(inputs, DI_IS_RUNNING_ACU_FAN_2_INDEX, DI_IS_RUNNING_ACU_FAN_2_POLARITY) &&
            read_di(inputs, DI_IS_RUNNING_ACU_FAN_3_INDEX, DI_IS_RUNNING_ACU_FAN_3_POLARITY);

        // Update supervisory digital inputs from hardware I/O with configurable channel mapping.
        control->external_ios.set_digital_inputs(read_di(inputs, io_params.di_auxiliaries_are_on_index.read()),
            read_di(inputs, io_params.di_mv_breaker_is_closed_index.read()),
            read_di(inputs, io_params.di_start_command_index.read()),
            read_di(inputs, io_params.di_stop_command_index.read()),
            read_di(inputs, io_params.di_mv_breaker_is_tripping_index.read()),
            read_di(inputs, io_params.di_emergency_off_index.read()),
            read_di(inputs, io_params.di_door_is_open_index.read()),
            read_di(inputs, io_params.di_external_fault_index.read()), di_fans_are_running);
    }

    // Process fiber-optic RX data from machine-side controller
    // Fiber-optic link provides cross-coupled signals between grid and machine sides
    bool fiber_comm_valid = false;
    if (inputs->fiber_optic_rx.data_ready && inputs->fiber_optic_rx.link_ok) {
        // Use control class method to decode all signals (ensures consistency with host simulator)
        // This updates external_ios with: measured_speed, current_reference, machine_voltage_magnitude,
        // force_commutation_request, shutdown_request
        fiber_comm_valid = control->decode_fiber_optic_rx_from_machine(
            inputs->fiber_optic_rx.payload, inputs->fiber_optic_rx.payload_length);
    }

    // Feed communication validity into the protections watchdog (trips after configured timeout).
    // This is updated every cycle so loss of updates or link errors are detected.
    bool remote_ok = control->get_external_ios().get_fiber_optics_inputs_from_machine().communication_ok.read();
    control->grid_protections.update_communication_status(fiber_comm_valid, remote_ok);

    // Pass machine-side protection status to grid-side protections
    // When machine trips, grid-side should also trip (severity-configurable)
    bool machine_tripped =
        control->get_external_ios().get_fiber_optics_inputs_from_machine().machine_protection_tripped.read();
    control->grid_protections.update_machine_protection_tripped(machine_tripped);

    // ==================== EXECUTE CONTROL UPDATE ====================
    // Digital input overrides are automatically applied within the state machine lambdas
    // No separate apply_digital_input_overrides() call needed - the lambdas check
    // Execute 1 kHz control update (current controller)
    control->update_1ms();

    // ==================== FIBER-OPTIC TX → MACHINE-SIDE CONTROLLER ====================
    // Prepare fiber-optic TX data to send to machine-side board via I/O interface outputs
    // Transmit grid-side status and cross-coupled signals
    uint8_t payload_words = control->encode_fiber_optic_tx_to_machine();
    outputs->fiber_optic_tx.payload_length = payload_words;

    std::memcpy(
        outputs->fiber_optic_tx.payload, control->get_fiber_optic_tx_payload(), payload_words * sizeof(uint32_t));
    outputs->fiber_optic_tx.transmit_request = true;

    // ==================== DIGITAL OUTPUTS → EXTERNAL COMMANDS ====================
    // Drive mapped breaker commands onto the physical DO array
    write_do(outputs, io_params.do_mv_breaker_close_index.read(), control->external_ios.get_mv_breaker_close());
    write_do(outputs, io_params.do_mv_breaker_open_index.read(), control->external_ios.get_mv_breaker_open());
    // Note: auxiliaries_on_command is available but not currently mapped to hardware DO

    // ACU fan commands
    write_do(outputs, io_params.do_on_command_acu_fan_1_index.read(), control->external_ios.get_on_command_acu_fan_1());
    write_do(outputs, io_params.do_on_command_acu_fan_2_index.read(), control->external_ios.get_on_command_acu_fan_2());
    write_do(outputs, io_params.do_on_command_acu_fan_3_index.read(), control->external_ios.get_on_command_acu_fan_3());
    write_do(outputs, io_params.do_on_command_acu_fan_4_index.read(), control->external_ios.get_on_command_acu_fan_4());

    // Software trip signals
    write_do(outputs, io_params.do_software_trip_1_index.read(), control->external_ios.get_software_trip_1());
    write_do(outputs, io_params.do_software_trip_2_index.read(), control->external_ios.get_software_trip_2());
    write_do(outputs, io_params.do_software_trip_3_index.read(), control->external_ios.get_software_trip_3());

    // Emergency off reset pulse
    write_do(outputs, io_params.do_emergency_off_reset_index.read(), control->external_ios.get_emergency_off_reset());
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

// ==================== Wrapper Functions for Embedded Builds ====================
// For embedded builds, provide the yeet_* function names that main.c expects
// These delegate to the grid_* functions with the static instances

#if defined(YEET_EMBEDDED_GRID_SIDE) && !defined(YEET_BUILD_DUAL_CONTROLLER)

ARM_FUNCTION
void yeet_embedded_init(void)
{
    _initialize_static_instances();
    grid_embedded_init(static_grid_control_ptr);
}

ARM_FUNCTION
void yeet_task_200us(const task_inputs_t* inputs, task_outputs_t* outputs)
{
    _initialize_static_instances();
    grid_task_200us(static_grid_control_ptr, inputs, outputs);
}

ARM_FUNCTION
void yeet_task_1000us(const task_inputs_t* inputs, task_outputs_t* outputs)
{
    _initialize_static_instances();
    grid_task_1000us(static_grid_control_ptr, inputs, outputs);
}

ARM_FUNCTION
void yeet_task_10000us(const task_inputs_t* inputs, task_outputs_t* outputs)
{
    _initialize_static_instances();
    grid_task_10000us(static_grid_control_ptr, inputs, outputs);
}

ARM_FUNCTION
void yeet_network(const network_inputs_t* inputs, network_outputs_t* outputs)
{
    _initialize_static_instances();
    grid_network(static_grid_infrastructure_ptr, inputs, outputs);
}

#endif  // YEET_EMBEDDED_GRID_SIDE && !YEET_BUILD_DUAL_CONTROLLER

}  // extern "C"
