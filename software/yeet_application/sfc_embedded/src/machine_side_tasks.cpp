#include "sfc_embedded/machine_side_tasks.hpp"

#include <cstdint>
#include <memory>

#include "sfc/sfc_types.hpp"
#include "sfc_integration/sfc_machine_side_control.hpp"

// Ensure functions are compiled with Thumb interworking
#ifdef __arm__
#define ARM_FUNCTION __attribute__((target("thumb")))
#else
#define ARM_FUNCTION
#endif

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

static inline yeet::application::sfc::integration::sfc_machine_side_control& get_machine_side_control()
{
    static auto per_unit_system = create_per_unit_system();
    static yeet::application::sfc::integration::sfc_machine_side_control machine_side_control(per_unit_system);
    return machine_side_control;
}

// Execution counters for diagnostics
static volatile std::uint32_t counter_task1 = 0;
static volatile std::uint32_t counter_task2 = 0;
static volatile std::uint32_t counter_task3 = 0;

// ==================== MOCK GRID-SIDE CONTROLLER OUTPUTS ====================
// These signal_outputs simulate the outputs from the grid-side controller
// In production, these would be received via communication interface

// Pulse release enable from grid-side state machine
static yeet::signal_output<bool> mock_pulse_releases;

// Operating mode from grid-side operation mode monitor
static yeet::signal_output<yeet::application::sfc::sfc_operating_mode_t> mock_operating_mode;

// DC link current from grid-side measurements
static yeet::signal_output<yeet::current_pu_t> mock_dc_link_current_pu;

// Force commutation acknowledgment from grid-side modulator
static yeet::signal_output<bool> mock_force_commutation_acknowledged;

// Freeze integrator from grid-side state machine (for speed controller anti-windup)
static yeet::signal_output<bool> mock_freeze_integrator;

// Current controller saturation from grid-side (for speed controller anti-windup)
static yeet::signal_output<bool> mock_current_controller_is_saturated;

// Modulator freeze signal from grid-side modulator
static yeet::signal_output<bool> mock_modulator_freeze_integrator;

// Enable speed control from grid-side state machine
static yeet::signal_output<bool> mock_enable_speed_control;

// ==================== MOCK MACHINE-SIDE MEASUREMENT RAW INPUTS ====================
// These simulate hardware ADC + timing/angle sources for machine-side measurements

static yeet::signal_output<yeet::control::adc_counts> mock_v_ab_counts;     // machine voltage AB
static yeet::signal_output<yeet::control::adc_counts> mock_v_bc_counts;     // machine voltage BC
static yeet::signal_output<yeet::frequency_t> mock_machine_side_frequency;  // machine frequency
static yeet::signal_output<yeet::angle_wrapped> mock_machine_side_angle;    // machine electrical angle
static yeet::signal_output<yeet::control::six_pulse_generator::pulses>
    mock_machine_pulses;  // machine commutation events

ARM_FUNCTION
void yeet_embedded_init(void)
{
    // ==================== MOCK EXTERNAL COMMAND SIGNALS ====================
    // Speed reference from supervisory controller
    get_machine_side_control().speed_reference_pu.write(yeet::speed_pu_t{0.0});

    // ==================== MOCK GRID-SIDE CONTROLLER SIGNALS ====================
    // Initialize with safe default values
    mock_pulse_releases.write(false);
    mock_operating_mode.write(yeet::application::sfc::sfc_operating_mode_t::natural_commutation);
    mock_dc_link_current_pu.write(yeet::current_pu_t{0.0});
    mock_force_commutation_acknowledged.write(false);
    mock_freeze_integrator.write(false);
    mock_current_controller_is_saturated.write(false);
    mock_modulator_freeze_integrator.write(false);
    mock_enable_speed_control.write(false);

    // ==================== INITIALIZE RAW MEASUREMENT MOCKS ====================
    mock_v_ab_counts.write(yeet::control::adc_counts{0});
    mock_v_bc_counts.write(yeet::control::adc_counts{0});
    mock_machine_side_frequency.write(yeet::frequency_t{50.0});  // nominal frequency
    mock_machine_side_angle.write(yeet::angle_wrapped{0.0});
    mock_machine_pulses.write(yeet::control::six_pulse_generator::pulses{});

    // ==================== CONNECT GRID-SIDE MOCK OUTPUTS TO MACHINE-SIDE INPUTS ====================

    // Connect pulse releases to modulator enable
    yeet::signals_connector::connect_output_to_input(
        mock_pulse_releases, get_machine_side_control().machine_side_modulator.get_inputs().enable);

    // Connect operating mode to modulator
    yeet::signals_connector::connect_output_to_input(
        mock_operating_mode, get_machine_side_control().machine_side_modulator.get_inputs().operating_mode);

    // Connect DC link current to machine controller
    yeet::signals_connector::connect_output_to_input(
        mock_dc_link_current_pu, get_machine_side_control().machine_side_controller.get_inputs().dc_link_current);

    // Connect force commutation acknowledgment to modulator
    yeet::signals_connector::connect_output_to_input(mock_force_commutation_acknowledged,
        get_machine_side_control().machine_side_modulator.get_inputs().force_commutation_acknowledged);

    // Connect freeze integrator signals to speed controller
    yeet::signals_connector::connect_output_to_input(
        mock_freeze_integrator, get_machine_side_control().speed_controller.get_inputs().freeze_integrator);

    yeet::signals_connector::connect_output_to_input(mock_current_controller_is_saturated,
        get_machine_side_control().speed_controller.get_inputs().current_controller_is_saturated);

    yeet::signals_connector::connect_output_to_input(mock_modulator_freeze_integrator,
        get_machine_side_control().speed_controller.get_inputs().modulator_freeze_integrator);

    // Connect enable speed control
    yeet::signals_connector::connect_output_to_input(
        mock_enable_speed_control, get_machine_side_control().speed_controller.get_inputs().enable_speed_control);

    // ==================== CONNECT RAW MEASUREMENT MOCKS ====================

    // Connect voltage measurements
    yeet::signals_connector::connect_output_to_input(
        mock_v_ab_counts, get_machine_side_control().machine_measurements.get_inputs().v_ab_counts);

    yeet::signals_connector::connect_output_to_input(
        mock_v_bc_counts, get_machine_side_control().machine_measurements.get_inputs().v_bc_counts);

    // Connect machine pulses for measurement synchronization
    yeet::signals_connector::connect_output_to_input(
        mock_machine_pulses, get_machine_side_control().machine_measurements.get_inputs().machine_pulses);

    // Connect frequency and angle references
    yeet::signals_connector::connect_output_to_input(mock_machine_side_frequency,
        get_machine_side_control().machine_measurements.get_inputs().machine_side_frequency);

    yeet::signals_connector::connect_output_to_input(
        mock_machine_side_angle, get_machine_side_control().machine_measurements.get_inputs().machine_side_angle);

    // Initialize machine-side control subsystem
    get_machine_side_control().initialize();
}

ARM_FUNCTION
void yeet_task_200us(const task1_inputs_t* inputs, task1_outputs_t* outputs)
{
    // HIGH-FREQUENCY CONTROL TASK
    // Configured rate: 200 µs (5 kHz)
    // Execution time budget: < 100 µs (50% of period)
    // Called from: TIM2_IRQHandler
    // Priority: Highest (measurements, observer, modulator, pulse generator)

    counter_task1++;

    // Update mock ADC inputs with real hardware measurements
    // In embedded operation, these would come from actual ADC DMA buffers
    if (inputs != nullptr) {
        // TODO: Map ADC channels to machine-side measurements when configuration is finalized
        // For now, keep existing mock behavior
    }

    // Execute the 5 kHz control update
    get_machine_side_control().update_200us();

    // Extract gate pulse outputs from the modulator
    // In production, these would be applied to thyristor gate driver hardware
    if (outputs != nullptr) {
        // For now, initialize all pulses to false (safe default)
        // TODO: Read actual pulse commands from machine_side_modulator output when available
        outputs->pulse_1 = false;
        outputs->pulse_2 = false;
        outputs->pulse_3 = false;
        outputs->pulse_4 = false;
        outputs->pulse_5 = false;
        outputs->pulse_6 = false;
    }
}

ARM_FUNCTION
void yeet_task_1000us(void)
{
    // MEDIUM-FREQUENCY CONTROL TASK
    // Configured rate: 1 ms (1 kHz)
    // Execution time budget: < 500 µs (50% of period)
    // Called from: TIM3_IRQHandler
    // Priority: Medium (machine controller)

    counter_task2++;
    get_machine_side_control().update_1ms();
}

ARM_FUNCTION
void yeet_task_10000us(void)
{
    // LOW-FREQUENCY CONTROL TASK
    // Configured rate: 10 ms (100 Hz)
    // Execution time budget: < 5 ms (50% of period)
    // Called from: TIM4_IRQHandler
    // Priority: Lowest (speed controller)

    counter_task3++;
    get_machine_side_control().update_10ms();
}

}  // extern "C"