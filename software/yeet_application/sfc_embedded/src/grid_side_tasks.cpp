#include "sfc_embedded/grid_side_tasks.hpp"

#include <cstdint>
#include <memory>

#include "sfc/sfc_types.hpp"
#include "sfc_integration/sfc_grid_side_control.hpp"

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

static inline yeet::application::sfc::integration::sfc_grid_side_control& get_grid_side_control()
{
    static auto per_unit_system = create_per_unit_system();
    static yeet::application::sfc::integration::sfc_grid_side_control grid_side_control(per_unit_system);
    return grid_side_control;
}

// Execution counters for diagnostics
static volatile std::uint32_t counter_task1 = 0;
static volatile std::uint32_t counter_task2 = 0;
static volatile std::uint32_t counter_task3 = 0;

// ==================== MOCK MACHINE-SIDE CONTROLLER OUTPUTS ====================
// These signal_outputs simulate the outputs from the machine-side controller
// In production, these would be received via communication interface

// Operating mode from machine-side mode monitor
static yeet::signal_output<yeet::speed_pu_t> mock_measured_speed_pu;

// Current reference from machine-side speed controller
static yeet::signal_output<yeet::current_pu_t> mock_dc_link_current_reference;

// Machine voltage magnitude from machine-side observer
static yeet::signal_output<yeet::voltage_pu_t> mock_machine_voltage_magnitude;

// Force commutation request from machine-side modulator
static yeet::signal_output<bool> mock_request_force_commutation;

// Force commutation acknowledged from machine-side modulator
static yeet::signal_output<bool> mock_force_commutation_acknowledged;

// Current controller saturation status from machine-side
static yeet::signal_output<bool> mock_current_controller_saturated;

// ==================== MOCK GRID-SIDE MEASUREMENT RAW INPUTS ====================
// These simulate hardware ADC + timing/angle sources which are not present in the
// embedded-only build (no plant / hardware mock). They feed the grid_side_measurements runner.
static yeet::signal_output<yeet::control::adc_counts> mock_v_ab_counts;  // line voltage AB differential sample
static yeet::signal_output<yeet::control::adc_counts> mock_v_bc_counts;  // line voltage BC differential sample
static yeet::signal_output<yeet::control::adc_counts> mock_i_a_counts;   // phase A current sample
static yeet::signal_output<yeet::control::adc_counts> mock_i_c_counts;   // phase C current sample
static yeet::signal_output<yeet::frequency_t> mock_grid_side_frequency;  // fundamental grid frequency
static yeet::signal_output<yeet::angle_wrapped> mock_grid_side_angle;    // electrical angle reference
static yeet::signal_output<yeet::control::six_pulse_generator::pulses> mock_grid_pulses;  // commutation events

ARM_FUNCTION
void yeet_embedded_init(void)
{
    // ==================== MOCK SUPERVISORY CONTROLLER SIGNALS ====================
    // State machine command inputs - Safe default values
    get_grid_side_control().auxiliaries_are_on.write(false);
    get_grid_side_control().breakers_are_on.write(false);
    get_grid_side_control().start_command.write(false);
    get_grid_side_control().stop_command.write(false);
    get_grid_side_control().shutdown_request.write(false);
    get_grid_side_control().is_current_extinguished.write(true);

    // ==================== MOCK MACHINE-SIDE CONTROLLER SIGNALS ====================
    // Initialize mock outputs with safe default values
    mock_measured_speed_pu.write(yeet::speed_pu_t{0.0});
    mock_dc_link_current_reference.write(yeet::current_pu_t{0.0});
    mock_machine_voltage_magnitude.write(yeet::voltage_pu_t{0.0});
    mock_request_force_commutation.write(false);

    // ==================== INITIALIZE RAW MEASUREMENT MOCKS ====================
    mock_v_ab_counts.write(yeet::control::adc_counts{0});
    mock_v_bc_counts.write(yeet::control::adc_counts{0});
    mock_i_a_counts.write(yeet::control::adc_counts{0});
    mock_i_c_counts.write(yeet::control::adc_counts{0});
    // Choose nominal frequency to keep any internal observers stable (50 Hz base)
    mock_grid_side_frequency.write(yeet::frequency_t{50.0});
    mock_grid_side_angle.write(yeet::angle_wrapped{0.0});
    // All pulses false (no thyristor conduction yet)
    mock_grid_pulses.write(yeet::control::six_pulse_generator::pulses{});

    // Connect mock machine-side outputs to grid-side control inputs
    yeet::signals_connector::connect_output_to_input(
        mock_measured_speed_pu, get_grid_side_control().operation_mode_monitor.get_inputs().measured_speed_pu);

    yeet::signals_connector::connect_output_to_input(mock_dc_link_current_reference,
        get_grid_side_control().current_controller.get_inputs().dc_link_current_reference);

    yeet::signals_connector::connect_output_to_input(mock_machine_voltage_magnitude,
        get_grid_side_control().current_controller.get_inputs().machine_side_voltage_magnitude);

    yeet::signals_connector::connect_output_to_input(mock_request_force_commutation,
        get_grid_side_control().grid_side_modulator.get_inputs().request_force_commutation);

    // ==================== CONNECT RAW MEASUREMENT MOCKS ====================
    // Feed ADC-based measurement runner inputs
    yeet::signals_connector::connect_output_to_input(
        mock_v_ab_counts, get_grid_side_control().grid_measurements.get_inputs().v_ab_counts);
    yeet::signals_connector::connect_output_to_input(
        mock_v_bc_counts, get_grid_side_control().grid_measurements.get_inputs().v_bc_counts);
    yeet::signals_connector::connect_output_to_input(
        mock_i_a_counts, get_grid_side_control().grid_measurements.get_inputs().i_a_counts);
    yeet::signals_connector::connect_output_to_input(
        mock_i_c_counts, get_grid_side_control().grid_measurements.get_inputs().i_c_counts);
    yeet::signals_connector::connect_output_to_input(
        mock_grid_pulses, get_grid_side_control().grid_measurements.get_inputs().grid_pulses);
    yeet::signals_connector::connect_output_to_input(
        mock_grid_side_frequency, get_grid_side_control().grid_measurements.get_inputs().grid_side_frequency);
    yeet::signals_connector::connect_output_to_input(
        mock_grid_side_angle, get_grid_side_control().grid_measurements.get_inputs().grid_side_angle);

    // ==================== DIRECT MOCKING OF UNCONNECTED CONTROLLER FREEZE INPUTS ====================
    // Ensure deterministic start: explicitly force all freeze inputs false prior to state machine activity.
    yeet::mock_signal_input::write(get_grid_side_control().current_controller.get_inputs().freeze_integrator, false);
    yeet::mock_signal_input::write(
        get_grid_side_control().current_controller.get_inputs().modulator_freeze_integrator, false);

    // Initialize grid - side control subsystem
    get_grid_side_control().initialize();
}

ARM_FUNCTION
void yeet_task_1(const task1_inputs_t* inputs, task1_outputs_t* outputs)
{
    // HIGH-FREQUENCY CONTROL TASK
    // Configured rate: 200 µs (5 kHz)
    // Execution time budget: < 100 µs (50% of period)
    // Called from: TIM2_IRQHandler
    // Priority: Highest (innermost current control loop)

    counter_task1++;

    // Update mock ADC inputs with real hardware measurements
    // In embedded operation, these would come from actual ADC DMA buffers
    if (inputs != nullptr) {
        mock_i_a_counts.write(yeet::control::adc_counts{inputs->adc_channels[10]});
        mock_i_c_counts.write(yeet::control::adc_counts{inputs->adc_channels[11]});
        mock_v_ab_counts.write(yeet::control::adc_counts{inputs->adc_channels[12]});
        mock_v_bc_counts.write(yeet::control::adc_counts{inputs->adc_channels[13]});
    }

    // Execute the 5 kHz control update
    get_grid_side_control().update_200us();

    // Extract gate pulse outputs from the modulator
    // In production, these would be applied to thyristor gate driver hardware
    if (outputs != nullptr) {
        // For now, initialize all pulses to false (safe default)
        // TODO: Read actual pulse commands from grid_side_modulator output when available
        outputs->pulse_1 = false;
        outputs->pulse_2 = false;
        outputs->pulse_3 = false;
        outputs->pulse_4 = false;
        outputs->pulse_5 = false;
        outputs->pulse_6 = false;
    }
}

ARM_FUNCTION
void yeet_task_2(void)
{
    // MEDIUM-FREQUENCY CONTROL TASK
    // Configured rate: 1 ms (1 kHz)
    // Execution time budget: < 500 µs (50% of period)
    // Called from: TIM3_IRQHandler
    // Priority: Medium (outer control loops)

    counter_task2++;
    get_grid_side_control().update_1ms();
}

ARM_FUNCTION
void yeet_task_3(void)
{
    // LOW-FREQUENCY MONITORING TASK
    // Configured rate: 10 ms (100 Hz)
    // Execution time budget: < 5 ms (50% of period)
    // Called from: TIM4_IRQHandler
    // Priority: Lowest (monitoring and diagnostics)

    counter_task3++;
    get_grid_side_control().update_10ms();
}

}  // extern "C"