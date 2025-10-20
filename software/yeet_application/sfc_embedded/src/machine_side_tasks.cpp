#include "sfc_embedded/machine_side_tasks.hpp"

#include <cstdint>
#include <cstring>
#include <memory>

#include "sfc/sfc_types.hpp"
#include "sfc_embedded/fiber_optic_protocol.hpp"
#include "sfc_integration/sfc_machine_side_control.hpp"
#include "yeet/signal.hpp"

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

ARM_FUNCTION
void yeet_embedded_init(void)
{
    // ==================== INITIALIZE ANALOG INPUTS ====================
    // Initialize ADC inputs with zero values (will be updated from hardware in 200µs task)
    yeet::mock_signal_input::write(
        get_machine_side_control().get_io().analog_inputs.v_ab_counts, yeet::control::adc_counts{0});
    yeet::mock_signal_input::write(
        get_machine_side_control().get_io().analog_inputs.v_bc_counts, yeet::control::adc_counts{0});

    // ==================== INITIALIZE DIGITAL INPUTS ====================
    // Set safe default values for supervisory controller inputs
    yeet::mock_signal_input::write(get_machine_side_control().get_io().digital_inputs.exciter_ready, false);
    yeet::mock_signal_input::write(get_machine_side_control().get_io().digital_inputs.excitation_start_command, false);
    yeet::mock_signal_input::write(get_machine_side_control().get_io().digital_inputs.excitation_stop_command, false);
    yeet::mock_signal_input::write(get_machine_side_control().get_io().digital_inputs.run_creep_speed, false);
    yeet::mock_signal_input::write(get_machine_side_control().get_io().digital_inputs.run_purge_accel, false);
    yeet::mock_signal_input::write(get_machine_side_control().get_io().digital_inputs.run_ignition_accel, false);
    yeet::mock_signal_input::write(get_machine_side_control().get_io().digital_inputs.run_assist_accel, false);
    yeet::mock_signal_input::write(get_machine_side_control().get_io().digital_inputs.hold_speed, false);
    yeet::mock_signal_input::write(get_machine_side_control().get_io().digital_inputs.decelerate, false);
    yeet::mock_signal_input::write(get_machine_side_control().get_io().digital_inputs.idle, true);

    // ==================== INITIALIZE GRID-SIDE INPUTS ====================
    // Initialize cross-coupled inputs from grid-side controller with safe default values
    // In production, these come from fiber-optic communication interface
    yeet::mock_signal_input::write(
        get_machine_side_control().get_io().grid_side_inputs.dc_link_current_measured, yeet::current_pu_t{0.0});
    yeet::mock_signal_input::write(get_machine_side_control().get_io().grid_side_inputs.operating_mode,
        yeet::application::sfc::sfc_operating_mode_t::natural_commutation);
    yeet::mock_signal_input::write(
        get_machine_side_control().get_io().grid_side_inputs.force_commutation_acknowledged, false);
    yeet::mock_signal_input::write(get_machine_side_control().get_io().grid_side_inputs.freeze_integrator, false);
    yeet::mock_signal_input::write(
        get_machine_side_control().get_io().grid_side_inputs.current_controller_saturated, false);
    yeet::mock_signal_input::write(get_machine_side_control().get_io().grid_side_inputs.pulse_releases, false);
    yeet::mock_signal_input::write(get_machine_side_control().get_io().grid_side_inputs.current_state_id,
        yeet::application::sfc::sfc_state_id::IDLE);

    // Initialize machine-side control subsystem
    get_machine_side_control().initialize();
}

ARM_FUNCTION
void yeet_task_200us(const task_inputs_t* inputs, task_outputs_t* outputs)
{
    // HIGH-FREQUENCY CONTROL TASK
    // Configured rate: 200 µs (5 kHz)
    // Execution time budget: < 100 µs (50% of period)
    // Called from: TIM2_IRQHandler
    // Priority: Highest (measurements, observer, modulator, pulse generator)

    counter_task1++;

    // ==================== ANALOG INPUTS → MEASUREMENTS ====================
    // Map hardware ADC channels to machine-side measurements via I/O interface
    // Based on sfc_io_definitions.md:
    //   AI1 (X1-1): Machine voltage Vab
    //   AI2 (X1-2): Machine voltage Vbc

    // Write ADC values directly to I/O interface analog inputs
    if (inputs != nullptr && inputs->analog_inputs.valid) {
        yeet::mock_signal_input::write(get_machine_side_control().get_io().analog_inputs.v_ab_counts,
            yeet::control::adc_counts{static_cast<int16_t>(inputs->analog_inputs.channels[0])});
        yeet::mock_signal_input::write(get_machine_side_control().get_io().analog_inputs.v_bc_counts,
            yeet::control::adc_counts{static_cast<int16_t>(inputs->analog_inputs.channels[1])});
    }

    // Execute the 5 kHz control update (measurements → observers → modulators → pulse generator)
    get_machine_side_control().update_200us();

    // ==================== THYRISTOR PULSES → PWM OUTPUTS ====================
    // Extract firing pulses from machine-side I/O interface and map to hardware PWM channels
    if (outputs != nullptr) {
        const auto& firing_pulses = get_machine_side_control().get_io().pulse_outputs.firing_pulses.read();

        outputs->thyristor_pulses.a_plus = firing_pulses.p1;   // T1 (A+) → p1
        outputs->thyristor_pulses.b_minus = firing_pulses.p2;  // T2 (B-) → p2
        outputs->thyristor_pulses.b_plus = firing_pulses.p3;   // T3 (B+) → p3
        outputs->thyristor_pulses.a_minus = firing_pulses.p4;  // T4 (A-) → p4
        outputs->thyristor_pulses.c_plus = firing_pulses.p5;   // T5 (C+) → p5
        outputs->thyristor_pulses.c_minus = firing_pulses.p6;  // T6 (C-) → p6
    }
}

ARM_FUNCTION
void yeet_task_1000us(const task_inputs_t* inputs, task_outputs_t* outputs)
{
    // MEDIUM-FREQUENCY CONTROL TASK
    // Configured rate: 1 ms (1 kHz)
    // Execution time budget: < 500 µs (50% of period)
    // Called from: TIM3_IRQHandler
    // Priority: Medium (machine controller)

    counter_task2++;

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
        yeet::mock_signal_input::write(get_machine_side_control().get_io().digital_inputs.exciter_ready, exciter_ready);
        yeet::mock_signal_input::write(
            get_machine_side_control().get_io().digital_inputs.excitation_start_command, excitation_start);
        yeet::mock_signal_input::write(
            get_machine_side_control().get_io().digital_inputs.excitation_stop_command, excitation_stop);
        yeet::mock_signal_input::write(get_machine_side_control().get_io().digital_inputs.run_creep_speed, run_creep);
        yeet::mock_signal_input::write(get_machine_side_control().get_io().digital_inputs.run_purge_accel, run_purge);
        yeet::mock_signal_input::write(
            get_machine_side_control().get_io().digital_inputs.run_ignition_accel, run_ignition);
        yeet::mock_signal_input::write(get_machine_side_control().get_io().digital_inputs.run_assist_accel, run_assist);
        yeet::mock_signal_input::write(get_machine_side_control().get_io().digital_inputs.hold_speed, hold_speed_cmd);
        yeet::mock_signal_input::write(get_machine_side_control().get_io().digital_inputs.decelerate, decelerate_cmd);
        yeet::mock_signal_input::write(get_machine_side_control().get_io().digital_inputs.idle, idle_cmd);
    }

    // Process fiber-optic RX data from grid-side controller
    // Fiber-optic link provides cross-coupled signals between grid and machine sides
    if (inputs != nullptr && inputs->fiber_optic_rx.data_ready && inputs->fiber_optic_rx.link_ok) {
        // Decode Grid → Machine protocol message
        if (auto rx_msg = yeet::application::sfc::fiber_optic::decode_grid_to_machine(
                inputs->fiber_optic_rx.payload, inputs->fiber_optic_rx.payload_length)) {
            // Map fiber-optic protocol enum to SFC operating mode
            yeet::application::sfc::sfc_operating_mode_t operating_mode;
            if (rx_msg->mode == yeet::application::sfc::fiber_optic::operating_mode::forced_commutation) {
                operating_mode = yeet::application::sfc::sfc_operating_mode_t::forced_commutation;
            }
            else if (rx_msg->mode == yeet::application::sfc::fiber_optic::operating_mode::natural_commutation) {
                operating_mode = yeet::application::sfc::sfc_operating_mode_t::natural_commutation;
            }
            else {
                operating_mode = yeet::application::sfc::sfc_operating_mode_t::forced_commutation;  // Safe default
            }

            // Write grid-side cross-coupled signals directly to I/O interface
            yeet::mock_signal_input::write(
                get_machine_side_control().get_io().grid_side_inputs.operating_mode, operating_mode);
            yeet::mock_signal_input::write(
                get_machine_side_control().get_io().grid_side_inputs.force_commutation_acknowledged,
                rx_msg->force_commutation_acknowledged);
            yeet::mock_signal_input::write(
                get_machine_side_control().get_io().grid_side_inputs.pulse_releases, rx_msg->pulse_releases);

            // Note: is_current_zero decoded but not currently used by machine-side
        }
    }

    // Execute 1 kHz control update (machine controller)
    get_machine_side_control().update_1ms();

    // ==================== FIBER-OPTIC TX → GRID-SIDE CONTROLLER ====================
    // Prepare fiber-optic TX data to send to grid-side board via I/O interface outputs
    // Transmit machine-side status and cross-coupled signals
    if (outputs != nullptr) {
        // Prepare Machine → Grid protocol message
        yeet::application::sfc::fiber_optic::machine_to_grid_message tx_msg{
            .measured_speed_pu =
                static_cast<float>(get_machine_side_control().get_io().grid_side_outputs.measured_speed_pu.read()),
            .dc_link_current_reference_pu =
                static_cast<float>(get_machine_side_control().get_io().grid_side_outputs.current_reference.read()),
            .machine_voltage_magnitude_pu = static_cast<float>(
                get_machine_side_control().get_io().grid_side_outputs.machine_voltage_magnitude.read()),
            .force_commutation_request =
                get_machine_side_control().get_io().grid_side_outputs.request_force_commutation.read()};

        // Encode and transmit
        outputs->fiber_optic_tx.payload_length =
            yeet::application::sfc::fiber_optic::encode_machine_to_grid(tx_msg, outputs->fiber_optic_tx.payload);
        outputs->fiber_optic_tx.transmit_request = true;
    }

    // ==================== DIGITAL OUTPUTS → STATUS INDICATORS ====================
    // Map control system status to hardware digital outputs via I/O interface
    if (outputs != nullptr) {
        // TODO: Add digital output mappings as needed for machine-side status
        // Example: Speed controller active, at setpoint, faults, etc.
        outputs->digital_outputs.signals[0] = inputs->fiber_optic_rx.link_ok;  // DO1: Fiber link status
    }
}

ARM_FUNCTION
void yeet_task_10000us(const task_inputs_t* inputs, task_outputs_t* outputs)
{
    // LOW-FREQUENCY CONTROL TASK
    // Configured rate: 10 ms (100 Hz)
    // Execution time budget: < 5 ms (50% of period)
    // Called from: TIM4_IRQHandler
    // Priority: Lowest (speed controller)

    counter_task3++;

    // Optional: Use inputs for monitoring/diagnostics
    (void)inputs;  // Suppress unused parameter warning

    get_machine_side_control().update_10ms();

    // Optional: Update diagnostic outputs
    (void)outputs;  // Suppress unused parameter warning
}

ARM_FUNCTION
void yeet_network(const network_inputs_t* inputs, network_outputs_t* outputs)
{
    // NON-REAL-TIME NETWORK COMMUNICATION TASK
    // Called from: Main loop (not time-critical)
    // Purpose: Handle TCP/IP and UDP communication for:
    //   - HMI/SCADA supervisory control (speed reference, start/stop)
    //   - Configuration updates
    //   - Diagnostic data requests
    //   - Parameter tuning
    //   - Event logging and telemetry

    // Process incoming network data
    if (inputs != nullptr && inputs->network_rx.data_ready) {
        // TODO: Parse received network packet
        // Examples for machine side:
        //   - Speed reference commands from HMI
        //   - Start/stop/emergency stop commands
        //   - Configuration updates (PI gains, limits)
        //   - Diagnostic requests
        //
        // const uint8_t* rx_data = inputs->network_rx.rx_buffer;
        // uint16_t rx_len = inputs->network_rx.rx_length;
        // bool connected = inputs->network_rx.tcp_connected;
        //
        // if (rx_len > 0) {
        //     parse_supervisory_command(rx_data, rx_len);
        //     // Update speed reference, control mode, etc.
        // }
    }

    // Prepare network response or periodic telemetry
    if (outputs != nullptr) {
        // TODO: Encode status/diagnostic data for transmission
        // Examples for machine side:
        //   - Current speed, torque, and motor status
        //   - Response to configuration command
        //   - Periodic telemetry data
        //   - Fault/event notifications
        //
        // uint8_t* tx_data = outputs->network_tx.tx_buffer;
        // outputs->network_tx.tx_length = encode_machine_status(tx_data);
        // outputs->network_tx.transmit_request = true;
        // outputs->network_tx.socket_id = 0;

        // Default: No transmission
        outputs->network_tx.transmit_request = false;
        outputs->network_tx.tx_length = 0;
    }
}

}  // extern "C"