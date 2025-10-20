#include "sfc_embedded/grid_side_tasks.hpp"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>

#include "sfc/sfc_types.hpp"
#include "sfc_integration/sfc_grid_side_control.hpp"
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

static inline yeet::application::sfc::integration::sfc_grid_side_control& get_grid_side_control()
{
    static auto per_unit_system = create_per_unit_system();
    static yeet::application::sfc::integration::sfc_grid_side_control grid_side_control(per_unit_system);
    return grid_side_control;
}

// ==================== NETWORK STREAMING STATE ====================
// State machine for TCP streaming data protocol
struct network_streaming_state {
    bool is_streaming = false;                           // Streaming active flag
    float test_signal_amplitude = 1.0f;                  // Configurable amplitude
    float noise_level = 0.1f;                            // Configurable noise level
    uint32_t sample_count = 0;                           // Sample counter for signal generation
    uint32_t tx_packet_count = 0;                        // Transmitted packet counter
    static constexpr uint16_t SAMPLES_PER_PACKET = 361;  // Max floats that fit in 1460 byte packet
};

static network_streaming_state net_stream_state;

ARM_FUNCTION
void yeet_embedded_init(void)
{
    // ==================== INITIALIZE DIGITAL INPUTS ====================
    // Set safe default values for supervisory controller inputs
    yeet::mock_signal_input::write(get_grid_side_control().get_io().digital_inputs.auxiliaries_are_on, false);
    yeet::mock_signal_input::write(get_grid_side_control().get_io().digital_inputs.breakers_are_on, false);
    yeet::mock_signal_input::write(get_grid_side_control().get_io().digital_inputs.start_command, false);
    yeet::mock_signal_input::write(get_grid_side_control().get_io().digital_inputs.stop_command, false);
    yeet::mock_signal_input::write(get_grid_side_control().get_io().digital_inputs.shutdown_request, false);
    yeet::mock_signal_input::write(get_grid_side_control().get_io().digital_inputs.is_current_extinguished, true);

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

    // Initialize grid-side control subsystem
    // This will also connect internal signals (observer → measurements for angle/frequency/pulses)
    get_grid_side_control().initialize();
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
        yeet::control::adc_counts{static_cast<int16_t>(inputs->analog_inputs.channels[0])});
    yeet::mock_signal_input::write(get_grid_side_control().get_io().analog_inputs.v_bc_counts,
        yeet::control::adc_counts{static_cast<int16_t>(inputs->analog_inputs.channels[1])});
    yeet::mock_signal_input::write(get_grid_side_control().get_io().analog_inputs.i_a_counts,
        yeet::control::adc_counts{static_cast<int16_t>(inputs->analog_inputs.channels[4])});
    yeet::mock_signal_input::write(get_grid_side_control().get_io().analog_inputs.i_c_counts,
        yeet::control::adc_counts{static_cast<int16_t>(inputs->analog_inputs.channels[6])});

    // Execute the 5 kHz control update (measurements → observers → modulators → pulse generator)
    get_grid_side_control().update_200us();

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

        // Write digital input values directly to I/O interface
        yeet::mock_signal_input::write(
            get_grid_side_control().get_io().digital_inputs.auxiliaries_are_on, auxiliaries_on);
        yeet::mock_signal_input::write(get_grid_side_control().get_io().digital_inputs.breakers_are_on, breakers_on);
        yeet::mock_signal_input::write(get_grid_side_control().get_io().digital_inputs.start_command, start_cmd);
        yeet::mock_signal_input::write(get_grid_side_control().get_io().digital_inputs.stop_command, stop_cmd);
        yeet::mock_signal_input::write(get_grid_side_control().get_io().digital_inputs.shutdown_request, shutdown_req);
        yeet::mock_signal_input::write(
            get_grid_side_control().get_io().digital_inputs.is_current_extinguished, current_extinguished);
    }

    // Process fiber-optic RX data from machine-side controller
    // Fiber-optic link provides cross-coupled signals between grid and machine sides
    if (inputs != nullptr && inputs->fiber_optic_rx.data_ready && inputs->fiber_optic_rx.link_ok) {
        // Expected payload format from machine-side board (uint32_t words):
        //   payload[0]: Measured speed [pu] (as float bits)
        //   payload[1]: DC link current reference [pu] (as float bits)
        //   payload[2]: Machine voltage magnitude [pu] (as float bits)
        //   payload[3]: Force commutation request [boolean as uint32_t: 0 or 1]

        if (inputs->fiber_optic_rx.payload_length >= 4) {
            // Convert uint32_t payload to float using type punning
            float speed_pu_value;
            float current_ref_value;
            float voltage_mag_value;
            std::memcpy(&speed_pu_value, &inputs->fiber_optic_rx.payload[0], sizeof(float));
            std::memcpy(&current_ref_value, &inputs->fiber_optic_rx.payload[1], sizeof(float));
            std::memcpy(&voltage_mag_value, &inputs->fiber_optic_rx.payload[2], sizeof(float));

            // Write machine-side cross-coupled signals directly to I/O interface
            yeet::mock_signal_input::write(get_grid_side_control().get_io().machine_side_inputs.measured_speed_pu,
                yeet::speed_pu_t{speed_pu_value});
            yeet::mock_signal_input::write(
                get_grid_side_control().get_io().machine_side_inputs.dc_link_current_reference,
                yeet::current_pu_t{current_ref_value});
            yeet::mock_signal_input::write(
                get_grid_side_control().get_io().machine_side_inputs.machine_voltage_magnitude,
                yeet::voltage_pu_t{voltage_mag_value});
            yeet::mock_signal_input::write(
                get_grid_side_control().get_io().machine_side_inputs.request_force_commutation,
                inputs->fiber_optic_rx.payload[3] != 0);
        }
    }

    // Execute 1 kHz control update (current controller)
    get_grid_side_control().update_1ms();

    // ==================== FIBER-OPTIC TX → MACHINE-SIDE CONTROLLER ====================
    // Prepare fiber-optic TX data to send to machine-side board via I/O interface outputs
    // Transmit grid-side status and cross-coupled signals
    if (outputs != nullptr) {
        // Payload format to machine-side (uint32_t words):
        //   payload[0]: Operating mode [enumeration as uint32_t]
        //   payload[1]: Is current zero [boolean as uint32_t: 0 or 1]
        //   payload[2]: Force commutation acknowledged [boolean as uint32_t: 0 or 1]
        //   payload[3]: Pulse releases [boolean as uint32_t: 0 or 1]

        // Read from I/O interface machine-side outputs
        const auto operating_mode = get_grid_side_control().get_io().machine_side_outputs.operating_mode.read();
        const bool is_current_zero = get_grid_side_control().get_io().machine_side_outputs.is_current_zero.read();
        const bool force_comm_ack =
            get_grid_side_control().get_io().machine_side_outputs.force_commutation_acknowledged.read();
        const bool pulse_releases = get_grid_side_control().get_io().digital_outputs.pulse_releases.read();

        // Encode as uint32_t values
        outputs->fiber_optic_tx.payload[0] = static_cast<uint32_t>(static_cast<int>(operating_mode));
        outputs->fiber_optic_tx.payload[1] = is_current_zero ? 1U : 0U;
        outputs->fiber_optic_tx.payload[2] = force_comm_ack ? 1U : 0U;
        outputs->fiber_optic_tx.payload[3] = pulse_releases ? 1U : 0U;
        outputs->fiber_optic_tx.payload_length = 4;
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
        outputs->digital_outputs.signals[0] = pulse_releases;     // DO1: System running
        outputs->digital_outputs.signals[1] = is_current_zero;    // DO2: Current zero status
        outputs->digital_outputs.signals[2] = current_saturated;  // DO3: Controller saturation
        outputs->digital_outputs.signals[3] =
            (operating_mode ==
                yeet::application::sfc::sfc_operating_mode_t::forced_commutation);  // DO4: Forced commutation mode
        outputs->digital_outputs.signals[4] =
            (operating_mode ==
                yeet::application::sfc::sfc_operating_mode_t::natural_commutation);  // DO5: Natural commutation mode
        outputs->digital_outputs.signals[5] = inputs->fiber_optic_rx.link_ok;        // DO6: Fiber link status

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
    //   - Configuration updates
    //   - Diagnostic data requests
    //   - Parameter tuning
    //   - Event logging and telemetry

    // ==================== COMMAND PROTOCOL ====================
    // Simple text-based protocol:
    //   "START\n"  - Begin streaming data
    //   "STOP\n"   - Stop streaming data
    //   "AMP:X.X\n" - Set signal amplitude (e.g., "AMP:2.5\n")
    //   "NOISE:X.X\n" - Set noise level (e.g., "NOISE:0.05\n")
    //
    // ==================== DATA PACKET FORMAT ====================
    // Binary format for bulk streaming (maximum throughput):
    //   [Header: 4 bytes = 0xDEADBEEF]
    //   [Packet Count: 4 bytes uint32_t]
    //   [Start Sample Index: 4 bytes uint32_t]
    //   [Num Samples: 2 bytes uint16_t]
    //   [Sample Data: N x 4 bytes floats (max 361 samples = 1444 bytes)]
    //   [Checksum: 2 bytes uint16_t]
    // Total packet size: 16 + (N * 4) bytes, max 1460 bytes

    // Process incoming network commands
    if (inputs != nullptr && inputs->network_rx.data_ready && inputs->network_rx.rx_length > 0) {
        const uint8_t* rx_data = inputs->network_rx.rx_buffer;
        uint16_t rx_len = inputs->network_rx.rx_length;

        // Parse text-based commands (null-terminated or newline-terminated)
        if (rx_len >= 5 && rx_data[0] == 'S' && rx_data[1] == 'T' && rx_data[2] == 'A' && rx_data[3] == 'R' &&
            rx_data[4] == 'T') {
            // START command received
            net_stream_state.is_streaming = true;
            net_stream_state.sample_count = 0;
            net_stream_state.tx_packet_count = 0;
        }
        else if (rx_len >= 4 && rx_data[0] == 'S' && rx_data[1] == 'T' && rx_data[2] == 'O' && rx_data[3] == 'P') {
            // STOP command received
            net_stream_state.is_streaming = false;
        }
        else if (rx_len >= 4 && rx_data[0] == 'A' && rx_data[1] == 'M' && rx_data[2] == 'P' && rx_data[3] == ':') {
            // AMP:X.X command - parse float value
            // Simple ASCII to float conversion (assumes format "AMP:1.23")
            float new_amp = 0.0f;
            bool is_negative = false;
            uint16_t idx = 4;

            if (idx < rx_len && rx_data[idx] == '-') {
                is_negative = true;
                idx++;
            }

            // Parse integer part
            while (idx < rx_len && rx_data[idx] >= '0' && rx_data[idx] <= '9') {
                new_amp = new_amp * 10.0f + (rx_data[idx] - '0');
                idx++;
            }

            // Parse decimal part
            if (idx < rx_len && rx_data[idx] == '.') {
                idx++;
                float decimal_place = 0.1f;
                while (idx < rx_len && rx_data[idx] >= '0' && rx_data[idx] <= '9') {
                    new_amp += (rx_data[idx] - '0') * decimal_place;
                    decimal_place *= 0.1f;
                    idx++;
                }
            }

            if (is_negative) new_amp = -new_amp;

            // Clamp to reasonable range
            if (new_amp >= 0.0f && new_amp <= 10.0f) {
                net_stream_state.test_signal_amplitude = new_amp;
            }
        }
        else if (rx_len >= 6 && rx_data[0] == 'N' && rx_data[1] == 'O' && rx_data[2] == 'I' && rx_data[3] == 'S' &&
                 rx_data[4] == 'E' && rx_data[5] == ':') {
            // NOISE:X.X command - parse float value
            float new_noise = 0.0f;
            uint16_t idx = 6;

            // Parse integer part
            while (idx < rx_len && rx_data[idx] >= '0' && rx_data[idx] <= '9') {
                new_noise = new_noise * 10.0f + (rx_data[idx] - '0');
                idx++;
            }

            // Parse decimal part
            if (idx < rx_len && rx_data[idx] == '.') {
                idx++;
                float decimal_place = 0.1f;
                while (idx < rx_len && rx_data[idx] >= '0' && rx_data[idx] <= '9') {
                    new_noise += (rx_data[idx] - '0') * decimal_place;
                    decimal_place *= 0.1f;
                    idx++;
                }
            }

            // Clamp to reasonable range
            if (new_noise >= 0.0f && new_noise <= 1.0f) {
                net_stream_state.noise_level = new_noise;
            }
        }
    }

    // Prepare network transmission (streaming data packets)
    if (outputs != nullptr) {
        if (net_stream_state.is_streaming && inputs != nullptr && inputs->network_rx.tcp_connected) {
            if (outputs->network_tx.transmit_request) {
                // Previous packet still waiting to be transmitted; do not overwrite the buffer
                return;
            }

            const uint16_t num_samples = network_streaming_state::SAMPLES_PER_PACKET;
            const uint32_t start_sample_idx = net_stream_state.sample_count;

            // Build binary packet header
            uint8_t* tx_data = outputs->network_tx.tx_buffer;
            uint16_t tx_idx = 0;

            // Header: 0xDEADBEEF
            tx_data[tx_idx++] = 0xDE;
            tx_data[tx_idx++] = 0xAD;
            tx_data[tx_idx++] = 0xBE;
            tx_data[tx_idx++] = 0xEF;

            // Packet count (big-endian)
            tx_data[tx_idx++] = (net_stream_state.tx_packet_count >> 24) & 0xFF;
            tx_data[tx_idx++] = (net_stream_state.tx_packet_count >> 16) & 0xFF;
            tx_data[tx_idx++] = (net_stream_state.tx_packet_count >> 8) & 0xFF;
            tx_data[tx_idx++] = net_stream_state.tx_packet_count & 0xFF;

            // Start sample index (big-endian)
            tx_data[tx_idx++] = (start_sample_idx >> 24) & 0xFF;
            tx_data[tx_idx++] = (start_sample_idx >> 16) & 0xFF;
            tx_data[tx_idx++] = (start_sample_idx >> 8) & 0xFF;
            tx_data[tx_idx++] = start_sample_idx & 0xFF;

            // Number of samples (big-endian, 16-bit)
            tx_data[tx_idx++] = (num_samples >> 8) & 0xFF;
            tx_data[tx_idx++] = num_samples & 0xFF;

            // Generate and pack sample data (IEEE 754 float, big-endian)
            union {
                float f;
                uint32_t u;
            } float_converter;

            // CONSTANT SIGNAL FOR TESTING - all samples have identical value
            // This ensures every packet has the same checksum, making it easy to detect corruption/framing issues
            float constant_value = net_stream_state.test_signal_amplitude;  // Use amplitude parameter as constant value
            float_converter.f = constant_value;

            for (uint16_t i = 0; i < num_samples; i++) {
                // Pack constant value as big-endian float (same value for all samples)
                tx_data[tx_idx++] = (float_converter.u >> 24) & 0xFF;
                tx_data[tx_idx++] = (float_converter.u >> 16) & 0xFF;
                tx_data[tx_idx++] = (float_converter.u >> 8) & 0xFF;
                tx_data[tx_idx++] = float_converter.u & 0xFF;
            }

            // Checksum: simple sum of all bytes (excluding checksum itself)
            uint16_t checksum = 0;
            for (uint16_t i = 0; i < tx_idx; i++) {
                checksum += tx_data[i];
            }
            tx_data[tx_idx++] = (checksum >> 8) & 0xFF;
            tx_data[tx_idx++] = checksum & 0xFF;

            // Set transmission parameters
            outputs->network_tx.tx_length = tx_idx;  // Should be 1460 bytes max
            outputs->network_tx.transmit_request = true;
            outputs->network_tx.socket_id = inputs->network_rx.socket_id;

            // Increment counters
            net_stream_state.sample_count += num_samples;  // Advance by chunk size
            net_stream_state.tx_packet_count++;
        }
        else {
            // Default: No transmission
            outputs->network_tx.transmit_request = false;
            outputs->network_tx.tx_length = 0;
        }
    }
}

}  // extern "C"