#include "sfc_embedded/grid_side_tasks.hpp"

#include <cmath>
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
void yeet_task_200us(const task_inputs_t* inputs, task_outputs_t* outputs)
{
    // HIGH-FREQUENCY CONTROL TASK
    // Configured rate: 200 µs (5 kHz)
    // Execution time budget: < 100 µs (50% of period)
    // Called from: TIM2_IRQHandler
    // Priority: Highest (innermost current control loop)

    // Update mock ADC inputs with real hardware measurements
    // In embedded operation, these would come from actual ADC DMA buffers
    if (inputs != nullptr && inputs->analog_inputs.valid) {
        // Map generic analog inputs to application-specific mock signals
        // TODO: Replace with actual signal mapping based on hardware configuration
        // Example mapping (adjust indices based on your ADC channel assignment):
        //   channels[0-2] = Phase voltages
        //   channels[3-5] = Phase currents
        //   channels[6-7] = DC link measurements
        mock_i_a_counts.write(yeet::control::adc_counts{inputs->analog_inputs.channels[0]});
        mock_i_c_counts.write(yeet::control::adc_counts{inputs->analog_inputs.channels[1]});
        mock_v_ab_counts.write(yeet::control::adc_counts{inputs->analog_inputs.channels[2]});
        mock_v_bc_counts.write(yeet::control::adc_counts{inputs->analog_inputs.channels[3]});
    }

    // Execute the 5 kHz control update
    get_grid_side_control().update_200us();

    // Extract gate pulse outputs from the modulator
    // In production, these would be applied to thyristor gate driver hardware
    if (outputs != nullptr) {
        // For now, initialize all pulses to false (safe default)
        // TODO: Read actual pulse commands from grid_side_modulator output when available
        outputs->thyristor_pulses.a_plus = false;
        outputs->thyristor_pulses.c_minus = false;
        outputs->thyristor_pulses.b_plus = false;
        outputs->thyristor_pulses.a_minus = false;
        outputs->thyristor_pulses.c_plus = false;
        outputs->thyristor_pulses.b_minus = false;
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

    // Process fiber-optic RX data if available
    if (inputs != nullptr && inputs->fiber_optic_rx.data_ready) {
        // TODO: Process received data from remote board
        // Example: Read speed reference, status flags, or setpoints
        // float remote_speed = inputs->fiber_optic_rx.payload[0];
        // float remote_current = inputs->fiber_optic_rx.payload[1];
    }

    // Update digital input states
    if (inputs != nullptr && inputs->digital_inputs.valid) {
        // TODO: Map generic digital inputs to control signals
        // Example mappings:
        //   signals[0] = Emergency stop
        //   signals[1] = Enable command
        //   signals[2] = Fault reset
        // bool emergency_stop = inputs->digital_inputs.signals[0];
    }

    get_grid_side_control().update_1ms();

    // Prepare fiber-optic TX data
    if (outputs != nullptr) {
        // TODO: Populate fiber TX payload with data for remote board
        // Example: Send measured values, status flags, or commands
        // outputs->fiber_optic_tx.payload[0] = measured_voltage;
        // outputs->fiber_optic_tx.payload[1] = measured_current;
        // outputs->fiber_optic_tx.payload_length = 2;
        // outputs->fiber_optic_tx.transmit_request = true;
        outputs->fiber_optic_tx.payload_length = 0;
        outputs->fiber_optic_tx.transmit_request = false;
    }

    // Update digital outputs
    if (outputs != nullptr) {
        // TODO: Map control status to generic digital outputs
        // Example mappings:
        //   signals[0] = Fault indicator
        //   signals[1] = Running indicator
        //   signals[16] = Communication status
        // outputs->digital_outputs.signals[0] = system_fault;
        // outputs->digital_outputs.signals[1] = system_running;
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