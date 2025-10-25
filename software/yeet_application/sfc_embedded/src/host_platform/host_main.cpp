/**
 * @file host_main.cpp
 * @brief Main entry point for SFC host simulator
 *
 * Provides a standalone executable that runs the SFC control algorithms
 * on the host platform (Windows/Linux) without hardware. Enables:
 * - Closed-loop testing without embedded hardware
 * - Connection to telemetry dashboard for visualization
 * - Debugging of control algorithms in familiar environment
 *
 * Architecture:
 * - Uses host_tcp_server for telemetry (replaces W5200)
 * - Uses host_timing_loop for 100 Hz control cycle (replaces FreeRTOS)
 * - Uses host_io_mocks for ADC/DIO (replaces hardware peripherals)
 * - Executes actual grid_side_tasks.cpp control code (95%+ reuse)
 *
 * Command Line:
 *   sfc_host_simulator [--port PORT] [--frequency HZ] [--verbose]
 *
 * Example:
 *   sfc_host_simulator --port 7000 --frequency 100 --verbose
 *   (then connect telemetry dashboard to localhost:7000)
 */

#ifdef HOST_BUILD

#include <atomic>
#include <csignal>
#include <cstring>
#include <iostream>

#include "sfc_embedded/grid_side_tasks.hpp"
#include "sfc_embedded/host_platform/host_io_mocks.hpp"
#include "sfc_embedded/host_platform/host_tcp_server.hpp"
#include "sfc_embedded/host_platform/host_timing.hpp"
#include "sfc_embedded/network_telemetry.hpp"
#include "sfc_embedded/parameter_handler.hpp"
#include "sfc_embedded/task_io.h"

namespace {
// Global flag for graceful shutdown on Ctrl+C
std::atomic<bool> shutdown_requested(false);

// Signal handler for Ctrl+C
void signal_handler(int signal)
{
    if (signal == SIGINT) {
        std::cout << "\n[MAIN] Shutdown requested (Ctrl+C)" << std::endl;
        shutdown_requested.store(true);
    }
}

// Command line arguments
struct config {
    int port = 7000;              // Default telemetry port
    float frequency_hz = 100.0f;  // Default control frequency
    bool verbose = false;         // Verbose logging
};

// Parse command line arguments
config parse_args(int argc, char* argv[])
{
    config cfg;

    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--port") == 0 && i + 1 < argc) {
            cfg.port = std::atoi(argv[++i]);
        }
        else if (std::strcmp(argv[i], "--frequency") == 0 && i + 1 < argc) {
            cfg.frequency_hz = std::atof(argv[++i]);
        }
        else if (std::strcmp(argv[i], "--verbose") == 0) {
            cfg.verbose = true;
        }
        else if (std::strcmp(argv[i], "--help") == 0 || std::strcmp(argv[i], "-h") == 0) {
            std::cout << "Usage: sfc_host_simulator [OPTIONS]\n"
                      << "Options:\n"
                      << "  --port PORT        Telemetry TCP port (default: 7000)\n"
                      << "  --frequency HZ     Control loop frequency (default: 100.0)\n"
                      << "  --verbose          Enable verbose logging\n"
                      << "  --help, -h         Show this help message\n";
            std::exit(0);
        }
        else {
            std::cerr << "[ERROR] Unknown argument: " << argv[i] << std::endl;
            std::exit(1);
        }
    }

    return cfg;
}
}  // namespace

int main(int argc, char* argv[])
{
    using namespace yeet::application::sfc::host;

    // Parse command line
    config cfg = parse_args(argc, argv);

    std::cout << "======================================" << std::endl;
    std::cout << "  SFC Host Simulator" << std::endl;
    std::cout << "======================================" << std::endl;
    std::cout << "Telemetry port:    " << cfg.port << std::endl;
    std::cout << "Control frequency: " << cfg.frequency_hz << " Hz" << std::endl;
    std::cout << "Verbose logging:   " << (cfg.verbose ? "ON" : "OFF") << std::endl;
    std::cout << "======================================" << std::endl;

    // Install signal handler for graceful shutdown
    std::signal(SIGINT, signal_handler);

    // Initialize subsystems
    std::cout << "[INIT] Initializing mock I/O..." << std::endl;
    initialize_mock_io();

    std::cout << "[INIT] Initializing TCP server..." << std::endl;
    host_tcp_server telemetry_server;
    if (!telemetry_server.initialize(cfg.port, "0.0.0.0")) {
        std::cerr << "[ERROR] Failed to initialize TCP server on port " << cfg.port << std::endl;
        return 1;
    }
    std::cout << "[INIT] TCP server listening on port " << cfg.port << std::endl;

    std::cout << "[INIT] Initializing timing loop..." << std::endl;
    host_timing_loop control_loop(cfg.frequency_hz);

    std::cout << "[INIT] Initializing SFC grid-side control..." << std::endl;
    yeet_embedded_init();

    std::cout << "[READY] Starting control loop at " << cfg.frequency_hz << " Hz" << std::endl;
    std::cout << "[READY] Connect telemetry dashboard to localhost:" << cfg.port << std::endl;
    std::cout << "[READY] Press Ctrl+C to stop" << std::endl;
    std::cout << "======================================" << std::endl;

    // Start control loop
    control_loop.start();

    std::uint32_t cycle_count = 0;
    const std::uint32_t stats_interval = static_cast<std::uint32_t>(cfg.frequency_hz * 10.0f);  // Every 10 seconds

    while (control_loop.is_running() && !shutdown_requested.load()) {
        // Wait for next cycle
        if (!control_loop.wait_for_next_cycle()) {
            break;
        }

        // Get current cycle number and timestamp
        cycle_count = control_loop.get_cycle_number();
        std::uint32_t timestamp_ms = static_cast<std::uint32_t>(control_loop.get_elapsed_time_ms());

        // Update mock I/O (advances waveform generators, etc.)
        update_mock_io(timestamp_ms);

        // Process TCP server (accept clients, receive commands)
        telemetry_server.process();

        // Read mock inputs
        mock_adc_measurements adc = read_mock_adc();
        mock_dio_inputs dio_in = read_mock_dio();

        // Prepare task inputs structure
        // Convert per-unit values to ADC counts (assume 16-bit ADC, ±10V range, centered at 32768)
        // For voltage: 1.0 pu = full scale = 40000 counts (approx)
        // For current: 1.0 pu = full scale = 40000 counts (approx)
        task_inputs_t task_inputs{};
        task_inputs.analog_inputs.channels[0] = static_cast<std::uint16_t>(32768 + adc.grid_voltage_a_pu * 10000);
        task_inputs.analog_inputs.channels[1] = static_cast<std::uint16_t>(32768 + adc.grid_voltage_b_pu * 10000);
        task_inputs.analog_inputs.channels[2] = static_cast<std::uint16_t>(32768 + adc.grid_voltage_c_pu * 10000);
        task_inputs.analog_inputs.channels[3] = static_cast<std::uint16_t>(32768 + adc.dc_link_current_pu * 10000);
        task_inputs.analog_inputs.valid = true;

        task_inputs.digital_inputs.signals[0] = dio_in.fault_reset_command;
        task_inputs.digital_inputs.signals[1] = dio_in.enable_command;
        task_inputs.digital_inputs.signals[2] = dio_in.start_command;
        task_inputs.digital_inputs.signals[3] = dio_in.stop_command;
        task_inputs.digital_inputs.valid = true;
        task_inputs.fiber_optic_rx.data_ready = false;
        task_inputs.fiber_optic_rx.link_ok = false;
        task_inputs.fiber_optic_rx.payload_length = 0;

        // Prepare task outputs structure
        task_outputs_t task_outputs{};

        // Execute control tasks based on timing
        // 200µs task runs every cycle (main control loop)
        yeet_task_200us(&task_inputs, &task_outputs);

        // 1ms task runs every 10 cycles (at 100 Hz = 10ms period, every 10ms)
        if (cycle_count % 10 == 0) {
            yeet_task_1000us(&task_inputs, &task_outputs);
        }

        // 10ms task runs every 100 cycles (at 100 Hz = 10ms period, every 100ms)
        if (cycle_count % 100 == 0) {
            yeet_task_10000us(&task_inputs, &task_outputs);
        }

        // Write mock outputs
        mock_dio_outputs dio_out{};
        dio_out.fault_status = task_outputs.digital_outputs.signals[0];
        dio_out.running_status = task_outputs.digital_outputs.signals[1];
        dio_out.ready_status = task_outputs.digital_outputs.signals[2];
        dio_out.grid_contactor_command = task_outputs.digital_outputs.signals[3];
        dio_out.machine_contactor_command = task_outputs.digital_outputs.signals[4];
        write_mock_dio(dio_out);

        // Handle network communication (telemetry and parameters)
        // The yeet_network() function (in grid_side_tasks.cpp) handles all packet parsing,
        // parameter operations, and telemetry control - shared code between embedded and host builds
        if (telemetry_server.is_client_connected()) {
            // Process all pending packets in a loop to handle rapid-fire parameter requests
            // Limit iterations to prevent blocking the control loop
            constexpr int max_packets_per_cycle = 20;
            int packets_processed = 0;

            while (packets_processed < max_packets_per_cycle) {
                network_inputs_t network_inputs{};
                network_outputs_t network_outputs{};

                // Check for received data from TCP client
                std::uint8_t rx_buffer[1024];
                int rx_len = telemetry_server.receive_data(rx_buffer, sizeof(rx_buffer));

                if (rx_len <= 0) {
                    // No more received packets, but still need to call yeet_network() for telemetry transmission
                    // Only do this once per cycle (when packets_processed == 0)
                    if (packets_processed == 0) {
                        // Prepare network inputs with connection status but no received data
                        network_inputs.network_rx.data_ready = false;
                        network_inputs.network_rx.rx_length = 0;
                        network_inputs.network_rx.tcp_connected = true;
                        network_inputs.network_rx.socket_id = 0;

                        // Execute shared network task (will transmit telemetry if ready)
                        yeet_network(&network_inputs, &network_outputs);

                        // Send telemetry packet if generated
                        if (network_outputs.network_tx.transmit_request && network_outputs.network_tx.tx_length > 0) {
                            telemetry_server.send_data(
                                network_outputs.network_tx.tx_buffer, network_outputs.network_tx.tx_length);
                        }
                    }
                    break;
                }

                packets_processed++;

                // Copy received data into network_inputs structure
                std::memcpy(network_inputs.network_rx.rx_buffer, rx_buffer,
                    std::min(rx_len, static_cast<int>(sizeof(network_inputs.network_rx.rx_buffer))));
                network_inputs.network_rx.data_ready = true;
                network_inputs.network_rx.rx_length = static_cast<std::uint16_t>(rx_len);
                network_inputs.network_rx.tcp_connected = true;
                network_inputs.network_rx.socket_id = 0;  // Single client socket

                // Execute shared network task (handles both parameter protocol and telemetry commands)
                yeet_network(&network_inputs, &network_outputs);

                // Send response if network task prepared one
                if (network_outputs.network_tx.transmit_request && network_outputs.network_tx.tx_length > 0) {
                    telemetry_server.send_data(
                        network_outputs.network_tx.tx_buffer, network_outputs.network_tx.tx_length);
                }
            }
        }

        // Mark end of cycle for timing measurement
        control_loop.mark_cycle_end();

        // Print statistics periodically
        if (cfg.verbose && (cycle_count % stats_interval == 0)) {
            timing_statistics stats = control_loop.get_statistics();
            std::cout << "[STATS] Cycles: " << stats.total_cycles << " | Avg: " << stats.average_cycle_time_ms << " ms"
                      << " | Max: " << stats.max_cycle_time_ms << " ms"
                      << " | Overruns: " << stats.overrun_count
                      << " | Client: " << (telemetry_server.is_client_connected() ? "CONNECTED" : "disconnected")
                      << std::endl;
        }
    }

    // Shutdown
    std::cout << "\n[SHUTDOWN] Stopping control loop..." << std::endl;
    control_loop.stop();

    std::cout << "[SHUTDOWN] Closing TCP server..." << std::endl;
    telemetry_server.shutdown();

    // Print final statistics
    timing_statistics final_stats = control_loop.get_statistics();
    std::cout << "\n======================================" << std::endl;
    std::cout << "  Final Statistics" << std::endl;
    std::cout << "======================================" << std::endl;
    std::cout << "Total cycles:      " << final_stats.total_cycles << std::endl;
    std::cout << "Average cycle:     " << final_stats.average_cycle_time_ms << " ms" << std::endl;
    std::cout << "Max cycle:         " << final_stats.max_cycle_time_ms << " ms" << std::endl;
    std::cout << "Min cycle:         " << final_stats.min_cycle_time_ms << " ms" << std::endl;
    std::cout << "Overrun count:     " << final_stats.overrun_count << std::endl;
    std::cout << "Total runtime:     " << control_loop.get_elapsed_time_ms() / 1000.0f << " s" << std::endl;
    std::cout << "======================================" << std::endl;

    std::cout << "[SHUTDOWN] Complete" << std::endl;

    return 0;
}

#endif  // HOST_BUILD
