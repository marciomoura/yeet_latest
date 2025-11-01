/**
 * @file host_main.cpp
 * @brief Main entry point for SFC dual-controller host simulator
 *
 * Provides a standalone executable that runs BOTH grid-side and machine-side
 * SFC control algorithms on the host platform (Windows/Linux) without hardware.
 *
 * Features:
 * - Simultaneous execution of both controllers in single process
 * - Dual TCP servers for independent telemetry (port 7000: grid, port 7001: machine)
 * - Fiber-optic data exchange simulation between controllers
 * - Closed-loop testing without embedded hardware
 * - Debugging of both controllers and their interaction
 *
 * Architecture:
 * - Two host_tcp_server instances for independent telemetry
 * - Single host_timing_loop for synchronized 100 Hz control cycle
 * - Shared host_io_mocks for ADC/DIO (replaces hardware peripherals)
 * - Executes both grid_side_tasks.cpp and machine_side_tasks.cpp (95%+ code reuse)
 * - Simulated fiber-optic link for inter-controller communication
 *
 * Command Line:
 *   sfc_host_simulator [--grid-port PORT] [--machine-port PORT] [--frequency HZ] [--verbose]
 *
 * Example:
 *   sfc_host_simulator --grid-port 7000 --machine-port 7001 --frequency 100 --verbose
 *   (connect telemetry dashboard to localhost:7000 for grid, localhost:7001 for machine)
 */

#ifdef HOST_BUILD

#include <array>
#include <atomic>
#include <csignal>
#include <cstring>
#include <iostream>

// Include both controller task headers
#include "sfc_embedded/grid_side_tasks.hpp"
#include "sfc_embedded/host_platform/host_io_mocks.hpp"
#include "sfc_embedded/host_platform/host_tcp_server.hpp"
#include "sfc_embedded/host_platform/host_timing.hpp"
#include "sfc_embedded/machine_side_tasks.hpp"
#include "sfc_embedded/task_io.h"
#include "yeet/network_telemetry.hpp"
#include "yeet/parameter_handler.hpp"

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
    int grid_port = 7000;         // Telemetry port for grid-side
    int machine_port = 7001;      // Telemetry port for machine-side
    float frequency_hz = 100.0f;  // Control frequency
    bool verbose = false;         // Verbose logging
};

// Parse command line arguments
config parse_args(int argc, char* argv[])
{
    config cfg;

    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--grid-port") == 0 && i + 1 < argc) {
            cfg.grid_port = std::atoi(argv[++i]);
        }
        else if (std::strcmp(argv[i], "--machine-port") == 0 && i + 1 < argc) {
            cfg.machine_port = std::atoi(argv[++i]);
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
                      << "  --grid-port PORT       Grid-side telemetry TCP port (default: 7000)\n"
                      << "  --machine-port PORT    Machine-side telemetry TCP port (default: 7001)\n"
                      << "  --frequency HZ         Control loop frequency (default: 100.0)\n"
                      << "  --verbose              Enable verbose logging\n"
                      << "  --help, -h             Show this help message\n";
            std::exit(0);
        }
        else {
            std::cerr << "[ERROR] Unknown argument: " << argv[i] << std::endl;
            std::exit(1);
        }
    }

    return cfg;
}

// Simulated fiber-optic link between grid and machine controllers
struct fiber_optic_link {
    // Simulate data transfer from grid to machine
    void transfer_grid_to_machine(const task_outputs_t& grid_outputs, task_inputs_t& machine_inputs)
    {
        if (grid_outputs.fiber_optic_tx.transmit_request && grid_outputs.fiber_optic_tx.payload_length > 0) {
            std::memcpy(machine_inputs.fiber_optic_rx.payload, grid_outputs.fiber_optic_tx.payload,
                std::min(grid_outputs.fiber_optic_tx.payload_length, static_cast<std::uint8_t>(4)) *
                    sizeof(std::uint32_t));
            machine_inputs.fiber_optic_rx.payload_length = grid_outputs.fiber_optic_tx.payload_length;
            machine_inputs.fiber_optic_rx.data_ready = true;
            machine_inputs.fiber_optic_rx.link_ok = true;
        }
        else {
            machine_inputs.fiber_optic_rx.data_ready = false;
            machine_inputs.fiber_optic_rx.link_ok = true;
            machine_inputs.fiber_optic_rx.payload_length = 0;
        }
    }

    // Simulate data transfer from machine to grid
    void transfer_machine_to_grid(const task_outputs_t& machine_outputs, task_inputs_t& grid_inputs)
    {
        if (machine_outputs.fiber_optic_tx.transmit_request && machine_outputs.fiber_optic_tx.payload_length > 0) {
            std::memcpy(grid_inputs.fiber_optic_rx.payload, machine_outputs.fiber_optic_tx.payload,
                std::min(machine_outputs.fiber_optic_tx.payload_length, static_cast<std::uint8_t>(4)) *
                    sizeof(std::uint32_t));
            grid_inputs.fiber_optic_rx.payload_length = machine_outputs.fiber_optic_tx.payload_length;
            grid_inputs.fiber_optic_rx.data_ready = true;
            grid_inputs.fiber_optic_rx.link_ok = true;
        }
        else {
            grid_inputs.fiber_optic_rx.data_ready = false;
            grid_inputs.fiber_optic_rx.link_ok = true;
            grid_inputs.fiber_optic_rx.payload_length = 0;
        }
    }
};
}  // namespace

int main(int argc, char* argv[])
{
    using namespace yeet::application::sfc::host;

    // Parse command line
    config cfg = parse_args(argc, argv);

    std::cout << "======================================" << std::endl;
    std::cout << "  SFC Dual-Controller Host Simulator" << std::endl;
    std::cout << "======================================" << std::endl;
    std::cout << "Grid telemetry:    localhost:" << cfg.grid_port << std::endl;
    std::cout << "Machine telemetry: localhost:" << cfg.machine_port << std::endl;
    std::cout << "Control frequency: " << cfg.frequency_hz << " Hz" << std::endl;
    std::cout << "Verbose logging:   " << (cfg.verbose ? "ON" : "OFF") << std::endl;
    std::cout << "======================================" << std::endl;

    // Install signal handler for graceful shutdown
    std::signal(SIGINT, signal_handler);

    // Initialize subsystems
    std::cout << "[INIT] Initializing mock I/O..." << std::endl;
    initialize_mock_io();

    std::cout << "[INIT] Initializing grid-side TCP server..." << std::endl;
    host_tcp_server grid_telemetry_server;
    if (!grid_telemetry_server.initialize(cfg.grid_port, "0.0.0.0")) {
        std::cerr << "[ERROR] Failed to initialize grid TCP server on port " << cfg.grid_port << std::endl;
        return 1;
    }
    std::cout << "[INIT] Grid TCP server listening on port " << cfg.grid_port << std::endl;

    std::cout << "[INIT] Initializing machine-side TCP server..." << std::endl;
    host_tcp_server machine_telemetry_server;
    if (!machine_telemetry_server.initialize(cfg.machine_port, "0.0.0.0")) {
        std::cerr << "[ERROR] Failed to initialize machine TCP server on port " << cfg.machine_port << std::endl;
        return 1;
    }
    std::cout << "[INIT] Machine TCP server listening on port " << cfg.machine_port << std::endl;

    std::cout << "[INIT] Initializing timing loop..." << std::endl;
    host_timing_loop control_loop(cfg.frequency_hz);

    std::cout << "[INIT] Initializing fiber-optic link simulation..." << std::endl;
    fiber_optic_link fiber_link;

    std::cout << "[INIT] Initializing SFC grid-side control..." << std::endl;
    grid_embedded_init();

    std::cout << "[INIT] Initializing SFC machine-side control..." << std::endl;
    machine_embedded_init();

    std::cout << "[READY] Starting dual-controller loop at " << cfg.frequency_hz << " Hz" << std::endl;
    std::cout << "[READY] Grid telemetry:    localhost:" << cfg.grid_port << std::endl;
    std::cout << "[READY] Machine telemetry: localhost:" << cfg.machine_port << std::endl;
    std::cout << "[READY] Press Ctrl+C to stop" << std::endl;
    std::cout << "======================================" << std::endl;

    // Start control loop
    control_loop.start();

    std::uint32_t cycle_count = 0;
    const std::uint32_t stats_interval = static_cast<std::uint32_t>(cfg.frequency_hz * 10.0f);  // Every 10 seconds

    // Task I/O structures for both controllers
    task_outputs_t grid_outputs{};
    task_outputs_t machine_outputs{};

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

        // Process TCP servers (accept clients, receive commands)
        grid_telemetry_server.process();
        machine_telemetry_server.process();

        // Read mock inputs (shared between both controllers)
        mock_adc_measurements adc = read_mock_adc();
        mock_dio_inputs dio_in = read_mock_dio();

        // ==================== Prepare Grid-Side Inputs ====================
        task_inputs_t grid_inputs{};
        grid_inputs.analog_inputs.channels[0] = static_cast<std::uint16_t>(32768 + adc.grid_voltage_a_pu * 10000);
        grid_inputs.analog_inputs.channels[1] = static_cast<std::uint16_t>(32768 + adc.grid_voltage_b_pu * 10000);
        grid_inputs.analog_inputs.channels[2] = static_cast<std::uint16_t>(32768 + adc.grid_voltage_c_pu * 10000);
        grid_inputs.analog_inputs.channels[3] = static_cast<std::uint16_t>(32768 + adc.dc_link_current_pu * 10000);
        grid_inputs.analog_inputs.valid = true;

        grid_inputs.digital_inputs.signals[0] = dio_in.fault_reset_command;
        grid_inputs.digital_inputs.signals[1] = dio_in.enable_command;
        grid_inputs.digital_inputs.signals[2] = dio_in.start_command;
        grid_inputs.digital_inputs.signals[3] = dio_in.stop_command;
        grid_inputs.digital_inputs.valid = true;

        // Fiber-optic input from machine to grid (populated by previous cycle's machine output)
        fiber_link.transfer_machine_to_grid(machine_outputs, grid_inputs);

        // ==================== Prepare Machine-Side Inputs ====================
        // Note: In the mock simulator, both grid and machine see the same voltages
        // In a real system, machine voltages would be different (motor back-EMF)
        task_inputs_t machine_inputs{};
        machine_inputs.analog_inputs.channels[0] = static_cast<std::uint16_t>(32768 + adc.grid_voltage_a_pu * 10000);
        machine_inputs.analog_inputs.channels[1] = static_cast<std::uint16_t>(32768 + adc.grid_voltage_b_pu * 10000);
        machine_inputs.analog_inputs.channels[2] = static_cast<std::uint16_t>(32768 + adc.grid_voltage_c_pu * 10000);
        machine_inputs.analog_inputs.channels[3] = static_cast<std::uint16_t>(32768 + adc.dc_link_current_pu * 10000);
        machine_inputs.analog_inputs.valid = true;

        machine_inputs.digital_inputs.signals[0] = dio_in.fault_reset_command;
        machine_inputs.digital_inputs.signals[1] = dio_in.enable_command;
        machine_inputs.digital_inputs.signals[2] = dio_in.start_command;
        machine_inputs.digital_inputs.signals[3] = dio_in.stop_command;
        machine_inputs.digital_inputs.valid = true;

        // Fiber-optic input from grid to machine (populated by previous cycle's grid output)
        fiber_link.transfer_grid_to_machine(grid_outputs, machine_inputs);

        // ==================== Execute Grid-Side Control Tasks ====================
        grid_task_200us(&grid_inputs, &grid_outputs);

        if (cycle_count % 10 == 0) {
            grid_task_1000us(&grid_inputs, &grid_outputs);
        }

        if (cycle_count % 100 == 0) {
            grid_task_10000us(&grid_inputs, &grid_outputs);
        }

        // ==================== Execute Machine-Side Control Tasks ====================
        machine_task_200us(&machine_inputs, &machine_outputs);

        if (cycle_count % 10 == 0) {
            machine_task_1000us(&machine_inputs, &machine_outputs);
        }

        if (cycle_count % 100 == 0) {
            machine_task_10000us(&machine_inputs, &machine_outputs);
        }

        // ==================== Write Mock Outputs ====================
        mock_dio_outputs dio_out{};
        dio_out.fault_status = grid_outputs.digital_outputs.signals[0] || machine_outputs.digital_outputs.signals[0];
        dio_out.running_status = grid_outputs.digital_outputs.signals[1] && machine_outputs.digital_outputs.signals[1];
        dio_out.ready_status = grid_outputs.digital_outputs.signals[2] && machine_outputs.digital_outputs.signals[2];
        dio_out.grid_contactor_command = grid_outputs.digital_outputs.signals[3];
        dio_out.machine_contactor_command = machine_outputs.digital_outputs.signals[4];
        write_mock_dio(dio_out);

        // ==================== Grid-Side Network Communication ====================
        if (grid_telemetry_server.is_client_connected()) {
            constexpr int max_packets_per_cycle = 20;
            int packets_processed = 0;

            while (packets_processed < max_packets_per_cycle) {
                network_inputs_t network_inputs{};
                network_outputs_t network_outputs{};

                std::uint8_t rx_buffer[1024];
                int rx_len = grid_telemetry_server.receive_data(rx_buffer, sizeof(rx_buffer));

                if (rx_len <= 0) {
                    break;
                }

                packets_processed++;

                std::memcpy(network_inputs.network_rx.rx_buffer, rx_buffer,
                    std::min(rx_len, static_cast<int>(sizeof(network_inputs.network_rx.rx_buffer))));
                network_inputs.network_rx.data_ready = true;
                network_inputs.network_rx.rx_length = static_cast<std::uint16_t>(rx_len);
                network_inputs.network_rx.tcp_connected = true;
                network_inputs.network_rx.socket_id = 0;

                grid_network(&network_inputs, &network_outputs);

                if (network_outputs.network_tx.transmit_request && network_outputs.network_tx.tx_length > 0) {
                    grid_telemetry_server.send_data(
                        network_outputs.network_tx.tx_buffer, network_outputs.network_tx.tx_length);
                }
            }

            // Transmit pending telemetry
            {
                network_inputs_t network_inputs{};
                network_outputs_t network_outputs{};

                network_inputs.network_rx.data_ready = false;
                network_inputs.network_rx.rx_length = 0;
                network_inputs.network_rx.tcp_connected = true;
                network_inputs.network_rx.socket_id = 0;

                grid_network(&network_inputs, &network_outputs);

                if (network_outputs.network_tx.transmit_request && network_outputs.network_tx.tx_length > 0) {
                    grid_telemetry_server.send_data(
                        network_outputs.network_tx.tx_buffer, network_outputs.network_tx.tx_length);
                }
            }
        }

        // ==================== Machine-Side Network Communication ====================
        if (machine_telemetry_server.is_client_connected()) {
            constexpr int max_packets_per_cycle = 20;
            int packets_processed = 0;

            while (packets_processed < max_packets_per_cycle) {
                network_inputs_t network_inputs{};
                network_outputs_t network_outputs{};

                std::uint8_t rx_buffer[1024];
                int rx_len = machine_telemetry_server.receive_data(rx_buffer, sizeof(rx_buffer));

                if (rx_len <= 0) {
                    break;
                }

                packets_processed++;

                std::memcpy(network_inputs.network_rx.rx_buffer, rx_buffer,
                    std::min(rx_len, static_cast<int>(sizeof(network_inputs.network_rx.rx_buffer))));
                network_inputs.network_rx.data_ready = true;
                network_inputs.network_rx.rx_length = static_cast<std::uint16_t>(rx_len);
                network_inputs.network_rx.tcp_connected = true;
                network_inputs.network_rx.socket_id = 0;

                machine_network(&network_inputs, &network_outputs);

                if (network_outputs.network_tx.transmit_request && network_outputs.network_tx.tx_length > 0) {
                    machine_telemetry_server.send_data(
                        network_outputs.network_tx.tx_buffer, network_outputs.network_tx.tx_length);
                }
            }

            // Transmit pending telemetry
            {
                network_inputs_t network_inputs{};
                network_outputs_t network_outputs{};

                network_inputs.network_rx.data_ready = false;
                network_inputs.network_rx.rx_length = 0;
                network_inputs.network_rx.tcp_connected = true;
                network_inputs.network_rx.socket_id = 0;

                machine_network(&network_inputs, &network_outputs);

                if (network_outputs.network_tx.transmit_request && network_outputs.network_tx.tx_length > 0) {
                    machine_telemetry_server.send_data(
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
                      << " | Grid: " << (grid_telemetry_server.is_client_connected() ? "CONNECTED" : "disconnected")
                      << " | Machine: "
                      << (machine_telemetry_server.is_client_connected() ? "CONNECTED" : "disconnected") << std::endl;
        }
    }

    // Shutdown
    std::cout << "\n[SHUTDOWN] Stopping control loop..." << std::endl;
    control_loop.stop();

    std::cout << "[SHUTDOWN] Closing grid TCP server..." << std::endl;
    grid_telemetry_server.shutdown();

    std::cout << "[SHUTDOWN] Closing machine TCP server..." << std::endl;
    machine_telemetry_server.shutdown();

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
