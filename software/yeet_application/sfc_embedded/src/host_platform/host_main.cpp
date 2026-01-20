/**
 * @file host_main_with_plant.cpp
 * @brief Main entry point for SFC dual-controller host simulator with complete plant model
 *
 * Provides a standalone executable that runs BOTH grid-side and machine-side
 * SFC control algorithms on the host platform with realistic plant dynamics.
 *
 * Features:
 * - Simultaneous execution of both controllers in single process
 * - Complete plant model (6p-6p topology, synchronous machine, thyristor bridges)
 * - Dual TCP servers for independent telemetry (port 7000: grid, port 7001: machine)
 * - Fiber-optic data exchange simulation between controllers
 * - Closed-loop testing with realistic dynamics
 *
 * Architecture:
 * - Two host_tcp_server instances for independent telemetry
 * - Single host_timing_loop for synchronized control cycle
 * - plant_complete_sfc_closed_loop_simulation_builder for plant + hardware mocks
 * - Low-level task interface (grid/machine_tasks.cpp) for control algorithms
 *
 * Integration:
 * The plant model provides:
 * - Grid and machine side voltage/current measurements
 * - Realistic electromagnetic dynamics
 * - Thyristor bridge models with firing logic
 * - Synchronous machine with excitation control
 *
 * Command Line:
 *   sfc_host_simulator_with_plant [--grid-port PORT] [--machine-port PORT] [--frequency HZ] [--verbose]
 *
 * Example:
 *   sfc_host_simulator_with_plant --grid-port 7000 --machine-port 7001 --frequency 100 --verbose
 */

#ifdef HOST_BUILD

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstring>
#include <iostream>
#include <thread>

// Control task interface
#include "sfc_embedded/grid_side_tasks.hpp"
#include "sfc_embedded/machine_side_tasks.hpp"

// Host platform layer
#include "sfc_embedded/host_platform/host_tcp_server.hpp"
#include "sfc_embedded/host_platform/host_timing.hpp"
#include "sfc_embedded/task_io.h"

// Plant model integration
#include "../../sfc_integration/tests/sfc_plant_configuration.hpp"
#include "../../sfc_integration/tests/sfc_startup_procedure.hpp"
#include "sfc_integration/plant_complete_sfc_closed_loop_simulation_builder.hpp"

// Signal utilities for test/simulation
#include "yeet/signal.hpp"  // For mock_signal_input

// Telemetry
#include "yeet/network_telemetry.hpp"
#include "yeet/parameter_handler.hpp"

namespace {
// Global flag for graceful shutdown on Ctrl+C
std::atomic<bool> shutdown_requested(false);
// Global flag for verbose logging (used in signal handler)
std::atomic<bool> global_verbose(false);

// Signal handler for Ctrl+C
void signal_handler(int signal)
{
    if (signal == SIGINT) {
        if (global_verbose.load()) {
            std::cout << "\n[MAIN] Shutdown requested (Ctrl+C)" << std::endl;
        }
        shutdown_requested.store(true);
    }
}

// Command line arguments
struct config {
    int grid_port = 7000;           // Telemetry port for grid-side
    int machine_port = 7001;        // Telemetry port for machine-side
    float frequency_hz = 10000.0f;  // Control frequency (5 kHz recommended for plant simulation)
    bool verbose = false;           // Verbose logging
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
            cfg.verbose = false;
        }
        else if (std::strcmp(argv[i], "--help") == 0 || std::strcmp(argv[i], "-h") == 0) {
            std::cout << "Usage: sfc_host_simulator_with_plant [OPTIONS]\n"
                      << "Options:\n"
                      << "  --grid-port PORT       Grid-side telemetry TCP port (default: 7000)\n"
                      << "  --machine-port PORT    Machine-side telemetry TCP port (default: 7001)\n"
                      << "  --frequency HZ         Control loop frequency (default: 5000.0 Hz)\n"
                      << "  --verbose              Enable verbose logging\n"
                      << "  --help, -h             Show this help message\n"
                      << "\n"
                      << "Note: Telemetry always runs at 50µs (20 kHz) and simulator runs as fast as possible.\n";
            std::exit(0);
        }
        else {
            if (cfg.verbose) {
                std::cerr << "[ERROR] Unknown argument: " << argv[i] << std::endl;
            }
            std::exit(1);
        }
    }

    return cfg;
}

// Handle grid-side telemetry communication
void handle_grid_telemetry(
    yeet::application::sfc::host::host_tcp_server& telemetry_server, grid_side_network_infrastructure& network_infra)
{
    if (!telemetry_server.is_client_connected()) {
        return;
    }

    constexpr int max_packets_per_cycle = 20;
    int packets_processed = 0;

    // Process incoming network packets
    while (packets_processed < max_packets_per_cycle) {
        network_inputs_t network_inputs{};
        network_outputs_t network_outputs{};

        std::uint8_t rx_buffer[1024];
        int rx_len = telemetry_server.receive_data(rx_buffer, sizeof(rx_buffer));

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

        grid_network(&network_infra, &network_inputs, &network_outputs);

        if (network_outputs.network_tx.transmit_request && network_outputs.network_tx.tx_length > 0) {
            if (!telemetry_server.send_data(
                    network_outputs.network_tx.tx_buffer, network_outputs.network_tx.tx_length)) {
                // Queue overflow - force sync frame on next successful packet to help client recover
                network_infra.network_handler.force_resync();
            }
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

        grid_network(&network_infra, &network_inputs, &network_outputs);

        if (network_outputs.network_tx.transmit_request && network_outputs.network_tx.tx_length > 0) {
            if (!telemetry_server.send_data(
                    network_outputs.network_tx.tx_buffer, network_outputs.network_tx.tx_length)) {
                // Queue overflow - force sync frame on next successful packet
                network_infra.network_handler.force_resync();
            }
        }
    }
}

// Handle machine-side telemetry communication
void handle_machine_telemetry(
    yeet::application::sfc::host::host_tcp_server& telemetry_server, machine_side_network_infrastructure& network_infra)
{
    if (!telemetry_server.is_client_connected()) {
        return;
    }

    constexpr int max_packets_per_cycle = 20;
    int packets_processed = 0;

    // Process incoming network packets
    while (packets_processed < max_packets_per_cycle) {
        network_inputs_t network_inputs{};
        network_outputs_t network_outputs{};

        std::uint8_t rx_buffer[1024];
        int rx_len = telemetry_server.receive_data(rx_buffer, sizeof(rx_buffer));

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

        machine_network(&network_infra, &network_inputs, &network_outputs);

        if (network_outputs.network_tx.transmit_request && network_outputs.network_tx.tx_length > 0) {
            if (!telemetry_server.send_data(
                    network_outputs.network_tx.tx_buffer, network_outputs.network_tx.tx_length)) {
                // Queue overflow - force sync frame on next successful packet
                network_infra.network_handler.force_resync();
            }
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

        machine_network(&network_infra, &network_inputs, &network_outputs);

        if (network_outputs.network_tx.transmit_request && network_outputs.network_tx.tx_length > 0) {
            if (!telemetry_server.send_data(
                    network_outputs.network_tx.tx_buffer, network_outputs.network_tx.tx_length)) {
                // Queue overflow - force sync frame on next successful packet
                network_infra.network_handler.force_resync();
            }
        }
    }
}

}  // namespace

int main(int argc, char* argv[])
{
    using namespace yeet::application::sfc::host;
    using namespace yeet::application::sfc::integration;

    // Parse command line
    config cfg = parse_args(argc, argv);
    global_verbose.store(cfg.verbose);

    if (cfg.verbose) {
        std::cout << "======================================" << std::endl;
        std::cout << "  SFC Dual-Controller with Plant Model" << std::endl;
        std::cout << "======================================" << std::endl;
        std::cout << "Grid telemetry:    localhost:" << cfg.grid_port << std::endl;
        std::cout << "Machine telemetry: localhost:" << cfg.machine_port << std::endl;
        std::cout << "Control frequency: " << cfg.frequency_hz << " Hz" << std::endl;
        std::cout << "Telemetry rate:    20000 Hz (50 µs)" << std::endl;
        std::cout << "Real-time pacing:  OFF (as fast as possible)" << std::endl;
        std::cout << "Verbose logging:   " << (cfg.verbose ? "ON" : "OFF") << std::endl;
        std::cout << "Plant model:       Complete 6p-6p topology" << std::endl;
        std::cout << "======================================" << std::endl;
    }

    // Install signal handler for graceful shutdown
    std::signal(SIGINT, signal_handler);

    // Initialize plant and control systems first
    if (cfg.verbose) {
        std::cout << "[INIT] Initializing plant model and control systems..." << std::endl;
    }
    plant_complete_sfc_closed_loop_simulation_builder simulator_builder;

    // Use unified simulator architecture for real-time tick-based execution
    if (cfg.verbose) {
        std::cout << "[INIT] Creating unified simulator for real-time execution..." << std::endl;
    }
    yeet::simulator simulator;

    // Register all applications with the simulator
    // This sets up multi-rate execution: 10ms (state machine, speed control),
    // 1ms (current control, fiber-optics), 200µs (measurements, observers, modulators)
    simulator_builder.register_applications(simulator);

    // Initialize mock signal inputs to safe state (matches test fixture setup)
    initialize_mock_signal_inputs(simulator_builder);

    // Initialize the system (SAME as test fixture SetUp())
    if (cfg.verbose) {
        std::cout << "[INIT] Initializing control system..." << std::endl;
    }
    simulator_builder.initialize();

    if (cfg.verbose) {
        std::cout << "[INIT] Initializing timing loop..." << std::endl;
    }
    host_timing_loop control_loop(cfg.frequency_hz);

    if (cfg.verbose) {
        std::cout << "[INIT] Initializing embedded task interface (for telemetry system)..." << std::endl;
    }
    // Initialize using the plant builder's control instances instead of the static instances
    // grid_embedded_init(&simulator_builder.grid_side_control);
    // machine_embedded_init(&simulator_builder.machine_side_control);

    // Create network infrastructure for telemetry (tied to the plant builder's control instances)
    if (cfg.verbose) {
        std::cout << "[INIT] Creating network telemetry infrastructure..." << std::endl;
    }
    grid_side_network_infrastructure grid_network_infra(simulator_builder.grid_side_control);
    machine_side_network_infrastructure machine_network_infra(simulator_builder.machine_side_control);

    grid_network_infra.parameter_handler.set_verbose(cfg.verbose);
    grid_network_infra.network_handler.set_verbose(cfg.verbose);
    grid_network_infra.telemetry_manager.set_verbose(cfg.verbose);

    machine_network_infra.parameter_handler.set_verbose(cfg.verbose);
    machine_network_infra.network_handler.set_verbose(cfg.verbose);
    machine_network_infra.telemetry_manager.set_verbose(cfg.verbose);

    // Register plant model parameters in machine-side registry (for host application only)
    if (cfg.verbose) {
        std::cout << "[INIT] Registering plant model parameters in machine-side parameter handler..." << std::endl;
    }
    simulator_builder.machine_side_control.register_plant_datamodel(
        machine_network_infra.parameter_handler, simulator_builder.plant);

    // ==================== TCP Server Initialization ====================
    if (cfg.verbose) {
        std::cout << "[INIT] Starting TCP servers for telemetry..." << std::endl;
    }
    host_tcp_server grid_telemetry_server;
    grid_telemetry_server.set_verbose(cfg.verbose);
    if (!grid_telemetry_server.initialize(cfg.grid_port, "0.0.0.0")) {
        if (cfg.verbose) {
            std::cerr << "[ERROR] Failed to initialize grid TCP server on port " << cfg.grid_port << std::endl;
        }
        return 1;
    }
    if (cfg.verbose) {
        std::cout << "[INIT] Grid TCP server listening on port " << cfg.grid_port << std::endl;
    }

    host_tcp_server machine_telemetry_server;
    machine_telemetry_server.set_verbose(cfg.verbose);
    if (!machine_telemetry_server.initialize(cfg.machine_port, "0.0.0.0")) {
        if (cfg.verbose) {
            std::cerr << "[ERROR] Failed to initialize machine TCP server on port " << cfg.machine_port << std::endl;
        }
        return 1;
    }
    if (cfg.verbose) {
        std::cout << "[INIT] Machine TCP server listening on port " << cfg.machine_port << std::endl;
    }

    if (cfg.verbose) {
        std::cout << "\n======================================" << std::endl;
        std::cout << "  TCP Servers Ready - You can connect monitors now" << std::endl;
        std::cout << "======================================" << std::endl;
        std::cout << "Grid telemetry:    localhost:" << cfg.grid_port << std::endl;
        std::cout << "Machine telemetry: localhost:" << cfg.machine_port << std::endl;
        std::cout << "\nWaiting 3 seconds for monitor connections..." << std::endl;
        std::cout << "======================================\n" << std::endl;

        // Give time for monitors to connect
        std::this_thread::sleep_for(std::chrono::seconds(3));
    }

    // ==================== Telemetry Lambda Runner (50µs = 20 kHz) ====================
    if (cfg.verbose) {
        std::cout << "[INIT] Registering telemetry lambda runner (20 kHz / 50 µs)..." << std::endl;
    }
    constexpr double telemetry_period = 50e-6;  // 50 microseconds
    simulator.register_lambda_runner(telemetry_period, [&]() {
        // TCP server processing
        grid_telemetry_server.process();
        machine_telemetry_server.process();

        // Grid-side network communication
        handle_grid_telemetry(grid_telemetry_server, grid_network_infra);

        // Machine-side network communication
        handle_machine_telemetry(machine_telemetry_server, machine_network_infra);
    });

    // ==================== Prepare System for Startup (wait for user start command) ====================
    if (cfg.verbose) {
        std::cout << "[INIT] Preparing system for startup (waiting for start command)..." << std::endl;
        std::cout << "[CONFIG] Configuration:" << std::endl;
        std::cout << "         - Initial speed: 0.01 pu (near standstill)" << std::endl;
        std::cout << "         - Virtual frequency: 0 → 0.1 pu in 10s" << std::endl;
        std::cout << "         - Virtual torque: 0 → 0.3 pu in 1s" << std::endl;
        std::cout << "         - Speed rate limit: 20 s/pu (0.05 pu/s)" << std::endl;
        std::cout << "         - Pre-fluxing: 1.0s" << std::endl;
    }

    // Use the shared preparation procedure (configures plant and control, but does NOT start)
    auto startup_config = yeet::application::sfc::integration::create_default_startup_config();
    yeet::application::sfc::integration::prepare_for_startup(simulator_builder, simulator, startup_config);

    if (cfg.verbose) {
        std::cout << "[INIT] System prepared - auxiliaries ON, MCB closed" << std::endl;
        std::cout << "[INIT] State machine: OPERATION_OFF (waiting for start command)" << std::endl;

        // ==================== START REAL-TIME CONTROL LOOP ====================
        std::cout << "\n======================================" << std::endl;
        std::cout << "  Initialization Complete - Waiting for Start Command" << std::endl;
        std::cout << "======================================" << std::endl;
        std::cout << "[READY] Control loop running at " << cfg.frequency_hz << " Hz" << std::endl;
        std::cout << "[READY] Grid telemetry:    localhost:" << cfg.grid_port << std::endl;
        std::cout << "[READY] Machine telemetry: localhost:" << cfg.machine_port << std::endl;
        std::cout << "[READY] Send 'start_command = true' via telemetry to begin" << std::endl;
        std::cout << "[READY] Press Ctrl+C to stop" << std::endl;
        std::cout << "======================================" << std::endl;
    }

    // Start control loop AFTER initialization
    control_loop.start();

    std::uint32_t cycle_count = 0;
    const std::uint32_t stats_interval = static_cast<std::uint32_t>(cfg.frequency_hz * 10.0f);  // Every 10 seconds

    // Task I/O structures for both controllers (used by network and direct access)
    task_outputs_t grid_outputs{};
    task_outputs_t machine_outputs{};

    // ==================== MAIN CONTROL LOOP - Unified Simulator Architecture ====================
    // Uses yeet::simulator::run_single_tick() for real-time execution without data logging
    // The simulator manages multi-rate execution (plant, 200µs, 1ms, 10ms)
    // Runs as fast as possible (no real-time pacing)

    while (control_loop.is_running() && !shutdown_requested.load()) {
        // Wait for next cycle at configured frequency
        if (!control_loop.wait_for_next_cycle()) {
            break;
        }

        // Get current cycle number and timestamp
        cycle_count = control_loop.get_cycle_number();

        // ========== UNIFIED SIMULATOR TICK ==========
        // Executes one tick of simulation: calls all registered runners at their scheduled times
        // Runners execute at their own sampling rates (plant, 200µs, 1ms, 10ms, telemetry at 50µs)
        // No data logging or checkers - optimized for real-time host execution
        simulator.run_single_tick();

        // Mark end of cycle for timing measurement
        control_loop.mark_cycle_end();

        // Print statistics periodically
        if (cfg.verbose && (cycle_count % stats_interval == 0)) {
            timing_statistics stats = control_loop.get_statistics();
            std::size_t grid_queue = grid_telemetry_server.get_send_queue_size();
            std::size_t machine_queue = machine_telemetry_server.get_send_queue_size();

            std::cout << "[STATS] Cycles: " << stats.total_cycles << " | Avg: " << stats.average_cycle_time_ms
                      << " ms | Max: " << stats.max_cycle_time_ms << " ms"
                      << " | Overruns: " << stats.overrun_count
                      << " | Grid: " << (grid_telemetry_server.is_client_connected() ? "CONNECTED" : "disconnected")
                      << " (queue: " << grid_queue << " bytes)"
                      << " | Machine: "
                      << (machine_telemetry_server.is_client_connected() ? "CONNECTED" : "disconnected")
                      << " (queue: " << machine_queue << " bytes)" << std::endl;

            if (cfg.verbose && (grid_telemetry_server.get_queue_overflow_count() > 0 ||
                                   machine_telemetry_server.get_queue_overflow_count() > 0)) {
                std::cout << "[WARNING] Queue overflows - Grid: " << grid_telemetry_server.get_queue_overflow_count()
                          << " | Machine: " << machine_telemetry_server.get_queue_overflow_count() << std::endl;
            }
        }
    }

    // Shutdown
    if (cfg.verbose) {
        std::cout << "\n[SHUTDOWN] Stopping control loop..." << std::endl;
    }
    control_loop.stop();

    if (cfg.verbose) {
        std::cout << "[SHUTDOWN] Closing grid TCP server..." << std::endl;
    }
    grid_telemetry_server.shutdown();

    if (cfg.verbose) {
        std::cout << "[SHUTDOWN] Closing machine TCP server..." << std::endl;
    }
    machine_telemetry_server.shutdown();

    // Print final statistics
    if (cfg.verbose) {
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
    }

    return 0;
}

#endif  // HOST_BUILD
