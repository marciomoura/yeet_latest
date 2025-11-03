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
#include <csignal>
#include <cstring>
#include <iostream>

// Control task interface
#include "sfc_embedded/grid_side_tasks.hpp"
#include "sfc_embedded/machine_side_tasks.hpp"

// Host platform layer
#include "sfc_embedded/host_platform/host_tcp_server.hpp"
#include "sfc_embedded/host_platform/host_timing.hpp"
#include "sfc_embedded/task_io.h"

// Plant model integration
#include "../../sfc_integration/tests/sfc_plant_configuration.hpp"
#include "sfc_integration/plant_complete_sfc_closed_loop_simulation_builder.hpp"

// Signal utilities for test/simulation
#include "yeet/signal.hpp"  // For mock_signal_input

// Telemetry
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
    int grid_port = 7000;          // Telemetry port for grid-side
    int machine_port = 7001;       // Telemetry port for machine-side
    float frequency_hz = 5000.0f;  // Control frequency (5 kHz recommended for plant simulation)
    bool verbose = false;          // Verbose logging
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
            std::cout << "Usage: sfc_host_simulator_with_plant [OPTIONS]\n"
                      << "Options:\n"
                      << "  --grid-port PORT       Grid-side telemetry TCP port (default: 7000)\n"
                      << "  --machine-port PORT    Machine-side telemetry TCP port (default: 7001)\n"
                      << "  --frequency HZ         Control loop frequency (default: 5000.0 Hz)\n"
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
            telemetry_server.send_data(network_outputs.network_tx.tx_buffer, network_outputs.network_tx.tx_length);
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
            telemetry_server.send_data(network_outputs.network_tx.tx_buffer, network_outputs.network_tx.tx_length);
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
            telemetry_server.send_data(network_outputs.network_tx.tx_buffer, network_outputs.network_tx.tx_length);
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
            telemetry_server.send_data(network_outputs.network_tx.tx_buffer, network_outputs.network_tx.tx_length);
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

    std::cout << "======================================" << std::endl;
    std::cout << "  SFC Dual-Controller with Plant Model" << std::endl;
    std::cout << "======================================" << std::endl;
    std::cout << "Grid telemetry:    localhost:" << cfg.grid_port << std::endl;
    std::cout << "Machine telemetry: localhost:" << cfg.machine_port << std::endl;
    std::cout << "Control frequency: " << cfg.frequency_hz << " Hz" << std::endl;
    std::cout << "Verbose logging:   " << (cfg.verbose ? "ON" : "OFF") << std::endl;
    std::cout << "Plant model:       Complete 6p-6p topology" << std::endl;
    std::cout << "======================================" << std::endl;

    // Install signal handler for graceful shutdown
    std::signal(SIGINT, signal_handler);

    // Initialize subsystems
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

    std::cout << "[INIT] Initializing plant model and control systems..." << std::endl;
    plant_complete_sfc_closed_loop_simulation_builder simulator_builder;

    // Use unified simulator architecture for real-time tick-based execution
    std::cout << "[INIT] Creating unified simulator for real-time execution..." << std::endl;
    yeet::simulator simulator;

    // Register all applications with the simulator
    // This sets up multi-rate execution: 10ms (state machine, speed control),
    // 1ms (current control, fiber-optics), 200µs (measurements, observers, modulators)
    simulator_builder.register_applications(simulator);

    // ==================== External I/O Lambda Runner (EXACT from test fixture) ====================
    // External input command variables (simulates external digital I/O)
    bool run_creep_speed = false;
    bool run_purge_accel = false;
    bool run_ignition_accel = false;
    bool run_assist_accel = false;
    bool hold_speed = false;
    bool decelerate = false;
    bool idle = true;
    bool exciter_ready = true;
    bool excitation_start_command = true;
    bool excitation_stop_command = false;

    // Register lambda runner for external speed profile commands (SAME as test fixture)
    std::cout << "[INIT] Registering external I/O lambda runner (10ms)..." << std::endl;
    simulator.register_lambda_runner(10e-3, [&]() {
        simulator_builder.machine_side_control.external_ios.update_excitation_inputs(
            exciter_ready, excitation_start_command, excitation_stop_command);

        simulator_builder.machine_side_control.external_ios.update_speed_generator_inputs(
            run_creep_speed, run_purge_accel, run_ignition_accel, run_assist_accel, hold_speed, decelerate, idle);
    });

    // Initialize the system (SAME as test fixture SetUp())
    std::cout << "[INIT] Initializing control system..." << std::endl;
    simulator_builder.initialize();

    std::cout << "[INIT] Initializing timing loop..." << std::endl;
    host_timing_loop control_loop(cfg.frequency_hz);

    std::cout << "[INIT] Initializing embedded task interface (for telemetry system)..." << std::endl;
    // Initialize using the plant builder's control instances instead of the static instances
    // grid_embedded_init(&simulator_builder.grid_side_control);
    // machine_embedded_init(&simulator_builder.machine_side_control);

    // Create network infrastructure for telemetry (tied to the plant builder's control instances)
    std::cout << "[INIT] Creating network telemetry infrastructure..." << std::endl;
    grid_side_network_infrastructure grid_network_infra(simulator_builder.grid_side_control);
    machine_side_network_infrastructure machine_network_infra(simulator_builder.machine_side_control);

    // ==================== Telemetry Lambda Runner (1ms) ====================
    std::cout << "[INIT] Registering telemetry lambda runner (1ms)..." << std::endl;
    simulator.register_lambda_runner(50e-6, [&]() {
        // TCP server processing
        grid_telemetry_server.process();
        machine_telemetry_server.process();

        // Grid-side network communication
        handle_grid_telemetry(grid_telemetry_server, grid_network_infra);

        // Machine-side network communication
        handle_machine_telemetry(machine_telemetry_server, machine_network_infra);
    });

    // ==================== Plant Configuration (EXACT from LowSpeedStartupWithBreakawayBoostTest) ====================
    std::cout << "[CONFIG] Configuring plant for low-speed startup with breakaway boost..." << std::endl;

    // Configure plant at near-standstill (simulates sticky bearings)
    yeet::application::sfc::integration::test::variable_speed_config plant_config{};
    plant_config.initial_speed = yeet::speed_pu_t{0.001f};
    yeet::application::sfc::integration::test::configure_variable_speed_plant(simulator_builder.plant, plant_config);

    // Configure Profile 1 with faster acceleration and explicit boost settings
    simulator_builder.machine_side_control.speed_generator.get_parameters().profile_1_acceleration_time =
        yeet::duration_t{15.0f};
    simulator_builder.machine_side_control.speed_generator.get_parameters().profile_1_initial_torque_boost =
        yeet::torque_pu_t{0.20f};
    simulator_builder.machine_side_control.speed_generator.get_parameters().profile_1_boost_duration =
        yeet::duration_t{2.0f};

    // High load simulating static friction
    yeet::mock_signal_input::write(simulator_builder.plant.get_inputs().load_torque_pu, yeet::torque_pu_t(0.20f));

    std::cout << "[CONFIG] Configuration complete:" << std::endl;
    std::cout << "         - Initial speed: 0.001 pu (near-standstill)" << std::endl;
    std::cout << "         - Load torque: 0.20 pu (static friction)" << std::endl;
    std::cout << "         - Profile 1 acceleration: 15.0s" << std::endl;
    std::cout << "         - Breakaway boost: 0.20 pu for 2.0s" << std::endl;

    // ==================== PRE-LOOP INITIALIZATION (EXACT from test) ====================
    std::cout << "[INIT] Executing state machine startup sequence..." << std::endl;

    // State machine startup (EXACT from execute_startup_sequence())
    yeet::mock_signal_input::write(
        simulator_builder.grid_side_control.state_machine.get_inputs().auxiliaries_are_on, true);
    yeet::mock_signal_input::write(
        simulator_builder.grid_side_control.state_machine.get_inputs().breakers_are_on, true);
    simulator.simulate_for(0.2);

    yeet::mock_signal_input::write(simulator_builder.grid_side_control.state_machine.get_inputs().start_command, true);
    simulator.simulate_for(0.1);

    // Enable speed control and set voltage mode (EXACT from test)
    yeet::mock_signal_input::write(
        simulator_builder.machine_side_control.speed_controller.get_inputs().enable_speed_control, true);
    simulator_builder.plant.get_plant_model().set_voltage_control_mode(
        yeet::application::sfc::plant_model_voltage_control_mode::speed_dependent);

    std::cout << "[INIT] State machine startup complete" << std::endl;

    // Verify state machine is operational
    bool pulse_releases = simulator_builder.grid_side_control.state_machine.get_outputs().pulse_releases.read();
    if (!pulse_releases) {
        std::cerr << "[ERROR] State machine did not reach OPERATION_ON state!" << std::endl;
        return 1;
    }
    std::cout << "[INIT] State machine: OPERATION_ON (pulse releases active)" << std::endl;

    // ==================== Activate Profile 1 (Creep) ====================
    std::cout << "[INIT] Activating Profile 1 (Creep speed)..." << std::endl;

    // EXACT from test
    idle = false;
    simulator.simulate_for(0.1);

    run_creep_speed = true;
    simulator.simulate_for(0.1);

    std::cout << "[INIT] Profile 1 activated" << std::endl;

    // ==================== Simulate Until Steady State ====================
    std::cout << "[INIT] Simulating to steady state (20 seconds)..." << std::endl;

    // Simulate through boost phase and acceleration
    simulator.simulate_for(20.0);

    // Check final speed
    auto final_speed =
        simulator_builder.machine_side_control.machine_side_observer.get_outputs().estimated_speed.read();
    std::cout << "[INIT] Initialization complete - Speed: " << final_speed.value() << " pu" << std::endl;

    if (final_speed.value() < 0.04f) {
        std::cerr << "[WARNING] Speed may not have reached target (expected ~0.05 pu)" << std::endl;
    }

    // ==================== START REAL-TIME CONTROL LOOP ====================
    std::cout << "\n======================================" << std::endl;
    std::cout << "  Initialization Complete - Starting Real-Time Operation" << std::endl;
    std::cout << "======================================" << std::endl;
    std::cout << "[READY] Starting control loop at " << cfg.frequency_hz << " Hz" << std::endl;
    std::cout << "[READY] Grid telemetry:    localhost:" << cfg.grid_port << std::endl;
    std::cout << "[READY] Machine telemetry: localhost:" << cfg.machine_port << std::endl;
    std::cout << "[READY] Press Ctrl+C to stop" << std::endl;
    std::cout << "======================================" << std::endl;

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

    while (control_loop.is_running() && !shutdown_requested.load()) {
        // Wait for next cycle at configured frequency
        if (!control_loop.wait_for_next_cycle()) {
            break;
        }

        // Get current cycle number and timestamp
        cycle_count = control_loop.get_cycle_number();
        std::uint32_t timestamp_ms = static_cast<std::uint32_t>(control_loop.get_elapsed_time_ms());

        // ========== UNIFIED SIMULATOR TICK ==========
        // Executes one tick of simulation: calls all registered runners at their scheduled times
        // Runners execute at their own sampling rates (plant, 200µs, 1ms, 10ms)
        // No data logging or checkers - optimized for real-time host execution
        simulator.run_single_tick();

        // Mark end of cycle for timing measurement
        control_loop.mark_cycle_end();

        // Print statistics periodically
        if (cfg.verbose && (cycle_count % stats_interval == 0)) {
            timing_statistics stats = control_loop.get_statistics();
            std::cout << "[STATS] Cycles: " << stats.total_cycles << " | Avg: " << stats.average_cycle_time_ms
                      << " ms | Max: " << stats.max_cycle_time_ms << " ms"
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
