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
            cfg.verbose = true;
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
    std::cout << "Telemetry rate:    20000 Hz (50 µs)" << std::endl;
    std::cout << "Real-time pacing:  OFF (as fast as possible)" << std::endl;
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
    // DISABLED: Speed generator replaced by startup_sequencer
    // bool run_creep_speed = false;
    // bool run_purge_accel = false;
    // bool run_ignition_accel = false;
    // bool run_assist_accel = false;
    // bool hold_speed = false;
    // bool decelerate = false;
    // bool idle = true;
    bool exciter_ready = true;
    bool excitation_start_command = true;
    bool excitation_stop_command = false;
    bool startup_sequencer_start_command = false;
    bool startup_sequencer_stop_command = false;

    // Register lambda runner for external speed profile commands (SAME as test fixture)
    std::cout << "[INIT] Registering external I/O lambda runner (10ms)..." << std::endl;
    simulator.register_lambda_runner(10e-3, [&]() {
        simulator_builder.machine_side_control.external_ios.update_excitation_inputs(
            exciter_ready, excitation_start_command, excitation_stop_command);

        // DISABLED: Speed generator replaced by startup_sequencer
        // simulator_builder.machine_side_control.external_ios.update_speed_generator_inputs(
        //     run_creep_speed, run_purge_accel, run_ignition_accel, run_assist_accel, hold_speed, decelerate, idle);

        simulator_builder.machine_side_control.external_ios.update_startup_sequencer_inputs(
            startup_sequencer_start_command, startup_sequencer_stop_command);
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

    // ==================== Telemetry Lambda Runner (50µs = 20 kHz) ====================
    std::cout << "[INIT] Registering telemetry lambda runner (20 kHz / 50 µs)..." << std::endl;
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

    // ==================== Startup Sequencer Configuration (from StartupSequencerIntegrationTest) ====================
    std::cout << "[CONFIG] Configuring startup sequencer for soft-starter operation..." << std::endl;

    // Virtual frequency profile: 0 Hz → 5 Hz (0.1 pu) in 10 seconds
    simulator_builder.machine_side_control.startup_sequencer.get_parameters().virtual_frequency_initial =
        yeet::frequency_pu_t{0.0};
    simulator_builder.machine_side_control.startup_sequencer.get_parameters().virtual_frequency_final =
        yeet::frequency_pu_t{0.1};
    simulator_builder.machine_side_control.startup_sequencer.get_parameters().virtual_frequency_duration =
        yeet::duration_t{10.0};

    // Virtual torque profile: 0.0 pu → 0.3 pu in 1 second
    simulator_builder.machine_side_control.startup_sequencer.get_parameters().virtual_torque_initial =
        yeet::torque_pu_t{0.0};
    simulator_builder.machine_side_control.startup_sequencer.get_parameters().virtual_torque_final =
        yeet::torque_pu_t{0.3};
    simulator_builder.machine_side_control.startup_sequencer.get_parameters().virtual_torque_duration =
        yeet::duration_t{1.0};

    // Pre-fluxing duration before virtual startup
    simulator_builder.machine_side_control.excitation_controller.get_parameters().prefluxing_duration =
        yeet::duration_t{1.0f};

    // Speed reference rate limiter: 20 seconds per pu = 0.05 pu/s
    simulator_builder.machine_side_control.startup_sequencer.get_parameters()
        .speed_reference_rate_limit_seconds_per_pu = yeet::real_t{20.0f};
    simulator_builder.machine_side_control.startup_sequencer.get_parameters().speed_reference_rate_limit_enabled = true;

    // ==================== Plant Configuration (Soft-Starter Application) ====================
    std::cout << "[CONFIG] Configuring plant for soft-starter operation..." << std::endl;

    // Configure plant starting from standstill
    yeet::application::sfc::integration::test::variable_speed_config plant_config{};
    plant_config.initial_speed = yeet::speed_pu_t{0.0f};
    yeet::application::sfc::integration::test::configure_variable_speed_plant(simulator_builder.plant, plant_config);

    // Moderate load torque (typical for gas turbine compressor)
    yeet::mock_signal_input::write(simulator_builder.plant.get_inputs().load_torque, yeet::torque_pu_t(0.15f));

    // Configure torque limits for startup
    yeet::mock_signal_input::write(
        simulator_builder.machine_side_control.torque_controller.get_inputs().dynamic_max_torque_limit,
        yeet::torque_pu_t{1.0});
    yeet::mock_signal_input::write(
        simulator_builder.machine_side_control.torque_controller.get_inputs().dynamic_min_torque_limit,
        yeet::torque_pu_t{0.0});

    std::cout << "[CONFIG] Configuration complete:" << std::endl;
    std::cout << "         - Initial speed: 0.0 pu (standstill)" << std::endl;
    std::cout << "         - Load torque: 0.15 pu (moderate load)" << std::endl;
    std::cout << "         - Virtual frequency: 0 → 0.1 pu in 10s" << std::endl;
    std::cout << "         - Virtual torque: 0 → 0.3 pu in 1s" << std::endl;
    std::cout << "         - Speed rate limit: 20 s/pu (0.05 pu/s)" << std::endl;
    std::cout << "         - Pre-fluxing: 1.0s" << std::endl;

    // ==================== PRE-LOOP INITIALIZATION (from StartupSequencerIntegrationTest) ====================
    std::cout << "[INIT] Executing state machine startup sequence..." << std::endl;

    // State machine startup (EXACT from execute_startup_sequence())
    yeet::mock_signal_input::write(
        simulator_builder.grid_side_control.state_machine.get_inputs().auxiliaries_are_on, true);
    yeet::mock_signal_input::write(
        simulator_builder.grid_side_control.state_machine.get_inputs().breakers_are_on, true);
    simulator.simulate_for(0.2);

    yeet::mock_signal_input::write(simulator_builder.grid_side_control.state_machine.get_inputs().start_command, true);
    simulator.simulate_for(0.1);

    std::cout << "[INIT] State machine startup complete" << std::endl;

    // Verify state machine is operational
    bool pulse_releases = simulator_builder.grid_side_control.state_machine.get_outputs().release_pulses.read();
    if (!pulse_releases) {
        std::cerr << "[ERROR] State machine did not reach OPERATION_ON state!" << std::endl;
        return 1;
    }
    std::cout << "[INIT] State machine: OPERATION_ON (pulse releases active)" << std::endl;

    // ==================== Trigger Startup Sequencer ====================
    std::cout << "[INIT] Starting startup sequencer..." << std::endl;

    // Send start command to the startup sequencer (via external I/O override)
    simulator_builder.machine_side_control.startup_sequencer.get_parameters().override_enable_start_command.write(true);
    simulator_builder.machine_side_control.startup_sequencer.get_parameters().override_value_start_command.write(true);
    simulator.simulate_for(0.1);  // Allow command to propagate

    // Pre-flux duration
    std::cout << "[INIT] Pre-fluxing (1.0s)..." << std::endl;
    // simulator.simulate_for(1.0);

    // Wait for flux buildup and virtual startup to complete
    // Pre-flux (1s already done) + Virtual frequency ramp (10s) = 10s more
    std::cout << "[INIT] Virtual startup procedure (10.0s)..." << std::endl;
    // simulator.simulate_for(11.5);  // 10s virtual ramp + 0.5s settling

    // // Verify startup sequencer reached closed-loop operation
    // auto startup_state = simulator_builder.machine_side_control.startup_sequencer.get_outputs().current_state.read();
    // if (startup_state != yeet::application::sfc::startup_sequencer_state_t::CLOSED_LOOP_OPERATION) {
    //     std::cerr << "[ERROR] Startup sequencer did not reach CLOSED_LOOP_OPERATION state!" << std::endl;
    //     std::cerr << "         Current state: " << static_cast<int>(startup_state) << std::endl;
    //     return 1;
    // }

    // Set external speed reference to 1.0 pu
    std::cout << "[INIT] Setting speed reference to 1.0 pu..." << std::endl;
    simulator_builder.machine_side_control.external_ios.get_fixed_speed_reference().write(yeet::speed_pu_t{1.0});
    // simulator.simulate_for(0.1);

    // Ramp to 1.0 pu with rate limiting (20 s/pu = 20 seconds)
    // std::cout << "[INIT] Ramping to full speed (20s at 0.05 pu/s)..." << std::endl;
    // simulator.simulate_for(20.0);

    // // Check final speed
    // auto final_speed =
    //     simulator_builder.machine_side_control.machine_side_observer.get_outputs().estimated_speed.read();
    // std::cout << "[INIT] Initialization complete - Speed: " << final_speed.value() << " pu" << std::endl;

    // if (final_speed.value() < 0.85f) {
    //     std::cerr << "[WARNING] Speed may not have reached target (expected ~1.0 pu)" << std::endl;
    // }

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

            if (grid_telemetry_server.get_queue_overflow_count() > 0 ||
                machine_telemetry_server.get_queue_overflow_count() > 0) {
                std::cout << "[WARNING] Queue overflows - Grid: " << grid_telemetry_server.get_queue_overflow_count()
                          << " | Machine: " << machine_telemetry_server.get_queue_overflow_count() << std::endl;
            }
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
