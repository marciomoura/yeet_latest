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

    std::cout << "[INIT] Initializing timing loop..." << std::endl;
    host_timing_loop control_loop(cfg.frequency_hz);

    std::cout << "[INIT] Initializing plant model and control systems..." << std::endl;
    plant_complete_sfc_closed_loop_simulation_builder simulator;
    simulator.initialize();

    std::cout << "[INIT] Configuring state machine for automatic startup..." << std::endl;
    // Configure state machine for automatic startup to MODULATING state
    // Use parameter overrides to inject digital input signals
    // simulator.grid_side_control.external_ios.get_parameters().override_digital_inputs.write(false);
    // simulator.grid_side_control.external_ios.get_parameters().auxiliaries_are_on.write(true);
    // simulator.grid_side_control.external_ios.get_parameters().breakers_are_on.write(true);
    // simulator.grid_side_control.external_ios.get_parameters().start_command.write(false);
    // simulator.grid_side_control.external_ios.get_parameters().stop_command.write(false);
    // simulator.grid_side_control.external_ios.get_parameters().shutdown_request.write(false);

    // Apply the parameter overrides by calling update_digital_inputs (with dummy values - overrides will be used)
    simulator.grid_side_control.external_ios.update_digital_inputs(true, true, false, false, false);

    std::cout << "[INIT] Configuring speed generator to start at creep speed (Profile 1)..." << std::endl;
    // Configure machine-side speed generator to automatically activate creep speed
    // This allows the operator to start from a running drive rather than standstill
    // simulator.machine_side_control.external_ios.get_parameters().override_speed_generator_profiles.write(false);
    // simulator.machine_side_control.external_ios.get_parameters().idle.write(false);
    // simulator.machine_side_control.external_ios.get_parameters().run_creep_speed.write(true);
    // simulator.machine_side_control.external_ios.get_parameters().run_purge_accel.write(false);
    // simulator.machine_side_control.external_ios.get_parameters().run_ignition_accel.write(false);
    // simulator.machine_side_control.external_ios.get_parameters().run_assist_accel.write(false);
    // simulator.machine_side_control.external_ios.get_parameters().hold_speed.write(false);
    // simulator.machine_side_control.external_ios.get_parameters().decelerate.write(false);

    std::cout << "[INIT] Initializing embedded task interface (for telemetry system)..." << std::endl;
    // Initialize using the plant builder's control instances instead of the static instances
    grid_embedded_init(&simulator.grid_side_control);
    machine_embedded_init(&simulator.machine_side_control);

    // Create network infrastructure for telemetry (tied to the plant builder's control instances)
    std::cout << "[INIT] Creating network telemetry infrastructure..." << std::endl;
    grid_side_network_infrastructure grid_network_infra(simulator.grid_side_control);
    machine_side_network_infrastructure machine_network_infra(simulator.machine_side_control);

    std::cout << "[READY] Starting control loop at " << cfg.frequency_hz << " Hz" << std::endl;
    std::cout << "[READY] Grid telemetry:    localhost:" << cfg.grid_port << std::endl;
    std::cout << "[READY] Machine telemetry: localhost:" << cfg.machine_port << std::endl;
    std::cout << "[READY] Press Ctrl+C to stop" << std::endl;
    std::cout << "======================================" << std::endl;

    // ==================== Plant Model Configuration ====================
    std::cout << "[INIT] Configuring plant model..." << std::endl;

    // Configure plant starting from near-standstill (matches integration test setup)
    yeet::application::sfc::integration::test::variable_speed_config plant_config{};
    plant_config.initial_speed = yeet::speed_pu_t{0.001f};  // Near standstill to avoid high back-EMF
    plant_config.load_torque = yeet::torque_pu_t{0.20f};    // High load simulating static friction
    yeet::application::sfc::integration::test::configure_variable_speed_plant(simulator.plant, plant_config);
    // yeet::application::sfc::integration::test::configure_fixed_speed_plant(simulator.plant);

    // Configure Profile 1 (creep speed) with faster acceleration and explicit boost settings
    simulator.machine_side_control.speed_generator.get_parameters().profile_1_acceleration_time =
        yeet::duration_t{10.0f};
    simulator.machine_side_control.speed_generator.get_parameters().profile_1_target_speed = yeet::speed_pu_t{0.05f};
    simulator.machine_side_control.speed_generator.get_parameters().profile_1_initial_torque_boost =
        yeet::torque_pu_t{0.20f};
    simulator.machine_side_control.speed_generator.get_parameters().profile_1_boost_duration = yeet::duration_t{2.0f};

    std::cout << "[INIT] Plant model configuration complete" << std::endl;

    // Start control loop
    control_loop.start();

    std::uint32_t cycle_count = 0;
    const std::uint32_t stats_interval = static_cast<std::uint32_t>(cfg.frequency_hz * 10.0f);  // Every 10 seconds

    // Task I/O structures for both controllers
    task_outputs_t grid_outputs{};
    task_outputs_t machine_outputs{};

    // Plant model timing: 10µs per plant step, 200µs per control cycle
    constexpr int plant_steps_per_control_cycle = 20;  // 20 x 10µs = 200µs

    while (control_loop.is_running() && !shutdown_requested.load()) {
        // Wait for next cycle
        if (!control_loop.wait_for_next_cycle()) {
            break;
        }

        // Get current cycle number and timestamp
        cycle_count = control_loop.get_cycle_number();
        std::uint32_t timestamp_ms = static_cast<std::uint32_t>(control_loop.get_elapsed_time_ms());

        // Run plant model and hardware mocks 20 times per control cycle (10µs plant tick x 20 = 200µs)
        for (int plant_step = 0; plant_step < plant_steps_per_control_cycle; ++plant_step) {
            simulator.plant.run();
            simulator.grid_side_hardware_mock.run();
            simulator.machine_side_hardware_mock.run();
        }

        // Process TCP servers (accept clients, receive commands)
        grid_telemetry_server.process();
        machine_telemetry_server.process();

        // ==================== Prepare Grid-Side Inputs ====================
        // Convert plant outputs through hardware mock ADC conversion
        task_inputs_t grid_inputs{};
        grid_inputs.analog_inputs.channels[1] =
            simulator.grid_side_hardware_mock.get_outputs().v_ab_counts.read().value;
        grid_inputs.analog_inputs.channels[2] =
            simulator.grid_side_hardware_mock.get_outputs().v_bc_counts.read().value;
        grid_inputs.analog_inputs.channels[5] = simulator.grid_side_hardware_mock.get_outputs().i_a_counts.read().value;
        grid_inputs.analog_inputs.channels[7] = simulator.grid_side_hardware_mock.get_outputs().i_c_counts.read().value;
        grid_inputs.analog_inputs.valid = true;

        grid_inputs.digital_inputs.valid = true;

        // Fiber-optic RX initially empty (will be filled by machine-side TX data below)
        grid_inputs.fiber_optic_rx.data_ready = false;
        grid_inputs.fiber_optic_rx.link_ok = true;
        grid_inputs.fiber_optic_rx.payload_length = 0;

        // ==================== Prepare Machine-Side Inputs ====================
        task_inputs_t machine_inputs{};
        machine_inputs.analog_inputs.channels[1] =
            simulator.machine_side_hardware_mock.get_outputs().v_ab_counts.read().value;
        machine_inputs.analog_inputs.channels[2] =
            simulator.machine_side_hardware_mock.get_outputs().v_bc_counts.read().value;
        machine_inputs.analog_inputs.valid = true;

        machine_inputs.digital_inputs.valid = true;

        // Fiber-optic RX initially empty (will be filled by grid-side TX data below)
        machine_inputs.fiber_optic_rx.data_ready = false;
        machine_inputs.fiber_optic_rx.link_ok = true;
        machine_inputs.fiber_optic_rx.payload_length = 0;

        // Update external_ios digital inputs to apply parameter overrides
        // This must be called every cycle to keep the signal_outputs updated
        // Start command (third parameter) is activated after 1 second
        bool start_command = (control_loop.get_cycle_number() >= 10000);
        start_command = simulator.machine_side_control.get_is_field_established().read();

        simulator.grid_side_control.external_ios.update_digital_inputs(true, true, start_command, false, false);

        if (start_command) {
            // simulator.machine_side_control.get_speed_generator()
            //     .get_parameters()
            //     .override_speed_reference_generation.write(true);
            // simulator.machine_side_control.get_speed_generator().get_parameters().override_speed_reference.write(
            //     yeet::speed_pu_t{0.15f});
            // simulator.machine_side_control.get_speed_generator().get_parameters().override_max_torque_limit.write(
            //     yeet::torque_pu_t{0.4f});
            // simulator.machine_side_control.get_speed_generator().get_parameters().override_min_torque_limit.write(
            //     yeet::torque_pu_t{-0.4f});
            // //
            // simulator.machine_side_control.external_ios.get_parameters().override_speed_generator_profiles.write(true);
        }

        // ==================== Execute Grid-Side 200µs Control Task ====================
        grid_task_200us(&simulator.grid_side_control, &grid_inputs, &grid_outputs);

        // ==================== Execute Machine-Side 200µs Control Task ====================
        machine_task_200us(&simulator.machine_side_control, &machine_inputs, &machine_outputs);

        // ==================== 1ms Tasks and Fiber-Optic Exchange ====================
        if (cycle_count % 10 == 0) {
            // Exchange fiber-optic data between controllers BEFORE running 1ms tasks
            // Grid → Machine
            if (grid_outputs.fiber_optic_tx.transmit_request && grid_outputs.fiber_optic_tx.payload_length > 0) {
                machine_inputs.fiber_optic_rx.data_ready = true;
                machine_inputs.fiber_optic_rx.payload_length = grid_outputs.fiber_optic_tx.payload_length;
                std::memcpy(machine_inputs.fiber_optic_rx.payload, grid_outputs.fiber_optic_tx.payload,
                    sizeof(uint32_t) * grid_outputs.fiber_optic_tx.payload_length);
            }

            // Machine → Grid
            if (machine_outputs.fiber_optic_tx.transmit_request && machine_outputs.fiber_optic_tx.payload_length > 0) {
                grid_inputs.fiber_optic_rx.data_ready = true;
                grid_inputs.fiber_optic_rx.payload_length = machine_outputs.fiber_optic_tx.payload_length;
                std::memcpy(grid_inputs.fiber_optic_rx.payload, machine_outputs.fiber_optic_tx.payload,
                    sizeof(uint32_t) * machine_outputs.fiber_optic_tx.payload_length);
            }

            // Run 1ms tasks
            grid_task_1000us(&simulator.grid_side_control, &grid_inputs, &grid_outputs);
            machine_task_1000us(&simulator.machine_side_control, &machine_inputs, &machine_outputs);
        }

        // ==================== 10ms Tasks ====================
        if (cycle_count % 100 == 0) {
            // Update excitation inputs (exciter always ready, start commanded)
            simulator.machine_side_control.external_ios.update_excitation_inputs(true,  // exciter_ready
                true,                                                                   // excitation_start_command
                false);                                                                 // excitation_stop_command

            if (start_command) {
                // Update speed generator profile commands
                // For automatic creep speed startup:
                simulator.machine_side_control.external_ios.update_speed_generator_inputs(
                    true,    // run_creep_speed - ACTIVATE PROFILE 1
                    false,   // run_purge_accel
                    false,   // run_ignition_accel
                    false,   // run_assist_accel
                    false,   // hold_speed
                    false,   // decelerate
                    false);  // idle
            }
            else {
                simulator.machine_side_control.external_ios.update_speed_generator_inputs(
                    false,   // run_creep_speed - ACTIVATE PROFILE 1
                    false,   // run_purge_accel
                    false,   // run_ignition_accel
                    false,   // run_assist_accel
                    false,   // hold_speed
                    false,   // decelerate
                    false);  // idle
            }

            grid_task_10000us(&simulator.grid_side_control, &grid_inputs, &grid_outputs);
            machine_task_10000us(&simulator.machine_side_control, &machine_inputs, &machine_outputs);
        }

        // ==================== Feedback Outputs to Plant ====================
        // Convert grid-side abc pulses to 6-pulse format (p1-p6)
        // Grid-side 6-pulse bridge: P1=A+, P2=B+, P3=C+, P4=A-, P5=B-, P6=C-
        yeet::control::six_pulse_generator::pulses grid_pulses{};
        grid_pulses.p1 = grid_outputs.thyristor_pulses.a_plus;
        grid_pulses.p2 = grid_outputs.thyristor_pulses.b_plus;
        grid_pulses.p3 = grid_outputs.thyristor_pulses.c_plus;
        grid_pulses.p4 = grid_outputs.thyristor_pulses.a_minus;
        grid_pulses.p5 = grid_outputs.thyristor_pulses.b_minus;
        grid_pulses.p6 = grid_outputs.thyristor_pulses.c_minus;
        yeet::mock_signal_input::write(simulator.plant.get_inputs().grid_side_firing_pulses, grid_pulses);

        // Convert machine-side abc pulses to 6-pulse format (p1-p6)
        // Machine-side 6-pulse bridge: P1=A+, P2=B+, P3=C+, P4=A-, P5=B-, P6=C-
        yeet::control::six_pulse_generator::pulses machine_pulses{};
        machine_pulses.p1 = machine_outputs.thyristor_pulses.a_plus;
        machine_pulses.p2 = machine_outputs.thyristor_pulses.b_plus;
        machine_pulses.p3 = machine_outputs.thyristor_pulses.c_plus;
        machine_pulses.p4 = machine_outputs.thyristor_pulses.a_minus;
        machine_pulses.p5 = machine_outputs.thyristor_pulses.b_minus;
        machine_pulses.p6 = machine_outputs.thyristor_pulses.c_minus;
        yeet::mock_signal_input::write(simulator.plant.get_inputs().machine_side_firing_pulses, machine_pulses);

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

                grid_network(&grid_network_infra, &network_inputs, &network_outputs);

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

                grid_network(&grid_network_infra, &network_inputs, &network_outputs);

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

                machine_network(&machine_network_infra, &network_inputs, &network_outputs);

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

                machine_network(&machine_network_infra, &network_inputs, &network_outputs);

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
