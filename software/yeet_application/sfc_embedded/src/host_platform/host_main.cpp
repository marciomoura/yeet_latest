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

#include "sfc_embedded/host_platform/host_io_mocks.hpp"
#include "sfc_embedded/host_platform/host_tcp_server.hpp"
#include "sfc_embedded/host_platform/host_timing.hpp"
#include "sfc_embedded/network_telemetry.hpp"

// TODO: Include actual SFC control headers when integration complete
// #include "sfc_embedded/grid_side_tasks.h"
// #include "sfc_embedded/telemetry_manager.h"

#include <atomic>
#include <csignal>
#include <cstring>
#include <iostream>

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

    // TODO: Initialize actual SFC control modules here
    // grid_side_controller controller;
    // controller.initialize();

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

        // TODO: Execute actual control code here
        // controller.update(adc, dio_in);
        // mock_dio_outputs dio_out = controller.get_outputs();

        // Write mock outputs
        mock_dio_outputs dio_out;  // Placeholder
        write_mock_dio(dio_out);

        // Send telemetry packet if client connected
        if (telemetry_server.is_client_connected()) {
            using namespace yeet::application::sfc::telemetry;

            // Create test telemetry packet
            grid_side_telemetry_packet packet{};

            // Fill header
            packet.header.packet_type = static_cast<std::uint8_t>(packet_type::grid_side_telemetry);
            packet.header.protocol_version = protocol_version;
            packet.header.timestamp_ms = timestamp_ms;
            packet.header.sequence_number = cycle_count;
            packet.header.flags = telemetry_flags::measurements | telemetry_flags::observer |
                                  telemetry_flags::modulator | telemetry_flags::controller |
                                  telemetry_flags::state_machine;

            // Fill measurement data (from mock I/O)
            packet.measurements.grid_voltage_a_pu = adc.grid_voltage_a_pu;
            packet.measurements.grid_voltage_b_pu = adc.grid_voltage_b_pu;
            packet.measurements.grid_voltage_c_pu = adc.grid_voltage_c_pu;
            packet.measurements.grid_voltage_magnitude_pu = 1.0f;
            packet.measurements.dc_link_current_pu = adc.dc_link_current_pu;

            // Fill observer data (test values)
            packet.observer.estimated_frequency_pu = 1.0f;  // 50 Hz nominal
            packet.observer.estimated_angle_rad = 0.0f;

            // Fill modulator data (test values)
            packet.modulator.firing_angle_reference_rad = 0.0f;
            packet.modulator.firing_angle_actual_rad = 0.0f;
            packet.modulator.force_commutation_ack = 0;

            // Fill controller data (test values)
            packet.controller.current_error_pu = 0.0f;
            packet.controller.controller_output_pu = 0.0f;
            packet.controller.operating_mode = 0;
            packet.controller.controller_saturated = 0;
            packet.controller.freeze_integrator = 0;

            // Fill state machine data (test values)
            packet.state_machine.current_state = 0;  // INIT state
            packet.state_machine.pulse_releases = 0;

            // Encode and send (encode function sets payload_length automatically)
            std::uint8_t buffer[1024];
            std::size_t size = encode_grid_side_telemetry(packet, buffer);
            telemetry_server.send_data(buffer, size);
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
