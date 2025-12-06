#pragma once

#include <array>
#include <string_view>

#include "sfc_integration/sfc_grid_side_control.hpp"
#include "sfc_integration/sfc_machine_side_control.hpp"
#include "sfc_integration/sfc_plant_model_runner.hpp"
#include "yeet/signal.hpp"
#include "yeet_control/units.hpp"
#include "yeet_testing/parameter_collection.hpp"
#include "yeet_testing/simulator.hpp"

// Mocks
#include "sfc_grid_side_hardware_mock_runner.hpp"
#include "sfc_integration/plant_hardware_definition.hpp"  // hardware scaling parameter staging
#include "sfc_machine_side_hardware_mock_runner.hpp"

namespace yeet::application::sfc::integration {

// Parameter collection simplified - only plant model parameters
// Control classes manage their own parameters internally
using sfc_complete_sfc_closed_loop_parameters = yeet::parameter_collection<sfc_plant_model_datamodel::parameters>;

/**
 * @brief Complete SFC closed-loop simulation builder using control classes
 *
 * This configuration provides:
 * - Grid-side control: State machine, current controller, observer, modulator, measurements
 * - Machine-side control: Speed controller, machine controller, observer, modulator, measurements
 * - Plant model with augmented 6p-6p topology
 * - Time-based execution: 10ms, 1ms, 200µs sampling rates
 * - Cross-coupling between grid and machine side for operating mode and force commutation
 *
 * Signal Flow:
 * Plant → Control Classes → Observers → Controllers → Modulators → Pulse Generators → Plant
 */
class plant_complete_sfc_closed_loop_simulation_builder {
public:
    static constexpr std::string_view name = "PlantCompleteSFCClosedLoop";

    plant_complete_sfc_closed_loop_simulation_builder()
        : _per_unit_system{create_per_unit_system()},
          grid_side_control{},
          machine_side_control{},
          plant{_per_unit_system},
          grid_side_hardware_mock{duration_t{10e-6}},
          machine_side_hardware_mock{duration_t{10e-6}}
    {
        *grid_side_control.per_unit_system = *_per_unit_system;
        *machine_side_control.per_unit_system = *_per_unit_system;

        // Stage parameters for hardware mocks (ADC + transducers)
        plant_hardware::stage_grid_side_hardware_mock_parameters(grid_side_hardware_mock);
        plant_hardware::stage_machine_side_hardware_mock_parameters(machine_side_hardware_mock);
        // Apply transformer ratio scaling for hardware mocks so ADC counts reflect primary values
        plant_hardware::stage_transformer_ratios_for_grid_side(grid_side_hardware_mock, *_per_unit_system);
        plant_hardware::stage_transformer_ratios_for_machine_side(machine_side_hardware_mock, *_per_unit_system);

        // Stage measurement runner parameters (so runners configure ADC converters during initialize)
        plant_hardware::stage_grid_side_measurement_parameters(grid_side_control.grid_measurements);
        plant_hardware::stage_machine_side_measurement_parameters(machine_side_control.machine_measurements);
        // Apply inverse transformer ratio scaling in measurement reconstruction
        plant_hardware::stage_transformer_ratios_for_grid_side_measurements(
            grid_side_control.grid_measurements, *_per_unit_system);
        plant_hardware::stage_transformer_ratios_for_machine_side_measurements(
            machine_side_control.machine_measurements, *_per_unit_system);

        //  Configure plant model to use external excitation voltage control
        configure_plant_for_external_excitation_control();

        // Connect all signal flows between control classes and plant
        connect_all_components();
        mock_inputs();
    }

    void mock_inputs()
    {
        // Configure machine-side excitation control via datamodel overrides
        machine_side_control.external_ios.get_parameters().override_excitation_outputs.write(true);
        machine_side_control.external_ios.get_parameters().excitation_start_command.write(true);
        machine_side_control.external_ios.get_parameters().excitation_stop_command.write(false);

        // Note: Grid-side external I/O digital inputs are initialized with safe defaults
        // in sfc_grid_side_external_ios_datamodel constructor. This ensures signal storage
        // is allocated before the wire() DSL connects them to state machine inputs.
    }

    void initialize()
    {
        // Initialize in dependency order
        plant.initialize();
        grid_side_hardware_mock.initialize();
        machine_side_hardware_mock.initialize();
        grid_side_control.initialize();
        machine_side_control.initialize();
    }

    void register_applications(yeet::simulator& simulation)
    {
        // ==================== Complete Signal Flow Architecture ====================
        // The simulation executes at three distinct sampling rates with a clear feedback loop:
        //
        // PLANT SAMPLING (fastest, e.g., 10µs):
        //   Plant dynamics → Grid/Machine Hardware Mocks (ADC simulation + scaling)
        //
        // 200µs SAMPLING (5 kHz) - Measurement & Observation:
        //   Hardware Mock outputs → Measurement runners (voltage/current reconstruction)
        //   Measurements → Grid/Machine observers (speed/position estimation, filtering)
        //   Observers → Modulators & pulse generators (continuous updates)
        //
        // 1ms SAMPLING (1 kHz) - High-Speed Control & Fiber-Optic Cross-Link:
        //   Measurements → Current controllers (firing angle calculation)
        //   Fiber-optic TX: Encode local state (mode, currents, sync signals)
        //   Fiber-optic RX: Decode peer state (speed, voltage, state machine sync)
        //   Machine-side controller → current reference generation
        //
        // 10ms SAMPLING (100 Hz) - Low-Speed Control & Coordination:
        //   State machine (grid-side): Sequencing, fault detection, mode transitions
        //   Speed controller (machine-side): Ramp generation, acceleration profiles
        //   Torque limiter: Dynamic current limit calculation
        //
        // CLOSED LOOP FEEDBACK:
        //   Plant outputs (voltage, current) → Measurements (via mocks)
        //   Measurements → Controllers (cascade: speed → torque → current → firing angle)
        //   Controllers → Firing pulses → Plant power converters
        //   → Back to plant dynamics
        //
        // CROSS-COMMUNICATION:
        //   Grid ↔ Machine via fiber-optic protocol at 1ms:
        //   - Grid sends: mode, DC current, status flags, state ID, integrator freeze, PI reset request
        //   - Machine sends: speed estimate, current reference, voltage magnitude, force comm request
        //   - Keeps both controllers synchronized for stable LCI operation

        // Plant model runs at its own sampling time
        simulation.register_lambda_runner(plant.get_sampling_time(), [&]() {
            plant.run();
            grid_side_hardware_mock.run();
            machine_side_hardware_mock.run();
        });

        // 200µs sampling (5 kHz): Measurements, observers, modulators, pulse generators
        // This is the fastest control loop - handles real-time measurement processing
        simulation.register_lambda_runner(200e-6, [&]() {
            // ========== MEASUREMENT ACQUISITION (Mock ADCs + Transducers) ==========
            // Hardware mocks simulate the physical ADC converters and transducers
            // They receive real plant quantities and convert them to ADC counts
            // This step simulates: transformer scaling, ADC quantization, measurement delays

            // Grid-side measurements: Voltage (V_ab, V_bc) and current (I_a, I_c) in ADC counts
            // These are fed to the measurement runner for reconstruction in per-unit
            grid_side_control.get_external_ios().set_analog_inputs(
                grid_side_hardware_mock.get_outputs().v_ab_counts.read(),
                grid_side_hardware_mock.get_outputs().v_bc_counts.read(),
                grid_side_hardware_mock.get_outputs().i_a_counts.read(),
                grid_side_hardware_mock.get_outputs().i_c_counts.read());

            // Machine-side measurements: Voltage (V_ab, V_bc) in ADC counts
            // The plant provides 3-phase voltage, mocks convert to ADC counts
            auto debug_ab = machine_side_hardware_mock.get_outputs().v_ab_counts.read();
            auto debug_bc = machine_side_hardware_mock.get_outputs().v_bc_counts.read();
            machine_side_control.external_ios.set_analog_inputs(debug_ab, debug_bc);

            // ========== SIGNAL PROCESSING & OBSERVATION ==========
            // Measurement runners process raw ADC counts:
            //   - Reconstruct 3-phase voltages/currents from two measurements
            //   - Apply inverse transformer scaling to get primary-side quantities
            //   - Filter and condition signals
            //
            // Observers estimate non-measurable states:
            //   - Machine-side observer: Speed (rpm) and position (electrical angle)
            //     Uses voltage/current measurements and model equations
            //   - Grid-side observer: Phase angle and frequency tracking
            //
            // Output of this stage feeds the 1ms control loop with quality measurements

            machine_side_control.update_200us();
            grid_side_control.update_200us();
        });

        // 1ms sampling (1 kHz): Current controller, machine controller, operation mode monitor, fiber-optic exchange
        // High-speed inner control loop: Regulates DC link current (grid-side) and machine current (machine-side)
        simulation.register_lambda_runner(1e-3, [&]() {
            // ========== GRID-SIDE CONTROL UPDATE ==========
            // Grid-side current controller:
            //   Input: Measured grid current, current reference (from higher-level speed control)
            //   Calculates: Firing angle to regulate DC link current
            //   Output: Firing pulses for grid-side thyristor bridge
            grid_side_control.update_1ms();

            // ========== FIBER-OPTIC CROSS-LINK: GRID → MACHINE ==========
            // Grid sends its state to machine-side for synchronization
            // Encodes 9 signals: mode, DC current, status flags (5), state ID, integrator freeze, PI reset
            grid_side_control.encode_fiber_optic_tx_to_machine();

            // Machine receives grid state:
            //   - Operating mode: Determines machine control strategy
            //   - State ID: Synchronizes with grid-side state machine sequencing
            //   - Freeze integrator: When grid is frozen, machine must freeze too
            //   - PI reset: When grid modulator resets its PI, machine must reset its PI
            // This ensures tight synchronization between the two converters
            bool machine_comm_valid = machine_side_control.decode_fiber_optic_rx_from_grid(
                grid_side_control.get_fiber_optic_tx_payload(), grid_side_control.get_fiber_optic_tx_length());

            // Update protection system with communication status for watchdog monitoring
            machine_side_control.machine_protections.update_communication_status(machine_comm_valid);

            // ========== MACHINE-SIDE CONTROL UPDATE ==========
            // Machine-side inner control loop:
            //   Input: Machine measurements, speed reference (from 10ms speed controller), grid state
            //   Calculates: Firing angle to regulate machine current per speed error
            //   Output: Firing pulses for machine-side thyristor bridge
            machine_side_control.update_1ms();

            // ========== FIBER-OPTIC CROSS-LINK: MACHINE → GRID ==========
            // Machine sends its measurements and requests to grid
            // Encodes 4 signals: speed estimate (float), current ref (float), voltage magnitude (float), force comm
            // request
            machine_side_control.encode_fiber_optic_tx_to_grid();

            // Grid receives machine measurements:
            //   - Speed: Feedback for checking if machine is up to speed
            //   - Current reference: Tells grid the desired current magnitude
            //   - Voltage magnitude: Available machine voltage (affects current control limits)
            //   - Force comm request: Machine requests low-speed assist when load torque is high
            grid_side_control.decode_fiber_optic_rx_from_machine(
                machine_side_control.get_fiber_optic_tx_payload(), machine_side_control.get_fiber_optic_tx_length());
        });

        // 10ms sampling (100 Hz): State machine, speed controller
        // Outer control loop: Manages high-level sequencing and speed regulation
        simulation.register_lambda_runner(10e-3, [&]() {
            // ========== MACHINE-SIDE 10ms CONTROL ==========
            // Speed controller (outer loop for machine-side inner current control):
            //   Input: Measured speed (from 200µs observer)
            //   Calculates: Speed error, ramps current reference through acceleration profiles
            //   Prevents sudden current jumps, implements smooth speed profiles
            //   Output: Current reference → sent to 1ms inner control loop
            //
            // State machine (low-speed sequencing):
            //   May include: Creep speed, purge acceleration, ignition acceleration, assist acceleration
            machine_side_control.update_10ms();

            // ========== GRID-SIDE 10ms CONTROL ==========
            // State machine (main sequence orchestration):
            //   Manages the complete starting sequence:
            //   - Idle: Waiting for start command
            //   - DC ramping: Slowly increases DC link voltage
            //   - Synchronization: Waits for grid voltage to match machine back-EMF
            //   - Running: Normal acceleration toward synchronous speed
            //   - Synchronized: Running synchronized with grid
            //   - Fault/Trip: Handles abnormal conditions
            //
            //   Coordinates with machine-side state (via fiber-optic sync signals)
            //   For each state:
            //   - Updates operating mode (sent to machine in next 1ms cycle)
            //   - Determines if integrator should freeze (prevents integral windup)
            //   - Requests PI reset if needed (resynchronization after faults)
            //
            // Operation mode monitor:
            //   Watches the state machine and generates:
            //   - pulse_releases: Gate enable/disable signal
            //   - is_current_zero: Detection flag for transition decisions
            //   - Communicates these to machine-side via fiber-optic protocol
            grid_side_control.update_10ms();
        });
    }

    // Public members for direct access
    std::shared_ptr<sfc::sfc_per_unit_system> _per_unit_system;

    // Control subsystems
    sfc_grid_side_control grid_side_control;
    sfc_machine_side_control machine_side_control;

    // Hardware mocks
    sfc_grid_side_hardware_mock_runner grid_side_hardware_mock;
    sfc_machine_side_hardware_mock_runner machine_side_hardware_mock;

    // Plant model
    sfc_plant_model_runner plant;

    // External control signals for state machine
    signal_output<bool> auxiliaries_are_on;
    signal_output<bool> mcb_is_closed;
    signal_output<bool> start_command;
    signal_output<bool> stop_command;
    signal_output<bool> shutdown_request;
    signal_output<bool> is_current_extinguished;

private:
    static std::shared_ptr<sfc::sfc_per_unit_system> create_per_unit_system()
    {
        // 3 MVA per-unit system for Typhoon HIL testing
        // Grid side: 2090V RMS, 50Hz, DC link: 1036A, Machine: 1980V RMS, 3.0 MVA
        // Machine parameters match Typhoon synchronous machine model
        return std::make_shared<sfc::sfc_per_unit_system>(voltage_t(2090.0),  // Grid line-to-line voltage (V RMS)
            frequency_t(50.0),                                                // Grid frequency (Hz)
            current_t(1036.3),                                                // DC link current base (A) - for 3 MVA
            power_t(3.0e6),                                                   // Rated power (VA) - 3 MVA
            voltage_t(1980.0),                                                // Machine line-to-line voltage (V RMS)
            frequency_t(50.0),                                                // Machine base frequency (Hz)
            2                                                                 // Number of poles (2-pole, 3000 RPM)
        );
    }

    void configure_plant_for_external_excitation_control()
    {
        // Configure plant model for variable-speed operation with external excitation control
        // The excitation controller provides field current reference to regulate machine flux
        // This enables dynamic voltage control (E = Φ × ω) where both flux and speed vary
        plant.get_parameters().operation_mode = sfc_plant_model_datamodel::machine_operation_mode::variable_speed;
        plant.get_parameters().voltage_control_mode =
            sfc_plant_model_datamodel::machine_voltage_control_mode::variable_speed;
        plant.get_parameters().initial_speed = speed_pu_t{0.001f};  // Near-standstill initial condition
    }

    void connect_all_components()
    {
        // ==================== Plant ↔ Hardware Mocks Connection ====================
        // Hardware mocks simulate the ADC and transducer interfaces between plant and controllers
        // Flow: Plant outputs → Hardware Mock inputs (scales primary quantities to ADC counts)
        // Then: Hardware Mock outputs → Control inputs (ADC counts, subject to scaling)

        // Grid-side hardware mock: Receives 3-phase grid voltage and current from plant
        // Scales them according to transformer ratios and ADC resolution
        signals_connector::connect_output_to_input(
            plant.get_outputs().grid_voltage_abc, grid_side_hardware_mock.get_inputs().grid_voltage_abc);
        signals_connector::connect_output_to_input(
            plant.get_outputs().grid_current_abc, grid_side_hardware_mock.get_inputs().grid_current_abc);

        // Machine-side hardware mock: Receives 3-phase machine voltage from plant
        // Scales according to transformer ratios and ADC resolution
        signals_connector::connect_output_to_input(
            plant.get_outputs().machine_voltage_abc, machine_side_hardware_mock.get_inputs().machine_voltage_abc);

        // ==================== Control Outputs → Plant Inputs ====================
        // Controllers generate firing pulses and excitation commands, which drive the power converters
        // Flow: Control logic → Modulators → Pulse Generators → Plant power converters

        // Grid-side firing pulses: Controls the grid-side 6-pulse thyristor converter
        // Determines when grid-side thyristors are gated on/off
        signals_connector::connect_output_to_input(
            grid_side_control.get_firing_pulses(), plant.get_inputs().grid_firing_pulses);

        // Machine-side firing pulses: Controls the machine-side 6-pulse thyristor converter
        // Determines when machine-side thyristors are gated on/off
        signals_connector::connect_output_to_input(
            machine_side_control.machine_side_pulse_generator.get_outputs().firing_pulses,
            plant.get_inputs().machine_firing_pulses);

        // Excitation control: Connect field current reference from excitation controller to plant model
        // The plant model contains an integrated field current regulator that converts the reference
        // to exciter voltage using PI control with realistic field circuit L/R dynamics
        signals_connector::connect_output_to_input(
            machine_side_control.get_excitation_controller().get_outputs().field_current_reference,
            plant.get_inputs().field_current_reference);

        // ==================== External Disturbances → Plant ====================
        // Note: Load torque is now configured via plant.get_parameters().constant_torque parameter
        // instead of a signal connection. This allows runtime configuration via telemetry interface.
    }
};

}  // namespace yeet::application::sfc::integration
