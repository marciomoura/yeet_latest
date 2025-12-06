#pragma once

#include "sfc_integration/plant_complete_sfc_closed_loop_simulation_builder.hpp"
#include "sfc_plant_configuration.hpp"
#include "yeet/signal.hpp"
#include "yeet_control/units.hpp"
#include "yeet_testing/simulator.hpp"

/**
 * @file sfc_startup_procedure.hpp
 * @brief Common startup procedures for SFC integration tests and host simulator
 *
 * This file contains reusable startup sequence functions that are shared between
 * the host simulator (host_main.cpp) and integration tests. The goal is to ensure
 * consistency between the production startup sequence and test validation.
 */

namespace yeet::application::sfc::integration {

/**
 * @brief Configuration parameters for the startup-to-nominal-speed procedure
 */
struct startup_to_nominal_config {
    /// Virtual frequency profile: initial frequency [pu]
    frequency_pu_t virtual_frequency_initial{0.0f};

    /// Virtual frequency profile: final frequency [pu]
    frequency_pu_t virtual_frequency_final{0.1f};

    /// Virtual frequency profile: ramp duration [s]
    duration_t virtual_frequency_duration{10.0f};

    /// Virtual torque profile: initial torque [pu]
    torque_pu_t virtual_torque_initial{0.0f};

    /// Virtual torque profile: final torque [pu]
    torque_pu_t virtual_torque_final{0.3f};

    /// Virtual torque profile: ramp duration [s]
    duration_t virtual_torque_duration{1.0f};

    /// Pre-fluxing duration before virtual startup [s]
    duration_t prefluxing_duration{1.0f};

    /// Observer settling time [s] (must be <= prefluxing_duration)
    duration_t observer_settling_time{0.5f};

    /// Speed reference rate limit [s/pu] (e.g., 20 s/pu = 0.05 pu/s)
    real_t speed_reference_rate_limit_seconds_per_pu{20.0f};

    /// Enable speed reference rate limiting
    bool speed_reference_rate_limit_enabled{true};

    /// Load torque applied after pre-flux [pu]
    torque_pu_t load_torque{0.15f};

    /// Dynamic maximum torque limit [pu]
    torque_pu_t dynamic_max_torque_limit{1.0f};

    /// Dynamic minimum torque limit [pu]
    torque_pu_t dynamic_min_torque_limit{0.0f};

    /// Target speed reference [pu]
    speed_pu_t target_speed_reference{1.0f};

    /// Initial plant speed [pu]
    speed_pu_t initial_plant_speed{0.01f};

    /// Duration for virtual startup procedure after pre-flux [s]
    double virtual_startup_duration{15.0};

    /// Duration for speed ramp from closed-loop to target [s]
    double speed_ramp_duration{20.0};

    /// Duration for stabilization at target speed [s]
    double stabilization_duration{5.0};
};

/**
 * @brief Default startup configuration for soft-starter applications
 */
inline startup_to_nominal_config create_default_startup_config() { return startup_to_nominal_config{}; }

/**
 * @brief Result of the startup-to-nominal-speed procedure
 */
struct startup_result {
    /// True if startup completed successfully
    bool success{false};

    /// True if state machine reached OPERATION_ON
    bool state_machine_operational{false};

    /// True if startup sequencer reached CLOSED_LOOP_OPERATION
    bool closed_loop_reached{false};

    /// True if reference frame is ESTIMATED (observer-based)
    bool reference_frame_estimated{false};

    /// True if target speed was approximately reached
    bool target_speed_reached{false};

    /// Final estimated speed [pu]
    speed_pu_t final_speed{0.0f};

    /// Final speed reference [pu]
    speed_pu_t final_speed_reference{0.0f};
};

/**
 * @brief Initialize mock signal inputs to safe (off) state
 *
 * This initializes torque controller inputs that need mock signal storage.
 * Note: State machine inputs are now initialized via external_ios.set_digital_inputs()
 * in the plant_complete_sfc_closed_loop_simulation_builder constructor.
 *
 * @param system The plant-complete SFC closed-loop simulation builder
 */
inline void initialize_mock_signal_inputs(plant_complete_sfc_closed_loop_simulation_builder& system)
{
    // Torque controller inputs - initialize to safe defaults
    mock_signal_input::write(
        system.machine_side_control.torque_controller.get_inputs().dynamic_max_torque_limit, torque_pu_t{1.0f});
    mock_signal_input::write(
        system.machine_side_control.torque_controller.get_inputs().dynamic_min_torque_limit, torque_pu_t{0.0f});
}

/**
 * @brief Configure startup sequencer parameters
 *
 * @param system The plant-complete SFC closed-loop simulation builder
 * @param config Startup configuration parameters
 */
inline void configure_startup_sequencer(
    plant_complete_sfc_closed_loop_simulation_builder& system, const startup_to_nominal_config& config)
{
    // Virtual frequency profile
    system.machine_side_control.startup_sequencer.get_parameters().virtual_frequency_initial =
        config.virtual_frequency_initial;
    system.machine_side_control.startup_sequencer.get_parameters().virtual_frequency_final =
        config.virtual_frequency_final;
    system.machine_side_control.startup_sequencer.get_parameters().virtual_frequency_duration =
        config.virtual_frequency_duration;

    // Virtual torque profile
    system.machine_side_control.startup_sequencer.get_parameters().virtual_torque_initial =
        config.virtual_torque_initial;
    system.machine_side_control.startup_sequencer.get_parameters().virtual_torque_final = config.virtual_torque_final;
    system.machine_side_control.startup_sequencer.get_parameters().virtual_torque_duration =
        config.virtual_torque_duration;

    // Pre-fluxing duration
    system.machine_side_control.excitation_controller.get_parameters().prefluxing_duration = config.prefluxing_duration;

    // Observer settling time
    system.machine_side_control.startup_sequencer.get_parameters().flying_restart_observer_settling_time =
        config.observer_settling_time;

    // Speed reference rate limiter
    system.machine_side_control.startup_sequencer.get_parameters().speed_reference_rate_limit_seconds_per_pu =
        config.speed_reference_rate_limit_seconds_per_pu;
    system.machine_side_control.startup_sequencer.get_parameters().speed_reference_rate_limit_enabled =
        config.speed_reference_rate_limit_enabled;
}

/**
 * @brief Configure plant model for soft-starter operation
 *
 * @param system The plant-complete SFC closed-loop simulation builder
 * @param config Startup configuration parameters
 */
inline void configure_plant_for_startup(
    plant_complete_sfc_closed_loop_simulation_builder& system, const startup_to_nominal_config& config)
{
    // Configure topology to use internal model (matches closed_loop tests)
    system.plant.get_parameters().topology.write(sfc_plant_model_datamodel::topology_type::internal);

    // Configure plant for gas turbine soft-starter application (matches closed_loop tests)
    auto gas_turbine_config = test::create_gas_turbine_config();
    gas_turbine_config.initial_speed = config.initial_plant_speed;
    test::configure_variable_speed_plant(system.plant, gas_turbine_config);

    // Set initial speed (low value to allow observer startup)
    system.plant.get_parameters().initial_speed.write(config.initial_plant_speed);
}

/**
 * @brief Execute state machine startup sequence
 *
 * Transitions the state machine from OPERATION_OFF to OPERATION_ON by:
 * 1. Enabling auxiliaries
 * 2. Closing main circuit breaker
 * 3. Issuing start command
 *
 * Uses the external_ios parameter override mechanism to control digital inputs,
 * which then propagate through the signal wiring to the state machine.
 *
 * @param system The plant-complete SFC closed-loop simulation builder
 * @param simulator The simulator instance
 * @return True if state machine reached OPERATION_ON
 */
inline bool execute_state_machine_startup(
    plant_complete_sfc_closed_loop_simulation_builder& system, yeet::simulator& simulator)
{
    // Use parameter override mechanism to set digital inputs
    // This ensures proper signal flow through external_ios wiring
    auto& params = system.grid_side_control.external_ios.get_parameters();

    // Enable parameter overrides
    params.override_digital_inputs.write(true);

    // Enable auxiliaries and close breakers
    params.auxiliaries_are_on.write(true);
    params.mcb_is_closed.write(true);
    simulator.simulate_for(0.2);

    // Issue start command
    params.start_command.write(true);
    simulator.simulate_for(0.1);

    // Verify state machine is operational
    return system.grid_side_control.state_machine.get_outputs().release_pulses.read();
}

/**
 * @brief Execute pre-flux phase and configure load/torque limits
 *
 * @param system The plant-complete SFC closed-loop simulation builder
 * @param simulator The simulator instance
 * @param config Startup configuration parameters
 */
inline void execute_preflux_and_configure_load(plant_complete_sfc_closed_loop_simulation_builder& system,
    yeet::simulator& simulator,
    const startup_to_nominal_config& config)
{
    // Pre-flux duration
    simulator.simulate_for(config.prefluxing_duration.value());

    // Configure load torque (AFTER pre-flux but BEFORE virtual startup)
    system.plant.get_parameters().constant_torque.write(config.load_torque);

    // Configure torque limits for startup
    mock_signal_input::write(system.machine_side_control.torque_controller.get_inputs().dynamic_max_torque_limit,
        config.dynamic_max_torque_limit);
    mock_signal_input::write(system.machine_side_control.torque_controller.get_inputs().dynamic_min_torque_limit,
        config.dynamic_min_torque_limit);
}

/**
 * @brief Execute virtual startup and wait for closed-loop transition
 *
 * @param system The plant-complete SFC closed-loop simulation builder
 * @param simulator The simulator instance
 * @param duration Duration to wait for virtual startup completion [s]
 * @return True if startup sequencer reached CLOSED_LOOP_OPERATION
 */
inline bool execute_virtual_startup(
    plant_complete_sfc_closed_loop_simulation_builder& system, yeet::simulator& simulator, double duration)
{
    // simulator.simulate_for(duration);
    for (int i = 0; i < duration; ++i) {
        simulator.simulate_for(1.0);
        auto speed = system.machine_side_control.machine_side_observer.get_outputs().estimated_speed.read();
        auto virt_freq = system.machine_side_control.startup_sequencer.get_outputs().virtual_rotor_frequency.read();
    }
    simulator.simulate_for(duration - (int)duration);

    auto startup_state = system.machine_side_control.startup_sequencer.get_outputs().current_state.read();
    return startup_state == startup_sequencer_state_t::CLOSED_LOOP_OPERATION;
}

/**
 * @brief Ramp speed to target and stabilize
 *
 * @param system The plant-complete SFC closed-loop simulation builder
 * @param simulator The simulator instance
 * @param config Startup configuration parameters
 * @return Final estimated speed
 */
inline speed_pu_t ramp_to_target_speed(plant_complete_sfc_closed_loop_simulation_builder& system,
    yeet::simulator& simulator,
    const startup_to_nominal_config& config)
{
    // Set external speed reference to target
    system.machine_side_control.startup_sequencer.get_parameters().speed_reference.write(config.target_speed_reference);
    simulator.simulate_for(0.1);

    // Ramp to target with rate limiting
    // simulator.simulate_for(config.speed_ramp_duration);
    for (int i = 0; i < config.speed_ramp_duration; ++i) {
        simulator.simulate_for(1.0);
        auto speed = system.machine_side_control.machine_side_observer.get_outputs().estimated_speed.read();
        auto ref = system.machine_side_control.startup_sequencer.get_outputs().speed_reference.read();
        auto volt = system.machine_side_control.machine_side_observer.get_outputs().estimated_voltage_magnitude.read();
    }
    simulator.simulate_for(config.speed_ramp_duration - (int)config.speed_ramp_duration);  // Remainder

    // Stabilize at target speed
    // simulator.simulate_for(config.stabilization_duration);
    for (int i = 0; i < config.stabilization_duration; ++i) {
        simulator.simulate_for(1.0);
        auto speed = system.machine_side_control.machine_side_observer.get_outputs().estimated_speed.read();
    }
    simulator.simulate_for(config.stabilization_duration - (int)config.stabilization_duration);  // Remainder

    return system.machine_side_control.machine_side_observer.get_outputs().estimated_speed.read();
}

/**
 * @brief Complete startup-to-nominal-speed procedure
 *
 * This function performs the complete startup sequence from standstill to nominal speed:
 * 1. Configure startup sequencer parameters
 * 2. Configure plant model for soft-starter operation
 * 3. Execute state machine startup (OPERATION_OFF → OPERATION_ON)
 * 4. Execute pre-flux phase and configure load/torque limits
 * 5. Execute virtual startup procedure (V/Hz control)
 * 6. Transition to closed-loop observer-based control
 * 7. Ramp speed reference to target (e.g., 1.0 pu)
 * 8. Stabilize at target speed
 *
 * This procedure matches the startup sequence in host_main.cpp and ensures consistency
 * between the host simulator and integration tests.
 *
 * @param system The plant-complete SFC closed-loop simulation builder
 * @param simulator The simulator instance
 * @param config Startup configuration parameters (optional, uses default if not specified)
 * @return startup_result containing success flags and final values
 */
inline startup_result startup_to_nominal_speed(plant_complete_sfc_closed_loop_simulation_builder& system,
    yeet::simulator& simulator,
    const startup_to_nominal_config& config = create_default_startup_config())
{
    startup_result result;

    // Step 1: Configure startup sequencer
    configure_startup_sequencer(system, config);

    // Step 2: Configure plant model
    configure_plant_for_startup(system, config);

    // Step 3: Execute state machine startup
    result.state_machine_operational = execute_state_machine_startup(system, simulator);
    if (!result.state_machine_operational) {
        return result;
    }

    // Step 4: Execute pre-flux and configure load/torque limits
    execute_preflux_and_configure_load(system, simulator, config);

    // Step 5-6: Execute virtual startup and transition to closed-loop
    result.closed_loop_reached = execute_virtual_startup(system, simulator, config.virtual_startup_duration);
    if (!result.closed_loop_reached) {
        return result;
    }

    // Check reference frame
    auto ref_frame = system.machine_side_control.startup_sequencer.get_outputs().reference_frame_source.read();
    result.reference_frame_estimated = (ref_frame == reference_frame_source_t::ESTIMATED);

    // Step 7-8: Ramp to target speed and stabilize
    result.final_speed = ramp_to_target_speed(system, simulator, config);
    result.final_speed_reference = system.machine_side_control.startup_sequencer.get_outputs().speed_reference.read();

    // Check if target speed was approximately reached (within 15% tolerance)
    const real_t speed_tolerance = 0.15f;
    result.target_speed_reached =
        std::abs(result.final_speed.value() - config.target_speed_reference.value()) < speed_tolerance;

    // Overall success
    result.success = result.state_machine_operational && result.closed_loop_reached && result.target_speed_reached;

    return result;
}

}  // namespace yeet::application::sfc::integration
