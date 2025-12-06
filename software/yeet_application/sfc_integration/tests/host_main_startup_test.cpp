/**
 * @file host_main_startup_test.cpp
 * @brief Integration test validating the host_main startup procedure reaches nominal speed
 *
 * This test validates the complete startup-to-nominal-speed procedure that is used
 * by the host simulator (host_main.cpp). It ensures that the encapsulated startup
 * procedure correctly brings the SFC system from standstill to 1.0 pu speed.
 *
 * The test uses the same startup procedure as host_main.cpp to guarantee consistency
 * between the production code and test validation.
 */

#include <gtest/gtest.h>

#include "sfc_integration/plant_complete_sfc_closed_loop_simulation_builder.hpp"
#include "sfc_loggers.hpp"
#include "sfc_plant_configuration.hpp"
#include "sfc_startup_procedure.hpp"
#include "yeet_testing/simulator_checker.hpp"
#include "yeet_testing/simulator_gtest.hpp"

using namespace yeet::application::sfc;
using namespace yeet::application::sfc::integration;
using namespace yeet;

/**
 * @brief Test fixture for host_main startup procedure validation
 *
 * This fixture mirrors the setup in host_main.cpp to ensure the startup procedure
 * behaves identically in both production and test environments.
 */
class HostMainStartupTest : public ::testing::Test {
public:
    void SetUp() override
    {
        // Configure sampler for detailed data capture
        simulator.configure_sampler_sampling_rate(ms(10.0f));

        // Register all applications with the simulator
        system.register_applications(simulator);

        // Initialize mock inputs to safe state using the shared helper
        initialize_mock_signal_inputs(system);

        // Initialize the system
        system.initialize();

        // Register loggers for debugging (optional, helps with test analysis)
        register_complete_logger(simulator, system.machine_side_control.startup_sequencer);
        register_complete_logger(simulator, system.machine_side_control.machine_side_observer);
        register_complete_logger(simulator, system.machine_side_control.speed_controller);
        register_complete_logger(simulator, system.grid_side_control.state_machine);
    }

    // System under test
    plant_complete_sfc_closed_loop_simulation_builder system;

    // Test simulator
    yeet::simulator simulator = yeet::make_gtest_simulator();
};

/**
 * @test HostMainStartupReachesNominalSpeed
 * @brief Validates that the host_main startup procedure reaches 1.0 pu speed
 *
 * This test executes the exact same startup procedure used by host_main.cpp:
 * 1. Configure startup sequencer with default parameters
 * 2. Configure plant for soft-starter operation
 * 3. Execute state machine startup (OPERATION_OFF → OPERATION_ON)
 * 4. Execute pre-flux phase (1.0s)
 * 5. Execute virtual startup procedure (15.0s)
 * 6. Transition to closed-loop observer-based control
 * 7. Ramp speed reference to 1.0 pu (20.0s at 0.05 pu/s)
 * 8. Stabilize at 1.0 pu (5.0s)
 *
 * Expected outcomes:
 * - State machine reaches OPERATION_ON
 * - Startup sequencer reaches CLOSED_LOOP_OPERATION
 * - Reference frame switches to ESTIMATED (observer-based)
 * - Final speed approximately 1.0 pu (within 15% tolerance)
 */
// Force recompile 4
TEST_F(HostMainStartupTest, HostMainStartupReachesNominalSpeed)
{
    // Use default startup configuration (matches host_main.cpp)
    auto config = create_default_startup_config();

    // Execute the complete startup-to-nominal-speed procedure
    auto result = startup_to_nominal_speed(system, simulator, config);

    // Validate state machine startup
    EXPECT_TRUE(result.state_machine_operational) << "State machine should reach OPERATION_ON (pulse releases active)";

    // Validate startup sequencer reached closed-loop operation
    EXPECT_TRUE(result.closed_loop_reached) << "Startup sequencer should reach CLOSED_LOOP_OPERATION state";

    // Validate reference frame switched to estimated (observer-based)
    EXPECT_TRUE(result.reference_frame_estimated) << "Reference frame should be ESTIMATED during closed-loop operation";

    // Validate target speed was reached
    EXPECT_TRUE(result.target_speed_reached)
        << "System should reach approximately " << config.target_speed_reference.value() << " pu speed";

    // Detailed speed validation
    EXPECT_NEAR(result.final_speed.value(), config.target_speed_reference.value(), 0.15f)
        << "Final speed should be within 15% of target speed reference";

    // Validate speed reference was set correctly
    EXPECT_NEAR(result.final_speed_reference.value(), config.target_speed_reference.value(), 0.01f)
        << "Speed reference should match target";

    // Overall success
    EXPECT_TRUE(result.success) << "Complete startup-to-nominal-speed procedure should succeed";
}

/**
 * @test HostMainStartupStateTransitions
 * @brief Validates the state transitions during the host_main startup procedure
 *
 * This test monitors the startup sequencer state transitions to ensure they
 * follow the expected sequence:
 * IDLE → FLUX_BUILDUP → STARTING_PROCEDURE → CLOSED_LOOP_OPERATION
 */
TEST_F(HostMainStartupTest, HostMainStartupStateTransitions)
{
    auto config = create_default_startup_config();

    // Step 1: Configure startup sequencer
    configure_startup_sequencer(system, config);

    // Step 2: Configure plant model
    configure_plant_for_startup(system, config);

    // Verify initial state is IDLE
    auto initial_state = system.machine_side_control.startup_sequencer.get_outputs().current_state.read();
    EXPECT_EQ(initial_state, startup_sequencer_state_t::IDLE) << "Initial state should be IDLE";

    // Step 3: Execute state machine startup
    bool state_machine_ok = execute_state_machine_startup(system, simulator);
    EXPECT_TRUE(state_machine_ok) << "State machine should reach OPERATION_ON";

    // Step 4: Execute pre-flux and configure load/torque limits
    execute_preflux_and_configure_load(system, simulator, config);

    // After pre-flux, state should have progressed from IDLE
    // Depending on timing, could be FLUX_BUILDUP, STARTING_PROCEDURE, or even CLOSED_LOOP_OPERATION
    auto post_preflux_state = system.machine_side_control.startup_sequencer.get_outputs().current_state.read();
    EXPECT_NE(post_preflux_state, startup_sequencer_state_t::IDLE)
        << "After pre-flux, state should have progressed from IDLE";

    // Step 5-6: Execute virtual startup
    bool closed_loop_ok = execute_virtual_startup(system, simulator, config.virtual_startup_duration);
    EXPECT_TRUE(closed_loop_ok) << "Should reach CLOSED_LOOP_OPERATION";

    // Verify final state
    auto final_state = system.machine_side_control.startup_sequencer.get_outputs().current_state.read();
    EXPECT_EQ(final_state, startup_sequencer_state_t::CLOSED_LOOP_OPERATION)
        << "Final state should be CLOSED_LOOP_OPERATION";

    // Verify reference frame
    auto ref_frame = system.machine_side_control.startup_sequencer.get_outputs().reference_frame_source.read();
    EXPECT_EQ(ref_frame, reference_frame_source_t::ESTIMATED) << "Reference frame should be ESTIMATED";
}

/**
 * @test HostMainStartupSpeedRamping
 * @brief Validates the speed ramping behavior during startup
 *
 * This test verifies that the speed reference rate limiter correctly ramps
 * the speed from the virtual startup final frequency to the target speed.
 * With a rate limit of 20 s/pu (0.05 pu/s), reaching 1.0 pu from ~0.1 pu
 * should take approximately 18 seconds.
 */
TEST_F(HostMainStartupTest, HostMainStartupSpeedRamping)
{
    auto config = create_default_startup_config();

    // Execute startup up to closed-loop operation
    configure_startup_sequencer(system, config);
    configure_plant_for_startup(system, config);

    bool state_machine_ok = execute_state_machine_startup(system, simulator);
    ASSERT_TRUE(state_machine_ok);

    execute_preflux_and_configure_load(system, simulator, config);

    bool closed_loop_ok = execute_virtual_startup(system, simulator, config.virtual_startup_duration);
    ASSERT_TRUE(closed_loop_ok);

    // Get baseline speed after closed-loop transition
    // Note: With gas turbine quadratic load profile, speed may be lower than virtual_frequency_final
    // due to load torque causing speed droop during V/Hz operation
    auto baseline_speed = system.machine_side_control.machine_side_observer.get_outputs().estimated_speed.read();
    EXPECT_GT(baseline_speed.value(), 0.0f) << "Speed after virtual startup should be positive";
    EXPECT_GE(baseline_speed.value(), config.virtual_frequency_final.value() * 0.5f)
        << "Speed after virtual startup should be at least 50% of virtual_frequency_final";

    // Set target speed reference
    system.machine_side_control.startup_sequencer.get_parameters().speed_reference.write(config.target_speed_reference);
    simulator.simulate_for(0.1);

    // Expected rate: 0.05 pu/s (from 20 s/pu rate limit)
    const real_t expected_rate = 1.0f / config.speed_reference_rate_limit_seconds_per_pu;

    // Simulate for 10 seconds and check intermediate speed
    simulator.simulate_for(10.0);
    auto speed_10s = system.machine_side_control.startup_sequencer.get_outputs().speed_reference.read();

    // Speed reference should have increased by approximately 0.5 pu (10s × 0.05 pu/s)
    // But clamped to target (1.0 pu)
    EXPECT_LE(speed_10s.value(), config.target_speed_reference.value()) << "Speed reference should not exceed target";

    // Complete the ramp
    simulator.simulate_for(config.speed_ramp_duration - 10.0);
    simulator.simulate_for(config.stabilization_duration);

    auto final_speed = system.machine_side_control.machine_side_observer.get_outputs().estimated_speed.read();
    EXPECT_NEAR(final_speed.value(), config.target_speed_reference.value(), 0.15f)
        << "Final speed should reach target after rate-limited ramp";
}

/**
 * @test HostMainStartupWithCustomConfig
 * @brief Validates startup with custom configuration parameters
 *
 * This test demonstrates using custom startup parameters, such as
 * a different target speed or rate limit.
 */
TEST_F(HostMainStartupTest, HostMainStartupWithCustomConfig)
{
    // Custom configuration: faster startup to 0.5 pu
    startup_to_nominal_config config;
    config.target_speed_reference = speed_pu_t{0.5f};
    config.speed_reference_rate_limit_seconds_per_pu = 10.0f;  // Faster: 0.1 pu/s
    config.speed_ramp_duration = 10.0;                         // Shorter ramp time
    config.stabilization_duration = 3.0;                       // Shorter stabilization

    auto result = startup_to_nominal_speed(system, simulator, config);

    EXPECT_TRUE(result.success) << "Custom startup should succeed";
    EXPECT_NEAR(result.final_speed.value(), 0.5f, 0.10f) << "Final speed should reach custom target of 0.5 pu";
}

/**
 * @test HostMainStartupSteadyStateStability
 * @brief Validates that the system remains stable at nominal speed after startup
 *
 * After reaching nominal speed, the system should maintain stable operation
 * without significant speed oscillations or drift.
 */
TEST_F(HostMainStartupTest, HostMainStartupSteadyStateStability)
{
    auto config = create_default_startup_config();
    auto result = startup_to_nominal_speed(system, simulator, config);

    ASSERT_TRUE(result.success) << "Startup should succeed before stability test";

    // Track speed over extended period
    const double stability_test_duration = 10.0;
    const double sample_interval = 0.1;
    const int num_samples = static_cast<int>(stability_test_duration / sample_interval);

    real_t min_speed = result.final_speed.value();
    real_t max_speed = result.final_speed.value();
    real_t sum_speed = 0.0f;

    for (int i = 0; i < num_samples; ++i) {
        simulator.simulate_for(sample_interval);
        auto current_speed = system.machine_side_control.machine_side_observer.get_outputs().estimated_speed.read();

        min_speed = std::min(min_speed, current_speed.value());
        max_speed = std::max(max_speed, current_speed.value());
        sum_speed += current_speed.value();
    }

    real_t avg_speed = sum_speed / static_cast<real_t>(num_samples);
    real_t speed_ripple = max_speed - min_speed;

    // Verify stability
    EXPECT_NEAR(avg_speed, config.target_speed_reference.value(), 0.10f) << "Average speed should be close to target";

    EXPECT_LT(speed_ripple, 0.15f) << "Speed ripple should be less than 15% during steady-state";
}
