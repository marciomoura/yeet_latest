/**
 * @file closed_loop_rectifier_test.cpp
 * @brief Closed-loop tests for rectifier-only topology with complete controller
 *
 * This test validates running the complete SFC controller with a rectifier-only
 * plant model (machine-side short-circuited with zero back-EMF). This configuration
 * allows testing the grid-side current control loop in isolation.
 *
 * Key configuration:
 * - Plant topology: rectifier (machine-side short-circuited)
 * - Operating mode: force natural commutation (bypass speed-based detection)
 * - Torque control: override with fixed torque reference (controls DC-link current)
 * - Machine-side protections: disabled (V/Hz and stall don't apply)
 * - Speed control: bypassed via torque override
 */

#include <gtest/gtest.h>

#include <cmath>

#include "sfc_integration/plant_complete_sfc_closed_loop_simulation_builder.hpp"
#include "sfc_integration/sfc_plant_model_datamodel.hpp"
#include "sfc_loggers.hpp"
#include "yeet_testing/simulator_gtest.hpp"

using namespace yeet::application::sfc;
using namespace yeet::application::sfc::integration;
using namespace yeet;

class ClosedLoopRectifierTest : public ::testing::Test {
public:
    void SetUp() override
    {
        // ==================== Configure Plant as Rectifier ====================
        system.plant.get_parameters().topology.write(sfc_plant_model_datamodel::topology_type::rectifier);

        // ==================== Force Natural Commutation Mode ====================
        // Bypass speed-based mode detection since there's no machine speed
        system.grid_side_control.operation_mode_monitor.get_parameters().force_natural_commutation.write(true);

        // ==================== Disable Machine-Side Protections ====================
        // V/Hz and stall protections don't apply without a machine
        system.machine_side_control.machine_protections.get_parameters().protection_module_enabled.write(false);

        // ==================== Configure Torque Override ====================
        // Override torque reference to directly control DC-link current
        system.machine_side_control.torque_controller.get_parameters().torque_reference_override.write(true);
        system.machine_side_control.torque_controller.get_parameters().torque_reference_override_value.write(
            torque_pu_t{0.0f});

        // Disable torque rate limiting for faster response in tests
        system.machine_side_control.torque_controller.get_parameters().enable_torque_rate_limiting.write(false);

        // ==================== Register Applications ====================
        system.register_applications(simulator);

        // ==================== Logger Registration ====================
        register_complete_logger(simulator, system.plant);
        register_complete_logger(simulator, system.grid_side_control.grid_side_pulse_generator);
        register_complete_logger(simulator, system.grid_side_hardware_mock);
        register_complete_logger(simulator, system.grid_side_control.grid_measurements);
        register_complete_logger(simulator, system.grid_side_control.dc_link_measurements);
        register_complete_logger(simulator, system.grid_side_control.current_controller);
        register_complete_logger(simulator, system.grid_side_control.operation_mode_monitor);
        register_minimal_logger(simulator, system.machine_side_control.torque_controller);
    }

    void initialize() { system.initialize(); }

    void simulate_for(double time) { simulator.simulate_for(time); }

    // State machine control helpers
    // turn_on_command: Energizes system (IDLE → AUXILIARIES_ON → BREAKERS_ON via IO mock)
    // start_command: Starts operation (BREAKERS_ON → OPERATION_ON)
    void set_turn_on_command(bool on)
    {
        system.grid_side_control.state_machine.get_parameters().turn_on_command.write(on);
    }

    void set_start_command(bool start) { system.grid_io_mock.get_parameters().start_command.write(start); }

    void set_stop_command(bool stop) { system.grid_io_mock.get_parameters().stop_command.write(stop); }

    // Execute startup sequence to reach OPERATION_ON state
    // NOTE: start_command must remain true to stay in OPERATION_ON state
    // Uses turn_on_command to trigger IO mock → auxiliaries → breakers flow
    void execute_startup_sequence()
    {
        set_turn_on_command(true);
        simulate_for(0.3);  // Allow IO mock to process auxiliaries and breakers with delays
        set_start_command(true);
        simulate_for(0.1);
        // Keep start_command=true - state machine requires it to stay in OPERATION_ON
    }

    // Set torque reference (which controls DC-link current)
    void set_torque_reference(torque_pu_t torque)
    {
        system.machine_side_control.torque_controller.get_parameters().torque_reference_override_value.write(torque);
    }

    // Get DC-link current in per-unit
    current_pu_t get_dc_link_current_pu() const
    {
        return system.grid_side_control.dc_link_measurements.get_outputs().dc_current_pu.read();
    }

    // Get operating mode
    sfc_operating_mode_t get_operating_mode() const
    {
        return system.grid_side_control.operation_mode_monitor.get_outputs().operating_mode.read();
    }

    // System under test
    plant_complete_sfc_closed_loop_simulation_builder system;
    yeet::simulator simulator = yeet::make_gtest_simulator();
};

/**
 * @test RectifierClosedLoopCurrentControl
 * @brief Validates closed-loop DC-link current control with rectifier topology
 *
 * This test verifies that the complete controller can regulate DC-link current
 * when operating with a rectifier-only plant (no machine). The torque reference
 * is overridden to directly command the current controller.
 *
 * Test sequence:
 * 1. Execute startup sequence to reach OPERATION_ON
 * 2. Command 0.5 pu torque (= 0.5 pu current)
 * 3. Verify DC-link current tracks reference
 * 4. Step to 0.8 pu torque
 * 5. Verify current increases to new setpoint
 */
TEST_F(ClosedLoopRectifierTest, RectifierClosedLoopCurrentControl)
{
    initialize();

    // ==================== Execute Startup Sequence ====================
    execute_startup_sequence();

    // Let system settle
    simulate_for(0.5);

    // Verify operating mode is natural commutation (forced)
    EXPECT_EQ(get_operating_mode(), sfc_operating_mode_t::natural_commutation)
        << "Operating mode should be forced to natural commutation";

    // Verify pulses are released
    EXPECT_TRUE(system.grid_side_control.state_machine.get_outputs().release_pulses.read())
        << "State machine should release pulses after startup";

    // ==================== Step 1: Command 0.5 pu Current ====================
    set_torque_reference(torque_pu_t{0.5f});

    // Allow current to ramp up and settle
    simulate_for(1.0);

    auto dc_current_1 = get_dc_link_current_pu();
    EXPECT_NEAR(dc_current_1.value(), 0.5f, 0.15f) << "DC-link current should track 0.5 pu torque reference";

    // ==================== Step 2: Command 0.8 pu Current ====================
    set_torque_reference(torque_pu_t{0.8f});

    simulate_for(1.0);

    auto dc_current_2 = get_dc_link_current_pu();
    EXPECT_NEAR(dc_current_2.value(), 0.8f, 0.15f) << "DC-link current should track 0.8 pu torque reference";

    // Current should have increased
    EXPECT_GT(dc_current_2.value(), dc_current_1.value())
        << "DC-link current should increase with higher torque reference";

    // ==================== Step 3: Reduce to 0.3 pu ====================
    set_torque_reference(torque_pu_t{0.3f});

    simulate_for(1.0);

    auto dc_current_3 = get_dc_link_current_pu();
    EXPECT_NEAR(dc_current_3.value(), 0.3f, 0.15f) << "DC-link current should track 0.3 pu torque reference";

    // Current should have decreased
    EXPECT_LT(dc_current_3.value(), dc_current_2.value())
        << "DC-link current should decrease with lower torque reference";
}

/**
 * @test RectifierSteadyStateOperation
 * @brief Validates steady-state operation stability
 *
 * This test verifies that the controller maintains stable operation
 * at a fixed current setpoint over an extended period.
 */
TEST_F(ClosedLoopRectifierTest, RectifierSteadyStateOperation)
{
    initialize();
    execute_startup_sequence();

    // Command moderate current
    set_torque_reference(torque_pu_t{0.6f});

    // Allow initial settling
    simulate_for(1.0);

    // Record baseline
    auto baseline_current = get_dc_link_current_pu();
    EXPECT_NEAR(baseline_current.value(), 0.6f, 0.15f) << "DC-link current should reach 0.6 pu setpoint";

    // Run for extended period
    simulate_for(3.0);

    // Verify current remains stable
    auto final_current = get_dc_link_current_pu();
    EXPECT_NEAR(final_current.value(), 0.6f, 0.15f) << "DC-link current should remain stable at 0.6 pu";

    // Verify operating mode hasn't changed
    EXPECT_EQ(get_operating_mode(), sfc_operating_mode_t::natural_commutation)
        << "Operating mode should remain natural commutation";

    // Verify system still operational
    EXPECT_TRUE(system.grid_side_control.state_machine.get_outputs().release_pulses.read())
        << "Pulses should remain released during steady-state";
}

/**
 * @test RectifierZeroCurrentOperation
 * @brief Validates operation at zero current reference
 *
 * Tests that the controller can handle zero current demand gracefully.
 */
TEST_F(ClosedLoopRectifierTest, RectifierZeroCurrentOperation)
{
    initialize();
    execute_startup_sequence();

    // Start with some current
    set_torque_reference(torque_pu_t{0.4f});
    simulate_for(1.0);

    auto initial_current = get_dc_link_current_pu();
    EXPECT_GT(initial_current.value(), 0.2f) << "Initial current should be positive";

    // Command zero current
    set_torque_reference(torque_pu_t{0.0f});
    simulate_for(1.0);

    auto zero_current = get_dc_link_current_pu();
    EXPECT_NEAR(zero_current.value(), 0.0f, 0.1f) << "DC-link current should approach zero with zero torque reference";

    // Verify system remains operational
    EXPECT_TRUE(system.grid_side_control.state_machine.get_outputs().release_pulses.read())
        << "Pulses should remain released even at zero current";
}

/**
 * @test RectifierCurrentRampProfile
 * @brief Validates current tracking through a ramp profile
 *
 * Tests the controller's ability to track a gradual current ramp.
 */
TEST_F(ClosedLoopRectifierTest, RectifierCurrentRampProfile)
{
    initialize();
    execute_startup_sequence();
    simulate_for(0.5);

    // Ramp current from 0.2 to 1.0 pu in steps
    std::vector<std::pair<float, float>> setpoint_results;

    for (float setpoint = 0.2f; setpoint <= 1.0f; setpoint += 0.2f) {
        set_torque_reference(torque_pu_t{setpoint});
        simulate_for(0.5);

        auto current = get_dc_link_current_pu();
        setpoint_results.push_back({setpoint, current.value()});

        EXPECT_NEAR(current.value(), setpoint, 0.2f) << "DC-link current should track " << setpoint << " pu setpoint";
    }

    // Verify monotonic increase
    for (size_t i = 1; i < setpoint_results.size(); ++i) {
        EXPECT_GT(setpoint_results[i].second, setpoint_results[i - 1].second)
            << "Current should increase monotonically with setpoint";
    }
}

/**
 * @test BreakerOpenDuringCurrentRegulation
 * @brief Validates system behavior when breaker opens during active current regulation
 *
 * This test verifies that:
 * 1. Opening the breaker masks firing pulses (no new thyristor turn-ons)
 * 2. Grid voltage remains present (pulse masking, not voltage zeroing)
 * 3. The controller remains in operational state (unaware of physical breaker)
 * 4. The DC link current decays to zero as thyristors stop conducting
 */
TEST_F(ClosedLoopRectifierTest, BreakerOpenDuringCurrentRegulation)
{
    initialize();
    execute_startup_sequence();

    // Command moderate current and let it settle
    set_torque_reference(torque_pu_t{0.6f});
    simulate_for(1.0);

    auto current_before = get_dc_link_current_pu();
    EXPECT_NEAR(current_before.value(), 0.6f, 0.15f) << "DC-link current should reach 0.6 pu setpoint";

    // Get plant model reference and open the breaker
    auto& plant = system.plant.get_plant_model();
    EXPECT_TRUE(plant.is_grid_circuit_breaker_closed()) << "Breaker should be closed initially";

    plant.set_grid_circuit_breaker_closed(false);
    EXPECT_FALSE(plant.is_grid_circuit_breaker_closed()) << "Breaker should be open now";

    // Simulate with open breaker
    simulate_for(0.5);

    // Verify current decays to zero
    EXPECT_NEAR(get_dc_link_current_pu().value(), 0.0f, 0.01f)
        << "DC-link current should decay to zero with an open breaker";

    // Verify breaker state is maintained
    EXPECT_FALSE(plant.is_grid_circuit_breaker_closed()) << "Breaker should still be open";

    // System should still be in operational state (pulses released)
    // The control layer is unaware of the breaker state
    EXPECT_TRUE(system.grid_side_control.state_machine.get_outputs().release_pulses.read())
        << "State machine should still have pulses released (control layer unaware of breaker)";
}

/**
 * @test BreakerRecloseDuringOperation
 * @brief Validates system recovery when breaker is reclosed after being opened
 *
 * This test verifies that:
 * 1. System can resume normal operation after breaker reclosing
 * 2. Current controller can re-establish current regulation
 */
TEST_F(ClosedLoopRectifierTest, BreakerRecloseDuringOperation)
{
    initialize();
    execute_startup_sequence();

    // Establish current
    set_torque_reference(torque_pu_t{0.5f});
    simulate_for(1.0);

    auto current_initial = get_dc_link_current_pu();
    EXPECT_NEAR(current_initial.value(), 0.5f, 0.15f) << "DC-link current should reach 0.5 pu setpoint";

    // Open breaker
    auto& plant = system.plant.get_plant_model();
    plant.set_grid_circuit_breaker_closed(false);
    simulate_for(0.2);  // Brief open period

    EXPECT_NEAR(get_dc_link_current_pu().value(), 0.0f, 0.01f)
        << "DC-link current should reach 0.0 pu setpoint with an open breaker";

    // Verify breaker is open
    EXPECT_FALSE(plant.is_grid_circuit_breaker_closed()) << "Breaker should be open";

    // Reclose breaker
    plant.set_grid_circuit_breaker_closed(true);
    EXPECT_TRUE(plant.is_grid_circuit_breaker_closed()) << "Breaker should be reclosed";

    // Allow current controller to re-establish regulation
    simulate_for(2.0);

    auto current_after_reclose = get_dc_link_current_pu();

    // Current should have recovered to near setpoint
    // Allowing wider tolerance since controller may need time to stabilize after disturbance
    EXPECT_NEAR(current_after_reclose.value(), 0.5f, 0.25f) << "DC-link current should recover after breaker reclose";
}

/**
 * @test BreakerOpenAtHighCurrent
 * @brief Validates breaker opening behavior at high current levels
 *
 * Tests system behavior when breaker opens while high current is flowing.
 * This test verifies that:
 * 1. The breaker state can be changed during high current operation
 * 2. The system remains stable (doesn't crash or enter undefined state)
 * 3. The breaker state is correctly maintained
 * 4. The DC link current decays to zero
 */
TEST_F(ClosedLoopRectifierTest, BreakerOpenAtHighCurrent)
{
    initialize();
    execute_startup_sequence();

    // Command high current
    set_torque_reference(torque_pu_t{0.9f});
    simulate_for(1.5);

    auto current_before = get_dc_link_current_pu();
    EXPECT_NEAR(current_before.value(), 0.9f, 0.15f) << "DC-link current should reach 0.9 pu setpoint";

    // Open breaker at high current
    auto& plant = system.plant.get_plant_model();
    EXPECT_TRUE(plant.is_grid_circuit_breaker_closed()) << "Breaker should be closed initially";

    plant.set_grid_circuit_breaker_closed(false);
    EXPECT_FALSE(plant.is_grid_circuit_breaker_closed()) << "Breaker should be open now";

    // Simulate - system should remain stable
    simulate_for(0.5);

    // Verify current decays to zero
    EXPECT_NEAR(get_dc_link_current_pu().value(), 0.0f, 0.01f)
        << "DC-link current should decay to zero with an open breaker";

    // Verify breaker state is maintained
    EXPECT_FALSE(plant.is_grid_circuit_breaker_closed()) << "Breaker should still be open";

    // System should still be operational (state machine unaware of physical breaker)
    EXPECT_TRUE(system.grid_side_control.state_machine.get_outputs().release_pulses.read())
        << "State machine should still have pulses released";
}
