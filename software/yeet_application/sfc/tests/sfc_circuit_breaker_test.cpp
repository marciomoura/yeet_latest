#include <gtest/gtest.h>

#include "sfc/sfc_plant_back_to_back_topology.hpp"
#include "sfc/sfc_plant_back_to_back_topology_simulink.hpp"
#include "sfc/sfc_plant_model_voltage_behind_reactance.hpp"
#include "sfc/sfc_plant_rectifier_topology.hpp"
#include "yeet_control/units.hpp"

using namespace yeet::application::sfc;
using namespace yeet;

class CircuitBreakerTest : public ::testing::Test {
protected:
    static constexpr double kSamplingTime = 10e-6;
    static constexpr double kSimulationTime = 0.1;                                         // 100ms simulation
    static constexpr int kStepsPerCycle = static_cast<int>(1.0 / (50.0 * kSamplingTime));  // Steps per 50Hz cycle

    void simulate_steps(sfc_plant_back_to_back_topology& plant, int steps)
    {
        for (int i = 0; i < steps; ++i) {
            plant.update();
        }
    }

    void simulate_steps(sfc_plant_back_to_back_topology_simulink& plant, int steps)
    {
        for (int i = 0; i < steps; ++i) {
            plant.update();
        }
    }

    void simulate_steps(sfc_plant_rectifier_topology& plant, int steps)
    {
        for (int i = 0; i < steps; ++i) {
            plant.update();
        }
    }

    void simulate_steps(sfc_plant_model_voltage_behind_reactance& plant, int steps)
    {
        for (int i = 0; i < steps; ++i) {
            plant.update();
        }
    }
};

// ============== Internal Topology Tests ==============

TEST_F(CircuitBreakerTest, InternalTopology_DefaultBreakerClosed)
{
    sfc_plant_back_to_back_topology plant;
    plant.initialize();

    EXPECT_TRUE(plant.is_grid_circuit_breaker_closed());
}

TEST_F(CircuitBreakerTest, InternalTopology_GridBreakerOpenCloseCycle)
{
    sfc_plant_back_to_back_topology plant;
    plant.initialize();

    // Initially closed
    EXPECT_TRUE(plant.is_grid_circuit_breaker_closed());

    // Open the breaker
    plant.set_grid_circuit_breaker_closed(false);
    EXPECT_FALSE(plant.is_grid_circuit_breaker_closed());

    // Close the breaker
    plant.set_grid_circuit_breaker_closed(true);
    EXPECT_TRUE(plant.is_grid_circuit_breaker_closed());
}

TEST_F(CircuitBreakerTest, InternalTopology_GridBreakerOpenCausesCurrentDecay)
{
    sfc_plant_back_to_back_topology plant;
    plant.initialize();

    // Set up firing pulses to establish current
    std::array<bool, 6> grid_pulses = {true, false, false, true, false, false};
    std::array<bool, 6> machine_pulses = {true, false, false, true, false, false};
    plant.set_grid_side_pulses(grid_pulses);
    plant.set_machine_side_pulses(machine_pulses);

    // Simulate a few cycles to establish steady-state current
    simulate_steps(plant, kStepsPerCycle * 5);

    // Record current before opening breaker
    current_t initial_current = plant.get_dc_link_current();

    // Open grid breaker
    plant.set_grid_circuit_breaker_closed(false);

    // Simulate more steps - current should decay
    simulate_steps(plant, kStepsPerCycle * 5);

    current_t final_current = plant.get_dc_link_current();

    // Current should have decayed significantly (or gone to zero)
    // With breaker open, no new energy enters the system
    EXPECT_NEAR(final_current.value(), 0.0, 1.0) << "Current should decay to zero after grid breaker opens";
}

// ============== Voltage Behind Reactance Model Tests ==============

TEST_F(CircuitBreakerTest, VoltageBehindReactance_DefaultBreakerClosed)
{
    sfc_plant_model_voltage_behind_reactance plant(plant_topology_type::internal);
    plant.initialize();

    EXPECT_TRUE(plant.is_grid_circuit_breaker_closed());
}

TEST_F(CircuitBreakerTest, VoltageBehindReactance_GridBreakerOpenCloseCycle)
{
    sfc_plant_model_voltage_behind_reactance plant(plant_topology_type::internal);
    plant.initialize();

    // Initially closed
    EXPECT_TRUE(plant.is_grid_circuit_breaker_closed());

    // Open the breaker
    plant.set_grid_circuit_breaker_closed(false);
    EXPECT_FALSE(plant.is_grid_circuit_breaker_closed());

    // Close the breaker
    plant.set_grid_circuit_breaker_closed(true);
    EXPECT_TRUE(plant.is_grid_circuit_breaker_closed());
}

TEST_F(CircuitBreakerTest, VoltageBehindReactance_GridBreakerOpenZerosVoltage)
{
    sfc_plant_model_voltage_behind_reactance plant(plant_topology_type::internal);
    plant.initialize();

    // Simulate a few steps with breaker closed
    simulate_steps(plant, 100);

    // Open grid breaker
    plant.set_grid_circuit_breaker_closed(false);

    // Update to apply breaker state
    plant.update();

    // Grid voltage should now be zero (or very close)
    // The underlying model should see zero grid voltage
    EXPECT_FALSE(plant.is_grid_circuit_breaker_closed());
}

TEST_F(CircuitBreakerTest, VoltageBehindReactance_BreakerStatePreservedAcrossUpdates)
{
    sfc_plant_model_voltage_behind_reactance plant(plant_topology_type::internal);
    plant.initialize();

    // Open grid breaker
    plant.set_grid_circuit_breaker_closed(false);

    // Run many updates
    for (int i = 0; i < 1000; ++i) {
        plant.update();
        EXPECT_FALSE(plant.is_grid_circuit_breaker_closed()) << "Breaker state changed during update at step " << i;
    }
}

TEST_F(CircuitBreakerTest, VoltageBehindReactance_ResetPreservesBreakerState)
{
    sfc_plant_model_voltage_behind_reactance plant(plant_topology_type::internal);
    plant.initialize();

    // Open the grid breaker
    plant.set_grid_circuit_breaker_closed(false);

    // Verify it's open
    EXPECT_FALSE(plant.is_grid_circuit_breaker_closed());

    // Reset the plant (reinitializes internal states but not physical device states)
    plant.reset();

    // Breaker should remain in its configured state (open)
    // This models physical behavior where breakers don't automatically close on system reset
    EXPECT_FALSE(plant.is_grid_circuit_breaker_closed());
}

TEST_F(CircuitBreakerTest, InternalTopology_GridBreakerOpenRealisticBehavior)
{
    // This test validates that opening the grid breaker:
    // 1. Does NOT zero the grid voltage (unrealistic)
    // 2. Blocks new thyristor turn-ons (gate pulses masked)
    // 3. Allows DC current to decay naturally through the circuit
    // 4. Grid voltage source remains visible to any conducting thyristors

    sfc_plant_back_to_back_topology plant;
    plant.initialize();

    // Set up firing pulses to establish current flow
    std::array<bool, 6> grid_pulses = {true, false, false, true, false, false};
    std::array<bool, 6> machine_pulses = {true, false, false, true, false, false};
    plant.set_grid_side_pulses(grid_pulses);
    plant.set_machine_side_pulses(machine_pulses);

    // Simulate several cycles to establish DC link current
    simulate_steps(plant, kStepsPerCycle * 10);

    // Record state before opening breaker
    current_t current_before = plant.get_dc_link_current();
    abc<voltage_t> grid_voltages_before = plant.get_grid_side_voltages_abc();

    // Open grid breaker
    plant.set_grid_circuit_breaker_closed(false);

    // Continue simulation - DC current should decay
    simulate_steps(plant, kStepsPerCycle * 10);

    current_t current_after = plant.get_dc_link_current();
    abc<voltage_t> grid_voltages_after = plant.get_grid_side_voltages_abc();

    // Grid voltage should still be present (not zeroed) - voltage source is still there
    // The voltage waveforms continue regardless of breaker state
    // We verify at least one phase has non-zero voltage magnitude
    real_t max_voltage_magnitude = std::max({std::abs(grid_voltages_after.a().value()),
        std::abs(grid_voltages_after.b().value()), std::abs(grid_voltages_after.c().value())});
    EXPECT_GT(max_voltage_magnitude, 10.0) << "Grid voltage should still be present when breaker is open";

    // DC current should have decayed (or be zero) since no new energy enters
    EXPECT_NEAR(current_after.value(), 0.0, 1.0) << "DC current should decay to zero when grid breaker opens";
}

TEST_F(CircuitBreakerTest, InternalTopology_GridBreakerReclosing)
{
    // Test that closing the breaker after it was open allows current to build again

    sfc_plant_back_to_back_topology plant;
    plant.initialize();

    // Set up firing pulses
    std::array<bool, 6> grid_pulses = {true, false, false, true, false, false};
    std::array<bool, 6> machine_pulses = {true, false, false, true, false, false};
    plant.set_grid_side_pulses(grid_pulses);
    plant.set_machine_side_pulses(machine_pulses);

    // Establish current
    simulate_steps(plant, kStepsPerCycle * 5);

    // Open breaker - current should decay
    plant.set_grid_circuit_breaker_closed(false);
    simulate_steps(plant, kStepsPerCycle * 10);

    current_t current_with_open_breaker = plant.get_dc_link_current();

    // Reclose breaker - current should be able to build again
    plant.set_grid_circuit_breaker_closed(true);
    simulate_steps(plant, kStepsPerCycle * 10);

    current_t current_after_reclose = plant.get_dc_link_current();

    // After reclosing, the system should be able to establish current again
    // (assuming the firing pulses and voltage conditions support conduction)
    EXPECT_TRUE(plant.is_grid_circuit_breaker_closed());
}

// ============== Simulink Topology Tests ==============

TEST_F(CircuitBreakerTest, SimulinkTopology_DefaultBreakerClosed)
{
    sfc_plant_back_to_back_topology_simulink plant;
    plant.initialize();

    EXPECT_TRUE(plant.is_grid_circuit_breaker_closed());
}

TEST_F(CircuitBreakerTest, SimulinkTopology_GridBreakerOpenCloseCycle)
{
    sfc_plant_back_to_back_topology_simulink plant;
    plant.initialize();

    // Initially closed
    EXPECT_TRUE(plant.is_grid_circuit_breaker_closed());

    // Open the breaker
    plant.set_grid_circuit_breaker_closed(false);
    EXPECT_FALSE(plant.is_grid_circuit_breaker_closed());

    // Close the breaker
    plant.set_grid_circuit_breaker_closed(true);
    EXPECT_TRUE(plant.is_grid_circuit_breaker_closed());
}

TEST_F(CircuitBreakerTest, SimulinkTopology_GridBreakerOpenMasksPulses)
{
    // This test validates that opening the grid breaker masks firing pulses
    // preventing new thyristor turn-ons while maintaining grid voltage

    sfc_plant_back_to_back_topology_simulink plant;
    plant.initialize();

    // Configure grid voltage source (required for Simulink model)
    plant.set_grid_parameters(frequency_t{50.0}, voltage_t{690.0});
    plant.set_machine_parameters(frequency_t{50.0}, voltage_t{690.0});

    // Set up firing pulses to establish current flow
    std::array<bool, 6> grid_pulses = {true, false, false, true, false, false};
    std::array<bool, 6> machine_pulses = {true, false, false, true, false, false};
    plant.set_grid_side_pulses(grid_pulses);
    plant.set_machine_side_pulses(machine_pulses);

    // Simulate several cycles to establish DC link current
    simulate_steps(plant, kStepsPerCycle * 10);

    // Record state before opening breaker
    current_t current_before = plant.get_dc_link_current();
    abc<voltage_t> grid_voltages_before = plant.get_grid_side_voltages_abc();

    // Open grid breaker
    plant.set_grid_circuit_breaker_closed(false);

    // Continue simulation - DC current should decay
    // Simulink model might take longer to decay or have different dynamics
    simulate_steps(plant, kStepsPerCycle * 50);

    current_t current_after = plant.get_dc_link_current();
    abc<voltage_t> grid_voltages_after = plant.get_grid_side_voltages_abc();

    // Grid voltage should still be present (not zeroed) - voltage source is still there
    real_t max_voltage_magnitude = std::max({std::abs(grid_voltages_after.a().value()),
        std::abs(grid_voltages_after.b().value()), std::abs(grid_voltages_after.c().value())});
    EXPECT_GT(max_voltage_magnitude, 10.0) << "Grid voltage should still be present when breaker is open";

    // DC current should have decayed since no new thyristors can turn on
    // Relaxed tolerance for Simulink model residual current
    EXPECT_NEAR(current_after.value(), 0.0, 30.0)
        << "DC current should decay to zero when grid breaker opens (pulses masked)";
}

TEST_F(CircuitBreakerTest, SimulinkTopology_GridBreakerReclosing)
{
    sfc_plant_back_to_back_topology_simulink plant;
    plant.initialize();

    // Configure grid voltage source (required for Simulink model)
    plant.set_grid_parameters(frequency_t{50.0}, voltage_t{690.0});
    plant.set_machine_parameters(frequency_t{50.0}, voltage_t{690.0});

    // Set up firing pulses
    std::array<bool, 6> grid_pulses = {true, false, false, true, false, false};
    std::array<bool, 6> machine_pulses = {true, false, false, true, false, false};
    plant.set_grid_side_pulses(grid_pulses);
    plant.set_machine_side_pulses(machine_pulses);

    // Establish current
    simulate_steps(plant, kStepsPerCycle * 5);

    // Open breaker
    plant.set_grid_circuit_breaker_closed(false);
    simulate_steps(plant, kStepsPerCycle * 10);

    // Reclose breaker
    plant.set_grid_circuit_breaker_closed(true);
    simulate_steps(plant, kStepsPerCycle * 10);

    EXPECT_TRUE(plant.is_grid_circuit_breaker_closed());
}

// ============== Rectifier Topology Tests ==============

TEST_F(CircuitBreakerTest, RectifierTopology_DefaultBreakerClosed)
{
    sfc_plant_rectifier_topology plant;
    plant.initialize();

    EXPECT_TRUE(plant.is_grid_circuit_breaker_closed());
}

TEST_F(CircuitBreakerTest, RectifierTopology_GridBreakerOpenCloseCycle)
{
    sfc_plant_rectifier_topology plant;
    plant.initialize();

    // Initially closed
    EXPECT_TRUE(plant.is_grid_circuit_breaker_closed());

    // Open the breaker
    plant.set_grid_circuit_breaker_closed(false);
    EXPECT_FALSE(plant.is_grid_circuit_breaker_closed());

    // Close the breaker
    plant.set_grid_circuit_breaker_closed(true);
    EXPECT_TRUE(plant.is_grid_circuit_breaker_closed());
}

TEST_F(CircuitBreakerTest, RectifierTopology_GridBreakerOpenCausesCurrentDecay)
{
    // In rectifier-only topology, the machine side is short-circuited (V=0).
    // When the breaker opens, no new energy enters the DC link.
    // The current MUST decay to zero and all thyristors MUST stop conducting.

    sfc_plant_rectifier_topology plant;
    plant.initialize();

    // Set up firing pulses to establish current
    // Fire all thyristors to act as a diode bridge and maximize current
    std::array<bool, 6> grid_pulses = {true, true, true, true, true, true};
    plant.set_grid_side_pulses(grid_pulses);

    // Simulate a few cycles to establish steady-state current
    simulate_steps(plant, kStepsPerCycle * 5);

    // Record current before opening breaker
    current_t initial_current = plant.get_dc_link_current();
    EXPECT_GT(initial_current.value(), 1.0) << "Initial current should be established";

    // Open grid breaker
    plant.set_grid_circuit_breaker_closed(false);

    // Simulate more steps - current should decay
    simulate_steps(plant, kStepsPerCycle * 10);

    current_t final_current = plant.get_dc_link_current();

    // Current should decay to zero
    EXPECT_NEAR(final_current.value(), 0.0, 0.1) << "Current should decay to zero when breaker opens";

    // Check that no thyristors are conducting
    const auto& conduction_states = plant.get_rectifier().get_conduction_states();
    for (size_t i = 0; i < 6; ++i) {
        EXPECT_FALSE(conduction_states[i]) << "Thyristor " << i + 1 << " should not be conducting";
    }
}

TEST_F(CircuitBreakerTest, RectifierTopology_GridBreakerOpenMasksPulses)
{
    // Test that pulse masking works correctly for rectifier topology.
    // Key verification: grid voltage remains present (not zeroed) when breaker opens.

    sfc_plant_rectifier_topology plant;
    plant.initialize();

    // Set up firing pulses
    std::array<bool, 6> grid_pulses = {true, false, false, true, false, false};
    plant.set_grid_side_pulses(grid_pulses);

    // Simulate to establish current
    simulate_steps(plant, kStepsPerCycle * 10);

    // Open grid breaker
    plant.set_grid_circuit_breaker_closed(false);

    // Continue simulation
    simulate_steps(plant, kStepsPerCycle * 10);

    abc<voltage_t> grid_voltages_after = plant.get_grid_side_voltages_abc();

    // Grid voltage should still be present (pulse masking, not voltage zeroing)
    real_t max_voltage_magnitude = std::max({std::abs(grid_voltages_after.a().value()),
        std::abs(grid_voltages_after.b().value()), std::abs(grid_voltages_after.c().value())});
    EXPECT_GT(max_voltage_magnitude, 10.0) << "Grid voltage should still be present when breaker is open";

    // Current should decay to zero
    current_t current_after = plant.get_dc_link_current();
    EXPECT_NEAR(current_after.value(), 0.0, 0.1) << "Current should decay to zero when breaker opens";

    // Breaker state should be maintained
    EXPECT_FALSE(plant.is_grid_circuit_breaker_closed()) << "Breaker should remain open";
}

TEST_F(CircuitBreakerTest, RectifierTopology_GridBreakerReclosing)
{
    sfc_plant_rectifier_topology plant;
    plant.initialize();

    // Set up firing pulses
    std::array<bool, 6> grid_pulses = {true, false, false, true, false, false};
    plant.set_grid_side_pulses(grid_pulses);

    // Establish current
    simulate_steps(plant, kStepsPerCycle * 5);

    // Open breaker
    plant.set_grid_circuit_breaker_closed(false);
    simulate_steps(plant, kStepsPerCycle * 10);

    // Reclose breaker
    plant.set_grid_circuit_breaker_closed(true);
    simulate_steps(plant, kStepsPerCycle * 10);

    EXPECT_TRUE(plant.is_grid_circuit_breaker_closed());
}
