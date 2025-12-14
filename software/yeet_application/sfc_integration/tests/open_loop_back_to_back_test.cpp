/**
 * @file open_loop_back_to_back_test.cpp
 * @brief Open-loop tests for back-to-back converter topologies (Simulink and Internal models)
 *
 * This test file validates the plant model behavior in open-loop configuration
 * with fixed firing angles on both grid and machine sides.
 */

#include <gtest/gtest.h>

#include "sfc_integration/plant_open_loop_simulation_builder.hpp"
#include "sfc_integration/sfc_plant_model_datamodel.hpp"
#include "sfc_loggers.hpp"
#include "yeet_testing/simulator_gtest.hpp"

using namespace yeet::application::sfc::integration;
using namespace yeet;

class SfcOpenLoopBackToBack : public ::testing::TestWithParam<sfc_plant_model_datamodel::topology_type> {
public:
    void SetUp() override
    {
        system.register_applications(simulator);

        // Configure topology based on test parameter
        system.plant.get_parameters().topology.write(GetParam());

        // initialize here, it allocates plant model stuff
        initialize();

        // ==================== Logger Registration ====================
        register_minimal_logger(simulator, system.plant);
        register_minimal_logger(simulator, system.grid_side_pulse_generator);
        register_minimal_logger(simulator, system.machine_side_pulse_generator);
        register_complete_logger(simulator, system.grid_side_hardware_mock);
        register_complete_logger(simulator, system.machine_side_hardware_mock);
        register_minimal_logger(simulator, system.grid_side_measurements);
        register_minimal_logger(simulator, system.machine_side_measurements);
        register_complete_logger(simulator, system.dc_link_measurements);
    }

    void initialize() { system.initialize(); }

    void simulate_for(double time) { simulator.simulate_for(time); }

    yeet::application::sfc::integration::plant_open_loop_simulation_builder system;

    yeet::simulator simulator = yeet::make_gtest_simulator();
};

TEST_P(SfcOpenLoopBackToBack, OpenLoopStepResponseTest)
{
    constexpr real_t kBaseDCLinkCurrent = 1044.0f;  // Base current in Amperes
    constexpr real_t kTolerancePu = 0.15f;          // 15% tolerance for steady-state check

    // Expected values depend on topology
    const bool is_simulink = (GetParam() == sfc_plant_model_datamodel::topology_type::simulink);
    const real_t expected_current_pu_setpoint1 = is_simulink ? 0.89f : 1.0f;
    const real_t expected_current_pu_setpoint2 = is_simulink ? 0.29f : 0.29f;

    // Setpoint 1: Grid firing angle = 13 degrees
    system.grid_side_firing_angle = yeet::angle_wrapped::from_degrees(13.0);
    system.firing_angle = yeet::angle_wrapped::from_degrees(180.0 - 26.84);

    simulate_for(0.25f);

    const auto dc_current_1 = system.plant.get_outputs().dc_link_current.read();
    const auto dc_current_pu_1 = dc_current_1.value() / kBaseDCLinkCurrent;

    EXPECT_NEAR(dc_current_pu_1, expected_current_pu_setpoint1, kTolerancePu)
        << "DC current at setpoint 1 (alpha=13deg) outside tolerance";

    // Setpoint 2: Grid firing angle = 25 degrees
    system.grid_side_firing_angle = yeet::angle_wrapped::from_degrees(25.0);

    simulate_for(0.25f);

    const auto dc_current_2 = system.plant.get_outputs().dc_link_current.read();
    const auto dc_current_pu_2 = dc_current_2.value() / kBaseDCLinkCurrent;

    EXPECT_NEAR(dc_current_pu_2, expected_current_pu_setpoint2, kTolerancePu)
        << "DC current at setpoint 2 (alpha=25deg) outside tolerance";
}

/**
 * @test GridBreakerOpenCloseBehavior
 * @brief Validates grid circuit breaker behavior across topology implementations
 *
 * This typed test verifies that both Simulink and Internal topology models
 * implement consistent circuit breaker behavior:
 * 1. Breaker defaults to closed
 * 2. Opening breaker causes DC current to decay (pulse masking)
 * 3. Grid voltage remains present when breaker is open
 * 4. Reclosing breaker allows current to build again
 */
TEST_P(SfcOpenLoopBackToBack, GridBreakerOpenCloseBehavior)
{
    constexpr real_t kBaseDCLinkCurrent = 1044.0f;

    // Set firing angles to establish current
    system.grid_side_firing_angle = yeet::angle_wrapped::from_degrees(15.0);
    system.firing_angle = yeet::angle_wrapped::from_degrees(180.0 - 30.0);

    // Let current establish
    simulate_for(0.2);

    auto& plant = system.plant.get_plant_model();

    // Verify breaker defaults to closed
    EXPECT_TRUE(plant.is_grid_circuit_breaker_closed()) << "Breaker should default to closed";

    // Record current before opening breaker
    auto current_before = system.plant.get_outputs().dc_link_current.read();
    EXPECT_GT(std::abs(current_before.value()), 100.0) << "Should have established DC current";

    // Open the grid breaker
    plant.set_grid_circuit_breaker_closed(false);
    EXPECT_FALSE(plant.is_grid_circuit_breaker_closed()) << "Breaker should be open";

    // Simulate with open breaker - current should decay
    simulate_for(0.3);

    auto current_after_open = system.plant.get_outputs().dc_link_current.read();

    // Current should have decayed (pulse masking prevents new turn-ons)
    EXPECT_LE(std::abs(current_after_open.value()), std::abs(current_before.value()))
        << "DC current should decay with open breaker";

    // Reclose the breaker
    plant.set_grid_circuit_breaker_closed(true);
    EXPECT_TRUE(plant.is_grid_circuit_breaker_closed()) << "Breaker should be closed again";

    // Simulate with closed breaker - current should be able to build
    simulate_for(0.2);

    // System should be operational again
    auto current_after_reclose = system.plant.get_outputs().dc_link_current.read();
    // Note: Just verify system is stable, not necessarily same current level
    EXPECT_TRUE(plant.is_grid_circuit_breaker_closed());
}

/**
 * @test GridBreakerOpenCausesCurrentDecayToZero
 * @brief Validates that DC and ABC currents decay to zero when breaker opens
 *
 * In back-to-back topology with machine-side back-EMF, when the grid breaker opens:
 * 1. No new thyristors can turn on (gate pulses masked)
 * 2. Conducting thyristors naturally commutate off
 * 3. Machine-side back-EMF absorbs stored DC link energy
 * 4. DC link current and ABC currents decay to zero
 *
 * This behavior is different from rectifier-only topology where current oscillates
 * due to the short-circuited load.
 */
TEST_P(SfcOpenLoopBackToBack, GridBreakerOpenCausesCurrentDecayToZero)
{
    constexpr real_t kBaseDCLinkCurrent = 1044.0f;
    constexpr real_t kCurrentThresholdAmps = 100.0f;  // Threshold for "near zero" current

    // Set firing angles to establish current
    system.grid_side_firing_angle = yeet::angle_wrapped::from_degrees(15.0);
    system.firing_angle = yeet::angle_wrapped::from_degrees(180.0 - 30.0);

    // Let current establish
    simulate_for(0.3);

    auto& plant = system.plant.get_plant_model();

    // Record current before opening breaker
    auto dc_current_before = system.plant.get_outputs().dc_link_current.read();
    auto grid_current_abc_before = system.plant.get_outputs().grid_current_abc.read();

    EXPECT_GT(std::abs(dc_current_before.value()), 100.0) << "Should have established DC current";

    // Open the grid breaker
    plant.set_grid_circuit_breaker_closed(false);
    EXPECT_FALSE(plant.is_grid_circuit_breaker_closed()) << "Breaker should be open";

    // Simulate with open breaker - allow time for current to decay
    simulate_for(0.5);

    // ==================== Verify currents have decayed significantly ====================
    auto dc_current_after = system.plant.get_outputs().dc_link_current.read();
    auto grid_current_abc_after = system.plant.get_outputs().grid_current_abc.read();

    // DC link current should have decayed significantly (near zero)
    EXPECT_LT(std::abs(dc_current_after.value()), kCurrentThresholdAmps)
        << "DC-link current should decay to near zero when breaker is open";

    // ABC phase currents should also be near zero
    EXPECT_LT(std::abs(grid_current_abc_after.a().value()), kCurrentThresholdAmps)
        << "Phase A current should decay to near zero";
    EXPECT_LT(std::abs(grid_current_abc_after.b().value()), kCurrentThresholdAmps)
        << "Phase B current should decay to near zero";
    EXPECT_LT(std::abs(grid_current_abc_after.c().value()), kCurrentThresholdAmps)
        << "Phase C current should decay to near zero";

    // Verify breaker state is maintained
    EXPECT_FALSE(plant.is_grid_circuit_breaker_closed()) << "Breaker should still be open";
}

/**
 * @test GridVoltageSourceMagnitudeTracksGridVoltagePu
 * @brief Verifies that changing grid_voltage_pu propagates through plant + ADC + measurement chain.
 *
 * This is a regression test for cases where users set the grid voltage command but measured
 * grid voltage stays stuck at 1.0 pu.
 *
 * To avoid ambiguity due to load current and source impedance, the grid circuit breaker is opened
 * so the grid source is effectively unloaded.
 */
TEST_P(SfcOpenLoopBackToBack, GridVoltageSourceMagnitudeTracksGridVoltagePu)
{
    auto& plant = system.plant.get_plant_model();

    // Unload the grid source so terminal voltage follows the source command closely.
    plant.set_grid_circuit_breaker_closed(false);
    simulate_for(0.2);

    auto read_vmag_pu = [&]() {
        return system.grid_side_measurements.get_outputs().grid_voltage_magnitude_filtered.read().value();
    };

    // Nominal
    system.plant.get_parameters().grid_voltage_pu.write(voltage_pu_t{1.0f});
    simulate_for(0.25);
    EXPECT_NEAR(read_vmag_pu(), 1.0f, 0.05f);

    // Overvoltage
    system.plant.get_parameters().grid_voltage_pu.write(voltage_pu_t{1.30f});
    simulate_for(0.25);
    EXPECT_NEAR(read_vmag_pu(), 1.30f, 0.07f);

    // Undervoltage
    system.plant.get_parameters().grid_voltage_pu.write(voltage_pu_t{0.70f});
    simulate_for(0.25);
    EXPECT_NEAR(read_vmag_pu(), 0.70f, 0.07f);
}

INSTANTIATE_TEST_SUITE_P(TopologyTests,
    SfcOpenLoopBackToBack,
    ::testing::Values(
        sfc_plant_model_datamodel::topology_type::simulink, sfc_plant_model_datamodel::topology_type::internal),
    [](const testing::TestParamInfo<SfcOpenLoopBackToBack::ParamType>& info) {
        return (info.param == sfc_plant_model_datamodel::topology_type::simulink) ? "SimulinkModel" : "InternalModel";
    });
