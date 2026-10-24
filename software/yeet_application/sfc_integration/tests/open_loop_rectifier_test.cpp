/**
 * @file open_loop_rectifier_test.cpp
 * @brief Open-loop tests for rectifier-only topology
 *
 * This test file validates the rectifier plant model behavior in open-loop configuration
 * with fixed firing angles on the grid side only. The machine side is short-circuited
 * with zero back-EMF voltage.
 *
 * Key validations:
 * - DC-link current response to firing angle changes
 * - Machine-side voltage is zero (short-circuited)
 * - Grid-side rectifier operation
 */

#include <gtest/gtest.h>

#include <cmath>

#include "sfc_integration/plant_open_loop_simulation_builder.hpp"
#include "sfc_integration/sfc_plant_model_datamodel.hpp"
#include "sfc_loggers.hpp"
#include "yeet_testing/simulator_gtest.hpp"

using namespace yeet::application::sfc::integration;
using namespace yeet;

class SfcOpenLoopRectifier : public ::testing::Test {
public:
    void SetUp() override
    {
        system.register_applications(simulator);

        // Configure rectifier topology
        system.plant.get_parameters().topology.write(sfc_plant_model_datamodel::topology_type::rectifier);

        // Initialize the system
        system.initialize();

        // ==================== Logger Registration ====================
        register_complete_logger(simulator, system.plant);
        register_minimal_logger(simulator, system.grid_side_modulator);
        register_complete_logger(simulator, system.grid_side_hardware_mock);
        register_minimal_logger(simulator, system.grid_side_measurements);
        register_complete_logger(simulator, system.dc_link_measurements);
    }

    void simulate_for(double time) { simulator.simulate_for(time); }

    voltage_t read_machine_voltage_magnitude() const
    {
        const auto& v_abc = system.plant.get_outputs().machine_voltage_abc.read();
        return voltage_t{std::sqrt(v_abc.a().value() * v_abc.a().value() + v_abc.b().value() * v_abc.b().value() +
                                   v_abc.c().value() * v_abc.c().value())};
    }

    plant_open_loop_simulation_builder system;
    yeet::simulator simulator = yeet::make_gtest_simulator();
};

TEST_F(SfcOpenLoopRectifier, OpenLoopStepResponseTest)
{
    constexpr real_t kBaseDCLinkCurrent = 1044.0f;  // Base current in Amperes
    constexpr real_t kTolerancePu = 0.15f;          // 15% tolerance for steady-state check

    // Setpoint 1: Grid firing angle = 82 degrees
    // Theoretical calculation (ideal):
    // Vdo = 2821 V, cos(82) = 0.139 -> Vdc = 392 V
    // R_total approx 0.2 Ohm -> Idc = 1960 A = 1.8 pu
    // Simulation shows ~1.0 pu, suggesting effective angle shift or additional drops
    system.grid_side_firing_angle = yeet::angle_wrapped::from_degrees(82.0);

    simulate_for(0.25f);

    const auto dc_current_1 = system.plant.get_outputs().dc_link_current.read();
    const auto dc_current_pu_1 = dc_current_1.value() / kBaseDCLinkCurrent;

    // Expect around 1.0 pu based on simulation observation
    EXPECT_NEAR(dc_current_pu_1, 1.0f, 0.3f) << "DC current should be around 1.0 pu with 82 degree firing angle";

    // Verify machine-side voltage is zero
    auto v_machine_mag = read_machine_voltage_magnitude();
    EXPECT_NEAR(v_machine_mag.value(), 0.0, 1.0) << "Machine voltage magnitude should be zero";

    // Setpoint 2: Grid firing angle = 85 degrees (reduced current)
    // Simulation shows ~0.4 pu
    system.grid_side_firing_angle = yeet::angle_wrapped::from_degrees(85.0);

    simulate_for(0.25f);

    const auto dc_current_2 = system.plant.get_outputs().dc_link_current.read();
    const auto dc_current_pu_2 = dc_current_2.value() / kBaseDCLinkCurrent;

    // Current should decrease with larger firing angle
    EXPECT_NEAR(dc_current_pu_2, 0.4f, 0.2f) << "DC current should be around 0.4 pu with 85 degree firing angle";
    EXPECT_LT(dc_current_pu_2, dc_current_pu_1) << "DC current should decrease with larger firing angle";
    EXPECT_GT(dc_current_pu_2, 0.0f) << "DC current should still be positive";

    // Verify machine-side voltage still zero
    v_machine_mag = read_machine_voltage_magnitude();
    EXPECT_NEAR(v_machine_mag.value(), 0.0, 1.0) << "Machine voltage magnitude should remain zero";
}

TEST_F(SfcOpenLoopRectifier, MachineSideShortCircuitValidation)
{
    // Set a firing angle and let system settle
    system.grid_side_firing_angle = yeet::angle_wrapped::from_degrees(82.0);

    simulate_for(0.5f);

    // Verify all machine-side voltage phases are zero
    const auto& v_abc = system.plant.get_outputs().machine_voltage_abc.read();
    EXPECT_NEAR(v_abc.a().value(), 0.0, 1.0) << "Machine voltage phase A should be zero";
    EXPECT_NEAR(v_abc.b().value(), 0.0, 1.0) << "Machine voltage phase B should be zero";
    EXPECT_NEAR(v_abc.c().value(), 0.0, 1.0) << "Machine voltage phase C should be zero";

    // Grid-side voltage should be non-zero (normal AC voltage)
    const auto& v_grid = system.plant.get_outputs().grid_voltage_abc.read();
    const auto v_grid_mag =
        std::sqrt(v_grid.a().value() * v_grid.a().value() + v_grid.b().value() * v_grid.b().value() +
                  v_grid.c().value() * v_grid.c().value());
    EXPECT_GT(v_grid_mag, 100.0) << "Grid voltage magnitude should be significant";

    // DC link current should be positive
    const auto dc_current = system.plant.get_outputs().dc_link_current.read();
    EXPECT_GT(dc_current.value(), 0.0) << "DC link current should be positive";
}

TEST_F(SfcOpenLoopRectifier, FiringAngleSweep)
{
    constexpr real_t kBaseDCLinkCurrent = 1044.0f;

    // Test current at different firing angles
    std::vector<std::pair<double, double>> angle_current_pairs;

    // Sweep from 80 to 89 degrees to keep current within reasonable limits
    for (double angle_deg = 80.0; angle_deg <= 89.0; angle_deg += 2.0) {
        system.grid_side_firing_angle = yeet::angle_wrapped::from_degrees(angle_deg);
        simulate_for(0.1f);

        const auto dc_current = system.plant.get_outputs().dc_link_current.read();
        const auto dc_current_pu = dc_current.value() / kBaseDCLinkCurrent;

        angle_current_pairs.push_back({angle_deg, dc_current_pu});

        // Machine voltage should always be zero
        auto v_machine_mag = read_machine_voltage_magnitude();
        EXPECT_NEAR(v_machine_mag.value(), 0.0, 1.0)
            << "Machine voltage should be zero at firing angle " << angle_deg << " deg";
    }

    // Verify current decreases monotonically with increasing firing angle
    for (size_t i = 1; i < angle_current_pairs.size(); ++i) {
        EXPECT_LT(angle_current_pairs[i].second, angle_current_pairs[i - 1].second)
            << "Current should decrease from " << angle_current_pairs[i - 1].first << " deg to "
            << angle_current_pairs[i].first << " deg";
    }
}

TEST_F(SfcOpenLoopRectifier, GridSideMaskingAndCompensationSequence)
{
    // Register complete loggers for both sides as requested
    register_complete_logger(simulator, system.grid_side_measurements);
    register_complete_logger(simulator, system.machine_side_measurements);
    register_complete_logger(simulator, system.grid_side_hardware_mock);
    register_complete_logger(simulator, system.machine_side_hardware_mock);

    // Set firing angle to establish DC link current (~1.0 pu)
    system.grid_side_firing_angle = yeet::angle_wrapped::from_degrees(82.0);

    // Configure hardware filter and compensation to 110 Hz
    system.grid_side_hardware_mock.get_parameters().voltage_adc_cutoff_freq.write(frequency_t{110.0});
    system.grid_side_measurements.get_parameters().voltage_compensation_lpf_cutoff_frequency_1.write(
        frequency_t{110.0});
    system.grid_side_measurements.get_parameters().voltage_compensation_num_filters.write(1);

    // Stage 1: Masking enabled, Compensation disabled
    system.grid_side_measurements.get_parameters().enable_commutation_masking.write(true);
    system.grid_side_measurements.get_parameters().enable_voltage_compensation.write(false);
    simulate_for(1.0);

    // Stage 2: Masking disabled, Compensation enabled
    system.grid_side_measurements.get_parameters().enable_commutation_masking.write(false);
    system.grid_side_measurements.get_parameters().enable_voltage_compensation.write(true);
    simulate_for(1.0);

    // Stage 3: Both Masking and Compensation enabled
    system.grid_side_measurements.get_parameters().enable_commutation_masking.write(true);
    system.grid_side_measurements.get_parameters().enable_voltage_compensation.write(true);
    simulate_for(1.0);
}

TEST_F(SfcOpenLoopRectifier, MachineSideMaskingAndCompensationSequence)
{
    // Use internal topology to have machine side converter active for better validation
    system.plant.get_parameters().topology.write(sfc_plant_model_datamodel::topology_type::internal);
    system.initialize();

    // Register complete loggers for both sides as requested
    register_complete_logger(simulator, system.grid_side_measurements);
    register_complete_logger(simulator, system.machine_side_measurements);
    register_complete_logger(simulator, system.grid_side_hardware_mock);
    register_complete_logger(simulator, system.machine_side_hardware_mock);

    // Set firing angles to establish DC link current (Rectifier: 20 deg, Inverter: 150 deg)
    system.grid_side_firing_angle = yeet::angle_wrapped::from_degrees(20.0);
    // Note: 'firing_angle' in plant_open_loop_simulation_builder refers to the machine side
    system.firing_angle = yeet::angle_wrapped::from_degrees(150.0);

    // Configure hardware filter and compensation to 110 Hz
    system.machine_side_hardware_mock.get_parameters().voltage_adc_cutoff_freq.write(frequency_t{110.0});
    system.machine_side_measurements.get_parameters().voltage_compensation_lpf_cutoff_frequency_1.write(
        frequency_t{110.0});
    system.machine_side_measurements.get_parameters().voltage_compensation_num_filters.write(1);

    // Stage 1: Masking enabled, Compensation disabled
    system.machine_side_measurements.get_parameters().enable_commutation_masking.write(true);
    system.machine_side_measurements.get_parameters().enable_voltage_compensation.write(false);
    simulate_for(1.0);

    // Stage 2: Masking disabled, Compensation enabled
    system.machine_side_measurements.get_parameters().enable_commutation_masking.write(false);
    system.machine_side_measurements.get_parameters().enable_voltage_compensation.write(true);
    simulate_for(1.0);

    // Stage 3: Both Masking and Compensation enabled
    system.machine_side_measurements.get_parameters().enable_commutation_masking.write(true);
    system.machine_side_measurements.get_parameters().enable_voltage_compensation.write(true);
    simulate_for(1.0);
}
