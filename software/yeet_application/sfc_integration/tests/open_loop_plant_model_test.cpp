#include <gtest/gtest.h>

#include "sfc_integration/plant_open_loop_simulation_builder.hpp"
#include "sfc_integration/sfc_plant_model_datamodel.hpp"
#include "sfc_loggers.hpp"
#include "yeet_testing/simulator_gtest.hpp"

using namespace yeet::application::sfc::integration;
using namespace yeet;

class SfcOpenLoopPlantModel : public ::testing::TestWithParam<sfc_plant_model_datamodel::topology_type> {
public:
    // using parameter_types ::parameter_types;

    void SetUp() override
    {
        // simulator.configure_sampler_sampling_rate(200e-6);
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

TEST_P(SfcOpenLoopPlantModel, OpenLoopStepResponseTest)
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

INSTANTIATE_TEST_SUITE_P(TopologyTests,
    SfcOpenLoopPlantModel,
    ::testing::Values(
        sfc_plant_model_datamodel::topology_type::simulink, sfc_plant_model_datamodel::topology_type::internal),
    [](const testing::TestParamInfo<SfcOpenLoopPlantModel::ParamType>& info) {
        return (info.param == sfc_plant_model_datamodel::topology_type::simulink) ? "SimulinkModel" : "InternalModel";
    });
