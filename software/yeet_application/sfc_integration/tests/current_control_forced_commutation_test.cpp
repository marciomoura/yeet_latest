#include <gtest/gtest.h>

#include "sfc_integration/plant_current_control_with_modulators_simulation_builder.hpp"
#include "sfc_integration/sfc_plant_model_datamodel.hpp"
#include "sfc_loggers.hpp"
#include "yeet_testing/simulator_checker.hpp"
#include "yeet_testing/simulator_gtest.hpp"

using namespace yeet;
using namespace yeet::application::sfc::integration;

// ---------- Param struct (machine frequency, voltage, and topology are parameterized) ----------
struct forced_comm_init {
    float freq_pu;
    float voltage_pu;
    sfc_plant_model_datamodel::topology_type topology;
};

std::string ForcedCommParamName(const ::testing::TestParamInfo<forced_comm_init>& info)
{
    // Create a simple alphanumeric name (e.g. Freq005_Volt020_Simulink)
    const auto scale = [](float v) {
        char buf[16];
        std::snprintf(buf, sizeof(buf), "%03d", int(v * 100));  // scale by 100
        return std::string(buf);
    };
    const std::string topology_str =
        (info.param.topology == sfc_plant_model_datamodel::topology_type::simulink) ? "Simulink" : "Internal";
    return "Freq" + scale(info.param.freq_pu) + "_Volt" + scale(info.param.voltage_pu) + "_" + topology_str;
}

// ---------- Test Fixture ----------
class CurrentControlForcedCommutationParamTest : public ::testing::TestWithParam<forced_comm_init> {
public:
    void SetUp() override
    {
        simulator.configure_sampler_sampling_rate(100e-6);
        system.register_applications(simulator);

        // Configure topology based on test parameter
        const auto& p = GetParam();
        system.plant.get_parameters().topology.write(p.topology);

        system.initialize();

        // // Force the plant to fixed speed mode, constant voltage, and the monitor to forced commutation
        system.plant.get_parameters().operation_mode.write(
            sfc_plant_model_datamodel::machine_operation_mode::fixed_speed);
        system.operation_mode_monitor.get_parameters().force_to_forced_commutation.write(true);
        // system.plant.get_parameters().voltage_control_mode.write(
        //     sfc_plant_model_datamodel::machine_voltage_control_mode::fixed_voltage);

        // Configure speed and voltage after initialization

        system.plant.get_parameters().fixed_machine_voltage.write(voltage_pu_t{p.voltage_pu});
        system.plant.get_parameters().voltage_control_mode.write(
            sfc_plant_model_datamodel::machine_voltage_control_mode::fixed_voltage);

        system.plant.get_plant_model().set_machine_frequency_pu(frequency_pu_t{p.freq_pu});
        system.plant.get_plant_model().set_machine_voltage_pu(voltage_pu_t{p.voltage_pu});

        register_complete_logger(simulator, system.current_controller);
        register_complete_logger(simulator, system.dc_link_measurements);
        register_complete_logger(simulator, system.operation_mode_monitor);
        register_complete_logger(simulator, system.grid_side_modulator);
        register_complete_logger(simulator, system.machine_side_modulator);
        register_complete_logger(simulator, system.machine_measurements);

        // Starting with disabled modulators
        system.grid_side_modulator.get_parameters().enable.write(false);
        system.machine_side_modulator.get_parameters().enable.write(false);
        simulator.simulate_for(0.2);
        system.grid_side_modulator.get_parameters().enable.write(true);
        system.machine_side_modulator.get_parameters().enable.write(true);
    }

    // Current pased as reference
    std::shared_ptr<checker_user_activation> create_current_error_checker(
        current_pu_t& current_reference, current_pu_t margin)
    {
        return simulator.register_checker(yeet::make_range_checker<current_pu_t>(
            "dc current error (forced)",
            // Error = reference - measured
            [&]() -> current_pu_t {
                const current_pu_t measured =
                    system.dc_link_measurements.get_outputs().dc_current_heavy_filtered_pu.read();
                return current_reference - measured;
            },
            -margin, margin, yeet::checker_activation_always{}));
    }

protected:
    plant_current_control_with_modulators_simulation_builder system;
    yeet::simulator simulator = yeet::make_gtest_simulator();
};

// ---------- Parameterized Test: Three hardcoded current steps (0.2 -> 0.5 -> 1.0 pu) ----------
TEST_P(CurrentControlForcedCommutationParamTest, CurrentRegulationThreeStepProfile)
{
    current_pu_t current_reference{0.0f};  // External reference variable tracked by checker
    auto checker = create_current_error_checker(current_reference, current_pu_t{0.2f});

    // Step 1: 0.2 pu
    checker->deactivate();
    current_reference = current_pu_t{0.2f};
    system.current_reference = current_reference;

    // // First 0.2s of Iref != 0: do not enable modulation
    // EXPECT_TRUE(simulator.simulate_for(0.2));

    // // Enable modulation after the initial oscillation window
    // system.grid_side_modulator.get_parameters().enable.write(true);
    // system.machine_side_modulator.get_parameters().enable.write(true);

    EXPECT_TRUE(simulator.simulate_for(0.5));  // allow more time for transient (1.0s total)
    EXPECT_EQ(system.operation_mode_monitor.get_outputs().operating_mode.read(),
        yeet::application::sfc::sfc_operating_mode_t::forced_commutation);
    // checker->activate();
    EXPECT_TRUE(simulator.simulate_for(0.5));  // steady-state window

    // Step 2: 0.5 pu
    checker->deactivate();
    current_reference = current_pu_t{0.5f};
    system.current_reference = current_reference;
    EXPECT_TRUE(simulator.simulate_for(0.5));
    EXPECT_EQ(system.operation_mode_monitor.get_outputs().operating_mode.read(),
        yeet::application::sfc::sfc_operating_mode_t::forced_commutation);
    // checker->activate();
    EXPECT_TRUE(simulator.simulate_for(0.5));  // steady-state window

    // Step 3: 1.0 pu
    checker->deactivate();
    current_reference = current_pu_t{1.0f};
    system.current_reference = current_reference;
    EXPECT_TRUE(simulator.simulate_for(0.5));
    EXPECT_EQ(system.operation_mode_monitor.get_outputs().operating_mode.read(),
        yeet::application::sfc::sfc_operating_mode_t::forced_commutation);
    // checker->activate();
    EXPECT_TRUE(simulator.simulate_for(0.6));  // steady-state window
}

// ---------- Parameter Set ----------
INSTANTIATE_TEST_SUITE_P(ForcedLowSpeedOperatingPoints,
    CurrentControlForcedCommutationParamTest,
    ::testing::Values(
        // Simulink model
        forced_comm_init{0.01f, 0.10f, sfc_plant_model_datamodel::topology_type::simulink},
        forced_comm_init{0.05f, 0.20f, sfc_plant_model_datamodel::topology_type::simulink},
        forced_comm_init{0.1f, 0.20f, sfc_plant_model_datamodel::topology_type::simulink},
        // Internal model
        forced_comm_init{0.01f, 0.10f, sfc_plant_model_datamodel::topology_type::internal},
        forced_comm_init{0.05f, 0.20f, sfc_plant_model_datamodel::topology_type::internal},
        forced_comm_init{0.1f, 0.20f, sfc_plant_model_datamodel::topology_type::internal}),
    ForcedCommParamName);
