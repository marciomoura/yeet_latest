#include <gtest/gtest.h>

#include "sfc_integration/plant_current_control_with_modulators_simulation_builder.hpp"
#include "sfc_loggers.hpp"
#include "yeet_testing/simulator_checker.hpp"
#include "yeet_testing/simulator_gtest.hpp"

using namespace yeet;
using namespace yeet::application::sfc::integration;

// ---------- Param struct (only machine frequency & voltage are parameterized) ----------
struct forced_comm_init {
    float freq_pu;
    float voltage_pu;
};

std::string ForcedCommParamName(const ::testing::TestParamInfo<forced_comm_init>& info)
{
    // Create a simple alphanumeric name (e.g. Freq005_Volt020)
    const auto scale = [](float v) {
        char buf[16];
        std::snprintf(buf, sizeof(buf), "%03d", int(v * 100));  // scale by 100
        return std::string(buf);
    };
    return "Freq" + scale(info.param.freq_pu) + "_Volt" + scale(info.param.voltage_pu);
}

// ---------- Test Fixture ----------
class CurrentControlForcedCommutationParamTest : public ::testing::TestWithParam<forced_comm_init> {
public:
    void SetUp() override
    {
        simulator.configure_sampler_sampling_rate(100e-6);
        simulation_system.register_applications(simulator);
        simulation_system.initialize();

        register_complete_logger(simulator, simulation_system.plant);
        register_complete_logger(simulator, simulation_system.current_controller);
        register_complete_logger(simulator, simulation_system.dc_link_measurements);
        register_complete_logger(simulator, simulation_system.operation_mode_monitor);
        register_complete_logger(simulator, simulation_system.grid_side_modulator);
        register_complete_logger(simulator, simulation_system.machine_side_modulator);
        register_complete_logger(simulator, simulation_system.machine_measurements);

        const auto& p = GetParam();
        simulation_system.plant.get_plant_model().set_machine_frequency_pu(frequency_pu_t{p.freq_pu});
        // Initialize machine voltage magnitude to correct machine-voltage value (1980 V)
        simulation_system.plant.get_plant_model().set_machine_voltage(voltage_t{1980.0f * p.voltage_pu});

        simulation_system.grid_side_pulse_generator.get_parameters().enable.write(false);
        simulation_system.machine_side_pulse_generator.get_parameters().enable.write(false);
        simulator.simulate_for(0.2);
        simulation_system.grid_side_pulse_generator.get_parameters().enable.write(true);
        simulation_system.machine_side_pulse_generator.get_parameters().enable.write(true);
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
                    simulation_system.dc_link_measurements.get_outputs().dc_current_heavy_filtered_pu.read();
                return current_reference - measured;
            },
            -margin, margin, yeet::checker_activation_always{}));
    }

    std::shared_ptr<checker_user_activation> create_machine_current_sum_checker()
    {
        return simulator.register_checker(yeet::make_range_checker<current_pu_t>(
            "machine current sum zero",
            [&]() -> current_pu_t {
                const auto currents = simulation_system.machine_measurements.get_outputs().machine_current_abc.read();
                current_pu_t sum = currents.a() + currents.b() + currents.c();
                return sum;
            },
            current_pu_t{-0.1f}, current_pu_t{0.1f}, yeet::checker_activation_always{}));
    }

    std::shared_ptr<checker_user_activation> create_grid_current_sum_checker()
    {
        return simulator.register_checker(yeet::make_range_checker<current_pu_t>(
            "grid current sum zero",
            [&]() -> current_pu_t {
                const auto currents = simulation_system.grid_measurements.get_outputs().grid_current_abc.read();
                current_pu_t sum = currents.a() + currents.b() + currents.c();
                return sum;
            },
            current_pu_t{-0.1f}, current_pu_t{0.1f}, yeet::checker_activation_always{}));
    }

    std::shared_ptr<checker_user_activation> create_dc_current_magnitude_checker()
    {
        return simulator.register_checker(yeet::make_range_checker<current_pu_t>(
            "dc current magnitude equality (machine vs dc_link)",
            [&]() -> current_pu_t {
                const auto machine_currents =
                    simulation_system.machine_measurements.get_outputs().machine_current_abc.read();
                const auto dc_current =
                    simulation_system.dc_link_measurements.get_outputs().dc_current_heavy_filtered_pu.read();

                const current_pu_t machine_peak_current = current_pu_t{std::max({std::abs(machine_currents.a().value()),
                    std::abs(machine_currents.b().value()), std::abs(machine_currents.c().value())})};

                return abs(machine_peak_current) - abs(dc_current);
            },
            current_pu_t{-0.1f}, current_pu_t{0.61}, yeet::checker_activation_always{}));
    }

protected:
    plant_current_control_with_modulators_simulation_builder simulation_system;
    yeet::simulator simulator = yeet::make_gtest_simulator();
};

// ---------- Parameterized Test: Three hardcoded current steps (0.2 -> 0.5 -> 1.0 pu) ----------
TEST_P(CurrentControlForcedCommutationParamTest, CurrentRegulationThreeStepProfile)
{
    current_pu_t current_reference{0.0f};  // External reference variable tracked by checker
    auto dc_reference_checker = create_current_error_checker(current_reference, current_pu_t{0.2f});
    auto machine_sum_checker = create_machine_current_sum_checker();
    auto grid_sum_checker = create_grid_current_sum_checker();
    auto dc_magnitude_checker = create_dc_current_magnitude_checker();

    // Step 1: 0.2 pu
    // checker->deactivate();
    // dc_magnitude_checker->deactivate();
    // machine_sum_checker->deactivate();
    // grid_sum_checker->deactivate();

    dc_reference_checker->deactivate();
    machine_sum_checker->deactivate();
    grid_sum_checker->deactivate();
    dc_magnitude_checker->deactivate();

    current_reference = current_pu_t{0.4f};
    simulation_system.current_reference = current_reference;
    EXPECT_TRUE(simulator.simulate_for(0.2));  // transient

    dc_reference_checker->activate();
    // Keep these disabled to test original model behavior
    // machine_sum_checker->activate();
    // grid_sum_checker->activate();
    dc_magnitude_checker->activate();

    EXPECT_TRUE(simulator.simulate_for(0.5));  // transient
    // EXPECT_TRUE(simulator.simulate_for(0.6758 - 0.2));  // this is where the thyristor doesnt turn on...
    // EXPECT_TRUE(simulator.simulate_for(0.1));           // transient
    EXPECT_EQ(simulation_system.operation_mode_monitor.get_outputs().operating_mode.read(),
        yeet::application::sfc::sfc_operating_mode_t::forced_commutation);
    // checker->activate();
    dc_magnitude_checker->activate();
    machine_sum_checker->activate();
    grid_sum_checker->activate();
    EXPECT_TRUE(simulator.simulate_for(1.5));  // steady-state window

    // Step 2: 0.5 pu
    // checker->deactivate();
    // dc_magnitude_checker->deactivate();
    // machine_sum_checker->deactivate();
    // grid_sum_checker->deactivate();
    // current_reference = current_pu_t{0.5f};
    // simulation_system.current_reference = current_reference;
    // EXPECT_TRUE(simulator.simulate_for(0.5));
    // EXPECT_EQ(simulation_system.operation_mode_monitor.get_outputs().operating_mode.read(),
    //     yeet::application::sfc::sfc_operating_mode_t::forced_commutation);
    // // checker->activate();
    // // dc_magnitude_checker->activate();
    // // machine_sum_checker->activate();
    // // grid_sum_checker->activate();
    // EXPECT_TRUE(simulator.simulate_for(0.5));  // steady-state window

    // // Step 3: 1.0 pu

    // // checker->deactivate();
    // dc_magnitude_checker->deactivate();
    // // machine_sum_checker->deactivate();
    // // grid_sum_checker->deactivate();
    // current_reference = current_pu_t{1.0f};
    // simulation_system.current_reference = current_reference;
    // EXPECT_TRUE(simulator.simulate_for(0.5));
    // EXPECT_EQ(simulation_system.operation_mode_monitor.get_outputs().operating_mode.read(),
    //     yeet::application::sfc::sfc_operating_mode_t::forced_commutation);
    // checker->activate();
    // dc_magnitude_checker->activate();
    // machine_sum_checker->activate();
    // grid_sum_checker->activate();
    // EXPECT_TRUE(simulator.simulate_for(0.6));  // steady-state window
}

// ---------- Parameter Set ----------
INSTANTIATE_TEST_SUITE_P(ForcedLowSpeedOperatingPoints,
    CurrentControlForcedCommutationParamTest,
    ::testing::Values(forced_comm_init{0.01f, 0.10f},  // freq_pu, voltage_pu
        forced_comm_init{0.05f, 0.20f},                // freq_pu, voltage_pu
        forced_comm_init{0.1f, 0.20f}),                // freq_pu, voltage_pu
    ForcedCommParamName);
