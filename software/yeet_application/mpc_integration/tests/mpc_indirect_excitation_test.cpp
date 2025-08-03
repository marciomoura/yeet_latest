#include <gtest/gtest.h>

#include "mpc_integration/indirect_excitation_2mva_plant_model_integration_runner.hpp"
#include "mpc_integration/mpc_integration_model.hpp"
#include "mpc_integration/testing_closed_loop_mpc.hpp"
#include "yeet_control/constants.hpp"
#include "yeet_testing/parameter_collection.hpp"
#include "yeet_testing/simulator_gtest.hpp"

using namespace yeet::application::mpc;

class mpc_indirect_excitation_test : public ::testing::Test {
public:
    using parameter_types = closed_loop_mpc_with_plant_model::parameter_types;

    void SetUp() override
    {
        system.register_applications(simulator);

        // initialize here, it allocates plant model stuff
        initialize();

        // Set the parameters for the plant model
        control_parameters()->l_s = 0.06;
        control_parameters()->l_ad = 1.3575;
        control_parameters()->l_aq = 0.649;
        control_parameters()->l_fd = 0.4725;
        control_parameters()->r_s = 0.003;
        control_parameters()->r_fd = 0.000537147932935147;
        control_parameters()->w_base = 2.0 * yeet::pi * 50.0f;
        control_parameters()->h = 3.5f;
        control_parameters()->kd = 0.5f;
        control_parameters()->l_g = 0.1f;
        control_parameters()->r_g = 0.1f / 20.0f;

        // MPC specific parameters
        control_parameters()->mpc_feedforward_output = 1.0f;
        control_parameters()->mpc_speed_deviation_penalization = 100.0f;
        control_parameters()->mpc_lambda = 200e-4;
        control_parameters()->mpc_umax = 35.0f;
        control_parameters()->mpc_umin = -15.0f;
        control_parameters()->mpc_terminal_voltage_reference = 1.0f;

        plant_parameters()->pmech = 0.2f;

        simulator.register_sampler_entry(yeet::testing::data_sampler_entry(
            "efd", [&]() { return system.mpc.get_signal_flow_datamodel().get_outputs().efd; }));

        simulator.register_sampler_entry(yeet::testing::data_sampler_entry(
            "vt", [&]() { return system.plant.get_signal_flow_datamodel().get_outputs().vt; }));

        simulator.register_sampler_entry(yeet::testing::data_sampler_entry(
            "wm", [&]() { return system.plant.get_signal_flow_datamodel().get_outputs().wm; }));

        simulator.register_sampler_entry(yeet::testing::data_sampler_entry(
            "load_angle", [&]() { return system.plant.get_signal_flow_datamodel().get_outputs().load_angle_deg; }));

        simulator.register_sampler_entry(yeet::testing::data_sampler_entry(
            "it", [&]() { return system.plant.get_signal_flow_datamodel().get_outputs().it; }));
    }

    void initialize() { system.initialize(); }

    void simulate_for(double time) { simulator.simulate_for(time); }

    // make it constructible/copyable/movable
    parameter_types parameter_collection() { return system.create_parameter_collection(); }

    template <typename T>
    yeet::testing::parameter_set<T> parameters()
    {
        return parameter_collection().get<T>();
    }

    yeet::testing::parameter_set<mpc_integration_datamodel::parameters> control_parameters()
    {
        return parameters<mpc_integration_datamodel::parameters>();
    }

    yeet::testing::parameter_set<synchronous_machine_infinite_bus_integration_datamodel::parameters> plant_parameters()
    {
        return parameters<synchronous_machine_infinite_bus_integration_datamodel::parameters>();
    }

    closed_loop_mpc_with_indirect_excitation_plant_model system;

    yeet::testing::simulator simulator = yeet::testing::make_gtest_simulator();
};

TEST_F(mpc_indirect_excitation_test, mechanical_power_step)
{
    simulate_for(5.0f);

    plant_parameters()->pmech = 0.3f;
    simulate_for(10.0f);

    plant_parameters()->pmech = 0.5f;
    simulate_for(10.0f);

    // Terminal voltage
    simulator.register_checker(yeet::testing::make_range_checker<double>(
        "terminal voltage", [this]() { return system.plant.get_signal_flow_datamodel().get_outputs().vt; },
        1.0f - 0.025f,  // Minimum terminal voltage
        1.0f + 0.025f,  // Maximum terminal voltage
        yeet::testing::checker_activation_time_range{simulator.get_current_simulation_time()}));

    simulate_for(20.0f);
}

TEST_F(mpc_indirect_excitation_test, terminal_voltage_step)
{
    simulate_for(5.0f);

    control_parameters()->mpc_terminal_voltage_reference = 1.02f;
    simulate_for(30.0f);

    // Terminal voltage
    simulator.register_checker(yeet::testing::make_range_checker<double>(
        "terminal voltage", [this]() { return system.plant.get_signal_flow_datamodel().get_outputs().vt; },
        1.02f - 0.0025f,  // Minimum terminal voltage
        1.02f + 0.0025f,  // Maximum terminal voltage
        yeet::testing::checker_activation_time_range{simulator.get_current_simulation_time()}));

    simulate_for(10.0f);
}
