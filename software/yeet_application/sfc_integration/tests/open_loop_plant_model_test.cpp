#include <gtest/gtest.h>

#include "sfc_integration/plant_simplified_open_loop.hpp"
#include "yeet_control/six_pulse_generator.hpp"
#include "yeet_testing/data_sampler_entry.hpp"
#include "yeet_testing/simulator_gtest.hpp"

using namespace yeet::application::sfc::integration;
using namespace yeet;

class SfcBuilderTest : public ::testing::Test {
public:
    // using parameter_types ::parameter_types;

    void SetUp() override
    {
        // simulator.configure_sampler_sampling_rate(200e-6);
        system.register_applications(simulator);

        // initialize here, it allocates plant model stuff
        initialize();

        simulator.register_sampler_entry(
            {"dc_link_current", [&]() {
                 return static_cast<float>(
                     system.plant.get_signal_flow_datamodel().get_outputs().dc_link_current.read());
             }});

        simulator.register_sampler_entry(yeet::make_lambda_array_sampler<yeet::control::six_pulse_generator::pulses>(
            "grid_firing_pulses", [&]() -> yeet::control::six_pulse_generator::pulses {
                return system.grid_side_pulse_generator.get_signal_flow_datamodel().get_outputs().firing_pulses.read();
            }));

        simulator.register_sampler_entry(yeet::make_lambda_array_sampler<yeet::control::six_pulse_generator::pulses>(
            "machine_firing_pulses", [&]() -> yeet::control::six_pulse_generator::pulses {
                return system.grid_side_pulse_generator.get_signal_flow_datamodel().get_outputs().firing_pulses.read();
            }));

        simulator.register_sampler_entry(yeet::make_lambda_array_sampler<abc<voltage_t>>("grid_voltage_pu",
            [&]() { return system.plant.get_signal_flow_datamodel().get_outputs().grid_voltage.read(); }));

        simulator.register_sampler_entry(yeet::make_lambda_array_sampler<abc<voltage_t>>("machine_voltage_pu",
            [&]() { return system.plant.get_signal_flow_datamodel().get_outputs().machine_voltage.read(); }));
    }

    void initialize() { system.initialize(); }

    void simulate_for(double time) { simulator.simulate_for(time); }

    // make it constructible/copyable/movable
    auto parameter_collection() { return system.create_parameter_collection(); }

    template <typename T>
    yeet::parameter_set<T> parameters()
    {
        return parameter_collection().get<T>();
    }

    yeet::parameter_set<sfc_machine_side_pulse_generator_datamodel::parameters> machine_side_pulse_generator()
    {
        return parameters<sfc_machine_side_pulse_generator_datamodel::parameters>();
    }

    yeet::parameter_set<sfc_grid_side_pulse_generator_datamodel::parameters> grid_side_pulse_generator()
    {
        return parameters<sfc_grid_side_pulse_generator_datamodel::parameters>();
    }

    yeet::application::sfc::integration::sfc_simulator_builder system;

    yeet::simulator simulator = yeet::make_gtest_simulator();
};

TEST_F(SfcBuilderTest, InitialTest)
{
    system.grid_side_firing_angle = yeet::angle_wrapped::from_degrees(13.0);
    system.machine_side_firing_angle = yeet::angle_wrapped::from_degrees(180.0 - 26.84);

    machine_side_pulse_generator()->enable = true;
    grid_side_pulse_generator()->enable = true;

    simulate_for(0.25f);

    system.grid_side_firing_angle = yeet::angle_wrapped::from_degrees(25.0);

    simulate_for(0.25f);
}
