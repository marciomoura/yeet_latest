#pragma once

#include "sfc_6p_6p_plant_model.hpp"
#include "yeet_testing/simulator.hpp"

namespace yeet::application::sfc {

class sfc_plant_model_simulator {
public:
    sfc_plant_model_simulator() {}

    // set inputs/outputs via interface
    sfc_6p_6p_plant_model& plant() { return _plant; }

    void setup_simulator(yeet::testing::simulator& simulator) {
        // Register the plant model with the simulator
        simulator.register_lambda_runner(sfc_6p_6p_plant_model::kSamplingTime, [&]() { _plant.step(); });
    }

    void setup_all_logging(yeet::testing::simulator& simulator) {
        setup_grid_side_logging(simulator);
        setup_machine_side_logging(simulator);
        setup_dc_link_logging(simulator);
    }

    void setup_grid_side_logging(yeet::testing::simulator& simulator) {
        // Log grid-side voltages (abc)
        simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "grid_voltage_a", [&]() -> double { return _plant.get_grid_side_voltages_abc().a(); }));
        simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "grid_voltage_b", [&]() -> double { return _plant.get_grid_side_voltages_abc().b(); }));
        simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "grid_voltage_c", [&]() -> double { return _plant.get_grid_side_voltages_abc().c(); }));

        // Log grid-side currents (abc)
        simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "grid_current_a", [&]() -> double { return _plant.get_grid_side_currents_abc().a(); }));
        simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "grid_current_b", [&]() -> double { return _plant.get_grid_side_currents_abc().b(); }));
        simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "grid_current_c", [&]() -> double { return _plant.get_grid_side_currents_abc().c(); }));

        // Log grid-side angle
        simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "grid_angle_rad", [&]() -> double { return _plant.get_grid_voltage_source_angle().get_radians(); }));
    }

    void setup_machine_side_logging(yeet::testing::simulator& simulator) {
        // Log machine-side voltages (abc)
        simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "machine_voltage_a", [&]() -> double { return _plant.get_machine_side_voltages_abc().a(); }));
        simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "machine_voltage_b", [&]() -> double { return _plant.get_machine_side_voltages_abc().b(); }));
        simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "machine_voltage_c", [&]() -> double { return _plant.get_machine_side_voltages_abc().c(); }));

        // Log machine-side currents (abc)
        simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "machine_current_a", [&]() -> double { return _plant.get_machine_side_currents_abc().a(); }));
        simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "machine_current_b", [&]() -> double { return _plant.get_machine_side_currents_abc().b(); }));
        simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "machine_current_c", [&]() -> double { return _plant.get_machine_side_currents_abc().c(); }));

        // Log machine-side angle
        simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "machine_angle_rad", [&]() -> double { return _plant.get_machine_voltage_source_angle().get_radians(); }));
    }

    void setup_dc_link_logging(yeet::testing::simulator& simulator) {
        // Log DC link current
        simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "dc_link_current", [&]() -> double { return _plant.get_dc_link_current(); }));
    }

private:
    sfc_6p_6p_plant_model _plant;
};

}  // namespace yeet::application::sfc
