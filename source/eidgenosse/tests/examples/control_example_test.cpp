#include <gtest/gtest.h>

#include <cmath>

#include "eidgenosse/parameter.hpp"
#include "eidgenosse/signal.hpp"
#include "eidgenosse/utils/low_level_access.hpp"

//
#include "../examples/control_integration.hpp"
#include "eidgenosse/testing/control_task_simulator_runner.hpp"
#include "eidgenosse/testing/simulator.hpp"


class gtest_testing_info : public testing_info_interface {
public:
    gtest_testing_info(const ::testing::UnitTest* unit_test) : _unit_test(unit_test) {}

    std::string get_test_suite_name() const override { return _unit_test->current_test_info()->test_suite_name(); }
    std::string get_test_case_name() const override { return _unit_test->current_test_info()->name(); }

private:
    const ::testing::UnitTest* _unit_test{};
};

class simulator_test : public ::testing::Test {
public:
    void SetUp() override {
        _simulator = std::make_unique<simulator>();

        for (std::size_t i = 0; i < _example.get_number_task_levels(); ++i) {
            auto control_task_simulator =
                std::make_shared<control_task_simulator_runner>(_example.get_control_task_level(i));

            _simulator->register_cyclic_application(control_task_simulator);
        }

        // this can be improved
        _simulator->register_applications();

        // this is something to take care of in a better way
        register_data_sampler();
    }

    void TearDown() override {
        gtest_testing_info gtest_info(::testing::UnitTest::GetInstance());

        _simulator->dump_json(&gtest_info);

        _simulator.reset();
    }

    void initialize() { _simulator->initialize(); }
    void simulate_for(double time) { _simulator->simulate_for(time); }

    void register_data_sampler() {
        auto control_model_application = _example.get_control_application_datamodel();
        auto plant_model_application = _example.get_plant_model_application_datamodel();

        // register data model
        control_model_application.register_data_model(_model);
        plant_model_application.register_data_model(_model);

        _model.iterate_map_entries(
            [this](const std::string& group, const std::string& name, const low_level_access& access) {
                low_level_access::sampler_accessor const accessor = access.get_sampler_accessor();
                if (accessor.read) {
                    _simulator->register_data_sampler_entry(
                        data_sampler_entry{group + " " + name, [accessor]() { return accessor.read(); }});
                }
            });
    }

    void set_reference(float reference) {
        auto control_model_application = _example.get_control_application_datamodel();

        control_model_application.model_parameters->reference.write(reference);
    }

    auto plant_parameters() {
        auto plant_model_application = _example.get_plant_model_application_datamodel();

        return plant_model_application.model_parameters;
    }

    auto control_parameters() {
        auto control_model_application = _example.get_control_application_datamodel();

        return control_model_application.model_parameters;
    }

    model_registry _model;

    closed_loop_example _example;

    std::unique_ptr<simulator> _simulator;
};

TEST_F(simulator_test, closed_loop_simulation_a) {
    initialize();

    set_reference(0.5f);
    simulate_for(1.0f);

    set_reference(1.0f);
    simulate_for(1.0f);
}

TEST_F(simulator_test, closed_loop_simulation_b) {
    auto plant = plant_parameters();
    auto control = control_parameters();

    initialize();

    set_reference(0.5f);
    simulate_for(1.0f);

    plant->ti = 0.5f;
    control->kp = 40.0f;

    set_reference(1.0f);
    simulate_for(1.0f);
}

TEST_F(simulator_test, closed_loop_simulation_c) {
    auto plant = plant_parameters();
    auto control = control_parameters();

    initialize();

    set_reference(0.5f);
    simulate_for(2.0f);

    plant->ti = 1.5f;
    control->kp = 20.0f;
    control->ki = 200.0f;

    set_reference(1.0f);
    simulate_for(2.0f);
}
