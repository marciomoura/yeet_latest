#include <gtest/gtest.h>

#include <cmath>

#include "eidgenosse/parameter.hpp"
#include "eidgenosse/signal.hpp"
#include "eidgenosse/testing/testing.hpp"

//
#include "../examples/control_integration.hpp"

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

        // this is something to take care of in a better way
        register_data_sampler();
    }

    void TearDown() override {
        gtest_testing_info gtest_info(::testing::UnitTest::GetInstance());

        _simulator->dump_json(&gtest_info);

        _simulator.reset();
    }

    void simulate_for(double time) { _simulator->simulate_for(time); }

    void register_data_sampler() {
        auto control_model_application = _example.get_control_application_datamodel();
        auto plant_model_application = _example.get_plant_model_application_datamodel();

        _simulator->register_data_sampler_entry(data_sampler_entry{
            "control output", [=]() { return control_model_application.model_outputs->control_output.read(); }});

        _simulator->register_data_sampler_entry(data_sampler_entry{
            "plant output", [=]() { return plant_model_application.model_outputs->plant_output.read(); }});

        _simulator->register_data_sampler_entry(data_sampler_entry{
            "control reference", [=]() { return control_model_application.model_parameters->reference.read(); }});
    }

    void set_reference(float reference) {
        auto control_model_application = _example.get_control_application_datamodel();

        control_model_application.model_parameters->reference.write(reference);
    }

    closed_loop_example _example;

    std::unique_ptr<simulator> _simulator;
};

TEST_F(simulator_test, closed_loop_simulation) {
    set_reference(0.5f);
    simulate_for(1.0f);

    set_reference(1.0f);
    simulate_for(1.0f);
}
