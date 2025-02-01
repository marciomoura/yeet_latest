#include <gtest/gtest.h>

#include <cmath>

#include "yeet/testing/simulator.hpp"

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
protected:
    using parameter_types = yeet::closed_loop_example::parameter_types;

    void SetUp() override { _simulation = yeet::simulation_builder<parameter_types>::create(_simulator, _example); }

    void TearDown() override {
        gtest_testing_info gtest_info(::testing::UnitTest::GetInstance());
        _simulation->dump_results(&gtest_info);
    }

    void initialize() { _simulation->initialize(); }

    void simulate_for(double time) { _simulation->run_for(time); }

    template <typename T>
    yeet::parameter_set<T>& parameters() {
        return _simulation->get_parameters<T>();
    }

private:
    yeet::closed_loop_example _example;
    yeet::simulator _simulator;
    std::unique_ptr<yeet::simulation_runner<parameter_types>> _simulation{};
};

TEST_F(simulator_test, closed_loop_simulation_a) {
    initialize();

    parameters<yeet::control_parameters>()->reference = 0.5f;
    parameters<yeet::control_parameters>()->ki = 0.5f;
    simulate_for(1.0f);

    parameters<yeet::control_parameters>()->reference = 1.0f;
    simulate_for(1.0f);
}

TEST_F(simulator_test, closed_loop_simulation_b) {
    initialize();

    parameters<yeet::control_parameters>()->reference = 0.5f;
    simulate_for(1.0f);

    parameters<yeet::plant_parameters>()->ti = 0.5f;
    parameters<yeet::control_parameters>()->kp = 40.0f;
    parameters<yeet::control_parameters>()->reference = 1.0f;
    simulate_for(1.0f);
}

TEST_F(simulator_test, closed_loop_simulation_c) {
    initialize();

    parameters<yeet::control_parameters>()->reference = 0.5f;
    simulate_for(2.0f);

    parameters<yeet::plant_parameters>()->ti = 1.5f;
    parameters<yeet::control_parameters>()->kp = 20.0f;
    parameters<yeet::control_parameters>()->ki = 200.0f;
    parameters<yeet::control_parameters>()->reference = 1.0f;
    simulate_for(2.0f);
}

// TEST swapping plant model but keeping the same "datamodel" interface
class simulator_test_double_filter_plant : public ::testing::Test {
protected:
    using parameter_types = yeet::closed_loop_example::parameter_types;

    void SetUp() override { _simulation = yeet::simulation_builder<parameter_types>::create(_simulator, _example); }

    void TearDown() override {
        gtest_testing_info gtest_info(::testing::UnitTest::GetInstance());
        _simulation->dump_results(&gtest_info);
    }

    void initialize() { _simulation->initialize(); }

    void simulate_for(double time) { _simulation->run_for(time); }

    template <typename T>
    yeet::parameter_set<T>& parameters() {
        return _simulation->get_parameters<T>();
    }

private:
    yeet::closed_loop_example _example{std::make_shared<yeet::plant_model_double_filter_application_runner>()};
    yeet::simulator _simulator;
    std::unique_ptr<yeet::simulation_runner<parameter_types>> _simulation{};
};

TEST_F(simulator_test_double_filter_plant, closed_loop_simulation_c) {
    initialize();

    parameters<yeet::plant_parameters>()->ti = 0.1f;
    parameters<yeet::control_parameters>()->kp = 10.0f;
    parameters<yeet::control_parameters>()->ki = 50.0f;
    parameters<yeet::control_parameters>()->reference = 1.0f;
    simulate_for(2.0f);

    parameters<yeet::control_parameters>()->reference = 0.5f;
    simulate_for(2.0f);
}
