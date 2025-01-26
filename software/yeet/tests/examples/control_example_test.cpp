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
public:
    void SetUp() override {
        closed_loop_example_simulator::configure(_simulator, _example);

        parameters = closed_loop_example_simulator::get_parameters(_example);
    }

    void TearDown() override {
        gtest_testing_info gtest_info(::testing::UnitTest::GetInstance());

        _simulator.dump_json(&gtest_info);
    }

    void initialize() { _simulator.initialize(); }

    void simulate_for(double time) { _simulator.simulate_for(time); }

    closed_loop_example_simulator::parameters parameters;

private:
    closed_loop_example _example;

    simulator _simulator;
};

TEST_F(simulator_test, closed_loop_simulation_a) {
    initialize();

    parameters.control->reference = 0.5f;
    simulate_for(1.0f);

    parameters.control->reference = 1.0f;
    simulate_for(1.0f);
}

TEST_F(simulator_test, closed_loop_simulation_b) {
    initialize();

    parameters.control->reference = 0.5f;
    simulate_for(1.0f);

    parameters.plant->ti = 0.5f;
    parameters.control->kp = 40.0f;
    parameters.control->reference = 1.0f;
    simulate_for(1.0f);
}

TEST_F(simulator_test, closed_loop_simulation_c) {
    initialize();

    parameters.control->reference = 0.5f;
    simulate_for(2.0f);

    parameters.plant->ti = 1.5f;
    parameters.control->kp = 20.0f;
    parameters.control->ki = 200.0f;
    parameters.control->reference = 1.0f;
    simulate_for(2.0f);
}
