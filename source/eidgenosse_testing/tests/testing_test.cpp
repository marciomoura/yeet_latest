#include "eidgenosse_testing/testing.hpp"

#include <gtest/gtest.h>

class gtest_testing_info : public testing_info_interface {
public:
    gtest_testing_info(const ::testing::UnitTest* unit_test) : _unit_test(unit_test) {}

    std::string get_test_suite_name() const override { return _unit_test->current_test_info()->test_suite_name(); }
    std::string get_test_case_name() const override { return _unit_test->current_test_info()->name(); }

private:
    const ::testing::UnitTest* _unit_test{};
};

// Create test fixture
class simulator_test : public ::testing::Test {
public:
    void SetUp() override { _simulator = std::make_unique<simulator>(); }

    void TearDown() override {
        gtest_testing_info gtest_info(::testing::UnitTest::GetInstance());
        _simulator->dump_json(&gtest_info);

        _simulator.reset();
    }

    std::unique_ptr<simulator> _simulator;
};

class application_one_mock : public cyclic_application_interface {
public:
    double get_sampling_rate() override { return 0.01f; }

    void run() override { value += 1.0f; }

    void register_data_sampler_entries(data_sampler& data_sampler) override {
        data_sampler.register_entry(data_sampler_entry("value_one", [this]() { return value; }));
    }

    float value = 0.0f;
};

class application_two_mock : public cyclic_application_interface {
public:
    double get_sampling_rate() override { return 0.02f; }

    void run() override { value += 2.0f; }

    void register_data_sampler_entries(data_sampler& data_sampler) override {
        data_sampler.register_entry(data_sampler_entry("value_two", [this]() { return value; }));
    }

    float value = 0.0f;
};

class application_sinusoidal_mock : public cyclic_application_interface {
public:
    application_sinusoidal_mock() {
        _sin = std::sin(_angle);
        _cos = std::cos(_angle);
    }

    double get_sampling_rate() override { return 100e-6; }

    void run() override {
        _sin = std::sin(_angle);
        _cos = std::cos(_angle);
        _angle += _angle_increment;
    }

    void register_data_sampler_entries(data_sampler& data_sampler) override {
        data_sampler.register_entry(data_sampler_entry("sin", [this]() { return _sin; }));
        data_sampler.register_entry(data_sampler_entry("cos", [this]() { return _cos; }));
    }

    static constexpr float pi = 3.14159265358979323846;
    static constexpr float ts = 25e-6;
    float _angle_increment = 10.0f * 2.0f * pi * ts;
    float _angle = 0.0f;
    float _sin = 0.0f;
    float _cos = 0.0f;
};

// // create test
TEST_F(simulator_test, register_cyclic_application) {
    _simulator->register_cyclic_application(std::make_shared<application_one_mock>());
    _simulator->register_cyclic_application(std::make_shared<application_two_mock>());
    _simulator->simulate_for(0.25f);
    _simulator->simulate_for(0.25f);
}

TEST_F(simulator_test, sin_cos_simulation) {
    _simulator->register_cyclic_application(std::make_shared<application_sinusoidal_mock>());
    _simulator->simulate_for(0.2f);
}
