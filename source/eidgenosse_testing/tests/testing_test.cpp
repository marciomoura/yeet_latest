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
    void SetUp() override {
        // Create simulator
        _simulator = std::make_unique<simulator>();
    }

    void TearDown() override {
        gtest_testing_info gtest_info(::testing::UnitTest::GetInstance());
        _simulator->dump_json(&gtest_info);

        _simulator.reset();
    }

    std::unique_ptr<simulator> _simulator;
};

class application_one_mock : public cyclic_application_interface {
public:
    void run() override { value += 1.0f; }

    void register_data_sampler_entries(data_sampler& data_sampler) override {
        data_sampler.register_entry(data_sampler_entry("value_one", [this]() { return value; }));
    }

    float value = 0.0f;
};

class application_two_mock : public cyclic_application_interface {
public:
    void run() override { value += 2.0f; }

    void register_data_sampler_entries(data_sampler& data_sampler) override {
        data_sampler.register_entry(data_sampler_entry("value_two", [this]() { return value; }));
    }

    float value = 0.0f;
};

// // create test
TEST_F(simulator_test, register_cyclic_application_1) {
    // get the gtest test name
    std::cout << ::testing::UnitTest::GetInstance()->current_test_info()->name() << std::endl;
    std::cout << ::testing::UnitTest::GetInstance()->current_test_info()->test_case_name() << std::endl;
    std::cout << ::testing::UnitTest::GetInstance()->current_test_info()->test_suite_name() << std::endl;

    _simulator->register_cyclic_application(1, std::make_shared<application_one_mock>());
    _simulator->register_cyclic_application(2, std::make_shared<application_two_mock>());
    _simulator->simulate_for(1.0f);
    _simulator->simulate_for(1.0f);
}

TEST_F(simulator_test, register_cyclic_application_2) {
    // get the gtest test name
    std::cout << ::testing::UnitTest::GetInstance()->current_test_info()->name() << std::endl;
    std::cout << ::testing::UnitTest::GetInstance()->current_test_info()->test_case_name() << std::endl;
    std::cout << ::testing::UnitTest::GetInstance()->current_test_info()->test_suite_name() << std::endl;

    _simulator->register_cyclic_application(1, std::make_shared<application_one_mock>());
    _simulator->register_cyclic_application(2, std::make_shared<application_two_mock>());
    _simulator->simulate_for(1.0f);
    _simulator->simulate_for(1.0f);
}
