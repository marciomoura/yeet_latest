#include "eidgenosse_testing/testing.hpp"

#include <gtest/gtest.h>

// Create test fixture
class simulator_test : public ::testing::Test {
public:
    void SetUp() override {
        // Create simulator
        simulator_ = std::make_unique<simulator>();
    }

    void TearDown() override {
        // Destroy simulator
        simulator_.reset();
    }

    std::unique_ptr<simulator> simulator_;
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
TEST_F(simulator_test, register_cyclic_application) {
    simulator_->register_cyclic_application(1, std::make_shared<application_one_mock>());
    simulator_->register_cyclic_application(2, std::make_shared<application_two_mock>());
    simulator_->simulate_for(1.0f);
    simulator_->simulate_for(1.0f);
    simulator_->dump_json();
}
