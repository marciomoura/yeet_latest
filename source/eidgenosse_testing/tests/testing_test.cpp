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

class cyclic_application_mock : public cyclic_application_interface {
public:
    void run() override { value += 1.0f; }

    void register_data_sampler_entries(data_sampler& data_sampler) override {
        data_sampler.register_entry(data_sampler_entry("value", [this]() { return value; }));
    }

    float value = 100.0f;
};

// // create test
TEST_F(simulator_test, register_cyclic_application) {
    simulator_->register_cyclic_application(10, std::make_shared<cyclic_application_mock>());

    simulator_->simulate_for();

    simulator_->dump();
}

//     // Create cyclic application
//     class cyclic_application_mock : public cyclic_application {
//     public:
//         void run() override {}
//     };

//     // Register cyclic application
//     std::shared_ptr<cyclic_application_mock> cyclic_application = std::make_shared<cyclic_application_mock>();
//     simulator_->register_cyclic_application(1, cyclic_application);

//     // Check if cyclic application is registered
//     // EXPECT_EQ(simulator_->cyclic_applications_.size(), 1);
// }