#include <gtest/gtest.h>

#include "sfc/sfc_startup_sequencer.hpp"
#include "yeet_control/constants.hpp"

using namespace yeet;
using namespace yeet::application::sfc;

class SfcStartupSequencerRateLimiterTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        sequencer = std::make_unique<sfc_startup_sequencer>(sampling_time);

        // Configure virtual frequency for fast test
        virtual_frequency_config virtual_freq_cfg{
            .initial = frequency_pu_t{0.02},  // 0.02 pu = 1 Hz at 50 Hz base
            .final = frequency_pu_t{0.2},     // 0.2 pu = 10 Hz at 50 Hz base
            .duration = duration_t{1.0}       // 1 second ramp
        };
        sequencer->configure_virtual_frequency(virtual_freq_cfg);

        // Configure virtual torque
        virtual_torque_config virtual_torque_cfg{
            .initial = torque_pu_t{0.3}, .final = torque_pu_t{0.3}, .duration = duration_t{0.0}};
        sequencer->configure_virtual_torque(virtual_torque_cfg);
    }

    void run_cycles(int count, bool field_established = true, speed_pu_t external_speed_ref = speed_pu_t{1.0})
    {
        for (int i = 0; i < count; ++i) {
            sequencer->update(field_established, angle_t{0.0}, speed_pu_t{0.0}, voltage_pu_t{0.0},
                sfc_state_id::OPERATION_ON, external_speed_ref, base_frequency);
        }
    }

    const duration_t sampling_time{ms(1.0)};  // 1 ms
    const frequency_t base_frequency{50.0};   // 50 Hz base frequency
    std::unique_ptr<sfc_startup_sequencer> sequencer;
};

TEST_F(SfcStartupSequencerRateLimiterTest, SpeedReferenceIsRateLimited)
{
    // Configure rate limiter: 10 seconds per pu = 0.1 pu/s
    speed_reference_rate_limit_config rate_limit_cfg{.seconds_per_pu = 10.0f, .enabled = true};
    sequencer->configure_speed_reference_rate_limit(rate_limit_cfg);

    // Start sequence and get to closed-loop operation
    sequencer->start_sequence();
    run_cycles(1, true, speed_pu_t{0.0});     // Flux buildup completes
    run_cycles(1001, true, speed_pu_t{0.0});  // Starting procedure completes

    ASSERT_EQ(sequencer->get_current_state(), startup_sequencer_state_t::CLOSED_LOOP_OPERATION);

    // After starting procedure, speed reference should be at final virtual frequency (0.2 pu)
    auto outputs = sequencer->get_outputs();
    real_t baseline = outputs.speed_control.speed_reference.value();
    EXPECT_NEAR(baseline, 0.2f, 0.001f);

    // Apply step input from baseline to 1.0 pu
    // With 10 s/pu rate limit = 0.1 pu/s = 0.0001 pu/ms
    // After 100 ms, should increase by 0.01 pu
    run_cycles(100, true, speed_pu_t{1.0});
    outputs = sequencer->get_outputs();
    EXPECT_NEAR(outputs.speed_control.speed_reference.value(), baseline + 0.01f, 0.002f);

    // After 500 ms total, should have increased by 0.05 pu
    run_cycles(400, true, speed_pu_t{1.0});
    outputs = sequencer->get_outputs();
    EXPECT_NEAR(outputs.speed_control.speed_reference.value(), baseline + 0.05f, 0.002f);

    // After 1000 ms total, should have increased by 0.1 pu
    run_cycles(500, true, speed_pu_t{1.0});
    outputs = sequencer->get_outputs();
    EXPECT_NEAR(outputs.speed_control.speed_reference.value(), baseline + 0.1f, 0.002f);
}

TEST_F(SfcStartupSequencerRateLimiterTest, RateLimiterCanBeDisabled)
{
    // Configure rate limiter as disabled
    speed_reference_rate_limit_config rate_limit_cfg{.seconds_per_pu = 10.0f, .enabled = false};
    sequencer->configure_speed_reference_rate_limit(rate_limit_cfg);

    // Start sequence and get to closed-loop operation
    sequencer->start_sequence();
    run_cycles(1, true, speed_pu_t{0.0});     // Flux buildup completes
    run_cycles(1001, true, speed_pu_t{0.0});  // Starting procedure completes

    ASSERT_EQ(sequencer->get_current_state(), startup_sequencer_state_t::CLOSED_LOOP_OPERATION);

    // When disabled, step input should pass through immediately
    sequencer->update(true, angle_t{0.0}, speed_pu_t{0.0}, voltage_pu_t{0.0}, sfc_state_id::OPERATION_ON,
        speed_pu_t{1.0}, base_frequency);
    auto outputs = sequencer->get_outputs();
    EXPECT_NEAR(outputs.speed_control.speed_reference.value(), 1.0f, 0.001f);
}

TEST_F(SfcStartupSequencerRateLimiterTest, RateLimiterWorksInBothDirections)
{
    // Configure fast rate limiter for quick test: 1 s/pu = 1.0 pu/s = 0.001 pu/ms
    speed_reference_rate_limit_config rate_limit_cfg{.seconds_per_pu = 1.0f, .enabled = true};
    sequencer->configure_speed_reference_rate_limit(rate_limit_cfg);

    // Start sequence and get to closed-loop operation
    sequencer->start_sequence();
    run_cycles(1, true, speed_pu_t{0.0});     // Flux buildup completes
    run_cycles(1001, true, speed_pu_t{0.0});  // Starting procedure completes

    // Get baseline (should be at final virtual frequency 0.2 pu)
    auto outputs = sequencer->get_outputs();
    real_t baseline = outputs.speed_control.speed_reference.value();
    EXPECT_NEAR(baseline, 0.2f, 0.01f);

    // Ramp up from baseline to 0.5 pu: delta = 0.3 pu, time = 300 ms at 1.0 pu/s
    run_cycles(300, true, speed_pu_t{0.5});
    outputs = sequencer->get_outputs();
    EXPECT_NEAR(outputs.speed_control.speed_reference.value(), 0.5f, 0.01f);

    // Now ramp down to 0.2 pu: delta = -0.3 pu, time = 300 ms (should decrease at same rate)
    run_cycles(300, true, speed_pu_t{0.2});
    outputs = sequencer->get_outputs();
    EXPECT_NEAR(outputs.speed_control.speed_reference.value(), 0.2f, 0.01f);
}

TEST_F(SfcStartupSequencerRateLimiterTest, RateLimiterInitializesToZeroAtStartup)
{
    // Configure rate limiter
    speed_reference_rate_limit_config rate_limit_cfg{.seconds_per_pu = 10.0f, .enabled = true};
    sequencer->configure_speed_reference_rate_limit(rate_limit_cfg);

    // Even before starting sequence, output should be zero
    auto outputs = sequencer->get_outputs();
    EXPECT_NEAR(outputs.speed_control.speed_reference.value(), 0.0f, 0.001f);

    // During flux buildup and starting procedure, speed reference tracks virtual frequency
    // After flux buildup (1 cycle) and entering starting procedure, it should be at initial freq (0.02 pu)
    sequencer->start_sequence();
    run_cycles(10, true, speed_pu_t{1.0});  // Complete flux buildup and enter starting procedure
    outputs = sequencer->get_outputs();
    // Speed reference should track virtual frequency, which increases slightly over 10 cycles
    // Virtual frequency starts at 0.02 pu and ramps at (0.2-0.02)/1000ms = 0.00018 pu/ms
    // After 10ms: 0.02 + 10*0.00018 = 0.0218 pu
    EXPECT_NEAR(outputs.speed_control.speed_reference.value(), 0.0218f, 0.001f);
}
