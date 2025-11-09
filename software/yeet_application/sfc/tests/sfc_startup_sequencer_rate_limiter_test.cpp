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
            .initial = frequency_t{1.0},  // 1 Hz start
            .final = frequency_t{10.0},   // 10 Hz end
            .duration = duration_t{1.0}   // 1 second ramp
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
            sequencer->update(field_established, angle_t{0.0}, external_speed_ref);
        }
    }

    const duration_t sampling_time{ms(1.0)};  // 1 ms
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

    // Initial speed reference should be 0
    auto outputs = sequencer->get_outputs();
    EXPECT_NEAR(outputs.speed_control.speed_reference.value(), 0.0f, 0.001f);

    // Apply step input from 0 to 1.0 pu
    // With 10 s/pu rate limit = 0.1 pu/s = 0.0001 pu/ms
    // After 100 ms, should be at 0.01 pu
    run_cycles(100, true, speed_pu_t{1.0});
    outputs = sequencer->get_outputs();
    EXPECT_NEAR(outputs.speed_control.speed_reference.value(), 0.01f, 0.002f);

    // After 500 ms total, should be at 0.05 pu
    run_cycles(400, true, speed_pu_t{1.0});
    outputs = sequencer->get_outputs();
    EXPECT_NEAR(outputs.speed_control.speed_reference.value(), 0.05f, 0.002f);

    // After 1000 ms total, should be at 0.1 pu
    run_cycles(500, true, speed_pu_t{1.0});
    outputs = sequencer->get_outputs();
    EXPECT_NEAR(outputs.speed_control.speed_reference.value(), 0.1f, 0.002f);
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
    sequencer->update(true, angle_t{0.0}, speed_pu_t{1.0});
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

    // Ramp up to 0.5 pu
    run_cycles(500, true, speed_pu_t{1.0});
    auto outputs = sequencer->get_outputs();
    EXPECT_NEAR(outputs.speed_control.speed_reference.value(), 0.5f, 0.01f);

    // Now ramp down to 0.2 pu (should decrease at same rate)
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

    // During flux buildup and starting procedure, speed reference limiter is active
    // With 10 s/pu = 0.1 pu/s = 0.0001 pu/ms
    // After 10 cycles at 1ms each with 1.0 pu input = 0.001 pu
    sequencer->start_sequence();
    run_cycles(10, true, speed_pu_t{1.0});  // Still in flux buildup
    outputs = sequencer->get_outputs();
    EXPECT_NEAR(outputs.speed_control.speed_reference.value(), 0.001f, 0.0002f);  // Limiter is active from cycle 1
}
