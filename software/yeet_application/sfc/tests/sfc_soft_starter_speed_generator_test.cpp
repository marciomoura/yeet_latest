#include "sfc/sfc_soft_starter_speed_generator.hpp"

#include <gtest/gtest.h>

using namespace yeet::application::sfc;
using namespace yeet;

class SfcSoftStarterSpeedGeneratorTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        sampling_time = duration_t{0.001};  // 1ms
        speed_gen = std::make_unique<sfc_soft_starter_speed_generator>(sampling_time);
    }

    duration_t sampling_time;
    std::unique_ptr<sfc_soft_starter_speed_generator> speed_gen;
};

// Test construction and initial state
TEST_F(SfcSoftStarterSpeedGeneratorTest, ConstructionInitializesCorrectly)
{
    EXPECT_EQ(speed_gen->get_active_command(), speed_command_t::IDLE);
    EXPECT_FLOAT_EQ(speed_gen->get_speed_reference_pu().value(), 0.0);
    EXPECT_FLOAT_EQ(speed_gen->get_torque_limit_max_pu().value(), 0.0);
    EXPECT_FLOAT_EQ(speed_gen->get_torque_limit_min_pu().value(), 0.0);
    EXPECT_FALSE(speed_gen->is_profile_complete());
    EXPECT_FALSE(speed_gen->is_breakaway_boost_active());
}

// Test factory method for creep speed profile
TEST_F(SfcSoftStarterSpeedGeneratorTest, FactoryMethodCreepSpeed)
{
    auto config = sfc_soft_starter_speed_generator::make_creep_speed_profile();
    EXPECT_TRUE(config.enabled);
    EXPECT_FLOAT_EQ(config.target_speed.value(), 0.05);
    EXPECT_GT(config.torque_limit_max.value(), 0.0);
}

// Test factory method for purge acceleration
TEST_F(SfcSoftStarterSpeedGeneratorTest, FactoryMethodPurgeAccel)
{
    auto config = sfc_soft_starter_speed_generator::make_purge_accel_profile();
    EXPECT_TRUE(config.enabled);
    EXPECT_FLOAT_EQ(config.target_speed.value(), 0.20);
    EXPECT_GT(config.torque_limit_max.value(), 0.0);
}

// Test profile configuration and selection
TEST_F(SfcSoftStarterSpeedGeneratorTest, ConfigureAndSelectProfile)
{
    auto purge_config = sfc_soft_starter_speed_generator::make_purge_accel_profile();
    speed_gen->configure_profile(acceleration_profile_t::PROFILE_2, purge_config);

    EXPECT_TRUE(speed_gen->is_profile_enabled(acceleration_profile_t::PROFILE_2));
    EXPECT_TRUE(speed_gen->select_profile(acceleration_profile_t::PROFILE_2));
    EXPECT_EQ(speed_gen->get_selected_profile(), acceleration_profile_t::PROFILE_2);
}

// Test cannot select disabled profile
TEST_F(SfcSoftStarterSpeedGeneratorTest, CannotSelectDisabledProfile)
{
    // PROFILE_3 is not configured (disabled by default)
    EXPECT_FALSE(speed_gen->is_profile_enabled(acceleration_profile_t::PROFILE_3));
    EXPECT_FALSE(speed_gen->select_profile(acceleration_profile_t::PROFILE_3));
}

// Test profile disable functionality
TEST_F(SfcSoftStarterSpeedGeneratorTest, DisableProfileWorks)
{
    auto config = sfc_soft_starter_speed_generator::make_purge_accel_profile();
    speed_gen->configure_profile(acceleration_profile_t::PROFILE_2, config);
    EXPECT_TRUE(speed_gen->is_profile_enabled(acceleration_profile_t::PROFILE_2));

    speed_gen->disable_profile(acceleration_profile_t::PROFILE_2);
    EXPECT_FALSE(speed_gen->is_profile_enabled(acceleration_profile_t::PROFILE_2));
    EXPECT_FALSE(speed_gen->select_profile(acceleration_profile_t::PROFILE_2));
}

// Test IDLE command
TEST_F(SfcSoftStarterSpeedGeneratorTest, IdleCommandWorks)
{
    EXPECT_TRUE(speed_gen->execute_command(speed_command_t::IDLE));
    EXPECT_EQ(speed_gen->get_active_command(), speed_command_t::IDLE);

    speed_gen->update(speed_pu_t{0.0});
    EXPECT_FLOAT_EQ(speed_gen->get_speed_reference_pu().value(), 0.0);
    EXPECT_FLOAT_EQ(speed_gen->get_torque_limit_max_pu().value(), 0.0);
}

// Test HOLD_SPEED command
TEST_F(SfcSoftStarterSpeedGeneratorTest, HoldSpeedCommandWorks)
{
    // Set up initial speed
    speed_gen->reset(speed_pu_t{0.50});

    EXPECT_TRUE(speed_gen->execute_command(speed_command_t::HOLD_SPEED));
    EXPECT_EQ(speed_gen->get_active_command(), speed_command_t::HOLD_SPEED);

    speed_gen->update(speed_pu_t{0.50});
    EXPECT_FLOAT_EQ(speed_gen->get_speed_reference_pu().value(), 0.50);
    EXPECT_GT(speed_gen->get_torque_limit_max_pu().value(), 0.0);
}

// Test DECELERATE command
TEST_F(SfcSoftStarterSpeedGeneratorTest, DecelerateCommandWorks)
{
    // Start at some speed
    speed_gen->reset(speed_pu_t{1.0});
    speed_gen->configure_deceleration(duration_t{10.0});

    EXPECT_TRUE(speed_gen->execute_command(speed_command_t::DECELERATE));
    EXPECT_EQ(speed_gen->get_active_command(), speed_command_t::DECELERATE);

    // Run for 5 seconds
    for (int i = 0; i < 5000; ++i) {
        speed_gen->update(speed_pu_t{0.0});
    }

    // Should be around 50% down
    EXPECT_LT(speed_gen->get_speed_reference_pu().value(), 0.60);
    EXPECT_GT(speed_gen->get_speed_reference_pu().value(), 0.40);
}

// Test RUN_PROFILE command
TEST_F(SfcSoftStarterSpeedGeneratorTest, RunProfileCommandWorks)
{
    auto config = sfc_soft_starter_speed_generator::make_purge_accel_profile(speed_pu_t{0.50}, duration_t{10.0});
    speed_gen->configure_profile(acceleration_profile_t::PROFILE_2, config);
    speed_gen->select_profile(acceleration_profile_t::PROFILE_2);

    EXPECT_TRUE(speed_gen->execute_command(speed_command_t::RUN_PROFILE));
    EXPECT_EQ(speed_gen->get_active_command(), speed_command_t::RUN_PROFILE);

    // Run for 5 seconds (half the acceleration time)
    for (int i = 0; i < 5000; ++i) {
        speed_gen->update(speed_pu_t{0.0});
    }

    // Rate is 1.0/10.0 = 0.1 per second, so after 5 seconds we travel 0.5 pu
    // Target is 0.50 pu, so we should reach it exactly
    EXPECT_NEAR(speed_gen->get_speed_reference_pu().value(), 0.50, 0.01);
}

// Test cannot run profile if not selected
TEST_F(SfcSoftStarterSpeedGeneratorTest, CannotRunProfileIfNotEnabled)
{
    // Try to run without configuring profile
    speed_gen->select_profile(acceleration_profile_t::PROFILE_1);  // This will fail
    EXPECT_FALSE(speed_gen->execute_command(speed_command_t::RUN_PROFILE));
}

// Test breakaway boost activation
TEST_F(SfcSoftStarterSpeedGeneratorTest, BreakawayBoostActivates)
{
    auto config = sfc_soft_starter_speed_generator::make_purge_accel_profile();
    config.initial_torque_boost = torque_pu_t{0.30};
    config.boost_duration = duration_t{1.0};

    speed_gen->configure_profile(acceleration_profile_t::PROFILE_2, config);
    speed_gen->select_profile(acceleration_profile_t::PROFILE_2);
    speed_gen->execute_command(speed_command_t::RUN_PROFILE);

    speed_gen->update(speed_pu_t{0.0});

    EXPECT_TRUE(speed_gen->is_breakaway_boost_active());
    // Torque limit should include boost
    const real_t expected_torque = config.torque_limit_max.value() + config.initial_torque_boost.value();
    EXPECT_FLOAT_EQ(speed_gen->get_torque_limit_max_pu().value(), expected_torque);
}

// Test breakaway boost expiration
TEST_F(SfcSoftStarterSpeedGeneratorTest, BreakawayBoostExpires)
{
    auto config = sfc_soft_starter_speed_generator::make_purge_accel_profile();
    config.initial_torque_boost = torque_pu_t{0.30};
    config.boost_duration = duration_t{1.0};

    speed_gen->configure_profile(acceleration_profile_t::PROFILE_2, config);
    speed_gen->select_profile(acceleration_profile_t::PROFILE_2);
    speed_gen->execute_command(speed_command_t::RUN_PROFILE);

    // Run for 1.5 seconds (beyond boost duration)
    for (int i = 0; i < 1500; ++i) {
        speed_gen->update(speed_pu_t{0.0});
    }

    EXPECT_FALSE(speed_gen->is_breakaway_boost_active());
    // Torque limit should be back to normal
    EXPECT_FLOAT_EQ(speed_gen->get_torque_limit_max_pu().value(), config.torque_limit_max.value());
}

// Test profile completion detection
TEST_F(SfcSoftStarterSpeedGeneratorTest, ProfileCompletionDetected)
{
    auto config = sfc_soft_starter_speed_generator::make_purge_accel_profile(speed_pu_t{0.20}, duration_t{1.0});
    speed_gen->configure_profile(acceleration_profile_t::PROFILE_2, config);
    speed_gen->select_profile(acceleration_profile_t::PROFILE_2);
    speed_gen->execute_command(speed_command_t::RUN_PROFILE);

    EXPECT_FALSE(speed_gen->is_profile_complete());

    // Run for 1.5 seconds (beyond acceleration time)
    for (int i = 0; i < 1500; ++i) {
        speed_gen->update(speed_pu_t{0.0});
    }

    EXPECT_TRUE(speed_gen->is_profile_complete());
    EXPECT_NEAR(speed_gen->get_speed_reference_pu().value(), 0.20, 0.01);
}

// Test absolute limits enforcement
TEST_F(SfcSoftStarterSpeedGeneratorTest, AbsoluteLimitsEnforced)
{
    speed_gen->configure_absolute_limits(speed_pu_t{0.0}, speed_pu_t{0.80}, torque_pu_t{1.5}, torque_pu_t{-1.5});

    auto config = sfc_soft_starter_speed_generator::make_assist_accel_profile(
        speed_pu_t{1.0}, duration_t{1.0});       // Target above limit
    config.torque_limit_max = torque_pu_t{2.0};  // Torque above limit

    speed_gen->configure_profile(acceleration_profile_t::PROFILE_4, config);
    speed_gen->select_profile(acceleration_profile_t::PROFILE_4);
    speed_gen->execute_command(speed_command_t::RUN_PROFILE);

    // Run to completion
    for (int i = 0; i < 1500; ++i) {
        speed_gen->update(speed_pu_t{0.0});
    }

    // Speed should be clamped to max limit (within tolerance)
    EXPECT_LE(speed_gen->get_speed_reference_pu().value(), 0.80 + 1e-6);
    // Torque should be clamped to max limit (within tolerance)
    EXPECT_LE(speed_gen->get_torque_limit_max_pu().value(), 1.5 + 1e-6);
}

// Test sequencing through multiple profiles
TEST_F(SfcSoftStarterSpeedGeneratorTest, SequenceThroughMultipleProfiles)
{
    // Configure profiles
    auto purge = sfc_soft_starter_speed_generator::make_purge_accel_profile(speed_pu_t{0.20}, duration_t{1.0});
    auto ignition = sfc_soft_starter_speed_generator::make_ignition_accel_profile(speed_pu_t{0.55}, duration_t{1.0});

    speed_gen->configure_profile(acceleration_profile_t::PROFILE_2, purge);
    speed_gen->configure_profile(acceleration_profile_t::PROFILE_3, ignition);

    // Step 1: Run purge profile
    speed_gen->select_profile(acceleration_profile_t::PROFILE_2);
    speed_gen->execute_command(speed_command_t::RUN_PROFILE);

    for (int i = 0; i < 1500; ++i) {
        speed_gen->update(speed_pu_t{0.0});
    }

    EXPECT_TRUE(speed_gen->is_profile_complete());
    const real_t purge_speed = speed_gen->get_speed_reference_pu().value();
    EXPECT_NEAR(purge_speed, 0.20, 0.01);

    // Step 2: Hold at purge speed
    speed_gen->execute_command(speed_command_t::HOLD_SPEED);
    speed_gen->update(speed_pu_t{purge_speed});
    EXPECT_FLOAT_EQ(speed_gen->get_speed_reference_pu().value(), purge_speed);

    // Step 3: Run ignition profile
    speed_gen->select_profile(acceleration_profile_t::PROFILE_3);
    speed_gen->execute_command(speed_command_t::RUN_PROFILE);

    for (int i = 0; i < 1500; ++i) {
        speed_gen->update(speed_pu_t{purge_speed});
    }

    EXPECT_TRUE(speed_gen->is_profile_complete());
    EXPECT_NEAR(speed_gen->get_speed_reference_pu().value(), 0.55, 0.02);
}

// Test reset functionality
TEST_F(SfcSoftStarterSpeedGeneratorTest, ResetWorks)
{
    auto config = sfc_soft_starter_speed_generator::make_purge_accel_profile();
    speed_gen->configure_profile(acceleration_profile_t::PROFILE_2, config);
    speed_gen->select_profile(acceleration_profile_t::PROFILE_2);
    speed_gen->execute_command(speed_command_t::RUN_PROFILE);

    speed_gen->update(speed_pu_t{0.0});

    // Reset to zero
    speed_gen->reset();

    EXPECT_EQ(speed_gen->get_active_command(), speed_command_t::IDLE);
    EXPECT_FLOAT_EQ(speed_gen->get_speed_reference_pu().value(), 0.0);
    EXPECT_FALSE(speed_gen->is_profile_complete());
    EXPECT_FALSE(speed_gen->is_breakaway_boost_active());
}

// Test reset with initial speed
TEST_F(SfcSoftStarterSpeedGeneratorTest, ResetWithInitialSpeed)
{
    speed_gen->reset(speed_pu_t{0.30});

    EXPECT_FLOAT_EQ(speed_gen->get_speed_reference_pu().value(), 0.30);
}
