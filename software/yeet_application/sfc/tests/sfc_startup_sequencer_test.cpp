#include "sfc/sfc_startup_sequencer.hpp"

#include <gtest/gtest.h>

#include "yeet_control/constants.hpp"

using namespace yeet;
using namespace yeet::application::sfc;

class SfcStartupSequencerTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        // Create sequencer with 1ms sampling time
        sequencer = std::make_unique<sfc_startup_sequencer>(sampling_time);

        // Configure with default test parameters
        sequencer->configure_virtual_frequency(virtual_freq_cfg);
        sequencer->configure_virtual_torque(virtual_torque_cfg);
    }

    void run_cycles(int count, bool field_established = true, angle_t estimated_angle = angle_t{0.0})
    {
        for (int i = 0; i < count; ++i) {
            sequencer->update(field_established, estimated_angle);
        }
    }

    const duration_t sampling_time{ms(1.0)};  // 1 ms

    virtual_frequency_config virtual_freq_cfg{
        .initial = frequency_t{1.0},  // 1 Hz start
        .final = frequency_t{10.0},   // 10 Hz end
        .duration = duration_t{1.0}   // 1 second ramp
    };

    virtual_torque_config virtual_torque_cfg{
        .initial = torque_pu_t{0.2},  // 20% start
        .final = torque_pu_t{0.4},    // 40% end
        .duration = duration_t{0.5}   // 500ms ramp
    };

    std::unique_ptr<sfc_startup_sequencer> sequencer;
};

// ========== Construction and Initialization Tests ==========

TEST_F(SfcStartupSequencerTest, ConstructorInitializesToIDLE)
{
    EXPECT_EQ(sequencer->get_current_state(), startup_sequencer_state_t::IDLE);
    EXPECT_FALSE(sequencer->is_startup_complete());
}

TEST_F(SfcStartupSequencerTest, InitialOutputsAreSafe)
{
    auto outputs = sequencer->get_outputs();

    EXPECT_EQ(outputs.controller_commands.reference_frame_source, reference_frame_source_t::ESTIMATED);
    EXPECT_FALSE(outputs.controller_commands.release_machine_side_pulses);
    EXPECT_FALSE(outputs.controller_commands.release_excitation);
    EXPECT_FALSE(outputs.controller_commands.release_machine_observer);
    EXPECT_FALSE(outputs.controller_commands.release_speed_controller);
    EXPECT_FALSE(outputs.controller_commands.release_torque_controller);
    EXPECT_EQ(outputs.virtual_reference.torque_reference.value(), 0.0f);
}

// ========== State Transition Tests ==========

TEST_F(SfcStartupSequencerTest, StartSequenceTransitionsToFluxBuildup)
{
    EXPECT_TRUE(sequencer->start_sequence());
    EXPECT_EQ(sequencer->get_current_state(), startup_sequencer_state_t::FLUX_BUILDUP);
}

TEST_F(SfcStartupSequencerTest, CannotStartSequenceFromNonIDLEState)
{
    sequencer->start_sequence();                // Move to FLUX_BUILDUP
    EXPECT_FALSE(sequencer->start_sequence());  // Should fail
    EXPECT_EQ(sequencer->get_current_state(), startup_sequencer_state_t::FLUX_BUILDUP);
}

TEST_F(SfcStartupSequencerTest, FluxBuildupTransitionsToStartingProcedureWhenComplete)
{
    sequencer->start_sequence();
    EXPECT_EQ(sequencer->get_current_state(), startup_sequencer_state_t::FLUX_BUILDUP);

    // Field not yet established - should stay in FLUX_BUILDUP
    run_cycles(500, false);
    EXPECT_EQ(sequencer->get_current_state(), startup_sequencer_state_t::FLUX_BUILDUP);

    // Field established and timer elapsed (need 501 cycles total for > comparison)
    run_cycles(2, true);
    EXPECT_EQ(sequencer->get_current_state(), startup_sequencer_state_t::STARTING_PROCEDURE);
}

TEST_F(SfcStartupSequencerTest, StartingProcedureTransitionsAutomaticallyAfterDuration)
{
    // Start and get through flux buildup (transitions when is_field_established=true)
    sequencer->start_sequence();
    run_cycles(1, true);  // Flux buildup completes when is_field_established=true
    ASSERT_EQ(sequencer->get_current_state(), startup_sequencer_state_t::STARTING_PROCEDURE);

    // Run for virtual frequency duration (1000ms)
    run_cycles(1000, true);
    EXPECT_EQ(sequencer->get_current_state(), startup_sequencer_state_t::STARTING_PROCEDURE);

    // One more cycle should trigger transition
    run_cycles(1, true);
    EXPECT_EQ(sequencer->get_current_state(), startup_sequencer_state_t::CLOSED_LOOP_OPERATION);
}

TEST_F(SfcStartupSequencerTest, StopSequenceReturnsToIDLEFromAnyState)
{
    sequencer->start_sequence();
    EXPECT_EQ(sequencer->get_current_state(), startup_sequencer_state_t::FLUX_BUILDUP);

    sequencer->stop_sequence();
    EXPECT_EQ(sequencer->get_current_state(), startup_sequencer_state_t::IDLE);

    // Test from STARTING_PROCEDURE
    sequencer->start_sequence();
    run_cycles(1, true);  // Flux buildup completes when is_field_established=true
    EXPECT_EQ(sequencer->get_current_state(), startup_sequencer_state_t::STARTING_PROCEDURE);

    sequencer->stop_sequence();
    EXPECT_EQ(sequencer->get_current_state(), startup_sequencer_state_t::IDLE);

    // Test from CLOSED_LOOP_OPERATION
    sequencer->start_sequence();
    run_cycles(1, true);     // Flux buildup completes when is_field_established=true
    run_cycles(1001, true);  // Starting procedure (need 1001 cycles for timer > comparison)
    EXPECT_EQ(sequencer->get_current_state(), startup_sequencer_state_t::CLOSED_LOOP_OPERATION);

    sequencer->stop_sequence();
    EXPECT_EQ(sequencer->get_current_state(), startup_sequencer_state_t::IDLE);
}

// ========== Output Generation Tests ==========

TEST_F(SfcStartupSequencerTest, IDLEStateReleaseCommandsAreCorrect)
{
    auto outputs = sequencer->get_outputs();

    EXPECT_FALSE(outputs.controller_commands.release_excitation);
    EXPECT_FALSE(outputs.controller_commands.release_machine_observer);
    EXPECT_FALSE(outputs.controller_commands.release_speed_controller);
    EXPECT_FALSE(outputs.controller_commands.release_torque_controller);
}

TEST_F(SfcStartupSequencerTest, FluxBuildupOutputsAreCorrect)
{
    sequencer->start_sequence();
    // Run one cycle without field established to verify FLUX_BUILDUP outputs
    sequencer->update(false, angle_t{0.0});  // is_field_established = false
    auto outputs = sequencer->get_outputs();

    // Should still be in FLUX_BUILDUP state
    EXPECT_EQ(sequencer->get_current_state(), startup_sequencer_state_t::FLUX_BUILDUP);

    EXPECT_EQ(outputs.controller_commands.reference_frame_source, reference_frame_source_t::VIRTUAL);
    EXPECT_EQ(outputs.controller_commands.torque_control_mode, sfc_torque_controller::control_mode::MANUAL_TORQUE);
    EXPECT_EQ(outputs.virtual_reference.torque_reference.value(), 0.0f);
    EXPECT_FALSE(outputs.controller_commands.release_machine_side_pulses);

    // Release commands
    EXPECT_TRUE(outputs.controller_commands.release_excitation);  // Excitation enabled
    EXPECT_FALSE(outputs.controller_commands.release_machine_observer);
    EXPECT_FALSE(outputs.controller_commands.release_speed_controller);
    EXPECT_FALSE(outputs.controller_commands.release_torque_controller);
}

TEST_F(SfcStartupSequencerTest, StartingProcedureOutputsAreCorrect)
{
    sequencer->start_sequence();
    run_cycles(1, true);  // Flux buildup completes when is_field_established=true
    auto outputs = sequencer->get_outputs();

    EXPECT_EQ(outputs.controller_commands.reference_frame_source, reference_frame_source_t::VIRTUAL);
    EXPECT_EQ(outputs.controller_commands.torque_control_mode, sfc_torque_controller::control_mode::MANUAL_TORQUE);
    // Torque should be at initial value
    EXPECT_NEAR(outputs.virtual_reference.torque_reference.value(), 0.2f, 0.01f);
    EXPECT_TRUE(outputs.controller_commands.release_machine_side_pulses);

    // Release commands
    EXPECT_TRUE(outputs.controller_commands.release_excitation);
    EXPECT_TRUE(outputs.controller_commands.release_machine_observer);  // Observer enabled
    EXPECT_FALSE(outputs.controller_commands.release_speed_controller);
    EXPECT_TRUE(outputs.controller_commands.release_torque_controller);  // Torque controller enabled
}

TEST_F(SfcStartupSequencerTest, ClosedLoopOperationOutputsAreCorrect)
{
    sequencer->start_sequence();
    run_cycles(1, true);     // Flux buildup completes when is_field_established=true
    run_cycles(1001, true);  // Starting procedure (need 1001 cycles for timer > comparison)

    auto outputs = sequencer->get_outputs();

    EXPECT_EQ(outputs.controller_commands.reference_frame_source, reference_frame_source_t::ESTIMATED);
    EXPECT_EQ(outputs.controller_commands.torque_control_mode, sfc_torque_controller::control_mode::SPEED_CONTROL);
    EXPECT_TRUE(outputs.controller_commands.release_machine_side_pulses);

    // Release commands - all enabled
    EXPECT_TRUE(outputs.controller_commands.release_excitation);
    EXPECT_TRUE(outputs.controller_commands.release_machine_observer);
    EXPECT_TRUE(outputs.controller_commands.release_speed_controller);  // Speed controller enabled
    EXPECT_TRUE(outputs.controller_commands.release_torque_controller);
}

// ========== Virtual Reference Frame Tests ==========

TEST_F(SfcStartupSequencerTest, VirtualFrequencyRampsLinearly)
{
    sequencer->start_sequence();
    run_cycles(1, true);  // Flux buildup completes when is_field_established=true

    ASSERT_EQ(sequencer->get_current_state(), startup_sequencer_state_t::STARTING_PROCEDURE);

    // Check frequency at start (should be initial)
    EXPECT_NEAR(sequencer->get_virtual_frequency().value(), 1.0f, 0.01f);

    // Check frequency at 50% (500ms into 1000ms ramp)
    run_cycles(500, true);
    real_t expected_mid = 1.0f + (10.0f - 1.0f) * 0.5f;  // 5.5 Hz
    EXPECT_NEAR(sequencer->get_virtual_frequency().value(), expected_mid, 0.1f);

    // Check frequency at end (should be final)
    run_cycles(500, true);
    // Note: We're now in CLOSED_LOOP, so virtual frequency stops
}

TEST_F(SfcStartupSequencerTest, VirtualAngleIntegratesCorrectly)
{
    sequencer->start_sequence();
    run_cycles(1, true);  // Flux buildup completes when is_field_established=true, enter STARTING_PROCEDURE

    angle_t initial_angle = sequencer->get_outputs().virtual_reference.rotor_angle;

    // Run for 100ms in STARTING_PROCEDURE
    run_cycles(100, true);

    angle_t final_angle = sequencer->get_outputs().virtual_reference.rotor_angle;

    // Angle should have increased (integration of frequency)
    EXPECT_GT(final_angle.value(), initial_angle.value());
}

TEST_F(SfcStartupSequencerTest, VirtualAngleWrapsToTwoPi)
{
    sequencer->start_sequence();
    run_cycles(1, true);  // Flux buildup completes when is_field_established=true, enter STARTING_PROCEDURE

    // Run for full duration in STARTING_PROCEDURE
    for (int i = 0; i < 1000; ++i) {
        sequencer->update(true, angle_t{0.0});
        auto outputs = sequencer->get_outputs();
        // Angle should always be in [0, 2π]
        EXPECT_GE(outputs.virtual_reference.rotor_angle.value(), 0.0f);
        EXPECT_LT(outputs.virtual_reference.rotor_angle.value(), two_pi);
    }
}

// ========== Zero Duration Virtual Flux Tests ==========

TEST_F(SfcStartupSequencerTest, ZeroDurationVirtualFluxSkipsToClosedLoop)
{
    virtual_freq_cfg.duration = duration_t{0.0};
    sequencer->configure_virtual_frequency(virtual_freq_cfg);

    sequencer->start_sequence();
    run_cycles(1, true);  // Flux buildup completes when is_field_established=true

    // Should skip STARTING_PROCEDURE and go directly to CLOSED_LOOP
    EXPECT_EQ(sequencer->get_current_state(), startup_sequencer_state_t::CLOSED_LOOP_OPERATION);
}

// ========== Constant Frequency Tests ==========

TEST_F(SfcStartupSequencerTest, ConstantFrequencyMaintainedWhenInitialEqualsFinal)
{
    virtual_freq_cfg.initial = frequency_t{5.0};
    virtual_freq_cfg.final = frequency_t{5.0};
    virtual_freq_cfg.duration = duration_t{1.0};
    sequencer->configure_virtual_frequency(virtual_freq_cfg);

    sequencer->start_sequence();
    run_cycles(1, true);  // Flux buildup completes when is_field_established=true

    // Check frequency at start
    EXPECT_NEAR(sequencer->get_virtual_frequency().value(), 5.0f, 0.01f);

    // Check frequency at middle
    run_cycles(500, true);
    EXPECT_NEAR(sequencer->get_virtual_frequency().value(), 5.0f, 0.01f);

    // Should transition after duration (need 1001 cycles total in STARTING_PROCEDURE)
    run_cycles(501, true);  // 501 more = 1001 total in STARTING_PROCEDURE
    EXPECT_EQ(sequencer->get_current_state(), startup_sequencer_state_t::CLOSED_LOOP_OPERATION);
}

// ========== Torque Ramping Tests ==========

TEST_F(SfcStartupSequencerTest, VirtualTorqueRampsLinearly)
{
    sequencer->start_sequence();
    run_cycles(1, true);  // Flux buildup completes when is_field_established=true

    ASSERT_EQ(sequencer->get_current_state(), startup_sequencer_state_t::STARTING_PROCEDURE);

    // Check torque at start (should be initial)
    auto outputs = sequencer->get_outputs();
    EXPECT_NEAR(outputs.virtual_reference.torque_reference.value(), 0.2f, 0.01f);

    // Check torque at 50% (250ms into 500ms ramp)
    run_cycles(250, true);
    outputs = sequencer->get_outputs();
    real_t expected_mid = 0.2f + (0.4f - 0.2f) * 0.5f;  // 0.3 pu
    EXPECT_NEAR(outputs.virtual_reference.torque_reference.value(), expected_mid, 0.02f);

    // Check torque at end (500ms into 500ms ramp)
    run_cycles(250, true);
    outputs = sequencer->get_outputs();
    EXPECT_NEAR(outputs.virtual_reference.torque_reference.value(), 0.4f, 0.01f);
}

TEST_F(SfcStartupSequencerTest, ZeroDurationTorqueJumpsToFinal)
{
    virtual_torque_cfg.duration = duration_t{0.0};
    sequencer->configure_virtual_torque(virtual_torque_cfg);

    sequencer->start_sequence();
    run_cycles(1, true);  // Flux buildup completes when is_field_established=true

    // Torque should immediately be at final value
    // We need one more cycle to apply the zero-duration jump in update_starting_procedure()
    run_cycles(1, true);
    auto outputs = sequencer->get_outputs();
    EXPECT_NEAR(outputs.virtual_reference.torque_reference.value(), 0.4f, 0.01f);
}

TEST_F(SfcStartupSequencerTest, ConstantTorqueWhenInitialEqualsFinal)
{
    virtual_torque_cfg.initial = torque_pu_t{0.35};
    virtual_torque_cfg.final = torque_pu_t{0.35};
    virtual_torque_cfg.duration = duration_t{1.0};
    sequencer->configure_virtual_torque(virtual_torque_cfg);

    sequencer->start_sequence();
    run_cycles(1, true);  // Flux buildup completes when is_field_established=true

    // Check torque at start
    auto outputs = sequencer->get_outputs();
    EXPECT_NEAR(outputs.virtual_reference.torque_reference.value(), 0.35f, 0.01f);

    // Check torque at middle
    run_cycles(500, true);
    outputs = sequencer->get_outputs();
    EXPECT_NEAR(outputs.virtual_reference.torque_reference.value(), 0.35f, 0.01f);
}

// ========== Reset Tests ==========

TEST_F(SfcStartupSequencerTest, ResetReturnsToIDLE)
{
    sequencer->start_sequence();
    run_cycles(500, true);
    ASSERT_NE(sequencer->get_current_state(), startup_sequencer_state_t::IDLE);

    sequencer->reset();
    EXPECT_EQ(sequencer->get_current_state(), startup_sequencer_state_t::IDLE);
}

TEST_F(SfcStartupSequencerTest, ResetClearsVirtualReferenceFrame)
{
    sequencer->start_sequence();
    run_cycles(500, true);  // Flux buildup
    run_cycles(500, true);  // Half of starting procedure

    sequencer->reset();

    auto outputs = sequencer->get_outputs();
    EXPECT_EQ(outputs.virtual_reference.rotor_angle.value(), 0.0f);
    EXPECT_EQ(outputs.virtual_reference.rotor_frequency.value(), 0.0f);
    EXPECT_EQ(outputs.virtual_reference.torque_reference.value(), 0.0f);
}

// ========== Status Query Tests ==========

TEST_F(SfcStartupSequencerTest, IsStartupCompleteOnlyTrueInClosedLoop)
{
    EXPECT_FALSE(sequencer->is_startup_complete());

    sequencer->start_sequence();
    EXPECT_FALSE(sequencer->is_startup_complete());

    run_cycles(1, true);  // Flux buildup completes when is_field_established=true
    EXPECT_FALSE(sequencer->is_startup_complete());

    run_cycles(1001, true);  // STARTING_PROCEDURE complete (need 1001 cycles for timer > comparison)
    EXPECT_TRUE(sequencer->is_startup_complete());

    sequencer->stop_sequence();
    EXPECT_FALSE(sequencer->is_startup_complete());
}

TEST_F(SfcStartupSequencerTest, TimeInCurrentStateTracksCorrectly)
{
    sequencer->start_sequence();

    run_cycles(100, true);  // Still in FLUX_BUILDUP, is_field_established = true
    // Time in FLUX_BUILDUP should be near 100ms (will transition on first cycle)
    // Actually, with is_field_established=true, it transitions on first call to update()
    // So we're actually in STARTING_PROCEDURE after the first cycle
    // Let me verify state and time accordingly

    // Run 100 cycles in FLUX_BUILDUP with field NOT established
    sequencer->stop_sequence();
    sequencer->reset();
    sequencer->start_sequence();

    run_cycles(100, false);  // Keep field NOT established to stay in FLUX_BUILDUP
    EXPECT_NEAR(sequencer->get_time_in_current_state().value(), 0.1, 0.001);

    run_cycles(100, false);  // Continue in FLUX_BUILDUP
    EXPECT_NEAR(sequencer->get_time_in_current_state().value(), 0.2, 0.001);

    // Transition to STARTING_PROCEDURE should reset timer
    run_cycles(1, true);                                              // Set field_established=true, triggers transition
    EXPECT_LT(sequencer->get_time_in_current_state().value(), 0.01);  // Should be reset
}

// ========== Configuration Tests ==========

TEST_F(SfcStartupSequencerTest, ConfigurationIsApplied)
{
    // Create new configurations for virtual frequency and torque
    virtual_frequency_config new_freq_cfg{
        .initial = frequency_t{2.0}, .final = frequency_t{8.0}, .duration = duration_t{0.5}};

    virtual_torque_config new_torque_cfg{
        .initial = torque_pu_t{0.4}, .final = torque_pu_t{0.4}, .duration = duration_t{0.5}};

    // Note: flux_buildup configuration is no longer available; field establishment is now controlled
    // by the excitation controller via the is_field_established signal
    sequencer->configure_virtual_frequency(new_freq_cfg);
    sequencer->configure_virtual_torque(new_torque_cfg);

    sequencer->start_sequence();
    run_cycles(1, true);  // Flux buildup completes when is_field_established=true

    EXPECT_EQ(sequencer->get_current_state(), startup_sequencer_state_t::STARTING_PROCEDURE);

    auto outputs = sequencer->get_outputs();
    EXPECT_NEAR(outputs.virtual_reference.torque_reference.value(), 0.4f, 0.01f);
    EXPECT_NEAR(outputs.virtual_reference.rotor_frequency.value(), 2.0f, 0.01f);
}

// ========== Edge Case Tests ==========

TEST_F(SfcStartupSequencerTest, FieldNotEstablishedKeepsInFluxBuildup)
{
    sequencer->start_sequence();

    // Run for longer than configured duration but field never establishes
    run_cycles(1000, false);  // false = field not established

    // Should still be in FLUX_BUILDUP
    EXPECT_EQ(sequencer->get_current_state(), startup_sequencer_state_t::FLUX_BUILDUP);
}

TEST_F(SfcStartupSequencerTest, VeryShortDurationsHandledCorrectly)
{
    // Configure very short durations for virtual frequency and torque
    // Note: flux_buildup duration is no longer configurable; it transitions immediately when is_field_established=true
    virtual_freq_cfg.duration = duration_t{0.001};  // 1ms
    sequencer->configure_virtual_frequency(virtual_freq_cfg);

    virtual_torque_cfg.duration = duration_t{0.001};  // 1ms
    sequencer->configure_virtual_torque(virtual_torque_cfg);

    sequencer->start_sequence();
    run_cycles(1, true);  // Flux buildup completes immediately when is_field_established=true
    EXPECT_EQ(sequencer->get_current_state(), startup_sequencer_state_t::STARTING_PROCEDURE);

    run_cycles(1, true);
    EXPECT_EQ(sequencer->get_current_state(), startup_sequencer_state_t::CLOSED_LOOP_OPERATION);
}
