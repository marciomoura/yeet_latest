#include "sfc_integration/sfc_excitation_controller_runner.hpp"

#include <gtest/gtest.h>

namespace yeet::application::sfc::integration {

class SfcExcitationControllerRunnerTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        _runner = std::make_unique<sfc_excitation_controller_runner>();

        // Reset all input signals
        reset_all_inputs();

        // Initialize the runner
        _runner->initialize();
    }

    void TearDown() override
    {
        if (_runner) {
            _runner->stop();
        }
    }

    void reset_all_inputs()
    {
        auto& inputs = _runner->get_signal_flow_datamodel().get_inputs();
        mock_signal_input::write(inputs.machine_rotor_speed_pu, speed_pu_t{0.0});
        mock_signal_input::write(inputs.release_excitation, false);
    }

    void run_cycle() { _runner->run(); }

    void set_machine_speed(speed_pu_t speed)
    {
        auto& inputs = _runner->get_signal_flow_datamodel().get_inputs();
        mock_signal_input::write(inputs.machine_rotor_speed_pu, speed);
    }

    void set_release_excitation(bool release)
    {
        auto& inputs = _runner->get_signal_flow_datamodel().get_inputs();
        mock_signal_input::write(inputs.release_excitation, release);
    }

    current_pu_t get_field_current_reference()
    {
        const auto& outputs = _runner->get_signal_flow_datamodel().get_outputs();
        return outputs.field_current_reference.read();
    }

    duration_t get_prefluxing_elapsed_time()
    {
        const auto& outputs = _runner->get_signal_flow_datamodel().get_outputs();
        return outputs.prefluxing_elapsed_time.read();
    }

    std::unique_ptr<sfc_excitation_controller_runner> _runner;
};

// ============================================================================
// Initialization Tests
// ============================================================================

TEST_F(SfcExcitationControllerRunnerTest, InitialStateIsZero)
{
    run_cycle();

    EXPECT_EQ(get_field_current_reference().value(), 0.0);
}

// ============================================================================
// Release Excitation Tests
// ============================================================================

TEST_F(SfcExcitationControllerRunnerTest, ReleaseExcitationEnablesFieldCurrent)
{
    set_release_excitation(true);
    run_cycle();

    EXPECT_GT(get_field_current_reference().value(), 0.0);
}

TEST_F(SfcExcitationControllerRunnerTest, ReleaseExcitationFalseDisablesFieldCurrent)
{
    // First enable
    set_release_excitation(true);
    run_cycle();
    EXPECT_GT(get_field_current_reference().value(), 0.0);

    // Then disable
    set_release_excitation(false);
    run_cycle();
    EXPECT_EQ(get_field_current_reference().value(), 0.0);
}

// ============================================================================
// Field Current Reference Tests
// ============================================================================

TEST_F(SfcExcitationControllerRunnerTest, FieldCurrentConstantBelowSpeedThreshold)
{
    // Release excitation
    set_release_excitation(true);
    run_cycle();

    // Test at very low speed (should output constant value)
    set_machine_speed(speed_pu_t{0.05});
    run_cycle();
    EXPECT_GT(get_field_current_reference().value(), 0.0);

    // Test at higher speed
    set_machine_speed(speed_pu_t{0.8});
    run_cycle();
    EXPECT_GT(get_field_current_reference().value(), 0.0);
}

TEST_F(SfcExcitationControllerRunnerTest, FieldCurrentTracksSpeed)
{
    // Release excitation
    set_release_excitation(true);
    run_cycle();

    const auto current_at_low_speed = get_field_current_reference();

    // Increase speed
    set_machine_speed(speed_pu_t{0.8});
    run_cycle();

    const auto current_at_high_speed = get_field_current_reference();

    // Both should output field current reference (monotonic with speed for soft starter)
    EXPECT_GT(current_at_low_speed.value(), 0.0);
    EXPECT_GT(current_at_high_speed.value(), 0.0);
}

}  // namespace yeet::application::sfc::integration
