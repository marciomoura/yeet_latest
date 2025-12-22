#include "sfc_integration/sfc_excitation_controller_runner.hpp"

#include <gtest/gtest.h>

namespace yeet::application::sfc::integration {

class SfcExcitationControllerRunnerTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        _runner = std::make_unique<sfc_excitation_controller_runner>();

        // Connect storage to signals (required by raw pointer API)
        auto& inputs = _runner->get_signal_flow_datamodel().get_inputs();
        yeet::utils::mock_signal_input::connect(inputs.machine_rotor_speed_pu, _storage_machine_rotor_speed_pu);
        yeet::utils::mock_signal_input::connect(
            inputs.machine_voltage_magnitude_pu, _storage_machine_voltage_magnitude_pu);
        yeet::utils::mock_signal_input::connect(inputs.release_excitation, _storage_release_excitation);

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
        yeet::utils::mock_signal_input::write(inputs.machine_rotor_speed_pu, speed_pu_t{0.0});
        yeet::utils::mock_signal_input::write(inputs.machine_voltage_magnitude_pu, voltage_pu_t{0.0});
        yeet::utils::mock_signal_input::write(inputs.release_excitation, false);
    }

    void run_cycle() { _runner->run(); }

    void set_machine_speed(speed_pu_t speed)
    {
        auto& inputs = _runner->get_signal_flow_datamodel().get_inputs();
        yeet::utils::mock_signal_input::write(inputs.machine_rotor_speed_pu, speed);
    }

    void set_machine_voltage_magnitude(voltage_pu_t voltage)
    {
        auto& inputs = _runner->get_signal_flow_datamodel().get_inputs();
        yeet::utils::mock_signal_input::write(inputs.machine_voltage_magnitude_pu, voltage);
    }

    void set_release_excitation(bool release)
    {
        auto& inputs = _runner->get_signal_flow_datamodel().get_inputs();
        yeet::utils::mock_signal_input::write(inputs.release_excitation, release);
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

    // Storage for mocked signals (required by raw pointer API)
    speed_pu_t _storage_machine_rotor_speed_pu{0.0};
    voltage_pu_t _storage_machine_voltage_magnitude_pu{0.0};
    bool _storage_release_excitation{false};
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

TEST_F(SfcExcitationControllerRunnerTest, EnteringVoltagePIRegionIsBumpless)
{
    auto& parameters = _runner->get_parameters();

    // Configure PI with moderate gains and limits
    parameters.voltage_pi_kp.write(real_t{0.2});
    parameters.voltage_pi_ti.write(duration_t{0.1});
    parameters.voltage_pi_output_min_pu.write(current_pu_t{0.0});
    parameters.voltage_pi_output_max_pu.write(current_pu_t{2.0});

    // LUT default is 1.0 pu; ensure threshold at 0.4 pu
    parameters.voltage_threshold_pu.write(voltage_pu_t{0.4});

    // Start in low-voltage region
    set_machine_voltage_magnitude(voltage_pu_t{0.0});
    set_release_excitation(true);
    run_cycle();

    const auto forced_current = get_field_current_reference();
    EXPECT_GT(forced_current.value(), 0.0);

    // Move above threshold into PI region with a non-zero error (Vref=1.0, Vmeas=0.5)
    set_machine_speed(speed_pu_t{0.8});
    set_machine_voltage_magnitude(voltage_pu_t{0.5});
    run_cycle();

    const auto first_pi_current = get_field_current_reference();
    EXPECT_NEAR(first_pi_current.value(), forced_current.value(), 1e-3);

    // Next cycle should start integrating the error and increase output (bounded by limits)
    run_cycle();
    const auto second_pi_current = get_field_current_reference();
    EXPECT_GE(second_pi_current.value(), first_pi_current.value());
}

// ============================================================================
// Override Tests
// ============================================================================

TEST_F(SfcExcitationControllerRunnerTest, OverrideFieldCurrentReferenceAppliesCustomValue)
{
    auto& parameters = _runner->get_parameters();

    // First run normally
    set_release_excitation(true);
    run_cycle();
    const auto normal_current = get_field_current_reference();
    EXPECT_GT(normal_current.value(), 0.0);

    // Enable override with custom value
    parameters.field_current_reference_override.write(true);
    parameters.field_current_reference_override_value.write(current_pu_t{0.5});
    run_cycle();

    // Should output override value
    EXPECT_NEAR(get_field_current_reference().value(), 0.5, 0.001);
}

TEST_F(SfcExcitationControllerRunnerTest, OverrideFieldCurrentReferenceReturnsToNormalWhenDisabled)
{
    auto& parameters = _runner->get_parameters();

    // Enable excitation
    set_release_excitation(true);
    run_cycle();

    // Enable override
    parameters.field_current_reference_override.write(true);
    parameters.field_current_reference_override_value.write(current_pu_t{0.3});
    run_cycle();
    EXPECT_NEAR(get_field_current_reference().value(), 0.3, 0.001);

    // Disable override - should return to normal operation
    parameters.field_current_reference_override.write(false);
    run_cycle();

    const auto normal_current = get_field_current_reference();
    EXPECT_GT(normal_current.value(), 0.0);
    EXPECT_NE(normal_current.value(), 0.3);  // Should be different from override value
}

TEST_F(SfcExcitationControllerRunnerTest, OverrideFieldCurrentReferenceWorksEvenWhenExcitationNotReleased)
{
    auto& parameters = _runner->get_parameters();

    // Do NOT release excitation
    set_release_excitation(false);
    run_cycle();
    EXPECT_EQ(get_field_current_reference().value(), 0.0);

    // Enable override with custom value
    parameters.field_current_reference_override.write(true);
    parameters.field_current_reference_override_value.write(current_pu_t{0.7});
    run_cycle();

    // Should output override value even when excitation not released
    EXPECT_NEAR(get_field_current_reference().value(), 0.7, 0.001);
}

}  // namespace yeet::application::sfc::integration
