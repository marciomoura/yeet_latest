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
        mock_signal_input::write(inputs.exciter_ready, false);
        mock_signal_input::write(inputs.start_command, false);
        mock_signal_input::write(inputs.stop_command, false);
    }

    void run_cycle() { _runner->run(); }

    void set_machine_speed(speed_pu_t speed)
    {
        auto& inputs = _runner->get_signal_flow_datamodel().get_inputs();
        mock_signal_input::write(inputs.machine_rotor_speed_pu, speed);
    }

    void set_exciter_ready(bool ready)
    {
        auto& inputs = _runner->get_signal_flow_datamodel().get_inputs();
        mock_signal_input::write(inputs.exciter_ready, ready);
    }

    void set_start_command(bool start)
    {
        auto& inputs = _runner->get_signal_flow_datamodel().get_inputs();
        mock_signal_input::write(inputs.start_command, start);
    }

    void set_stop_command(bool stop)
    {
        auto& inputs = _runner->get_signal_flow_datamodel().get_inputs();
        mock_signal_input::write(inputs.stop_command, stop);
    }

    voltage_pu_t get_excitation_voltage_setpoint()
    {
        const auto& outputs = _runner->get_signal_flow_datamodel().get_outputs();
        return outputs.excitation_voltage_setpoint_pu.read();
    }

    bool get_enable_excitation()
    {
        const auto& outputs = _runner->get_signal_flow_datamodel().get_outputs();
        return outputs.enable_excitation.read();
    }

    bool get_is_field_established()
    {
        const auto& outputs = _runner->get_signal_flow_datamodel().get_outputs();
        return outputs.is_field_established.read();
    }

    bool get_is_running()
    {
        const auto& outputs = _runner->get_signal_flow_datamodel().get_outputs();
        return outputs.is_running.read();
    }

    std::unique_ptr<sfc_excitation_controller_runner> _runner;
};

// ============================================================================
// Initialization Tests
// ============================================================================

TEST_F(SfcExcitationControllerRunnerTest, InitialStateIsZero)
{
    run_cycle();

    EXPECT_EQ(get_excitation_voltage_setpoint().value(), 0.0);
    EXPECT_FALSE(get_enable_excitation());
    EXPECT_FALSE(get_is_field_established());
    EXPECT_FALSE(get_is_running());
}

// ============================================================================
// Start Command Tests
// ============================================================================

TEST_F(SfcExcitationControllerRunnerTest, StartCommandInitiatesExcitation)
{
    set_exciter_ready(true);

    // Send start command (rising edge)
    set_start_command(true);
    run_cycle();

    // Controller should be running and excitation enabled
    EXPECT_TRUE(get_is_running());
    EXPECT_TRUE(get_enable_excitation());
    EXPECT_GT(get_excitation_voltage_setpoint().value(), 0.0);
}

TEST_F(SfcExcitationControllerRunnerTest, StartCommandEdgeDetection)
{
    set_exciter_ready(true);

    // First cycle: start command high (rising edge)
    set_start_command(true);
    run_cycle();
    EXPECT_TRUE(get_is_running());

    // Stop it
    set_stop_command(true);
    run_cycle();
    set_stop_command(false);
    for (int i = 0; i < 2000; ++i) {
        run_cycle();
    }
    EXPECT_FALSE(get_is_running());

    // Keep start command high (no rising edge, should not restart)
    run_cycle();
    EXPECT_FALSE(get_is_running());

    // Release and press again (rising edge, should restart)
    set_start_command(false);
    run_cycle();
    set_start_command(true);
    run_cycle();
    EXPECT_TRUE(get_is_running());
}

// ============================================================================
// Pre-Fluxing Tests
// ============================================================================

TEST_F(SfcExcitationControllerRunnerTest, PrefluxingPhaseMaintainsInitialVoltage)
{
    set_exciter_ready(true);
    set_start_command(true);
    run_cycle();
    set_start_command(false);

    const real_t initial_voltage = get_excitation_voltage_setpoint().value();

    // Run for 1 second (prefluxing is 2 seconds by default)
    for (int i = 0; i < 1000; ++i) {
        run_cycle();
    }

    // Field should not be established yet
    EXPECT_FALSE(get_is_field_established());

    // Voltage should remain approximately at initial value
    EXPECT_NEAR(get_excitation_voltage_setpoint().value(), initial_voltage, 0.01);
}

TEST_F(SfcExcitationControllerRunnerTest, FieldEstablishedAfterPrefluxing)
{
    set_exciter_ready(true);
    set_start_command(true);
    run_cycle();
    set_start_command(false);

    // Run past prefluxing duration (default is 2 seconds)
    for (int i = 0; i < 2500; ++i) {
        run_cycle();
    }

    // Field should be established
    EXPECT_TRUE(get_is_field_established());
}

// ============================================================================
// Speed Tracking Tests
// ============================================================================

TEST_F(SfcExcitationControllerRunnerTest, ExcitationVoltageTracksSpeed)
{
    set_exciter_ready(true);
    set_start_command(true);
    run_cycle();
    set_start_command(false);

    // Complete prefluxing (2 seconds default)
    for (int i = 0; i < 2500; ++i) {
        run_cycle();
    }

    ASSERT_TRUE(get_is_field_established());

    const real_t voltage_at_zero_speed = get_excitation_voltage_setpoint().value();

    // Increase speed to 50%
    set_machine_speed(speed_pu_t{0.5});
    for (int i = 0; i < 1000; ++i) {
        run_cycle();
    }

    const real_t voltage_at_half_speed = get_excitation_voltage_setpoint().value();

    // Voltage should increase with speed (V/Hz control)
    EXPECT_GT(voltage_at_half_speed, voltage_at_zero_speed);
}

// ============================================================================
// Stop Command Tests
// ============================================================================

TEST_F(SfcExcitationControllerRunnerTest, StopCommandRampsDownExcitation)
{
    set_exciter_ready(true);
    set_start_command(true);
    run_cycle();
    set_start_command(false);

    // Let it run for a while
    for (int i = 0; i < 100; ++i) {
        run_cycle();
    }

    const real_t voltage_before_stop = get_excitation_voltage_setpoint().value();
    ASSERT_GT(voltage_before_stop, 0.0);

    // Send stop command
    set_stop_command(true);
    run_cycle();
    set_stop_command(false);

    // Run for a while to allow ramp-down
    for (int i = 0; i < 2000; ++i) {
        run_cycle();
    }

    // Should be stopped
    EXPECT_FALSE(get_is_running());
    EXPECT_FALSE(get_enable_excitation());
    EXPECT_NEAR(get_excitation_voltage_setpoint().value(), 0.0, 0.01);
}

// ============================================================================
// Fault Handling Tests
// ============================================================================

TEST_F(SfcExcitationControllerRunnerTest, ExciterNotReadyTriggersShutdown)
{
    set_exciter_ready(true);
    set_start_command(true);
    run_cycle();
    set_start_command(false);

    // Let it run
    for (int i = 0; i < 100; ++i) {
        run_cycle();
    }

    ASSERT_TRUE(get_is_running());

    // Simulate exciter fault
    set_exciter_ready(false);

    // Run to allow shutdown
    for (int i = 0; i < 2000; ++i) {
        run_cycle();
    }

    // Should be stopped
    EXPECT_FALSE(get_is_running());
    EXPECT_FALSE(get_enable_excitation());
}

}  // namespace yeet::application::sfc::integration
