#include <gtest/gtest.h>

#include "sfc_integration/sfc_machine_side_protections_runner.hpp"

namespace yeet::application::sfc::integration {

/**
 * @brief Test fixture for alarm confirmation feature in machine-side protections
 *
 * Tests the alarm confirmation workflow for machine-side protections:
 * - V/Hz (over-flux and under-flux)
 * - Stall protection
 * - Communication watchdog
 * - Alarm latching and confirmation
 * - Protection module enable/disable
 */
class SfcMachineSideProtectionsAlarmConfirmationTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        _runner = std::make_unique<sfc_machine_side_protections_runner>();
        _runner->initialize();

        // Configure protection thresholds for testing
        auto& params = _runner->get_parameters();
        params.protection_module_enabled.write(true);

        // Ensure default behavior for these tests: protections are tripping (not alarm-only)
        params.over_flux_severity.write(protection_severity::trip);
        params.under_flux_severity.write(protection_severity::trip);
        params.stall_severity.write(protection_severity::trip);
        params.overspeed_severity.write(protection_severity::trip);
        params.machine_overvoltage_severity.write(protection_severity::trip);
        params.communication_loss_severity.write(protection_severity::trip);

        // V/Hz protection
        // Machine-side runner uses per-unit speed as the denominator (V_pu / speed_pu).
        // With V_pu=1 and speed_pu=1, V/Hz should be 1.0.
        params.over_flux_trip_threshold.write(flux_pu_t{1.15f});
        params.under_flux_trip_threshold.write(flux_pu_t{0.80f});

        // Stall protection
        params.stall_current_threshold.write(current_pu_t{1.3f});
        params.stall_speed_threshold.write(speed_pu_t{0.10f});

        // Overspeed protection (set high so it's safe by default)
        params.overspeed_trip_threshold.write(speed_pu_t{1.50f});
        params.overspeed_trip_delay.write(duration_t{0.0});

        // Machine-side overvoltage protection (set high so it's safe by default)
        params.machine_overvoltage_trip_threshold.write(voltage_pu_t{1.50f});
        params.machine_overvoltage_trip_delay.write(duration_t{0.0});

        // Set zero delays for instant trip response in tests
        params.over_flux_trip_delay.write(duration_t{0.0});
        params.under_flux_trip_delay.write(duration_t{0.0});
        params.stall_trip_delay.write(duration_t{0.0});

        params.confirm_alarms.write(false);

        // Set safe initial inputs
        reset_inputs();
    }

    void reset_inputs()
    {
        auto& inputs = _runner->get_signal_flow_datamodel().get_inputs();

        // Safe baseline for all tests:
        // - Keep V/Hz at a safe value
        // - Keep speed below overspeed thresholds used in tests
        mock_signal_input::write(inputs.machine_voltage_magnitude_pu, voltage_pu_t{0.5f});

        // Safe machine frequency (50 Hz)
        mock_signal_input::write(inputs.machine_frequency, frequency_t{50.0});

        // Safe DC link current (0.5 pu)
        mock_signal_input::write(inputs.dc_link_current_pu, current_pu_t{0.5f});

        mock_signal_input::write(inputs.machine_speed, speed_pu_t{0.5f});
        mock_signal_input::write(inputs.machine_frequency_pu, frequency_pu_t{0.5f});

        // Not in startup
        mock_signal_input::write(inputs.startup_active, false);

        // Not in closed loop
        mock_signal_input::write(inputs.closed_loop_active, false);

        // Release protections
        mock_signal_input::write(inputs.release_protections, true);

        // Excitation is on by default
        mock_signal_input::write(inputs.excitation_is_on, true);
    }

    void trigger_over_flux()
    {
        auto& inputs = _runner->get_signal_flow_datamodel().get_inputs();
        // V/Hz_pu = V_pu / speed_pu.
        // With speed_pu=1.0: V/Hz_pu = V_pu.
        mock_signal_input::write(inputs.machine_voltage_magnitude_pu, voltage_pu_t{1.2f});
        mock_signal_input::write(inputs.machine_speed, speed_pu_t{1.0f});
        mock_signal_input::write(inputs.machine_frequency_pu, frequency_pu_t{1.0f});
        // VHz protection requires closed-loop operation
        mock_signal_input::write(inputs.closed_loop_active, true);
    }

    void trigger_under_flux()
    {
        auto& inputs = _runner->get_signal_flow_datamodel().get_inputs();
        // V/Hz_pu = V_pu / speed_pu.
        // With speed_pu=1.0: V/Hz_pu = V_pu.
        mock_signal_input::write(inputs.machine_voltage_magnitude_pu, voltage_pu_t{0.75f});
        mock_signal_input::write(inputs.machine_speed, speed_pu_t{1.0f});
        mock_signal_input::write(inputs.machine_frequency_pu, frequency_pu_t{1.0f});
        // VHz protection requires closed-loop operation
        mock_signal_input::write(inputs.closed_loop_active, true);
    }

    void trigger_stall()
    {
        auto& inputs = _runner->get_signal_flow_datamodel().get_inputs();
        // Stall requires: current > 1.3 pu AND speed < 0.10 pu AND startup_active
        mock_signal_input::write(inputs.dc_link_current_pu, current_pu_t{1.4f});
        mock_signal_input::write(inputs.machine_speed, speed_pu_t{0.05f});
        mock_signal_input::write(inputs.startup_active, true);
    }

    void trigger_overspeed(speed_pu_t trip_threshold = speed_pu_t{0.90f}, speed_pu_t speed = speed_pu_t{1.0f})
    {
        auto& params = _runner->get_parameters();
        params.overspeed_trip_threshold.write(trip_threshold);
        params.overspeed_trip_delay.write(duration_t{0.0});

        auto& inputs = _runner->get_signal_flow_datamodel().get_inputs();
        mock_signal_input::write(inputs.machine_speed, speed);
        // Overspeed protection requires closed-loop operation
        mock_signal_input::write(inputs.closed_loop_active, true);
    }

    void trigger_machine_overvoltage(
        voltage_pu_t trip_threshold = voltage_pu_t{0.90f}, voltage_pu_t voltage = voltage_pu_t{1.0f})
    {
        auto& params = _runner->get_parameters();
        params.machine_overvoltage_trip_threshold.write(trip_threshold);
        params.machine_overvoltage_trip_delay.write(duration_t{0.0});

        auto& inputs = _runner->get_signal_flow_datamodel().get_inputs();
        mock_signal_input::write(inputs.machine_voltage_magnitude_pu, voltage);
    }

    void clear_fault_conditions()
    {
        reset_inputs();
        // VHz and overspeed protections need to see safe values while closed_loop is active
        // to allow their internal detectors to reset before the trip handshake can clear
        auto& inputs = _runner->get_signal_flow_datamodel().get_inputs();
        mock_signal_input::write(inputs.closed_loop_active, true);
        run_cycle();  // Let protections see safe values and reset detectors
        mock_signal_input::write(inputs.closed_loop_active, false);
    }

    void set_confirm_alarms(bool value)
    {
        auto& params = _runner->get_parameters();
        params.confirm_alarms.write(value);
    }

    bool is_tripped() const
    {
        const auto& outputs = _runner->get_signal_flow_datamodel().get_outputs();
        return outputs.is_tripped.read();
    }

    bool is_inhibited() const
    {
        const auto& outputs = _runner->get_signal_flow_datamodel().get_outputs();
        return outputs.is_inhibited.read();
    }

    void run_cycle() { _runner->run(); }

    std::unique_ptr<sfc_machine_side_protections_runner> _runner;
};

/**
 * @brief Test alarm latching on over-flux trip
 */
TEST_F(SfcMachineSideProtectionsAlarmConfirmationTest, AlarmLatchesOnOverFluxTrip)
{
    // Initial state: no faults
    run_cycle();
    EXPECT_FALSE(is_tripped());
    EXPECT_FALSE(is_inhibited());

    // Trigger over-flux fault
    trigger_over_flux();
    run_cycle();
    EXPECT_TRUE(is_tripped()) << "Protection should trip on over-flux";
    EXPECT_TRUE(is_inhibited()) << "Alarm should latch when protection trips";

    // Clear fault condition
    clear_fault_conditions();
    run_cycle();

    EXPECT_TRUE(is_tripped()) << "Trip should stay active until confirmed";
    EXPECT_TRUE(is_inhibited()) << "Alarm should remain latched";

    // Confirm alarms
    set_confirm_alarms(true);
    run_cycle();
    set_confirm_alarms(false);
    run_cycle();

    EXPECT_FALSE(is_tripped()) << "Trip should clear after confirmation";
    EXPECT_FALSE(is_inhibited()) << "Alarm should clear after confirmation";
}

/**
 * @brief Test alarm latching on stall trip
 */
TEST_F(SfcMachineSideProtectionsAlarmConfirmationTest, AlarmLatchesOnStallTrip)
{
    // Trigger stall fault
    trigger_stall();
    run_cycle();
    EXPECT_TRUE(is_tripped()) << "Protection should trip on stall";
    EXPECT_TRUE(is_inhibited()) << "Alarm should latch";

    const auto& outputs = _runner->get_signal_flow_datamodel().get_outputs();
    EXPECT_TRUE(outputs.stall_tripped.read()) << "Stall protection should be tripped";

    // Clear fault
    clear_fault_conditions();
    run_cycle();
    EXPECT_TRUE(is_tripped()) << "Trip should stay active until confirmed";
    EXPECT_TRUE(is_inhibited()) << "Alarm should remain latched";

    // Confirm
    set_confirm_alarms(true);
    run_cycle();
    set_confirm_alarms(false);
    run_cycle();

    EXPECT_FALSE(is_tripped()) << "Trip should clear after confirmation";
    EXPECT_FALSE(is_inhibited()) << "Alarm should clear after confirmation";
}

/**
 * @brief Test alarm latching on overspeed trip
 */
TEST_F(SfcMachineSideProtectionsAlarmConfirmationTest, AlarmLatchesOnOverspeedTrip)
{
    // Trigger overspeed fault (threshold 0.9 pu, speed 1.0 pu)
    trigger_overspeed();
    run_cycle();
    EXPECT_TRUE(is_tripped()) << "Protection should trip on overspeed";
    EXPECT_TRUE(is_inhibited()) << "Alarm should latch";

    const auto& outputs = _runner->get_signal_flow_datamodel().get_outputs();
    EXPECT_TRUE(outputs.overspeed_tripped.read()) << "Overspeed protection should be tripped";

    // Clear fault
    clear_fault_conditions();
    run_cycle();
    EXPECT_TRUE(is_tripped()) << "Trip should stay active until confirmed";
    EXPECT_TRUE(is_inhibited()) << "Alarm still latched";

    // Confirm
    set_confirm_alarms(true);
    run_cycle();
    set_confirm_alarms(false);
    run_cycle();

    EXPECT_FALSE(is_tripped()) << "Trip should clear after confirmation";
    EXPECT_FALSE(is_inhibited()) << "Alarm should clear after confirmation";
}

/**
 * @brief Test alarm latching on machine-side overvoltage trip
 */
TEST_F(SfcMachineSideProtectionsAlarmConfirmationTest, AlarmLatchesOnMachineOvervoltageTrip)
{
    // Trigger machine-side overvoltage fault (threshold 0.9 pu, voltage 1.0 pu)
    trigger_machine_overvoltage();
    run_cycle();
    EXPECT_TRUE(is_tripped()) << "Protection should trip on machine-side overvoltage";
    EXPECT_TRUE(is_inhibited()) << "Alarm should latch";

    const auto& outputs = _runner->get_signal_flow_datamodel().get_outputs();
    EXPECT_TRUE(outputs.machine_overvoltage_tripped.read()) << "Machine-side overvoltage should be tripped";

    // Clear fault
    clear_fault_conditions();
    run_cycle();
    EXPECT_TRUE(is_tripped()) << "Trip should stay active until confirmed";
    EXPECT_TRUE(is_inhibited()) << "Alarm still latched";

    // Confirm
    set_confirm_alarms(true);
    run_cycle();
    set_confirm_alarms(false);
    run_cycle();

    EXPECT_FALSE(is_tripped()) << "Trip should clear after confirmation";
    EXPECT_FALSE(is_inhibited()) << "Alarm should clear after confirmation";
}

/**
 * @brief Test alarm confirmation workflow
 */
TEST_F(SfcMachineSideProtectionsAlarmConfirmationTest, ConfirmationClearsAlarmWhenSafe)
{
    // Trigger under-flux fault
    trigger_under_flux();
    run_cycle();
    ASSERT_TRUE(is_tripped());
    ASSERT_TRUE(is_inhibited());

    // Clear fault
    clear_fault_conditions();
    run_cycle();
    ASSERT_TRUE(is_tripped());
    ASSERT_TRUE(is_inhibited());

    // Confirm alarm (rising edge)
    set_confirm_alarms(false);
    run_cycle();
    set_confirm_alarms(true);
    run_cycle();

    EXPECT_FALSE(is_inhibited()) << "Alarm should be cleared after confirmation";
    EXPECT_FALSE(is_tripped()) << "Protection should not be tripped";
}

/**
 * @brief Test cannot confirm while tripped (safety feature)
 */
TEST_F(SfcMachineSideProtectionsAlarmConfirmationTest, CannotConfirmWhileTripped)
{
    // Trigger fault
    trigger_over_flux();
    run_cycle();
    ASSERT_TRUE(is_tripped());
    ASSERT_TRUE(is_inhibited());

    // Attempt to confirm while fault active (rising edge)
    set_confirm_alarms(false);
    run_cycle();
    set_confirm_alarms(true);
    run_cycle();

    EXPECT_TRUE(is_inhibited()) << "Cannot confirm alarm while protection is currently tripped (safety)";
    EXPECT_TRUE(is_tripped()) << "Protection should remain tripped while fault active";

    // Clear fault
    clear_fault_conditions();
    run_cycle();
    EXPECT_TRUE(is_tripped());
    EXPECT_TRUE(is_inhibited());

    // Now confirmation should work
    set_confirm_alarms(false);
    run_cycle();
    set_confirm_alarms(true);
    run_cycle();
    EXPECT_FALSE(is_inhibited()) << "Alarm should clear after fault removed and confirmation given";
}

/**
 * @brief Test disabling protection module immediately clears all outputs
 */
TEST_F(SfcMachineSideProtectionsAlarmConfirmationTest, DisablingModuleClearsAllOutputs)
{
    // Trigger over-flux fault
    trigger_over_flux();
    run_cycle();
    ASSERT_TRUE(is_tripped());
    ASSERT_TRUE(is_inhibited());

    const auto& outputs = _runner->get_signal_flow_datamodel().get_outputs();
    ASSERT_TRUE(outputs.over_flux_tripped.read()) << "Individual over-flux trip should be set";
    ASSERT_TRUE(outputs.any_fault_active.read()) << "Fault active flag should be set";

    // Disable protection module
    auto& params = _runner->get_parameters();
    params.protection_module_enabled.write(false);
    run_cycle();

    // Verify ALL outputs are cleared
    EXPECT_FALSE(outputs.is_tripped.read()) << "is_tripped should clear when module disabled";
    EXPECT_FALSE(outputs.is_inhibited.read()) << "is_inhibited should clear when module disabled";
    EXPECT_FALSE(outputs.any_fault_active.read()) << "any_fault_active should clear when module disabled";
    EXPECT_FALSE(outputs.over_flux_tripped.read()) << "over_flux_tripped should clear";
    EXPECT_FALSE(outputs.under_flux_tripped.read()) << "under_flux_tripped should clear";
    EXPECT_FALSE(outputs.stall_tripped.read()) << "stall_tripped should clear";
    EXPECT_FALSE(outputs.over_flux_warning.read()) << "over_flux_warning should clear";
    EXPECT_FALSE(outputs.under_flux_warning.read()) << "under_flux_warning should clear";
    EXPECT_FALSE(outputs.stall_warning.read()) << "stall_warning should clear";
    EXPECT_FALSE(outputs.communication_loss_tripped.read()) << "communication_loss_tripped should clear";
    EXPECT_FALSE(outputs.communication_loss_warning.read()) << "communication_loss_warning should clear";

    // Re-enable module with fault still present
    params.protection_module_enabled.write(true);
    run_cycle();

    // Protections should detect fault and trip again
    EXPECT_TRUE(outputs.is_tripped.read()) << "Should trip again when re-enabled with fault present";
    EXPECT_TRUE(outputs.is_inhibited.read()) << "Should latch alarm again";
}

/**
 * @brief Test disabling module allows immediate restart without acknowledgment
 */
TEST_F(SfcMachineSideProtectionsAlarmConfirmationTest, DisablingModuleAllowsImmediateRestart)
{
    // Trigger stall fault and create latched alarm
    trigger_stall();
    run_cycle();
    ASSERT_TRUE(is_tripped());
    ASSERT_TRUE(is_inhibited());

    // Disable module to bypass acknowledgment workflow
    auto& params = _runner->get_parameters();
    params.protection_module_enabled.write(false);
    run_cycle();

    EXPECT_FALSE(is_tripped()) << "Trip cleared by disabling module";
    EXPECT_FALSE(is_inhibited()) << "Inhibit cleared - no acknowledgment needed";

    // Clear fault condition and re-enable protections
    clear_fault_conditions();
    params.protection_module_enabled.write(true);
    run_cycle();

    // System should be ready to operate
    EXPECT_FALSE(is_tripped()) << "No trip with fault cleared";
    EXPECT_FALSE(is_inhibited()) << "No inhibit - ready for restart";
}

/**
 * @brief Test module stays disabled across multiple cycles with severe faults
 */
TEST_F(SfcMachineSideProtectionsAlarmConfirmationTest, ModuleStaysDisabled)
{
    auto& params = _runner->get_parameters();
    params.protection_module_enabled.write(false);

    // Inject multiple severe fault conditions
    auto& inputs = _runner->get_signal_flow_datamodel().get_inputs();
    mock_signal_input::write(inputs.machine_voltage_magnitude_pu, voltage_pu_t{2.0f});  // Severe over-flux
    mock_signal_input::write(inputs.machine_frequency, frequency_t{10.0});              // Very low frequency
    mock_signal_input::write(inputs.dc_link_current_pu, current_pu_t{2.0f});            // High current
    mock_signal_input::write(inputs.machine_speed, speed_pu_t{0.01f});                  // Near-stall speed

    // Run multiple cycles
    for (int i = 0; i < 10; ++i) {
        run_cycle();
        EXPECT_FALSE(is_tripped()) << "Should not trip while module disabled (cycle " << i << ")";
        EXPECT_FALSE(is_inhibited()) << "Should not inhibit while module disabled (cycle " << i << ")";
    }
}

/**
 * @brief Test multiple different protection types with alarm confirmation
 */
TEST_F(SfcMachineSideProtectionsAlarmConfirmationTest, MultipleFaultTypesCycle)
{
    // First: Over-flux
    trigger_over_flux();
    run_cycle();
    ASSERT_TRUE(is_tripped());
    clear_fault_conditions();
    run_cycle();
    ASSERT_TRUE(is_tripped());
    set_confirm_alarms(false);
    run_cycle();
    set_confirm_alarms(true);
    run_cycle();
    EXPECT_FALSE(is_inhibited()) << "First alarm cleared";

    // Second: Stall
    trigger_stall();
    run_cycle();
    EXPECT_TRUE(is_tripped());
    EXPECT_TRUE(is_inhibited()) << "Second fault should re-latch alarm";
    clear_fault_conditions();
    run_cycle();
    ASSERT_TRUE(is_tripped());
    set_confirm_alarms(false);
    run_cycle();
    set_confirm_alarms(true);
    run_cycle();
    EXPECT_FALSE(is_inhibited()) << "Second alarm cleared";
}

/**
 * @brief Test force_reset_all parameter clears all protection states
 */
TEST_F(SfcMachineSideProtectionsAlarmConfirmationTest, ForceResetClearsAllStates)
{
    // Trigger fault and latch alarm
    trigger_over_flux();
    run_cycle();
    ASSERT_TRUE(is_tripped());
    ASSERT_TRUE(is_inhibited());

    // Use force_reset_all parameter
    auto& params = _runner->get_parameters();
    params.force_reset_all.write(true);
    run_cycle();

    EXPECT_FALSE(is_tripped()) << "force_reset_all should clear trip";
    EXPECT_FALSE(is_inhibited()) << "force_reset_all should clear alarm";
    EXPECT_FALSE(params.force_reset_all.read()) << "force_reset_all should auto-clear after use";
}

}  // namespace yeet::application::sfc::integration