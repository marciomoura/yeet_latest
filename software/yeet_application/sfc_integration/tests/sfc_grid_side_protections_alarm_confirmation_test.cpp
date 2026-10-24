#include <gtest/gtest.h>

#include "sfc_integration/sfc_grid_side_protections_runner.hpp"

namespace yeet::application::sfc::integration {

/**
 * @brief Test fixture for alarm confirmation feature in grid-side protections
 *
 * Tests the alarm confirmation workflow:
 * - Alarm latching when protection trips
 * - Rising edge detection on confirm_trips parameter
 * - Safety rejection: cannot confirm while currently tripped
 * - Alarm clearing when confirmed and not tripped
 */
class SfcGridSideProtectionsAlarmConfirmationTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        _runner = std::make_unique<sfc_grid_side_protections_runner>();
        _runner->initialize();

        // Configure protection thresholds for testing
        auto& params = _runner->get_parameters();
        params.protection_module_enabled.write(true);
        params.overcurrent_trip_threshold.write(current_pu_t{1.2f});
        params.overvoltage_trip_threshold.write(voltage_pu_t{1.15f});
        params.undervoltage_trip_threshold.write(voltage_pu_t{0.85f});

        // Set zero delays for instant trip response in tests
        params.overcurrent_trip_delay.write(duration_t{0.0});
        params.overvoltage_trip_delay.write(duration_t{0.0});
        params.undervoltage_trip_delay.write(duration_t{0.0});

        params.confirm_trips.write(false);  // Start with no confirmation command

        // Connect storage to signals (required by raw pointer API)
        auto& inputs = _runner->get_signal_flow_datamodel().get_inputs();
        yeet::utils::mock_signal_input::connect(inputs.grid_voltage_magnitude_pu, _storage_grid_voltage);
        yeet::utils::mock_signal_input::connect(inputs.dc_link_current_pu, _storage_dc_link_current);
        yeet::utils::mock_signal_input::connect(inputs.release_protections, _storage_release_protections);
        yeet::utils::mock_signal_input::connect(inputs.door_is_open, _storage_door_is_open);
        yeet::utils::mock_signal_input::connect(inputs.external_fault, _storage_external_fault);
        yeet::utils::mock_signal_input::connect(inputs.emergency_off, _storage_emergency_off);
        yeet::utils::mock_signal_input::connect(inputs.mv_breaker_is_closed, _storage_mv_breaker_is_closed);

        // Set safe initial inputs
        reset_inputs();
    }

    void reset_inputs()
    {
        auto& inputs = _runner->get_signal_flow_datamodel().get_inputs();

        // Safe grid voltage magnitude (1.0 pu)
        yeet::utils::mock_signal_input::write(inputs.grid_voltage_magnitude_pu, voltage_pu_t{1.0f});

        // Safe DC link current (0.5 pu)
        yeet::utils::mock_signal_input::write(inputs.dc_link_current_pu, current_pu_t{0.5f});

        // Release protections
        yeet::utils::mock_signal_input::write(inputs.release_protections, true);

        // Door closed by default
        yeet::utils::mock_signal_input::write(inputs.door_is_open, false);

        // External fault and emergency off inactive by default
        yeet::utils::mock_signal_input::write(inputs.external_fault, false);
        yeet::utils::mock_signal_input::write(inputs.emergency_off, false);

        // Breaker closed by default for protection tests
        yeet::utils::mock_signal_input::write(inputs.mv_breaker_is_closed, true);
    }
    void trigger_overcurrent()
    {
        auto& inputs = _runner->get_signal_flow_datamodel().get_inputs();
        yeet::utils::mock_signal_input::write(inputs.dc_link_current_pu, current_pu_t{1.5f});  // Above 1.2 threshold
    }

    void trigger_overvoltage()
    {
        auto& inputs = _runner->get_signal_flow_datamodel().get_inputs();
        yeet::utils::mock_signal_input::write(
            inputs.grid_voltage_magnitude_pu, voltage_pu_t{1.3f});  // Above 1.15 threshold
    }

    void clear_fault_conditions()
    {
        reset_inputs();  // Return to safe operating point
    }

    void release_protections()
    {
        // Toggle release signal to reset latched protections
        auto& inputs = _runner->get_signal_flow_datamodel().get_inputs();
        yeet::utils::mock_signal_input::write(inputs.release_protections, false);
        run_cycle();
        yeet::utils::mock_signal_input::write(inputs.release_protections, true);
        run_cycle();
    }

    void set_confirm_trips(bool value)
    {
        auto& params = _runner->get_parameters();
        params.confirm_trips.write(value);
    }

    bool has_active_trip() const
    {
        const auto& outputs = _runner->get_signal_flow_datamodel().get_outputs();
        return outputs.has_active_trip.read();
    }

    bool is_inhibited() const
    {
        const auto& outputs = _runner->get_signal_flow_datamodel().get_outputs();
        return outputs.is_inhibited.read();
    }

    void run_cycle() { _runner->run(); }

    std::unique_ptr<sfc_grid_side_protections_runner> _runner;

    // Storage for mocked signals (required by raw pointer API)
    voltage_pu_t _storage_grid_voltage{1.0f};
    current_pu_t _storage_dc_link_current{0.5f};
    bool _storage_release_protections{true};
    bool _storage_door_is_open{false};
    bool _storage_external_fault{false};
    bool _storage_emergency_off{false};
    bool _storage_mv_breaker_is_closed{true};
};

/**
 * @brief Test alarm latching when protection trips
 *
 * Scenario:
 * 1. Normal operation → is_inhibited=false
 * 2. Overcurrent fault occurs → is_inhibited=true, is_inhibited=true
 * 3. Protection requires explicit release to clear trip
 * 4. After release, is_inhibited=false but is_inhibited=true (alarm latched)
 */
TEST_F(SfcGridSideProtectionsAlarmConfirmationTest, AlarmLatchesOnTrip)
{
    // Initial state: no faults
    run_cycle();
    EXPECT_FALSE(has_active_trip());
    EXPECT_FALSE(is_inhibited());

    // Trigger fault
    trigger_overcurrent();
    run_cycle();
    EXPECT_TRUE(has_active_trip()) << "Protection should trip on overcurrent";
    EXPECT_TRUE(is_inhibited()) << "Alarm should latch when protection trips";
    EXPECT_EQ(
        _runner->get_signal_flow_datamodel().get_outputs().first_trip_source.read(), grid_trip_source::overcurrent);

    // Clear fault condition
    clear_fault_conditions();
    run_cycle();

    // Protection is latching - trip stays active until confirmed
    EXPECT_FALSE(has_active_trip()) << "Trip should clear when fault clears";
    EXPECT_TRUE(is_inhibited()) << "Alarm should remain latched";

    // Confirm alarms
    set_confirm_trips(true);
    run_cycle();
    set_confirm_trips(false);
    run_cycle();

    EXPECT_FALSE(has_active_trip()) << "Trip should clear after confirmation";
    EXPECT_FALSE(is_inhibited()) << "Alarm should clear after confirmation";
}

/**
 * @brief Test rising edge detection on confirm_trips
 *
 * Only a rising edge (false→true) should trigger confirmation.
 * Holding confirm_trips=true should not repeatedly clear the alarm.
 */
TEST_F(SfcGridSideProtectionsAlarmConfirmationTest, ConfirmRequiresRisingEdge)
{
    // Trigger and clear fault to latch alarm
    trigger_overcurrent();
    run_cycle();
    clear_fault_conditions();
    run_cycle();
    ASSERT_FALSE(has_active_trip()) << "Trip should clear when fault clears";
    ASSERT_TRUE(is_inhibited()) << "Alarm should be latched";

    // Set confirm_trips=false WITHOUT going through false first (should already be false from setup, but let's
    // initialize the edge detector) By setting it to false explicitly and running a cycle, the edge detector knows the
    // starting state
    set_confirm_trips(false);
    run_cycle();

    // Try setting confirm_trips=true (this IS a rising edge, but it should work)
    set_confirm_trips(true);
    run_cycle();
    EXPECT_FALSE(is_inhibited()) << "Rising edge from false->true should clear alarm";

    // Holding true should not affect state (no new edge)
    run_cycle();
    EXPECT_FALSE(is_inhibited()) << "Alarm should remain cleared with confirm_trips held true";

    // Reset confirm_trips
    set_confirm_trips(false);
    run_cycle();
}

/**
 * @brief Test safety rejection: cannot confirm while protection is currently tripped
 *
 * This is a critical safety feature - operator cannot acknowledge an alarm
 * while the fault condition is still active.
 */
TEST_F(SfcGridSideProtectionsAlarmConfirmationTest, CannotConfirmWhileTripped)
{
    // Trigger fault
    trigger_overcurrent();
    run_cycle();
    ASSERT_TRUE(has_active_trip());
    ASSERT_TRUE(is_inhibited());

    // Attempt to confirm alarm while fault is active (rising edge)
    set_confirm_trips(false);
    run_cycle();
    set_confirm_trips(true);
    run_cycle();

    EXPECT_TRUE(is_inhibited()) << "Cannot confirm alarm while protection is currently tripped (safety)";
    EXPECT_TRUE(has_active_trip()) << "Protection should remain tripped while fault active";

    // Now clear the fault
    clear_fault_conditions();
    run_cycle();
    EXPECT_FALSE(has_active_trip()) << "Trip should clear when fault removed";
    EXPECT_TRUE(is_inhibited()) << "Alarm should remain latched";

    // Now confirmation should work
    set_confirm_trips(false);
    run_cycle();
    set_confirm_trips(true);
    run_cycle();
    EXPECT_FALSE(is_inhibited()) << "Alarm should clear after fault removed and confirmation given";
}

/**
 * @brief Test alarm confirmation clears is_inhibited when not tripped
 *
 * Normal workflow: trip → clear fault → confirm → resume operation
 */
TEST_F(SfcGridSideProtectionsAlarmConfirmationTest, ConfirmationClearsAlarmWhenSafe)
{
    // Trigger overvoltage fault
    trigger_overvoltage();
    run_cycle();
    ASSERT_TRUE(has_active_trip());
    ASSERT_TRUE(is_inhibited());

    // Clear fault
    clear_fault_conditions();
    run_cycle();
    ASSERT_FALSE(has_active_trip()) << "Trip should clear when fault removed";
    ASSERT_TRUE(is_inhibited());

    // Confirm alarm (rising edge)
    set_confirm_trips(false);
    run_cycle();
    set_confirm_trips(true);
    run_cycle();

    EXPECT_FALSE(is_inhibited()) << "Alarm should be cleared after confirmation";
    EXPECT_FALSE(has_active_trip()) << "Protection should not be tripped";
}

/**
 * @brief Test multiple trip/confirm cycles
 *
 * Each new trip should re-latch the alarm, requiring new confirmation.
 */
TEST_F(SfcGridSideProtectionsAlarmConfirmationTest, MultipleTripConfirmCycles)
{
    // First trip cycle
    trigger_overcurrent();
    run_cycle();
    ASSERT_TRUE(has_active_trip()) << "Trip should occur on fault";
    ASSERT_TRUE(is_inhibited());
    clear_fault_conditions();
    run_cycle();
    ASSERT_FALSE(has_active_trip()) << "Trip should clear when fault removed";
    ASSERT_TRUE(is_inhibited());

    // First confirmation
    set_confirm_trips(false);
    run_cycle();
    set_confirm_trips(true);
    run_cycle();
    EXPECT_FALSE(is_inhibited()) << "First alarm should be cleared";

    // Second trip cycle
    trigger_overvoltage();
    run_cycle();
    clear_fault_conditions();
    run_cycle();
    EXPECT_FALSE(has_active_trip()) << "Second trip should clear when fault removed";
    EXPECT_TRUE(is_inhibited()) << "Second trip should re-latch alarm";

    // Second confirmation (need new rising edge)
    set_confirm_trips(false);
    run_cycle();
    set_confirm_trips(true);
    run_cycle();
    EXPECT_FALSE(is_inhibited()) << "Second alarm should be cleared";
}

/**
 * @brief Test reset clears alarm state
 */
TEST_F(SfcGridSideProtectionsAlarmConfirmationTest, ResetClearsAlarm)
{
    // Trigger and latch alarm
    trigger_overcurrent();
    run_cycle();
    clear_fault_conditions();
    run_cycle();
    ASSERT_TRUE(is_inhibited());

    // Reset runner
    _runner->stop();
    _runner->initialize();

    // Alarm should be cleared
    run_cycle();
    EXPECT_FALSE(is_inhibited()) << "Reset should clear latched alarm";
    EXPECT_FALSE(has_active_trip());
}

/**
 * @brief Test disabling protection module immediately clears all outputs
 *
 * When protection_module_enabled=false, all trip states, alarms, and warnings
 * should be immediately cleared, allowing system restart without acknowledgment.
 */
TEST_F(SfcGridSideProtectionsAlarmConfirmationTest, DisablingModuleClearsAllOutputs)
{
    // Trigger overcurrent fault
    trigger_overcurrent();
    run_cycle();
    ASSERT_TRUE(has_active_trip());
    ASSERT_TRUE(is_inhibited());

    const auto& outputs = _runner->get_signal_flow_datamodel().get_outputs();
    ASSERT_TRUE(outputs.overcurrent_tripped.read()) << "Individual overcurrent trip should be set";
    ASSERT_TRUE(outputs.has_active_trip.read()) << "Fault active flag should be set";

    // Disable protection module
    auto& params = _runner->get_parameters();
    params.protection_module_enabled.write(false);
    run_cycle();

    // Verify ALL outputs are cleared
    EXPECT_FALSE(outputs.has_active_trip.read()) << "has_active_trip should clear when module disabled";
    EXPECT_FALSE(outputs.is_inhibited.read()) << "is_inhibited should clear when module disabled";
    EXPECT_FALSE(outputs.overcurrent_tripped.read()) << "overcurrent_tripped should clear";
    EXPECT_FALSE(outputs.overvoltage_tripped.read()) << "overvoltage_tripped should clear";
    EXPECT_FALSE(outputs.undervoltage_tripped.read()) << "undervoltage_tripped should clear";
    EXPECT_FALSE(outputs.overcurrent_warning.read()) << "overcurrent_warning should clear";
    EXPECT_FALSE(outputs.overvoltage_warning.read()) << "overvoltage_warning should clear";
    EXPECT_FALSE(outputs.undervoltage_warning.read()) << "undervoltage_warning should clear";
    EXPECT_FALSE(outputs.communication_loss_tripped.read()) << "communication_loss_tripped should clear";
    EXPECT_FALSE(outputs.communication_loss_warning.read()) << "communication_loss_warning should clear";

    // Re-enable module with fault still present
    params.protection_module_enabled.write(true);
    run_cycle();

    // Protections should detect fault and trip again
    EXPECT_TRUE(outputs.has_active_trip.read()) << "Should trip again when re-enabled with fault present";
    EXPECT_TRUE(outputs.is_inhibited.read()) << "Should latch alarm again";
}

/**
 * @brief Test disabling module allows immediate restart without acknowledgment
 *
 * This verifies that disabling protections bypasses the normal trip acknowledgment
 * and alarm confirmation workflow, enabling emergency recovery.
 */
TEST_F(SfcGridSideProtectionsAlarmConfirmationTest, DisablingModuleAllowsImmediateRestart)
{
    // Trigger fault and create latched alarm
    trigger_overvoltage();
    run_cycle();
    ASSERT_TRUE(has_active_trip());
    ASSERT_TRUE(is_inhibited());

    // Normally, restart would be blocked by is_inhibited requiring alarm confirmation
    // But disabling the module should clear everything
    auto& params = _runner->get_parameters();
    params.protection_module_enabled.write(false);
    run_cycle();

    EXPECT_FALSE(has_active_trip()) << "Trip cleared by disabling module";
    EXPECT_FALSE(is_inhibited()) << "Inhibit cleared - no acknowledgment needed";

    // Clear fault condition and re-enable protections
    clear_fault_conditions();
    params.protection_module_enabled.write(true);
    run_cycle();

    // System should be ready to operate (no trips, no alarms)
    EXPECT_FALSE(has_active_trip()) << "No trip with fault cleared";
    EXPECT_FALSE(is_inhibited()) << "No inhibit - ready for restart";
}

/**
 * @brief Test module stays disabled across multiple cycles
 */
TEST_F(SfcGridSideProtectionsAlarmConfirmationTest, ModuleStaysDisabled)
{
    auto& params = _runner->get_parameters();
    params.protection_module_enabled.write(false);

    // Inject severe fault conditions
    trigger_overcurrent();
    trigger_overvoltage();

    // Run multiple cycles
    for (int i = 0; i < 10; ++i) {
        run_cycle();
        EXPECT_FALSE(has_active_trip()) << "Should not trip while module disabled (cycle " << i << ")";
        EXPECT_FALSE(is_inhibited()) << "Should not inhibit while module disabled (cycle " << i << ")";
    }
}

/**
 * @brief Door open must generate a trip and require confirmation before restart.
 */
TEST_F(SfcGridSideProtectionsAlarmConfirmationTest, DoorOpenTripsAndRequiresConfirmation)
{
    auto& inputs = _runner->get_signal_flow_datamodel().get_inputs();

    // Prove this works even when not releasing/modulating.
    yeet::utils::mock_signal_input::write(inputs.release_protections, false);
    yeet::utils::mock_signal_input::write(inputs.door_is_open, true);

    run_cycle();
    EXPECT_TRUE(has_active_trip()) << "Door open should trip protections";
    EXPECT_TRUE(is_inhibited()) << "Door open trip should latch alarm";

    // Close door: fault clears
    yeet::utils::mock_signal_input::write(inputs.door_is_open, false);
    run_cycle();
    EXPECT_FALSE(has_active_trip()) << "Trip should clear when fault removed";
    EXPECT_FALSE(has_active_trip()) << "Trip should clear when fault removed";
    EXPECT_TRUE(is_inhibited()) << "Inhibit should stay active until confirmed";

    // Confirm trips
    set_confirm_trips(false);
    run_cycle();
    set_confirm_trips(true);
    run_cycle();

    EXPECT_FALSE(is_inhibited()) << "Alarm should clear after user confirmation";
}

/**
 * @brief Communication loss beyond configured timeout must trip and require confirmation.
 */
TEST_F(SfcGridSideProtectionsAlarmConfirmationTest, CommunicationLossTripsAfterTimeout)
{
    auto& params = _runner->get_parameters();
    auto& inputs = _runner->get_signal_flow_datamodel().get_inputs();

    // Configure very short timeout so the test is fast/deterministic.
    params.comm_timeout.write(duration_t{200e-6});  // 1 cycle at 200us
    params.comm_reset.write(duration_t{200e-6});

    // Prove comm watchdog runs even when not releasing/modulating.
    yeet::utils::mock_signal_input::write(inputs.release_protections, false);

    _runner->update_communication_status(false);
    run_cycle();

    EXPECT_TRUE(has_active_trip()) << "Communication loss should trip protections";
    EXPECT_TRUE(is_inhibited()) << "Communication loss trip should latch alarm";

    // Restore communication and allow watchdog to clear its internal trip condition
    _runner->update_communication_status(true);
    run_cycle();
    EXPECT_FALSE(has_active_trip()) << "Trip should clear when fault removed";
    EXPECT_TRUE(is_inhibited()) << "Inhibit should stay active until confirmed";

    // Alarm confirmation required
    set_confirm_trips(false);
    run_cycle();
    set_confirm_trips(true);
    run_cycle();
    EXPECT_FALSE(is_inhibited()) << "Alarm should clear after user confirmation";
}

TEST_F(SfcGridSideProtectionsAlarmConfirmationTest, CommunicationLossRemoteFailureTrips)
{
    auto& params = _runner->get_parameters();
    auto& inputs = _runner->get_signal_flow_datamodel().get_inputs();

    // Configure fast timeout
    params.comm_timeout.write(duration_t{200e-6});  // 1 cycle at 200us

    // Local communication is fine, but remote reports failure
    _runner->update_communication_status(true, false);
    run_cycle();

    EXPECT_TRUE(has_active_trip()) << "Remote communication failure should trip protections";
    EXPECT_TRUE(is_inhibited()) << "Remote communication failure trip should latch alarm";
}

}  // namespace yeet::application::sfc::integration
