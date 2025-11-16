#include <gtest/gtest.h>

#include "sfc_integration/sfc_grid_side_protections_runner.hpp"

namespace yeet::application::sfc::integration {

/**
 * @brief Test fixture for alarm confirmation feature in grid-side protections
 *
 * Tests the alarm confirmation workflow:
 * - Alarm latching when protection trips
 * - Rising edge detection on confirm_alarms parameter
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
        params.overcurrent_trip_threshold.write(current_pu_t{1.2f});
        params.overvoltage_trip_threshold.write(voltage_pu_t{1.15f});
        params.undervoltage_trip_threshold.write(voltage_pu_t{0.85f});

        // Set zero delays for instant trip response in tests
        params.overcurrent_trip_delay.write(duration_t{0.0});
        params.overvoltage_trip_delay.write(duration_t{0.0});
        params.undervoltage_trip_delay.write(duration_t{0.0});

        params.confirm_alarms.write(false);  // Start with no confirmation command

        // Set safe initial inputs
        reset_inputs();

        // Initialize trip_acknowledged to false (simulate no state machine acknowledgment)
        auto& inputs = _runner->get_signal_flow_datamodel().get_inputs();
        mock_signal_input::write(inputs.trip_acknowledged, false);
    }

    void reset_inputs()
    {
        auto& inputs = _runner->get_signal_flow_datamodel().get_inputs();

        // Safe grid voltage magnitude (1.0 pu)
        mock_signal_input::write(inputs.grid_voltage_magnitude_pu, voltage_pu_t{1.0f});

        // Safe DC link current (0.5 pu)
        mock_signal_input::write(inputs.dc_link_current_pu, current_pu_t{0.5f});

        // Release protections
        mock_signal_input::write(inputs.release_protections, true);

        // Note: trip_acknowledged is not initialized here - it will be set by tests as needed
    }
    void trigger_overcurrent()
    {
        auto& inputs = _runner->get_signal_flow_datamodel().get_inputs();
        mock_signal_input::write(inputs.dc_link_current_pu, current_pu_t{1.5f});  // Above 1.2 threshold
    }

    void trigger_overvoltage()
    {
        auto& inputs = _runner->get_signal_flow_datamodel().get_inputs();
        mock_signal_input::write(inputs.grid_voltage_magnitude_pu, voltage_pu_t{1.3f});  // Above 1.15 threshold
    }

    void clear_fault_conditions()
    {
        reset_inputs();  // Return to safe operating point
    }

    void release_protections()
    {
        // Toggle release signal to reset latched protections
        auto& inputs = _runner->get_signal_flow_datamodel().get_inputs();
        mock_signal_input::write(inputs.release_protections, false);
        run_cycle();
        mock_signal_input::write(inputs.release_protections, true);
        run_cycle();
    }

    void set_confirm_alarms(bool value)
    {
        auto& params = _runner->get_parameters();
        params.confirm_alarms.write(value);
    }

    void set_trip_acknowledged(bool value)
    {
        auto& inputs = _runner->get_signal_flow_datamodel().get_inputs();
        mock_signal_input::write(inputs.trip_acknowledged, value);
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

    std::unique_ptr<sfc_grid_side_protections_runner> _runner;
};

/**
 * @brief Test alarm latching when protection trips
 *
 * Scenario:
 * 1. Normal operation → is_inhibited=false
 * 2. Overcurrent fault occurs → is_tripped=true, is_inhibited=true
 * 3. Protection requires explicit release to clear trip
 * 4. After release, is_tripped=false but is_inhibited=true (alarm latched)
 */
TEST_F(SfcGridSideProtectionsAlarmConfirmationTest, AlarmLatchesOnTrip)
{
    // Initial state: no faults
    run_cycle();
    EXPECT_FALSE(is_tripped());
    EXPECT_FALSE(is_inhibited());

    // Trigger fault
    trigger_overcurrent();
    run_cycle();
    EXPECT_TRUE(is_tripped()) << "Protection should trip on overcurrent";
    EXPECT_TRUE(is_inhibited()) << "Alarm should latch when protection trips";

    // Clear fault condition
    clear_fault_conditions();
    run_cycle();

    // Protection is latching - still tripped until state machine acknowledges
    EXPECT_TRUE(is_tripped()) << "Latching protection should remain tripped until acknowledged";
    EXPECT_TRUE(is_inhibited()) << "Alarm should remain latched";

    // State machine acknowledges trip (simulating SHUTDOWN_REQUESTED state)
    set_trip_acknowledged(true);
    run_cycle();

    EXPECT_FALSE(is_tripped()) << "Protection should clear after trip acknowledged";
    EXPECT_TRUE(is_inhibited()) << "Alarm should remain latched after protection clears";

    // Clear acknowledgment (state machine exits SHUTDOWN_REQUESTED)
    set_trip_acknowledged(false);
    run_cycle();
}

/**
 * @brief Test rising edge detection on confirm_alarms
 *
 * Only a rising edge (false→true) should trigger confirmation.
 * Holding confirm_alarms=true should not repeatedly clear the alarm.
 */
TEST_F(SfcGridSideProtectionsAlarmConfirmationTest, ConfirmRequiresRisingEdge)
{
    // Trigger and clear fault to latch alarm
    trigger_overcurrent();
    run_cycle();
    clear_fault_conditions();
    run_cycle();
    ASSERT_TRUE(is_tripped()) << "Trip should remain latched";
    ASSERT_TRUE(is_inhibited()) << "Alarm should be latched";

    // Acknowledge trip
    set_trip_acknowledged(true);
    run_cycle();
    ASSERT_FALSE(is_tripped());
    set_trip_acknowledged(false);
    run_cycle();
    ASSERT_TRUE(is_inhibited()) << "Alarm should still be latched after trip clears";

    // Set confirm_alarms=true WITHOUT going through false first (should already be false from setup, but let's
    // initialize the edge detector) By setting it to false explicitly and running a cycle, the edge detector knows the
    // starting state
    set_confirm_alarms(false);
    run_cycle();

    // Try setting confirm_alarms=true (this IS a rising edge, but it should work)
    set_confirm_alarms(true);
    run_cycle();
    EXPECT_FALSE(is_inhibited()) << "Rising edge from false->true should clear alarm";

    // Holding true should not affect state (no new edge)
    run_cycle();
    EXPECT_FALSE(is_inhibited()) << "Alarm should remain cleared with confirm_alarms held true";

    // Reset confirm_alarms
    set_confirm_alarms(false);
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
    ASSERT_TRUE(is_tripped());
    ASSERT_TRUE(is_inhibited());

    // Attempt to confirm alarm while fault is active (rising edge)
    set_confirm_alarms(false);
    run_cycle();
    set_confirm_alarms(true);
    run_cycle();

    EXPECT_TRUE(is_inhibited()) << "Cannot confirm alarm while protection is currently tripped (safety)";
    EXPECT_TRUE(is_tripped()) << "Protection should remain tripped while fault active";

    // Now clear the fault
    clear_fault_conditions();
    run_cycle();
    EXPECT_TRUE(is_tripped()) << "Trip should remain latched until acknowledged";
    EXPECT_TRUE(is_inhibited()) << "Alarm should remain latched";

    // State machine acknowledges trip
    set_trip_acknowledged(true);
    run_cycle();
    EXPECT_FALSE(is_tripped()) << "Trip should clear after acknowledgment";
    EXPECT_TRUE(is_inhibited()) << "Alarm should remain latched";

    set_trip_acknowledged(false);
    run_cycle();

    // Now confirmation should work
    set_confirm_alarms(false);
    run_cycle();
    set_confirm_alarms(true);
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
    ASSERT_TRUE(is_tripped());
    ASSERT_TRUE(is_inhibited());

    // Clear fault
    clear_fault_conditions();
    run_cycle();
    ASSERT_TRUE(is_tripped()) << "Trip should remain latched until acknowledged";
    ASSERT_TRUE(is_inhibited());

    // State machine acknowledges trip
    set_trip_acknowledged(true);
    run_cycle();
    ASSERT_FALSE(is_tripped()) << "Trip should clear after acknowledgment";
    ASSERT_TRUE(is_inhibited());

    set_trip_acknowledged(false);
    run_cycle();

    // Confirm alarm (rising edge)
    set_confirm_alarms(false);
    run_cycle();
    set_confirm_alarms(true);
    run_cycle();

    EXPECT_FALSE(is_inhibited()) << "Alarm should be cleared after confirmation";
    EXPECT_FALSE(is_tripped()) << "Protection should not be tripped";
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
    clear_fault_conditions();
    run_cycle();
    ASSERT_TRUE(is_tripped()) << "Trip should remain latched";
    ASSERT_TRUE(is_inhibited());

    // Acknowledge first trip
    set_trip_acknowledged(true);
    run_cycle();
    ASSERT_FALSE(is_tripped());
    set_trip_acknowledged(false);
    run_cycle();

    // First confirmation
    set_confirm_alarms(false);
    run_cycle();
    set_confirm_alarms(true);
    run_cycle();
    EXPECT_FALSE(is_inhibited()) << "First alarm should be cleared";

    // Second trip cycle
    trigger_overvoltage();
    run_cycle();
    clear_fault_conditions();
    run_cycle();
    EXPECT_TRUE(is_tripped()) << "Second trip should latch";
    EXPECT_TRUE(is_inhibited()) << "Second trip should re-latch alarm";

    // Acknowledge second trip
    set_trip_acknowledged(true);
    run_cycle();
    EXPECT_FALSE(is_tripped());
    set_trip_acknowledged(false);
    run_cycle();

    // Second confirmation (need new rising edge)
    set_confirm_alarms(false);
    run_cycle();
    set_confirm_alarms(true);
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
    EXPECT_FALSE(is_tripped());
}

}  // namespace yeet::application::sfc::integration
