#include "sfc/sfc_grid_side_protections.hpp"

#include <gtest/gtest.h>

namespace yeet::application::sfc {

TEST(SfcGridSideProtectionsTest, DoorOpenTripClearsAutomatically)
{
    constexpr duration_t ts{200e-6};
    sfc_grid_side_protections protections(ts);

    protections.configure_door_open_is_trip(true);
    protections.configure_overcurrent_is_trip(true);
    protections.configure_overvoltage_is_trip(true);
    protections.configure_undervoltage_is_trip(true);
    protections.configure_communication_loss_is_trip(true);

    protections.reset();

    // Trip on door open
    protections.update_communication_watchdog(true);
    protections.update_overcurrent_protection(current_pu_t{0.0f});
    protections.update_overvoltage_protection(voltage_pu_t{1.0f});
    protections.update_undervoltage_protection(voltage_pu_t{1.0f});
    protections.resolve_protections(true, true, false);

    EXPECT_TRUE(protections.get_door_open_tripped());
    EXPECT_TRUE(protections.get_has_active_trip());
    EXPECT_TRUE(protections.get_is_inhibited());

    // Close door - trip clears automatically with auto_clear policy
    protections.update_communication_watchdog(true);
    protections.update_overcurrent_protection(current_pu_t{0.0f});
    protections.update_overvoltage_protection(voltage_pu_t{1.0f});
    protections.update_undervoltage_protection(voltage_pu_t{1.0f});
    protections.resolve_protections(false, true, false);

    EXPECT_FALSE(protections.get_has_active_trip());
    EXPECT_TRUE(protections.get_is_inhibited());

    // Confirm rising edge clears inhibit
    protections.update_communication_watchdog(true);
    protections.update_overcurrent_protection(current_pu_t{0.0f});
    protections.update_overvoltage_protection(voltage_pu_t{1.0f});
    protections.update_undervoltage_protection(voltage_pu_t{1.0f});
    protections.resolve_protections(false, true, true);

    EXPECT_FALSE(protections.get_is_inhibited());
}

TEST(SfcGridSideProtectionsTest, CommunicationWatchdogRunsWhenModuleEnabled)
{
    constexpr duration_t ts{200e-6};
    sfc_grid_side_protections protections(ts);

    protections.configure_communication_loss_is_trip(true);
    protections.configure_comm_timeout_duration(ts);  // 1 cycle
    protections.configure_comm_reset_duration(ts);

    protections.reset();

    // Module enabled - watchdog should run and trip on communication loss
    protections.update_communication_watchdog(false);
    protections.update_overcurrent_protection(current_pu_t{0.0f});
    protections.update_overvoltage_protection(voltage_pu_t{1.0f});
    protections.update_undervoltage_protection(voltage_pu_t{1.0f});
    protections.resolve_protections(false, true, false);

    EXPECT_TRUE(protections.get_communication_loss_tripped());
    EXPECT_TRUE(protections.get_has_active_trip());
}

TEST(SfcGridSideProtectionsTest, AlarmSeverityDoesNotTripAggregate)
{
    constexpr duration_t ts{200e-6};
    sfc_grid_side_protections protections(ts);

    protections.configure_door_open_is_trip(false);

    protections.reset();

    protections.update_communication_watchdog(true);
    protections.update_overcurrent_protection(current_pu_t{0.0f});
    protections.update_overvoltage_protection(voltage_pu_t{1.0f});
    protections.update_undervoltage_protection(voltage_pu_t{1.0f});
    protections.resolve_protections(true, true, false);

    EXPECT_FALSE(protections.get_has_active_trip());
    EXPECT_FALSE(protections.get_is_inhibited());
    EXPECT_FALSE(protections.get_door_open_tripped());
    EXPECT_TRUE(protections.get_door_open_alarmed());
}

TEST(SfcGridSideProtectionsTest, ChangingSeverityFromTripToAlarmClearsProtectionState)
{
    constexpr duration_t ts{200e-6};
    sfc_grid_side_protections protections(ts);

    // Configure undervoltage as a trip with zero delay for immediate response
    protections.configure_undervoltage_is_trip(true);
    protections.configure_undervoltage_trip_threshold(voltage_pu_t{0.9f});
    protections.configure_undervoltage_trip_delay(duration_t{0.0});  // No delay

    protections.reset();

    // Apply undervoltage condition - should trip immediately
    protections.update_communication_watchdog(true);
    protections.update_overcurrent_protection(current_pu_t{0.5f});
    protections.update_overvoltage_protection(voltage_pu_t{1.0f});
    protections.update_undervoltage_protection(voltage_pu_t{0.8f});  // Below threshold
    protections.resolve_protections(false, true, false);

    EXPECT_TRUE(protections.get_undervoltage_tripped()) << "Should trip immediately with zero delay";
    EXPECT_TRUE(protections.get_has_active_trip());
    EXPECT_TRUE(protections.get_is_inhibited()) << "Inhibit should latch when trip occurs";

    // Now change severity from trip to alarm while fault is still present
    // This should reset the underlying protection block
    protections.configure_undervoltage_is_trip(false);

    // Run another cycle - the protection should no longer contribute to trips
    protections.update_communication_watchdog(true);
    protections.update_overcurrent_protection(current_pu_t{0.5f});
    protections.update_overvoltage_protection(voltage_pu_t{1.0f});
    protections.update_undervoltage_protection(voltage_pu_t{0.8f});  // Still below threshold
    protections.resolve_protections(false, true, false);

    // After changing to alarm severity, the trip should clear
    EXPECT_FALSE(protections.get_undervoltage_tripped()) << "Trip should clear after severity change";
    EXPECT_TRUE(protections.get_undervoltage_alarmed()) << "Should now be in alarm state";
    EXPECT_FALSE(protections.get_has_active_trip()) << "No trips should be active";

    // Because no trips are active, confirm should clear inhibit
    protections.update_communication_watchdog(true);
    protections.update_overcurrent_protection(current_pu_t{0.5f});
    protections.update_overvoltage_protection(voltage_pu_t{1.0f});
    protections.update_undervoltage_protection(voltage_pu_t{0.8f});
    protections.resolve_protections(false, true, true);  // Rising edge of confirm

    EXPECT_FALSE(protections.get_is_inhibited()) << "Inhibit should clear since no trips are active";
}

}  // namespace yeet::application::sfc
