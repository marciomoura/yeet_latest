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
    EXPECT_TRUE(protections.get_is_tripped());
    EXPECT_TRUE(protections.get_is_inhibited());

    // Close door - trip clears automatically with auto_clear policy
    protections.update_communication_watchdog(true);
    protections.update_overcurrent_protection(current_pu_t{0.0f});
    protections.update_overvoltage_protection(voltage_pu_t{1.0f});
    protections.update_undervoltage_protection(voltage_pu_t{1.0f});
    protections.resolve_protections(false, true, false);

    EXPECT_TRUE(protections.get_is_tripped());
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
    EXPECT_TRUE(protections.get_is_tripped());
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

    EXPECT_FALSE(protections.get_is_tripped());
    EXPECT_FALSE(protections.get_is_inhibited());
    EXPECT_FALSE(protections.get_door_open_tripped());
    EXPECT_TRUE(protections.get_door_open_alarmed());
}

}  // namespace yeet::application::sfc
