#include "sfc/sfc_grid_side_protections.hpp"

#include <gtest/gtest.h>

namespace yeet::application::sfc {

TEST(SfcGridSideProtectionsTest, DoorOpenTripClearsWithAckThenConfirm)
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
    protections.update(current_pu_t{0.0f}, voltage_pu_t{1.0f}, true, true, true, true, false, false);

    EXPECT_TRUE(protections.get_door_open_tripped());
    EXPECT_TRUE(protections.get_is_tripped());
    EXPECT_TRUE(protections.get_is_inhibited());

    // Close door + acknowledge clears trip (grid-side policy = ack_only)
    protections.update(current_pu_t{0.0f}, voltage_pu_t{1.0f}, false, true, true, true, true, false);

    EXPECT_FALSE(protections.get_is_tripped());
    EXPECT_TRUE(protections.get_is_inhibited());

    // Confirm rising edge clears inhibit
    protections.update(current_pu_t{0.0f}, voltage_pu_t{1.0f}, false, true, true, true, false, true);

    EXPECT_FALSE(protections.get_is_inhibited());
}

TEST(SfcGridSideProtectionsTest, CommunicationWatchdogRunsEvenWhenReleaseIsFalse)
{
    constexpr duration_t ts{200e-6};
    sfc_grid_side_protections protections(ts);

    protections.configure_communication_loss_is_trip(true);
    protections.configure_comm_timeout_duration(ts);  // 1 cycle
    protections.configure_comm_reset_duration(ts);

    protections.reset();

    // Module enabled but release_protections is false -> watchdog should still run and trip.
    protections.update(current_pu_t{0.0f}, voltage_pu_t{1.0f}, false, true, false, false, false, false);

    EXPECT_TRUE(protections.get_communication_loss_tripped());
    EXPECT_TRUE(protections.get_is_tripped());
}

TEST(SfcGridSideProtectionsTest, AlarmSeverityDoesNotTripAggregate)
{
    constexpr duration_t ts{200e-6};
    sfc_grid_side_protections protections(ts);

    protections.configure_door_open_is_trip(false);

    protections.reset();

    protections.update(current_pu_t{0.0f}, voltage_pu_t{1.0f}, true, true, true, true, false, false);

    EXPECT_FALSE(protections.get_is_tripped());
    EXPECT_FALSE(protections.get_is_inhibited());
    EXPECT_FALSE(protections.get_door_open_tripped());
    EXPECT_TRUE(protections.get_door_open_alarmed());
}

}  // namespace yeet::application::sfc
