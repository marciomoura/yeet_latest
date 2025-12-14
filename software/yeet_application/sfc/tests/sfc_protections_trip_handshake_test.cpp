#include "sfc/sfc_protections_trip_handshake.hpp"

#include <gtest/gtest.h>

namespace yeet::application::sfc {

TEST(SfcProtectionsTripHandshakeTest, StartsCleared)
{
    sfc_protections_trip_handshake handshake;
    handshake.configure_trip_clear_policy(sfc_trip_clear_policy::ack_only);
    handshake.reset();

    EXPECT_FALSE(handshake.get_is_tripped());
    EXPECT_FALSE(handshake.get_is_inhibited());
    EXPECT_FALSE(handshake.get_trip_cleared_pulse());
}

TEST(SfcProtectionsTripHandshakeTest, TripLatchesAndRequiresAckToClear_AckOnly)
{
    sfc_protections_trip_handshake handshake;
    handshake.configure_trip_clear_policy(sfc_trip_clear_policy::ack_only);
    handshake.reset();

    // Trip occurs
    handshake.update(true, true, false, false);
    EXPECT_TRUE(handshake.get_is_tripped());
    EXPECT_TRUE(handshake.get_is_inhibited());

    // Fault clears, but no ack -> still tripped
    handshake.update(false, false, false, false);
    EXPECT_TRUE(handshake.get_is_tripped());
    EXPECT_TRUE(handshake.get_is_inhibited());

    // Ack -> clears trip (even if fault were still active in ack_only policy)
    handshake.update(false, true, true, false);
    EXPECT_FALSE(handshake.get_is_tripped());
    EXPECT_TRUE(handshake.get_is_inhibited());
    EXPECT_TRUE(handshake.get_trip_cleared_pulse());

    // Pulse is one-cycle
    handshake.update(false, false, false, false);
    EXPECT_FALSE(handshake.get_trip_cleared_pulse());
}

TEST(SfcProtectionsTripHandshakeTest, AckAndFaultClearPolicyRequiresFaultClear)
{
    sfc_protections_trip_handshake handshake;
    handshake.configure_trip_clear_policy(sfc_trip_clear_policy::ack_and_fault_clear);
    handshake.reset();

    // Trip occurs
    handshake.update(true, true, false, false);
    EXPECT_TRUE(handshake.get_is_tripped());
    EXPECT_TRUE(handshake.get_is_inhibited());

    // Ack while fault still active -> must NOT clear
    handshake.update(false, true, true, false);
    EXPECT_TRUE(handshake.get_is_tripped());
    EXPECT_TRUE(handshake.get_is_inhibited());

    // Fault clears and ack true -> clears
    handshake.update(false, false, true, false);
    EXPECT_FALSE(handshake.get_is_tripped());
    EXPECT_TRUE(handshake.get_is_inhibited());
    EXPECT_TRUE(handshake.get_trip_cleared_pulse());
}

TEST(SfcProtectionsTripHandshakeTest, ConfirmAlarmsClearsInhibitOnlyAfterTripClears)
{
    sfc_protections_trip_handshake handshake;
    handshake.configure_trip_clear_policy(sfc_trip_clear_policy::ack_only);
    handshake.reset();

    // Trip occurs
    handshake.update(true, true, false, false);
    EXPECT_TRUE(handshake.get_is_tripped());
    EXPECT_TRUE(handshake.get_is_inhibited());

    // Rising edge confirm while still tripped must NOT clear inhibit
    handshake.update(true, true, false, true);
    EXPECT_TRUE(handshake.get_is_tripped());
    EXPECT_TRUE(handshake.get_is_inhibited());

    // Clear trip via ack
    handshake.update(false, true, true, false);
    EXPECT_FALSE(handshake.get_is_tripped());
    EXPECT_TRUE(handshake.get_is_inhibited());

    // Rising edge confirm clears inhibit
    handshake.update(false, false, false, true);
    EXPECT_FALSE(handshake.get_is_tripped());
    EXPECT_FALSE(handshake.get_is_inhibited());

    // Holding confirm high does not re-trigger
    handshake.update(false, false, false, true);
    EXPECT_FALSE(handshake.get_is_inhibited());

    // Another rising edge after going low works again
    handshake.update(true, true, false, false);
    handshake.update(false, false, true, false);
    EXPECT_TRUE(handshake.get_is_inhibited());
    handshake.update(false, false, false, true);
    EXPECT_FALSE(handshake.get_is_inhibited());
}

}  // namespace yeet::application::sfc
