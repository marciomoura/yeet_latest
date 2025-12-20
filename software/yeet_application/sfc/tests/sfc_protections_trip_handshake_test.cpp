#include "sfc/sfc_protections_trip_handshake.hpp"

#include <gtest/gtest.h>

namespace yeet::application::sfc {

TEST(SfcProtectionsTripHandshakeTest, StartsCleared)
{
    sfc_protections_trip_handshake handshake;
    handshake.configure_trip_clear_policy(sfc_trip_clear_policy::auto_clear);
    handshake.reset();

    EXPECT_FALSE(handshake.get_is_tripped());
    EXPECT_FALSE(handshake.get_is_inhibited());
    EXPECT_FALSE(handshake.get_trip_cleared_pulse());
}

TEST(SfcProtectionsTripHandshakeTest, TripAutoClearsWhenFaultClears_AutoClear)
{
    sfc_protections_trip_handshake handshake;
    handshake.configure_trip_clear_policy(sfc_trip_clear_policy::auto_clear);
    handshake.reset();

    // Trip occurs
    handshake.update(true, true, false);
    EXPECT_TRUE(handshake.get_is_tripped());
    EXPECT_TRUE(handshake.get_is_inhibited());

    // Fault clears -> trip auto-clears
    handshake.update(false, false, false);
    EXPECT_FALSE(handshake.get_is_tripped());
    EXPECT_TRUE(handshake.get_is_inhibited());
    EXPECT_TRUE(handshake.get_trip_cleared_pulse());

    // Pulse is one-cycle
    handshake.update(false, false, false);
    EXPECT_FALSE(handshake.get_trip_cleared_pulse());
}

TEST(SfcProtectionsTripHandshakeTest, ConfirmAlarmsClearsInhibitOnlyAfterTripClears)
{
    sfc_protections_trip_handshake handshake;
    handshake.configure_trip_clear_policy(sfc_trip_clear_policy::auto_clear);
    handshake.reset();

    // Trip occurs
    handshake.update(true, true, false);
    EXPECT_TRUE(handshake.get_is_tripped());
    EXPECT_TRUE(handshake.get_is_inhibited());

    // Rising edge confirm while still tripped must NOT clear inhibit
    handshake.update(true, true, true);
    EXPECT_TRUE(handshake.get_is_tripped());
    EXPECT_TRUE(handshake.get_is_inhibited());

    // Fault clears -> trip auto-clears
    handshake.update(false, false, false);
    EXPECT_FALSE(handshake.get_is_tripped());
    EXPECT_TRUE(handshake.get_is_inhibited());

    // Rising edge confirm clears inhibit
    handshake.update(false, false, true);
    EXPECT_FALSE(handshake.get_is_tripped());
    EXPECT_FALSE(handshake.get_is_inhibited());

    // Holding confirm high does not re-trigger
    handshake.update(false, false, true);
    EXPECT_FALSE(handshake.get_is_inhibited());

    // Another rising edge after going low works again
    handshake.update(true, true, false);
    handshake.update(false, false, false);
    EXPECT_TRUE(handshake.get_is_inhibited());
    handshake.update(false, false, true);
    EXPECT_FALSE(handshake.get_is_inhibited());
}

}  // namespace yeet::application::sfc
