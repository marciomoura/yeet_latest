#include "sfc/sfc_protections_trip_handshake.hpp"

#include <gtest/gtest.h>

namespace yeet::application::sfc {

TEST(SfcProtectionsTripHandshakeTest, StartsCleared)
{
    sfc_protections_trip_handshake handshake;
    handshake.reset();

    EXPECT_FALSE(handshake.get_has_active_trip());
    EXPECT_FALSE(handshake.get_is_inhibited());
}

TEST(SfcProtectionsTripHandshakeTest, TripLatchesAndRequiresConfirmation)
{
    sfc_protections_trip_handshake handshake;
    handshake.reset();

    // Trip occurs
    handshake.update(true, false);
    EXPECT_TRUE(handshake.get_has_active_trip());
    EXPECT_TRUE(handshake.get_is_inhibited());

    // Fault clears -> has_active_trip clears, but is_inhibited remains latched (requires confirmation)
    handshake.update(false, false);
    EXPECT_FALSE(handshake.get_has_active_trip());
    EXPECT_TRUE(handshake.get_is_inhibited());

    // Rising edge confirm clears inhibit
    handshake.update(false, true);
    EXPECT_FALSE(handshake.get_has_active_trip());
    EXPECT_FALSE(handshake.get_is_inhibited());
}

TEST(SfcProtectionsTripHandshakeTest, ConfirmationDoesNotClearIfFaultActive)
{
    sfc_protections_trip_handshake handshake;
    handshake.reset();

    // Trip occurs
    handshake.update(true, false);
    EXPECT_TRUE(handshake.get_has_active_trip());
    EXPECT_TRUE(handshake.get_is_inhibited());

    // Rising edge confirm while fault is still active must NOT clear inhibit
    handshake.update(true, true);
    EXPECT_TRUE(handshake.get_has_active_trip());
    EXPECT_TRUE(handshake.get_is_inhibited());

    // Fault clears
    handshake.update(false, false);
    EXPECT_FALSE(handshake.get_has_active_trip());
    EXPECT_TRUE(handshake.get_is_inhibited());

    // New rising edge confirm clears everything
    handshake.update(false, true);
    EXPECT_FALSE(handshake.get_has_active_trip());
    EXPECT_FALSE(handshake.get_is_inhibited());
}

}  // namespace yeet::application::sfc
