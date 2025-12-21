#include "sfc/sfc_protections_trip_handshake.hpp"

#include <gtest/gtest.h>

namespace yeet::application::sfc {

TEST(SfcProtectionsTripHandshakeTest, StartsCleared)
{
    sfc_protections_trip_handshake handshake;
    handshake.reset();

    EXPECT_FALSE(handshake.get_is_tripped());
    EXPECT_FALSE(handshake.get_is_inhibited());
    EXPECT_FALSE(handshake.get_trip_cleared_pulse());
}

TEST(SfcProtectionsTripHandshakeTest, TripLatchesAndRequiresConfirmation)
{
    sfc_protections_trip_handshake handshake;
    handshake.reset();

    // Trip occurs
    handshake.update(true, true, false);
    EXPECT_TRUE(handshake.get_is_tripped());
    EXPECT_TRUE(handshake.get_is_inhibited());

    // Fault clears -> trip remains latched (requires confirmation)
    handshake.update(false, false, false);
    EXPECT_TRUE(handshake.get_is_tripped());
    EXPECT_TRUE(handshake.get_is_inhibited());
    EXPECT_FALSE(handshake.get_trip_cleared_pulse());

    // Rising edge confirm clears trip and inhibit
    handshake.update(false, false, true);
    EXPECT_FALSE(handshake.get_is_tripped());
    EXPECT_FALSE(handshake.get_is_inhibited());
    EXPECT_TRUE(handshake.get_trip_cleared_pulse());

    // Pulse is one-cycle
    handshake.update(false, false, false);
    EXPECT_FALSE(handshake.get_trip_cleared_pulse());
}

TEST(SfcProtectionsTripHandshakeTest, ConfirmationDoesNotClearIfFaultActive)
{
    sfc_protections_trip_handshake handshake;
    handshake.reset();

    // Trip occurs
    handshake.update(true, true, false);
    EXPECT_TRUE(handshake.get_is_tripped());
    EXPECT_TRUE(handshake.get_is_inhibited());

    // Rising edge confirm while fault is still active must NOT clear trip or inhibit
    handshake.update(false, true, true);
    EXPECT_TRUE(handshake.get_is_tripped());
    EXPECT_TRUE(handshake.get_is_inhibited());

    // Fault clears
    handshake.update(false, false, false);
    EXPECT_TRUE(handshake.get_is_tripped());

    // New rising edge confirm clears everything
    handshake.update(false, false, true);
    EXPECT_FALSE(handshake.get_is_tripped());
    EXPECT_FALSE(handshake.get_is_inhibited());
}

}  // namespace yeet::application::sfc
