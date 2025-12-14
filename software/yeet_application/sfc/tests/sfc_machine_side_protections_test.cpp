#include "sfc/sfc_machine_side_protections.hpp"

#include <gtest/gtest.h>

namespace yeet::application::sfc {

class SfcMachineSideProtectionsFixture : public ::testing::Test {
protected:
    static constexpr duration_t ts{200e-6};

    sfc_machine_side_protections protections{ts};

    void SetUp() override
    {
        // Configure a deterministic, "no-surprises" baseline for tests.
        protections.configure_over_flux_is_trip(true);
        protections.configure_under_flux_is_trip(true);
        protections.configure_stall_is_trip(true);
        protections.configure_overspeed_is_trip(true);
        protections.configure_machine_overvoltage_is_trip(true);
        protections.configure_communication_loss_is_trip(true);

        protections.configure_over_flux_trip_threshold(flux_pu_t{1.10f});
        protections.configure_under_flux_trip_threshold(flux_pu_t{0.80f});

        protections.configure_stall_current_threshold(current_pu_t{1.3f});
        protections.configure_stall_speed_threshold(speed_pu_t{0.10f});

        protections.configure_comm_timeout_duration(duration_t{50e-3f});
        protections.configure_comm_reset_duration(duration_t{5e-3f});

        protections.configure_overspeed_trip_threshold(speed_pu_t{1.1f});
        protections.configure_overspeed_trip_delay(duration_t{0.0});

        protections.configure_machine_overvoltage_trip_threshold(voltage_pu_t{1.1f});
        protections.configure_machine_overvoltage_trip_delay(duration_t{0.0});

        protections.reset();
    }
};

TEST(SfcMachineSideProtectionsDefaultConfigTest, NominalInputsDoNotTripOrInhibit)
{
    constexpr duration_t ts{200e-6};
    sfc_machine_side_protections protections(ts);
    protections.reset();

    // closed_loop_active=true to enable VHz/overspeed monitoring with nominal values
    protections.update(current_pu_t{0.0f}, voltage_pu_t{1.0f}, frequency_pu_t{1.0f}, speed_pu_t{1.0f}, false, true,
        true, true, true, false, false);

    EXPECT_FALSE(protections.get_is_tripped());
    EXPECT_FALSE(protections.get_is_inhibited());
    EXPECT_FALSE(protections.get_any_fault_active());
}

TEST_F(SfcMachineSideProtectionsFixture, OverspeedTripLatchesAndClearsWithAckThenConfirm)
{
    // Trigger overspeed (closed_loop_active=true to enable overspeed protection)
    protections.update(current_pu_t{0.0f}, voltage_pu_t{1.0f}, frequency_pu_t{1.0f}, speed_pu_t{1.2f}, false, true,
        true, true, true, false, false);

    EXPECT_TRUE(protections.get_overspeed_tripped());
    EXPECT_TRUE(protections.get_is_tripped());
    EXPECT_TRUE(protections.get_is_inhibited());

    // Fault clears but without ack, trip remains (ack_and_fault_clear policy)
    protections.update(current_pu_t{0.0f}, voltage_pu_t{1.0f}, frequency_pu_t{1.0f}, speed_pu_t{1.0f}, false, true,
        true, true, true, false, false);
    EXPECT_TRUE(protections.get_is_tripped());

    // Ack clears trip only if fault condition is cleared
    protections.update(current_pu_t{0.0f}, voltage_pu_t{1.0f}, frequency_pu_t{1.0f}, speed_pu_t{1.0f}, false, true,
        true, true, true, true, false);
    EXPECT_FALSE(protections.get_is_tripped());
    EXPECT_TRUE(protections.get_is_inhibited());

    // Ensure confirm is deasserted for at least one cycle, then apply a rising edge.
    // Keep trip_acknowledged=true throughout to maintain cleared trip state
    protections.update(current_pu_t{0.0f}, voltage_pu_t{1.0f}, frequency_pu_t{1.0f}, speed_pu_t{1.0f}, false, true,
        true, true, true, true, false);

    // Confirm rising edge clears inhibit (requires trip_acknowledged=true && !any_fault_condition_active)
    protections.update(current_pu_t{0.0f}, voltage_pu_t{1.0f}, frequency_pu_t{1.0f}, speed_pu_t{1.0f}, false, true,
        true, true, true, true, true);
    EXPECT_FALSE(protections.get_is_inhibited());
}

TEST_F(SfcMachineSideProtectionsFixture, AlarmSeverityDoesNotTripAggregate)
{
    // Overspeed as ALARM (not trip)
    protections.configure_overspeed_is_trip(false);
    protections.reset();

    // closed_loop_active=true to enable overspeed protection
    protections.update(current_pu_t{0.0f}, voltage_pu_t{1.0f}, frequency_pu_t{1.0f}, speed_pu_t{1.2f}, false, true,
        true, true, true, false, false);

    EXPECT_FALSE(protections.get_is_tripped());
    EXPECT_FALSE(protections.get_is_inhibited());
    EXPECT_FALSE(protections.get_overspeed_tripped());
    EXPECT_TRUE(protections.get_overspeed_alarmed());
}

}  // namespace yeet::application::sfc
