#include "sfc/sfc_machine_side_protections.hpp"

#include <gtest/gtest.h>

namespace yeet::application::sfc {

class SfcMachineSideProtectionsFixture : public ::testing::Test {
protected:
    static constexpr duration_t ts{200e-6};

    sfc_machine_side_protections protections{ts};

    void update_all(current_pu_t dc_link_current_pu,
        voltage_pu_t machine_voltage_magnitude_pu,
        frequency_pu_t machine_frequency_pu,
        speed_pu_t machine_speed_pu,
        bool startup_active,
        bool closed_loop_active,
        bool module_enabled,
        bool release_protections,
        bool communication_valid,
        bool excitation_is_on,
        bool confirm_alarms)
    {
        if (module_enabled) {
            protections.update_communication_watchdog(communication_valid);
            protections.update_vhz_protection(machine_voltage_magnitude_pu, machine_frequency_pu, closed_loop_active);
            protections.update_stall_protection(dc_link_current_pu, machine_speed_pu, startup_active);
            protections.update_overspeed_protection(machine_speed_pu, closed_loop_active);
            protections.update_machine_overvoltage_protection(machine_voltage_magnitude_pu);
            protections.update_loss_of_excitation_protection(excitation_is_on, startup_active, closed_loop_active);
        }
        protections.resolve_protections(module_enabled, confirm_alarms);
    }

    void SetUp() override
    {
        // Configure a deterministic, "no-surprises" baseline for tests.
        protections.configure_over_flux_is_trip(true);
        protections.configure_under_flux_is_trip(true);
        protections.configure_stall_is_trip(true);
        protections.configure_overspeed_is_trip(true);
        protections.configure_machine_overvoltage_is_trip(true);
        protections.configure_communication_loss_is_trip(true);
        protections.configure_loss_of_excitation_is_trip(true);

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

        protections.configure_loss_of_excitation_trip_delay(duration_t{0.0});

        protections.reset();
    }
};

TEST(SfcMachineSideProtectionsDefaultConfigTest, NominalInputsDoNotTripOrInhibit)
{
    constexpr duration_t ts{200e-6};
    sfc_machine_side_protections protections(ts);
    protections.reset();

    // closed_loop_active=true to enable VHz/overspeed monitoring with nominal values
    // excitation_is_on=true for nominal operation
    protections.update_communication_watchdog(true);
    protections.update_vhz_protection(voltage_pu_t{1.0f}, frequency_pu_t{1.0f}, true);
    protections.update_stall_protection(current_pu_t{0.0f}, speed_pu_t{1.0f}, false);
    protections.update_overspeed_protection(speed_pu_t{1.0f}, true);
    protections.update_machine_overvoltage_protection(voltage_pu_t{1.0f});
    protections.update_loss_of_excitation_protection(true, false, true);
    protections.resolve_protections(true, false);

    EXPECT_FALSE(protections.get_is_tripped());
    EXPECT_FALSE(protections.get_is_inhibited());
    EXPECT_FALSE(protections.get_any_fault_active());
}

TEST_F(SfcMachineSideProtectionsFixture, OverspeedTripLatchesAndClearsAutomatically)
{
    // Trigger overspeed (closed_loop_active=true to enable overspeed protection)
    update_all(current_pu_t{0.0f}, voltage_pu_t{1.0f}, frequency_pu_t{1.0f}, speed_pu_t{1.2f}, false, true, true, true,
        true, true, false);

    EXPECT_TRUE(protections.get_overspeed_tripped());
    EXPECT_TRUE(protections.get_is_tripped());
    EXPECT_TRUE(protections.get_is_inhibited());

    // Fault clears automatically with auto_clear policy
    update_all(current_pu_t{0.0f}, voltage_pu_t{1.0f}, frequency_pu_t{1.0f}, speed_pu_t{1.0f}, false, true, true, true,
        true, true, false);
    EXPECT_FALSE(protections.get_is_tripped());
    EXPECT_TRUE(protections.get_is_inhibited());

    // Ensure confirm is deasserted for at least one cycle, then apply a rising edge.
    update_all(current_pu_t{0.0f}, voltage_pu_t{1.0f}, frequency_pu_t{1.0f}, speed_pu_t{1.0f}, false, true, true, true,
        true, true, false);

    // Confirm rising edge clears inhibit
    update_all(current_pu_t{0.0f}, voltage_pu_t{1.0f}, frequency_pu_t{1.0f}, speed_pu_t{1.0f}, false, true, true, true,
        true, true, true);
    EXPECT_FALSE(protections.get_is_inhibited());
}

TEST_F(SfcMachineSideProtectionsFixture, AlarmSeverityDoesNotTripAggregate)
{
    // Overspeed as ALARM (not trip)
    protections.configure_overspeed_is_trip(false);
    protections.reset();

    // closed_loop_active=true to enable overspeed protection
    update_all(current_pu_t{0.0f}, voltage_pu_t{1.0f}, frequency_pu_t{1.0f}, speed_pu_t{1.2f}, false, true, true, true,
        true, true, false);

    EXPECT_FALSE(protections.get_is_tripped());
    EXPECT_FALSE(protections.get_is_inhibited());
    EXPECT_FALSE(protections.get_overspeed_tripped());
    EXPECT_TRUE(protections.get_overspeed_alarmed());
}

TEST_F(SfcMachineSideProtectionsFixture, LossOfExcitationTripLatchesAndClearsAutomatically)
{
    // Normal operation with excitation ON
    update_all(current_pu_t{0.0f}, voltage_pu_t{1.0f}, frequency_pu_t{1.0f}, speed_pu_t{1.0f}, true, false, true, true,
        true, true, false);

    EXPECT_FALSE(protections.get_loss_of_excitation_tripped());
    EXPECT_FALSE(protections.get_is_tripped());

    // Loss of excitation during startup (startup_active=true)
    update_all(current_pu_t{0.5f}, voltage_pu_t{1.0f}, frequency_pu_t{1.0f}, speed_pu_t{0.5f}, true, false, true, true,
        true, false, false);

    EXPECT_TRUE(protections.get_loss_of_excitation_tripped());
    EXPECT_TRUE(protections.get_is_tripped());
    EXPECT_TRUE(protections.get_is_inhibited());

    // Excitation restored - trip clears automatically with auto_clear policy
    update_all(current_pu_t{0.5f}, voltage_pu_t{1.0f}, frequency_pu_t{1.0f}, speed_pu_t{0.5f}, true, false, true, true,
        true, true, false);
    EXPECT_FALSE(protections.get_is_tripped());
    EXPECT_TRUE(protections.get_is_inhibited());

    // Ensure confirm is deasserted, then apply rising edge
    update_all(current_pu_t{0.5f}, voltage_pu_t{1.0f}, frequency_pu_t{1.0f}, speed_pu_t{0.5f}, true, false, true, true,
        true, true, false);

    // Confirm clears inhibit
    update_all(current_pu_t{0.5f}, voltage_pu_t{1.0f}, frequency_pu_t{1.0f}, speed_pu_t{0.5f}, true, false, true, true,
        true, true, true);
    EXPECT_FALSE(protections.get_is_inhibited());
}

TEST_F(SfcMachineSideProtectionsFixture, LossOfExcitationOnlyActivesDuringStartupOrClosedLoop)
{
    // Loss of excitation before startup should NOT trip (not active)
    update_all(current_pu_t{0.0f}, voltage_pu_t{1.0f}, frequency_pu_t{1.0f}, speed_pu_t{0.0f}, false, false, true, true,
        true, false, false);

    EXPECT_FALSE(protections.get_loss_of_excitation_tripped());
    EXPECT_FALSE(protections.get_is_tripped());

    // Loss of excitation during closed-loop operation SHOULD trip
    update_all(current_pu_t{1.0f}, voltage_pu_t{1.0f}, frequency_pu_t{1.0f}, speed_pu_t{1.0f}, false, true, true, true,
        true, false, false);

    EXPECT_TRUE(protections.get_loss_of_excitation_tripped());
    EXPECT_TRUE(protections.get_is_tripped());
}

TEST_F(SfcMachineSideProtectionsFixture, LossOfExcitationAlarmSeverity)
{
    // Configure loss of excitation as alarm (not trip)
    protections.configure_loss_of_excitation_is_trip(false);
    protections.reset();

    // Loss of excitation during startup with alarm severity
    update_all(current_pu_t{0.5f}, voltage_pu_t{1.0f}, frequency_pu_t{1.0f}, speed_pu_t{0.5f}, true, false, true, true,
        true, false, false);

    EXPECT_FALSE(protections.get_is_tripped());
    EXPECT_FALSE(protections.get_is_inhibited());
    EXPECT_FALSE(protections.get_loss_of_excitation_tripped());
    EXPECT_TRUE(protections.get_loss_of_excitation_alarmed());
}

TEST_F(SfcMachineSideProtectionsFixture, LossOfExcitationTripDuringStartup)
{
    // Trigger loss of excitation during startup (startup_active=true)
    // excitation_is_on=false triggers the protection
    update_all(current_pu_t{0.5f}, voltage_pu_t{0.8f}, frequency_pu_t{0.5f}, speed_pu_t{0.3f}, true, false, true, true,
        true, false, false);

    EXPECT_TRUE(protections.get_loss_of_excitation_tripped());
    EXPECT_TRUE(protections.get_is_tripped());
    EXPECT_TRUE(protections.get_is_inhibited());
    EXPECT_TRUE(protections.get_any_fault_active());
}

TEST_F(SfcMachineSideProtectionsFixture, LossOfExcitationTripDuringClosedLoop)
{
    // Trigger loss of excitation during closed-loop operation (closed_loop_active=true)
    // excitation_is_on=false triggers the protection
    update_all(current_pu_t{0.8f}, voltage_pu_t{1.0f}, frequency_pu_t{1.0f}, speed_pu_t{1.0f}, false, true, true, true,
        true, false, false);

    EXPECT_TRUE(protections.get_loss_of_excitation_tripped());
    EXPECT_TRUE(protections.get_is_tripped());
    EXPECT_TRUE(protections.get_is_inhibited());
    EXPECT_TRUE(protections.get_any_fault_active());
}

TEST_F(SfcMachineSideProtectionsFixture, LossOfExcitationNoTripWhenInactive)
{
    // Loss of excitation should NOT trip when neither startup nor closed_loop is active
    // Even with excitation_is_on=false, protection should not be active
    update_all(current_pu_t{0.0f}, voltage_pu_t{0.5f}, frequency_pu_t{0.5f}, speed_pu_t{0.0f}, false, false, true, true,
        true, false, false);

    EXPECT_FALSE(protections.get_loss_of_excitation_tripped());
    EXPECT_FALSE(protections.get_is_tripped());
    EXPECT_FALSE(protections.get_is_inhibited());
}

TEST_F(SfcMachineSideProtectionsFixture, LossOfExcitationAlarmSeverityClosedLoop)
{
    // Configure loss of excitation as alarm (not trip)
    protections.configure_loss_of_excitation_is_trip(false);
    protections.reset();

    // Trigger loss of excitation during closed-loop (excitation_is_on=false)
    update_all(current_pu_t{0.8f}, voltage_pu_t{1.0f}, frequency_pu_t{1.0f}, speed_pu_t{1.0f}, false, true, true, true,
        true, false, false);

    EXPECT_FALSE(protections.get_is_tripped());
    EXPECT_FALSE(protections.get_is_inhibited());
    EXPECT_FALSE(protections.get_loss_of_excitation_tripped());
    EXPECT_TRUE(protections.get_loss_of_excitation_alarmed());
}

}  // namespace yeet::application::sfc
