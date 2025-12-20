#include <gtest/gtest.h>

#include "sfc_integration/sfc_machine_side_protections_runner.hpp"
#include "yeet/signal.hpp"

using namespace yeet;
using namespace yeet::application::sfc::integration;

namespace {

class MachineSideProtectionsRunnerUnitTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        dut.initialize();

        // Ensure all inputs have backing storage and deterministic initial values.
        mock_signal_input::write(dut.get_inputs().dc_link_current_pu, current_pu_t{0.0f});
        mock_signal_input::write(dut.get_inputs().machine_voltage_magnitude_pu, voltage_pu_t{0.0f});
        mock_signal_input::write(dut.get_inputs().machine_frequency, frequency_t{0.0f});
        mock_signal_input::write(dut.get_inputs().machine_frequency_pu, frequency_pu_t{0.0f});
        mock_signal_input::write(dut.get_inputs().machine_speed, speed_pu_t{0.0f});
        mock_signal_input::write(dut.get_inputs().startup_active, false);
        mock_signal_input::write(dut.get_inputs().closed_loop_active, true);
        mock_signal_input::write(dut.get_inputs().release_protections, true);
        mock_signal_input::write(dut.get_inputs().excitation_is_on, true);

        auto& p = dut.get_parameters();
        p.protection_module_enabled.write(true);

        // Disable non-target protections to keep this test focused.
        p.machine_overvoltage_trip_threshold.write(voltage_pu_t{10.0f});
        p.machine_overvoltage_trip_delay.write(duration_t{0.0});

        p.over_flux_trip_threshold.write(flux_pu_t{10.0f});
        p.over_flux_trip_delay.write(duration_t{0.0});

        // Disable under-flux: at low speed V/Hz reports 0 pu.
        p.under_flux_trip_threshold.write(flux_pu_t{-10.0f});
        p.under_flux_trip_delay.write(duration_t{0.0});

        p.stall_current_threshold.write(current_pu_t{10.0f});
        p.stall_speed_threshold.write(speed_pu_t{0.0f});
        p.stall_trip_delay.write(duration_t{0.0});

        // Ensure confirm is low.
        p.confirm_alarms.write(false);

        // One cycle to apply initial config.
        step(5);
    }

    void step(std::size_t n)
    {
        for (std::size_t i = 0; i < n; ++i) {
            dut.run();
        }
    }

    void pulse_confirm_alarms()
    {
        auto& p = dut.get_parameters();
        p.confirm_alarms.write(false);
        step(10);
        p.confirm_alarms.write(true);
        step(10);
        p.confirm_alarms.write(false);
        step(10);
    }

    sfc_machine_side_protections_runner dut;
};

TEST_F(MachineSideProtectionsRunnerUnitTest, OverspeedTripCanBeClearedByAcknowledgeThenConfirm)
{
    auto& p = dut.get_parameters();
    p.overspeed_trip_threshold.write(speed_pu_t{0.90f});
    p.overspeed_trip_delay.write(duration_t{0.0});

    mock_signal_input::write(dut.get_inputs().machine_speed, speed_pu_t{1.0f});
    step(5);

    EXPECT_TRUE(dut.get_outputs().overspeed_tripped.read());
    EXPECT_TRUE(dut.get_outputs().is_tripped.read());
    EXPECT_TRUE(dut.get_outputs().is_inhibited.read());

    // Clear the fault condition first.
    mock_signal_input::write(dut.get_inputs().machine_speed, speed_pu_t{0.0f});
    step(20);

    // Confirm alarms
    p.confirm_alarms.write(false);
    step(5);
    p.confirm_alarms.write(true);
    step(10);
    p.confirm_alarms.write(false);
    step(10);

    EXPECT_FALSE(dut.get_outputs().any_fault_active.read())
        << "any_fault_active should be false after overspeed clears and is acknowledged";
    EXPECT_FALSE(dut.get_outputs().overspeed_tripped.read()) << "overspeed_tripped should clear after reset";
    EXPECT_FALSE(dut.get_outputs().is_tripped.read()) << "trip latch should clear after acknowledge";
    EXPECT_FALSE(dut.get_outputs().is_inhibited.read()) << "inhibit should clear after confirm";
}

}  // namespace
