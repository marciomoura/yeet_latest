#include <gtest/gtest.h>

#include "sfc_integration/sfc_machine_side_protections_runner.hpp"
#include "yeet/signal.hpp"
#include "yeet/signal_wiring.hpp"

using namespace yeet;
using namespace yeet::application::sfc::integration;

namespace {

class MachineSideProtectionsRunnerUnitTest : public ::testing::Test {
protected:
    struct mock_storage {
        current_pu_t dc_link_current_pu{0.0f};
        voltage_pu_t machine_voltage_magnitude_pu{0.0f};
        frequency_t machine_frequency{0.0f};
        frequency_pu_t machine_frequency_pu{0.0f};
        speed_pu_t machine_speed{0.0f};
        bool startup_active{false};
        bool closed_loop_active{true};
        bool release_protections{true};
        bool excitation_is_on{true};
    } _storage;

    void SetUp() override
    {
        dut.initialize();

        // Wire mock inputs to storage
        yeet::wire(dut)
            .mocked(_storage.dc_link_current_pu)
            .to(&sfc_machine_side_protections_datamodel::inputs::dc_link_current_pu)
            .mocked(_storage.machine_voltage_magnitude_pu)
            .to(&sfc_machine_side_protections_datamodel::inputs::machine_voltage_magnitude_pu)
            .mocked(_storage.machine_frequency)
            .to(&sfc_machine_side_protections_datamodel::inputs::machine_frequency)
            .mocked(_storage.machine_frequency_pu)
            .to(&sfc_machine_side_protections_datamodel::inputs::machine_frequency_pu)
            .mocked(_storage.machine_speed)
            .to(&sfc_machine_side_protections_datamodel::inputs::machine_speed)
            .mocked(_storage.startup_active)
            .to(&sfc_machine_side_protections_datamodel::inputs::startup_active)
            .mocked(_storage.closed_loop_active)
            .to(&sfc_machine_side_protections_datamodel::inputs::closed_loop_active)
            .mocked(_storage.release_protections)
            .to(&sfc_machine_side_protections_datamodel::inputs::release_protections)
            .mocked(_storage.excitation_is_on)
            .to(&sfc_machine_side_protections_datamodel::inputs::excitation_is_on);

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

    _storage.machine_speed = speed_pu_t{1.0f};
    step(5);

    EXPECT_TRUE(dut.get_outputs().overspeed_tripped.read());
    EXPECT_TRUE(dut.get_outputs().is_tripped.read());
    EXPECT_TRUE(dut.get_outputs().is_inhibited.read());

    // Clear the fault condition first.
    _storage.machine_speed = speed_pu_t{0.0f};
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
