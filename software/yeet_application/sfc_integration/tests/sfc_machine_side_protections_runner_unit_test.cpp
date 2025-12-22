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

        // Connect storage to signals (required by raw pointer API)
        yeet::utils::mock_signal_input::connect(dut.get_inputs().dc_link_current_pu, _storage_dc_link_current);
        yeet::utils::mock_signal_input::connect(
            dut.get_inputs().machine_voltage_magnitude_pu, _storage_machine_voltage);
        yeet::utils::mock_signal_input::connect(dut.get_inputs().machine_frequency, _storage_machine_frequency);
        yeet::utils::mock_signal_input::connect(dut.get_inputs().machine_frequency_pu, _storage_machine_frequency_pu);
        yeet::utils::mock_signal_input::connect(dut.get_inputs().machine_speed, _storage_machine_speed);
        yeet::utils::mock_signal_input::connect(dut.get_inputs().startup_active, _storage_startup_active);
        yeet::utils::mock_signal_input::connect(dut.get_inputs().closed_loop_active, _storage_closed_loop_active);
        yeet::utils::mock_signal_input::connect(dut.get_inputs().release_protections, _storage_release_protections);
        yeet::utils::mock_signal_input::connect(dut.get_inputs().excitation_is_on, _storage_excitation_is_on);
        yeet::utils::mock_signal_input::connect(dut.get_inputs().external_fault, _storage_external_fault);

        // Ensure all inputs have backing storage and deterministic initial values.
        yeet::utils::mock_signal_input::write(dut.get_inputs().dc_link_current_pu, current_pu_t{0.0f});
        yeet::utils::mock_signal_input::write(dut.get_inputs().machine_voltage_magnitude_pu, voltage_pu_t{0.0f});
        yeet::utils::mock_signal_input::write(dut.get_inputs().machine_frequency, frequency_t{0.0f});
        yeet::utils::mock_signal_input::write(dut.get_inputs().machine_frequency_pu, frequency_pu_t{0.0f});
        yeet::utils::mock_signal_input::write(dut.get_inputs().machine_speed, speed_pu_t{0.0f});
        yeet::utils::mock_signal_input::write(dut.get_inputs().startup_active, false);
        yeet::utils::mock_signal_input::write(dut.get_inputs().closed_loop_active, true);
        yeet::utils::mock_signal_input::write(dut.get_inputs().release_protections, true);
        yeet::utils::mock_signal_input::write(dut.get_inputs().excitation_is_on, true);
        yeet::utils::mock_signal_input::write(dut.get_inputs().external_fault, false);

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

    // Storage for mocked signals (required by raw pointer API)
    current_pu_t _storage_dc_link_current{0.0f};
    voltage_pu_t _storage_machine_voltage{0.0f};
    frequency_t _storage_machine_frequency{0.0f};
    frequency_pu_t _storage_machine_frequency_pu{0.0f};
    speed_pu_t _storage_machine_speed{0.0f};
    bool _storage_startup_active{false};
    bool _storage_closed_loop_active{true};
    bool _storage_release_protections{true};
    bool _storage_excitation_is_on{true};
    bool _storage_external_fault{false};
};

TEST_F(MachineSideProtectionsRunnerUnitTest, OverspeedTripCanBeClearedByAcknowledgeThenConfirm)
{
    auto& p = dut.get_parameters();
    p.overspeed_trip_threshold.write(speed_pu_t{0.90f});
    p.overspeed_trip_delay.write(duration_t{0.0});

    yeet::utils::mock_signal_input::write(dut.get_inputs().machine_speed, speed_pu_t{1.0f});
    step(5);

    EXPECT_TRUE(dut.get_outputs().overspeed_tripped.read());
    EXPECT_TRUE(dut.get_outputs().is_tripped.read());
    EXPECT_TRUE(dut.get_outputs().is_inhibited.read());

    // Clear the fault condition first.
    yeet::utils::mock_signal_input::write(dut.get_inputs().machine_speed, speed_pu_t{0.0f});
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
