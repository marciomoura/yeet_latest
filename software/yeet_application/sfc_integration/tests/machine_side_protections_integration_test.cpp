#include <gtest/gtest.h>

#include "sfc_integration/sfc_machine_side_protections_runner.hpp"
#include "sfc_integration/sfc_state_machine_runner.hpp"
#include "yeet/signal.hpp"
#include "yeet/signal_wiring.hpp"
#include "yeet_testing/simulator.hpp"

using namespace yeet::application::sfc;
using namespace yeet::application::sfc::integration;
using namespace yeet;

namespace {

class MachineSideProtectionsIntegrationTest : public ::testing::Test {
protected:
    struct mock_storage {
        current_pu_t dc_link_current_pu;
        voltage_pu_t machine_voltage_magnitude_pu;
        frequency_t machine_frequency;
        frequency_pu_t machine_frequency_pu;
        speed_pu_t machine_speed;
        bool release_protections;
        bool startup_active;
        bool closed_loop_active;
        bool excitation_is_on;

        bool auxiliaries_are_on;
        bool mv_breaker_is_closed;
        bool start_command;
        bool stop_command;
        bool shutdown_request;
        bool mv_breaker_is_tripping;
        bool emergency_off;
        bool door_is_open;
        bool external_fault;
        bool protection_inhibited;
        bool machine_side_shutdown_request;
        bool any_protection_fault_active;
        bool is_current_extinguished;
    };

    static inline mock_storage _storage{};

    void SetUp() override
    {
        simulator.configure_sampler_sampling_rate(yeet::ms(10.0f));

        machine_protections.initialize();
        state_machine.initialize();

        // Connect mock storage using signal wiring DSL
        yeet::wire(machine_protections)
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
            .mocked(_storage.release_protections)
            .to(&sfc_machine_side_protections_datamodel::inputs::release_protections)
            .mocked(_storage.startup_active)
            .to(&sfc_machine_side_protections_datamodel::inputs::startup_active)
            .mocked(_storage.closed_loop_active)
            .to(&sfc_machine_side_protections_datamodel::inputs::closed_loop_active)
            .mocked(_storage.excitation_is_on)
            .to(&sfc_machine_side_protections_datamodel::inputs::excitation_is_on);

        yeet::wire(state_machine)
            .mocked(_storage.auxiliaries_are_on)
            .to(&sfc_state_machine_datamodel::inputs::auxiliaries_are_on)
            .mocked(_storage.mv_breaker_is_closed)
            .to(&sfc_state_machine_datamodel::inputs::mv_breaker_is_closed)
            .mocked(_storage.start_command)
            .to(&sfc_state_machine_datamodel::inputs::start_command)
            .mocked(_storage.stop_command)
            .to(&sfc_state_machine_datamodel::inputs::stop_command)
            .mocked(_storage.shutdown_request)
            .to(&sfc_state_machine_datamodel::inputs::shutdown_request)
            .mocked(_storage.mv_breaker_is_tripping)
            .to(&sfc_state_machine_datamodel::inputs::mv_breaker_is_tripping)
            .mocked(_storage.emergency_off)
            .to(&sfc_state_machine_datamodel::inputs::emergency_off)
            .mocked(_storage.door_is_open)
            .to(&sfc_state_machine_datamodel::inputs::door_is_open)
            .mocked(_storage.external_fault)
            .to(&sfc_state_machine_datamodel::inputs::external_fault)
            .mocked(_storage.protection_inhibited)
            .to(&sfc_state_machine_datamodel::inputs::protection_inhibited)
            .mocked(_storage.machine_side_shutdown_request)
            .to(&sfc_state_machine_datamodel::inputs::machine_side_shutdown_request)
            .mocked(_storage.any_protection_fault_active)
            .to(&sfc_state_machine_datamodel::inputs::any_protection_fault_active)
            .mocked(_storage.is_current_extinguished)
            .to(&sfc_state_machine_datamodel::inputs::is_current_extinguished);

        // Connect machine-side protections trip into the grid-side state machine input.
        // This is the integration contract: any machine-side trip must force grid-side control shutdown.
        yeet::signals_connector::connect_output_to_input(
            machine_protections.get_outputs().is_tripped, state_machine.get_inputs().machine_side_protection_tripped);

        // Also wire inhibit into the state machine, mirroring production behavior.
        yeet::signals_connector::connect_output_to_input(machine_protections.get_outputs().is_inhibited,
            state_machine.get_inputs().machine_side_protection_inhibited);

        // Register runners in simulator.
        simulator.register_runner(machine_protections);
        simulator.register_runner(state_machine);

        // Fixed state-machine inputs for all tests (no other trips/inhibits requested).
        _storage.auxiliaries_are_on = true;
        _storage.mv_breaker_is_closed = true;
        _storage.start_command = false;
        _storage.stop_command = false;
        _storage.shutdown_request = false;
        _storage.mv_breaker_is_tripping = false;
        _storage.external_fault = false;
        _storage.emergency_off = false;
        _storage.door_is_open = false;
        _storage.protection_inhibited = false;
        _storage.machine_side_shutdown_request = false;
        _storage.any_protection_fault_active = false;
        _storage.is_current_extinguished = true;

        simulator.initialize();
    }

    void simulate_for(double seconds) { simulator.simulate_for(seconds); }

    void enter_operation_on()
    {
        // First, set turn_on_command to transition from IDLE to AUXILIARIES_ON
        state_machine.get_parameters().turn_on_command.write(true);
        simulate_for(0.05);

        // Then set start_command to transition through BREAKERS_OFF -> BREAKERS_ON -> OPERATION_ON
        _storage.start_command = true;
        simulate_for(0.05);

        const auto& sm_out = state_machine.get_outputs();
        EXPECT_FALSE(sm_out.mv_breaker_open.read());
        EXPECT_TRUE(sm_out.auxiliaries_on_command.read());
    }

    void clear_trip_and_confirm_alarms()
    {
        // The simulator sampling rate is 10ms in this fixture.
        // Use durations > 10ms to guarantee at least one simulation tick per step.
        constexpr double tick = 0.02;

        // With simplified handshake, trip auto-clears when fault clears (no trip_acknowledged needed).
        // We just need to confirm alarms to clear the inhibit latch.
        // Note: Fault condition must already be cleared before calling this function.
        simulate_for(tick);

        // Confirm alarms: rising edge on confirm_alarms clears inhibit when no faults active
        auto& params = machine_protections.get_parameters();
        params.confirm_alarms.write(false);
        simulate_for(tick);
        params.confirm_alarms.write(true);
        simulate_for(tick);
        params.confirm_alarms.write(false);
        simulate_for(tick);

        // After a protection trip, the IDLE state has a trip latch that requires
        // turn_on_command to be released before allowing restart.
        state_machine.get_parameters().turn_on_command.write(false);
        simulate_for(tick);
    }

    void restore_safe_protection_parameter_defaults()
    {
        auto& params = machine_protections.get_parameters();

        // Keep non-target protections from tripping unless a test sets them.
        params.machine_overvoltage_trip_threshold.write(voltage_pu_t{1.50f});
        params.machine_overvoltage_trip_delay.write(duration_t{0.0});

        params.overspeed_trip_threshold.write(speed_pu_t{1.50f});
        params.overspeed_trip_delay.write(duration_t{0.0});

        params.over_flux_trip_threshold.write(flux_pu_t{10.0f});
        params.over_flux_trip_delay.write(duration_t{0.0});
        // Disable under-flux in tests: V/Hz protection outputs 0 pu at very low speed,
        // and an under-flux trip threshold of 0.0 would otherwise trip and stay asserted.
        params.under_flux_trip_threshold.write(flux_pu_t{-10.0f});
        params.under_flux_trip_delay.write(duration_t{0.0});

        params.stall_current_threshold.write(current_pu_t{10.0f});
        params.stall_speed_threshold.write(speed_pu_t{0.0f});
        params.stall_trip_delay.write(duration_t{0.0});

        // Be explicit about severities for test determinism.
        params.over_flux_severity.write(protection_severity::trip);
        params.under_flux_severity.write(protection_severity::trip);
        params.stall_severity.write(protection_severity::trip);
        params.overspeed_severity.write(protection_severity::trip);
        params.machine_overvoltage_severity.write(protection_severity::trip);
        params.communication_loss_severity.write(protection_severity::trip);
    }

    void configure_machine_side_protections_for_test()
    {
        auto& params = machine_protections.get_parameters();
        params.protection_module_enabled.write(true);

        // Always release protections in these tests.
        _storage.release_protections = true;
        _storage.startup_active = false;

        restore_safe_protection_parameter_defaults();

        // Leave comm watchdog defaults.
    }

    void expect_trip_shutdown() const
    {
        const auto& sm_out = state_machine.get_outputs();
        EXPECT_TRUE(sm_out.mv_breaker_open.read()) << "Trip should command MV Breaker open";
        EXPECT_FALSE(sm_out.auxiliaries_on_command.read()) << "Trip should command auxiliaries off";
    }

    void expect_no_individual_protection_trips() const
    {
        const auto& out = machine_protections.get_outputs();
        EXPECT_FALSE(out.over_flux_tripped.read());
        EXPECT_FALSE(out.under_flux_tripped.read());
        EXPECT_FALSE(out.stall_tripped.read());
        EXPECT_FALSE(out.communication_loss_tripped.read());
        EXPECT_FALSE(out.overspeed_tripped.read());
        EXPECT_FALSE(out.machine_overvoltage_tripped.read());
    }

    yeet::simulator simulator;
    sfc_machine_side_protections_runner machine_protections;
    sfc_state_machine_runner state_machine;
};

TEST_F(MachineSideProtectionsIntegrationTest, OverspeedTripsControl)
{
    configure_machine_side_protections_for_test();

    // Overspeed protection requires closed_loop_active to be true
    _storage.closed_loop_active = true;
    _storage.machine_speed = speed_pu_t{1.0f};

    auto& p = machine_protections.get_parameters();
    p.overspeed_trip_threshold.write(speed_pu_t{0.90f});
    p.overspeed_trip_delay.write(duration_t{0.0});

    simulate_for(0.2);

    EXPECT_TRUE(machine_protections.get_outputs().is_tripped.read())
        << "Machine-side protections should trip (overspeed)";

    simulate_for(0.2);
    expect_trip_shutdown();
}

TEST_F(MachineSideProtectionsIntegrationTest, MachineOvervoltageTripsControl)
{
    configure_machine_side_protections_for_test();

    _storage.machine_voltage_magnitude_pu = voltage_pu_t{1.0f};

    // Trip at 0.9 pu while operating near 1.0 pu voltage magnitude.
    auto& p = machine_protections.get_parameters();
    p.machine_overvoltage_trip_threshold.write(voltage_pu_t{0.90f});
    p.machine_overvoltage_trip_delay.write(duration_t{0.0});

    simulate_for(0.2);

    EXPECT_TRUE(machine_protections.get_outputs().is_tripped.read())
        << "Machine-side protections should trip (machine overvoltage)";

    simulate_for(0.2);
    expect_trip_shutdown();
}

TEST_F(MachineSideProtectionsIntegrationTest, VhzOverFluxTripsControl)
{
    configure_machine_side_protections_for_test();

    _storage.machine_voltage_magnitude_pu = voltage_pu_t{1.0f};
    _storage.machine_frequency_pu = frequency_pu_t{1.0f};
    _storage.machine_speed = speed_pu_t{1.0f};

    // With V≈1.0 pu and speed≈1.0 pu, flux_pu ≈ V_pu/speed_pu ≈ 1.0.
    // Trip at 0.9 pu => deterministic trip.
    auto& p = machine_protections.get_parameters();
    p.over_flux_trip_threshold.write(flux_pu_t{0.90f});
    p.over_flux_trip_delay.write(duration_t{0.0});

    simulate_for(0.2);

    EXPECT_TRUE(machine_protections.get_outputs().is_tripped.read())
        << "Machine-side protections should trip (V/Hz over-flux)";

    simulate_for(0.2);
    expect_trip_shutdown();
}

TEST_F(MachineSideProtectionsIntegrationTest, StallTripsControlWhenUnableToAccelerate)
{
    configure_machine_side_protections_for_test();

    // For determinism, force startup_active high here (even though production wiring comes from startup sequencer).
    _storage.startup_active = true;
    _storage.dc_link_current_pu = current_pu_t{1.0f};
    _storage.machine_speed = speed_pu_t{0.0f};

    // Stall protection is state-dependent on startup_active.
    auto& p = machine_protections.get_parameters();
    p.stall_current_threshold.write(current_pu_t{0.90f});
    p.stall_speed_threshold.write(speed_pu_t{0.10f});
    p.stall_trip_delay.write(duration_t{0.0});

    simulate_for(0.2);

    EXPECT_TRUE(machine_protections.get_outputs().is_tripped.read()) << "Machine-side protections should trip (stall)";

    simulate_for(0.2);
    expect_trip_shutdown();
}

TEST_F(MachineSideProtectionsIntegrationTest, SequentialTripsCanBeClearedAndRetried)
{
    configure_machine_side_protections_for_test();
    enter_operation_on();

    // ---- 1) Trip on machine overvoltage ----
    _storage.machine_voltage_magnitude_pu = voltage_pu_t{1.0f};
    {
        auto& p = machine_protections.get_parameters();
        p.machine_overvoltage_trip_threshold.write(voltage_pu_t{0.90f});
        p.machine_overvoltage_trip_delay.write(duration_t{0.0});
    }

    simulate_for(0.05);
    ASSERT_TRUE(machine_protections.get_outputs().is_tripped.read());
    expect_trip_shutdown();

    // Clear fault condition first
    _storage.machine_voltage_magnitude_pu = voltage_pu_t{0.0f};
    simulate_for(0.01);

    clear_trip_and_confirm_alarms();
    EXPECT_FALSE(machine_protections.get_outputs().any_fault_active.read())
        << "Fault condition should be cleared before re-start. "
        << "DEBUG: speed_pu=" << machine_protections.get_inputs().machine_speed.read().value() << ", "
        << "overspeed_trip_threshold=" << machine_protections.get_parameters().overspeed_trip_threshold.read().value()
        << ", voltage_pu=" << machine_protections.get_inputs().machine_voltage_magnitude_pu.read().value() << ", "
        << "machine_overvoltage_trip_threshold="
        << machine_protections.get_parameters().machine_overvoltage_trip_threshold.read().value() << ", "
        << "dc_link_current_pu=" << machine_protections.get_inputs().dc_link_current_pu.read().value() << ", "
        << "startup_active=" << machine_protections.get_inputs().startup_active.read() << ", "
        << "release_protections=" << machine_protections.get_inputs().release_protections.read() << ", "
        << "under_flux_trip_threshold=" << machine_protections.get_parameters().under_flux_trip_threshold.read().value()
        << ", "
        << "over_flux_trip_threshold=" << machine_protections.get_parameters().over_flux_trip_threshold.read().value();
    expect_no_individual_protection_trips();
    EXPECT_FALSE(machine_protections.get_outputs().is_tripped.read());
    EXPECT_FALSE(machine_protections.get_outputs().is_inhibited.read());

    // Prevent phase-1 settings from influencing subsequent phases.
    restore_safe_protection_parameter_defaults();

    enter_operation_on();

    // ---- 2) Trip on overspeed ----
    _storage.machine_speed = speed_pu_t{1.0f};
    {
        auto& p = machine_protections.get_parameters();
        p.overspeed_trip_threshold.write(speed_pu_t{0.90f});
        p.overspeed_trip_delay.write(duration_t{0.0});
    }

    simulate_for(0.05);
    ASSERT_TRUE(machine_protections.get_outputs().is_tripped.read());
    expect_trip_shutdown();

    // Clear fault condition
    _storage.machine_speed = speed_pu_t{0.0f};
    simulate_for(0.01);

    EXPECT_FLOAT_EQ(machine_protections.get_inputs().machine_speed.read().value(), 0.0f);
    EXPECT_TRUE(machine_protections.get_inputs().release_protections.read());
    EXPECT_FLOAT_EQ(machine_protections.get_parameters().overspeed_trip_threshold.read().value(), 0.90f);
    EXPECT_FLOAT_EQ(machine_protections.get_parameters().under_flux_trip_threshold.read().value(), -10.0f);
    EXPECT_FLOAT_EQ(machine_protections.get_parameters().under_flux_trip_delay.read().value(), 0.0f);

    clear_trip_and_confirm_alarms();
    EXPECT_FALSE(machine_protections.get_outputs().any_fault_active.read())
        << "Fault condition should be cleared before re-start";
    expect_no_individual_protection_trips();
    EXPECT_FALSE(machine_protections.get_outputs().is_tripped.read());
    EXPECT_FALSE(machine_protections.get_outputs().is_inhibited.read());

    // Prevent phase-2 settings from influencing subsequent phases.
    restore_safe_protection_parameter_defaults();

    enter_operation_on();

    // ---- 3) Trip on stall ----
    _storage.startup_active = true;
    _storage.dc_link_current_pu = current_pu_t{1.0f};
    _storage.machine_speed = speed_pu_t{0.0f};
    {
        auto& p = machine_protections.get_parameters();
        p.stall_current_threshold.write(current_pu_t{0.90f});
        p.stall_speed_threshold.write(speed_pu_t{0.10f});
        p.stall_trip_delay.write(duration_t{0.0});
    }

    simulate_for(0.05);
    ASSERT_TRUE(machine_protections.get_outputs().is_tripped.read());
    expect_trip_shutdown();

    // Clear fault condition
    // Restore defaults before switching signals used to clear stall.
    restore_safe_protection_parameter_defaults();
    _storage.startup_active = false;
    _storage.dc_link_current_pu = current_pu_t{0.0f};
    _storage.machine_speed = speed_pu_t{1.0f};
    simulate_for(0.01);

    clear_trip_and_confirm_alarms();
    EXPECT_FALSE(machine_protections.get_outputs().any_fault_active.read())
        << "Fault condition should be cleared before re-start";
    expect_no_individual_protection_trips();
    EXPECT_FALSE(machine_protections.get_outputs().is_tripped.read());
    EXPECT_FALSE(machine_protections.get_outputs().is_inhibited.read());

    enter_operation_on();
}

}  // namespace
