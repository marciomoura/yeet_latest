#include "yeet_control/permanent_magnet_synchronous_machine_per_unit.hpp"

#include <gtest/gtest.h>

#include "yeet_control/angle.hpp"
#include "yeet_control/constants.hpp"
#include "yeet_control/reference_frame.hpp"
#include "yeet_testing/simulator_gtest.hpp"

namespace {

using namespace yeet;
using namespace yeet::control;

class PermanentMagnetSynchronousMachinePerUnitTest : public ::testing::Test {
public:
    static constexpr double sampling_time = 1e-4;  // 10 kHz sampling rate

    void SetUp() override {
        // Configure machine with per-unit values
        // Base values: 400V RMS line voltage, 5kVA (3-phase), 50Hz
        machine = std::make_unique<permanent_magnet_synchronous_machine_per_unit>(
            per_unit_definition::from_rms_line_voltage(400.0,   // 400V RMS line voltage
                                                       5000.0,  // 5kVA 3-phase power
                                                       50.0),   // 50Hz base frequency
            sampling_time);

        // Configure with typical per-unit parameters for PMSM
        machine->parametrize(0.05,   // Rs: Stator resistance [p.u.]
                             0.2,    // Ld: d-axis inductance [p.u.]
                             0.2,    // Lq: q-axis inductance [p.u.] (non-salient)
                             0.9,    // PM flux linkage [p.u.]
                             1,      // Pole pairs
                             0.1,    // J: Inertia [p.u.]
                             0.02);  // B: Friction [p.u.]

        register_samplers();
        configure_simulation();
    }

protected:
    // Components
    std::unique_ptr<permanent_magnet_synchronous_machine_per_unit> machine;
    yeet::testing::simulator simulator = yeet::testing::make_gtest_simulator();

    // Variables to monitor
    dq<double> voltage_command_pu{0.0, 0.0};
    dq<double> stator_current_pu{0.0, 0.0};
    double mechanical_speed_pu{0.0};
    double electromagnetic_torque_pu{0.0};
    double electrical_angle{0.0};
    double time_counter{0.0};

private:
    void register_samplers() {
        // Voltage commands (per-unit)
        simulator.register_sampler_entry(
            yeet::testing::make_reference_sampler("voltage_d_pu", [this]() { return voltage_command_pu.d(); }));
        simulator.register_sampler_entry(
            yeet::testing::make_reference_sampler("voltage_q_pu", [this]() { return voltage_command_pu.q(); }));

        // Stator currents (per-unit)
        simulator.register_sampler_entry(
            yeet::testing::make_reference_sampler("current_d_pu", [this]() { return stator_current_pu.d(); }));
        simulator.register_sampler_entry(
            yeet::testing::make_reference_sampler("current_q_pu", [this]() { return stator_current_pu.q(); }));

        // Speed and torque (per-unit)
        simulator.register_sampler_entry(
            yeet::testing::make_reference_sampler("speed_pu", [this]() { return mechanical_speed_pu; }));
        simulator.register_sampler_entry(
            yeet::testing::make_reference_sampler("torque_pu", [this]() { return electromagnetic_torque_pu; }));

        // Electrical angle
        simulator.register_sampler_entry(yeet::testing::make_reference_sampler(
            "electrical_angle", [this]() { return machine->get_rotor_angle().get_radians(); }));
    }

    void configure_simulation() {
        double ramp_time = 0.5;  // 500ms voltage ramp-up

        simulator.register_lambda_runner(sampling_time, [this, ramp_time]() {
            // Update time counter
            time_counter += sampling_time;

            // For PMSM, apply voltage primarily in q-axis
            voltage_command_pu.d() = 0.0;  // No field weakening
            voltage_command_pu.q() = 1.0;

            // Update machine with voltage command and zero load torque
            machine->update(voltage_command_pu, 0.0);

            // Update monitored signals
            stator_current_pu = machine->get_currents_pu();
            mechanical_speed_pu = machine->get_mechanical_speed_pu();
            electromagnetic_torque_pu = machine->get_torque_pu();
            electrical_angle = machine->get_rotor_angle().get_radians();
        });
    }
};

TEST_F(PermanentMagnetSynchronousMachinePerUnitTest, NominalVoltageTest) {
    // Run simulation for 3 seconds to observe startup transient
    EXPECT_TRUE(simulator.simulate_for(3.0));
}

}  // namespace