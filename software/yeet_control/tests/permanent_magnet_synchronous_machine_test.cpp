#include "yeet_control/permanent_magnet_synchronous_machine.hpp"

#include <gtest/gtest.h>

#include "yeet_control/angle.hpp"
#include "yeet_control/constants.hpp"
#include "yeet_control/reference_frame.hpp"
#include "yeet_testing/simulator_gtest.hpp"

namespace {

using namespace yeet::control;

class PermanentMagnetSynchronousMachineTest : public ::testing::Test {
public:
    static constexpr double sampling_time = 1e-4;      // 10 kHz sampling rate
    static constexpr double nominal_frequency = 50.0;  // 50 Hz nominal frequency

    void SetUp() override {
        // Configure machine with SI units
        machine.parametrize(0.5,      // Rs: Stator resistance [ohm]
                            0.008,    // Ld: d-axis inductance [H]
                            0.008,    // Lq: q-axis inductance [H] (non-salient in this test)
                            0.9,      // PM flux linkage [Wb]
                            1,        // Pole pairs
                            0.005,    // J: Inertia [kg·m²]
                            0.0001);  // B: Friction [N·m·s/rad]

        register_samplers();
        configure_simulation();
    }

protected:
    permanent_magnet_synchronous_machine machine{sampling_time};
    yeet::testing::simulator simulator = yeet::testing::make_gtest_simulator();

    // Variables to monitor
    dq<double> voltage_command{0.0, 0.0};
    dq<double> stator_current{0.0, 0.0};
    double mechanical_speed{0.0};
    double electromagnetic_torque{0.0};
    double electrical_angle{0.0};
    double time_counter{0.0};

private:
    void register_samplers() {
        // Voltage commands
        simulator.register_sampler_entry(
            yeet::testing::make_reference_sampler("voltage_d", [this]() { return voltage_command.d(); }));
        simulator.register_sampler_entry(
            yeet::testing::make_reference_sampler("voltage_q", [this]() { return voltage_command.q(); }));

        // Stator currents
        simulator.register_sampler_entry(
            yeet::testing::make_reference_sampler("current_d", [this]() { return stator_current.d(); }));
        simulator.register_sampler_entry(
            yeet::testing::make_reference_sampler("current_q", [this]() { return stator_current.q(); }));

        // Speed and torque
        simulator.register_sampler_entry(
            yeet::testing::make_reference_sampler("speed_rad_s", [this]() { return mechanical_speed; }));
        simulator.register_sampler_entry(
            yeet::testing::make_reference_sampler("torque_Nm", [this]() { return electromagnetic_torque; }));

        // Electrical angle
        simulator.register_sampler_entry(yeet::testing::make_reference_sampler(
            "electrical_angle", [this]() { return machine.get_rotor_angle().get_radians(); }));
    }

    void configure_simulation() {
        double ramp_time = 0.5;  // 500ms voltage ramp-up

        // Nominal voltage in SI units (peak phase)
        double nominal_voltage = 325.0;  // 230V RMS = 325V peak

        simulator.register_lambda_runner(sampling_time, [this, ramp_time, nominal_voltage]() {
            // Update time counter
            time_counter += sampling_time;

            // Apply voltage in dq reference frame - align with rotor
            double voltage_magnitude = nominal_voltage;

            // For PMSM, apply voltage primarily in q-axis for optimal torque production
            voltage_command.d() = 0.0;  // No field weakening
            voltage_command.q() = voltage_magnitude;

            // Update machine with voltage command and zero load torque
            machine.update(voltage_command, 0.0);

            // Update monitored signals
            stator_current = machine.get_currents();
            mechanical_speed = machine.get_mechanical_speed_rad_s();
            electromagnetic_torque = machine.get_torque();
            electrical_angle = machine.get_rotor_angle().get_radians();
        });
    }
};

TEST_F(PermanentMagnetSynchronousMachineTest, NominalVoltageTest) {
    // Run simulation for 3 seconds to observe startup transient
    EXPECT_TRUE(simulator.simulate_for(8.0));
}

}  // namespace
