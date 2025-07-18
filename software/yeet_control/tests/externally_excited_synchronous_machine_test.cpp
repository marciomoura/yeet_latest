#include "yeet_control/externally_excited_synchronous_machine.hpp"

#include <gtest/gtest.h>

#include "yeet_control/angle.hpp"
#include "yeet_control/constants.hpp"
#include "yeet_control/reference_frame.hpp"
#include "yeet_testing/simulator_gtest.hpp"

namespace {

using namespace yeet;
using namespace yeet::control;

class ExternallyExcitedSynchronousMachineTest : public ::testing::Test {
public:
    static constexpr double sampling_time = 1e-4;      // 10 kHz sampling rate
    static constexpr double nominal_frequency = 50.0;  // 50 Hz nominal frequency

    void SetUp() override {
        // Configure machine with SI units
        machine.parametrize(0.5,    // Rs: Stator resistance [ohm]
                            15.0,   // Rf: Field resistance [ohm]
                            0.008,  // Ld: d-axis inductance [H]
                            0.008,  // Lq: q-axis inductance [H]
                            0.155,  // Lf: Field inductance [H]
                            0.135,  // Lmd: Mutual inductance [H]
                            1,      // Pole pairs
                            0.05,   // J: Inertia [kg·m²]
                            0.01);  // B: Friction [N·m·s/rad]

        register_samplers();
        configure_simulation();
    }

protected:
    externally_excited_synchronous_machine machine{sampling_time};
    yeet::testing::simulator simulator = yeet::testing::make_gtest_simulator();

    // Variables to monitor
    dq<double> stator_voltage_command{0.0, 0.0};
    double field_voltage{0.0};
    dq<double> stator_current{0.0, 0.0};
    double field_current{0.0};
    double mechanical_speed{0.0};
    double electromagnetic_torque{0.0};
    double electrical_angle{0.0};
    double time_counter{0.0};

private:
    void register_samplers() {
        // Voltage commands
        simulator.register_sampler_entry(
            yeet::testing::make_reference_sampler("voltage_d", [this]() { return stator_voltage_command.d(); }));
        simulator.register_sampler_entry(
            yeet::testing::make_reference_sampler("voltage_q", [this]() { return stator_voltage_command.q(); }));
        simulator.register_sampler_entry(
            yeet::testing::make_reference_sampler("field_voltage", [this]() { return field_voltage; }));

        // Stator currents
        simulator.register_sampler_entry(
            yeet::testing::make_reference_sampler("current_d", [this]() { return stator_current.d(); }));
        simulator.register_sampler_entry(
            yeet::testing::make_reference_sampler("current_q", [this]() { return stator_current.q(); }));
        simulator.register_sampler_entry(
            yeet::testing::make_reference_sampler("field_current", [this]() { return field_current; }));

        // Speed and torque
        simulator.register_sampler_entry(
            yeet::testing::make_reference_sampler("speed_rad_s", [this]() { return mechanical_speed; }));
        simulator.register_sampler_entry(yeet::testing::make_reference_sampler(
            "speed_rpm", [this]() { return mechanical_speed * 60.0 / (2 * pi); }));
        simulator.register_sampler_entry(
            yeet::testing::make_reference_sampler("torque_Nm", [this]() { return electromagnetic_torque; }));

        // Electrical angle
        simulator.register_sampler_entry(yeet::testing::make_reference_sampler(
            "electrical_angle", [this]() { return machine.get_rotor_angle().get_radians(); }));
    }

    void configure_simulation() {
        double ramp_time = 0.5;  // 500ms voltage ramp-up

        // Nominal voltage in SI units (peak phase)
        double nominal_stator_voltage = 325.0;  // 230V RMS = 325V peak
        double nominal_field_voltage = 100.0;   // Field excitation voltage

        simulator.register_lambda_runner(sampling_time,
                                         [this, ramp_time, nominal_stator_voltage, nominal_field_voltage]() {
                                             // Update time counter
                                             time_counter += sampling_time;

                                             // Smooth ramp for stator voltage
                                             double voltage_ramp_factor = std::min(1.0, time_counter / ramp_time);

                                             // Apply stator voltage in dq reference frame - align with rotor
                                             // For externally excited SM, d-axis aligns with field winding, so
                                             // we apply voltage primarily in q-axis for torque production
                                             stator_voltage_command.d() = 0.0;  // No d-axis voltage for simplicity
                                             stator_voltage_command.q() = voltage_ramp_factor * nominal_stator_voltage;

                                             // Apply field voltage (can also be ramped if desired)
                                             field_voltage = nominal_field_voltage;

                                             // Update machine with voltage commands and zero load torque
                                             machine.update(stator_voltage_command, field_voltage, 0.0);

                                             // Update monitored signals
                                             stator_current = machine.get_stator_currents();
                                             field_current = machine.get_field_current();
                                             mechanical_speed = machine.get_mechanical_speed_rad_s();
                                             electromagnetic_torque = machine.get_torque();
                                             electrical_angle = machine.get_rotor_angle().get_radians();
                                         });
    }
};

TEST_F(ExternallyExcitedSynchronousMachineTest, NominalVoltageTest) {
    // Run simulation for 8 seconds to observe startup and steady-state
    EXPECT_TRUE(simulator.simulate_for(8.0));

    // Calculate synchronous speed for reference
    double sync_speed_rad_s = 2 * pi * nominal_frequency / machine.get_pole_pairs();
    double sync_speed_rpm = sync_speed_rad_s * 60.0 / (2 * pi);

    // Output final values (without assertions)
    std::cout << "=== Externally Excited Synchronous Machine SI Test Results ===" << std::endl;
    std::cout << "Final speed: " << mechanical_speed << " rad/s (" << (mechanical_speed * 60.0 / (2 * pi)) << " RPM)"
              << std::endl;
    std::cout << "Synchronous speed: " << sync_speed_rad_s << " rad/s (" << sync_speed_rpm << " RPM)" << std::endl;
    std::cout << "Final torque: " << electromagnetic_torque << " N·m" << std::endl;
    std::cout << "Field current: " << field_current << " A" << std::endl;

    // Calculate stator current magnitude
    double stator_current_magnitude = std::sqrt(std::pow(stator_current.d(), 2) + std::pow(stator_current.q(), 2));
    std::cout << "Stator current magnitude: " << stator_current_magnitude << " A" << std::endl;

    // Calculate field flux linkage
    double field_flux = machine.get_field_flux_linkage();
    std::cout << "Field flux linkage: " << field_flux << " Wb" << std::endl;
}

}  // namespace