#include "yeet_control/assynchronous_machine.hpp"

#include <gtest/gtest.h>

#include "yeet_control/constants.hpp"
#include "yeet_control/reference_frame.hpp"
#include "yeet_testing/simulator_gtest.hpp"

namespace {

using namespace yeet::control;

class AssynchronousMachineOpenLoopTest : public ::testing::Test {
public:
    static constexpr double sampling_time = 1e-4;     // 10 kHz sampling rate
    static constexpr double stator_frequency = 50.0;  // 50 Hz stator frequency

    void SetUp() override {
        // Typical SI values for a small induction motor
        machine.parametrize(0.5,     // Rs: Stator resistance [ohm]
                            0.4,     // Rr: Rotor resistance [ohm]
                            0.05,    // Ls: Stator inductance [H]
                            0.05,    // Lr: Rotor inductance [H]
                            0.045,   // Lm: Mutual inductance [H]
                            2,       // Pole pairs
                            0.01,    // J: Inertia [kg·m²]
                            0.001);  // B: Friction [N·m·s/rad]

        register_samplers();
        configure_simulation();
    }

protected:
    // Components
    assynchronous_machine machine{sampling_time};
    yeet::testing::simulator simulator = yeet::testing::make_gtest_simulator();

    // Variables to monitor
    alphabeta<double> voltage_command{0.0, 0.0};
    alphabeta<double> stator_current{0.0, 0.0};
    double omega_m{0.0};
    double electromagnetic_torque{0.0};
    double electrical_angle{0.0};
    double time_counter{0.0};

private:
    void register_samplers() {
        // Voltage commands
        simulator.register_sampler_entry(
            yeet::testing::make_reference_sampler("voltage_alpha", [this]() { return voltage_command.alpha(); }));
        simulator.register_sampler_entry(
            yeet::testing::make_reference_sampler("voltage_beta", [this]() { return voltage_command.beta(); }));

        // Stator currents
        simulator.register_sampler_entry(
            yeet::testing::make_reference_sampler("current_alpha", [this]() { return stator_current.alpha(); }));
        simulator.register_sampler_entry(
            yeet::testing::make_reference_sampler("current_beta", [this]() { return stator_current.beta(); }));

        // Rotor fluxes
        simulator.register_sampler_entry(yeet::testing::make_reference_sampler(
            "flux_r_alpha", [this]() { return machine.get_rotor_flux().alpha(); }));
        simulator.register_sampler_entry(
            yeet::testing::make_reference_sampler("flux_r_beta", [this]() { return machine.get_rotor_flux().beta(); }));

        // Rotor fluxes
        simulator.register_sampler_entry(yeet::testing::make_reference_sampler(
            "flux_s_alpha", [this]() { return machine.get_stator_flux().alpha(); }));
        simulator.register_sampler_entry(yeet::testing::make_reference_sampler(
            "flux_s_beta", [this]() { return machine.get_stator_flux().beta(); }));

        // Speed and torque
        simulator.register_sampler_entry(yeet::testing::make_reference_sampler(
            "speed_rad_s", [this]() { return machine.get_mechanical_speed_rad_s(); }));
        simulator.register_sampler_entry(yeet::testing::make_reference_sampler(
            "speed_rpm", [this]() { return machine.get_mechanical_speed_rad_s() * 60.0 / (2 * pi<double>); }));
        simulator.register_sampler_entry(
            yeet::testing::make_reference_sampler("torque_Nm", [this]() { return machine.get_torque(); }));

        // Electrical angle
        simulator.register_sampler_entry(yeet::testing::make_reference_sampler(
            "electrical_angle", [this]() { return machine.get_stator_angle().get_radians(); }));
    }

    void configure_simulation() {
        double ramp_time = 0.5;  // 500ms frequency ramp-up

        // Nominal voltage and frequency values
        double nominal_voltage = 220.0 * std::sqrt(2.0 / 3.0);  // Peak phase voltage
        double nominal_freq = stator_frequency;                 // 50 Hz

        // V/f ratio for constant flux
        double v_to_f_ratio = nominal_voltage / nominal_freq;

        // Voltage boost for low frequencies to compensate for stator resistance
        double voltage_boost = 0.1 * nominal_voltage;  // 10% boost

        simulator.register_lambda_runner(
            sampling_time, [this, ramp_time, v_to_f_ratio, voltage_boost, nominal_voltage]() {
                // Update time counter
                time_counter += sampling_time;

                // Ramp frequency from min frequency to nominal
                double min_freq = 3.0;  // Minimum 3 Hz to avoid issues at very low freq
                double freq_range = stator_frequency - min_freq;

                // Calculate target frequency with smooth ramp
                double frequency;
                if (time_counter < ramp_time) {
                    // Smooth S-curve ramp for frequency
                    double ramp_factor = time_counter / ramp_time;
                    // Use smooth S-curve: 3*t² - 2*t³ (smoother than linear ramp)
                    double s_curve = 3.0 * std::pow(ramp_factor, 2) - 2.0 * std::pow(ramp_factor, 3);
                    frequency = min_freq + freq_range * s_curve;
                } else {
                    frequency = stator_frequency;  // Final frequency
                }

                double omega_s = 2.0 * pi<double> * frequency;

                // Calculate voltage to maintain constant flux (V/f with boost)
                double v_magnitude;
                if (frequency < 10.0) {
                    // More boost at very low frequencies
                    v_magnitude = frequency * v_to_f_ratio + voltage_boost * (1.0 - frequency / 10.0);
                } else {
                    // Constant V/f ratio above 10Hz
                    v_magnitude = frequency * v_to_f_ratio;
                }

                // Limit voltage to nominal value
                v_magnitude = std::min(v_magnitude, nominal_voltage);

                // Calculate alpha-beta voltage components
                voltage_command.alpha() = v_magnitude * std::cos(omega_s * time_counter);
                voltage_command.beta() = v_magnitude * std::sin(omega_s * time_counter);

                // Update machine with voltage command and zero load torque
                machine.update(voltage_command, 0.0);

                // Update monitored signals
                stator_current = machine.get_stator_current();
                omega_m = machine.get_mechanical_speed_rad_s();
                electromagnetic_torque = machine.get_torque();
                electrical_angle = machine.get_stator_angle().get_radians();
            });
    }
};

TEST_F(AssynchronousMachineOpenLoopTest, StartupTransient) {
    // Run simulation for 3 seconds to observe startup transient
    EXPECT_TRUE(simulator.simulate_for(3.0));

    // Check if machine has accelerated to near-synchronous speed
    double sync_speed = 2 * pi<double> * stator_frequency / 2;  // Mechanical speed for 2 pole pairs
    EXPECT_GT(machine.get_mechanical_speed_rad_s(), 0.9 * sync_speed);

    // Output final values
    std::cout << "Final speed: "
              << machine.get_mechanical_speed_rad_s() * 60.0 / (2 * pi<double>) << " RPM (synchronous: "
              << sync_speed * 60.0 / (2 * pi<double>) << " RPM)" << std::endl;
    std::cout << "Final torque: " << machine.get_torque() << " N·m" << std::endl;
    std::cout << "Stator current magnitude: "
              << std::sqrt(std::pow(stator_current.alpha(), 2) + std::pow(stator_current.beta(), 2)) << " A"
              << std::endl;
}

}  // namespace