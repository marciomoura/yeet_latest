#include "yeet_control/assynchronous_machine_per_unit.hpp"

#include <gtest/gtest.h>

#include "yeet_control/angle.hpp"
#include "yeet_control/constants.hpp"
#include "yeet_control/reference_frame.hpp"
#include "yeet_testing/simulator_gtest.hpp"

namespace {

using namespace yeet::control;

class AssynchronousMachinePerUnitOpenLoopTest : public ::testing::Test {
public:
    static constexpr double sampling_time = 1e-4;     // 10 kHz sampling rate
    static constexpr double stator_frequency = 50.0;  // 50 Hz stator frequency

    double nominal_electrical_speed = 2 * pi<double> * stator_frequency;  // Nominal electrical speed [rad/s]

    void SetUp() override {
        // Configure machine with per-unit values
        // Base values: 220V RMS line voltage (180V peak phase), 5kVA (3-phase), 50Hz
        machine = std::make_unique<assynchronous_machine_per_unit>(
            per_unit_definition::from_rms_line_voltage(400.0,            // 400V RMS line voltage
                                                       5000.0,           // 5kVA 3-phase power
                                                       stator_frequency  // 50Hz base frequency
                                                       ),
            sampling_time);

        // Configure with typical per-unit parameters for induction motor
        machine->parametrize(0.05,   // Rs: Stator resistance [p.u.]
                             0.05,   // Rr: Rotor resistance [p.u.]
                             1.5,    // Ls: Stator inductance [p.u.]
                             1.5,    // Lr: Rotor inductance [p.u.]
                             1.4,    // Lm: Mutual inductance [p.u.] (leakage factor ≈ 0.12)
                             1,      // Pole pairs
                             0.1,    // J: Inertia [p.u.] (higher value for smoother dynamics)
                             0.01);  // B: Friction [p.u.]

        register_samplers();
        configure_simulation();
    }

protected:
    // Components
    std::unique_ptr<assynchronous_machine_per_unit> machine;
    yeet::testing::simulator simulator = yeet::testing::make_gtest_simulator();

    // Variables to monitor
    alphabeta<double> voltage_command_pu{0.0, 0.0};
    alphabeta<double> stator_current_pu{0.0, 0.0};
    double omega_m_pu{0.0};
    double electromagnetic_torque_pu{0.0};
    double electrical_angle{0.0};
    double time_counter{0.0};

private:
    void register_samplers() {
        // Voltage commands (per-unit)
        simulator.register_sampler_entry(
            yeet::testing::make_reference_sampler("voltage_alpha_pu", [this]() { return voltage_command_pu.alpha(); }));
        simulator.register_sampler_entry(
            yeet::testing::make_reference_sampler("voltage_beta_pu", [this]() { return voltage_command_pu.beta(); }));

        // Stator currents (per-unit)
        simulator.register_sampler_entry(
            yeet::testing::make_reference_sampler("current_alpha_pu", [this]() { return stator_current_pu.alpha(); }));
        simulator.register_sampler_entry(
            yeet::testing::make_reference_sampler("current_beta_pu", [this]() { return stator_current_pu.beta(); }));

        // Rotor fluxes (per-unit)
        simulator.register_sampler_entry(yeet::testing::make_reference_sampler(
            "flux_r_alpha_pu", [this]() { return machine->get_rotor_flux_pu().alpha(); }));

        simulator.register_sampler_entry(yeet::testing::make_reference_sampler(
            "flux_r_beta_pu", [this]() { return machine->get_rotor_flux_pu().beta(); }));

        // Stator fluxes (per-unit)
        simulator.register_sampler_entry(yeet::testing::make_reference_sampler(
            "flux_s_alpha_pu", [this]() { return machine->get_stator_flux_pu().alpha(); }));

        simulator.register_sampler_entry(yeet::testing::make_reference_sampler(
            "flux_s_beta_pu", [this]() { return machine->get_stator_flux_pu().beta(); }));

        // Speed and torque (per-unit and physical)
        simulator.register_sampler_entry(
            yeet::testing::make_reference_sampler("speed_pu", [this]() { return machine->get_mechanical_speed_pu(); }));

        simulator.register_sampler_entry(
            yeet::testing::make_reference_sampler("torque_pu", [this]() { return machine->get_torque_pu(); }));

        // Electrical angle
        simulator.register_sampler_entry(yeet::testing::make_reference_sampler(
            "electrical_angle", [this]() { return machine->get_stator_angle().get_radians(); }));
    }

    void configure_simulation() {
        double ramp_time = 0.5;  // 500ms frequency ramp-up

        // In per-unit, nominal frequency = 1.0, nominal voltage = 1.0
        double nominal_freq_pu = 1.0;
        double nominal_voltage_pu = 1.0;

        // V/f ratio for constant flux (in per-unit, this is simply 1.0)
        double v_to_f_ratio_pu = nominal_voltage_pu / nominal_freq_pu;

        // Voltage boost for low frequencies to compensate for stator resistance
        double voltage_boost_pu = 0.1;  // 10% boost

        simulator.register_lambda_runner(sampling_time, [this, ramp_time, v_to_f_ratio_pu, voltage_boost_pu]() {
            // Update time counter
            time_counter += sampling_time;

            // Calculate alpha-beta voltage components in per-unit
            voltage_command_pu = yeet::control::make_alphabeta<double>(
                1.0, angle<zero_to_two_pi>{50.0 * 2.0 * pi<double> * time_counter});

            // Update machine with per-unit voltage command and zero load torque
            machine->update(voltage_command_pu, 0.0);

            // Update monitored signals
            stator_current_pu = machine->get_stator_current_pu();
            omega_m_pu = machine->get_mechanical_speed_pu();
            electromagnetic_torque_pu = machine->get_torque_pu();
            electrical_angle = machine->get_stator_angle().get_radians();
        });
    }
};

TEST_F(AssynchronousMachinePerUnitOpenLoopTest, OpenLoop) {
    // Run simulation for 3 seconds to observe startup transient
    EXPECT_TRUE(simulator.simulate_for(3.0));

    // Check if machine has accelerated to near-synchronous speed
    // double sync_speed = 2 * pi<double> * stator_frequency / 2;  // Mechanical speed for 2 pole pairs
    // EXPECT_GT(machine.get_mechanical_speed_rad_s(), 0.9 * sync_speed);
}

// TEST_F(AssynchronousMachinePerUnitOpenLoopTest, StartupTransient) {
//     // Run simulation for 3 seconds to observe startup transient
//     EXPECT_TRUE(simulator.simulate_for(3.0));

//     // Check if machine has accelerated to near-synchronous speed
//     // In per-unit, synchronous speed = 1.0 for a machine with base frequency = machine frequency
//     double sync_speed_pu = 1.0;
//     EXPECT_GT(machine->get_mechanical_speed_pu(), 0.9 * sync_speed_pu);

//     // Output final values in both per-unit and physical units
//     double speed_pu = machine->get_mechanical_speed_pu();
//     double speed_rpm = speed_pu * machine->get_base_values().speed * 60.0 / (2 * pi<double>);
//     double sync_speed_rpm = sync_speed_pu * machine->get_base_values().speed * 60.0 / (2 * pi<double>);

//     std::cout << "Final speed: " << speed_pu << " p.u. (" << speed_rpm << " RPM, synchronous: " << sync_speed_rpm
//               << " RPM)" << std::endl;

//     double torque_pu = machine->get_torque_pu();
//     double torque_nm = torque_pu * machine->get_base_values().torque;
//     std::cout << "Final torque: " << torque_pu << " p.u. (" << torque_nm << " N·m)" << std::endl;

//     double current_magnitude_pu =
//         std::sqrt(std::pow(stator_current_pu.alpha(), 2) + std::pow(stator_current_pu.beta(), 2));
//     double current_magnitude_a = current_magnitude_pu * machine->get_base_values().current;

//     std::cout << "Stator current magnitude: " << current_magnitude_pu << " p.u. (" << current_magnitude_a << " A)"
//               << std::endl;

//     // Show slip
//     std::cout << "Slip: " << (1.0 - speed_pu) * 100 << "%" << std::endl;
// }

// TEST_F(AssynchronousMachinePerUnitOpenLoopTest, LoadTorqueResponse) {
//     // First run startup transient
//     EXPECT_TRUE(simulator.simulate_for(3.0));

//     // Capture no-load operating point
//     double no_load_speed = machine->get_mechanical_speed_pu();
//     double no_load_torque = machine->get_torque_pu();

//     // Apply 0.5 p.u. load torque
//     double load_torque_pu = 0.5;

//     // Reconfigure simulation with load
//     simulator.register_lambda_runner(sampling_time, [this, load_torque_pu]() {
//         // Update time counter
//         time_counter += sampling_time;

//         // Keep the same voltage (at nominal frequency and voltage)
//         double omega_s = 2.0 * pi<double> * stator_frequency;
//         double v_magnitude_pu = 1.0;

//         voltage_command_pu.alpha() = v_magnitude_pu * std::cos(omega_s * time_counter);
//         voltage_command_pu.beta() = v_magnitude_pu * std::sin(omega_s * time_counter);

//         // Update machine with voltage command and load torque
//         machine->update(voltage_command_pu, load_torque_pu);

//         // Update monitored signals
//         stator_current_pu = machine->get_stator_current_pu();
//         omega_m_pu = machine->get_mechanical_speed_pu();
//         electromagnetic_torque_pu = machine->get_torque_pu();
//         electrical_angle = machine->get_stator_angle().get_radians();
//     });

//     // Run for an additional 2 seconds with load
//     EXPECT_TRUE(simulator.simulate_for(2.0));

//     // Check that speed decreases and torque increases with load
//     double loaded_speed = machine->get_mechanical_speed_pu();
//     double loaded_torque = machine->get_torque_pu();

//     EXPECT_LT(loaded_speed, no_load_speed);
//     EXPECT_GT(loaded_torque, no_load_torque);
//     EXPECT_NEAR(loaded_torque, load_torque_pu, 0.05);  // Torque should match load within 5%

//     // Output loaded operating point
//     std::cout << "No-load speed: " << no_load_speed << " p.u." << std::endl;
//     std::cout << "Loaded speed: " << loaded_speed << " p.u." << std::endl;
//     std::cout << "Speed reduction: " << (no_load_speed - loaded_speed) * 100 << "%" << std::endl;
//     std::cout << "Electromagnetic torque: " << loaded_torque << " p.u." << std::endl;

//     // Calculate new slip
//     std::cout << "Loaded slip: " << (1.0 - loaded_speed) * 100 << "%" << std::endl;

//     // Get current magnitude
//     double current_magnitude_pu =
//         std::sqrt(std::pow(stator_current_pu.alpha(), 2) + std::pow(stator_current_pu.beta(), 2));
//     std::cout << "Loaded stator current: " << current_magnitude_pu << " p.u." << std::endl;
// }

}  // namespace