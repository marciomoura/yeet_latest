#include "yeet_control/externally_excited_synchronous_machine_per_unit.hpp"

#include <gtest/gtest.h>

#include "yeet_control/angle.hpp"
#include "yeet_control/constants.hpp"
#include "yeet_control/reference_frame.hpp"
#include "yeet_testing/simulator_gtest.hpp"

namespace {

using namespace yeet;
using namespace yeet::control;

class ExternallyExcitedSynchronousMachinePerUnitTest : public ::testing::Test {
public:
    static constexpr double sampling_time = 1e-4;      // 10 kHz sampling rate
    static constexpr double nominal_frequency = 50.0;  // 50 Hz nominal frequency

    void SetUp() override {
        // Configure machine with per-unit values
        // Base values: 400V RMS line voltage, 5kVA (3-phase), 50Hz
        machine = std::make_unique<externally_excited_synchronous_machine_per_unit>(
            per_unit_definition::from_rms_line_voltage(400.0,   // 400V RMS line voltage
                                                       5000.0,  // 5kVA 3-phase power
                                                       50.0),   // 50Hz base frequency
            sampling_time);

        // Configure with typical per-unit parameters for synchronous machine
        machine->parametrize(0.05,   // Rs: Stator resistance [p.u.]
                             1.2,    // Rf: Field resistance [p.u.]
                             1.0,    // Ld: d-axis inductance [p.u.]
                             0.65,   // Lq: q-axis inductance [p.u.]
                             1.4,    // Lf: Field inductance [p.u.]
                             1.05,   // Lmd: Mutual inductance [p.u.]
                             1,      // Pole pairs
                             1.0,    // J: Inertia [p.u.]
                             0.02);  // B: Friction [p.u.]

        register_samplers();
        configure_simulation();
    }

protected:
    // Components
    std::unique_ptr<externally_excited_synchronous_machine_per_unit> machine;
    yeet::testing::simulator simulator = yeet::testing::make_gtest_simulator();

    // Variables to monitor
    dq<double> stator_voltage_command_pu{0.0, 0.0};
    double field_voltage_pu{0.0};
    dq<double> stator_current_pu{0.0, 0.0};
    double field_current_pu{0.0};
    double mechanical_speed_pu{0.0};
    double electromagnetic_torque_pu{0.0};
    double electrical_angle{0.0};
    double time_counter{0.0};

private:
    void register_samplers() {
        // Voltage commands (per-unit)
        simulator.register_sampler_entry(
            yeet::testing::make_reference_sampler("voltage_d_pu", [this]() { return stator_voltage_command_pu.d(); }));
        simulator.register_sampler_entry(
            yeet::testing::make_reference_sampler("voltage_q_pu", [this]() { return stator_voltage_command_pu.q(); }));
        simulator.register_sampler_entry(
            yeet::testing::make_reference_sampler("field_voltage_pu", [this]() { return field_voltage_pu; }));

        // Currents (per-unit)
        simulator.register_sampler_entry(
            yeet::testing::make_reference_sampler("current_d_pu", [this]() { return stator_current_pu.d(); }));
        simulator.register_sampler_entry(
            yeet::testing::make_reference_sampler("current_q_pu", [this]() { return stator_current_pu.q(); }));
        simulator.register_sampler_entry(
            yeet::testing::make_reference_sampler("field_current_pu", [this]() { return field_current_pu; }));

        // Speed and torque (per-unit)
        simulator.register_sampler_entry(
            yeet::testing::make_reference_sampler("speed_pu", [this]() { return mechanical_speed_pu; }));
        simulator.register_sampler_entry(
            yeet::testing::make_reference_sampler("torque_pu", [this]() { return electromagnetic_torque_pu; }));

        // Speed in RPM (converted from per-unit)
        simulator.register_sampler_entry(yeet::testing::make_reference_sampler("speed_rpm", [this]() {
            double speed_rad_s = mechanical_speed_pu * machine->get_base_values().speed;
            return speed_rad_s * 60.0 / (2 * pi);
        }));

        // Electrical angle
        simulator.register_sampler_entry(yeet::testing::make_reference_sampler(
            "electrical_angle", [this]() { return machine->get_rotor_angle().get_radians(); }));

        // Field flux linkage
        simulator.register_sampler_entry(yeet::testing::make_reference_sampler(
            "field_flux_pu", [this]() { return machine->get_field_flux_linkage_pu(); }));
    }

    void configure_simulation() {
        double ramp_time = 0.5;  // 500ms voltage ramp-up

        simulator.register_lambda_runner(sampling_time, [this, ramp_time]() {
            // Update time counter
            time_counter += sampling_time;

            // Smooth ramp for stator voltage
            double voltage_ramp_factor = std::min(1.0, time_counter / ramp_time);

            // Apply stator voltage in dq reference frame
            // For synchronous machine with field winding, we apply voltage primarily in q-axis
            stator_voltage_command_pu.d() = 0.0;                        // No d-axis voltage
            stator_voltage_command_pu.q() = voltage_ramp_factor * 1.0;  // Ramp to 1.0 p.u.

            // Apply field voltage (1.0 p.u.)
            field_voltage_pu = 0.6;  // Typical field voltage in p.u.

            // Update machine with voltage commands and zero load torque
            machine->update(stator_voltage_command_pu, field_voltage_pu, 0.0);

            // Update monitored signals
            stator_current_pu = machine->get_stator_currents_pu();
            field_current_pu = machine->get_field_current_pu();
            mechanical_speed_pu = machine->get_mechanical_speed_pu();
            electromagnetic_torque_pu = machine->get_torque_pu();
            electrical_angle = machine->get_rotor_angle().get_radians();
        });
    }
};

TEST_F(ExternallyExcitedSynchronousMachinePerUnitTest, NominalVoltageTest) {
    // Run simulation for 8 seconds to observe startup and steady-state
    EXPECT_TRUE(simulator.simulate_for(8.0));

    // Output final values (without assertions)
    std::cout << "=== Externally Excited Synchronous Machine Per-Unit Test Results ===" << std::endl;
    std::cout << "Final speed: " << mechanical_speed_pu << " p.u." << std::endl;

    double speed_rpm = mechanical_speed_pu * machine->get_base_values().speed * 60.0 / (2 * pi);
    double sync_speed_rpm =
        1.0 * machine->get_base_values().speed * 60.0 / (2 * pi) / machine->get_si_machine().get_pole_pairs();

    std::cout << "Final speed: " << speed_rpm << " RPM (synchronous: " << sync_speed_rpm << " RPM)" << std::endl;
    std::cout << "Final torque: " << electromagnetic_torque_pu << " p.u. ("
              << electromagnetic_torque_pu * machine->get_base_values().torque << " N·m)" << std::endl;

    // Calculate stator current magnitude
    double stator_current_magnitude_pu =
        std::sqrt(std::pow(stator_current_pu.d(), 2) + std::pow(stator_current_pu.q(), 2));
    std::cout << "Stator current magnitude: " << stator_current_magnitude_pu << " p.u. ("
              << stator_current_magnitude_pu * machine->get_base_values().current << " A)" << std::endl;

    std::cout << "Field current: " << field_current_pu << " p.u. ("
              << field_current_pu * machine->get_base_values().current << " A)" << std::endl;

    std::cout << "Field flux linkage: " << machine->get_field_flux_linkage_pu() << " p.u. ("
              << machine->get_field_flux_linkage_pu() * machine->get_base_values().flux << " Wb)" << std::endl;
}

}  // namespace