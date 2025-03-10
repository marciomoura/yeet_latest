#include "yeet_control/asynchronous_machine_vector_control.hpp"

#include <gtest/gtest.h>

#include "yeet_control/assynchronous_machine_per_unit.hpp"
#include "yeet_control/reference_frame.hpp"
#include "yeet_testing/simulator_gtest.hpp"

namespace {

using namespace yeet::control;

/**
 * @brief Test fixture for vector control of asynchronous machine
 *
 * Tests flux buildup and speed control using the FOC (Field Oriented Control)
 * strategy implemented in the asynchronous_machine_vector_controller.
 */
class AsynchronousMachineVectorControlTest : public ::testing::Test {
protected:
    static constexpr double kSamplingTime = 1e-4;  // 10 kHz sampling rate

    void SetUp() override {
        // Create and configure machine model with per-unit implementation
        // Base values: 400V RMS line voltage, 15kVA (3-phase), 50Hz
        machine = std::make_unique<assynchronous_machine_per_unit>(
            per_unit_definition::from_rms_line_voltage(400.0,    // 400V RMS line voltage
                                                       15000.0,  // 15kVA 3-phase power
                                                       50.0),    // 50Hz base frequency
            kSamplingTime);

        // Create vector controller
        controller = std::make_unique<asynchronous_machine_vector_controller<double>>(kSamplingTime);

        // Configure machine with typical per-unit parameters
        machine->parametrize(0.05,   // Rs: Stator resistance [p.u.]
                             0.05,   // Rr: Rotor resistance [p.u.]
                             1.6,    // Ls: Stator inductance [p.u.]
                             1.6,    // Lr: Rotor inductance [p.u.]
                             1.5,    // Lm: Mutual inductance [p.u.]
                             1,      // Pole pairs
                             1.0,    // J: Inertia [p.u.]
                             0.02);  // B: Friction [p.u.]

        // Calculate rotor time constant
        double Tr = 1.6 / 0.05;  // Lr/Rr = 2.0s

        // Configure current controllers
        controller->configure_current_controllers(0.05,   // Rs [p.u.]
                                                  0.05,   // Rr [p.u.]
                                                  1.6,    // Ls [p.u.]
                                                  1.6,    // Lr [p.u.]
                                                  1.5,    // Lm [p.u.]
                                                  0.1,    // 100ms response time
                                                  10.2);  // 1.2pu voltage limit

        // Configure flux controller
        controller->configure_flux_controller(Tr,    // Rotor time constant [s]
                                              0.1,   // 50ms response time
                                              1.0);  // Nominal flux [p.u.]

        // Configure speed controller
        controller->configure_speed_controller(1.0,    // J [p.u.]
                                               5.0,    // 5 rad/s bandwidth
                                               0.707,  // Damping ratio
                                               2.5);   // 1.5pu torque limit

        // Configure speed filter for smooth reference changes
        controller->configure_speed_filter(1.0);  // 1 Hz cutoff

        register_samplers();
        configure_simulation();
    }

    void register_samplers() {
        // References
        simulator.register_sampler_entry(
            yeet::testing::make_reference_sampler("speed_reference", [this]() { return speed_reference; }));
        simulator.register_sampler_entry(yeet::testing::make_reference_sampler(
            "flux_reference", [this]() { return controller->get_flux_reference(); }));

        // Machine state
        simulator.register_sampler_entry(
            yeet::testing::make_reference_sampler("speed_pu", [this]() { return machine->get_mechanical_speed_pu(); }));
        simulator.register_sampler_entry(
            yeet::testing::make_reference_sampler("torque_pu", [this]() { return machine->get_torque_pu(); }));

        // Current measurements and references in dq frame
        simulator.register_sampler_entry(
            yeet::testing::make_reference_sampler("id_measured", [this]() { return current_dq.d(); }));
        simulator.register_sampler_entry(
            yeet::testing::make_reference_sampler("iq_measured", [this]() { return current_dq.q(); }));

        // Voltage commands
        simulator.register_sampler_entry(
            yeet::testing::make_reference_sampler("vd_command", [this]() { return voltage_dq.d(); }));
        simulator.register_sampler_entry(
            yeet::testing::make_reference_sampler("vq_command", [this]() { return voltage_dq.q(); }));

        // Flux magnitude
        simulator.register_sampler_entry(yeet::testing::make_reference_sampler(
            "rotor_flux_magnitude_pu", [this]() { return rotor_flux_magnitude; }));

        // Angles and frequencies
        simulator.register_sampler_entry(yeet::testing::make_reference_sampler(
            "slip_frequency", [this]() { return controller->get_slip_frequency(); }));
        simulator.register_sampler_entry(yeet::testing::make_reference_sampler(
            "rotor_angle", [this]() { return machine->get_si_machine().get_rotor_flux().phase().get_radians(); }));
    }

    void configure_simulation() {
        simulator.register_lambda_runner(kSamplingTime, [this]() {
            // Get machine state in alpha-beta frame
            alphabeta<double> stator_current_ab = machine->get_stator_current_pu();
            alphabeta<double> rotor_flux_ab = machine->get_rotor_flux_pu();

            // Calculate rotor flux magnitude
            rotor_flux_magnitude = rotor_flux_ab.magnitude();

            // Calculate rotor flux angle for field orientation
            angle<zero_to_two_pi> rotor_flux_angle = rotor_flux_ab.phase();

            // Transform stator currents to dq frame using rotor flux orientation
            current_dq = stator_current_ab.to_dq(rotor_flux_angle);

            // Get mechanical speed
            double speed_measured = machine->get_mechanical_speed_pu();

            // Run controller update with measured values
            voltage_dq = controller->update(speed_reference, speed_measured, current_dq, rotor_flux_magnitude);

            // Convert voltage command back to alpha-beta for machine input
            alphabeta<double> voltage_ab = voltage_dq.to_alphabeta(rotor_flux_angle);

            // Update machine with voltage command and zero load torque
            machine->update(voltage_ab, load_torque);
        });
    }

protected:
    // Components
    std::unique_ptr<assynchronous_machine_per_unit> machine;
    std::unique_ptr<asynchronous_machine_vector_controller<double>> controller;
    yeet::testing::simulator simulator = yeet::testing::make_gtest_simulator();

    // Control and measurement variables
    double speed_reference{0.0};       // Speed reference [p.u.]
    dq<double> current_dq{0.0, 0.0};   // Measured currents in dq-frame [p.u.]
    double rotor_flux_magnitude{0.0};  // Measured rotor flux magnitude [p.u.]
    dq<double> voltage_dq{0.0, 0.0};   // Voltage command in dq-frame [p.u.]
    double load_torque{0.0};           // Load torque [p.u.]
};

/**
 * @brief Test for flux buildup and speed ramp without load
 *
 * This test verifies:
 * 1. Initial flux buildup phase (first 1s)
 * 2. Speed control phase (from 1s to 3s)
 * 3. Full speed operation phase (3s to 5s)
 */
TEST_F(AsynchronousMachineVectorControlTest, FluxBuildupAndSpeedRamp) {
    // Simulation sequence:
    // 1. Start with flux buildup (0-1s) with zero speed reference
    // 2. Ramp up speed reference (1-3s)
    // 3. Maintain at rated speed (3-5s)

    simulator.register_lambda_runner(kSamplingTime, [this]() {
        double time = simulator.get_current_simulation_time();

        // Phase 1: Flux buildup (0-1s)
        if (time < 1.0) {
            speed_reference = 0.0;  // Keep at standstill
        }
        // Phase 2: Speed ramp (1-3s)
        else if (time < 3.0) {
            speed_reference = (time - 1.0) * 0.5;  // Ramp to 1.0 p.u. over 2s
        }
        // Phase 3: Full speed (3-5s)
        else {
            speed_reference = 1.0;  // Maintain at rated speed
        }
    });

    // Check flux buildup
    simulator.register_checker(yeet::testing::make_range_checker<double>(
        "flux_buildup", [this]() { return rotor_flux_magnitude; },
        0.9,                                                       // 90% of nominal flux
        1.1,                                                       // 110% of nominal flux
        yeet::testing::checker_activation_time_range{0.9, 1.0}));  // Check at end of flux buildup phase

    // Check if speed reaches near reference at end of ramp
    simulator.register_checker(yeet::testing::make_range_checker<double>(
        "speed_tracking", [this]() { return machine->get_mechanical_speed_pu(); },
        0.95,                                                      // 95% of reference
        1.05,                                                      // 105% of reference
        yeet::testing::checker_activation_time_range{4.9, 5.0}));  // Check near end of simulation

    // Check that d-axis current stabilizes at a non-zero value for flux maintenance
    simulator.register_checker(yeet::testing::make_min_checker<double>(
        "d_current", [this]() { return current_dq.d(); },
        0.3,                                                       // At least 30% of nominal current for magnetization
        yeet::testing::checker_activation_time_range{3.0, 5.0}));  // Check during steady-state

    // Run the simulation
    EXPECT_TRUE(simulator.simulate_for(10.0));

    // load_torque = 0.5;

    // EXPECT_TRUE(simulator.simulate_for(10.0));

    // Output final values for reference
    std::cout << "=== Asynchronous Machine Vector Control Results ===" << std::endl;
    std::cout << "Final speed: " << machine->get_mechanical_speed_pu() << " p.u." << std::endl;
    std::cout << "Final torque: " << machine->get_torque_pu() << " p.u." << std::endl;
    std::cout << "Final rotor flux: " << rotor_flux_magnitude << " p.u." << std::endl;
    std::cout << "Final d-axis current: " << current_dq.d() << " p.u." << std::endl;
    std::cout << "Final q-axis current: " << current_dq.q() << " p.u." << std::endl;
    std::cout << "Final slip frequency: " << controller->get_slip_frequency() << " p.u." << std::endl;
}

}  // namespace