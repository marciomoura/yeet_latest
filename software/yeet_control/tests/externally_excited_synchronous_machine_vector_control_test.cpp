#include <gtest/gtest.h>

#include "yeet_control/externally_excited_synchronous_machine_per_unit.hpp"
#include "yeet_control/machine_current_controller.hpp"
#include "yeet_control/machine_flux_controller.hpp"
#include "yeet_control/machine_speed_controller.hpp"
#include "yeet_testing/simulator_gtest.hpp"

namespace {

using namespace yeet;
using namespace yeet::control;

/**
 * @brief Test fixture for modular vector control of Externally Excited Synchronous Machine (EESM)
 *
 * This test demonstrates the use of separate controllers for:
 * - Speed control (machine_speed_controller)
 * - Field excitation control (machine_flux_controller)
 * - Stator current control (machine_current_controller)
 */
class ExternallyExcitedSynchronousMachineVectorControlTest : public ::testing::Test {
protected:
    static constexpr double kSamplingTime = 1e-4;  // 10 kHz

    void SetUp() override {
        // Create and configure machine model with per-unit implementation
        // Base values: 400V RMS line voltage, 10kVA (3-phase), 50Hz
        machine = std::make_unique<externally_excited_synchronous_machine_per_unit>(
            per_unit_definition::from_rms_line_voltage(400.0,    // 400V RMS line voltage
                                                       10000.0,  // 10kVA 3-phase power
                                                       50.0),    // 50Hz base frequency
            kSamplingTime);

        // Create controllers
        speed_controller = std::make_unique<machine_speed_controller>(kSamplingTime);
        field_controller = std::make_unique<machine_flux_controller>(
            kSamplingTime, machine_flux_controller::control_mode::externally_excited);
        current_controller = std::make_unique<machine_current_controller>(
            kSamplingTime, machine_current_controller::machine_type::externally_excited);

        // Configure machine with per-unit parameters for EESM
        machine->parametrize(0.05,   // Rs: Stator resistance [p.u.]
                             1.2,    // Rf: Field resistance [p.u.]
                             1.0,    // Ld: d-axis inductance [p.u.]
                             0.65,   // Lq: q-axis inductance [p.u.]
                             1.4,    // Lf: Field inductance [p.u.]
                             0.9,    // Lmd: Mutual inductance between d-axis and field [p.u.]
                             1,      // Pole pairs
                             1.0,    // J: Inertia [p.u.]
                             0.02);  // B: Friction [p.u.]

        // Configure speed controller
        speed_controller->configure(1.0,          // Inertia [p.u.]
                                    5.0,          // Bandwidth [rad/s]
                                    0.9,          // Damping ratio
                                    1.5);         // Torque limit [p.u.]
        speed_controller->configure_filter(5.0);  // 5Hz cutoff

        // Configure field controller for external excitation
        field_controller->configure_for_external_excitation(1.2,   // Field resistance [p.u.]
                                                            1.4,   // Field inductance [p.u.]
                                                            0.9,   // Mutual inductance between d-axis and field [p.u.]
                                                            0.05,  // Response time [s]
                                                            1.0,   // Nominal field current [p.u.]
                                                            2.0);  // Field voltage limit [p.u.]

        // Configure current controller for EESM
        current_controller->configure_for_eesm(0.05,  // Stator resistance [p.u.]
                                               1.0,   // d-axis inductance [p.u.]
                                               0.65,  // q-axis inductance [p.u.]
                                               0.9,   // Mutual inductance between d-axis and field [p.u.]
                                               0.05,  // Response time [s]
                                               1.5);  // Voltage limit [p.u.]

        register_samplers();
        configure_simulation();
    }

    void register_samplers() {
        // References
        simulator.register_sampler_entry(yeet::testing::make_reference_sampler("speed_reference", speed_reference));
        // simulator.register_sampler_entry(yeet::testing::make_reference_sampler(
        //     "field_current_reference", [this]() { return field_controller->get_flux_reference(); }));
        simulator.register_sampler_entry(yeet::testing::make_reference_sampler(
            "speed_reference_filtered", [this]() { return speed_controller->get_filtered_speed_reference(); }));

        // Machine measurements
        simulator.register_sampler_entry(yeet::testing::make_reference_sampler("speed_measured", speed_measured));
        simulator.register_sampler_entry(
            yeet::testing::make_reference_sampler("field_current_measured", field_current_measured));
        simulator.register_sampler_entry(
            yeet::testing::make_reference_sampler("stator_current_d", stator_current_measured.d()));
        simulator.register_sampler_entry(
            yeet::testing::make_reference_sampler("stator_current_q", stator_current_measured.q()));

        // Control outputs
        simulator.register_sampler_entry(
            yeet::testing::make_reference_sampler("stator_voltage_d", stator_voltage_command.d()));
        simulator.register_sampler_entry(
            yeet::testing::make_reference_sampler("stator_voltage_q", stator_voltage_command.q()));
        simulator.register_sampler_entry(yeet::testing::make_reference_sampler("field_voltage", field_voltage_command));

        // Controller internal values
        simulator.register_sampler_entry(yeet::testing::make_reference_sampler("torque_reference", torque_reference));
        simulator.register_sampler_entry(
            yeet::testing::make_reference_sampler("stator_current_d_ref", stator_current_reference.d()));
        simulator.register_sampler_entry(
            yeet::testing::make_reference_sampler("stator_current_q_ref", stator_current_reference.q()));

        // Machine states
        simulator.register_sampler_entry(
            yeet::testing::make_reference_sampler("torque_pu", [this]() { return machine->get_torque_pu(); }));
        simulator.register_sampler_entry(yeet::testing::make_reference_sampler(
            "electrical_angle", [this]() { return machine->get_rotor_angle().get_radians(); }));
        simulator.register_sampler_entry(yeet::testing::make_reference_sampler(
            "electrical_speed_pu", [this]() { return machine->get_electrical_speed_pu(); }));

        // Field circuit monitoring
        simulator.register_sampler_entry(yeet::testing::make_reference_sampler(
            "field_flux_linkage", [this]() { return machine->get_field_flux_linkage_pu(); }));

        // Voltage monitoring
        simulator.register_sampler_entry(yeet::testing::make_reference_sampler("stator_voltage_magnitude", [this]() {
            return std::sqrt(stator_voltage_command.d() * stator_voltage_command.d() +
                             stator_voltage_command.q() * stator_voltage_command.q());
        }));
    }

    void configure_simulation() {
        simulator.register_lambda_runner(kSamplingTime, [this]() {
            // 1. Get machine states
            speed_measured = machine->get_mechanical_speed_pu();
            stator_current_measured = machine->get_stator_currents_pu();
            field_current_measured = machine->get_field_current_pu();

            // Get electrical speed for current controller
            double electrical_speed = machine->get_electrical_speed_pu();

            // 2. Field current control (for excitation)
            field_voltage_command = field_controller->update_external_excitation(field_current_measured);

            // 3. Speed control (outer loop)
            torque_reference = speed_controller->update(speed_reference, speed_measured);

            // 4. Calculate required d-axis and q-axis currents
            // For EESM, typically we use id=0 strategy (no reluctance torque exploitation)
            stator_current_reference.d() = 0.0;  // d-axis current is kept at zero

            // Calculate q-axis current for torque using current field current
            stator_current_reference.q() = field_controller->calculate_mtpa_current(torque_reference);

            // 5. Stator current control (inner loop)
            stator_voltage_command =
                current_controller->update(stator_current_reference, stator_current_measured, electrical_speed,
                                           field_current_measured  // Pass field current for back-EMF compensation
                );

            // 6. Update machine with voltage commands
            machine->update(stator_voltage_command, field_voltage_command, load_torque);
        });
    }

protected:
    // Components
    std::unique_ptr<externally_excited_synchronous_machine_per_unit> machine;
    std::unique_ptr<machine_speed_controller> speed_controller;
    std::unique_ptr<machine_flux_controller> field_controller;
    std::unique_ptr<machine_current_controller> current_controller;
    yeet::testing::simulator simulator = yeet::testing::make_gtest_simulator();

    // Control signals
    double speed_reference{0.0};                    // Speed reference [p.u.]
    double speed_measured{0.0};                     // Measured speed [p.u.]
    double torque_reference{0.0};                   // Torque reference from speed controller [p.u.]
    dq<double> stator_current_reference{0.0, 0.0};  // Current reference in dq-frame [p.u.]
    dq<double> stator_current_measured{0.0, 0.0};   // Measured current in dq-frame [p.u.]
    dq<double> stator_voltage_command{0.0, 0.0};    // Voltage command in dq-frame [p.u.]
    double field_current_measured{0.0};             // Measured field current [p.u.]
    double field_voltage_command{0.0};              // Field voltage command [p.u.]
    double load_torque{0.0};                        // External load torque [p.u.]
};

/**
 * @brief Test for field current buildup and speed step response
 */
TEST_F(ExternallyExcitedSynchronousMachineVectorControlTest, FieldBuildupAndSpeedRampUp) {
    // Set field current reference to 1.0 p.u.
    field_controller->set_flux_reference(1.0);  // For EESM, "flux" reference is field current

    // First establish field current (flux)
    ASSERT_TRUE(simulator.simulate_for(0.5));

    // Check if field current has reached reference
    EXPECT_NEAR(field_current_measured, 1.0, 0.1);

    // Step to rated speed
    speed_reference = 1.0;

    // Run simulation to reach steady state
    ASSERT_TRUE(simulator.simulate_for(1.0));

    // Check speed
    EXPECT_NEAR(speed_measured, 1.0, 0.1);

    // Check that d-axis current remains near zero (direct axis decoupling)
    EXPECT_NEAR(stator_current_measured.d(), 0.0, 0.1);

    // Print results
    std::cout << "FieldBuildupAndSpeedRampUp test results:" << std::endl;
    std::cout << "Final speed: " << speed_measured << " p.u." << std::endl;
    std::cout << "Final field current: " << field_current_measured << " p.u." << std::endl;
    std::cout << "Final field flux linkage: " << machine->get_field_flux_linkage_pu() << " p.u." << std::endl;
    std::cout << "Final d-axis current: " << stator_current_measured.d() << " p.u." << std::endl;
    std::cout << "Final q-axis current: " << stator_current_measured.q() << " p.u." << std::endl;
    std::cout << "Final torque: " << machine->get_torque_pu() << " p.u." << std::endl;
}

/**
 * @brief Test for field weakening by reducing field current
 */
TEST_F(ExternallyExcitedSynchronousMachineVectorControlTest, FieldWeakening) {
    // First establish nominal field and speed
    field_controller->set_flux_reference(1.0);
    speed_reference = 1.0;

    // Run simulation to reach steady state
    ASSERT_TRUE(simulator.simulate_for(1.0));

    // Store baseline values
    double baseline_q_current = stator_current_measured.q();
    double baseline_voltage = std::sqrt(stator_voltage_command.d() * stator_voltage_command.d() +
                                        stator_voltage_command.q() * stator_voltage_command.q());

    std::cout << "Baseline operation:" << std::endl;
    std::cout << "Field current: " << field_current_measured << " p.u." << std::endl;
    std::cout << "Q-axis current: " << baseline_q_current << " p.u." << std::endl;
    std::cout << "Stator voltage magnitude: " << baseline_voltage << " p.u." << std::endl;

    // Now reduce field current for field weakening (50%)
    field_controller->set_flux_reference(0.5);

    // Run simulation to reach new steady state
    ASSERT_TRUE(simulator.simulate_for(0.5));

    // Field current should be reduced
    EXPECT_NEAR(field_current_measured, 0.5, 0.1);

    // Speed should still be maintained
    EXPECT_NEAR(speed_measured, 1.0, 0.1);

    // Q-axis current should increase to maintain the same torque
    EXPECT_GT(stator_current_measured.q(), baseline_q_current * 1.5);

    // Print field weakening results
    std::cout << "Field weakening results:" << std::endl;
    std::cout << "Field current: " << field_current_measured << " p.u." << std::endl;
    std::cout << "Q-axis current: " << stator_current_measured.q() << " p.u." << std::endl;
    std::cout << "D-axis current: " << stator_current_measured.d() << " p.u." << std::endl;
    std::cout << "Stator voltage magnitude: "
              << std::sqrt(stator_voltage_command.d() * stator_voltage_command.d() +
                           stator_voltage_command.q() * stator_voltage_command.q())
              << " p.u." << std::endl;
}

/**
 * @brief Test for load disturbance rejection
 */
TEST_F(ExternallyExcitedSynchronousMachineVectorControlTest, LoadDisturbanceRejection) {
    // First establish field current and speed
    field_controller->set_flux_reference(1.0);
    speed_reference = 1.0;

    // Run simulation to reach steady state
    ASSERT_TRUE(simulator.simulate_for(1.0));

    // Apply load disturbance
    load_torque = 0.5;  // 50% of rated torque

    // Check speed dip
    simulator.register_checker(yeet::testing::make_min_checker<double>(
        "minimum_speed", [this]() { return speed_measured; },
        0.7,  // Can dip to 70% of reference
        yeet::testing::checker_activation_time_range{1.0, 1.2}));

    // Check speed recovery
    simulator.register_checker(yeet::testing::make_range_checker<double>(
        "speed_recovery", [this]() { return speed_measured; }, 0.95, 1.05,  // Within 5% of reference
        yeet::testing::checker_activation_time_range{1.5, 1.7}));

    // Run with load for 1.0s
    EXPECT_TRUE(simulator.simulate_for(1.0));

    // Print results with load
    std::cout << "LoadDisturbanceRejection test results:" << std::endl;
    std::cout << "Speed with load: " << speed_measured << " p.u." << std::endl;
    std::cout << "Field current: " << field_current_measured << " p.u." << std::endl;
    std::cout << "D-axis current: " << stator_current_measured.d() << " p.u." << std::endl;
    std::cout << "Q-axis current: " << stator_current_measured.q() << " p.u." << std::endl;
    std::cout << "Torque with load: " << machine->get_torque_pu() << " p.u." << std::endl;
}

/**
 * @brief Test for high-speed operation with field weakening
 */
TEST_F(ExternallyExcitedSynchronousMachineVectorControlTest, HighSpeedOperation) {
    // First establish nominal field
    field_controller->set_flux_reference(1.0);

    // Run to establish field
    ASSERT_TRUE(simulator.simulate_for(0.5));

    // Step to 1.5x rated speed (requires field weakening)
    speed_reference = 1.5;

    // Run to attempt reaching high speed
    ASSERT_TRUE(simulator.simulate_for(1.0));

    // Check if high speed is reached
    double high_speed_achieved = speed_measured;

    // Now with field weakening (reduced field current)
    field_controller->set_flux_reference(0.7);  // 70% field current

    // Run again with reduced field
    ASSERT_TRUE(simulator.simulate_for(1.0));

    // Check if high speed is better achieved
    EXPECT_NEAR(speed_measured, high_speed_achieved, 1e-3);

    // Print results
    std::cout << "HighSpeedOperation test results:" << std::endl;
    std::cout << "Speed with nominal field: " << high_speed_achieved << " p.u." << std::endl;
    std::cout << "Speed with reduced field: " << speed_measured << " p.u." << std::endl;
    std::cout << "Field current: " << field_current_measured << " p.u." << std::endl;
    std::cout << "D-axis current: " << stator_current_measured.d() << " p.u." << std::endl;
    std::cout << "Q-axis current: " << stator_current_measured.q() << " p.u." << std::endl;
    std::cout << "Stator voltage magnitude: "
              << std::sqrt(stator_voltage_command.d() * stator_voltage_command.d() +
                           stator_voltage_command.q() * stator_voltage_command.q())
              << " p.u." << std::endl;
}

/**
 * @brief Test for field current step response
 */
TEST_F(ExternallyExcitedSynchronousMachineVectorControlTest, FieldCurrentStepResponse) {
    // Initial state
    field_controller->set_flux_reference(0.5);  // Start with 50% field current

    // Run to establish initial field
    ASSERT_TRUE(simulator.simulate_for(0.5));

    // Check initial field current
    EXPECT_NEAR(field_current_measured, 0.5, 0.1);

    // Step to nominal field current
    field_controller->set_flux_reference(1.0);

    // Register field current step response checker
    simulator.register_checker(yeet::testing::make_range_checker<double>(
        "field_current_step_response", [this]() { return field_current_measured; }, 0.9,
        1.1,  // Within 10% of reference
        yeet::testing::checker_activation_time_range{0.8, 1.0}));

    // Run simulation for field current step response
    EXPECT_TRUE(simulator.simulate_for(0.5));

    // Print results
    std::cout << "FieldCurrentStepResponse test results:" << std::endl;
    std::cout << "Final field current: " << field_current_measured << " p.u." << std::endl;
    std::cout << "Field voltage command: " << field_voltage_command << " p.u." << std::endl;
    std::cout << "Field flux linkage: " << machine->get_field_flux_linkage_pu() << " p.u." << std::endl;
}

}  // namespace