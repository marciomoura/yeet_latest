/**
 * @file soft_starter_test.cpp
 * @brief Integration test for the soft-starter startup sequence to nominal speed (10Hz).
 *
 * This test verifies the complete startup sequence of a 50MW machine (modeled at 2MW base)
 * until it reaches 1.0 per-unit speed (10Hz).
 */

#include <gtest/gtest.h>

#include "sfc_integration/plant_complete_sfc_closed_loop_simulation_builder.hpp"
#include "sfc_integration/sfc_plant_model_datamodel.hpp"
#include "sfc_loggers.hpp"
#include "sfc_plant_configuration.hpp"
#include "yeet_testing/simulator.hpp"
#include "yeet_testing/simulator_checker.hpp"
#include "yeet_testing/simulator_gtest.hpp"

using namespace yeet::application::sfc;
using namespace yeet::application::sfc::integration;
using namespace yeet;

class SoftStarterTest : public ::testing::Test {
public:
    void SetUp() override
    {
        // Configure sampler sampling rate
        simulator.configure_sampler_sampling_rate(ms(5.0f));
        // Register all applications
        system.register_applications(simulator);

        // Initialize the system
        initialize();

        // ==================== PER-UNIT SYSTEM CONFIGURATION (from machine_side_tasks.cpp) ====================
        auto& pu_machine = system.machine_side_control.pu_system_datamodel.get_parameters();
        pu_machine.dc_link_current_base.write(current_t{866.0f});
        pu_machine.grid_frequency_base.write(frequency_t{50.0f});
        pu_machine.grid_voltage_base_rms.write(voltage_t{2090.0f});
        pu_machine.machine_frequency_base.write(frequency_t{10.0f});
        pu_machine.machine_num_poles.write(2);
        pu_machine.machine_power_base.write(power_t{2e6});  // 2 MW SFC Base
        // pu_machine.machine_voltage_base_rms.write(voltage_t{10500.0f});
        pu_machine.machine_voltage_base_rms.write(voltage_t{1880.0f});

        auto& pu_grid = system.grid_side_control.pu_system_datamodel.get_parameters();
        pu_grid.dc_link_current_base.write(current_t{866.0f});
        pu_grid.grid_frequency_base.write(frequency_t{50.0f});
        pu_grid.grid_voltage_base_rms.write(voltage_t{2090.0f});
        pu_grid.machine_frequency_base.write(frequency_t{10.0f});
        pu_grid.machine_num_poles.write(2);
        pu_grid.machine_power_base.write(power_t{2e6});  // 2 MW SFC Base
        // pu_grid.machine_voltage_base_rms.write(voltage_t{10500.0f});
        pu_grid.machine_voltage_base_rms.write(voltage_t{1880.0f});

        system.machine_side_control.machine_side_observer.get_parameters().nominal_frequency.write(frequency_t{10.0f});
        system.grid_side_control.operation_mode_monitor.get_parameters().force_to_natural_commutation.write(false);
        system.grid_side_control.operation_mode_monitor.get_parameters().force_to_forced_commutation.write(false);
        system.grid_side_control.operation_mode_monitor.get_parameters().mode_handover_speed_pu.write(
            yeet::speed_pu_t{0.4f});
        system.grid_side_control.operation_mode_monitor.get_parameters().mode_hysteresis_factor.write(
            yeet::speed_pu_t{0.1f});

        // ==================== STARTUP SEQUENCER CONFIGURATION ====================
        auto& seq = system.machine_side_control.startup_sequencer.get_parameters();
        seq.virtual_frequency_duration.write(duration_t{6.0f});
        seq.virtual_frequency_initial.write(frequency_pu_t{0.0f});
        seq.virtual_frequency_final.write(frequency_pu_t{0.1f});
        seq.virtual_torque_duration.write(duration_t{1.0f});
        seq.virtual_torque_initial.write(torque_pu_t{0.4f});
        seq.virtual_torque_final.write(torque_pu_t{0.4f});
        seq.speed_reference.write(speed_pu_t{1.0f});
        seq.speed_reference_rate_limit_enabled.write(true);
        seq.speed_reference_rate_limit_seconds_per_pu.write(60.0f);
        seq.virtual_torque_damping_enabled.write(true);
        seq.virtual_torque_damping_kp.write(2.0f);
        seq.virtual_torque_damping_ti.write(duration_t{0.5f});

        // ==================== PLANT CONFIGURATION ====================
        // Model a 50 MW machine. Since we operate at 10Hz base and 2MW SFC base:
        // H_base = H_nom * (S_nom/S_base) * (f_base/f_nom)^2 = 5 * 25 * 0.04 = 5.0 seconds.
        auto& plant = system.plant.get_parameters();
        plant.topology.write(sfc_plant_model_datamodel::topology_type::internal);
        system.plant.get_plant_model().configure_electrical_parameters(
            sfc_plant_configuration::get_50mw_machine_electrical_parameters());
        plant.motor_inertia_constant.write(inertia_constant_t{2.0f});
        plant.motor_quadratic_load_coefficient.write(0.1f);  // Typical gas turbine drag
        // plant.motor_linear_load_coefficient.write(torque_pu_t{0.01f});
        // plant.motor_damping_factor.write(damping_coefficient_pu_t{0.1f});
        // plant.constant_torque.write(torque_pu_t{0.0f});
        plant.operation_mode.write(sfc_plant_model_datamodel::machine_operation_mode::variable_speed);
        plant.voltage_control_mode.write(sfc_plant_model_datamodel::machine_voltage_control_mode::variable_speed);

        // Configuration of the stator-voltage control
        // system.machine_side_control.excitation_controller.get_parameters().voltage_reference_speed_axis_pu
        system.machine_side_control.excitation_controller.get_parameters().voltage_reference_speed_axis_pu.write(
            std::array<yeet::speed_pu_t, 7>{
                yeet::speed_pu_t{0.0f / 3000.0f},     // 0 pu
                yeet::speed_pu_t{150.0f / 3000.0f},   // 0.05 pu
                yeet::speed_pu_t{300.0f / 3000.0f},   // 0.1 pu
                yeet::speed_pu_t{600.0f / 3000.0f},   // 0.2 pu
                yeet::speed_pu_t{3150.0f / 3000.0f},  // 1.05 pu
                yeet::speed_pu_t{3150.0f / 3000.0f},  // 1.05 pu
                yeet::speed_pu_t{3150.0f / 3000.0f},  // 1.05 pu
            });
        system.machine_side_control.excitation_controller.get_parameters().voltage_reference_pu.write(
            std::array<yeet::voltage_pu_t, 7>{
                yeet::voltage_pu_t{0.2f},   // 0.0 pu
                yeet::voltage_pu_t{0.2f},   // 0.25 pu
                yeet::voltage_pu_t{0.15f},  // 0.4 pu
                yeet::voltage_pu_t{0.25f},  // 0.5 pu
                yeet::voltage_pu_t{1.0f},   // 1.0 pu
                yeet::voltage_pu_t{1.0f},   // 1.0 pu
                yeet::voltage_pu_t{1.0f},   // 1.0 pu
                // yeet::voltage_pu_t{0.0f},               // 0.0 pu
                // yeet::voltage_pu_t{470.0f / 1880.0f},   // 0.25 pu
                // yeet::voltage_pu_t{500.0f / 1880.0f},   // 0.4 pu
                // yeet::voltage_pu_t{940.0f / 1880.0f},   // 0.5 pu
                // yeet::voltage_pu_t{1880.0f / 1880.0f},  // 1.0 pu
                // yeet::voltage_pu_t{1880.0f / 1880.0f},  // 1.0 pu
                // // yeet::voltage_pu_t{1880.0f / 1880.0f},  // 1.0 pu
                // yeet::voltage_pu_t{1880.0f / 1880.0f},  // 1.0 pu
            });

        // Forced Mode Configuration (Low-Speed / Low-Voltage startup region)
        // -----------------------------------------------------------------
        // We use a constant forced field current until enough voltage or speed is built up.
        // Transition points (Release thresholds) are set here.

        // 1. Definition of the forced field current value (e.g., 40%)
        system.machine_side_control.excitation_controller.get_parameters().forced_field_current_pu.write(
            yeet::current_pu_t{0.25f});
        // system.machine_side_control.excitation_controller.get_parameters()..write(
        //     yeet::current_pu_t{0.0f});
        system.machine_side_control.excitation_controller.get_parameters().release_voltage_threshold_pu.write(
            yeet::voltage_pu_t{0.2});
        system.machine_side_control.excitation_controller.get_parameters().release_speed_threshold_pu.write(
            yeet::speed_pu_t{0.2});

        // ==================== EXCITATION CONFIGURATION ====================
        // auto& ex = system.machine_side_control.excitation_controller.get_parameters();
        // ex.forced_field_current_pu.write(current_pu_t{0.3f});
        // ex.prefluxing_duration.write(duration_t{1.0f});
        // ex.release_voltage_threshold_pu.write(voltage_pu_t{0.4f});
        // ex.release_speed_threshold_pu.write(speed_pu_t{0.4f});

        // 3. Time required for field establishment (Pre-fluxing)
        system.machine_side_control.excitation_controller.get_parameters().prefluxing_duration.write(
            yeet::duration_t{1.5f});

        // Configuration of the machine-side lookup-table for the machine-firing
        auto dc_link_base =
            system.machine_side_control.pu_system_datamodel.get_parameters().dc_link_current_base.read();

        system.machine_side_control.machine_side_controller.get_parameters().dc_current_pu_lookup_table.write(
            std::array<yeet::current_pu_t, 6>{yeet::current_pu_t{0.0 / dc_link_base},  //
                yeet::current_pu_t{500.0f / dc_link_base},                             //
                yeet::current_pu_t{866.0f / dc_link_base},                             //
                yeet::current_pu_t{2625.0f / dc_link_base},                            //
                yeet::current_pu_t{3000.0f / dc_link_base},                            //
                yeet::current_pu_t{3000.0f / dc_link_base}});

        system.machine_side_control.machine_side_controller.get_parameters().firing_angle_lookup_table.write(
            std::array<yeet::angle_wrapped, 6>{yeet::angle_wrapped::from_degrees(158.0f),
                yeet::angle_wrapped::from_degrees(158.0f), yeet::angle_wrapped::from_degrees(153.5f),
                yeet::angle_wrapped::from_degrees(132.1), yeet::angle_wrapped::from_degrees(125.0f),
                yeet::angle_wrapped::from_degrees(125.0f)});

        // Firing angle delta vs speed (RPM/pu)
        // 0 rpm, 600 rpm, 3150 rpm
        system.machine_side_control.machine_side_controller.get_parameters().speed_pu_alpha_delta_axis.write(
            std::array<yeet::speed_pu_t, 3>{yeet::speed_pu_t{0.0f}, yeet::speed_pu_t{0.2f}, yeet::speed_pu_t{1.05f}});

        system.machine_side_control.machine_side_controller.get_parameters().firing_angle_delta_lookup_table.write(
            std::array<yeet::angle_wrapped, 3>{yeet::angle_wrapped::from_degrees(0.0f),
                yeet::angle_wrapped::from_degrees(0.0f), yeet::angle_wrapped::from_degrees(-7.1f)});
        ///////
        system.grid_side_control.current_controller.get_parameters().max_firing_angle.write(
            angle_wrapped::from_degrees(135.0f));

        // Register loggers
        register_complete_logger(simulator, system.machine_side_control.speed_controller);
        register_complete_logger(simulator, system.machine_side_control.startup_sequencer);
        register_complete_logger(simulator, system.machine_side_control.excitation_controller);
        register_complete_logger(simulator, system.machine_side_control.machine_side_controller);
        register_complete_logger(simulator, system.grid_side_control.operation_mode_monitor);
        register_complete_logger(simulator, system.grid_side_control.current_controller);
        register_complete_logger(simulator, system.machine_side_control.machine_side_observer);
        register_complete_logger(simulator, system.plant);
    }

    void initialize() { system.initialize(); }

    void simulate_for(double time) { simulator.simulate_for(time); }

    void execute_startup()
    {
        system.grid_side_control.state_machine.get_parameters().turn_on_command.write(true);
        simulate_for(0.5);
        system.grid_io_mock.get_parameters().start_command.write(true);
        simulate_for(0.1);
    }

    plant_complete_sfc_closed_loop_simulation_builder system;
    yeet::simulator simulator = yeet::make_gtest_simulator();
};

TEST_F(SoftStarterTest, StartupToNominalSpeed)
{
    // Start the SFC sequence
    execute_startup();

    system.plant.get_parameters().constant_torque.write(torque_pu_t{0.05f});

    // The sequence is:
    // 1. Pre-fluxing: 1.5s
    // 2. Starting Procedure (Pulse Mode + Ramp): 6.0s (up to 0.1 pu)
    // 3. Acceleration to 1.0 pu at 60s/pu: 0.9 pu * 60s/pu = 54.0s
    // Total estimated time: 1.5 + 6.0 + 54.0 = 61.5 seconds.
    // We simulate for 100 seconds to ensure it settles.

    simulate_for(100.0);

    const auto speed = system.plant.get_outputs().machine_speed.read();
    // Machine is 50 Hertz. We control to 10 Hz (1.0 pu), hence the target is 0.2 pu of the machine-side speed.
    EXPECT_NEAR(speed.value(), 0.2f, 0.05f);
    // But for the machine-side, the speed is 1.0 pu.
    EXPECT_NEAR(system.machine_side_control.get_estimated_speed().read().value(), 1.0f, 0.02f);
    EXPECT_NEAR(system.machine_side_control.machine_side_observer.get_outputs().estimated_frequency_pu.read().value(),
        1.0f, 0.01f);
    EXPECT_NEAR(system.machine_side_control.machine_side_observer.get_outputs().estimated_frequency.read().value(),
        10.0f, 0.1f);
}
