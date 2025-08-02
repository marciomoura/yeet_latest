#pragma once

#include "sfc_integration/sfc_current_controller_runner.hpp"
#include "sfc_integration/sfc_dc_link_measurements_runner.hpp"
#include "sfc_integration/sfc_grid_side_measurements_runner.hpp"
#include "sfc_integration/sfc_grid_side_modulator_runner.hpp"
#include "sfc_integration/sfc_grid_side_observer_runner.hpp"
#include "sfc_integration/sfc_machine_side_measurements_runner.hpp"
#include "sfc_integration/sfc_machine_side_modulator_runner.hpp"
#include "sfc_integration/sfc_machine_side_observer_runner.hpp"
#include "sfc_integration/sfc_plant_model_runner.hpp"
#include "sfc_integration/sfc_speed_controller_runner.hpp"
#include "yeet/application_datamodel.hpp"
#include "yeet/application_interface.hpp"
#include "yeet/data_model_connector.hpp"
#include "yeet/model_registry.hpp"
#include "yeet/parameter.hpp"
#include "yeet/signal.hpp"
#include "yeet_testing/parameter_collection.hpp"
#include "yeet_testing/simulator.hpp"

namespace yeet::application::sfc::integration {

// ==================== Data Model Type Aliases ====================

using speed_controller_datamodel =
    yeet::signal_flow_datamodel<sfc_speed_controller_datamodel::inputs, sfc_speed_controller_datamodel::outputs>;

using current_controller_datamodel =
    yeet::signal_flow_datamodel<sfc_current_controller_datamodel::inputs, sfc_current_controller_datamodel::outputs>;

using grid_observer_datamodel =
    yeet::signal_flow_datamodel<sfc_grid_side_observer_datamodel::inputs, sfc_grid_side_observer_datamodel::outputs>;

using grid_modulator_datamodel =
    yeet::signal_flow_datamodel<sfc_grid_side_modulator_datamodel::inputs, sfc_grid_side_modulator_datamodel::outputs>;

using grid_measurements_datamodel = yeet::signal_flow_datamodel<sfc_grid_side_measurements_datamodel::inputs,
                                                                sfc_grid_side_measurements_datamodel::outputs>;

using machine_observer_datamodel = yeet::signal_flow_datamodel<sfc_machine_side_observer_datamodel::inputs,
                                                               sfc_machine_side_observer_datamodel::outputs>;

using machine_modulator_datamodel = yeet::signal_flow_datamodel<sfc_machine_side_modulator_datamodel::inputs,
                                                                sfc_machine_side_modulator_datamodel::outputs>;

using machine_measurements_datamodel = yeet::signal_flow_datamodel<sfc_machine_side_measurements_datamodel::inputs,
                                                                   sfc_machine_side_measurements_datamodel::outputs>;

using dc_link_measurements_datamodel = yeet::signal_flow_datamodel<sfc_dc_link_measurements_datamodel::inputs,
                                                                   sfc_dc_link_measurements_datamodel::outputs>;

using plant_datamodel =
    yeet::signal_flow_datamodel<sfc_plant_model_datamodel::inputs, sfc_plant_model_datamodel::outputs>;

// ==================== Speed Control Loop Connectors ====================

/**
 * @brief Connects machine-side observer outputs to speed controller inputs
 *
 * This connector provides the measured speed from the machine observer to the speed controller
 * for the outer control loop feedback.
 */
class machine_observer_to_speed_controller_connector
    : public yeet::data_model_connector<speed_controller_datamodel, machine_observer_datamodel> {
public:
    machine_observer_to_speed_controller_connector(speed_controller_datamodel& speed_ctrl,
                                                   machine_observer_datamodel& machine_obs)
        : data_model_connector(speed_ctrl.get_inputs(), machine_obs.get_outputs()) {}

    void connect() override {
        // Connect estimated speed from observer to speed controller
        yeet::signals_connector::connect_output_to_input(get_outputs().estimated_speed_pu,
                                                         get_inputs().measured_speed_pu);
    }
};

/**
 * @brief Connects speed controller output to current controller input
 *
 * This connector transfers the current reference from the speed controller to the
 * current controller, forming the cascade control structure.
 */
class speed_controller_to_current_controller_connector
    : public yeet::data_model_connector<current_controller_datamodel, speed_controller_datamodel> {
public:
    speed_controller_to_current_controller_connector(current_controller_datamodel& current_ctrl,
                                                     speed_controller_datamodel& speed_ctrl)
        : data_model_connector(current_ctrl.get_inputs(), speed_ctrl.get_outputs()) {}

    void connect() override {
        // Connect current reference from speed controller to current controller
        yeet::signals_connector::connect_output_to_input(get_outputs().current_reference,
                                                         get_inputs().current_reference_pu);
    }
};

// ==================== Current Control Loop Connectors ====================

/**
 * @brief Connects DC link measurements to current controller
 *
 * This connector provides the measured DC link current for current control loop feedback.
 */
class dc_link_to_current_controller_connector
    : public yeet::data_model_connector<current_controller_datamodel, dc_link_measurements_datamodel> {
public:
    dc_link_to_current_controller_connector(current_controller_datamodel& current_ctrl,
                                            dc_link_measurements_datamodel& dc_link)
        : data_model_connector(current_ctrl.get_inputs(), dc_link.get_outputs()) {}

    void connect() override {
        // Connect measured DC link current to current controller
        yeet::signals_connector::connect_output_to_input(get_outputs().dc_current_filtered_pu,
                                                         get_inputs().dc_link_current_measured_pu);
    }
};

/**
 * @brief Connects current controller output to grid-side modulator
 *
 * This connector transfers the firing angle reference from the current controller to
 * the grid-side modulator for pulse generation.
 */
class current_controller_to_grid_modulator_connector
    : public yeet::data_model_connector<grid_modulator_datamodel, current_controller_datamodel> {
public:
    current_controller_to_grid_modulator_connector(grid_modulator_datamodel& grid_mod,
                                                   current_controller_datamodel& current_ctrl)
        : data_model_connector(grid_mod.get_inputs(), current_ctrl.get_outputs()) {}

    void connect() override {
        // Connect firing angle reference from current controller to grid modulator
        yeet::signals_connector::connect_output_to_input(get_outputs().firing_angle_reference,
                                                         get_inputs().firing_angle_reference);
    }
};

// ==================== Grid-Side Observer Chain Connectors ====================

/**
 * @brief Connects grid-side measurements to grid-side observer
 *
 * This connector provides filtered grid voltage measurements to the PLL-based observer
 * for grid synchronization.
 */
class grid_measurements_to_grid_observer_connector
    : public yeet::data_model_connector<grid_observer_datamodel, grid_measurements_datamodel> {
public:
    grid_measurements_to_grid_observer_connector(grid_observer_datamodel& grid_obs,
                                                 grid_measurements_datamodel& grid_meas)
        : data_model_connector(grid_obs.get_inputs(), grid_meas.get_outputs()) {}

    void connect() override {
        // Connect filtered grid voltages to observer
        yeet::signals_connector::connect_output_to_input(get_outputs().grid_voltage_abc_pu,
                                                         get_inputs().grid_voltage_pu);
    }
};

/**
 * @brief Connects grid-side observer outputs to grid-side modulator
 *
 * This connector provides grid angle and frequency information from the observer to
 * the modulator for synchronized pulse generation.
 */
class grid_observer_to_grid_modulator_connector
    : public yeet::data_model_connector<grid_modulator_datamodel, grid_observer_datamodel> {
public:
    grid_observer_to_grid_modulator_connector(grid_modulator_datamodel& grid_mod, grid_observer_datamodel& grid_obs)
        : data_model_connector(grid_mod.get_inputs(), grid_obs.get_outputs()) {}

    void connect() override {
        // Connect estimated grid angle to modulator
        yeet::signals_connector::connect_output_to_input(get_outputs().estimated_angle,
                                                         get_inputs().grid_voltage_angle);
        // Connect estimated frequency for adaptive pulse generation
        yeet::signals_connector::connect_output_to_input(get_outputs().estimated_frequency,
                                                         get_inputs().grid_frequency);
    }
};

// ==================== Machine-Side Observer Chain Connectors ====================

/**
 * @brief Connects machine-side measurements to machine-side observer
 *
 * This connector provides filtered machine voltage measurements to the adaptive observer
 * for machine speed and position estimation.
 */
class machine_measurements_to_machine_observer_connector
    : public yeet::data_model_connector<machine_observer_datamodel, machine_measurements_datamodel> {
public:
    machine_measurements_to_machine_observer_connector(machine_observer_datamodel& machine_obs,
                                                       machine_measurements_datamodel& machine_meas)
        : data_model_connector(machine_obs.get_inputs(), machine_meas.get_outputs()) {}

    void connect() override {
        // Connect filtered machine voltages to observer
        yeet::signals_connector::connect_output_to_input(get_outputs().machine_voltage_abc_pu,
                                                         get_inputs().machine_voltage_pu);
    }
};

/**
 * @brief Connects machine-side observer outputs to machine-side modulator
 *
 * This connector provides rotor position information from the observer to the modulator
 * for position-synchronized pulse generation.
 */
class machine_observer_to_machine_modulator_connector
    : public yeet::data_model_connector<machine_modulator_datamodel, machine_observer_datamodel> {
public:
    machine_observer_to_machine_modulator_connector(machine_modulator_datamodel& machine_mod,
                                                    machine_observer_datamodel& machine_obs)
        : data_model_connector(machine_mod.get_inputs(), machine_obs.get_outputs()) {}

    void connect() override {
        // Connect estimated rotor position to machine modulator
        yeet::signals_connector::connect_output_to_input(get_outputs().estimated_position,
                                                         get_inputs().rotor_position_electrical);
    }
};

/**
 * @brief Connects DC link measurements to machine-side modulator
 *
 * This connector provides DC current information to the machine modulator for
 * firing angle lookup table operation and advance angle calculation.
 */
class dc_link_to_machine_modulator_connector
    : public yeet::data_model_connector<machine_modulator_datamodel, dc_link_measurements_datamodel> {
public:
    dc_link_to_machine_modulator_connector(machine_modulator_datamodel& machine_mod,
                                           dc_link_measurements_datamodel& dc_link)
        : data_model_connector(machine_mod.get_inputs(), dc_link.get_outputs()) {}

    void connect() override {
        // Connect DC link current to machine modulator
        yeet::signals_connector::connect_output_to_input(get_outputs().dc_current_filtered_pu,
                                                         get_inputs().dc_link_current_pu);
    }
};

// ==================== Plant Model Input Connectors ====================

/**
 * @brief Connects plant model outputs to grid-side measurements
 *
 * This connector provides raw electrical measurements from the plant model to the
 * grid-side measurement and filtering component.
 */
class plant_to_grid_measurements_connector
    : public yeet::data_model_connector<grid_measurements_datamodel, plant_datamodel> {
public:
    plant_to_grid_measurements_connector(grid_measurements_datamodel& grid_meas, plant_datamodel& plant)
        : data_model_connector(grid_meas.get_inputs(), plant.get_outputs()) {}

    void connect() override {
        // Connect grid voltages and currents from plant to measurements
        yeet::signals_connector::connect_output_to_input(get_outputs().grid_voltage_ln, get_inputs().line_voltages_raw);
        yeet::signals_connector::connect_output_to_input(get_outputs().grid_current, get_inputs().line_currents_raw);
        yeet::signals_connector::connect_output_to_input(get_outputs().dc_link_voltage,
                                                         get_inputs().dc_link_voltage_raw);
    }
};

/**
 * @brief Connects plant model outputs to machine-side measurements
 *
 * This connector provides raw machine electrical measurements from the plant model to
 * the machine-side measurement and filtering component.
 */
class plant_to_machine_measurements_connector
    : public yeet::data_model_connector<machine_measurements_datamodel, plant_datamodel> {
public:
    plant_to_machine_measurements_connector(machine_measurements_datamodel& machine_meas, plant_datamodel& plant)
        : data_model_connector(machine_meas.get_inputs(), plant.get_outputs()) {}

    void connect() override {
        // Connect machine voltages and currents from plant to measurements
        yeet::signals_connector::connect_output_to_input(get_outputs().machine_voltage_ln,
                                                         get_inputs().machine_voltages_raw);
        yeet::signals_connector::connect_output_to_input(get_outputs().machine_current,
                                                         get_inputs().machine_currents_raw);
        // Connect mechanical measurements
        yeet::signals_connector::connect_output_to_input(get_outputs().rotor_speed_mechanical,
                                                         get_inputs().machine_speed_raw);
        yeet::signals_connector::connect_output_to_input(get_outputs().rotor_position_mechanical,
                                                         get_inputs().rotor_position_raw);
    }
};

/**
 * @brief Connects plant model outputs to DC link measurements
 *
 * This connector provides raw DC link measurements from the plant model to the
 * DC link measurement and filtering component.
 */
class plant_to_dc_link_measurements_connector
    : public yeet::data_model_connector<dc_link_measurements_datamodel, plant_datamodel> {
public:
    plant_to_dc_link_measurements_connector(dc_link_measurements_datamodel& dc_link, plant_datamodel& plant)
        : data_model_connector(dc_link.get_inputs(), plant.get_outputs()) {}

    void connect() override {
        // Connect DC link measurements from plant
        yeet::signals_connector::connect_output_to_input(get_outputs().dc_link_voltage, get_inputs().dc_voltage);
        yeet::signals_connector::connect_output_to_input(get_outputs().dc_link_current, get_inputs().dc_current);
    }
};

// ==================== Plant Model Output Connectors ====================

/**
 * @brief Connects modulator outputs to plant model inputs
 *
 * This connector transfers firing pulses from both grid and machine modulators to
 * the plant model for converter operation simulation.
 */
class modulators_to_plant_connector {
private:
    plant_datamodel& _plant;
    grid_modulator_datamodel& _grid_mod;
    machine_modulator_datamodel& _machine_mod;

public:
    modulators_to_plant_connector(plant_datamodel& plant, grid_modulator_datamodel& grid_mod,
                                  machine_modulator_datamodel& machine_mod)
        : _plant(plant), _grid_mod(grid_mod), _machine_mod(machine_mod) {}

    void connect() {
        // Connect grid-side firing pulses
        yeet::signals_connector::connect_output_to_input(_grid_mod.get_outputs().firing_pulses,
                                                         _plant.get_inputs().grid_side_firing_pulses);
        // Connect machine-side firing pulses
        yeet::signals_connector::connect_output_to_input(_machine_mod.get_outputs().firing_pulses,
                                                         _plant.get_inputs().machine_side_firing_pulses);
    }
};

// ==================== Feedforward and Feedback Connectors ====================

/**
 * @brief Connects grid observer to current controller for feedforward compensation
 *
 * This connector provides ideal DC voltage estimation from the grid observer to the
 * current controller for improved dynamic response.
 */
class grid_observer_to_current_controller_connector
    : public yeet::data_model_connector<current_controller_datamodel, grid_observer_datamodel> {
public:
    grid_observer_to_current_controller_connector(current_controller_datamodel& current_ctrl,
                                                  grid_observer_datamodel& grid_obs)
        : data_model_connector(current_ctrl.get_inputs(), grid_obs.get_outputs()) {}

    void connect() override {
        // Connect ideal DC voltage for feedforward compensation
        yeet::signals_connector::connect_output_to_input(get_outputs().estimated_voltage_magnitude,
                                                         get_inputs().ideal_dc_voltage_pu);
    }
};

/**
 * @brief Connects machine observer back-EMF estimation to current controller
 *
 * This connector provides estimated back-EMF from the machine observer to the current
 * controller for feedforward compensation of machine-side effects.
 */
class machine_observer_to_current_controller_connector
    : public yeet::data_model_connector<current_controller_datamodel, machine_observer_datamodel> {
public:
    machine_observer_to_current_controller_connector(current_controller_datamodel& current_ctrl,
                                                     machine_observer_datamodel& machine_obs)
        : data_model_connector(current_ctrl.get_inputs(), machine_obs.get_outputs()) {}

    void connect() override {
        // Connect estimated back-EMF from machine observer to current controller
        yeet::signals_connector::connect_output_to_input(get_outputs().estimated_voltage_magnitude,
                                                         get_inputs().inverter_back_emf_machine_pu);
    }
};

/**
 * @brief Connects firing pulses back to observers for commutation detection
 *
 * This connector provides firing pulse information from modulators back to observers
 * for commutation event detection and timing synchronization.
 */
class modulators_to_observers_connector {
private:
    grid_observer_datamodel& _grid_obs;
    machine_observer_datamodel& _machine_obs;
    grid_modulator_datamodel& _grid_mod;
    machine_modulator_datamodel& _machine_mod;

public:
    modulators_to_observers_connector(grid_observer_datamodel& grid_obs, machine_observer_datamodel& machine_obs,
                                      grid_modulator_datamodel& grid_mod, machine_modulator_datamodel& machine_mod)
        : _grid_obs(grid_obs), _machine_obs(machine_obs), _grid_mod(grid_mod), _machine_mod(machine_mod) {}

    void connect() {
        // Connect grid-side firing pulses to grid observer
        yeet::signals_connector::connect_output_to_input(_grid_mod.get_outputs().firing_pulses,
                                                         _grid_obs.get_inputs().grid_side_pulses);
        // Connect machine-side firing pulses to machine observer
        yeet::signals_connector::connect_output_to_input(_machine_mod.get_outputs().firing_pulses,
                                                         _machine_obs.get_inputs().machine_side_pulses);
    }
};

// ==================== Parameter Collection Types ====================

using closed_loop_sfc_parameter_types = yeet::testing::parameter_collection<
    sfc_speed_controller_datamodel::parameters, sfc_current_controller_datamodel::parameters,
    sfc_grid_side_observer_datamodel::parameters, sfc_grid_side_modulator_datamodel::parameters,
    sfc_machine_side_observer_datamodel::parameters, sfc_machine_side_modulator_datamodel::parameters,
    sfc_grid_side_measurements_datamodel::parameters, sfc_machine_side_measurements_datamodel::parameters,
    sfc_dc_link_measurements_datamodel::parameters, sfc_plant_model_datamodel::parameters>;

// ==================== Closed-Loop Configuration ====================

/**
 * @brief Complete closed-loop SFC control system configuration
 *
 * This class implements a complete SFC/LCI control system by interconnecting all
 * control components, measurement systems, observers, modulators, and the plant model.
 * It follows the hierarchical control structure with speed control (outer loop),
 * current control (inner loop), and firing pulse generation (modulation layer).
 */
class closed_loop_sfc_with_plant_model
    : public yeet::testing::closed_loop_configuration<closed_loop_sfc_parameter_types> {
public:
    using parameter_types = closed_loop_sfc_parameter_types;

    closed_loop_sfc_with_plant_model() {
        // Create shared per-unit system for consistent unit handling
        _per_unit_system =
            std::make_shared<sfc::sfc_per_unit_system>(voltage_t(2090.0),  // Grid voltage RMS (Line-to-Line)
                                                       frequency_t(50.0),  // Grid frequency
                                                       current_t(1044.0),  // DC link current base
                                                       power_t(2950e3),    // Machine power
                                                       voltage_t(1980.0),  // Machine voltage RMS (Line-to-Line)
                                                       frequency_t(50.0),  // Machine frequency
                                                       2                   // Number of pole pairs
            );

        // Initialize all runners
        initialize_runners();

        // Connect all datamodel components
        connect_all_datamodels();
    }

    void initialize() override {
        // Initialize all components in proper sequence
        // 1. Measurements first (they provide sensor validation)
        grid_measurements.initialize();
        machine_measurements.initialize();
        dc_link_measurements.initialize();

        // 2. Observers (they need measurements)
        grid_observer.initialize();
        machine_observer.initialize();

        // 3. Controllers (they need observer outputs)
        current_controller.initialize();
        speed_controller.initialize();

        // 4. Modulators (they need controller outputs)
        grid_modulator.initialize();
        machine_modulator.initialize();

        // 5. Plant model last (it needs modulator outputs)
        plant.initialize();
    }

    void register_applications(yeet::testing::simulator& sim) override {
        // Register all runners with their respective sampling times
        sim.register_lambda_runner(speed_controller.get_sampling_time(), [&]() { speed_controller.run(); });
        sim.register_lambda_runner(current_controller.get_sampling_time(), [&]() { current_controller.run(); });
        sim.register_lambda_runner(grid_observer.get_sampling_time(), [&]() { grid_observer.run(); });
        sim.register_lambda_runner(grid_modulator.get_sampling_time(), [&]() { grid_modulator.run(); });
        sim.register_lambda_runner(machine_observer.get_sampling_time(), [&]() { machine_observer.run(); });
        sim.register_lambda_runner(machine_modulator.get_sampling_time(), [&]() { machine_modulator.run(); });
        sim.register_lambda_runner(grid_measurements.get_sampling_time(), [&]() { grid_measurements.run(); });
        sim.register_lambda_runner(machine_measurements.get_sampling_time(), [&]() { machine_measurements.run(); });
        sim.register_lambda_runner(dc_link_measurements.get_sampling_time(), [&]() { dc_link_measurements.run(); });
        sim.register_lambda_runner(plant.get_sampling_time(), [&]() { plant.run(); });
    }

    void register_datamodel(yeet::model_registry& registry) override {
        // Register all datamodels with the model registry
        speed_controller.register_datamodel(registry);
        current_controller.register_datamodel(registry);
        grid_observer.register_datamodel(registry);
        grid_modulator.register_datamodel(registry);
        machine_observer.register_datamodel(registry);
        machine_modulator.register_datamodel(registry);
        grid_measurements.register_datamodel(registry);
        machine_measurements.register_datamodel(registry);
        dc_link_measurements.register_datamodel(registry);
        plant.register_datamodel(registry);
    }

    parameter_types create_parameter_collection() override {
        return parameter_types(speed_controller.get_configuration_datamodel().get_parameters(),
                               current_controller.get_configuration_datamodel().get_parameters(),
                               grid_observer.get_configuration_datamodel().get_parameters(),
                               grid_modulator.get_configuration_datamodel().get_parameters(),
                               machine_observer.get_configuration_datamodel().get_parameters(),
                               machine_modulator.get_configuration_datamodel().get_parameters(),
                               grid_measurements.get_configuration_datamodel().get_parameters(),
                               machine_measurements.get_configuration_datamodel().get_parameters(),
                               dc_link_measurements.get_configuration_datamodel().get_parameters(),
                               plant.get_configuration_datamodel().get_parameters());
    }

    // Public members for direct access and testing
    sfc_speed_controller_runner speed_controller;
    sfc_current_controller_runner current_controller;
    sfc_grid_side_observer_runner grid_observer;
    sfc_grid_side_modulator_runner grid_modulator;
    sfc_machine_side_observer_runner machine_observer;
    sfc_machine_side_modulator_runner machine_modulator;
    sfc_grid_side_measurements_runner grid_measurements{_per_unit_system};
    sfc_machine_side_measurements_runner machine_measurements{_per_unit_system};
    sfc_dc_link_measurements_runner dc_link_measurements{_per_unit_system};
    sfc_plant_model_runner plant{_per_unit_system};

private:
    std::shared_ptr<sfc::sfc_per_unit_system> _per_unit_system;

    void initialize_runners() {
        // Runners requiring per-unit system are initialized in member initializer list
        // Other runners use default construction
    }

    void connect_all_datamodels() {
        // Get datamodel references for cleaner code
        auto& speed_ctrl_model = speed_controller.get_signal_flow_datamodel();
        auto& current_ctrl_model = current_controller.get_signal_flow_datamodel();
        auto& grid_obs_model = grid_observer.get_signal_flow_datamodel();
        auto& grid_mod_model = grid_modulator.get_signal_flow_datamodel();
        auto& machine_obs_model = machine_observer.get_signal_flow_datamodel();
        auto& machine_mod_model = machine_modulator.get_signal_flow_datamodel();
        auto& grid_meas_model = grid_measurements.get_signal_flow_datamodel();
        auto& machine_meas_model = machine_measurements.get_signal_flow_datamodel();
        auto& dc_link_model = dc_link_measurements.get_signal_flow_datamodel();
        auto& plant_model = plant.get_signal_flow_datamodel();

        // ========== Speed Control Loop Connections ==========
        machine_observer_to_speed_controller_connector machine_to_speed(speed_ctrl_model, machine_obs_model);
        machine_to_speed.connect();

        speed_controller_to_current_controller_connector speed_to_current(current_ctrl_model, speed_ctrl_model);
        speed_to_current.connect();

        // ========== Current Control Loop Connections ==========
        dc_link_to_current_controller_connector dc_to_current(current_ctrl_model, dc_link_model);
        dc_to_current.connect();

        current_controller_to_grid_modulator_connector current_to_grid_mod(grid_mod_model, current_ctrl_model);
        current_to_grid_mod.connect();

        // ========== Grid-Side Observer Chain ==========
        grid_measurements_to_grid_observer_connector grid_meas_to_obs(grid_obs_model, grid_meas_model);
        grid_meas_to_obs.connect();

        grid_observer_to_grid_modulator_connector grid_obs_to_mod(grid_mod_model, grid_obs_model);
        grid_obs_to_mod.connect();

        // ========== Machine-Side Observer Chain ==========
        machine_measurements_to_machine_observer_connector machine_meas_to_obs(machine_obs_model, machine_meas_model);
        machine_meas_to_obs.connect();

        machine_observer_to_machine_modulator_connector machine_obs_to_mod(machine_mod_model, machine_obs_model);
        machine_obs_to_mod.connect();

        // ========== Machine-Side Modulator Additional Connections ==========
        dc_link_to_machine_modulator_connector dc_to_machine_mod(machine_mod_model, dc_link_model);
        dc_to_machine_mod.connect();

        // ========== Plant Model Input Connections ==========
        plant_to_grid_measurements_connector plant_to_grid(grid_meas_model, plant_model);
        plant_to_grid.connect();

        plant_to_machine_measurements_connector plant_to_machine(machine_meas_model, plant_model);
        plant_to_machine.connect();

        plant_to_dc_link_measurements_connector plant_to_dc(dc_link_model, plant_model);
        plant_to_dc.connect();

        // ========== Plant Model Output Connections ==========
        modulators_to_plant_connector mods_to_plant(plant_model, grid_mod_model, machine_mod_model);
        mods_to_plant.connect();

        // ========== Feedforward and Feedback Connections ==========
        grid_observer_to_current_controller_connector grid_obs_to_current(current_ctrl_model, grid_obs_model);
        grid_obs_to_current.connect();

        machine_observer_to_current_controller_connector machine_obs_to_current(current_ctrl_model, machine_obs_model);
        machine_obs_to_current.connect();

        modulators_to_observers_connector mods_to_obs(grid_obs_model, machine_obs_model, grid_mod_model,
                                                      machine_mod_model);
        mods_to_obs.connect();
    }
};

}  // namespace yeet::application::sfc::integration
