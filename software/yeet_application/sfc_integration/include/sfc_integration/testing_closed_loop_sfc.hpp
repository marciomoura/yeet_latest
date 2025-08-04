#pragma once

#include "sfc_grid_side_pulse_generator_datamodel.hpp"
#include "sfc_integration/sfc_current_controller_runner.hpp"
#include "sfc_integration/sfc_dc_link_measurements_runner.hpp"
#include "sfc_integration/sfc_grid_side_measurements_runner.hpp"
#include "sfc_integration/sfc_grid_side_modulator_datamodel.hpp"
#include "sfc_integration/sfc_grid_side_modulator_runner.hpp"
#include "sfc_integration/sfc_grid_side_observer_runner.hpp"
#include "sfc_integration/sfc_grid_side_pulse_generator_runner.hpp"
#include "sfc_integration/sfc_machine_side_measurements_runner.hpp"
#include "sfc_integration/sfc_machine_side_modulator_datamodel.hpp"
#include "sfc_integration/sfc_machine_side_modulator_runner.hpp"
#include "sfc_integration/sfc_machine_side_observer_runner.hpp"
#include "sfc_integration/sfc_machine_side_pulse_generator_runner.hpp"
#include "sfc_integration/sfc_plant_model_runner.hpp"
#include "sfc_integration/sfc_speed_controller_runner.hpp"
#include "sfc_machine_side_pulse_generator_datamodel.hpp"
#include "yeet/data_model_connector.hpp"
#include "yeet/model_registry.hpp"
#include "yeet_testing/parameter_collection.hpp"
#include "yeet_testing/simulator.hpp"

namespace yeet::application::sfc::integration {

// Type aliases for datamodels - these define the signal contracts

// Speed and current controller datamodels

using speed_controller_datamodel =
    yeet::signal_flow_datamodel<sfc_speed_controller_datamodel::inputs, sfc_speed_controller_datamodel::outputs>;

using current_controller_datamodel =
    yeet::signal_flow_datamodel<sfc_current_controller_datamodel::inputs, sfc_current_controller_datamodel::outputs>;

// DC link measurements datamodel

using dc_link_measurements_datamodel = yeet::signal_flow_datamodel<sfc_dc_link_measurements_datamodel::inputs,
    sfc_dc_link_measurements_datamodel::outputs>;

// Grid side datamodels

using grid_measurements_datamodel = yeet::signal_flow_datamodel<sfc_grid_side_measurements_datamodel::inputs,
    sfc_grid_side_measurements_datamodel::outputs>;

using grid_observer_datamodel =
    yeet::signal_flow_datamodel<sfc_grid_side_observer_datamodel::inputs, sfc_grid_side_observer_datamodel::outputs>;

using grid_modulator_datamodel =
    yeet::signal_flow_datamodel<sfc_grid_side_modulator_datamodel::inputs, sfc_grid_side_modulator_datamodel::outputs>;

// Machine side datamodels

using machine_measurements_datamodel = yeet::signal_flow_datamodel<sfc_machine_side_measurements_datamodel::inputs,
    sfc_machine_side_measurements_datamodel::outputs>;

using machine_observer_datamodel = yeet::signal_flow_datamodel<sfc_machine_side_observer_datamodel::inputs,
    sfc_machine_side_observer_datamodel::outputs>;

using machine_modulator_datamodel = yeet::signal_flow_datamodel<sfc_machine_side_modulator_datamodel::inputs,
    sfc_machine_side_modulator_datamodel::outputs>;

// Pulse generator datamodels

using grid_side_pulse_generator_datamodel = yeet::signal_flow_datamodel<sfc_grid_side_pulse_generator_datamodel::inputs,
    sfc_grid_side_pulse_generator_datamodel::outputs>;

using machine_side_pulse_generator_datamodel =
    yeet::signal_flow_datamodel<sfc_machine_side_pulse_generator_datamodel::inputs,
        sfc_machine_side_pulse_generator_datamodel::outputs>;

// Plant model datamodel

using plant_datamodel =
    yeet::signal_flow_datamodel<sfc_plant_model_datamodel::inputs, sfc_plant_model_datamodel::outputs>;

// ==================== Aggregated Connector Classes ====================

/**
 * @brief Aggregated connector for all inputs to the speed controller
 *
 * Signal Flow: Machine Observer → Speed Controller
 * Key Connections:
 * - Machine Observer: estimated_speed_pu → measured_speed_pu
 */
class to_speed_controller_connector
    : public data_model_connector<inputs<speed_controller_datamodel>, outputs<machine_observer_datamodel>> {
public:
    to_speed_controller_connector(speed_controller_datamodel& speed_ctrl, machine_observer_datamodel& machine_obs)
        : data_model_connector(speed_ctrl.get_inputs(), machine_obs.get_outputs())
    {
    }

    void connect() override
    {
        // Connect machine observer outputs to speed controller
        signals_connector::connect_output_to_input(
            get_outputs<machine_observer_datamodel>().estimated_speed, get_inputs().measured_speed_pu);
    }
};

/**
 * @brief Aggregated connector for all inputs to the current controller
 *
 * Signal Flow: Speed Controller + DC Link Measurements + Grid Observer + Machine Observer → Current Controller
 * Key Connections:
 * - Speed Controller: current_reference → current_reference_pu
 * - DC Link Measurements: dc_link_current_pu → dc_link_current_measured_pu
 * - Grid Observer: ideal_dc_voltage → ideal_dc_voltage_pu (feedforward)
 * - Machine Observer: estimated_back_emf_pu → inverter_back_emf_machine_pu (feedforward)
 */
class to_current_controller_connector : public data_model_connector<inputs<current_controller_datamodel>,
                                            outputs<speed_controller_datamodel>,
                                            outputs<dc_link_measurements_datamodel>,
                                            outputs<grid_observer_datamodel>,
                                            outputs<machine_observer_datamodel>> {
public:
    to_current_controller_connector(current_controller_datamodel& current_ctrl,
        speed_controller_datamodel& speed_ctrl,
        dc_link_measurements_datamodel& dc_link,
        grid_observer_datamodel& grid_obs,
        machine_observer_datamodel& machine_obs)
        : data_model_connector(current_ctrl.get_inputs(),
              speed_ctrl.get_outputs(),
              dc_link.get_outputs(),
              grid_obs.get_outputs(),
              machine_obs.get_outputs())
    {
    }

    void connect() override
    {
        // Connect speed controller output (cascade control)
        signals_connector::connect_output_to_input(
            get_outputs<speed_controller_datamodel>().current_reference, get_inputs().current_reference_pu);

        // Connect DC link measurements (feedback)
        signals_connector::connect_output_to_input(get_outputs<dc_link_measurements_datamodel>().dc_current_filtered_pu,
            get_inputs().dc_link_current_measured_pu);

        // Connect grid observer feedforward
        signals_connector::connect_output_to_input(
            get_outputs<grid_observer_datamodel>().estimated_voltage_magnitude, get_inputs().ideal_dc_voltage_pu);

        // Connect machine observer feedforward
        signals_connector::connect_output_to_input(
            get_outputs<machine_observer_datamodel>().estimated_voltage_magnitude,
            get_inputs().inverter_back_emf_machine_pu);
    }
};

/**
 * @brief Aggregated connector for all inputs to the grid observer
 *
 * Signal Flow: Grid Measurements → Grid Observer
 * Key Connections:
 * - Grid Measurements: grid_voltage_abc_pu → grid_voltage_pu
 */
class to_grid_observer_connector
    : public data_model_connector<inputs<grid_observer_datamodel>, outputs<grid_measurements_datamodel>> {
public:
    to_grid_observer_connector(grid_observer_datamodel& grid_obs, grid_measurements_datamodel& grid_meas)
        : data_model_connector(grid_obs.get_inputs(), grid_meas.get_outputs())
    {
    }

    void connect() override
    {
        // Connect grid measurements to observer
        signals_connector::connect_output_to_input(
            get_outputs<grid_measurements_datamodel>().grid_voltage_abc_pu, get_inputs().grid_voltage_pu);
    }
};

/**
 * @brief Aggregated connector for all inputs to the grid modulator
 *
 * Signal Flow: Current Controller + Grid Observer → Grid Modulator
 * Key Connections:
 * - Current Controller: firing_angle_reference → firing_angle_reference
 * - Grid Observer: estimated_angle → grid_voltage_angle, estimated_frequency → grid_frequency
 */
class to_grid_modulator_connector : public data_model_connector<inputs<grid_modulator_datamodel>,
                                        outputs<current_controller_datamodel>,
                                        outputs<grid_observer_datamodel>> {
public:
    to_grid_modulator_connector(grid_modulator_datamodel& grid_mod,
        current_controller_datamodel& current_ctrl,
        grid_observer_datamodel& grid_obs)
        : data_model_connector(grid_mod.get_inputs(), current_ctrl.get_outputs(), grid_obs.get_outputs())
    {
    }

    void connect() override
    {
        // Connect current controller output
        signals_connector::connect_output_to_input(
            get_outputs<current_controller_datamodel>().firing_angle_reference, get_inputs().firing_angle_reference);

        // Connect grid observer synchronization signals
        signals_connector::connect_output_to_input(
            get_outputs<grid_observer_datamodel>().estimated_angle, get_inputs().grid_voltage_angle);
        signals_connector::connect_output_to_input(
            get_outputs<grid_observer_datamodel>().estimated_frequency, get_inputs().grid_frequency);

        // TODO: Connect enable command from system controller when available
        // For now, enable command will be handled in the runner
    }
};

/**
 * @brief Aggregated connector for all inputs to the machine observer
 *
 * Signal Flow: Machine Measurements → Machine Observer
 * Key Connections:
 * - Machine Measurements: machine_voltage_abc_pu → machine_voltage_pu, machine_current_abc_pu → machine_current_pu
 */
class to_machine_observer_connector
    : public data_model_connector<inputs<machine_observer_datamodel>, outputs<machine_measurements_datamodel>> {
public:
    to_machine_observer_connector(machine_observer_datamodel& machine_obs, machine_measurements_datamodel& machine_meas)
        : data_model_connector(machine_obs.get_inputs(), machine_meas.get_outputs())
    {
    }

    void connect() override
    {
        // Connect machine measurements to observer
        signals_connector::connect_output_to_input(
            get_outputs<machine_measurements_datamodel>().machine_voltage_abc_pu, get_inputs().machine_voltage_pu);
        // signals_connector::connect_output_to_input(
        // get_outputs<machine_measurements_datamodel>().machine_current_abc_pu, get_inputs().);
    }
};

/**
 * @brief Aggregated connector for all inputs to the machine modulator
 *
 * Signal Flow: DC Link Measurements + Machine Observer → Machine Modulator
 * Key Connections:
 * - DC Link Measurements: dc_link_current_pu → lookup_table_input (for firing angle calculation)
 * - Machine Observer: estimated_position → rotor_position_electrical
 */
class to_machine_modulator_connector : public data_model_connector<inputs<machine_modulator_datamodel>,
                                           outputs<dc_link_measurements_datamodel>,
                                           outputs<machine_observer_datamodel>> {
public:
    to_machine_modulator_connector(machine_modulator_datamodel& machine_mod,
        dc_link_measurements_datamodel& dc_link,
        machine_observer_datamodel& machine_obs)
        : data_model_connector(machine_mod.get_inputs(), dc_link.get_outputs(), machine_obs.get_outputs())
    {
    }

    void connect() override
    {
        // Connect DC current for lookup table firing angle calculation
        signals_connector::connect_output_to_input(
            get_outputs<dc_link_measurements_datamodel>().dc_current_filtered_pu, get_inputs().dc_link_current_pu);

        // Connect machine observer position for synchronization
        signals_connector::connect_output_to_input(
            get_outputs<machine_observer_datamodel>().estimated_position, get_inputs().rotor_position_electrical);

        // TODO: Connect lookup table firing angle when lookup table runner is implemented
        // TODO: Connect operating mode from mode detector when available
        // TODO: Connect commutation detection when available
        // For now, these signals will be handled in the runner with default values
    }
};

/**
 * @brief Aggregated connector for all inputs to grid measurements
 *
 * Signal Flow: Plant Model → Grid Measurements
 * Key Connections: Raw grid voltages, currents, and DC link voltage
 */
class to_grid_measurements_connector
    : public data_model_connector<inputs<grid_measurements_datamodel>, outputs<plant_datamodel>> {
public:
    to_grid_measurements_connector(grid_measurements_datamodel& grid_meas, plant_datamodel& plant)
        : data_model_connector(grid_meas.get_inputs(), plant.get_outputs())
    {
    }

    void connect() override
    {
        // Connect raw measurements from plant
        signals_connector::connect_output_to_input(
            get_outputs<plant_datamodel>().grid_voltage, get_inputs().line_voltages_raw);
        signals_connector::connect_output_to_input(
            get_outputs<plant_datamodel>().grid_current, get_inputs().line_currents_raw);
        signals_connector::connect_output_to_input(
            get_outputs<plant_datamodel>().dc_link_voltage, get_inputs().dc_link_voltage_raw);
    }
};

/**
 * @brief Aggregated connector for all inputs to machine measurements
 *
 * Signal Flow: Plant Model → Machine Measurements
 * Key Connections: Raw machine voltages and currents
 */
class to_machine_measurements_connector
    : public data_model_connector<inputs<machine_measurements_datamodel>, outputs<plant_datamodel>> {
public:
    to_machine_measurements_connector(machine_measurements_datamodel& machine_meas, plant_datamodel& plant)
        : data_model_connector(machine_meas.get_inputs(), plant.get_outputs())
    {
    }

    void connect() override
    {
        // Connect raw measurements from plant
        signals_connector::connect_output_to_input(
            get_outputs<plant_datamodel>().machine_voltage, get_inputs().machine_voltages_raw);
        signals_connector::connect_output_to_input(
            get_outputs<plant_datamodel>().machine_current, get_inputs().machine_currents_raw);
    }
};

/**
 * @brief Aggregated connector for all inputs to DC link measurements
 *
 * Signal Flow: Plant Model → DC Link Measurements
 * Key Connections: Raw DC link voltage and current
 */
class to_dc_link_measurements_connector
    : public data_model_connector<inputs<dc_link_measurements_datamodel>, outputs<plant_datamodel>> {
public:
    to_dc_link_measurements_connector(dc_link_measurements_datamodel& dc_link, plant_datamodel& plant)
        : data_model_connector(dc_link.get_inputs(), plant.get_outputs())
    {
    }

    void connect() override
    {
        // Connect raw measurements from plant
        signals_connector::connect_output_to_input(
            get_outputs<plant_datamodel>().dc_link_voltage, get_inputs().dc_voltage);
        signals_connector::connect_output_to_input(
            get_outputs<plant_datamodel>().dc_link_current, get_inputs().dc_current);
    }
};

/**
 * @brief Aggregated connector for all inputs to the plant model
 *
 * Signal Flow: Grid Modulator + Machine Modulator → Plant Model
 * Key Connections: Firing pulses from both modulators
 */
class to_plant_connector : public data_model_connector<inputs<plant_datamodel>,
                               outputs<grid_modulator_datamodel>,
                               outputs<machine_modulator_datamodel>> {
public:
    to_plant_connector(
        plant_datamodel& plant, grid_modulator_datamodel& grid_mod, machine_modulator_datamodel& machine_mod)
        : data_model_connector(plant.get_inputs(), grid_mod.get_outputs(), machine_mod.get_outputs())
    {
    }

    void connect() override
    {
        // Connect firing pulses from both modulators
        signals_connector::connect_output_to_input(
            get_outputs<grid_modulator_datamodel>().firing_pulses, get_inputs().grid_side_firing_pulses);
        signals_connector::connect_output_to_input(
            get_outputs<machine_modulator_datamodel>().firing_pulses, get_inputs().machine_side_firing_pulses);
    }
};

class ideal_pulse_generators_to_plant_model_connector : public data_model_connector<inputs<plant_datamodel>,
                                                            outputs<grid_side_pulse_generator_datamodel>,
                                                            outputs<machine_side_pulse_generator_datamodel>> {
public:
    ideal_pulse_generators_to_plant_model_connector(plant_datamodel& plant,
        grid_side_pulse_generator_datamodel& grid_pulse_gen,
        machine_side_pulse_generator_datamodel& machine_pulse_gen)
        : data_model_connector(plant.get_inputs(), grid_pulse_gen.get_outputs(), machine_pulse_gen.get_outputs())
    {
    }

    void connect() override
    {
        // Connect firing pulses from both modulators
        signals_connector::connect_output_to_input(
            get_outputs<grid_side_pulse_generator_datamodel>().firing_pulses, get_inputs().grid_side_firing_pulses);

        signals_connector::connect_output_to_input(get_outputs<machine_side_pulse_generator_datamodel>().firing_pulses,
            get_inputs().machine_side_firing_pulses);
    }
};

// Plant model to grid pulse generators connector
class plant_model_to_grid_pulse_generators_connector
    : public data_model_connector<inputs<grid_side_pulse_generator_datamodel>, outputs<plant_datamodel>> {
public:
    plant_model_to_grid_pulse_generators_connector(
        grid_side_pulse_generator_datamodel& grid_pulse_gen, plant_datamodel& plant)
        : data_model_connector(grid_pulse_gen.get_inputs(), plant.get_outputs())
    {
    }
    void connect() override
    {
        // Connect grid voltage source angle to pulse generator
        signals_connector::connect_output_to_input(
            get_outputs<plant_datamodel>().grid_voltage_source_angle, get_inputs().reference_angle);
    }
};

// Plant model to machine pulse generators connector
class plant_model_to_machine_pulse_generators_connector
    : public data_model_connector<inputs<machine_side_pulse_generator_datamodel>, outputs<plant_datamodel>> {
public:
    plant_model_to_machine_pulse_generators_connector(
        machine_side_pulse_generator_datamodel& machine_pulse_gen, plant_datamodel& plant)
        : data_model_connector(machine_pulse_gen.get_inputs(), plant.get_outputs())
    {
    }
    void connect() override
    {
        // Connect machine voltage source angle to pulse generator
        signals_connector::connect_output_to_input(
            get_outputs<plant_datamodel>().machine_voltage_source_angle, get_inputs().reference_angle);
    }
};

// ==================== Parameter Collection Types ====================

using closed_loop_sfc_parameter_types = yeet::parameter_collection<sfc_speed_controller_datamodel::parameters,
    sfc_current_controller_datamodel::parameters,
    sfc_grid_side_observer_datamodel::parameters,
    sfc_grid_side_modulator_datamodel::parameters,
    sfc_machine_side_observer_datamodel::parameters,
    sfc_machine_side_modulator_datamodel::parameters,
    sfc_grid_side_measurements_datamodel::parameters,
    sfc_machine_side_measurements_datamodel::parameters,
    sfc_dc_link_measurements_datamodel::parameters,
    sfc_plant_model_datamodel::parameters>;

// ==================== Closed-Loop Configuration ====================

using sfc_simulator_builder_parameters = yeet::parameter_collection<sfc_grid_side_pulse_generator_datamodel::parameters,
    sfc_machine_side_pulse_generator_datamodel::parameters>;

class sfc_simulator_builder : public yeet::closed_loop_configuration<sfc_simulator_builder_parameters> {
public:
    using parameter_types = sfc_simulator_builder_parameters;

    sfc_simulator_builder() : plant{create_per_unit_system()}
    {
        // Connect pulse generators to plant model
        connect_pulse_generators_to_plant();
    }

    void initialize()
    {
        // Initialize components in dependency order
        grid_side_pulse_generator.initialize();
        machine_side_pulse_generator.initialize();
        plant.initialize();
    }

    void register_applications(yeet::simulator& sim) override
    {
        // Register all runners with their respective sampling times
        sim.register_lambda_runner(
            grid_side_pulse_generator.get_sampling_time(), [&]() { grid_side_pulse_generator.run(); });
        sim.register_lambda_runner(
            machine_side_pulse_generator.get_sampling_time(), [&]() { machine_side_pulse_generator.run(); });
        sim.register_lambda_runner(plant.get_sampling_time(), [&]() { plant.run(); });
    }

    void register_datamodel(yeet::model_registry& registry) override
    {
        // Register all datamodels
        grid_side_pulse_generator.register_datamodel(registry);
        machine_side_pulse_generator.register_datamodel(registry);
        plant.register_datamodel(registry);
    }

    parameter_types create_parameter_collection() override
    {
        return parameter_types(grid_side_pulse_generator.get_configuration_datamodel().get_parameters(),
            machine_side_pulse_generator.get_configuration_datamodel().get_parameters());
    }

    // Public members for direct access
    sfc_grid_side_pulse_generator_runner grid_side_pulse_generator;
    sfc_machine_side_pulse_generator_runner machine_side_pulse_generator;
    sfc_plant_model_runner plant;

private:
    static std::shared_ptr<sfc::sfc_per_unit_system> create_per_unit_system()
    {
        return std::make_shared<sfc::sfc_per_unit_system>(voltage_t(2090.0),  // Grid voltage RMS
            frequency_t(50.0),                                                // Grid frequency
            current_t(1044.0),                                                // DC link current base
            power_t(2950e3),                                                  // Machine power
            voltage_t(1980.0),                                                // Machine voltage RMS
            frequency_t(50.0),                                                // Machine frequency
            2                                                                 // Number of poles
        );
    }

    void connect_pulse_generators_to_plant()
    {
        // Get datamodel references
        auto& grid_pulse_gen_model = grid_side_pulse_generator.get_signal_flow_datamodel();
        auto& machine_pulse_gen_model = machine_side_pulse_generator.get_signal_flow_datamodel();
        auto& plant_model = plant.get_signal_flow_datamodel();

        // Create and connect the pulse generators to plant model
        ideal_pulse_generators_to_plant_model_connector connector(
            plant_model, grid_pulse_gen_model, machine_pulse_gen_model);
        connector.connect();

        // Connect plant to grid pulse generators
        plant_model_to_grid_pulse_generators_connector grid_connector(
            grid_side_pulse_generator.get_signal_flow_datamodel(), plant.get_signal_flow_datamodel());
        grid_connector.connect();

        // Connect plant to machine pulse generators
        plant_model_to_machine_pulse_generators_connector machine_connector(
            machine_side_pulse_generator.get_signal_flow_datamodel(), plant.get_signal_flow_datamodel());
        machine_connector.connect();
    }
};

///

/**
 * @brief Complete closed-loop SFC control system configuration with aggregated connectors
 */
class closed_loop_sfc_with_plant_model : public yeet::closed_loop_configuration<closed_loop_sfc_parameter_types> {
public:
    using parameter_types = closed_loop_sfc_parameter_types;

    closed_loop_sfc_with_plant_model()
    {
        // Create shared per-unit system
        _per_unit_system = std::make_shared<sfc::sfc_per_unit_system>(voltage_t(2090.0),  // Grid voltage RMS
            frequency_t(50.0),                                                            // Grid frequency
            current_t(1044.0),                                                            // DC link current base
            power_t(2950e3),                                                              // Machine power
            voltage_t(1980.0),                                                            // Machine voltage RMS
            frequency_t(50.0),                                                            // Machine frequency
            2                                                                             // Number of poles
        );

        // Initialize all runners with per-unit system where needed
        initialize_runners();

        // Connect all components using aggregated connectors
        connect_all_datamodels();
    }

    void initialize()
    {
        // Initialize all components in dependency order
        plant.initialize();
        grid_measurements.initialize();
        machine_measurements.initialize();
        dc_link_measurements.initialize();
        grid_observer.initialize();
        machine_observer.initialize();
        current_controller.initialize();
        speed_controller.initialize();
        grid_modulator.initialize();
        machine_modulator.initialize();
    }

    void register_applications(yeet::simulator& sim) override
    {
        // Register all runners with their respective sampling times
        sim.register_lambda_runner(plant.get_sampling_time(), [&]() { plant.run(); });
        sim.register_lambda_runner(grid_measurements.get_sampling_time(), [&]() { grid_measurements.run(); });
        sim.register_lambda_runner(machine_measurements.get_sampling_time(), [&]() { machine_measurements.run(); });
        sim.register_lambda_runner(dc_link_measurements.get_sampling_time(), [&]() { dc_link_measurements.run(); });
        sim.register_lambda_runner(grid_observer.get_sampling_time(), [&]() { grid_observer.run(); });
        sim.register_lambda_runner(machine_observer.get_sampling_time(), [&]() { machine_observer.run(); });
        sim.register_lambda_runner(current_controller.get_sampling_time(), [&]() { current_controller.run(); });
        sim.register_lambda_runner(speed_controller.get_sampling_time(), [&]() { speed_controller.run(); });
        sim.register_lambda_runner(grid_modulator.get_sampling_time(), [&]() { grid_modulator.run(); });
        sim.register_lambda_runner(machine_modulator.get_sampling_time(), [&]() { machine_modulator.run(); });
    }

    void register_datamodel(yeet::model_registry& registry) override
    {
        // Register all datamodels
        plant.register_datamodel(registry);
        grid_measurements.register_datamodel(registry);
        machine_measurements.register_datamodel(registry);
        dc_link_measurements.register_datamodel(registry);
        grid_observer.register_datamodel(registry);
        machine_observer.register_datamodel(registry);
        current_controller.register_datamodel(registry);
        speed_controller.register_datamodel(registry);
        grid_modulator.register_datamodel(registry);
        machine_modulator.register_datamodel(registry);
    }

    parameter_types create_parameter_collection() override
    {
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

    // Public members for direct access if needed
    // Per-unit system must be declared first since other members depend on it
    std::shared_ptr<sfc::sfc_per_unit_system> _per_unit_system;

    sfc_speed_controller_runner speed_controller;
    sfc_current_controller_runner current_controller;
    sfc_dc_link_measurements_runner dc_link_measurements{_per_unit_system};
    sfc_grid_side_observer_runner grid_observer;
    sfc_grid_side_modulator_runner grid_modulator;
    sfc_grid_side_measurements_runner grid_measurements{_per_unit_system};
    sfc_machine_side_observer_runner machine_observer;
    sfc_machine_side_modulator_runner machine_modulator;
    sfc_machine_side_measurements_runner machine_measurements{_per_unit_system};
    sfc_plant_model_runner plant{_per_unit_system};

private:
    void initialize_runners()
    {
        // Most runners are default-constructed
        // Measurement runners and plant are constructed with per-unit system in member initializer list
    }

    void connect_all_datamodels()
    {
        // Get datamodel references
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

        // Create and connect all aggregated connectors
        to_speed_controller_connector to_speed_ctrl(speed_ctrl_model, machine_obs_model);
        to_speed_ctrl.connect();

        to_current_controller_connector to_current_ctrl(
            current_ctrl_model, speed_ctrl_model, dc_link_model, grid_obs_model, machine_obs_model);
        to_current_ctrl.connect();

        to_grid_observer_connector to_grid_obs(grid_obs_model, grid_meas_model);
        to_grid_obs.connect();

        to_grid_modulator_connector to_grid_mod(grid_mod_model, current_ctrl_model, grid_obs_model);
        to_grid_mod.connect();

        to_machine_observer_connector to_machine_obs(machine_obs_model, machine_meas_model);
        to_machine_obs.connect();

        to_machine_modulator_connector to_machine_mod(machine_mod_model, dc_link_model, machine_obs_model);
        to_machine_mod.connect();

        to_grid_measurements_connector to_grid_meas(grid_meas_model, plant_model);
        to_grid_meas.connect();

        to_machine_measurements_connector to_machine_meas(machine_meas_model, plant_model);
        to_machine_meas.connect();

        to_dc_link_measurements_connector to_dc_meas(dc_link_model, plant_model);
        to_dc_meas.connect();

        to_plant_connector to_plant_model(plant_model, grid_mod_model, machine_mod_model);
        to_plant_model.connect();
    }
};

}  // namespace yeet::application::sfc::integration
