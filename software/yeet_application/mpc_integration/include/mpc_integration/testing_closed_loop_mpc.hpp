#include "mpc_integration/mpc_integration_runner.hpp"
#include "yeet/application_datamodel.hpp"
#include "yeet/application_interface.hpp"
#include "yeet/data_model_connector.hpp"
#include "yeet/model_registry.hpp"
#include "yeet/parameter.hpp"
#include "yeet/signal.hpp"
#include "yeet_testing/parameter_collection.hpp"
#include "yeet_testing/simulator.hpp"

// Code generated plant models
#include "mpc_integration/indirect_excitation_2mva_plant_model_integration_runner.hpp"
#include "mpc_integration/synchronous_machine_infinite_bus_integration_runner.hpp"

namespace yeet::application::mpc {

// Define data model types for clarity
using control_datamodel =
    yeet::signal_flow_datamodel<mpc_integration_datamodel::inputs, mpc_integration_datamodel::outputs>;

using plant_datamodel = yeet::signal_flow_datamodel<synchronous_machine_infinite_bus_integration_datamodel::inputs,
                                                    synchronous_machine_infinite_bus_integration_datamodel::outputs>;

// Connector for control to plant and plant to control
class control_plant_connector : public yeet::data_model_connector<plant_datamodel, control_datamodel> {
public:
    control_plant_connector(plant_datamodel& plant, control_datamodel& control)
        : data_model_connector(plant.get_inputs(), control.get_outputs()) {}

    void connect() override { yeet::signals_connector::connect_output_to_input(get_outputs().efd, get_inputs().efd); }
};

class plant_control_connector : public yeet::data_model_connector<control_datamodel, plant_datamodel> {
public:
    plant_control_connector(control_datamodel& control, plant_datamodel& plant)
        : data_model_connector(control.get_inputs(), plant.get_outputs()) {}

    void connect() override {
        yeet::signals_connector::connect_output_to_input(get_outputs().wm, get_inputs().wm);
        yeet::signals_connector::connect_output_to_input(get_outputs().ifd, get_inputs().ifd);
        yeet::signals_connector::connect_output_to_input(get_outputs().it, get_inputs().it);
        yeet::signals_connector::connect_output_to_input(get_outputs().id, get_inputs().id);
        yeet::signals_connector::connect_output_to_input(get_outputs().iq, get_inputs().iq);
        yeet::signals_connector::connect_output_to_input(get_outputs().vt, get_inputs().vt);
        yeet::signals_connector::connect_output_to_input(get_outputs().vd, get_inputs().vd);
        yeet::signals_connector::connect_output_to_input(get_outputs().vq, get_inputs().vq);
        yeet::signals_connector::connect_output_to_input(get_outputs().vg, get_inputs().vg);
        yeet::signals_connector::connect_output_to_input(get_outputs().load_angle_deg, get_inputs().load_angle_deg);
    }
};

// Define parameter types for this configuration
using closed_loop_mpc_with_plant_model_parameter_types =
    yeet::testing::parameter_collection<mpc_integration_datamodel::parameters,
                                        synchronous_machine_infinite_bus_integration_datamodel::parameters>;

// Example of how to update the closed loop example
class closed_loop_mpc_with_plant_model
    : public yeet::testing::closed_loop_configuration<closed_loop_mpc_with_plant_model_parameter_types> {
public:
    using parameter_types = closed_loop_mpc_with_plant_model_parameter_types;

    closed_loop_mpc_with_plant_model() { connect_controller_and_plant_datamodels(); }

    void initialize() {
        mpc.initialize();
        plant.initialize();
    }

    void register_applications(yeet::testing::simulator& sim) override {
        sim.register_lambda_runner(mpc.get_sampling_time(), [&]() { mpc.run(); });
        sim.register_lambda_runner(plant.get_sampling_time(), [&]() { plant.run(); });
    }

    void register_datamodel(yeet::model_registry& registry) override {
        mpc.register_datamodel(registry);
        plant.register_datamodel(registry);
    }

    parameter_types create_parameter_collection() override {
        return parameter_types(mpc.get_configuration_datamodel().get_parameters(),
                               plant.get_configuration_datamodel().get_parameters());
    }

    mpc_integration_runner mpc;

    synchronous_machine_infinite_bus_integration_runner plant;

private:
    void connect_controller_and_plant_datamodels() {
        auto& control_model = mpc.get_signal_flow_datamodel();
        auto& plant_model = plant.get_signal_flow_datamodel();

        control_plant_connector control_to_plant(plant_model, control_model);
        plant_control_connector plant_to_control(control_model, plant_model);

        control_to_plant.connect();
        plant_to_control.connect();
    }
};

class closed_loop_mpc_with_indirect_excitation_plant_model
    : public yeet::testing::closed_loop_configuration<closed_loop_mpc_with_plant_model_parameter_types> {
public:
    using parameter_types = closed_loop_mpc_with_plant_model_parameter_types;

    closed_loop_mpc_with_indirect_excitation_plant_model() { connect_controller_and_plant_datamodels(); }

    void initialize() {
        mpc.initialize();
        plant.initialize();
    }

    void register_applications(yeet::testing::simulator& sim) override {
        sim.register_lambda_runner(mpc.get_sampling_time(), [&]() { mpc.run(); });
        sim.register_lambda_runner(plant.get_sampling_time(), [&]() { plant.run(); });
    }

    void register_datamodel(yeet::model_registry& registry) override {
        mpc.register_datamodel(registry);
        plant.register_datamodel(registry);
    }

    parameter_types create_parameter_collection() override {
        return parameter_types(mpc.get_configuration_datamodel().get_parameters(),
                               plant.get_configuration_datamodel().get_parameters());
    }

    mpc_integration_runner mpc;

    indirect_excitation_2mva_plant_model_integration_runner plant;

private:
    void connect_controller_and_plant_datamodels() {
        auto& control_model = mpc.get_signal_flow_datamodel();
        auto& plant_model = plant.get_signal_flow_datamodel();

        control_plant_connector control_to_plant(plant_model, control_model);
        plant_control_connector plant_to_control(control_model, plant_model);

        control_to_plant.connect();
        plant_to_control.connect();
    }
};

}  // namespace yeet::application::mpc