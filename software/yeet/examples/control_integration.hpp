
#ifndef YEET_EXAMPLES_CONTROL_INTEGRATION_HPP
#define YEET_EXAMPLES_CONTROL_INTEGRATION_HPP

#include <cmath>

#include "yeet/application_interface.hpp"
#include "yeet/parameter.hpp"
#include "yeet/signal.hpp"

//
#include "yeet/application_datamodel.hpp"
#include "yeet/data_model_connector.hpp"
#include "yeet/model_registry.hpp"
#include "yeet/parameter.hpp"
#include "yeet/signal.hpp"
#include "yeet/testing/parameter_collection.hpp"
#include "yeet/testing/simulator.hpp"

class integrator {
public:
    void set_sampling_time(double sampling_time) { _sampling_time = sampling_time; }

    double run(double input) { return _integral += input * _sampling_time; }

    double get_sampling_time() const { return _sampling_time; }

    double get_output() const { return _integral; }

private:
    double _integral{};
    double _sampling_time{};
};

class low_pass_filter {
public:
    void set_time_constant(double time_constant, double sampling_time) {
        _time_constant = time_constant;
        _alpha = 1.0f - std::exp(-sampling_time / time_constant);
    }

    double run(double input) { return _output = _output + _alpha * (input - _output); }

    double get_output() const { return _output; }

private:
    double _output{};
    double _time_constant{};
    double _alpha{};
};

class pi_controller {
public:
    double run(double error) { return _output = _ki * _integrator.run(error) + _kp * error; }

    double get_output() const { return _output; }

    void set_sampling_time(double sampling_time) { _integrator.set_sampling_time(sampling_time); }
    void set_kp(double kp) { _kp = kp; }
    void set_ki(double ki) { _ki = ki; }

private:
    double _kp{1.0f};
    double _ki{1.0f};
    double _output{};
    integrator _integrator;
};

namespace yeet {

// Data model definitions stay mostly the same
struct control_inputs {
    signal_input<double> measurement;
};

struct control_parameters {
    parameter<double> kp;
    parameter<double> ki;
    parameter<double> reference;
};

struct control_outputs {
    signal_output<double> output;
};

// Updated control application runner
class control_application_runner : public base_application<control_inputs, control_outputs, control_parameters> {
public:
    control_application_runner() : base_application(100e-6) {}  // 100us sampling time

    void initialize() override {
        _controller.set_sampling_time(get_sampling_time());

        _parameters.ki.register_callback([this]() { _controller.set_ki(_parameters.ki); });

        _parameters.kp.register_callback([this]() { _controller.set_kp(_parameters.kp); });

        _parameters.ki = 10.0f;
        _parameters.kp = 10.0f;
    }

    void run() override {
        const double error = _parameters.reference - _inputs.measurement;
        _controller.run(error);
        _outputs.output = _controller.get_output();
    }

    void stop() override {}

    void register_datamodel(model_registry& registry) override {
        model_registry_entry entry{signal_registry{}, parameter_registry{}};

        entry.parameter_registry.insert("kp", _parameters.kp);
        entry.parameter_registry.insert("ki", _parameters.ki);
        entry.parameter_registry.insert("reference", _parameters.reference);
        entry.signal_registry.insert("output", _outputs.output);

        registry.insert("control", std::move(entry));
    }

private:
    pi_controller _controller;
};

// Plant model structures
struct plant_inputs {
    signal_input<double> input;
};

struct plant_parameters {
    parameter<double> ti;
};

struct plant_outputs {
    signal_output<double> plant_output;
};

// Updated plant model application runner
class plant_model_application_runner : public base_application<plant_inputs, plant_outputs, plant_parameters> {
public:
    plant_model_application_runner() : base_application(100e-6) {}  // 100us sampling time

    void initialize() override {
        _parameters.ti.register_callback(
            [this]() { _plant_model.set_time_constant(_parameters.ti, this->get_sampling_time()); });

        _parameters.ti = 1.0f;
    }

    void run() override {
        _plant_model.run(_inputs.input);
        _outputs.plant_output = _plant_model.get_output();
    }

    void stop() override {}

    void register_datamodel(model_registry& registry) override {
        model_registry_entry entry{signal_registry{}, parameter_registry{}};

        entry.parameter_registry.insert("ti", _parameters.ti);
        entry.signal_registry.insert("outputs", _outputs.plant_output);

        registry.insert("plant", std::move(entry));
    }

private:
    low_pass_filter _plant_model;
};

class plant_model_double_filter_application_runner
    : public base_application<plant_inputs, plant_outputs, plant_parameters> {
public:
    plant_model_double_filter_application_runner() : base_application(100e-6) {}  // 100us sampling time

    void initialize() override {
        _parameters.ti.register_callback([this]() {
            _plant_model_1.set_time_constant(_parameters.ti, this->get_sampling_time());
            _plant_model_2.set_time_constant(_parameters.ti * 0.5, this->get_sampling_time());
        });

        _parameters.ti = 1.0f;
    }

    void run() override {
        _plant_model_1.run(_inputs.input);
        _plant_model_2.run(_plant_model_1.get_output());
        _outputs.plant_output = _plant_model_2.get_output();
    }

    void stop() override {}

    void register_datamodel(model_registry& registry) override {
        model_registry_entry entry{signal_registry{}, parameter_registry{}};

        entry.parameter_registry.insert("ti", _parameters.ti);
        entry.signal_registry.insert("outputs", _outputs.plant_output);

        registry.insert("plant", std::move(entry));
    }

private:
    low_pass_filter _plant_model_1;
    low_pass_filter _plant_model_2;
};

// Define data model types for clarity
using control_datamodel = application_datamodel<control_inputs, control_outputs, control_parameters>;
using plant_datamodel = application_datamodel<plant_inputs, plant_outputs, plant_parameters>;

// Connector for control to plant and plant to control
class control_plant_connector : public data_model_connector<plant_datamodel, control_datamodel> {
public:
    control_plant_connector(plant_datamodel& plant, control_datamodel& control)
        : data_model_connector(plant.get_inputs(), control.get_outputs()) {}

    void connect() override { signals_connector::connect_output_to_input(get_outputs().output, get_inputs().input); }
};

class plant_control_connector : public data_model_connector<control_datamodel, plant_datamodel> {
public:
    plant_control_connector(control_datamodel& control, plant_datamodel& plant)
        : data_model_connector(control.get_inputs(), plant.get_outputs()) {}

    void connect() override {
        signals_connector::connect_output_to_input(get_outputs().plant_output, get_inputs().measurement);
    }
};

// Define parameter types for this configuration
using parameter_types = parameter_collection<control_parameters, plant_parameters>;

// Example of how to update the closed loop example
class closed_loop_example : public closed_loop_configuration<parameter_types> {
public:
    using parameter_types = parameter_types;

    closed_loop_example(std::shared_ptr<base_application<plant_inputs, plant_outputs, plant_parameters>>
                            plant_model_application = std::make_shared<plant_model_application_runner>())
        : plant_model_application(plant_model_application) {
        // Connection logic stays the same
        auto& control_model = control_application->get_datamodel();
        auto& plant_model = plant_model_application->get_datamodel();

        control_plant_connector control_to_plant(plant_model, control_model);
        plant_control_connector plant_to_control(control_model, plant_model);

        control_to_plant.connect();
        plant_to_control.connect();
    }

    void register_applications(simulator& sim) override {
        sim.register_control_application_runner(control_application);
        sim.register_control_application_runner(plant_model_application);
    }

    void register_datamodel(model_registry& registry) override {
        control_application->register_datamodel(registry);
        plant_model_application->register_datamodel(registry);
    }

    parameter_types create_parameter_collection() override {
        return parameter_types(control_application->get_datamodel().get_parameters(),
                               plant_model_application->get_datamodel().get_parameters());
    }

    std::shared_ptr<control_application_runner> control_application = std::make_shared<control_application_runner>();

    std::shared_ptr<base_application<plant_inputs, plant_outputs, plant_parameters>> plant_model_application{};
};

}  // namespace yeet

#endif  // YEET_EXAMPLES_CONTROL_INTEGRATION_HPP