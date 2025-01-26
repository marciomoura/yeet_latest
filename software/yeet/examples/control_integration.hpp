
#ifndef YEET_EXAMPLES_CONTROL_INTEGRATION_HPP
#define YEET_EXAMPLES_CONTROL_INTEGRATION_HPP

#include <cmath>

#include "yeet/application_interface.hpp"
#include "yeet/parameter.hpp"
#include "yeet/signal.hpp"

//
#include "yeet/model_registry.hpp"
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

class control_application_datamodel {
public:
    struct inputs {
        signal_input<double> measurement;
    };

    struct parameters {
        parameter<double> kp;
        parameter<double> ki;
        parameter<double> reference;
    };

    struct outputs {
        signal_output<double> output;
    };

    control_application_datamodel(inputs* model_inputs, outputs* model_outputs, parameters* model_parameters)
        : model_inputs(model_inputs), model_outputs(model_outputs), model_parameters(model_parameters) {
        assert(model_inputs != nullptr);
        assert(model_outputs != nullptr);
        assert(model_parameters != nullptr);
    }

    inputs* model_inputs{};
    outputs* model_outputs{};
    parameters* model_parameters{};

    void register_data_model(model_registry& registry) {
        model_registry_entry entry{signal_registry{}, parameter_registry{}};

        entry.parameter_registry.insert("kp", model_parameters->kp);
        entry.parameter_registry.insert("ki", model_parameters->ki);
        entry.parameter_registry.insert("reference", model_parameters->reference);
        entry.signal_registry.insert("output", model_outputs->output);

        registry.insert("control", std::move(entry));
    }
};

class control_application_runner : public application_interface {
public:
    using datamodel = control_application_datamodel;

    control_application_runner() {
        static constexpr double sampling_time = 100e-6;
        set_sampling_time(sampling_time);
    }

    datamodel get_datamodel() { return datamodel{&_inputs, &_outputs, &_parameters}; }

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

private:
    datamodel::inputs _inputs;
    datamodel::outputs _outputs;
    datamodel::parameters _parameters;

    pi_controller _controller;
};

class plant_model_application_datamodel {
public:
    struct inputs {
        signal_input<double> input;
    };

    struct parameters {
        parameter<double> ti;
    };

    struct outputs {
        signal_output<double> plant_output;
    };

    plant_model_application_datamodel(inputs* model_inputs, outputs* model_outputs, parameters* model_parameters)
        : model_inputs(model_inputs), model_outputs(model_outputs), model_parameters(model_parameters) {
        assert(model_inputs != nullptr);
        assert(model_outputs != nullptr);
        assert(model_parameters != nullptr);
    }

    inputs* model_inputs{};
    outputs* model_outputs{};
    parameters* model_parameters{};

    void register_data_model(model_registry& registry) {
        model_registry_entry entry{signal_registry{}, parameter_registry{}};

        entry.parameter_registry.insert("ti", model_parameters->ti);
        entry.signal_registry.insert("outputs", model_outputs->plant_output);

        registry.insert("plant", std::move(entry));
    }
};

class plant_model_application_runner : public application_interface {
public:
    using datamodel = plant_model_application_datamodel;

    plant_model_application_runner() {
        static constexpr double sampling_time = 100e-6;
        set_sampling_time(sampling_time);
    }

    datamodel get_datamodel() { return datamodel{&_inputs, &_outputs, &_parameters}; }

    void initialize() override {
        _parameters.ti.register_callback(
            [this]() { _plant_model.set_time_constant(_parameters.ti, get_sampling_time()); });

        _parameters.ti = 1.0f;
    }

    void run() override {
        _plant_model.run(_inputs.input);

        _outputs.plant_output = _plant_model.get_output();
    }

    void stop() override {}

private:
    datamodel::inputs _inputs;
    datamodel::outputs _outputs;
    datamodel::parameters _parameters;

    low_pass_filter _plant_model;
};

class closed_loop_example_connector {
public:
    static void connect(control_application_datamodel control_datamodel,
                        plant_model_application_datamodel plant_datamodel) {
        signals_connector::connect_output_to_input(plant_datamodel.model_outputs->plant_output,
                                                   control_datamodel.model_inputs->measurement);

        signals_connector::connect_output_to_input(control_datamodel.model_outputs->output,
                                                   plant_datamodel.model_inputs->input);
    }
};

class closed_loop_example_simulator;

class closed_loop_example {
public:
    closed_loop_example() {
        closed_loop_example_connector::connect(control_application->get_datamodel(),
                                               plant_model_application->get_datamodel());
    }

private:
    friend class closed_loop_example_simulator;

    std::shared_ptr<control_application_runner> control_application = std::make_shared<control_application_runner>();

    std::shared_ptr<plant_model_application_runner> plant_model_application =
        std::make_shared<plant_model_application_runner>();
};

class closed_loop_example_simulator {
public:
    struct parameters {
        control_application_datamodel::parameters* control{};
        plant_model_application_datamodel::parameters* plant{};
    };

    static parameters get_parameters(closed_loop_example& example) {
        return parameters{example.control_application->get_datamodel().model_parameters,
                          example.plant_model_application->get_datamodel().model_parameters};
    }

    static void configure(simulator& simulator, closed_loop_example& example) {
        register_simulator_applications(simulator, example);

        register_data_sampler(simulator, example);
    }

private:
    static void register_simulator_applications(simulator& simulator, closed_loop_example& example) {
        simulator.register_control_application_runner(example.control_application);
        simulator.register_control_application_runner(example.plant_model_application);
    }

    static void register_data_sampler(simulator& simulator, closed_loop_example& example) {
        model_registry model;

        auto plant_model_application = example.plant_model_application->get_datamodel();
        auto control_model_application = example.control_application->get_datamodel();

        control_model_application.register_data_model(model);
        plant_model_application.register_data_model(model);

        simulator.register_data_sampler_entries(model);
    }
};

#endif  // YEET_EXAMPLES_CONTROL_INTEGRATION_HPP