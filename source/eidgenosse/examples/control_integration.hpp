
#ifndef EIDGENOSSE_EXAMPLES_CONTROL_INTEGRATION_HPP
#define EIDGENOSSE_EXAMPLES_CONTROL_INTEGRATION_HPP

#include <cmath>

#include "eidgenosse/application_interface.hpp"
#include "eidgenosse/application_registry.hpp"
#include "eidgenosse/parameter.hpp"
#include "eidgenosse/signal.hpp"

//
#include "eidgenosse/model_registry.hpp"

class integrator {
public:
    integrator(double sampling_time) : _sampling_time(sampling_time) {}

    double run(double input) { return _integral += input * _sampling_time; }

    double get_sampling_time() const { return _sampling_time; }

    double get_output() const { return _integral; }

private:
    double _integral{};
    double _sampling_time{};
};

class low_pass_filter {
public:
    low_pass_filter(double sampling_time, double time_constant) : _sampling_time(sampling_time) {
        set_time_constant(time_constant);
    }

    void set_time_constant(double time_constant) {
        _time_constant = time_constant;
        _alpha = 1.0f - std::exp(-_sampling_time / time_constant);
    }

    double run(double input) { return _output = _output + _alpha * (input - _output); }

    double get_output() const { return _output; }

private:
    double _sampling_time{};
    double _output{};
    double _time_constant{};
    double _alpha{};
};

class pi_controller {
public:
    pi_controller(double sampling_time, double kp, double ki) : _kp(kp), _ki(ki), _integrator(sampling_time) {}

    double run(double error) { return _output = _ki * _integrator.run(error) + _kp * error; }

    double get_output() const { return _output; }

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

    // get the registry entries?
    // connect signals...
    // maybe define `register_data_model` as static function?
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

private:
};

class control_application_runner : public application_interface {
public:
    using datamodel = control_application_datamodel;

    static constexpr double sampling_time = 100e-6;

    datamodel get_datamodel() { return datamodel{&_inputs, &_outputs, &_parameters}; }

    void initialize() override {
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

    pi_controller _controller{sampling_time, 10.0f, 10.0f};
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

    // get the registry entries?
    // connect signals...

    inputs* model_inputs{};
    outputs* model_outputs{};
    parameters* model_parameters{};

    void register_data_model(model_registry& registry) {
        model_registry_entry entry{signal_registry{}, parameter_registry{}};

        entry.parameter_registry.insert("ti", model_parameters->ti);
        entry.signal_registry.insert("outputs", model_outputs->plant_output);

        registry.insert("plant", std::move(entry));
    }

private:
};

class plant_model_application_runner : public application_interface {
public:
    using datamodel = plant_model_application_datamodel;

    static constexpr double sampling_time = 100e-6;

    datamodel get_datamodel() { return datamodel{&_inputs, &_outputs, &_parameters}; }

    void initialize() override {
        _parameters.ti.register_callback([this]() { _plant_model.set_time_constant(_parameters.ti); });
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

    low_pass_filter _plant_model{sampling_time, 1.0f};
};

class closed_loop_example {
public:
    closed_loop_example() {
        connect_outputs_to_inputs();
        register_control_tasks();
    }

    std::size_t get_number_task_levels() const { return control_task_level_registry.size(); }

    control_task_level get_control_task_level(std::size_t index) {
        return control_task_level_registry.get_control_task_level(index);
    }

    control_application_datamodel get_control_application_datamodel() { return control_application->get_datamodel(); }

    plant_model_application_datamodel get_plant_model_application_datamodel() {
        return plant_model_application->get_datamodel();
    }

private:
    void connect_outputs_to_inputs() {
        auto control_datamodel = control_application->get_datamodel();
        auto plant_datamodel = plant_model_application->get_datamodel();

        signals_connector::connect_output_to_input(plant_datamodel.model_outputs->plant_output,
                                                   control_datamodel.model_inputs->measurement);

        signals_connector::connect_output_to_input(control_datamodel.model_outputs->output,
                                                   plant_datamodel.model_inputs->input);
    }

    void register_control_tasks() {
        control_task_level_registry.insert(control_application,
                                           control_execution_policy{control_application->sampling_time});

        control_task_level_registry.insert(plant_model_application,
                                           control_execution_policy{plant_model_application->sampling_time});
    }

    std::shared_ptr<control_application_runner> control_application = std::make_shared<control_application_runner>();

    std::shared_ptr<plant_model_application_runner> plant_model_application =
        std::make_shared<plant_model_application_runner>();

    control_task_level_registry control_task_level_registry;
};

#endif  // EIDGENOSSE_EXAMPLES_CONTROL_INTEGRATION_HPP