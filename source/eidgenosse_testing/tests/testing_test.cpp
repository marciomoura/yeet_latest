#include "eidgenosse_testing/testing.hpp"

#include <gtest/gtest.h>

#include <cmath>

#include "eidgenosse/parameter.hpp"
#include "eidgenosse/signal.hpp"

class gtest_testing_info : public testing_info_interface {
public:
    gtest_testing_info(const ::testing::UnitTest* unit_test) : _unit_test(unit_test) {}

    std::string get_test_suite_name() const override { return _unit_test->current_test_info()->test_suite_name(); }
    std::string get_test_case_name() const override { return _unit_test->current_test_info()->name(); }

private:
    const ::testing::UnitTest* _unit_test{};
};

// Create test fixture
class simulator_test : public ::testing::Test {
public:
    void SetUp() override { _simulator = std::make_unique<simulator>(); }

    void TearDown() override {
        gtest_testing_info gtest_info(::testing::UnitTest::GetInstance());
        _simulator->dump_json(&gtest_info);

        _simulator.reset();
    }

    std::unique_ptr<simulator> _simulator;
};

class application_one_mock : public cyclic_application_interface {
public:
    double get_sampling_rate() override { return 0.01f; }

    void run() override { value += 1.0f; }

    void register_data_sampler_entries(data_sampler& data_sampler) override {
        data_sampler.register_entry(data_sampler_entry("value_one", [this]() { return value; }));
    }

    float value = 0.0f;
};

class application_two_mock : public cyclic_application_interface {
public:
    double get_sampling_rate() override { return 0.02f; }

    void run() override { value += 2.0f; }

    void register_data_sampler_entries(data_sampler& data_sampler) override {
        data_sampler.register_entry(data_sampler_entry("value_two", [this]() { return value; }));
    }

    float value = 0.0f;
};

class application_sinusoidal_mock : public cyclic_application_interface {
public:
    application_sinusoidal_mock() {
        _sin = std::sin(_angle);
        _cos = std::cos(_angle);
    }

    double get_sampling_rate() override { return 100e-6; }

    void run() override {
        _sin = std::sin(_angle);
        _cos = std::cos(_angle);
        _angle += _angle_increment;
    }

    void register_data_sampler_entries(data_sampler& data_sampler) override {
        data_sampler.register_entry(data_sampler_entry("sin", [this]() { return _sin; }));
        data_sampler.register_entry(data_sampler_entry("cos", [this]() { return _cos; }));
    }

    static constexpr float pi = 3.14159265358979323846;
    static constexpr float ts = 25e-6;
    float _angle_increment = 10.0f * 2.0f * pi * ts;
    float _angle = 0.0f;
    float _sin = 0.0f;
    float _cos = 0.0f;
};

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

private:
    double _kp{1.0f};
    double _ki{1.0f};
    double _output{};
    integrator _integrator;
};

class application_closed_control_loop : public cyclic_application_interface {
public:
    static constexpr double sample_time = 100e-6;

    double get_sampling_rate() override { return sample_time; }

    void set_reference(double reference) { _reference = reference; }

    void run() override {
        double error = _reference - _plant.get_output();
        double control_output = _controller.run(error);
        double sensor_output = _sensor.run(control_output);
        _plant.run(sensor_output);
    }

    void register_data_sampler_entries(data_sampler& data_sampler) override {
        data_sampler.register_entry(data_sampler_entry("reference", [this]() { return _reference; }));
        data_sampler.register_entry(data_sampler_entry("plant output", [this]() { return _plant.get_output(); }));
        data_sampler.register_entry(data_sampler_entry("error", [this]() { return _reference - _plant.get_output(); }));
        data_sampler.register_entry(data_sampler_entry("sensor", [this]() { return _sensor.get_output(); }));
        data_sampler.register_entry(
            data_sampler_entry("controller output", [this]() { return _controller.get_output(); }));
    }

private:
    double _reference{};
    pi_controller _controller{sample_time, 10.0f, 100.0f};
    integrator _plant{sample_time};
    low_pass_filter _sensor{sample_time, 0.01f};
};

class controller_application : public cyclic_application_interface {
public:
    static constexpr double sample_time = 100e-6;

    double get_sampling_rate() override { return sample_time; }

    void set_controller_gains(double kp, double ki) { _controller = pi_controller(sample_time, kp, ki); }

    void set_reference(double reference) { _reference = reference; }

    void run() override { _output = _controller.run(_reference - _measurement); }

    void register_data_sampler_entries(data_sampler& data_sampler) override {
        data_sampler.register_entry(data_sampler_entry("reference", [this]() { return _reference; }));
        data_sampler.register_entry(data_sampler_entry("controller output", [this]() { return _output; }));
    }

    signal_output<double> _output;

    signal_input<double> _measurement;

    parameter<double> _reference;

private:
    pi_controller _controller{sample_time, 10.0f, 100.0f};
};

class measurement_model_application : public cyclic_application_interface {
public:
    static constexpr double sample_time = 100e-6;

    double get_sampling_rate() override { return sample_time; }

    void run() override { _output = _filter.run(_input); }

    void register_data_sampler_entries(data_sampler& data_sampler) override {
        data_sampler.register_entry(data_sampler_entry("measurement output", [this]() { return _output; }));
    }

    signal_output<double> _output;

    signal_input<double> _input;

private:
    low_pass_filter _filter{sample_time, 0.01f};
};

class plant_model_application : public cyclic_application_interface {
public:
    static constexpr double sample_time = 100e-6;

    double get_sampling_rate() override { return sample_time; }

    void run() override { _output = _integrator.run(_input); }

    void register_data_sampler_entries(data_sampler& data_sampler) override {
        data_sampler.register_entry(data_sampler_entry("plant model output", [this]() { return _output; }));
    }

    signal_output<double> _output;

    signal_input<double> _input;

private:
    integrator _integrator{sample_time};
};

TEST_F(simulator_test, register_cyclic_application) {
    _simulator->register_cyclic_application(std::make_shared<application_one_mock>());
    _simulator->register_cyclic_application(std::make_shared<application_two_mock>());
    _simulator->simulate_for(0.25f);
    _simulator->simulate_for(0.25f);
}

TEST_F(simulator_test, sin_cos_simulation) {
    _simulator->register_cyclic_application(std::make_shared<application_sinusoidal_mock>());
    _simulator->simulate_for(0.2f);
}

TEST_F(simulator_test, closed_loop_simulation) {
    std::shared_ptr<application_closed_control_loop> application = std::make_shared<application_closed_control_loop>();

    _simulator->register_cyclic_application(application);

    application->set_reference(0.0f);
    _simulator->simulate_for(0.1f);

    application->set_reference(1.0f);
    _simulator->simulate_for(4.0f);

    application->set_reference(0.0f);
    _simulator->simulate_for(4.0f);
}

TEST_F(simulator_test, controller_application) {
    std::shared_ptr<controller_application> controller = std::make_shared<controller_application>();
    std::shared_ptr<measurement_model_application> measurement_model =
        std::make_shared<measurement_model_application>();
    std::shared_ptr<plant_model_application> plant_model = std::make_shared<plant_model_application>();

    _simulator->register_cyclic_application(controller);
    _simulator->register_cyclic_application(measurement_model);
    _simulator->register_cyclic_application(plant_model);

    signals_connector::connect_output_to_input(measurement_model->_output, plant_model->_input);
    signals_connector::connect_output_to_input(controller->_output, measurement_model->_input);
    signals_connector::connect_output_to_input(plant_model->_output, controller->_measurement);

    controller->set_controller_gains(10.0f, 20.0f);

    controller->set_reference(0.0f);

    _simulator->simulate_for(0.1f);

    controller->set_reference(1.0f);

    _simulator->simulate_for(1.0f);

    controller->set_reference(0.0f);

    _simulator->simulate_for(1.0f);
}