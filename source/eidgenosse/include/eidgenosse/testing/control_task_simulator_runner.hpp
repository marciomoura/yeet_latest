#ifndef EIDGENOSSE_TESTING_CONTROL_TASK_SIMULATOR_RUNNER_HPP
#define EIDGENOSSE_TESTING_CONTROL_TASK_SIMULATOR_RUNNER_HPP

#include "eidgenosse/application_registry.hpp"
#include "eidgenosse/testing/simulator_runner_application_interface.hpp"

class control_task_simulator_runner : public simulator_runner_application_interface {
public:
    control_task_simulator_runner(control_task_level control_task) : _control_task(control_task) {}

    double get_sampling_rate() override { return _control_task.get_control_execution_policy().get_sampling_rate(); }

    void run() override { _control_task.run(); }

    void initialize() override { _control_task.initialize(); }

    void stop() override { _control_task.stop(); }

private:
    control_task_level _control_task;
};

#endif  // EIDGENOSSE_TESTING_CONTROL_TASK_SIMULATOR_RUNNER_HPP