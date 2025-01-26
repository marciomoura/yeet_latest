#ifndef YEET_TESTING_SIMULATOR_RUNNER_APPLICATION_HPP
#define YEET_TESTING_SIMULATOR_RUNNER_APPLICATION_HPP

#include <memory>

#include "yeet/application_interface.hpp"
#include "yeet/testing/simulator_runner_application_interface.hpp"

class simulator_runner_application : public simulator_runner_application_interface {
public:
    simulator_runner_application(std::shared_ptr<application_interface> application_runner)
        : _application_runner(application_runner) {}

    double get_sampling_time() override { return _application_runner->get_sampling_time(); }

    void run() override { _application_runner->run(); }

    void initialize() override { _application_runner->initialize(); }

    void stop() override { _application_runner->stop(); }

private:
    std::shared_ptr<application_interface> _application_runner{};
};

#endif  // YEET_TESTING_SIMULATOR_RUNNER_APPLICATION_HPP