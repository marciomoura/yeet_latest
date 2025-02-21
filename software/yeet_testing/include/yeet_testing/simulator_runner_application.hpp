#ifndef YEET_TESTING_SIMULATOR_RUNNER_APPLICATION_HPP
#define YEET_TESTING_SIMULATOR_RUNNER_APPLICATION_HPP

#include <functional>
#include <memory>

#include "yeet/application_interface.hpp"
#include "yeet_testing/simulator_runner_application_interface.hpp"

namespace yeet {

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

class simulator_lambda_runner_application : public simulator_runner_application_interface {
public:
    simulator_lambda_runner_application(double sampling_time, std::function<void()> application_runner)
        : _sampling_time(sampling_time), _application_runner(application_runner) {}

    double get_sampling_time() override { return _sampling_time; }

    void run() override { _application_runner(); }

    void initialize() override {}

    void stop() override {}

private:
    double _sampling_time{};
    std::function<void()> _application_runner{};
};

}  // namespace yeet

#endif  // YEET_TESTING_SIMULATOR_RUNNER_APPLICATION_HPP