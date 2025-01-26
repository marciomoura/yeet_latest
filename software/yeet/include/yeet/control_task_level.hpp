#ifndef YEET_CONTROL_TASK_LEVEL_HPP
#define YEET_CONTROL_TASK_LEVEL_HPP

#include <algorithm>
#include <memory>
#include <vector>

#include "yeet/application_interface.hpp"
#include "yeet/control_execution_policy.hpp"

class control_task_level {
public:
    control_task_level(control_execution_policy policy) : _policy(policy) {}
    ~control_task_level() = default;

    control_execution_policy get_control_execution_policy() const { return _policy; }

    void run() {
        for (auto& app : _application) {
            app->run();
        }
    }

    void stop() {
        for (auto& app : _application) {
            app->stop();
        }
    }

    void initialize() {
        for (auto& app : _application) {
            app->initialize();
        }
    }

    void insert(std::shared_ptr<application_interface> application) {
        // check if application is inserted
        if (std::find(_application.begin(), _application.end(), application) != _application.end()) {
            return;
        }

        _application.push_back(application);
    }

    void clear() { _application.clear(); }

    bool empty() const { return _application.empty(); }

    std::size_t size() const { return _application.size(); }

    std::shared_ptr<const application_interface> get_application(std::size_t index) const {
        return _application[index];
    }

private:
    control_execution_policy _policy;
    std::vector<std::shared_ptr<application_interface>> _application;
};

#endif  // YEET_CONTROL_TASK_LEVEL_HPP