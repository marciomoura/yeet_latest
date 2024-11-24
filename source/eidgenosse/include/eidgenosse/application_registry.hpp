#ifndef EIDGENOSSE_APPLICATION_REGISTRY_HPP
#define EIDGENOSSE_APPLICATION_REGISTRY_HPP

#include <algorithm>
#include <list>
#include <memory>
#include <vector>

#include "eidgenosse/application_interface.hpp"

class control_execution_policy {
public:
    static constexpr double tick_base_rate = 1e-6;

    control_execution_policy() = default;
    control_execution_policy(double execution_rate) : _execution_rate(execution_rate) {}

    double get_tick_base_rate() const { return tick_base_rate; }

    std::size_t get_execution_ticks() const { return static_cast<std::size_t>(_execution_rate / tick_base_rate); }

    std::size_t get_execution_rate() const { return _execution_rate; }

private:
    double _execution_rate{0.0};
};

class application_registry {
public:
    application_registry();
    ~application_registry();

    bool insert(std::shared_ptr<application_interface> application, control_execution_policy control_execution_policy) {
        // check if application already not inserted by checking raw address
        if (is_registered(application)) {
            return false;
        }

        application_registry_entry entry;
        entry.application = application;
        entry.control_execution_policy = control_execution_policy;

        _registry_application.push_back(entry);
        return true;
    }

    bool is_registered(std::shared_ptr<application_interface> application) const {
        for (auto& app : _registry_application) {
            if (app.application.get() == application.get()) {
                return true;
            }
        }

        return false;
    }

private:
    struct application_registry_entry {
        std::shared_ptr<application_interface> application{};
        control_execution_policy control_execution_policy{};
    };

    std::list<application_registry_entry> _registry_application;
};

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

class control_task_level_registry {
public:
    control_task_level_registry() = default;
    ~control_task_level_registry() = default;

    void insert(std::shared_ptr<application_interface> application, control_execution_policy policy) {
        // check if the policy exists, if it does, then insert the application into the existing policy using
        // std::find_if
        auto search = std::find_if(
            _control_task_list.begin(), _control_task_list.end(), [&policy](const control_task_level& level) {
                return level.get_control_execution_policy().get_execution_ticks() == policy.get_execution_ticks();
            });

        if (search != _control_task_list.end()) {
            search->insert(application);
        } else {
            control_task_level level(policy);
            level.insert(application);
            _control_task_list.push_back(level);
        }
    }

    void clear() { _control_task_list.clear(); }

    bool empty() const { return _control_task_list.empty(); }

    std::size_t size() const { return _control_task_list.size(); }

    control_task_level get_control_task_level(std::size_t index) const { return _control_task_list[index]; }

private:
    std::vector<control_task_level> _control_task_list{};
};

// control-task executor for testing platform
// control-task executor for embedded platforms
// the same thing i've employed in the testing... bring it up to this level -> simulator may use it.

#endif  // EIDGENOSSE_APPLICATION_REGISTRY_HPP