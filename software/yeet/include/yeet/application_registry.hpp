#ifndef YEET_APPLICATION_REGISTRY_HPP
#define YEET_APPLICATION_REGISTRY_HPP

#include <list>
#include <memory>

#include "yeet/application_interface.hpp"
#include "yeet/control_execution_policy.hpp"

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
        std::shared_ptr<application_interface> application;
        control_execution_policy control_execution_policy;
    };

    std::list<application_registry_entry> _registry_application;
};

#endif  // YEET_APPLICATION_REGISTRY_HPP