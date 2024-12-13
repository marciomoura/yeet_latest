#ifndef EIDGENOSSE_APPLICATION_REGISTRY_HPP
#define EIDGENOSSE_APPLICATION_REGISTRY_HPP

#include <algorithm>
#include <memory>
#include <vector>

#include "eidgenosse/application_interface.hpp"
#include "eidgenosse/control_execution_policy.hpp"
#include "eidgenosse/control_task_level.hpp"

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

        sort_by_ascending_control_cycle();
    }

    void clear() { _control_task_list.clear(); }

    bool empty() const { return _control_task_list.empty(); }

    std::size_t size() const { return _control_task_list.size(); }

    control_task_level get_control_task_level(std::size_t index) const { return _control_task_list[index]; }

private:
    void sort_by_ascending_control_cycle() {
        std::sort(_control_task_list.begin(), _control_task_list.end(),
                  [](const control_task_level& a, const control_task_level& b) {
                      return a.get_control_execution_policy().get_execution_ticks() <
                             b.get_control_execution_policy().get_execution_ticks();
                  });
    }

    std::vector<control_task_level> _control_task_list{};
};

// control-task executor for testing platform
// control-task executor for embedded platforms
// the same thing i've employed in the testing... bring it up to this level -> simulator may use it.

// write an example that instantiates 2 applications, defines the connection in between things, register it in the
// "scheduler" and then have it tested

// maybe quickly define how the product should instantiate the set of applications to be tested?
// define a collection of applications that can be combined into a certain "application"
// a product may be a combination of multiple applications:
// [state-machine, monitoring, io logics] <-> [actual control algorithm]
// these things may be interchanged
// I think the interconnection of these algorithms can allow certain interesting feature-enabling stuff for the product
// and then we define how to re-utilize these guys beucase they only need to privde the missing interfaces

#endif  // EIDGENOSSE_APPLICATION_REGISTRY_HPP