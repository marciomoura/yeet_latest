#pragma once

#include <cassert>
#include <memory>
#include <vector>

#include "simulator_checker.hpp"

namespace yeet::testing {

class simulator_checker_runner {
public:
    void register_checker(std::unique_ptr<simulator_checker> checker) {
        assert(checker != nullptr);
        _checkers.push_back(std::move(checker));
    }

    template <typename CheckerType>
    void register_checker(CheckerType checker) {
        register_checker(make_unique_checker(std::move(checker)));
    }

    bool run_checks(double current_time) {
        for (const auto& checker : _checkers) {
            if (checker->is_active(current_time)) {
                if (!checker->check() && _max_failures > 0) {
                    _max_failures--;
                    _last_failure_message = checker->get_failure_message();
                    _last_failure_time = current_time;
                    return false;
                }
            }
        }
        return true;
    }

    std::string get_last_failure_message() const {
        return _last_failure_message + " at " + std::to_string(_last_failure_time) + " seconds.";
    }

    bool has_reached_max_failures() const { return _max_failures == 0; }

private:
    std::vector<std::unique_ptr<simulator_checker>> _checkers;
    std::size_t _max_failures{5};
    std::string _last_failure_message;
    double _last_failure_time{0.0};
};

}  // namespace yeet::testing