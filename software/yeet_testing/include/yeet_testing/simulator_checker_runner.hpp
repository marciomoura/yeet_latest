#pragma once

#include <cassert>
#include <memory>
#include <vector>

#include "checker_failures_post_processing_interface.hpp"
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
            // Skip the checker if it is not active
            if (checker->is_active(current_time)) {
                // Run the checker and store the failure metadata
                if (!checker->check() && _max_failures > 0) {
                    // Decrement the maximum number of failures
                    _max_failures--;

                    // Store the checker failure metadata
                    checker_fail_info info{};
                    info.name = checker->get_name();
                    info.message = checker->get_failure_message();
                    info.time = current_time;
                    _checker_failures.push_back(info);
                }
            }
        }
        return true;
    }

    std::string get_last_failure_message() const {
        if (_checker_failures.empty()) {
            return {};
        }

        // Implement using the checker_fail_info struct
        return _checker_failures.back().message + " at " + std::to_string(_checker_failures.back().time) + " seconds.";
    }

    bool has_reached_max_failures() const { return _max_failures == 0; }

    bool has_failure() const { return !_checker_failures.empty(); }

    void dump(checker_failures_post_processing_interface& post_processor) { post_processor.dump(_checker_failures); }

private:
    std::size_t _max_failures{20};
    std::vector<std::shared_ptr<simulator_checker>> _checkers;
    std::vector<checker_fail_info> _checker_failures{};
};

}  // namespace yeet::testing