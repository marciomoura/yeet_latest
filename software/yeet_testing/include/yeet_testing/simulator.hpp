#ifndef YEET_TESTING_SIMULATOR_HPP
#define YEET_TESTING_SIMULATOR_HPP

#include <algorithm>
#include <cassert>
#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

// experimental
#include <gtest/gtest.h>

#include "yeet/application_interface.hpp"
#include "yeet/model_registry.hpp"
#include "yeet_testing/data_sample_json_post_processing.hpp"
#include "yeet_testing/data_sample_terminal_post_processing.hpp"
#include "yeet_testing/data_sampler.hpp"
#include "yeet_testing/data_sampler_entry.hpp"
#include "yeet_testing/simulator_checker.hpp"
#include "yeet_testing/simulator_runner_application.hpp"
#include "yeet_testing/simulator_runner_application_interface.hpp"
#include "yeet_testing/testing_info_interface.hpp"

namespace yeet::testing {

// model_integration_simulator
// -> add the functionality to run the app integration and so on

// simulator
// basics to run the checker, add loggers and so on

class simulator {
public:
    simulator() {
        data_sampler_entry entry("time", [&]() { return _current_simulation_time_ticks * _fastest_sampling_rate; });

        _data_sampler.register_entry(entry);
    }

    void register_lambda_runner(double sampling_time, std::function<void()> lambda_runner) {
        assert(lambda_runner != nullptr && "lambda_runner must not be nullptr");

        register_application(std::make_shared<simulator_lambda_runner_application>(sampling_time, lambda_runner));

        update_simulator_applications_list_based_on_normalized_ticks();
    }

    void register_checker(std::unique_ptr<simulator_checker> checker) {
        assert(checker != nullptr && "checker must not be nullptr");
        _checkers.push_back(std::move(checker));
    }

    template <typename CheckerType>
    void register_checker(CheckerType checker) {
        auto unique_checker = make_unique_checker(checker);

        _checkers.push_back(std::move(unique_checker));
    }

    void register_control_application_runner(std::shared_ptr<application_interface> application) {
        assert(application != nullptr && "application must not be nullptr");

        register_application(std::make_shared<simulator_runner_application>(application));

        update_simulator_applications_list_based_on_normalized_ticks();
    }

    void register_data_sampler_entry(data_sampler_entry entry) { _data_sampler.register_entry(entry); }

    void register_data_sampler_entries(model_registry& model) {
        model.iterate_entries(
            [this](const std::string& group, const std::string& name, const low_level_access& access) {
                low_level_access::sampler_accessor const accessor = access.get_sampler_accessor();
                if (accessor.read) {
                    register_data_sampler_entry(
                        data_sampler_entry{group + " " + name, [accessor]() { return accessor.read(); }});
                }
            });
    }

    void initialize() {
        for (auto& [normalized_ticks, application_list] : _simulator_applications) {
            for (auto& n : application_list) {
                n->initialize();
            }
        }
    }

    void simulate_for(double time) {
        if (_max_number_checker_failures == 0) {
            // A threshold has been reached
            return;
        }

        assert(_fastest_sampling_rate < std::numeric_limits<float>::max() && "No cyclic applications registered");
        assert(_fastest_sampling_rate > 0.0f && "tick_sampling_rate_ must be greater than 0.0f");

        std::size_t const simulation_end_time_ticks = _current_simulation_time_ticks + convert_time_to_ticks(time);

        allocate_data_sampler_buffer(time);

        while (_current_simulation_time_ticks < simulation_end_time_ticks) {
            run_data_sampler();

            run_applications(_current_simulation_time_ticks);

            run_checkers();

            if (_max_number_checker_failures == 0) {
                // A threshold has been reached
                return;
            }

            _current_simulation_time_ticks += 1;
            assert(_current_simulation_time_ticks <= std::numeric_limits<std::size_t>::max() &&
                   "Simulation time overflow");
        }
    }

    double get_current_simulation_time() const { return _current_simulation_time_ticks * _fastest_sampling_rate; }

    void dump_terminal() {
        data_sample_terminal_post_processing terminal;
        _data_sampler.dump(terminal);
    }

    void dump_json(testing_info_interface const& testing_info) {
        data_sample_json_post_processing terminal;
        _data_sampler.dump(testing_info.get_test_suite_name(), testing_info.get_test_case_name(), terminal);
    }

private:
    void update_simulator_applications_list_based_on_normalized_ticks() {
        _simulator_applications = {};

        for (auto& cyclic_application : _simulator_applications_vector) {
            const std::size_t normalized_ticks = cyclic_application->get_sampling_time() / _fastest_sampling_rate;

            _simulator_applications[normalized_ticks].push_back(cyclic_application);
        }
    }

    void register_application(std::shared_ptr<simulator_runner_application_interface> application_interface) {
        _fastest_sampling_rate = std::min(_fastest_sampling_rate, application_interface->get_sampling_time());

        _simulator_applications_vector.push_back(application_interface);
    }

    void allocate_data_sampler_buffer(double time) {
        std::size_t const buffer_size = convert_time_to_ticks(time);

        _data_sampler.allocate(buffer_size);
    }

    std::size_t convert_time_to_ticks(double time) { return static_cast<std::size_t>(time / _fastest_sampling_rate); }

    void run_data_sampler() { _data_sampler.sample(); }

    void run_applications(std::size_t current_simulation_ticks) {
        for (auto& [normalized_ticks, application_list] : _simulator_applications) {
            if (current_simulation_ticks % normalized_ticks == 0) {
                for (auto& n : application_list) {
                    n->run();
                }
            }
        }
    }

    void run_checkers() {
        double current_time = get_current_simulation_time();
        for (const auto& checker : _checkers) {
            if (checker->is_active(current_time)) {
                bool const checker_result = checker->check();
                EXPECT_TRUE(checker_result)
                    << checker->get_failure_message() << " at " << std::to_string(current_time) << " seconds.";

                if (!checker_result && _max_number_checker_failures > 0) {
                    _max_number_checker_failures--;
                }
            }
        }
    }

    std::size_t _max_number_checker_failures = 5;

    double _fastest_sampling_rate = std::numeric_limits<double>::max();

    std::size_t _current_simulation_time_ticks{0};

    data_sampler _data_sampler;

    std::vector<std::unique_ptr<simulator_checker>> _checkers;

    std::vector<std::shared_ptr<simulator_runner_application_interface>> _simulator_applications_vector{};

    std::map<std::size_t, std::vector<std::shared_ptr<simulator_runner_application_interface>>>
        _simulator_applications{};
};

}  // namespace yeet::testing

#endif  // YEET_TESTING_SIMULATOR_HPP