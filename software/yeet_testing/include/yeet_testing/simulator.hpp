#pragma once

#include "data_sample_json_post_processing.hpp"
#include "simulator_checker_runner.hpp"
#include "simulator_runner.hpp"
#include "simulator_sampler_runner.hpp"
#include "yeet_testing/testing_info_interface.hpp"

namespace yeet::testing {

class simulator {
public:
    simulator() {
        // Register time sampler entry
        register_sampler_entry(data_sampler_entry("time", [this]() { return _current_simulation_time; }));
    }

    void register_lambda_runner(double sampling_time, std::function<void()> lambda_runner) {
        _runner.register_lambda_runner(sampling_time, std::move(lambda_runner));
    }

    void register_checker(std::unique_ptr<simulator_checker> checker) { _checker.register_checker(std::move(checker)); }

    template <typename CheckerType>
    void register_checker(CheckerType checker) {
        _checker.register_checker(std::move(checker));
    }

    void register_sampler_entry(data_sampler_entry entry) { _sampler.register_entry(std::move(entry)); }

    void initialize() {
        _runner.initialize();
        _current_simulation_time = 0.0;
    }

    bool simulate_for(double time, bool short_circuit_upon_reaching_max_failures = false) {
        if (short_circuit_upon_reaching_max_failures && _checker.has_reached_max_failures()) {
            return false;
        }

        const double sampling_rate = _runner.get_fastest_sampling_rate();
        const std::size_t num_ticks = static_cast<std::size_t>(time / sampling_rate);

        _sampler.allocate(num_ticks);

        bool status = true;

        for (std::size_t tick = 0; tick < num_ticks; ++tick) {
            _current_simulation_time = (_current_tick + tick) * sampling_rate;

            _sampler.sample();
            _runner.run(tick);

            if (!_checker.run_checks(_current_simulation_time)) {
                status = false;
                if (short_circuit_upon_reaching_max_failures && _checker.has_reached_max_failures()) {
                    _current_tick += tick;
                    return false;
                }
            }
        }

        _current_tick += num_ticks;
        return status;
    }

    double get_current_simulation_time() const { return _current_simulation_time; }

    void dump_json(testing_info_interface const& test_info) {
        data_sample_json_post_processing json_post_processor;
        _sampler.dump(test_info.get_test_suite_name(), test_info.get_test_case_name(), json_post_processor);
    }

private:
    simulator_runner _runner;
    simulator_checker_runner _checker;
    simulator_sampler_runner _sampler;

    std::size_t _current_tick{0};
    double _current_simulation_time{0.0};
};

}  // namespace yeet::testing