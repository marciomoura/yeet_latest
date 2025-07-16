#pragma once

#include "checker_failures_post_processing_interface.hpp"
#include "data_sample_json_post_processing.hpp"
#include "data_sample_post_processing_interface.hpp"
#include "simulator_checker_runner.hpp"
#include "simulator_runner.hpp"
#include "simulator_sampler_runner.hpp"
#include "testing_info_interface.hpp"

namespace yeet::testing {

class simulator {
public:
    simulator() { register_simulation_time_to_data_sampler(); }

    simulator(std::shared_ptr<testing_info_interface> test_info_interface)
        : _test_info_interface(std::move(test_info_interface)) {
        register_simulation_time_to_data_sampler();
    }

    ~simulator() {
        if (!_test_info_interface) {
            return;
        }

        // if (!_checker.has_failure()) {
        //     return;
        // }

        dump_data();
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
        const double sampling_rate = _runner.get_fastest_sampling_rate();
        const std::size_t num_ticks = static_cast<std::size_t>(time / sampling_rate);

        _sampler.allocate(num_ticks);

        for (std::size_t tick = 0; tick < num_ticks; ++tick) {
            _current_simulation_time = (_current_tick + tick) * sampling_rate;
            _sampler.sample();
            _runner.run(tick);
            _checker.run_checks(_current_simulation_time);
        }

        _current_tick += num_ticks;
        return !_checker.has_failure();
    }

    double get_current_simulation_time() const { return _current_simulation_time; }

    void dump_data() {
        assert(_test_info_interface && "Test info interface not set");

        _sampler.dump(_test_info_interface->get_test_suite_name(), _test_info_interface->get_test_case_name(),
                      _json_post_processor);

        _checker.dump(_json_post_processor);

        // Write the JSON data to file
        _json_post_processor.write_to_file();
    }

    simulator_sampler_runner const& get_sampler() const { return _sampler; }

private:
    void register_simulation_time_to_data_sampler() {
        register_sampler_entry(data_sampler_entry("time", [this]() { return _current_simulation_time; }));
    }

    simulator_runner _runner;
    simulator_checker_runner _checker;
    simulator_sampler_runner _sampler;

    std::shared_ptr<testing_info_interface> _test_info_interface{};
    data_sample_json_post_processing _json_post_processor{};

    std::size_t _current_tick{0};
    double _current_simulation_time{0.0};
};

}  // namespace yeet::testing