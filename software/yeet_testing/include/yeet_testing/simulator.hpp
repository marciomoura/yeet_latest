#pragma once

#include "checker_failures_post_processing_interface.hpp"
#include "data_sample_json_post_processing.hpp"
#include "data_sample_post_processing_interface.hpp"
#include "simulator_checker_runner.hpp"
#include "simulator_runner.hpp"
#include "simulator_sampler_runner.hpp"
#include "testing_info_interface.hpp"

namespace yeet {

class simulator {
public:
    simulator() { register_simulation_time_to_data_sampler(); }

    simulator(std::shared_ptr<testing_info_interface> test_info_interface)
        : _test_info_interface(std::move(test_info_interface))
    {
        register_simulation_time_to_data_sampler();
    }

    ~simulator()
    {
        if (!_test_info_interface) {
            return;
        }

#ifdef YEET_ENABLE_DATA_LOGGING
        // Only dump data if logging is not disabled
        dump_data();
#endif
    }

    void register_lambda_runner(double sampling_time, std::function<void()> lambda_runner)
    {
        _runner.register_lambda_runner(sampling_time, std::move(lambda_runner));
    }

    std::shared_ptr<checker_user_activation> register_checker(std::unique_ptr<simulator_checker> checker)
    {
        return _checker.register_checker(std::move(checker));
    }

    template <typename CheckerType>
    std::shared_ptr<checker_user_activation> register_checker(CheckerType checker)
    {
        return _checker.register_checker(std::move(checker));
    }

    void register_sampler_entry(data_sampler_entry entry)
    {
#ifdef YEET_ENABLE_DATA_LOGGING
        _sampler.register_entry(std::move(entry));
#else
        // When data logging is disabled, don't register samplers to avoid overhead
        (void)entry;  // Suppress unused parameter warning
#endif
    }

    /**
     * @brief Configures the sampling rate for data collection.
     * @param sampling_rate The desired sampling rate in seconds. If <= 0, uses the maximum simulation rate.
     */
    void configure_sampler_sampling_rate(double sampling_rate)
    {
        if (sampling_rate <= 0.0) {
            _use_custom_sampler_rate = false;
        }
        else {
            _custom_sampler_rate = sampling_rate;
            _use_custom_sampler_rate = true;
        }
    }

    void initialize()
    {
        _runner.initialize();
        _current_simulation_time = 0.0;
    }

    bool simulate_for(double time, bool short_circuit_upon_reaching_max_failures = false)
    {
        const double simulation_sampling_rate = _runner.get_fastest_sampling_rate();
        const std::size_t num_simulation_ticks = static_cast<std::size_t>(time / simulation_sampling_rate);

#ifdef YEET_ENABLE_DATA_LOGGING
        const double sampler_sampling_rate = _use_custom_sampler_rate ? _custom_sampler_rate : simulation_sampling_rate;
        const double num_sampler_ticks_float = time / sampler_sampling_rate;
        std::size_t num_sampler_ticks = static_cast<std::size_t>(num_sampler_ticks_float);
        // Add one to account for the fractional part, ensuring enough space is allocated.
        // This also accounts for the first sample at t=0.
        if (num_sampler_ticks_float > static_cast<double>(num_sampler_ticks)) {
            num_sampler_ticks++;
        }

        // Calculate the sampling ratio (how many simulation ticks per sample)
        const std::size_t sampling_ratio =
            _use_custom_sampler_rate
                ? std::max(static_cast<std::size_t>(sampler_sampling_rate / simulation_sampling_rate), std::size_t{1})
                : std::size_t{1};

        _sampler.allocate(num_sampler_ticks);
#endif

        for (std::size_t tick = 0; tick < num_simulation_ticks; ++tick) {
            _current_simulation_time = (_current_tick + tick) * simulation_sampling_rate;

#ifdef YEET_ENABLE_DATA_LOGGING
            // Sample only at the specified rate
            if (tick % sampling_ratio == 0) {
                _sampler.sample();
            }
#endif

            _runner.run(tick);
            _checker.run_checks(_current_simulation_time);
        }

        _current_tick += num_simulation_ticks;
        return !_checker.has_failure();
    }

    double get_current_simulation_time() const { return _current_simulation_time; }

    void dump_data()
    {
        assert(_test_info_interface && "Test info interface not set");

        _sampler.dump(_test_info_interface->get_test_suite_name(), _test_info_interface->get_test_case_name(),
            _json_post_processor);

        _checker.dump(_json_post_processor);

        // Write the JSON data to file
        _json_post_processor.write_to_file();
    }

    simulator_sampler_runner const& get_sampler() const { return _sampler; }

private:
    void register_simulation_time_to_data_sampler()
    {
        register_sampler_entry(data_sampler_entry("time", [this]() { return _current_simulation_time; }));
    }

    simulator_runner _runner;
    simulator_checker_runner _checker;
    simulator_sampler_runner _sampler;

    std::shared_ptr<testing_info_interface> _test_info_interface{};
    data_sample_json_post_processing _json_post_processor{};

    std::size_t _current_tick{0};
    double _current_simulation_time{0.0};

    // Sampler rate configuration
    bool _use_custom_sampler_rate{false};
    double _custom_sampler_rate{0.0};
};

}  // namespace yeet