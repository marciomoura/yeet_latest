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

    template <typename T>
    void register_runner(T& runner)
    {
        register_lambda_runner(runner.get_sampling_time(), [&]() { runner.run(); });
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
        _current_tick = 0;
    }

    /**
     * @brief Execute a single simulation tick at the fastest registered sampling rate.
     *
     * This method enables real-time execution where the host platform controls timing.
     * The simulator will execute all registered applications that should run at the
     * current tick based on their sampling rates.
     *
     * Data logging and checker validation are not performed in this mode - those are
     * intended for batch simulation using simulate_for(). Real-time execution typically
     * uses external monitoring (e.g., TCP telemetry servers).
     *
     * @note For time-based batch simulation with data logging, use simulate_for() instead.
     * @note Must call initialize() before first call to run_single_tick().
     */
    void run_single_tick()
    {
        const double simulation_sampling_rate = _runner.get_fastest_sampling_rate();
        _current_simulation_time = _current_tick * simulation_sampling_rate;

        _runner.run(_current_tick);

        _current_tick++;
    } /**
       * @brief Simulate for a specified duration using time-based execution.
       *
       * This is the traditional batch simulation method used in integration tests.
       * It executes the required number of ticks to cover the specified time duration.
       *
       * @param time Duration to simulate in seconds
       * @param short_circuit_upon_reaching_max_failures Stop immediately on checker failure
       * @return true if no checker failures detected, false otherwise
       */
    bool simulate_for(double time, bool short_circuit_upon_reaching_max_failures = false)
    {
        const double simulation_sampling_rate = _runner.get_fastest_sampling_rate();
        const std::size_t num_simulation_ticks = static_cast<std::size_t>(std::ceil(time / simulation_sampling_rate));

#ifdef YEET_ENABLE_DATA_LOGGING
        const double sampler_sampling_rate = _use_custom_sampler_rate ? _custom_sampler_rate : simulation_sampling_rate;

        // Calculate the sampling ratio (how many simulation ticks per sample)
        const std::size_t sampling_ratio =
            _use_custom_sampler_rate
                ? std::max(static_cast<std::size_t>(std::ceil(sampler_sampling_rate / simulation_sampling_rate)),
                      std::size_t{1})
                : std::size_t{1};

        // Calculate exact number of samples that will actually be taken
        // This matches the loop condition: tick % sampling_ratio == 0
        const std::size_t num_sampler_ticks = (num_simulation_ticks + sampling_ratio - 1) / sampling_ratio;

        _sampler.allocate(num_sampler_ticks);
#endif

        for (std::size_t tick = 0; tick < num_simulation_ticks; ++tick) {
            _current_simulation_time = _current_tick * simulation_sampling_rate;

#ifdef YEET_ENABLE_DATA_LOGGING
            // Sample only at the specified rate
            if (_current_tick % sampling_ratio == 0) {
                _sampler.sample();
            }
#endif

            _runner.run(_current_tick);
            _checker.run_checks(_current_simulation_time);

            _current_tick++;

            // Short circuit if requested and failures detected
            if (short_circuit_upon_reaching_max_failures && _checker.has_failure()) {
                return false;
            }
        }

        return !_checker.has_failure();
    }

    /**
     * @brief Get the current simulation time in seconds.
     * @return Current simulation time
     */
    double get_current_simulation_time() const { return _current_simulation_time; }

    /**
     * @brief Get the current simulation tick count.
     * @return Number of ticks executed since initialization
     */
    std::size_t get_current_tick() const { return _current_tick; }

    /**
     * @brief Get the period of a single simulation tick.
     * @return Tick period in seconds (inverse of fastest sampling rate)
     */
    double get_tick_period() const { return _runner.get_fastest_sampling_rate(); }

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