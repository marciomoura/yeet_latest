#ifndef EIDGENOSSE_TESTING_SIMULATOR_HPP
#define EIDGENOSSE_TESTING_SIMULATOR_HPP

#include <algorithm>
#include <cassert>
#include <functional>
#include <map>
#include <memory>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

#include "eidgenosse/testing/data_sample_json_post_processing.hpp"
#include "eidgenosse/testing/data_sample_terminal_post_processing.hpp"
#include "eidgenosse/testing/data_sampler.hpp"
#include "eidgenosse/testing/data_sampler_entry.hpp"
#include "eidgenosse/testing/simulator_runner_application_interface.hpp"
#include "eidgenosse/testing/testing_info_interface.hpp"


class simulator {
public:
    simulator() {
        data_sampler_entry entry("time", [&]() { return _current_simulation_time_ticks * _fastest_sampling_rate; });

        _data_sampler.register_entry(entry);
    }

    void register_cyclic_application(std::shared_ptr<simulator_runner_application_interface> application_interface) {
        _fastest_sampling_rate = std::min(_fastest_sampling_rate, application_interface->get_sampling_rate());

        _simulator_applications_vector.push_back(application_interface);
    }

    void register_data_sampler_entry(data_sampler_entry entry) { _data_sampler.register_entry(entry); }

    void register_applications() {
        _simulator_applications = {};

        for (auto& cyclic_application : _simulator_applications_vector) {
            const std::size_t normalized_ticks = cyclic_application->get_sampling_rate() / _fastest_sampling_rate;

            _simulator_applications[normalized_ticks].push_back(cyclic_application);
        }
    }

    void initialize() {
        for (auto& [normalized_ticks, application_list] : _simulator_applications) {
            for (auto& n : application_list) {
                n->initialize();
            }
        }
    }

    void simulate_for(double time) {
        assert(_fastest_sampling_rate < std::numeric_limits<float>::max() && "No cyclic applications registered");
        assert(_fastest_sampling_rate > 0.0f && "tick_sampling_rate_ must be greater than 0.0f");

        std::size_t const simulation_end_time_ticks = _current_simulation_time_ticks + convert_time_to_ticks(time);

        allocate_data_sampler_buffer(time);

        while (_current_simulation_time_ticks < simulation_end_time_ticks) {
            run_data_sampler();

            run_once(_current_simulation_time_ticks);

            _current_simulation_time_ticks += 1;
        }
    }

    void dump() {
        data_sample_terminal_post_processing terminal;
        _data_sampler.dump(terminal);
    }

    void dump_json(testing_info_interface* testing_info) {
        data_sample_json_post_processing terminal;
        _data_sampler.dump(testing_info->get_test_suite_name(), testing_info->get_test_case_name(), terminal);
    }

private:
    void allocate_data_sampler_buffer(double time) {
        std::size_t const buffer_size = convert_time_to_ticks(time);

        _data_sampler.allocate(buffer_size);
    }

    std::size_t convert_time_to_ticks(double time) { return static_cast<std::size_t>(time / _fastest_sampling_rate); }

    void run_data_sampler() { _data_sampler.sample(); }

    void run_once(std::size_t current_simulation_ticks) {
        for (auto& [normalized_ticks, application_list] : _simulator_applications) {
            if (current_simulation_ticks % normalized_ticks == 0) {
                for (auto& n : application_list) {
                    n->run();
                }
            }
        }
    }

    double _fastest_sampling_rate = std::numeric_limits<double>::max();

    std::size_t _current_simulation_time_ticks{0};

    data_sampler _data_sampler;

    std::vector<std::shared_ptr<simulator_runner_application_interface>> _simulator_applications_vector{};

    std::map<std::size_t, std::vector<std::shared_ptr<simulator_runner_application_interface>>>
        _simulator_applications{};
};

#endif  // EIDGENOSSE_TESTING_SIMULATOR_HPP