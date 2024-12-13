#ifndef EIDGENOSSE_TESTING_TEST
#define EIDGENOSSE_TESTING_TEST

#include <algorithm>
#include <cassert>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <nlohmann/json.hpp>
#include <sstream>
#include <string>
#include <vector>

#include "eidgenosse/application_interface.hpp"
#include "eidgenosse/application_registry.hpp"

class data_sampler_entry {
public:
    data_sampler_entry() = default;
    data_sampler_entry(std::string name, std::function<float()> sample_function)
        : _name(name), _sample_function(sample_function) {}

    float sample() {
        if (!_sample_function) {
            return 0.0f;
        }
        return _sample_function();
    }

    std::string get_name() const { return _name; }

private:
    std::string _name{};
    std::function<float()> _sample_function{};
};

class data_sample_post_processing_interface {
public:
    virtual void dump(std::map<std::unique_ptr<data_sampler_entry>, std::vector<float>> const& data) {
        // todo
    }

    virtual void dump(std::string test_suite, std::string test_name,
                      std::map<std::unique_ptr<data_sampler_entry>, std::vector<float>> const& data) {
        // todo
    }
};

class data_sample_terminal_post_processing : public data_sample_post_processing_interface {
public:
    virtual void dump(std::map<std::unique_ptr<data_sampler_entry>, std::vector<float>> const& data) {
        for (auto& [entry, data] : data) {
            std::cout << entry->get_name() << ": ";
            for (auto& value : data) {
                std::cout << value << " ";
            }
            std::cout << std::endl;
        }
    }
};

class data_sample_json_post_processing : public data_sample_post_processing_interface {
public:
    void dump(std::string test_suite, std::string test_name,
              std::map<std::unique_ptr<data_sampler_entry>, std::vector<float>> const& data_map) override {
        nlohmann::json json_test_data = nlohmann::json::array();
        for (auto& [entry, data] : data_map) {
            nlohmann::json data_elements{};
            for (auto& value : data) {
                data_elements.push_back(std::stod(format_and_trim_float_value(value, 5)));
            }

            nlohmann::json json_entry{};
            json_entry["name"] = entry->get_name();
            json_entry["data"] = data_elements;
            json_test_data.push_back(json_entry);
        }

        nlohmann::json json_data{};
        json_data["test_suite"] = test_suite;
        json_data["test_name"] = test_name;
        json_data["test_data"] = json_test_data;

        std::string const filename = "testing_" + test_suite + "_" + test_name + ".json";

        std::filesystem::path const path = std::filesystem::current_path() / filename;

        dump_json_to_file(json_data, path);
    }

private:
    static inline void dump_json_to_file(nlohmann::json const& json_data, std::filesystem::path const& path) {
        try {
            std::ofstream file(path);
            if (!file.is_open()) {
                throw std::ios_base::failure("Unable to open file");
            }
            file << json_data.dump();
            file.close();
        } catch (const std::ios_base::failure& e) {
            std::cerr << "File operation failed: " << e.what() << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "An error occurred: " << e.what() << std::endl;
        }
    }

    static inline std::string format_and_trim_float_value(float value, int precision) {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(precision) << value;
        std::string formatted_value = oss.str();

        // Remove trailing zeros
        formatted_value.erase(formatted_value.find_last_not_of('0') + 1, std::string::npos);

        // Add a `0` if the last decimal point is a '.'
        if (formatted_value.back() == '.') {
            formatted_value.push_back('0');
        }

        return formatted_value;
    }
};

class data_sampler {
public:
    void register_entry(data_sampler_entry entry) {
        _data[std::make_unique<data_sampler_entry>(entry)] = std::vector<float>();
    }

    void allocate(std::size_t size) {
        for (auto& [entry, data] : _data) {
            data.resize(data.size() + size);
        }
    }

    void sample() {
        for (auto& [entry, data] : _data) {
            if (_sample_position >= data.size()) {
                std::cout << "Sample position exceeds data size" << std::endl;
            } else {
                data[_sample_position] = entry->sample();
            }
        }

        _sample_position++;
    }

    void dump(data_sample_post_processing_interface& data_sample_post_processing_interface) {
        data_sample_post_processing_interface.dump(_data);
    }

    void dump(std::string test_suite, std::string test_name,
              data_sample_post_processing_interface& data_sample_post_processing_interface) {
        data_sample_post_processing_interface.dump(test_suite, test_name, _data);
    }

private:
    std::uint32_t _sample_position{0};

    std::map<std::unique_ptr<data_sampler_entry>, std::vector<float>> _data{};
};

class simulator_runner_application_interface {
public:
    virtual double get_sampling_rate() = 0;

    virtual void run() = 0;

    virtual void initialize() = 0;

    virtual void stop() = 0;

    // virtual void register_data_sampler_entries(data_sampler& data_sampler) = 0;
};

class control_task_simulator_runner : public simulator_runner_application_interface {
public:
    control_task_simulator_runner(control_task_level control_task) : _control_task(control_task) {}

    double get_sampling_rate() override { return _control_task.get_control_execution_policy().get_sampling_rate(); }

    void run() override { _control_task.run(); }

    void initialize() override { _control_task.initialize(); }

    void stop() override { _control_task.stop(); }

private:
    control_task_level _control_task;
};

class testing_info_interface {
public:
    virtual std::string get_test_suite_name() const = 0;
    virtual std::string get_test_case_name() const = 0;
};

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

#endif  // EIDGENOSSE_TESTING_TEST