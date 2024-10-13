/**
 * @copyright Techperts AG.
 * @date 10-12-2024
 * @brief
 */

#ifndef EIDGENOSSE_TESTING_TEST
#define EIDGENOSSE_TESTING_TEST

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

class data_sampler_entry {
public:
    data_sampler_entry() = default;
    data_sampler_entry(std::string name, std::function<float()> sample_function)
        : name_(name), sample_function_(sample_function) {}

    float sample() {
        if (!sample_function_) {
            return 0.0f;
        }
        return sample_function_();
    }

    std::string get_name() const { return name_; }

private:
    std::string name_{};
    std::function<float()> sample_function_{};
};

class data_sample_post_processing_interface {
public:
    virtual void dump(std::map<std::unique_ptr<data_sampler_entry>, std::vector<float>> const& data) = 0;
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
    virtual void dump(std::map<std::unique_ptr<data_sampler_entry>, std::vector<float>> const& data_map) {
        nlohmann::json json_data{};

        for (auto& [entry, data] : data_map) {
            nlohmann::json data_elements{};
            for (auto& value : data) {
                data_elements.push_back(std::stod(format_and_trim_float_value(value, 5)));
            }

            nlohmann::json json_entry{};
            json_entry["name"] = entry->get_name();
            json_entry["data"] = data_elements;
            json_data.push_back(json_entry);
        }

        std::filesystem::path path = std::filesystem::current_path() / "data.json";

        dump_json_to_file(json_data, path);
    }

private:
    static inline void dump_json_to_file(nlohmann::json const& json_data, std::filesystem::path const& path) {
        try {
            std::ofstream file(path);
            if (!file.is_open()) {
                throw std::ios_base::failure("Unable to open file");
            }
            file << json_data.dump(4);
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
        data_[std::make_unique<data_sampler_entry>(entry)] = std::vector<float>();
    }

    void allocate(std::size_t size) {
        for (auto& [entry, data] : data_) {
            data.resize(data.size() + size);
        }
    }

    void sample() {
        for (auto& [entry, data] : data_) {
            data[sample_position_] = entry->sample();
        }

        sample_position_++;
    }

    void dump(data_sample_post_processing_interface& data_sample_post_processing_interface) {
        data_sample_post_processing_interface.dump(data_);
    }

private:
    std::uint32_t sample_position_{0};

    std::map<std::unique_ptr<data_sampler_entry>, std::vector<float>> data_{};
};

class cyclic_application_interface {
public:
    virtual void run() = 0;

    virtual void register_data_sampler_entries(data_sampler& data_sampler) {}
};

class simulator {
public:
    simulator() {
        data_sampler_entry entry("time", [&]() { return current_simulation_time_; });

        data_sampler_.register_entry(entry);
    }

    // register new cyclic_application
    void register_cyclic_application(std::uint32_t ticks_per_second,
                                     std::shared_ptr<cyclic_application_interface> cyclic_application) {
        cyclic_application->register_data_sampler_entries(data_sampler_);
        cyclic_applications_[ticks_per_second] = cyclic_application;

        // tick_sampling_rate_ = 1.0f / static_cast<float>(ticks_per_second);
    }

    void simulate_for(float time) {
        assert(tick_sampling_rate_ > 0.0f && "tick_sampling_rate_ must be greater than 0.0f");

        float const simulation_end_time = current_simulation_time_ + time;

        allocate_data_sampler_buffer(time);

        while (current_simulation_time_ < simulation_end_time) {
            current_simulation_ticks_ += 1;
            current_simulation_time_ = current_simulation_ticks_ * tick_sampling_rate_;

            run_once(current_simulation_ticks_);
            run_data_sampler();
        }
    }

    void dump() {
        data_sample_terminal_post_processing terminal;
        data_sampler_.dump(terminal);
    }

    void dump_json() {
        data_sample_json_post_processing terminal;
        data_sampler_.dump(terminal);
    }

private:
    void allocate_data_sampler_buffer(float time) {
        std::size_t const buffer_size = static_cast<std::size_t>(time / tick_sampling_rate_);
        data_sampler_.allocate(buffer_size);
    }

    void run_data_sampler() { data_sampler_.sample(); }

    void run_once(std::size_t ticks) {
        for (auto& [id, cyclic_application] : cyclic_applications_) {
            if (ticks % id == 0) {
                cyclic_application->run();
            }
        }
    }

    float tick_sampling_rate_{0.1f};  // this needs to be configured - another class just to handle this?

    std::size_t current_simulation_ticks_{0};

    float current_simulation_time_{0};

    data_sampler data_sampler_;

    std::map<std::size_t, std::shared_ptr<cyclic_application_interface>> cyclic_applications_;
};

#endif  // EIDGENOSSE_TESTING_TEST