/**
 * @copyright Techperts AG.
 * @date 10-12-2024
 * @brief
 */

#ifndef EIDGENOSSE_TESTING_TEST
#define EIDGENOSSE_TESTING_TEST

#include <cstdint>
#include <functional>
#include <iostream>
#include <map>
#include <memory>
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

class data_sampler {
public:
    void register_entry(data_sampler_entry entry) {
        data_[std::make_unique<data_sampler_entry>(entry)] = std::vector<float>();
    }

    void resize(std::size_t size) {
        for (auto& [entry, data] : data_) {
            data.resize(size);
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

    std::map<std::unique_ptr<data_sampler_entry>, std::vector<float>> data_;
};

class cyclic_application_interface {
public:
    virtual void run() = 0;

    virtual void register_data_sampler_entries(data_sampler& data_sampler) {}
};

class simulator {
public:
    simulator() {
        data_sampler_entry entry("time", [&]() { return static_cast<float>(time_); });

        data_sampler_.register_entry(entry);
    }

    // register new cyclic_application
    void register_cyclic_application(std::uint32_t ticks_per_second,
                                     std::shared_ptr<cyclic_application_interface> cyclic_application) {
        cyclic_application->register_data_sampler_entries(data_sampler_);
        cyclic_applications_[ticks_per_second] = cyclic_application;
    }

    void simulate_for() {
        data_sampler_.resize(10);

        for (time_ = 0; time_ < 10; ++time_) {
            run_once();
        }
    }

    void dump() {
        data_sample_terminal_post_processing terminal;
        data_sampler_.dump(terminal);
    }

private:
    void run_once() {
        for (auto& [id, cyclic_application] : cyclic_applications_) {
            cyclic_application->run();
        }

        data_sampler_.sample();
    }

    int time_{0};

    data_sampler data_sampler_;

    std::map<std::uint32_t, std::shared_ptr<cyclic_application_interface>> cyclic_applications_;
};

#endif  // EIDGENOSSE_TESTING_TEST