#ifndef YEET_TESTING_DATA_SAMPLER
#define YEET_TESTING_DATA_SAMPLER

#include <cassert>
#include <cstdint>
#include <iostream>
#include <map>
#include <memory>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

#include "yeet_testing/data_sample_post_processing_interface.hpp"
#include "yeet_testing/data_sampler_entry.hpp"

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

#endif  // YEET_TESTING_DATA_SAMPLER