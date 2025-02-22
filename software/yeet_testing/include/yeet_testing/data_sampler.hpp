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

namespace yeet::testing {

class data_sampler {
public:
    void register_entry(data_sampler_entry entry) {
        auto ptr = std::make_unique<data_sampler_entry>(std::move(entry));
        _data[ptr.get()] = std::vector<std::vector<float>>();
        _entries.push_back(std::move(ptr));
    }

    void allocate(std::size_t size) {
        for (const auto& entry : _entries) {
            auto& data = _data[entry.get()];
            data.resize(size);
        }
    }

    void sample() {
        for (const auto& entry : _entries) {
            auto& data = _data[entry.get()];
            if (_sample_position >= data.size()) {
                std::cout << "Sample position exceeds data size" << std::endl;
                continue;
            }
            data[_sample_position] = entry->sample();
        }
        _sample_position++;
    }

    // Updated dump functions to handle multi-dimensional data
    void dump(data_sample_post_processing_interface& post_processor) { post_processor.dump(_data); }

    void dump(std::string test_suite, std::string test_name, data_sample_post_processing_interface& post_processor) {
        post_processor.dump(test_suite, test_name, _data);
    }

private:
    std::uint32_t _sample_position{0};
    std::vector<std::shared_ptr<data_sampler_entry>> _entries{};
    std::map<data_sampler_entry*, std::vector<std::vector<float>>> _data{};
};

}  // namespace yeet::testing

#endif  // YEET_TESTING_DATA_SAMPLER