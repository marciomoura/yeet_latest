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

namespace yeet {

class data_sampler {
public:
    void register_entry(data_sampler_entry entry)
    {
        auto ptr = std::make_unique<data_sampler_entry>(std::move(entry));
        _data[ptr.get()] = std::vector<std::vector<float>>();
        _entries.push_back(std::move(ptr));
    }

    void allocate(std::size_t size)
    {
        for (const auto& entry : _entries) {
            auto& data = _data[entry.get()];
            data.resize(size);
        }
    }

    void allocate_more(std::size_t size)
    {
        for (const auto& entry : _entries) {
            auto& data = _data[entry.get()];
            data.resize(data.size() + size);
            // Also we pre-allocate the space for the multi-dimensional data samples
            for (std::size_t i = 0; i < size; ++i) {
                data[data.size() - size + i].resize(entry->get_dimension());
            }
        }
    }

    void sample()
    {
        for (const auto& entry : _entries) {
            auto& data = _data[entry.get()];
            if (_sample_position >= data.size()) {
                std::cout << "Sample position exceeds data size. Sample position: " << _sample_position
                          << ", data size: " << data.size() << std::endl;
                continue;
            }
            data[_sample_position] = entry->sample();
        }
        _sample_position++;
    }

    // Updated dump functions to handle multi-dimensional data
    void dump(data_sample_post_processing_interface& post_processor) { post_processor.dump(_data); }

    void dump(const std::string& test_suite,
        const std::string& test_name,
        data_sample_post_processing_interface& post_processor)
    {
        post_processor.dump(test_suite, test_name, _data);
    }

    std::vector<std::vector<float>> const& get_sampled_data(const std::string& entry_name) const
    {
        for (const auto& entry : _entries) {
            if (entry->get_name() == entry_name) {
                auto it = _data.find(entry.get());
                if (it != _data.end()) {
                    return it->second;
                }
            }
        }

        static std::vector<std::vector<float>> empty_vector{};
        assert(!empty_vector.empty() && "No data found for the requested entry");
        return empty_vector;
    }

private:
    std::uint32_t _sample_position{0};
    std::vector<std::shared_ptr<data_sampler_entry>> _entries{};
    std::map<data_sampler_entry*, std::vector<std::vector<float>>> _data{};
};

}  // namespace yeet

#endif  // YEET_TESTING_DATA_SAMPLER