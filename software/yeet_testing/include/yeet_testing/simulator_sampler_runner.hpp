#pragma once

#include "data_sampler.hpp"

namespace yeet::testing {

class simulator_sampler_runner {
public:
    simulator_sampler_runner() {}

    void register_entry(data_sampler_entry entry) { _sampler.register_entry(std::move(entry)); }

    void allocate(std::size_t num_samples) { _sampler.allocate_more(num_samples); }

    void sample() { _sampler.sample(); }

    void dump(const std::string& test_suite, const std::string& test_name,
              data_sample_post_processing_interface& post_processor) {
        _sampler.dump(test_suite, test_name, post_processor);
    }

    std::vector<std::vector<float>> const& get_sampled_data(const std::string& entry_name) const {
        return _sampler.get_sampled_data(entry_name);
    }

private:
    data_sampler _sampler;
};

}  // namespace yeet::testing