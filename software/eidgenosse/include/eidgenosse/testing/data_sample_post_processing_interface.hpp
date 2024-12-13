#ifndef EIDGENOSSE_TESTING_DATA_SAMPLE_POST_PROCESSING_INTERFACE_HPP
#define EIDGENOSSE_TESTING_DATA_SAMPLE_POST_PROCESSING_INTERFACE_HPP

#include <cassert>
#include <map>
#include <memory>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

#include "eidgenosse/testing/data_sampler_entry.hpp"

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

#endif  // EIDGENOSSE_TESTING_DATA_SAMPLE_POST_PROCESSING_INTERFACE_HPP