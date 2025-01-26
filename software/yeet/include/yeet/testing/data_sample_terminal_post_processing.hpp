#ifndef YEET_TESTING_DATA_SAMPLER_TERMINAL_POST_PROCESSING_HPP
#define YEET_TESTING_DATA_SAMPLER_TERMINAL_POST_PROCESSING_HPP

#include <cassert>
#include <iostream>
#include <map>
#include <memory>
#include <vector>

#include "yeet/testing/data_sample_post_processing_interface.hpp"
#include "yeet/testing/data_sampler_entry.hpp"

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

#endif  // YEET_TESTING_DATA_SAMPLER_TERMINAL_POST_PROCESSING_HPP