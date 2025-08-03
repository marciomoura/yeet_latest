#ifndef YEET_TESTING_DATA_SAMPLER_TERMINAL_POST_PROCESSING_HPP
#define YEET_TESTING_DATA_SAMPLER_TERMINAL_POST_PROCESSING_HPP

#include <cassert>
#include <iostream>
#include <map>
#include <memory>
#include <vector>

#include "yeet_testing/data_sample_post_processing_interface.hpp"
#include "yeet_testing/data_sampler_entry.hpp"

namespace yeet {

class data_sample_terminal_post_processing : public data_sample_post_processing_interface {
public:
    virtual void dump(std::map<data_sampler_entry*, std::vector<std::vector<float>>> const& data)
    {
        for (auto& [entry, data] : data) {
            std::cout << entry->get_name() << ": ";
            for (auto& value : data) {
                for (auto& value : value) {
                    std::cout << value << " ";
                }
            }
            std::cout << std::endl;
        }
    }
};

}  // namespace yeet

#endif  // YEET_TESTING_DATA_SAMPLER_TERMINAL_POST_PROCESSING_HPP