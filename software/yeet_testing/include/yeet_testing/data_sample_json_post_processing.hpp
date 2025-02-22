#ifndef YEET_TESTING_DATA_SAMPLE_JSON_POST_PROCESSING_HPP
#define YEET_TESTING_DATA_SAMPLE_JSON_POST_PROCESSING_HPP

#include <cassert>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <nlohmann/json.hpp>
#include <sstream>
#include <string>
#include <vector>

#include "yeet_testing/data_sample_post_processing_interface.hpp"
#include "yeet_testing/data_sampler_entry.hpp"

namespace yeet::testing {

//    {
//        "test_suite": "example_suite",
//        "test_name": "example_test",
//        "test_data": [
//          {
//            "name": "scalar_value",
//            "dimension": 1,
//            "data": [1.0, 2.0, 3.0]
//          },
//          {
//            "name": "vector3d",
//            "dimension": 3,
//            "data": [
//              [1.0, 2.0, 3.0],
//              [4.0, 5.0, 6.0],
//              [7.0, 8.0, 9.0]
//            ]
//          }
//        ]
//      }
class data_sample_json_post_processing : public data_sample_post_processing_interface {
public:
    void dump(std::string test_suite, std::string test_name,
              std::map<data_sampler_entry*, std::vector<std::vector<float>>> const& data_map) override {
        nlohmann::json json_test_data = nlohmann::json::array();

        for (auto const& [entry, data] : data_map) {
            nlohmann::json json_entry{};
            json_entry["name"] = entry->get_name();

            // Handle multi-dimensional data
            nlohmann::json data_elements = nlohmann::json::array();
            for (auto const& sample : data) {
                if (sample.size() > 1) {
                    // Multi-dimensional case: create array of values
                    nlohmann::json sample_array = nlohmann::json::array();
                    for (auto const& value : sample) {
                        sample_array.push_back(std::stod(format_and_trim_float_value(value, 5)));
                    }
                    data_elements.push_back(sample_array);
                } else if (sample.size() == 1) {
                    // Single dimension case: add value directly
                    data_elements.push_back(std::stod(format_and_trim_float_value(sample[0], 5)));
                }
            }

            json_entry["dimension"] = entry->get_dimension();
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

}  // namespace yeet::testing

#endif  // YEET_TESTING_DATA_SAMPLE_JSON_POST_PROCESSING_HPP