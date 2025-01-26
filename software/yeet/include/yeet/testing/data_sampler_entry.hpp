#ifndef YEET_TESTING_DATA_SAMPLER_ENTRY
#define YEET_TESTING_DATA_SAMPLER_ENTRY

#include <cassert>
#include <functional>
#include <string>

class data_sampler_entry {
public:
    data_sampler_entry() = default;
    data_sampler_entry(std::string name, std::function<float()> sample_function)
        : _name(name), _sample_function(sample_function) {}

    float sample() {
        if (!_sample_function) {
            return 0.0f;
        }
        return _sample_function();
    }

    std::string get_name() const { return _name; }

private:
    std::string _name{};
    std::function<float()> _sample_function{};
};

#endif  // YEET_TESTING_DATA_SAMPLER_ENTRY