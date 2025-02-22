#ifndef YEET_TESTING_DATA_SAMPLER_ENTRY
#define YEET_TESTING_DATA_SAMPLER_ENTRY

#include <cassert>
#include <functional>
#include <string>
#include <vector>

namespace yeet::testing {

class data_sampler_entry {
public:
    data_sampler_entry() = default;

    // Constructor for scalar values
    data_sampler_entry(std::string name, std::function<float()> sample_function)
        : _name(std::move(name)),
          _sample_function([f = std::move(sample_function)]() { return std::vector<float>{f()}; }) {}

    // Construct for multi-dimensional data
    data_sampler_entry(std::string name, std::function<std::vector<float>()> sample_function)
        : _name(std::move(name)), _sample_function(std::move(sample_function)) {}

    // Constructor for fundamental types (int, float, double, etc.)
    template <typename T, typename = std::enable_if_t<std::is_fundamental_v<std::decay_t<T>>>>
    data_sampler_entry(std::string name, const T& data)
        : _name(std::move(name)),
          _sample_function([&data]() { return std::vector<float>{static_cast<float>(data)}; }),
          _dimension(1) {}

    // Constructor for container types (vector, array, etc.)
    template <typename ContainerT, typename = std::enable_if_t<!std::is_fundamental_v<std::decay_t<ContainerT>>>,
              typename = std::enable_if_t<
                  std::is_same_v<typename std::iterator_traits<typename ContainerT::iterator>::iterator_category,
                                 std::random_access_iterator_tag>>>
    data_sampler_entry(std::string name, const ContainerT& data)
        : _name(std::move(name)),
          _sample_function([&data]() {
              std::vector<float> samples;
              samples.reserve(data.size());
              for (const auto& value : data) {
                  samples.push_back(static_cast<float>(value));
              }
              return samples;
          }),
          _dimension(data.size()) {}

    // Sample function now returns a vector
    std::vector<float> sample() const {
        if (!_sample_function) {
            return {};
        }
        return _sample_function();
    }

    std::string get_name() const { return _name; }
    std::size_t get_dimension() const { return _dimension; }

private:
    std::string _name{};
    std::function<std::vector<float>()> _sample_function{};
    std::size_t _dimension{1};
};

// make sampler reference
template <typename T>
auto make_lambda_sampler(std::string name, std::function<T()> sample_function) {
    return data_sampler_entry{name, [f = std::move(sample_function)]() -> float { return static_cast<float>(f()); }};
}

// make sampler reference for fundamental types
template <typename T>
auto make_reference_sampler(std::string name, const T& data) {
    return data_sampler_entry{name, data};
}

// Make sampler for lambda functions returning containers
template <typename ContainerT>
auto make_lambda_array_sampler(std::string name, std::function<ContainerT()> sample_function) {
    return data_sampler_entry{name, [f = std::move(sample_function)]() -> std::vector<float> {
                                  auto container = f();
                                  std::vector<float> samples{};
                                  samples.reserve(container.size());
                                  for (const auto& value : container) {
                                      samples.push_back(static_cast<float>(value));
                                  }
                                  return samples;
                              }};
}

// make sampler reference for container types
template <typename ContainerT>
auto make_array_reference_sampler(std::string name, const ContainerT& data) {
    return data_sampler_entry{name, data};
}

}  // namespace yeet::testing

#endif  // YEET_TESTING_DATA_SAMPLER_ENTRY