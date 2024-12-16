#ifndef EIDGENOSSE_CONTROL_EXECUTION_POLICY_HPP
#define EIDGENOSSE_CONTROL_EXECUTION_POLICY_HPP

#include <cstddef>

class control_execution_policy {
public:
    static constexpr double tick_base_rate = 1e-6;

    constexpr control_execution_policy() = default;
    constexpr control_execution_policy(double execution_rate) : _sampling_rate(execution_rate) {}

    constexpr double get_tick_base_rate() const { return tick_base_rate; }

    constexpr std::size_t get_execution_ticks() const {
        return static_cast<std::size_t>(_sampling_rate / tick_base_rate);
    }

    constexpr double get_sampling_time() const { return _sampling_rate; }

private:
    double _sampling_rate{0.0};
};

#endif  // EIDGENOSSE_CONTROL_EXECUTION_POLICY_HPP