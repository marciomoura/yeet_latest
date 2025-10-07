#ifndef YEET_CONTROL_INTEGRATOR_HPP
#define YEET_CONTROL_INTEGRATOR_HPP

#include <cassert>

#include "type_traits.hpp"

namespace yeet::control {

template <typename T>
class integrator {
public:
    integrator() = default;
    explicit integrator(double sampling_time) : _sampling_time(sampling_time)
    {
        assert(sampling_time > double(0) && "Sampling time must be positive");
    }

    // Configure sampling time
    void configure(double sampling_time) { _sampling_time = sampling_time; }

    // Reset integrator state
    void reset(T value = T{}) { _value = value; }

    // Update function for fixed sampling time
    //    y[k] = y[k-1] + x[k] * Ts
    // where:
    //
    // y[k] is the current integrator output
    // y[k-1] is the previous integrator output
    // x[k] is the current input
    // Ts is the sampling time
    template <typename InputT = T>
    T update(InputT input)
    {
        assert(_sampling_time > double(0) && "Sampling time must be positive");

        // Update integrator using rectangular integration
        _value += _sampling_time * input;

        return _value;
    }

    // Getter methods
    double get_sampling_time() const { return _sampling_time; }
    T get_output() const { return _value; }

    operator T() const { return _value; }

private:
    double _sampling_time{};  // Sampling time
    T _value{};               // Current integrator value
};

}  // namespace yeet::control

#endif  // YEET_CONTROL_INTEGRATOR_HPP