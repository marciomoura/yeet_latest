#ifndef YEET_CONTROL_INTEGRATOR_HPP
#define YEET_CONTROL_INTEGRATOR_HPP

#include <cassert>

#include "type_traits.hpp"

namespace yeet::control {

template <typename T>
class integrator {
public:
    using value_type = fundamental_type_t<T>;

    // Configure sampling time
    void configure(value_type ts) { _ts = ts; }

    // Reset integrator state
    void reset() { _value = {}; }

    // Set specific integrator value
    void set_value(T value) { _value = value; }

    // Update function for fixed sampling time
    //    y[k] = y[k-1] + x[k] * Ts
    // where:
    //
    // y[k] is the current integrator output
    // y[k-1] is the previous integrator output
    // x[k] is the current input
    // Ts is the sampling time
    template <typename InputT = T>
    T update(InputT input) {
        assert(_ts > value_type(0) && "Sampling time must be positive");

        // Update integrator using rectangular integration
        _value += T{input * _ts};

        return _value;
    }

    // Getter methods
    value_type get_sampling_time() const { return _ts; }
    T get_value() const { return _value; }

private:
    value_type _ts{};  // Sampling time
    T _value{};        // Current integrator value
};

}  // namespace yeet::control

#endif  // YEET_CONTROL_INTEGRATOR_HPP