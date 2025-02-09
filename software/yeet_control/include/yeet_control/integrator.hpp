#ifndef YEET_CONTROL_INTEGRATOR_HPP
#define YEET_CONTROL_INTEGRATOR_HPP

#include <cassert>

namespace yeet::control {

template <typename T, typename CoefficientT = T>
class integrator {
public:
    // Configure sampling time
    void configure(CoefficientT ts) { _ts = ts; }

    // Reset integrator state
    void reset() { _value = T(0); }

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
        assert(_ts > CoefficientT(0) && "Sampling time must be positive");

        // Update integrator using rectangular integration
        _value += T{input * _ts};

        return _value;
    }

    // Getter methods
    CoefficientT get_sampling_time() const { return _ts; }
    T get_value() const { return _value; }

private:
    CoefficientT _ts{};  // Sampling time
    T _value{};          // Current integrator value
};

}  // namespace yeet::control

#endif  // YEET_CONTROL_INTEGRATOR_HPP