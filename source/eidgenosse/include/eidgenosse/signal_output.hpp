/**
 * @copyright Techperts AG.
 * @date 10-12-2024
 * @brief
 */

#ifndef EIDGENOSSE_SIGNAL_OUTPUT_HPP
#define EIDGENOSSE_SIGNAL_OUTPUT_HPP

#include <cassert>
#include <memory>
#include <type_traits>

#include "signal_input.hpp"
#include "utils/low_level_access.hpp"
#include "utils/storage_type.hpp"

template <typename T>
class signal_output {
public:
    signal_output() = default;
    ~signal_output() = default;

    low_level_access get_low_level_access() { return low_level_access(_value); }

    void set_metadata() {}

    void get_metadata() {}

    void connect(signal_input<T>& input) { input.connect(_value); }

    T read() const noexcept { return _value; }

    void write(T value) noexcept { _value = value; }

    T operator=(T value) noexcept { return _value = value; }

    operator T() const noexcept { return read(); }

private:
    storage_type<T> _value{};
};

#endif  // EIDGENOSSE_SIGNAL_OUTPUT_HPP