/**
 * @copyright Techperts AG.
 * @date 10-12-2024
 * @brief
 */

#ifndef EIDGENOSSE_SIGNAL_INPUT_HPP
#define EIDGENOSSE_SIGNAL_INPUT_HPP

#include <cassert>
#include <memory>
#include <type_traits>

#include "utils/low_level_access.hpp"
#include "utils/storage_type.hpp"

template <typename T>
class signal_input {
public:
    signal_input() = default;
    ~signal_input() = default;

    low_level_access get_low_level_access() { return low_level_access(_value); }

    void set_metadata() {}

    void get_metadata() {}

    void connect(storage_type<T>& storage) { _value = &storage; }

    T read() const noexcept { return *_value; }

    operator T() const noexcept { return read(); }

private:
    storage_type<T>* _value{};
};

#endif  // EIDGENOSSE_SIGNAL_INPUT_HPP