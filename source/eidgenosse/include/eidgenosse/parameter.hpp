/**
 * @copyright Techperts AG.
 * @date 10-12-2024
 * @brief
 */

#ifndef EIDGENOSSE_PARAMETER_HPP
#define EIDGENOSSE_PARAMETER_HPP

#include <cassert>
#include <memory>
#include <type_traits>

#include "utils/low_level_access.hpp"
#include "utils/storage_type.hpp"

template <typename T>
class parameter {
public:
    parameter() = default;
    ~parameter() = default;

    low_level_access get_low_level_access() { return low_level_access(value_); }

    void set_metadata() {}

    void get_metadata() {}

    T read() const noexcept { return value_; }

    void write(T value) noexcept { value_ = value; }

private:
    storage_type<T> value_{};
};

#endif  // EIDGENOSSE_PARAMETER_HPP