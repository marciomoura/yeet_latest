/**
 * @copyright Techperts AG.
 * @date 10-12-2024
 * @brief
 */

#ifndef EIDGENOSSE_UTILS_LOW_LEVEL_ACCESS_HPP
#define EIDGENOSSE_UTILS_LOW_LEVEL_ACCESS_HPP

#include <cassert>
#include <memory>
#include <type_traits>

#include "storage_type.hpp"

class low_level_access {
public:
    low_level_access() = default;

    template <typename T>
    low_level_access(storage_type<T> value) : value(value) {}

    low_level_access(storage_type_erased value) : value(value) {}
    ~low_level_access() = default;

    bool empty() const noexcept { return value.empty(); }

    template <typename T>
    T read() const noexcept {
        return value.get_internal_storage<T>();
    }

    template <typename T>
    void write(T value) noexcept {
        this->value = storage_type_erased(storage_type<T>(value));
    }

private:
    storage_type_erased value{};
};

#endif  // EIDGENOSSE_LOW_LEVEL_ACCESS_HPP