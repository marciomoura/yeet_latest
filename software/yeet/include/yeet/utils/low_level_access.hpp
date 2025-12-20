/**
 * @copyright Techperts AG.
 * @date 10-12-2024
 * @brief
 */

#ifndef YEET_UTILS_LOW_LEVEL_ACCESS_HPP
#define YEET_UTILS_LOW_LEVEL_ACCESS_HPP

#include <cassert>
#include <functional>

#include "storage_type.hpp"

class low_level_access {
public:
    struct sampler_accessor {
        std::function<float()> read{};
    };

    low_level_access() = default;

    template <typename T>
    low_level_access(storage_type<T>& storage) : _storage(storage)
    {
        const storage_type<T> typed_storage = _storage.get_storage_with_type<T>();

        T* internal_storage = typed_storage.get_internal_storage();

        _read_as_float_function = [internal_storage]() { return static_cast<float>(*internal_storage); };
    }

    // Constructor with explicit read/write functions
    // Useful for custom types that need conversion (angle_wrapped, duration_t, unit types)
    low_level_access(std::function<float()> read_func, std::function<void(float)> write_func)
        : _read_as_float_function(read_func), _write_as_float_function(write_func), _storage()
    {
    }

    low_level_access(storage_type_erased& storage) : _storage(storage) {}
    ~low_level_access() = default;

    bool empty() const { return _storage.empty(); }

    template <typename T>
    T read() const
    {
        // If custom read function exists and T is float, use it
        if constexpr (std::is_same_v<T, float>) {
            if (_read_as_float_function) {
                return _read_as_float_function();
            }
        }

        const storage_type<T> typed_storage = _storage.get_storage_with_type<T>();
        if (typed_storage.empty()) {
            return {};
        }

        T* internal_storage = typed_storage.get_internal_storage();
        if (!internal_storage) {
            return {};
        }

        return *internal_storage;
    }

    sampler_accessor get_sampler_accessor() const { return sampler_accessor{_read_as_float_function}; }

    template <typename T>
    void write(T value)
    {
        // If custom write function exists and T is float, use it
        if constexpr (std::is_same_v<T, float>) {
            if (_write_as_float_function) {
                _write_as_float_function(value);
                return;
            }
        }

        storage_type<T> typed_storage = _storage.get_storage_with_type<T>();
        if (typed_storage.empty()) {
            return;
        }

        T* internal_storage = typed_storage.get_internal_storage();
        if (!internal_storage) {
            return;
        }

        *internal_storage = value;
    }

private:
    std::function<float()> _read_as_float_function{};
    std::function<void(float)> _write_as_float_function{};

    storage_type_erased _storage{};
};

#endif  // YEET_LOW_LEVEL_ACCESS_HPP