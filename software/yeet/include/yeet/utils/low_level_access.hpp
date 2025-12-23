/**
 * @copyright Techperts AG.
 * @date 10-12-2024
 * @brief Lightweight parameter accessor using function pointers (24 bytes vs 64-128 bytes with std::function)
 */

#ifndef YEET_UTILS_LOW_LEVEL_ACCESS_HPP
#define YEET_UTILS_LOW_LEVEL_ACCESS_HPP

#include <cassert>

#include "storage_type.hpp"

class low_level_access {
public:
    // Function pointer types for read/write operations
    using read_fn_t = float (*)(void* context);
    using write_fn_t = void (*)(void* context, float value);

    struct sampler_accessor {
        read_fn_t read_fn{nullptr};
        void* context{nullptr};

        float operator()() const noexcept { return read_fn ? read_fn(context) : 0.0f; }
    };

    constexpr low_level_access() noexcept = default;

    template <typename T>
    low_level_access(storage_type<T>& storage) : _storage(storage)
    {
        const storage_type<T> typed_storage = _storage.get_storage_with_type<T>();
        T* internal_storage = typed_storage.get_internal_storage();

        // Use static function pointers with context
        _read_fn = &read_direct<T>;
        _write_fn = &write_direct<T>;
        _context = internal_storage;
    }

    // Constructor with function pointers and context
    // Useful for custom types that need conversion (angle_wrapped, duration_t, unit types)
    constexpr low_level_access(read_fn_t read_fn, write_fn_t write_fn, void* context) noexcept
        : _read_fn(read_fn), _write_fn(write_fn), _context(context), _storage()
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
            if (_read_fn) {
                return _read_fn(_context);
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

    sampler_accessor get_sampler_accessor() const { return sampler_accessor{_read_fn, _context}; }

    template <typename T>
    void write(T value)
    {
        // If custom write function exists and T is float, use it
        if constexpr (std::is_same_v<T, float>) {
            if (_write_fn) {
                _write_fn(_context, value);
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
    // Static template functions for direct storage access
    template <typename T>
    static float read_direct(void* context) noexcept
    {
        return static_cast<float>(*static_cast<T*>(context));
    }

    template <typename T>
    static void write_direct(void* context, float value) noexcept
    {
        *static_cast<T*>(context) = static_cast<T>(value);
    }

    read_fn_t _read_fn = nullptr;
    write_fn_t _write_fn = nullptr;
    void* _context = nullptr;

    storage_type_erased _storage{};
};

#endif  // YEET_UTILS_LOW_LEVEL_ACCESS_HPP