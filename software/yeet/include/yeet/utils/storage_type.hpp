#ifndef YEET_UTILS_STORAGE_TYPE_HPP
#define YEET_UTILS_STORAGE_TYPE_HPP

#include <cassert>
#include <memory>
#include <type_traits>

#ifndef YEET_BUILD_EMBEDDED
#include <iostream>
#include <typeinfo>

#include "stacktrace.hpp"
#endif

class storage_type_erased;

template <typename T>
class storage_type {
public:
    static_assert(!std::is_void_v<T>, "T must not be void");

    storage_type() : _value{nullptr} {}
    storage_type(T* value) : _value(value) {}
    ~storage_type() = default;

    operator T() const
    {
#ifndef YEET_BUILD_EMBEDDED
        if (_value == nullptr) {
            std::cerr << "\n=== STORAGE_TYPE ERROR ===\n";
            std::cerr << "Attempted to read from empty storage_type<" << typeid(T).name() << ">\n";
            std::cerr << "Call stack:\n";
            yeet::utils::print_stacktrace();
            std::cerr << "==========================\n" << std::endl;
            assert(false && "Attempted to read from empty storage_type");
        }
#endif
        return *_value;
    }

    operator T&()
    {
        assert(_value != nullptr);
        return *_value;
    }

    T& operator=(const T& value)
    {
        assert(_value != nullptr);
        return *_value = value;
    }

    void connect(storage_type<T> storage)
    {
        _value = storage.get_internal_storage();
        assert(_value != nullptr);
    }

    template <typename U>
    void connect(storage_type<U> storage)
    {
        _value = storage.template get_internal_storage<U>();
        assert(_value != nullptr);
    }

    bool empty() const { return _value == nullptr; }

    // Type safety checks in the get_internal_storage method to ensure that the type being retrieved is compatible with
    // the stored type.
    template <typename U = T>
    U* get_internal_storage() const
    {
        if constexpr (std::is_same_v<T, U>) {
            return _value;
        }
        else {
            return static_cast<U*>(static_cast<void*>(_value));
        }
    }

private:
    T* _value;
};

class storage_type_erased {
public:
    storage_type_erased() = default;

    template <typename T>
    storage_type_erased(storage_type<T> storage)
    {
        static_assert(!std::is_void_v<T>, "T must not be void");

        assert(!storage.empty() && "Storage must not be empty");

        _value = static_cast<void*>(storage.template get_internal_storage<T>());

        assert(_value != nullptr && "Failed to cast storage to void");
    }

    bool empty() const { return _value == nullptr; }

    template <typename T>
    storage_type<T> get_storage_with_type() const
    {
        if (empty()) {
            return storage_type<T>();
        }

        return storage_type<T>(static_cast<T*>(_value));
    }

private:
    void* _value{nullptr};
};

#endif  // YEET_UTILS_STORAGE_TYPE_HPP