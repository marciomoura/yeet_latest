/**
 * @copyright Techperts AG.
 * @date 10-12-2024
 * @brief
 */

#ifndef YEET_UTILS_STORAGE_TYPE_HPP
#define YEET_UTILS_STORAGE_TYPE_HPP

#include <cassert>
#include <iostream>
#include <memory>
#include <type_traits>

class storage_type_erased;

template <typename T>
class storage_type {
public:
    static_assert(!std::is_void_v<T>, "T must not be void");

    storage_type() : _value{} {}
    storage_type(std::shared_ptr<T> value) : _value(value) {}
    ~storage_type() = default;

    operator T() const
    {
        if (_value == nullptr) {
            std::cout << "Trap here\n";
            assert(_value != nullptr);
        }
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

    T& operator=(T&& value)
    {
        assert(_value != nullptr);
        return *_value = std::move(value);
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
    // the stored type. This can be done using std::dynamic_pointer_cast for polymorphic types or
    // std::static_pointer_cast for non-polymorphic types.
    template <typename U = T>
    std::shared_ptr<U> get_internal_storage() const
    {
        // static_assert(!std::is_void_v<U>, "U must not be void");
        // static_assert(std::is_copy_constructible_v<U>, "U must be copy constructible");
        // static_assert(std::is_base_of_v<T, U> || std::is_base_of_v<U, T>, "U must be a base or derived class of T");
        // static_assert(std::is_same_v<T, U> || std::is_polymorphic_v<T> || std::is_polymorphic_v<U>,
        //               "T and U must be the same type or polymorphic types");
        if constexpr (std::is_same_v<T, U>) {
            return _value;
        }
        else {
            static_assert(std::is_default_constructible_v<U>, "U must be default constructible");
            return std::static_pointer_cast<U>(_value);
        }
    }

private:
    std::shared_ptr<T> _value;
};

class storage_type_erased {
public:
    storage_type_erased() = default;

    template <typename T>
    storage_type_erased(storage_type<T> storage)
    {
        static_assert(!std::is_void_v<T>, "T must not be void");
        static_assert(std::is_default_constructible_v<T>, "T must be default constructible");
        static_assert(std::is_copy_constructible_v<T>, "T must be copy constructible");

        assert(!storage.empty() && "Storage must not be empty");

        std::shared_ptr<void> _value_casted_as_void =
            std::static_pointer_cast<void>(storage.template get_internal_storage<T>());
        _value = _value_casted_as_void;

        assert(_value != nullptr && "Failed to cast storage to void");
    }

    bool empty() const { return _value == nullptr; }

    template <typename T>
    storage_type<T> get_storage_with_type() const
    {
        if (empty()) {
            return storage_type<T>();
        }

        return storage_type<T>(std::static_pointer_cast<T>(_value));
    }

private:
    std::shared_ptr<void> _value{};
};

#endif  // YEET_UTILS_STORAGE_TYPE_HPP