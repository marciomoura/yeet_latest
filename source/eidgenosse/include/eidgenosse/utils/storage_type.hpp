/**
 * @copyright Techperts AG.
 * @date 10-12-2024
 * @brief
 */

#ifndef EIDGENOSSE_UTILS_STORAGE_TYPE_HPP
#define EIDGENOSSE_UTILS_STORAGE_TYPE_HPP

#include <cassert>
#include <memory>
#include <type_traits>

class storage_type_erased;

template <typename T>
class storage_type {
public:
    static_assert(!std::is_void_v<T>, "T must not be void");

    storage_type() : _value(std::make_shared<T>()) {}
    storage_type(std::shared_ptr<T> value) : _value(value) {}
    ~storage_type() = default;

    operator T() const {
        assert(_value != nullptr);
        return *_value;
    }

    operator T&() {
        assert(_value != nullptr);
        return *_value;
    }

    T& operator=(const T& value) {
        assert(_value != nullptr);
        return *_value = value;
    }

    T& operator=(T&& value) {
        assert(_value != nullptr);
        return *_value = std::move(value);
    }

    void connect(storage_type<T>& storage) {
        assert(_value != nullptr);
        _value = storage.get_internal_storage();
    }

    template <typename U>
    void connect(storage_type<U>& storage) {
        assert(_value != nullptr);
        _value = storage.get_internal_storage();
    }

    bool empty() const { return _value == nullptr; }

    std::shared_ptr<T> get_internal_storage() const { return _value; }

private:
    std::shared_ptr<T> _value;
};

class storage_type_erased {
public:
    storage_type_erased() = default;

    template <typename T>
    storage_type_erased(storage_type<T> storage) {
        static_assert(!std::is_void_v<T>, "T must not be void");
        std::shared_ptr<void> _value_casted_as_voi = std::static_pointer_cast<void>(storage.get_internal_storage());
        _value = _value_casted_as_voi;
    }

    bool empty() const { return _value.expired(); }

    template <typename T>
    storage_type<T> get_internal_storage() const {
        if (empty()) {
            return storage_type<T>();
        }

        return storage_type<T>(std::static_pointer_cast<T>(_value.lock()));
    }

private:
    std::weak_ptr<void> _value{};
};

#endif  // EIDGENOSSE_UTILS_STORAGE_TYPE_HPP