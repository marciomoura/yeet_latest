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

    storage_type() : value_(std::make_shared<T>()) {}
    storage_type(std::shared_ptr<T> value) : value_(value) {}
    ~storage_type() = default;

    operator T() const noexcept {
        assert(value_ != nullptr);
        return *value_;
    }

    operator T&() noexcept {
        assert(value_ != nullptr);
        return *value_;
    }

    T& operator=(const T& value) noexcept {
        assert(value_ != nullptr);
        return *value_ = value;
    }

    T& operator=(T&& value) noexcept {
        assert(value_ != nullptr);
        return *value_ = std::move(value);
    }

    bool empty() const noexcept { return value_ == nullptr; }

    std::shared_ptr<T> get_internal_storage() const noexcept { return value_; }

private:
    std::shared_ptr<T> value_;
};

class storage_type_erased {
public:
    storage_type_erased() = default;

    template <typename T>
    storage_type_erased(storage_type<T> storage) {
        static_assert(!std::is_void_v<T>, "T must not be void");
        std::shared_ptr<void> value_casted_as_void = std::static_pointer_cast<void>(storage.get_internal_storage());
        value_ = value_casted_as_void;
    }

    bool empty() const noexcept { return value_.expired(); }

    template <typename T>
    storage_type<T> get_internal_storage() const noexcept {
        if (empty()) {
            return storage_type<T>();
        }

        return storage_type<T>(std::static_pointer_cast<T>(value_.lock()));
    }

private:
    std::weak_ptr<void> value_{};
};

#endif  // EIDGENOSSE_UTILS_STORAGE_TYPE_HPP