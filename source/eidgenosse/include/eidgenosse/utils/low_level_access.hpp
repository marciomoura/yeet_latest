/**
 * @copyright Techperts AG.
 * @date 10-12-2024
 * @brief
 */

#ifndef EIDGENOSSE_UTILS_LOW_LEVEL_ACCESS_HPP
#define EIDGENOSSE_UTILS_LOW_LEVEL_ACCESS_HPP

#include <cassert>

#include "storage_type.hpp"

class low_level_access {
public:
    low_level_access() = default;

    template <typename T>
    low_level_access(storage_type<T>& storage) : _storage(storage) {}

    low_level_access(storage_type_erased& storage) : _storage(storage) {}
    ~low_level_access() = default;

    bool empty() const { return _storage.empty(); }

    template <typename T>
    T read() const {
        const storage_type<T> typed_storage = _storage.get_storage_with_type<T>();
        if (typed_storage.empty()) {
            return {};
        }

        const std::shared_ptr<T> internal_storage = typed_storage.get_internal_storage();
        if (!internal_storage) {
            return {};
        }

        return *internal_storage;
    }
    template <typename T>
    void write(T value) {
        storage_type<T> typed_storage = _storage.get_storage_with_type<T>();
        if (typed_storage.empty()) {
            return;
        }

        std::shared_ptr<T> internal_storage = typed_storage.get_internal_storage();
        if (!internal_storage) {
            return;
        }

        *internal_storage = value;
    }

private:
    storage_type_erased _storage{};
};

#endif  // EIDGENOSSE_LOW_LEVEL_ACCESS_HPP