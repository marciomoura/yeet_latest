/**
 * @copyright Techperts AG.
 * @date 10-12-2024
 * @brief
 */

#ifndef EIDGENOSSE_PARAMETER_HPP
#define EIDGENOSSE_PARAMETER_HPP

#include <cassert>
#include <memory>

#include "utils/low_level_access.hpp"
#include "utils/low_level_access_registry.hpp"
#include "utils/storage_type.hpp"

template <typename T>
class parameter {
public:
    parameter() = default;
    ~parameter() = default;

    low_level_access get_low_level_access() { return low_level_access(_value); }

    void set_metadata() {}

    void get_metadata() {}

    T read() const noexcept { return _value; }

    operator T() const noexcept { return read(); }

    T operator=(T value) noexcept { return _value = value; }

    void write(T value) noexcept { _value = value; }

private:
    storage_type<T> _value{std::make_shared<T>()};
};

class parameter_registry {
public:
    parameter_registry() = default;
    ~parameter_registry() = default;

    template <typename T>
    void insert(const std::string& name, parameter<T>& parameter) {
        _registry.insert(name, parameter.get_low_level_access());
    }

    bool empty() { return _registry.empty(); }

    void clear() { _registry.clear(); }

    std::optional<low_level_access> get(const std::string& name) const { return _registry.get(name); }

private:
    low_level_access_registry _registry{};
};

#endif  // EIDGENOSSE_PARAMETER_HPP