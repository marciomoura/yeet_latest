#ifndef YEET_PARAMETER_HPP
#define YEET_PARAMETER_HPP

#include <cassert>
#include <functional>
#include <memory>

#include "utils/low_level_access.hpp"
#include "utils/low_level_access_registry.hpp"
#include "utils/storage_type.hpp"

namespace yeet {

template <typename T>
class parameter {
public:
    parameter() = default;
    parameter(T value) : _value(std::make_shared<T>(value)) {}
    ~parameter() = default;

    low_level_access get_low_level_access() { return low_level_access(_value); }

    void set_metadata() {}

    void get_metadata() {}

    T read() const noexcept { return _value; }

    operator T() const noexcept { return read(); }

    T operator=(T value) noexcept
    {
        write(value);
        return _value;
    }

    void write(T value) noexcept
    {
        _value = value;

        if (_callback) {
            _callback();
        }
    }

    void register_callback(std::function<void(void)> callback, bool run_callback_immediately = true)
    {
        _callback = callback;

        // Run the callback immediately upon registering callback
        if (run_callback_immediately) {
            _callback();
        }
    }

private:
    std::function<void(void)> _callback{};

    storage_type<T> _value{std::make_shared<T>()};
};

class parameter_registry {
public:
    parameter_registry() = default;
    ~parameter_registry() = default;

    template <typename T>
    void insert(const std::string& name, parameter<T>& parameter)
    {
        _registry.insert(name, parameter.get_low_level_access());
    }

    // Direct insertion of low_level_access with custom read/write functions
    void insert(const std::string& name, low_level_access accessor) { _registry.insert(name, accessor); }

    bool empty() { return _registry.empty(); }

    void clear() { _registry.clear(); }

    std::optional<low_level_access> get(const std::string& name) const { return _registry.get(name); }

    auto begin() { return _registry.begin(); }
    auto end() { return _registry.end(); }

    auto cbegin() const { return _registry.cbegin(); }
    auto cend() const { return _registry.cend(); }

private:
    low_level_access_registry _registry{};
};

class parameter_registry_interface {
public:
    virtual ~parameter_registry_interface() = default;

    // Interface to return the parameter registry
    virtual parameter_registry get_parameter_registry() = 0;
};

}  // namespace yeet

#endif  // YEET_PARAMETER_HPP