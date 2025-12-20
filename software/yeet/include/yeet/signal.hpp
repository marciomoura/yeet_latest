#ifndef YEET_SIGNAL_HPP
#define YEET_SIGNAL_HPP

#include <cassert>
#include <memory>

#include "utils/low_level_access.hpp"
#include "utils/low_level_access_registry.hpp"
#include "utils/storage_type.hpp"

namespace yeet {

class signals_connector;

template <typename T>
class signal_output {
public:
    signal_output() = default;
    signal_output(T value) : _data(value) {}
    ~signal_output() = default;

    low_level_access get_low_level_access() { return low_level_access(_value); }

    void set_metadata() {}

    void get_metadata() {}

    T read() const noexcept { return _value; }

    void write(T value) noexcept { _value = value; }

    T operator=(T value) noexcept { return _value = value; }

    operator T() const noexcept { return read(); }

protected:
    storage_type<T> get_storage() { return _value; }

private:
    friend class signals_connector;

    T _data{};
    storage_type<T> _value{&_data};
};

class mock_signal_input;

template <typename T>
class signal_input {
public:
    void set_metadata() {}

    void get_metadata() {}

    void connect(signal_output<T> storage) { storage.connect(_value); }

    T read() const noexcept { return _value; }

    operator T() const noexcept { return read(); }

    void connect(storage_type<T> storage) { _value.connect(storage); }

    bool empty() const { return _value.empty(); }

private:
    friend class mock_signal_input;

    // Mocking for easy simulation access
    void write(T const& value) noexcept { _value = value; }

    storage_type<T> _value{};
};

class mock_signal_input {
public:
    mock_signal_input() = default;

    template <typename T>
    static void write(signal_input<T>& input, T const& value)
    {
        assert(!input.empty() && "Mock storage must be connected before writing");
        input.write(value);
    }

    template <typename T>
    static void connect(signal_input<T>& input, T& storage)
    {
        input.connect(storage_type<T>(&storage));
    }
};

class signals_connector {
public:
    template <typename T>
    static void connect_output_to_input(signal_output<T>& output, signal_input<T>& input)
    {
        input.connect(output.get_storage());
    }
};

class signal_registry {
public:
    template <typename T>
    void insert(const std::string& name, signal_output<T>& signal)
    {
        _registry.insert(name, signal.get_low_level_access());
    }

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

class signal_registry_interface {
public:
    virtual ~signal_registry_interface() = default;

    // Interface to return the signal registry
    virtual signal_registry get_signal_registry() = 0;
};

template <typename T>
using external_signal = signal_output<T>;

}  // namespace yeet

#endif  // YEET_SIGNAL_HPP