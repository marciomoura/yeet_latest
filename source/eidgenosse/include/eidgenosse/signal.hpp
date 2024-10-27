/**
 * @copyright Techperts AG.
 * @date 10-12-2024
 * @brief
 */

#ifndef EIDGENOSSE_SIGNAL_HPP
#define EIDGENOSSE_SIGNAL_HPP

#include <cassert>
#include <memory>
#include <type_traits>

#include "utils/low_level_access.hpp"
#include "utils/storage_type.hpp"

class signals_connector;

template <typename T>
class signal_output {
public:
    signal_output() = default;
    ~signal_output() = default;

    low_level_access get_low_level_access() { return low_level_access(_value); }

    void set_metadata() {}

    void get_metadata() {}

    T read() const noexcept { return _value; }

    void write(T value) noexcept { _value = value; }

    T operator=(T value) noexcept { return _value = value; }

    operator T() const noexcept { return read(); }

protected:
    storage_type<T>& get_storage() { return _value; }

private:
    friend class signals_connector;

    storage_type<T> _value{std::make_shared<T>()};
};

template <typename T>
class signal_input {
public:
    signal_input() = default;
    ~signal_input() = default;

    void set_metadata() {}

    void get_metadata() {}

    void connect(signal_output<T>& storage) { storage.connect(_value); }

    T read() const noexcept { return _value; }

    operator T() const noexcept { return read(); }

    void connect(storage_type<T>& storage) { _value.connect(storage); }

    bool empty() const { return _value.empty(); }

private:
    storage_type<const T> _value{};
};

class signals_connector {
public:
    template <typename T>
    static void connect_output_to_input(signal_output<T>& output, signal_input<T>& input) {
        input.connect(output.get_storage());
    }
};

#endif  // EIDGENOSSE_SIGNAL_HPP