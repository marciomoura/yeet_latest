#ifndef YEET_SIGNAL_WIRING_HPP
#define YEET_SIGNAL_WIRING_HPP

#include <type_traits>

#include "signal.hpp"

namespace yeet {

template <typename InputsStruct>
class input_wiring_builder;

template <typename InputsStruct, typename T>
class pending_connection {
public:
    pending_connection(InputsStruct& inputs, signal_output<T>& output) : _inputs(inputs), _output(output) {}

    input_wiring_builder<InputsStruct> to(signal_input<T> InputsStruct::* input_ptr)
    {
        signals_connector::connect_output_to_input(_output, _inputs.*input_ptr);
        return input_wiring_builder<InputsStruct>(_inputs);
    }

private:
    InputsStruct& _inputs;
    signal_output<T>& _output;
};

template <typename InputsStruct, typename T>
class pending_mock {
public:
    pending_mock(InputsStruct& inputs, T& storage) : _inputs(inputs), _storage(storage) {}

    input_wiring_builder<InputsStruct> to(signal_input<T> InputsStruct::* input_ptr)
    {
        utils::mock_signal_input::connect(_inputs.*input_ptr, _storage);
        return input_wiring_builder<InputsStruct>(_inputs);
    }

private:
    InputsStruct& _inputs;
    T& _storage;
};

template <typename InputsStruct>
class input_wiring_builder {
public:
    explicit input_wiring_builder(InputsStruct& inputs) : _inputs(inputs) {}

    template <typename T>
    pending_connection<InputsStruct, T> from(signal_output<T>& output)
    {
        return pending_connection<InputsStruct, T>(_inputs, output);
    }

    template <typename T>
    pending_mock<InputsStruct, T> mocked(T& storage)
    {
        return pending_mock<InputsStruct, T>(_inputs, storage);
    }

private:
    InputsStruct& _inputs;
};

template <typename T, typename = void>
struct has_get_inputs : std::false_type {};

template <typename T>
struct has_get_inputs<T, std::void_t<decltype(std::declval<T>().get_inputs())>> : std::true_type {};

template <typename InputsStruct>
input_wiring_builder<InputsStruct> wire_inputs(InputsStruct& inputs)
{
    return input_wiring_builder<InputsStruct>(inputs);
}

template <typename T>
auto wire(T& obj)
{
    if constexpr (has_get_inputs<T>::value) {
        return input_wiring_builder<std::remove_reference_t<decltype(obj.get_inputs())>>(obj.get_inputs());
    }
    else {
        return input_wiring_builder<T>(obj);
    }
}

}  // namespace yeet

#endif  // YEET_SIGNAL_WIRING_HPP
