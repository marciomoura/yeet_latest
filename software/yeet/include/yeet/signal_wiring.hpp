#ifndef YEET_SIGNAL_WIRING_HPP
#define YEET_SIGNAL_WIRING_HPP

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
        mock_signal_input::connect(_inputs.*input_ptr, _storage);
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

template <typename InputsStruct>
input_wiring_builder<InputsStruct> wire_inputs(InputsStruct& inputs)
{
    return input_wiring_builder<InputsStruct>(inputs);
}

template <typename Component>
auto wire(Component& component)
{
    return input_wiring_builder<std::remove_reference_t<decltype(component.get_inputs())>>(component.get_inputs());
}

}  // namespace yeet

#endif  // YEET_SIGNAL_WIRING_HPP
