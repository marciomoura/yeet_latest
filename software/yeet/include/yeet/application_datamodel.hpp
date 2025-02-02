#ifndef YEET_DATA_MODEL_INTERFACE_HPP
#define YEET_DATA_MODEL_INTERFACE_HPP

namespace yeet {

template <typename InputType, typename OutputType>
class signal_flow_datamodel {
public:
    using inputs_type = InputType;
    using outputs_type = OutputType;

    signal_flow_datamodel(inputs_type& model_inputs, outputs_type& model_outputs)
        : _model_inputs(&model_inputs), _model_outputs(&model_outputs) {
        assert(_model_inputs != nullptr);
        assert(_model_outputs != nullptr);
    }

    inputs_type& get_inputs() { return *_model_inputs; }

    outputs_type& get_outputs() { return *_model_outputs; }

private:
    inputs_type* _model_inputs{};
    outputs_type* _model_outputs{};
};

// Generic interface for all data models
template <typename ParameterType>
class application_configuration_datamodel {
public:
    using parameters_type = ParameterType;

    application_configuration_datamodel(parameters_type& model_parameters) : _model_parameters(&model_parameters) {
        assert(_model_parameters != nullptr);
    }

    parameters_type& get_parameters() { return *_model_parameters; }

private:
    parameters_type* _model_parameters{};
};

}  // namespace yeet

#endif  // YEET_DATA_MODEL_INTERFACE_HPP
