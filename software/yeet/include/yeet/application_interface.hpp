#ifndef YEET_APPLICATION_INTERFACE_HPP
#define YEET_APPLICATION_INTERFACE_HPP

#include "application_datamodel.hpp"
#include "model_registry.hpp"

namespace yeet {

class application_interface {
public:
    application_interface() = default;
    application_interface(double sampling_time) : _sampling_time(sampling_time) {}
    virtual ~application_interface() = default;

    // Core lifecycle methods
    virtual void initialize() = 0;

    virtual void run() = 0;

    virtual void stop() = 0;

    // Timing management
    double get_sampling_time() const { return _sampling_time; }

    void set_sampling_time(double sampling_time) { _sampling_time = sampling_time; }

private:
    double _sampling_time{-1.0};
};

// Base application combining interfaces
template <typename InputType, typename OutputType, typename ParameterType>
class base_application : public application_interface {
public:
    explicit base_application(double sampling_time) : application_interface(sampling_time) {}

    virtual void register_datamodel(model_registry& registry) {};

    signal_flow_datamodel<InputType, OutputType>& get_signal_flow_datamodel() { return _signal_flow_datamodel; }

    application_configuration_datamodel<ParameterType>& get_configuration_datamodel()
    {
        return _application_configuration_datamodel;
    }

    typename application_configuration_datamodel<ParameterType>::parameters_type& get_parameters()
    {
        return _application_configuration_datamodel.get_parameters();
    }

    typename signal_flow_datamodel<InputType, OutputType>::inputs_type& get_inputs()
    {
        return _signal_flow_datamodel.get_inputs();
    }

    typename signal_flow_datamodel<InputType, OutputType>::outputs_type& get_outputs()
    {
        return _signal_flow_datamodel.get_outputs();
    }

protected:
    InputType _inputs;
    OutputType _outputs;
    ParameterType _parameters;

    signal_flow_datamodel<InputType, OutputType> _signal_flow_datamodel{_inputs, _outputs};

    application_configuration_datamodel<ParameterType> _application_configuration_datamodel{_parameters};
};

}  // namespace yeet

#endif  // YEET_APPLICATION_INTERFACE_HPP