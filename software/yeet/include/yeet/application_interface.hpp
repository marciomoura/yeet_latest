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

    application_datamodel<InputType, OutputType, ParameterType>& get_datamodel() { return _data_model; }

    application_datamodel<InputType, OutputType, ParameterType> const& get_datamodel() const { return _data_model; }

protected:
    InputType _inputs;
    OutputType _outputs;
    ParameterType _parameters;
    application_datamodel<InputType, OutputType, ParameterType> _data_model{_inputs, _outputs, _parameters};
};

}  // namespace yeet

#endif  // YEET_APPLICATION_INTERFACE_HPP