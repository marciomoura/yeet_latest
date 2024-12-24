#ifndef EIDGENOSSE_APPLICATION_INTERFACE_HPP
#define EIDGENOSSE_APPLICATION_INTERFACE_HPP

#include "parameter.hpp"
#include "signal.hpp"

class application_interface {
public:
    application_interface() = default;
    application_interface(double sampling_time) : _sampling_time(sampling_time) {}
    virtual ~application_interface() = default;

    // Virtual method to initialize the application
    virtual void initialize() = 0;

    // Virtual method to run the application
    virtual void run() = 0;

    // Virtual method to stop the application
    virtual void stop() = 0;

    double get_sampling_time() const { return _sampling_time; }

    void set_sampling_time(double sampling_time) { _sampling_time = sampling_time; }

private:
    double _sampling_time = {-1.0};
};

class signal_registry_interface {
public:
    virtual ~signal_registry_interface() = default;

    // Interface to return the signal registry
    virtual signal_registry get_signal_registry() = 0;
};

class parameter_registry_interface {
public:
    virtual ~parameter_registry_interface() = default;

    // Interface to return the parameter registry
    virtual parameter_registry get_parameter_registry() = 0;
};

#endif  // EIDGENOSSE_APPLICATION_INTERFACE_HPP