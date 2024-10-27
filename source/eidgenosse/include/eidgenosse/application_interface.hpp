#ifndef EIDGENOSSE_APPLICATION_INTERFACE_HPP
#define EIDGENOSSE_APPLICATION_INTERFACE_HPP

#include "parameter.hpp"
#include "signal.hpp"

class application_interface {
public:
    virtual ~application_interface() = default;

    // Virtual method to run the application
    virtual void run() = 0;

    // Interface to return the signal registry
    virtual signal_registry&& get_signal_registry() = 0;

    // Interface to return the parameter registry
    virtual parameter_registry&& get_parameter_registry() = 0;
};

#endif  // EIDGENOSSE_APPLICATION_INTERFACE_HPP