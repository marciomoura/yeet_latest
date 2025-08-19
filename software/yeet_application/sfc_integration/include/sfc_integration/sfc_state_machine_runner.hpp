#pragma once

#include "sfc/sfc_state_machine.hpp"
#include "sfc_integration/sfc_state_machine_datamodel.hpp"
#include "yeet/application_interface.hpp"
#include "yeet_control/constants.hpp"

namespace yeet::application::sfc::integration {

class sfc_state_machine_runner : public yeet::base_application<sfc_state_machine_datamodel::inputs,
                                     sfc_state_machine_datamodel::outputs,
                                     sfc_state_machine_datamodel::parameters> {
public:
    using datamodel = sfc_state_machine_datamodel;

    static constexpr duration_t sampling_time = ms(10);

    sfc_state_machine_runner()
        : base_application(sampling_time),
          _state_machine_inputs{
              sfc::sfc_state_event("auxiliaries_are_on", [this]() { return _inputs.auxiliaries_are_on.read(); }),
              sfc::sfc_state_event("breakers_are_on", [this]() { return _inputs.breakers_are_on.read(); }),
              sfc::sfc_state_event("start_command", [this]() { return _inputs.start_command.read(); }),
              sfc::sfc_state_event("stop_command", [this]() { return _inputs.stop_command.read(); }),
              sfc::sfc_state_event("shutdown_request", [this]() { return _inputs.shutdown_request.read(); }),
              sfc::sfc_state_event(
                  "is_current_extinguished", [this]() { return _inputs.is_current_extinguished.read(); })}
    {
    }

    void initialize() override;
    void run() override;
    void stop() override;

private:
    void set_parameters();
    void set_parameter_callback();

    sfc::sfc_state_machine _sfc_state_machine;
    sfc::sfc_state_inputs _state_machine_inputs;
};

}  // namespace yeet::application::sfc::integration