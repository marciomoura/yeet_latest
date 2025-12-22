#pragma once

#include "sfc/sfc_state_machine.hpp"
#include "sfc_integration/sfc_state_machine_datamodel.hpp"
#include "yeet/application_interface.hpp"
#include "yeet_control/constants.hpp"
#include "yeet_control/edge_detector.hpp"

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
              sfc::sfc_state_event("turn_on_command", [this]() { return _parameters.turn_on_command.read(); }),
              sfc::sfc_state_event("start_command", [this]() { return _outputs.start_command.read(); }),
              sfc::sfc_state_event("stop_command", [this]() { return _outputs.stop_command.read(); }),
              sfc::sfc_state_event("shutdown_request",
                  [this]() {
                      // Combine emergency off, external fault, and latched trips
                      return _inputs.emergency_off.read() || _inputs.external_fault.read() ||
                             _inputs.machine_side_protection_tripped.read();
                  }),
              sfc::sfc_state_event("auxiliaries_are_on", [this]() { return _outputs.auxiliaries_are_on.read(); }),
              sfc::sfc_state_event("mcb_is_closed", [this]() { return _outputs.mv_breaker_is_closed.read(); }),
              sfc::sfc_state_event(
                  "is_current_extinguished", [this]() { return _outputs.is_current_extinguished.read(); }),
              sfc::sfc_state_event("any_fault_active",
                  [this]() {
                      // Immediate faults: emergency off, breaker tripping, active protection faults
                      // NOTE: Excludes latched trips (handled in shutdown_request above)
                      return _inputs.emergency_off.read() || _inputs.mv_breaker_is_tripping.read() ||
                             _inputs.any_protection_fault_active.read() || _inputs.external_fault.read();
                  }),
              sfc::sfc_state_event("is_inhibited_from_starting",
                  [this]() {
                      // Conditions that prevent startup: door open, protection system inhibited
                      return _inputs.door_is_open.read() || _inputs.protection_inhibited.read() ||
                             _inputs.machine_side_protection_inhibited.read();
                  }),
              sfc::sfc_state_event("protection_trip_active", [this]() {
                  // Combined protection trip status for safety interlocks
                  const bool protection_inhibit_active =
                      _inputs.protection_inhibited.read() || _inputs.machine_side_protection_inhibited.read();
                  const bool protection_trip_active = protection_inhibit_active ||
                                                      _inputs.machine_side_protection_tripped.read() ||
                                                      _inputs.external_fault.read();
                  return protection_trip_active;
              })}
    {
    }

    void register_datamodel(yeet::model_registry& registry) override;
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
