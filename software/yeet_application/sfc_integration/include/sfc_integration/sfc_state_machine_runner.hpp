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
              sfc::sfc_state_event("auxiliaries_are_on", [this]() { return _outputs.auxiliaries_are_on.read(); }),
              sfc::sfc_state_event("mcb_is_closed", [this]() { return _outputs.mcb_is_closed.read(); }),
              sfc::sfc_state_event("start_command", [this]() { return _outputs.start_command.read(); }),
              sfc::sfc_state_event("stop_command", [this]() { return _outputs.stop_command.read(); }),
              sfc::sfc_state_event("shutdown_request",
                  [this]() {
                      // Combine shutdown request with safety interlocks and protection trips.
                      // Any trip/inhibit must force the same shutdown behavior.
                      return _outputs.shutdown_request.read() || _inputs.emergency_off.read() ||
                             _inputs.breaker_is_tripping.read() || _inputs.door_is_open.read() ||
                             _inputs.protection_inhibited.read() || _inputs.machine_side_protection_inhibited.read() ||
                             _inputs.machine_side_shutdown_request.read() ||
                             _inputs.machine_side_protection_tripped.read();
                  }),
              sfc::sfc_state_event(
                  "is_current_extinguished", [this]() { return _outputs.is_current_extinguished.read(); }),
              sfc::sfc_state_event("protection_not_inhibited",
                  [this]() {
                      return !(_inputs.protection_inhibited.read() || _inputs.machine_side_protection_inhibited.read());
                  }),
              sfc::sfc_state_event("any_protection_fault_active",
                  [this]() {
                      // NOTE: machine_side_protection_tripped is a *latched* trip indication,
                      // not a raw condition-active signal. Including it here can deadlock the
                      // shutdown/acknowledgment handshake (the state machine can never reach a
                      // de-energized state to acknowledge, so the machine-side trip can never clear).
                      // Treat machine-side trips as shutdown requests (handled above), and only
                      // use true condition-active sources for this event.
                      return _inputs.any_protection_fault_active.read();
                  }),
              sfc::sfc_state_event("breaker_is_tripping", [this]() { return _inputs.breaker_is_tripping.read(); }),
              sfc::sfc_state_event("emergency_off", [this]() { return _inputs.emergency_off.read(); }),
              sfc::sfc_state_event("door_is_open", [this]() { return _inputs.door_is_open.read(); })}
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
