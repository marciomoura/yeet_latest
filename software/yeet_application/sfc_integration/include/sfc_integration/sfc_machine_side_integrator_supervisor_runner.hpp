#pragma once

#include "sfc_integration/sfc_machine_side_integrator_supervisor_datamodel.hpp"
#include "yeet/application_interface.hpp"
#include "yeet_control/constants.hpp"

namespace yeet::application::sfc::integration {

/**
 * @class sfc_machine_side_integrator_supervisor_runner
 * @brief Application runner for machine-side integrator supervisor component
 *
 * This runner bridges the machine-side integrator supervisor core component with
 * the datamodel interface. It coordinates freeze/preset/trip logic for both
 * speed and torque controllers by managing requests from grid-side, emergency stop,
 * and torque saturation signals.
 */
class sfc_machine_side_integrator_supervisor_runner
    : public yeet::base_application<sfc_machine_side_integrator_supervisor_datamodel::inputs,
          sfc_machine_side_integrator_supervisor_datamodel::outputs,
          sfc_machine_side_integrator_supervisor_datamodel::parameters> {
public:
    using datamodel = sfc_machine_side_integrator_supervisor_datamodel;

    /// Sampling time for the supervisor (typically matches control loop)
    static constexpr yeet::duration_t sampling_time = ms(1.0);

    /**
     * @brief Constructor
     */
    sfc_machine_side_integrator_supervisor_runner() : base_application(sampling_time) {}

    /// Initialize the supervisor component
    void initialize() override;

    /// Execute one cycle of supervisor logic
    void run() override;

    /// Stop the supervisor
    void stop() override;

    /// Register datamodel for network access
    void register_datamodel(yeet::telemetry::parameter_handler& handler) override;

private:
    bool _previous_release_speed_controller{false};
};

}  // namespace yeet::application::sfc::integration
