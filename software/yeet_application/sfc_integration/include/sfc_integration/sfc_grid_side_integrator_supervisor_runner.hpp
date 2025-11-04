#pragma once

#include "sfc/sfc_grid_side_integrator_supervisor.hpp"
#include "sfc_integration/sfc_grid_side_integrator_supervisor_datamodel.hpp"
#include "yeet/application_interface.hpp"
#include "yeet_control/constants.hpp"

namespace yeet::application::sfc::integration {

/**
 * @class sfc_grid_side_integrator_supervisor_runner
 * @brief Application runner for grid-side integrator supervisor component
 *
 * This runner bridges the grid-side integrator supervisor core component with
 * the datamodel interface. It coordinates freeze/preset/trip logic for the
 * current controller by managing requests from the modulator and external IOS.
 */
class sfc_grid_side_integrator_supervisor_runner
    : public yeet::base_application<sfc_grid_side_integrator_supervisor_datamodel::inputs,
          sfc_grid_side_integrator_supervisor_datamodel::outputs,
          sfc_grid_side_integrator_supervisor_datamodel::parameters> {
public:
    using datamodel = sfc_grid_side_integrator_supervisor_datamodel;

    /// Sampling time for the supervisor (typically matches control loop)
    static constexpr yeet::duration_t sampling_time = ms(1.0);

    /**
     * @brief Constructor
     */
    sfc_grid_side_integrator_supervisor_runner()
        : base_application(sampling_time), _sfc_grid_side_integrator_supervisor(sampling_time)
    {
    }

    /// Initialize the supervisor component
    void initialize() override;

    /// Execute one cycle of supervisor logic
    void run() override;

    /// Stop the supervisor
    void stop() override;

    /// Register datamodel for network access
    void register_datamodel(yeet::model_registry& registry) override;

private:
    sfc::sfc_grid_side_integrator_supervisor _sfc_grid_side_integrator_supervisor;
};

}  // namespace yeet::application::sfc::integration
