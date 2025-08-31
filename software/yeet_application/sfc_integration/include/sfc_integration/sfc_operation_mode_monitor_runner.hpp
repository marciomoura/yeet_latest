#pragma once

#include "sfc/sfc_discontinuous_conduction_detector.hpp"
#include "sfc/sfc_mode_detector.hpp"
#include "sfc_integration/sfc_operation_mode_monitor_datamodel.hpp"
#include "yeet/application_interface.hpp"
#include "yeet_control/constants.hpp"

namespace yeet::application::sfc::integration {

/**
 * @class sfc_operation_mode_monitor_runner
 * @brief Executes and integrates the SFC mode and discontinuous conduction detectors.
 *
 * This class serves as the runtime component that connects the control algorithms
 * for mode detection and discontinuous conduction monitoring with the application's
 * data model (signals and parameters).
 */
class sfc_operation_mode_monitor_runner : public yeet::base_application<sfc_operation_mode_monitor_datamodel::inputs,
                                              sfc_operation_mode_monitor_datamodel::outputs,
                                              sfc_operation_mode_monitor_datamodel::parameters> {
public:
    using datamodel = sfc_operation_mode_monitor_datamodel;

    /// @brief The execution interval for the monitoring logic.
    static constexpr duration_t sampling_time = us(200);

    /**
     * @brief Constructs the runner and its underlying detector components.
     */
    sfc_operation_mode_monitor_runner();

    /**
     * @brief Initializes the detectors by applying parameters and resetting states.
     */
    void initialize() override;

    /**
     * @brief Executes one time step of the monitoring logic.
     */
    void run_slow();

    /**
     * @brief Executes one time step of the monitoring logic.
     */
    void run() override;

    /**
     * @brief Resets the detectors to a safe, initial state.
     */
    void stop() override;

private:
    /**
     * @brief Applies all parameters from the datamodel to the components.
     */
    void set_parameters();

    /**
     * @brief Registers callbacks for online parameter changes.
     */
    void set_parameter_callback();

    sfc::sfc_mode_detector _mode_detector;
    sfc::sfc_discontinuous_conduction_detector _discontinuous_conduction_detector;
};

}  // namespace yeet::application::sfc::integration