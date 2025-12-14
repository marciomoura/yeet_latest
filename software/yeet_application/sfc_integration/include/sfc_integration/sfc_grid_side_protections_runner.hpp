#pragma once

#include "sfc/sfc_grid_side_protections.hpp"
#include "sfc_integration/sfc_grid_side_protections_datamodel.hpp"
#include "yeet/application_interface.hpp"
#include "yeet_control/units.hpp"

namespace yeet::application::sfc::integration {

/**
 * @brief Runner for Grid-Side Protection System
 *
 * Integrates overcurrent, overvoltage, and undervoltage protections
 * into a unified protection system for the grid-side converter.
 *
 * Protection behavior:
 * - All protections use latching trip logic
 * - Combined trip output (OR of all individual trips)
 * - Configurable thresholds and time delays
 * - Manual reset capability via force_reset_all parameter
 */
class sfc_grid_side_protections_runner : public yeet::base_application<sfc_grid_side_protections_datamodel::inputs,
                                             sfc_grid_side_protections_datamodel::outputs,
                                             sfc_grid_side_protections_datamodel::parameters> {
public:
    using datamodel = sfc_grid_side_protections_datamodel;

    static constexpr duration_t sampling_time = duration_t{200e-6};  // 200µs

    /**
     * @brief Construct protection runner with default sampling period (200µs)
     */
    sfc_grid_side_protections_runner();

    void register_datamodel(yeet::model_registry& registry) override;
    void initialize() override;
    void reset();
    void run() override;
    void stop() override;

    /**
     * @brief Update communication status for watchdog monitoring
     *
     * Call this method immediately after attempting to decode fiber-optic RX data.
     * The watchdog will track consecutive communication failures and trip if threshold exceeded.
     */
    void update_communication_status(bool communication_valid);

private:
    // Common grid-side protections application
    sfc::sfc_grid_side_protections _protections;

    // Communication status tracking (set via update_communication_status)
    bool _last_communication_valid{true};

    /**
     * @brief Apply parameter changes to protection components
     */
    void set_parameters();

    /**
     * @brief Register parameter change callbacks
     */
    void set_parameter_callback();
};

}  // namespace yeet::application::sfc::integration
