#pragma once

#include "sfc/sfc_communication_watchdog.hpp"
#include "sfc/sfc_current_protection.hpp"
#include "sfc/sfc_overvoltage_protection.hpp"
#include "sfc/sfc_undervoltage_protection.hpp"
#include "sfc_integration/sfc_grid_side_protections_datamodel.hpp"
#include "yeet/application_interface.hpp"
#include "yeet_control/edge_detector.hpp"
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

private:
    // Protection components
    sfc::sfc_current_protection _overcurrent_protection;
    sfc::sfc_overvoltage_protection _overvoltage_protection;
    sfc::sfc_undervoltage_protection _undervoltage_protection;
    sfc::sfc_communication_watchdog _communication_watchdog;

    // Trip handshake state
    bool _trip_latched{false};  // Latched trip signal (cleared when fault clears AND acknowledged)

    // Alarm confirmation state
    bool _alarm_pending{false};                         // Latching alarm state (is_inhibited)
    yeet::control::edge_detector _confirm_alarms_edge;  // Edge detector for confirm_alarms parameter

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
