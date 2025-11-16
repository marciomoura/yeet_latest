#pragma once

#include "sfc/sfc_communication_watchdog.hpp"
#include "sfc/sfc_stall_protection.hpp"
#include "sfc/sfc_vhz_protection.hpp"
#include "sfc_integration/sfc_machine_side_protections_datamodel.hpp"
#include "yeet/application_interface.hpp"
#include "yeet_control/edge_detector.hpp"
#include "yeet_control/units.hpp"

namespace yeet::application::sfc::integration {

/**
 * @brief Runner for Machine-Side Protection System
 *
 * Integrates V/Hz (flux) and stall protections into a unified protection system
 * for the machine-side converter.
 *
 * Protection behavior:
 * - All protections use latching trip logic
 * - Combined trip output (OR of all individual trips)
 * - Configurable thresholds and time delays
 * - Manual reset capability via force_reset_all parameter
 * - Trip status transmitted to grid-side via fiber-optic link
 */
class sfc_machine_side_protections_runner
    : public yeet::base_application<sfc_machine_side_protections_datamodel::inputs,
          sfc_machine_side_protections_datamodel::outputs,
          sfc_machine_side_protections_datamodel::parameters> {
public:
    using datamodel = sfc_machine_side_protections_datamodel;

    static constexpr duration_t sampling_time = duration_t{200e-6};  // 200µs

    /**
     * @brief Construct protection runner with default sampling period (200µs)
     */
    sfc_machine_side_protections_runner();

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
     *
     * @param communication_valid True if fiber-optic decode succeeded, false if failed
     *
     * Example usage in machine_side_control:
     * @code
     * bool rx_ok = decode_fiber_optic_rx_from_grid(payload, length);
     * machine_protections.update_communication_status(rx_ok);
     * @endcode
     */
    void update_communication_status(bool communication_valid);

private:
    // Protection components
    sfc::sfc_vhz_protection _vhz_protection;
    sfc::sfc_stall_protection _stall_protection;
    sfc::sfc_communication_watchdog _communication_watchdog;

    // Communication status tracking
    bool _last_communication_valid{true};  // Track decode success from previous cycle

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
