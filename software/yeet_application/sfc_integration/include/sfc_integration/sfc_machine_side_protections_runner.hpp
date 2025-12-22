#pragma once

#include "sfc/sfc_machine_side_protections.hpp"
#include "sfc_integration/sfc_machine_side_protections_datamodel.hpp"
#include "yeet/application_interface.hpp"
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
     * @param remote_communication_ok True if remote side reports it is receiving us
     *
     * Example usage in machine_side_control:
     * @code
     * bool rx_ok = decode_fiber_optic_rx_from_grid(payload, length);
     * bool remote_ok = msg.communication_ok;
     * machine_protections.update_communication_status(rx_ok, remote_ok);
     * @endcode
     */
    void update_communication_status(bool communication_valid, bool remote_communication_ok = true);

private:
    // Common machine-side protections application
    sfc::sfc_machine_side_protections _protections;

    // Communication status tracking
    bool _last_communication_valid{true};  // Track decode success from previous cycle
    bool _last_remote_communication_ok{true};

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
