#pragma once

#include "sfc/sfc_startup_sequencer.hpp"
#include "sfc_integration/sfc_startup_sequencer_datamodel.hpp"
#include "yeet/application_interface.hpp"
#include "yeet_control/constants.hpp"

namespace yeet::application::sfc::integration {

/**
 * @brief Runner for SFC Startup Sequencer
 *
 * This runner integrates the startup sequencer into the yeet application framework.
 * It handles:
 * - Parameter configuration from datamodel to sequencer
 * - Command edge detection for start/stop transitions
 * - Real-time execution at 1ms sampling rate
 * - Mapping sequencer outputs to datamodel signals
 * - Override handling for testing and diagnostics
 *
 * The runner coordinates the complete machine-side startup procedure from IDLE state
 * through flux build-up, virtual reference frame generation, and transition to
 * observer-based closed-loop control.
 */
class sfc_startup_sequencer_runner : public yeet::base_application<sfc_startup_sequencer_datamodel::inputs,
                                         sfc_startup_sequencer_datamodel::outputs,
                                         sfc_startup_sequencer_datamodel::parameters> {
public:
    using datamodel = sfc_startup_sequencer_datamodel;

    static constexpr duration_t sampling_time = ms(1.0);  // Machine-side 1ms control loop

    /**
     * @brief Constructs the startup sequencer runner
     */
    sfc_startup_sequencer_runner() : base_application(sampling_time), _sequencer(sampling_time) {}

    void register_datamodel(yeet::model_registry& registry) override;
    void initialize() override;
    void run() override;
    void stop() override;

private:
    /**
     * @brief Configure all sequencer parameters from datamodel
     */
    void set_parameters();

    /**
     * @brief Register callbacks for parameter changes
     */
    void set_parameter_callback();

    /**
     * @brief Handle start/stop commands with edge detection
     */
    void handle_commands();

    /// The core startup sequencer instance
    sfc::sfc_startup_sequencer _sequencer;

    /// Edge detection flags for commands
    bool _prev_start_command{false};
    bool _prev_stop_command{false};
};

}  // namespace yeet::application::sfc::integration
