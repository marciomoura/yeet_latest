#pragma once

#include <memory>

#include "sfc/sfc_per_unit_system.hpp"
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
     * @param per_unit_system Shared per-unit system for frequency conversions
     */
    explicit sfc_startup_sequencer_runner(std::shared_ptr<sfc::sfc_per_unit_system> per_unit_system)
        : base_application(sampling_time), _sequencer(sampling_time), _per_unit_system(std::move(per_unit_system))
    {
    }

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

    /// The core startup sequencer instance
    sfc::sfc_startup_sequencer _sequencer;

    /// Per-unit system for frequency conversions
    std::shared_ptr<sfc::sfc_per_unit_system> _per_unit_system;
};

}  // namespace yeet::application::sfc::integration
