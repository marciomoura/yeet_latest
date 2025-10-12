#pragma once

#include "sfc/sfc_soft_starter_speed_generator.hpp"
#include "sfc_integration/sfc_soft_starter_speed_generator_datamodel.hpp"
#include "yeet/application_interface.hpp"
#include "yeet_control/constants.hpp"

namespace yeet::application::sfc::integration {

/**
 * @brief Runner for SFC Soft-Starter Speed Reference Generator
 *
 * This runner integrates the soft-starter speed generator into the yeet application framework.
 * It handles:
 * - Parameter configuration from datamodel to application
 * - Command edge detection for profile and state transitions
 * - Real-time execution at 1ms sampling rate
 * - Output signal mapping to speed controller and torque limiting
 *
 * The runner coordinates with the state machine to automatically sequence through
 * acceleration profiles during the turbine starting process.
 */
class sfc_soft_starter_speed_generator_runner
    : public yeet::base_application<sfc_soft_starter_speed_generator_datamodel::inputs,
          sfc_soft_starter_speed_generator_datamodel::outputs,
          sfc_soft_starter_speed_generator_datamodel::parameters> {
public:
    using datamodel = sfc_soft_starter_speed_generator_datamodel;

    static constexpr duration_t sampling_time = ms(1.0);

    sfc_soft_starter_speed_generator_runner() : base_application(sampling_time), _speed_generator(sampling_time) {}

    void initialize() override;
    void run() override;
    void stop() override;

private:
    /**
     * @brief Configure all profiles and limits from datamodel parameters
     */
    void set_parameters();

    /**
     * @brief Register callbacks for parameter changes
     */
    void set_parameter_callback();

    /**
     * @brief Handle command inputs with edge detection and profile selection
     *
     * Processes all command inputs (run_profile_1-4, hold_speed, decelerate, idle)
     * and executes the appropriate command with proper edge detection.
     */
    void handle_commands();

    /**
     * @brief Build a profile configuration from datamodel parameters
     * @param profile Profile identifier (PROFILE_1 through PROFILE_4)
     * @return Configured acceleration_profile_config structure
     */
    sfc::acceleration_profile_config build_profile_config(sfc::acceleration_profile_t profile);

    /// The core speed generator instance
    sfc::sfc_soft_starter_speed_generator _speed_generator;

    /// Edge detection flags for all commands
    bool _prev_run_profile_1{false};
    bool _prev_run_profile_2{false};
    bool _prev_run_profile_3{false};
    bool _prev_run_profile_4{false};
    bool _prev_hold_speed{false};
    bool _prev_decelerate{false};
    bool _prev_idle{false};
};

}  // namespace yeet::application::sfc::integration
