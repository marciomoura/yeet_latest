#pragma once

#include <array>
#include <cstddef>

#include "yeet_control/elapsed_timer.hpp"
#include "yeet_control/rate_of_change_limiter.hpp"
#include "yeet_control/types.hpp"
#include "yeet_control/units.hpp"

namespace yeet::application::sfc {

/**
 * @brief Generic profile identifiers for field-configurable acceleration profiles.
 */
enum class acceleration_profile_t {
    PROFILE_1 = 0,  ///< First acceleration profile (typically: creep speed)
    PROFILE_2 = 1,  ///< Second acceleration profile (typically: purge acceleration)
    PROFILE_3 = 2,  ///< Third acceleration profile (typically: ignition acceleration)
    PROFILE_4 = 3   ///< Fourth acceleration profile (typically: assist acceleration)
};

/**
 * @brief Operational commands for the speed generator.
 */
enum class speed_command_t {
    IDLE,        ///< No active control, outputs zero
    HOLD_SPEED,  ///< Hold at current speed (stationary setpoint)
    DECELERATE,  ///< Controlled ramp down to zero speed
    RUN_PROFILE  ///< Execute the currently selected acceleration profile
};

/**
 * @brief Configuration for a single acceleration profile.
 */
struct acceleration_profile_config {
    bool enabled{false};                    ///< Profile is configured and ready to use
    duration_t acceleration_time{120.0};    ///< Time to go from 0 to 100% speed [s]
    torque_pu_t torque_limit_max{1.0};      ///< Maximum motoring torque limit [pu]
    torque_pu_t torque_limit_min{0.0};      ///< Minimum braking torque limit [pu]
    torque_pu_t initial_torque_boost{0.0};  ///< Breakaway torque boost [pu]
    duration_t boost_duration{1.0};         ///< Duration of torque boost [s]
    speed_pu_t target_speed{0.0};           ///< Target speed for this profile [pu]
};

/**
 * @brief Generates speed references and torque limits for soft-starter operation.
 *
 * This class implements a generic profile-based speed generation strategy for gas turbine
 * soft-starting applications. It provides:
 * - Four field-configurable acceleration profiles with enable/disable capability
 * - Factory methods for typical profile configurations (creep, purge, ignition, assist)
 * - Command-based execution model (IDLE, HOLD_SPEED, DECELERATE, RUN_PROFILE)
 * - Dynamic torque limit generation with breakaway boost support
 * - Profile completion detection and validation
 *
 * The class follows the standard yeet_application_sfc architectural pattern:
 * - Configuration via configure_*() methods
 * - Profile selection via select_profile()
 * - Command execution via execute_command()
 * - Real-time execution via update()
 * - Outputs retrieved via get_*() methods
 * - State reset via reset()
 *
 * **Usage Pattern:**
 * ```cpp
 * // 1. Construction
 * sfc_soft_starter_speed_generator speed_gen(sampling_time);
 *
 * // 2. Configuration using factory methods
 * auto purge_config = sfc_soft_starter_speed_generator::make_purge_accel_profile(
 *     speed_pu_t{0.20}, duration_t{120.0});
 * speed_gen.configure_profile(acceleration_profile_t::PROFILE_2, purge_config);
 *
 * // 3. Profile selection and execution
 * if (speed_gen.select_profile(acceleration_profile_t::PROFILE_2)) {
 *     speed_gen.execute_command(speed_command_t::RUN_PROFILE);
 * }
 *
 * // 4. Real-time execution
 * speed_gen.update(measured_speed);
 *
 * // 5. Output retrieval
 * auto speed_ref = speed_gen.get_speed_reference_pu();
 * auto torque_max = speed_gen.get_torque_limit_max_pu();
 *
 * // 6. Command execution
 * if (speed_gen.is_profile_complete()) {
 *     speed_gen.execute_command(speed_command_t::HOLD_SPEED);
 * }
 * ```
 */
class sfc_soft_starter_speed_generator {
public:
    /**
     * @brief Constructs the speed generator.
     * @param sampling_time The control loop sampling time [s].
     */
    explicit sfc_soft_starter_speed_generator(duration_t sampling_time);

    // ========== Factory Methods for Standard Profiles ==========

    /**
     * @brief Creates a typical creep speed profile configuration.
     *
     * Creep speed is used for maintenance operations where the turbine needs to
     * rotate very slowly for inspection or manual positioning.
     *
     * @param target_speed_pu Target speed (typically 0.05-0.10 pu).
     * @param acceleration_time Time to reach creep speed [s].
     * @return Configured profile with enabled=true.
     */
    static acceleration_profile_config make_creep_speed_profile(
        speed_pu_t target_speed_pu = speed_pu_t{0.05}, duration_t acceleration_time = duration_t{60.0});

    /**
     * @brief Creates a typical purge acceleration profile configuration.
     *
     * Purge acceleration brings the turbine to purge speed where combustion chamber
     * ventilation occurs to clear any residual fuel vapors before ignition.
     *
     * @param target_speed_pu Target purge speed (typically 0.15-0.25 pu).
     * @param acceleration_time Time to reach purge speed [s].
     * @return Configured profile with enabled=true.
     */
    static acceleration_profile_config make_purge_accel_profile(
        speed_pu_t target_speed_pu = speed_pu_t{0.20}, duration_t acceleration_time = duration_t{120.0});

    /**
     * @brief Creates a typical ignition acceleration profile configuration.
     *
     * Ignition acceleration brings the turbine to the speed required for successful
     * fuel ignition and flame stabilization.
     *
     * @param target_speed_pu Target ignition speed (typically 0.50-0.60 pu).
     * @param acceleration_time Time to reach ignition speed [s].
     * @return Configured profile with enabled=true.
     */
    static acceleration_profile_config make_ignition_accel_profile(
        speed_pu_t target_speed_pu = speed_pu_t{0.55}, duration_t acceleration_time = duration_t{90.0});

    /**
     * @brief Creates a typical assist acceleration profile configuration.
     *
     * Assist acceleration brings the turbine to self-sustaining speed where it can
     * maintain rotation under its own combustion power.
     *
     * @param target_speed_pu Target self-sustaining speed (typically 0.90-1.00 pu).
     * @param acceleration_time Time to reach target speed [s].
     * @return Configured profile with enabled=true.
     */
    static acceleration_profile_config make_assist_accel_profile(
        speed_pu_t target_speed_pu = speed_pu_t{0.95}, duration_t acceleration_time = duration_t{60.0});

    // ========== Configuration Methods ==========

    /**
     * @brief Configures a specific acceleration profile.
     * @param profile The profile identifier to configure.
     * @param config The configuration parameters (should have enabled=true).
     */
    void configure_profile(acceleration_profile_t profile, const acceleration_profile_config& config);

    /**
     * @brief Configures the deceleration profile.
     * @param deceleration_time Time to ramp from 100% to 0% speed [s].
     */
    void configure_deceleration(duration_t deceleration_time);

    /**
     * @brief Configures absolute speed and torque limits.
     * @param min_speed_pu Minimum allowed speed reference [pu].
     * @param max_speed_pu Maximum allowed speed reference [pu].
     * @param max_torque_pu Absolute maximum torque limit [pu].
     * @param min_torque_pu Absolute minimum torque limit [pu].
     */
    void configure_absolute_limits(
        speed_pu_t min_speed_pu, speed_pu_t max_speed_pu, torque_pu_t max_torque_pu, torque_pu_t min_torque_pu);

    // ========== Profile Selection and Validation ==========

    /**
     * @brief Checks if a profile is configured and enabled.
     * @param profile The profile to check.
     * @return True if the profile is ready to use.
     */
    bool is_profile_enabled(acceleration_profile_t profile) const;

    /**
     * @brief Disables a profile (prevents it from being selected).
     * @param profile The profile to disable.
     */
    void disable_profile(acceleration_profile_t profile);

    /**
     * @brief Selects which acceleration profile to run.
     *
     * This method validates that the profile is enabled before selecting it.
     * The profile will be executed when execute_command(RUN_PROFILE) is called.
     *
     * @param profile The profile to select (must be configured and enabled).
     * @return True if profile is enabled and was selected successfully.
     */
    bool select_profile(acceleration_profile_t profile);

    /**
     * @brief Gets the currently selected profile.
     * @return Selected profile (valid only when RUN_PROFILE command is active).
     */
    acceleration_profile_t get_selected_profile() const;

    // ========== Command Execution ==========

    /**
     * @brief Executes a speed generator command.
     *
     * Available commands:
     * - IDLE: Deactivate all control, output zero
     * - HOLD_SPEED: Maintain current speed as setpoint
     * - DECELERATE: Ramp down to zero speed
     * - RUN_PROFILE: Execute the currently selected profile
     *
     * @param command The command to execute.
     * @return True if command was accepted and executed.
     */
    bool execute_command(speed_command_t command);

    /**
     * @brief Gets the current active command.
     * @return Active command.
     */
    speed_command_t get_active_command() const;

    // ========== Real-Time Execution ==========

    // ========== Real-Time Execution ==========

    /**
     * @brief Executes one update cycle of the speed generator.
     * @param actual_speed_pu Current measured speed [pu] (used for trajectory tracking).
     */
    void update(speed_pu_t actual_speed_pu);

    // ========== Output Accessors ==========

    /**
     * @brief Gets the current speed reference output.
     * @return Speed reference [pu].
     */
    speed_pu_t get_speed_reference_pu() const;

    /**
     * @brief Gets the maximum torque limit for the current mode.
     * @return Maximum torque limit [pu].
     */
    torque_pu_t get_torque_limit_max_pu() const;

    /**
     * @brief Gets the minimum torque limit for the current mode.
     * @return Minimum torque limit [pu].
     */
    torque_pu_t get_torque_limit_min_pu() const;

    // ========== Status Methods ==========

    /**
     * @brief Checks if a profile has completed its target.
     * @return True if the active acceleration profile has reached its target speed.
     */
    bool is_profile_complete() const;

    /**
     * @brief Checks if breakaway torque boost is currently active.
     * @return True if boost is active.
     */
    bool is_breakaway_boost_active() const;

    /**
     * @brief Resets all internal states to initial conditions.
     * @param initial_speed_pu Starting speed for reset [pu].
     */
    void reset(speed_pu_t initial_speed_pu = speed_pu_t{0.0});

private:
    // Configuration storage
    duration_t _sampling_time;

    // Profile configurations (indexed by mode enum value)
    std::array<acceleration_profile_config, 4> _profile_configs;

    // Absolute limits
    speed_pu_t _min_speed_limit{0.0};
    speed_pu_t _max_speed_limit{1.2};
    torque_pu_t _max_torque_limit{1.5};
    torque_pu_t _min_torque_limit{-1.5};

    // Deceleration configuration
    duration_t _deceleration_time{20.0};

    // Active state
    speed_command_t _active_command{speed_command_t::IDLE};
    acceleration_profile_t _selected_profile{acceleration_profile_t::PROFILE_1};
    speed_pu_t _speed_reference_pu{0.0};
    speed_pu_t _hold_speed_setpoint{0.0};
    torque_pu_t _torque_limit_max_pu{0.0};
    torque_pu_t _torque_limit_min_pu{0.0};

    // Ramp generation components
    yeet::control::rate_of_change_limiter _speed_ramp_limiter;
    yeet::control::elapsed_timer _breakaway_boost_timer;

    // Internal state tracking
    bool _is_profile_complete{false};
    bool _is_breakaway_boost_active{false};
    speed_pu_t _profile_start_speed{0.0};
    speed_pu_t _profile_target_speed{0.0};

    // Private helper methods
    void execute_acceleration_profile(const acceleration_profile_config& profile);
    void execute_hold_mode();
    void execute_deceleration();
    void update_torque_limits();
    size_t get_profile_index(acceleration_profile_t profile) const;
};

}  // namespace yeet::application::sfc
