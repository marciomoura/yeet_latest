#pragma once

#include "yeet_control/types.hpp"
#include "yeet_control/units.hpp"

namespace yeet::control {

/**
 * @brief Robust boolean debouncer using an integrator-based approach.
 *
 * This component provides high noise immunity by accumulating input states over time.
 * Instead of resetting on a single mismatched sample, it increments/decrements
 * an internal state (like a saturating integrator), providing a low-pass effect
 * for boolean signals.
 *
 * - ON transition: Input must be predominantly true for on_delay_s.
 * - OFF transition: Input must be predominantly false for off_delay_s.
 *
 * This is particularly effective for filtering switch chatter or EMI in trip signals.
 */
class boolean_debouncer {
public:
    /**
     * @brief Default constructor.
     */
    boolean_debouncer() noexcept = default;

    /**
     * @brief Configures the sampling time.
     * @param sampling_time The execution interval in seconds.
     */
    void configure_sampling_time(duration_t sampling_time) noexcept;

    /**
     * @brief Configures the debounce durations.
     * @param on_delay_s duration input must be true to switch output to true.
     * @param off_delay_s duration input must be false to switch output to false.
     */
    void configure_delay(duration_t on_delay_s, duration_t off_delay_s) noexcept;

    /**
     * @brief Updates the debouncer state with latest input value.
     * @param input Raw boolean signal.
     */
    void update(bool input) noexcept;

    /**
     * @brief Resets the integrator and output to initial (false) state.
     */
    void reset() noexcept;

    /**
     * @brief Resets the integrator and output to a specific state.
     * @param initial_state The state to reset to.
     */
    void reset(bool initial_state) noexcept;

    /**
     * @brief Gets the current debounced output state.
     * @return true if filtered signal is active.
     */
    [[nodiscard]] bool get_output() const noexcept;

    /**
     * @brief Gets the internal integrator state [0.0, 1.0].
     * @return Current integrator value.
     */
    [[nodiscard]] real_t get_integrator() const noexcept;

private:
    void update_increments() noexcept;

    real_t _sampling_time{0.0f};
    real_t _on_delay_s{0.0f};
    real_t _off_delay_s{0.0f};
    real_t _on_increment{0.0f};
    real_t _off_decrement{0.0f};
    real_t _integrator{0.0f};
    bool _output{false};
};

}  // namespace yeet::control
