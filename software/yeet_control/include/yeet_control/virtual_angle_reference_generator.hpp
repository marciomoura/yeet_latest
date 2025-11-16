#pragma once

#include "yeet_control/angle_wrapped.hpp"
#include "yeet_control/types.hpp"
#include "yeet_control/units.hpp"

namespace yeet::control {

/**
 * @brief Generates a continuously integrating angle based on configurable frequency
 *
 * This component integrates a reference angle over time based on a specified frequency,
 * producing a linearly increasing angle that automatically wraps at 2π. It's primarily
 * used for open-loop testing and commissioning of control systems that normally rely
 * on external synchronization signals (e.g., PLL outputs).
 *
 * The angle integration follows: angle(t+dt) = angle(t) + 2π × frequency × dt
 *
 * Example usage:
 * @code
 * virtual_angle_reference_generator gen;
 * gen.configure_frequency(frequency_t{50.0});  // 50 Hz
 * gen.configure_sampling_time(us(100.0));      // 100 µs
 *
 * // In real-time loop:
 * gen.update();
 * angle_wrapped current_angle = gen.get_angle();
 * @endcode
 */
class virtual_angle_reference_generator {
public:
    /**
     * @brief Constructs a virtual angle reference generator with default parameters
     */
    virtual_angle_reference_generator() = default;

    /**
     * @brief Configures the reference frequency
     *
     * @param frequency The desired frequency in Hz (must be non-negative)
     */
    void configure_frequency(frequency_t frequency);

    /**
     * @brief Configures the sampling time for integration
     *
     * This should match the real-time loop period where update() is called.
     *
     * @param sampling_time The sampling period (must be positive)
     */
    void configure_sampling_time(duration_t sampling_time);

    /**
     * @brief Updates the internal angle state by integrating over one sampling period
     *
     * Performs: angle += 2π × frequency × sampling_time
     * The angle automatically wraps via angle_wrapped.
     */
    void update() noexcept;

    /**
     * @brief Resets the angle to zero
     */
    void reset() noexcept;

    /**
     * @brief Resets the angle to a specified initial value
     *
     * @param initial_angle The angle to reset to
     */
    void reset(angle_wrapped initial_angle) noexcept;

    /**
     * @brief Gets the current angle
     *
     * @return The current integrated angle
     */
    angle_wrapped get_angle() const noexcept;

    /**
     * @brief Gets the configured frequency
     *
     * @return The current frequency setting in Hz
     */
    frequency_t get_frequency() const noexcept;

    /**
     * @brief Gets the configured sampling time
     *
     * @return The current sampling time
     */
    duration_t get_sampling_time() const noexcept;

private:
    angle_wrapped _angle{0.0};             ///< Current integrated angle
    frequency_t _frequency{50.0};          ///< Reference frequency in Hz
    duration_t _sampling_time{us(100.0)};  ///< Integration time step
};

}  // namespace yeet::control
