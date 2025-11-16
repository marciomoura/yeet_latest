#pragma once

#include "yeet_control/threshold_protection.hpp"
#include "yeet_control/units.hpp"

namespace yeet::application::sfc {

/**
 * @brief V/Hz (voltage-to-frequency ratio) protection for LCI systems.
 *
 * This component monitors the voltage-to-frequency ratio (flux) and provides
 * dual-mode latching protection against both over-flux and under-flux conditions.
 *
 * Physical phenomena:
 * - Over-Flux: When V/Hz exceeds rated value, iron core saturates → excessive
 *   eddy currents → rapid thermal damage to transformer/motor windings
 * - Under-Flux: When V/Hz is too low, magnetic field weakens → low torque →
 *   increased current draw to compensate → potential overcurrent condition
 *
 * Typical LCI thresholds:
 * - Over-flux trip: 110% of rated V/Hz (1.10 pu)
 * - Over-flux warning: 105% of rated V/Hz (1.05 pu)
 * - Under-flux trip: 80% of rated V/Hz (0.80 pu)
 * - Under-flux warning: 90% of rated V/Hz (0.90 pu)
 *
 * The protection calculates V/Hz ratio from input voltage (RMS) and frequency,
 * normalizes to per-unit using rated V/Hz reference, and monitors against
 * independent over-flux and under-flux thresholds.
 *
 * Both trip conditions latch independently and require explicit reset after
 * fault conditions have cleared.
 */
class sfc_vhz_protection {
public:
    // Default protection thresholds for LCI V/Hz protection
    static constexpr flux_pu_t default_over_flux_trip_threshold = flux_pu_t{1.10};     // 110% V/Hz
    static constexpr flux_pu_t default_over_flux_warning_threshold = flux_pu_t{1.05};  // 105% V/Hz
    static constexpr duration_t default_over_flux_trip_delay = duration_t{1.0};        // 1.0 second
    static constexpr duration_t default_over_flux_warning_delay = duration_t{0.5};     // 0.5 second
    static constexpr real_t default_over_flux_hysteresis = real_t{0.05};               // 5%

    static constexpr flux_pu_t default_under_flux_trip_threshold = flux_pu_t{0.80};     // 80% V/Hz
    static constexpr flux_pu_t default_under_flux_warning_threshold = flux_pu_t{0.90};  // 90% V/Hz
    static constexpr duration_t default_under_flux_trip_delay = duration_t{5.0};        // 5.0 seconds
    static constexpr duration_t default_under_flux_warning_delay = duration_t{2.0};     // 2.0 seconds
    static constexpr real_t default_under_flux_hysteresis = real_t{0.05};               // 5%

    static constexpr flux_pu_t default_rated_vhz = flux_pu_t{1.0};                // 1.0 pu
    static constexpr frequency_t minimum_frequency_threshold = frequency_t{0.1};  // 0.1 Hz

    /**
     * @brief Constructs the V/Hz protection with specified sampling period.
     *
     * @param sampling_period The control loop sampling period [s]
     */
    explicit sfc_vhz_protection(duration_t sampling_period);

    /**
     * @brief Configures the over-flux trip threshold and hysteresis.
     *
     * @param trip_threshold V/Hz ratio threshold for over-flux trip [pu]
     * @param hysteresis_factor Hysteresis band as fraction of threshold (default 5%)
     */
    void configure_over_flux_trip_threshold(
        flux_pu_t trip_threshold, real_t hysteresis_factor = default_over_flux_hysteresis);

    /**
     * @brief Configures the over-flux warning threshold.
     *
     * Warning threshold should be lower than trip threshold.
     *
     * @param warning_threshold V/Hz ratio threshold for over-flux warning [pu]
     */
    void configure_over_flux_warning_threshold(flux_pu_t warning_threshold);

    /**
     * @brief Configures the time delay before over-flux trip activation.
     *
     * @param trip_delay Time delay for over-flux trip [s]
     */
    void configure_over_flux_trip_delay(duration_t trip_delay);

    /**
     * @brief Configures the time delay before over-flux warning activation.
     *
     * @param warning_delay Time delay for over-flux warning [s]
     */
    void configure_over_flux_warning_delay(duration_t warning_delay);

    /**
     * @brief Configures the under-flux trip threshold and hysteresis.
     *
     * @param trip_threshold V/Hz ratio threshold for under-flux trip [pu]
     * @param hysteresis_factor Hysteresis band as fraction of threshold (default 5%)
     */
    void configure_under_flux_trip_threshold(
        flux_pu_t trip_threshold, real_t hysteresis_factor = default_under_flux_hysteresis);

    /**
     * @brief Configures the under-flux warning threshold.
     *
     * Warning threshold should be higher than trip threshold.
     *
     * @param warning_threshold V/Hz ratio threshold for under-flux warning [pu]
     */
    void configure_under_flux_warning_threshold(flux_pu_t warning_threshold);

    /**
     * @brief Configures the time delay before under-flux trip activation.
     *
     * @param trip_delay Time delay for under-flux trip [s]
     */
    void configure_under_flux_trip_delay(duration_t trip_delay);

    /**
     * @brief Configures the time delay before under-flux warning activation.
     *
     * @param warning_delay Time delay for under-flux warning [s]
     */
    void configure_under_flux_warning_delay(duration_t warning_delay);

    /**
     * @brief Configures the rated V/Hz reference for per-unit normalization.
     *
     * For a 50 Hz system with rated voltage V_rated:
     * rated_vhz = V_rated / 50 Hz
     *
     * @param rated_vhz_value The rated V/Hz value for normalization
     */
    void configure_rated_vhz(flux_pu_t rated_vhz_value);

    /**
     * @brief Updates the protection with latest voltage and frequency measurements.
     *
     * Calculates V/Hz ratio, normalizes to per-unit, and evaluates against
     * over-flux and under-flux thresholds.
     *
     * Zero-frequency handling: When frequency is below minimum threshold (0.1 Hz),
     * the V/Hz ratio is set to zero and protection is effectively disabled
     * to prevent false trips during startup.
     *
     * @param voltage_rms RMS voltage magnitude in per-unit [pu]
     * @param frequency Electrical frequency [Hz]
     */
    void update(voltage_pu_t voltage_rms, frequency_t frequency);

    /**
     * @brief Resets both over-flux and under-flux trip latches if conditions cleared.
     *
     * Reset is only allowed when both fault conditions have cleared.
     */
    void reset();

    /**
     * @brief Gets the over-flux trip status.
     *
     * @return True if over-flux protection has tripped and remains latched
     */
    [[nodiscard]] bool is_over_flux_tripped() const noexcept;

    /**
     * @brief Gets the under-flux trip status.
     *
     * @return True if under-flux protection has tripped and remains latched
     */
    [[nodiscard]] bool is_under_flux_tripped() const noexcept;

    /**
     * @brief Gets combined trip status (either over-flux or under-flux).
     *
     * @return True if any flux protection has tripped
     */
    [[nodiscard]] bool is_any_tripped() const noexcept;

    /**
     * @brief Gets the non-latched over-flux warning status.
     *
     * @return True if V/Hz currently exceeds over-flux warning threshold
     */
    [[nodiscard]] bool is_over_flux_warning() const noexcept;

    /**
     * @brief Gets the non-latched under-flux warning status.
     *
     * @return True if V/Hz currently below under-flux warning threshold
     */
    [[nodiscard]] bool is_under_flux_warning() const noexcept;

    /**
     * @brief Checks if protection can be reset.
     *
     * Reset is allowed when both over-flux and under-flux conditions have cleared.
     *
     * @return True if both fault conditions cleared and reset is allowed
     */
    [[nodiscard]] bool can_reset() const noexcept;

    /**
     * @brief Gets the current V/Hz ratio in per-unit.
     *
     * @return The most recent V/Hz ratio calculated from voltage and frequency [pu]
     */
    [[nodiscard]] flux_pu_t get_vhz_ratio_pu() const noexcept;

    /**
     * @brief Gets the configured over-flux trip threshold.
     *
     * @return The over-flux trip threshold [pu]
     */
    [[nodiscard]] flux_pu_t get_over_flux_trip_threshold() const noexcept;

    /**
     * @brief Gets the configured under-flux trip threshold.
     *
     * @return The under-flux trip threshold [pu]
     */
    [[nodiscard]] flux_pu_t get_under_flux_trip_threshold() const noexcept;

private:
    control::threshold_protection<flux_pu_t, control::threshold_direction::above> _over_flux_protection;
    control::threshold_protection<flux_pu_t, control::threshold_direction::below> _under_flux_protection;
    flux_pu_t _rated_vhz{default_rated_vhz};
    flux_pu_t _current_vhz_pu{flux_pu_t{0.0}};
};

}  // namespace yeet::application::sfc
