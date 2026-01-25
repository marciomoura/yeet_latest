#pragma once

#include "yeet_control/off_delay.hpp"
#include "yeet_control/on_delay.hpp"
#include "yeet_control/units.hpp"

namespace yeet::application::sfc {

/**
 * @brief Grid-side undervoltage protection for LCI systems.
 *
 * This component monitors the grid voltage magnitude and provides:
 * 1. Non-latched 'detection' status (for immediate end-position action)
 * 2. Latched 'trip' protection (fatal error after timeout)
 * 3. Hysteresis via separate detection and recovery thresholds
 * 4. Recovery confirmation delay to prevent chattering
 *
 * Operational Strategy:
 * - V < detection_threshold: Enter 'detected' state (stop modulation/end-position)
 * - V > recovery_threshold for recovery_delay: Exit 'detected' state
 * - V < detection_threshold for trip_delay: Latch fatal 'trip'
 */
class sfc_undervoltage_protection {
public:
    // Default protection thresholds for LCI grid-side undervoltage
    static constexpr voltage_pu_t default_detection_threshold = voltage_pu_t{0.85f};  // 85% - enter safe mode
    static constexpr voltage_pu_t default_recovery_threshold = voltage_pu_t{0.90f};   // 90% - begin recovery
    static constexpr voltage_pu_t default_warning_threshold = voltage_pu_t{0.92f};    // 92% - pre-warning
    static constexpr duration_t default_trip_delay = duration_t{0.5};                 // 500ms
    static constexpr duration_t default_recovery_delay = duration_t{0.1};             // 100ms
    static constexpr duration_t default_warning_delay = duration_t{0.1};              // 100ms

    /**
     * @brief Constructs the undervoltage protection with specified sampling period.
     *
     * @param sampling_period The control loop sampling period [s]
     */
    explicit sfc_undervoltage_protection(duration_t sampling_period);

    /**
     * @brief Configures the detection threshold.
     *
     * @param detection_threshold Grid voltage threshold for dip detection [pu]
     */
    void configure_detection_threshold(voltage_pu_t detection_threshold);

    /**
     * @brief Configures the recovery threshold.
     *
     * @param recovery_threshold Grid voltage threshold for recovery [pu]
     */
    void configure_recovery_threshold(voltage_pu_t recovery_threshold);

    /**
     * @brief Configures the trip threshold (legacy compatibility).
     *
     * Maps to detection_threshold.
     */
    void configure_trip_threshold(voltage_pu_t trip_threshold, real_t hysteresis_factor = 0.05);

    /**
     * @brief Configures the warning threshold.
     */
    void configure_warning_threshold(voltage_pu_t warning_threshold);

    /**
     * @brief Configures the time delay before trip activation.
     */
    void configure_trip_delay(duration_t trip_delay);

    /**
     * @brief Configures the time delay before recovery activation.
     */
    void configure_recovery_delay(duration_t recovery_delay);

    /**
     * @brief Configures the time delay before warning activation.
     */
    void configure_warning_delay(duration_t warning_delay);

    /**
     * @brief Updates the protection with the latest grid voltage measurement.
     *
     * @param grid_voltage_magnitude_pu Grid voltage magnitude in per-unit [pu]
     * @param enabled If false, the protection clears internal detection and warning logic.
     */
    void update(voltage_pu_t grid_voltage_magnitude_pu, bool enabled = true);

    /**
     * @brief Resets the trip latch if fault condition has cleared.
     *
     * Reset is only allowed when voltage is above recovery threshold.
     */
    void reset();

    /**
     * @brief Gets the latched trip status.
     *
     * @return True if undervoltage protection has tripped and remains latched
     */
    [[nodiscard]] bool is_tripped() const noexcept;

    /**
     * @brief Gets the "dip detected" status (for end-position logic).
     *
     * This is non-latched but includes the recovery delay.
     *
     * @return True if voltage dip is active (V < detection or recovering)
     */
    [[nodiscard]] bool is_detected() const noexcept;

    /**
     * @brief Gets the non-latched warning status.
     */
    [[nodiscard]] bool is_warning() const noexcept;

    /**
     * @brief Checks if protection can be reset.
     *
     * @return True if fault condition has cleared and reset is allowed
     */
    [[nodiscard]] bool can_reset() const noexcept;

    [[nodiscard]] voltage_pu_t get_current_value() const noexcept;
    [[nodiscard]] voltage_pu_t get_detection_threshold() const noexcept;
    [[nodiscard]] voltage_pu_t get_recovery_threshold() const noexcept;
    [[nodiscard]] voltage_pu_t get_warning_threshold() const noexcept;

    // Legacy getter matching generic protection interface
    [[nodiscard]] voltage_pu_t get_trip_threshold() const noexcept { return get_detection_threshold(); }

private:
    voltage_pu_t _detection_threshold{default_detection_threshold};
    voltage_pu_t _recovery_threshold{default_recovery_threshold};
    voltage_pu_t _warning_threshold{default_warning_threshold};
    voltage_pu_t _current_value{0.0f};

    // Timers for different protection aspects
    control::on_delay _trip_timer;       ///< Trip if V < detection for trip_delay
    control::off_delay _recovery_timer;  ///< Keep 'detected' active until V > recovery for recovery_delay
    control::on_delay _warning_timer;    ///< Simple warning signal

    bool _is_tripped{false};
    bool _dip_raw_input{false};
};

}  // namespace yeet::application::sfc
