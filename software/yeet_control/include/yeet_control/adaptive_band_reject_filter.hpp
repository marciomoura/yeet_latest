#ifndef YEET_CONTROL_ADAPTIVE_BAND_REJECT_FILTER_HPP
#define YEET_CONTROL_ADAPTIVE_BAND_REJECT_FILTER_HPP

#include <cassert>
#include <cmath>

#include "second_order_filter.hpp"
#include "types.hpp"

namespace yeet::control {

/**
 * @brief An adaptive band-reject (notch) filter with two parallel filters for smooth frequency transitions.
 *
 * This class manages two second-order notch filters to provide a continuous, glitch-free output
 * even when the center rejection frequency changes significantly.
 *
 * How it works:
 * 1. One filter is always "active", providing the output signal.
 * 2. When the input signal's estimated frequency deviates from the active filter's center frequency
 *    by a configured threshold, a switch is triggered.
 * 3. The "inactive" filter is re-configured to the new estimated frequency.
 * 4. The system waits for a configured "settling time" to allow the new filter's internal state
 *    to stabilize. During this time, the "old" filter remains active.
 * 5. After the settling time has passed, the newly configured filter becomes active, ensuring a
 *    smooth transition.
 */
template <typename T>
class adaptive_band_reject_filter {
public:
    using type = fundamental_type_t<T>;

    /**
     * @brief Constructor
     *
     * @param sampling_period Sampling period in seconds
     */
    explicit adaptive_band_reject_filter(type sampling_period)
        : _sampling_time(sampling_period),
          _filters{second_order_band_reject_filter<T>(sampling_period),
                   second_order_band_reject_filter<T>(sampling_period)} {
        assert(sampling_period > type{0} && "Sampling time must be positive");
    }

    /**
     * @brief Configure the adaptive filter parameters.
     *
     * @param initial_center_freq   The starting center frequency for both filters (Hz).
     * @param bandwidth             The bandwidth of the notch filter (Hz).
     * @param frequency_threshold_hz The frequency deviation (Hz) from the active filter's center
     *                              frequency that will trigger a filter swap.
     * @param settling_time_s       The time (s) to wait for the inactive filter to settle after
     *                              re-configuration before making it active.
     */
    void configure(type initial_center_freq, type bandwidth, type frequency_threshold_hz, type settling_time_s) {
        assert(initial_center_freq > type{0} && "Initial center frequency must be positive");
        assert(bandwidth > type{0} && "Bandwidth must be positive");
        assert(frequency_threshold_hz > type{0} && "Frequency threshold must be positive");
        assert(settling_time_s >= type{0} && "Settling time must be non-negative");

        _initial_center_freq = initial_center_freq;
        _bandwidth = bandwidth;
        _frequency_threshold_hz = frequency_threshold_hz;
        _settling_samples_required = static_cast<int>(settling_time_s / _sampling_time);

        reset();
    }

    /**
     * @brief Reset the filter to its initial configured state.
     */
    void reset() {
        _filters[0].configure(_initial_center_freq, _bandwidth);
        _filters[1].configure(_initial_center_freq, _bandwidth);
        _filters[0].reset();
        _filters[1].reset();

        _center_freqs[0] = _initial_center_freq;
        _center_freqs[1] = _initial_center_freq;

        _active_filter_index = 0;
        _is_switching = false;
        _settling_counter = 0;
    }

    /**
     * @brief Update the filter with a new input sample and the current estimated frequency.
     *
     * @param input The input value to be filtered.
     * @param estimated_frequency_hz The current estimated frequency of the signal (Hz).
     * @return The filtered output value.
     */
    T update(T input, T estimated_frequency_hz) {
        // Continuously update both filters to keep their internal states valid.
        const T output0 = _filters[0].update(input);
        const T output1 = _filters[1].update(input);

        if (_is_switching) {
            // A switch is in progress; wait for the settling time to elapse.
            _settling_counter++;
            if (_settling_counter >= _settling_samples_required) {
                // Settling is complete. Make the newly configured filter active.
                _active_filter_index = 1 - _active_filter_index;
                _is_switching = false;
            }
        } else {
            // No switch is in progress. Check if one should be triggered.
            const type freq_deviation = std::abs(estimated_frequency_hz - _center_freqs[_active_filter_index]);

            if (freq_deviation > _frequency_threshold_hz) {
                // Trigger a switch.
                _is_switching = true;
                _settling_counter = 0;

                const int inactive_index = 1 - _active_filter_index;

                // Re-configure the inactive filter to the new frequency.
                _filters[inactive_index].configure(estimated_frequency_hz, _bandwidth);
                _center_freqs[inactive_index] = estimated_frequency_hz;
            }
        }

        // Return the output from the currently active filter.
        return (_active_filter_index == 0) ? output0 : output1;
    }

private:
    type _sampling_time;

    // Internal filter instances
    second_order_band_reject_filter<T> _filters[2];

    // Configuration
    type _initial_center_freq{50.0};
    type _bandwidth{10.0};
    type _frequency_threshold_hz{2.0};
    int _settling_samples_required{0};

    // State
    type _center_freqs[2]{};
    int _active_filter_index{0};
    bool _is_switching{false};
    int _settling_counter{0};
};

}  // namespace yeet::control

#endif  // YEET_CONTROL_ADAPTIVE_BAND_REJECT_FILTER_HPP