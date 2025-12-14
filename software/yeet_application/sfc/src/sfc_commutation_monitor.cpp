#include "sfc/sfc_commutation_monitor.hpp"

#include <cmath>

#include "yeet_control/reference_frame.hpp"

namespace yeet {

sfc_commutation_monitor::sfc_commutation_monitor(
    const sfc_commutation_monitor::Config& config, duration_t sampling_time) noexcept
    : _config(config),
      _sampling_time(sampling_time),
      _notch_duration_timer(sampling_time),
      _notch_duration_filter(static_cast<double>(sampling_time.value()))
{
    configure_notch_duration_filter_time_constant(_notch_duration_filter_time_constant);
}

void sfc_commutation_monitor::configure_notch_duration_filter_time_constant(duration_t time_constant) noexcept
{
    // Guard against non-positive values.
    const real_t tau_s = time_constant.value();
    const real_t ts_s = _sampling_time.value();
    if (tau_s <= 0.0F || ts_s <= 0.0F) {
        return;
    }

    _notch_duration_filter_time_constant = time_constant;

    const real_t nyquist_hz = 0.5F / ts_s;
    real_t cutoff_hz = 1.0F / (2.0F * pi * tau_s);

    // Avoid assertion in the filter if cutoff is at/above Nyquist.
    if (cutoff_hz >= nyquist_hz) {
        cutoff_hz = nyquist_hz * 0.45F;
    }
    if (cutoff_hz <= 0.0F) {
        cutoff_hz = nyquist_hz * 0.45F;
    }

    _notch_duration_filter.configure(cutoff_hz);
}

bool sfc_commutation_monitor::update(
    const abc<voltage_pu_t>& phase_voltages, const control::six_pulse_generator::pulses& pulses)
{
    const bool was_commuting = is_commuting();

    // Step 1: Convert phase voltages to line voltages
    const line_voltage<voltage_pu_t> line_voltages{phase_voltages};

    // Step 2: Detect rising edges for firing pulses
    _p1_detector.update(pulses.p1);
    _p2_detector.update(pulses.p2);
    _p3_detector.update(pulses.p3);
    _p4_detector.update(pulses.p4);
    _p5_detector.update(pulses.p5);
    _p6_detector.update(pulses.p6);

    // Step 3: Update commutation state for each line voltage
    // Commutation is initiated by firing T1 or T4.
    // When T1 (A+) is fired, T5 (C+) and T6 (B-) are conducting; phase (A+) shorts with phase (C+)
    // When T4 (A-) is fired, T2 (C-) and T2 (B+) are conducting; phase (A-) shorts with phase (C-)
    update_line_commutation_state(
        _is_commuting_ab, _is_armed_ab, abs(line_voltages.ca()), _p1_detector || _p4_detector);

    // Commutation is initiated by firing T3 or T6.
    // When T6 is fired, T5 (C+) and T4 (A-) are conducting; phase (B-) shorts with phase (A-)
    // When T3 is fired, T2 (C-) and T1 (A+) are conducting; phase (B+) shorts with phase (A+)
    update_line_commutation_state(
        _is_commuting_bc, _is_armed_bc, abs(line_voltages.ab()), _p3_detector || _p6_detector);

    // Commutation is initiated by firing T5 or T2.
    // When T2 (C-) is fired, T1 (A+) and T6 (B-) are conducting; phase (B-) shorts with phase (C-)
    // When T5 (C+) is fired, T4 (A-) and T3 (B+) are conducting; phase (B+) shorts with phase (C+)
    update_line_commutation_state(
        _is_commuting_ca, _is_armed_ca, abs(line_voltages.bc()), _p2_detector || _p5_detector);

    // Step 4: Determine the master 'is_commuting' signal
    const bool now_commuting = is_commuting();

    // Step 5: Measure commutation notch duration.
    if (now_commuting && !was_commuting) {
        _notch_duration_timer.reset();
    }

    _notch_duration_timer.update(now_commuting);

    if (!now_commuting && was_commuting) {
        // Falling edge: latch the last measured duration
        _notch_duration_last = _notch_duration_timer.get_elapsed_time();
        _notch_duration_timer.reset();
    }

    // Step 6: Filter the latest latched notch duration to reduce jitter.
    (void)_notch_duration_filter.update(_notch_duration_last);

    return now_commuting;
}

void sfc_commutation_monitor::set_parameters(const Config& config) { _config = config; }

bool sfc_commutation_monitor::is_commuting() const { return _is_commuting_ab || _is_commuting_bc || _is_commuting_ca; }

duration_t sfc_commutation_monitor::get_notch_duration_last() const noexcept { return _notch_duration_last; }

duration_t sfc_commutation_monitor::get_notch_duration_filtered() const noexcept
{
    return _notch_duration_filter.get_output();
}

void sfc_commutation_monitor::update_line_commutation_state(
    bool& is_line_commuting, bool& is_line_armed, voltage_pu_t line_voltage, bool has_fired)
{
    if (has_fired) {
        is_line_armed = true;
    }

    if (is_line_armed && !is_line_commuting) {
        // An arming pulse was received, and we are not yet in a commutation state.
        // We now wait for the voltage to drop below the start threshold.
        if (line_voltage < _config.commutation_start_threshold) {
            is_line_commuting = true;
        }
    }
    else if (is_line_commuting) {
        // If we are already in a commutation state, we only exit it if the voltage has recovered.
        if (line_voltage > _config.commutation_end_threshold) {
            is_line_commuting = false;
            is_line_armed = false;  // Disarm after commutation is complete.
        }
    }
}

}  // namespace yeet