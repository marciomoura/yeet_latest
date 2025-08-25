#include "sfc/sfc_commutation_monitor.hpp"

#include <cmath>

#include "yeet_control/reference_frame.hpp"

namespace yeet {

sfc_commutation_monitor::sfc_commutation_monitor(const sfc_commutation_monitor::Config& config) : _config(config) {}

bool sfc_commutation_monitor::update(
    const abc<voltage_pu_t>& phase_voltages, const control::six_pulse_generator::pulses& pulses)
{
    // Step 1: Convert phase voltages to line voltages
    const auto v_ab = (phase_voltages.a() - phase_voltages.b()) / std::sqrt(3.0f);
    const auto v_bc = (phase_voltages.b() - phase_voltages.c()) / std::sqrt(3.0f);
    const auto v_ca = (phase_voltages.c() - phase_voltages.a()) / std::sqrt(3.0f);

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
    update_line_commutation_state(_is_commuting_ab, _is_armed_ab, abs(v_ca), _p1_detector || _p4_detector);

    // Commutation is initiated by firing T3 or T6.
    // When T6 is fired, T5 (C+) and T4 (A-) are conducting; phase (B-) shorts with phase (A-)
    // When T3 is fired, T2 (C-) and T1 (A+) are conducting; phase (B+) shorts with phase (A+)
    update_line_commutation_state(_is_commuting_bc, _is_armed_bc, abs(v_ab), _p3_detector || _p6_detector);

    // Commutation is initiated by firing T5 or T2.
    // When T2 (C-) is fired, T1 (A+) and T6 (B-) are conducting; phase (B-) shorts with phase (C-)
    // When T5 (C+) is fired, T4 (A-) and T3 (B+) are conducting; phase (B+) shorts with phase (C+)
    update_line_commutation_state(_is_commuting_ca, _is_armed_ca, abs(v_bc), _p2_detector || _p5_detector);

    // Step 4: Determine the master 'is_commuting' signal
    return is_commuting();
}

void sfc_commutation_monitor::set_parameters(const Config& config) { _config = config; }

bool sfc_commutation_monitor::is_commuting() const { return _is_commuting_ab || _is_commuting_bc || _is_commuting_ca; }

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