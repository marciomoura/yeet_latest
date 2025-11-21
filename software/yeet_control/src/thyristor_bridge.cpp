#include "yeet_control/thyristor_bridge.hpp"

#include <vector>

namespace yeet::control {

void thyristor_bridge::update_conduction_state(
    const abc<voltage_t>& v_ac_abc, voltage_t v_dc_positive, voltage_t v_dc_negative, const thyristor_pulses& pulses)
{
    // Store source voltages for terminal voltage calculation
    _v_ac_source = v_ac_abc;

    // Correct physical connections for a standard 6-pulse bridge
    _anode_voltages = {
        v_ac_abc.a(),   // T1 anode is on Phase A
        v_dc_negative,  // T2 anode is on Negative DC Rail
        v_ac_abc.b(),   // T3 anode is on Phase B
        v_dc_negative,  // T4 anode is on Negative DC Rail
        v_ac_abc.c(),   // T5 anode is on Phase C
        v_dc_negative   // T6 anode is on Negative DC Rail
    };

    _cathode_voltages = {
        v_dc_positive,  // T1 cathode is on Positive DC Rail
        v_ac_abc.c(),   // T2 cathode is on Phase C
        v_dc_positive,  // T3 cathode is on Positive DC Rail
        v_ac_abc.a(),   // T4 cathode is on Phase A
        v_dc_positive,  // T5 cathode is on Positive DC Rail
        v_ac_abc.b()    // T6 cathode is on Phase B
    };

    // The thyristors conduct based on the condition that a pulse is applied
    // and the forward voltage on the thyristor itself is positive
    for (int i = 0; i < 6; ++i) {
        if (!_conducting_states[i]) {
            bool has_gate_pulse = pulses[i];
            bool is_forward_biased = (_anode_voltages[i] > _cathode_voltages[i]);
            if (has_gate_pulse && is_forward_biased) {
                // Identify from upper-branch which is the thyristor commutating
                // and which is the thyristor going out of commutation
                _commutation_incoming_thyristor_idx = i;
                _commutation_current = current_t{0.0};

                if (i == 1 - 1 || i == 3 - 1 || i == 5 - 1) {
                    if (_conducting_states[1 - 1]) _commutation_outgoing_thyristor_idx = 1 - 1;
                    if (_conducting_states[3 - 1]) _commutation_outgoing_thyristor_idx = 3 - 1;
                    if (_conducting_states[5 - 1]) _commutation_outgoing_thyristor_idx = 5 - 1;
                }

                // Identify from lower-branch which is the thyristor commutating
                // and which is the thyristor going out of commutation
                if (i == 4 - 1 || i == 6 - 1 || i == 2 - 1) {
                    if (_conducting_states[4 - 1]) _commutation_outgoing_thyristor_idx = 4 - 1;
                    if (_conducting_states[6 - 1]) _commutation_outgoing_thyristor_idx = 6 - 1;
                    if (_conducting_states[2 - 1]) _commutation_outgoing_thyristor_idx = 2 - 1;
                }

                // Update with new state now after identifying what is the outgoing thyristor
                _conducting_states[i] = true;
            }
        }
    }

    // Calculate actual terminal voltages considering commutation overlap
    // During commutation, when multiple thyristors on same rail conduct,
    // those phases are electrically connected through the DC link
    _v_phase_abc = v_ac_abc;  // Start with source voltages

    // Upper rail commutation: Check if multiple upper thyristors (T1, T3, T5) are conducting
    std::vector<int> upper_conducting;
    if (_conducting_states[0]) upper_conducting.push_back(0);  // T1 -> Phase A
    if (_conducting_states[2]) upper_conducting.push_back(1);  // T3 -> Phase B
    if (_conducting_states[4]) upper_conducting.push_back(2);  // T5 -> Phase C

    if (upper_conducting.size() > 1) {
        // Multiple phases connected to positive rail -> voltage notch
        // Terminal voltages collapse to average (phases shorted through DC link)
        voltage_t v_avg{0.0};
        for (int phase : upper_conducting) {
            v_avg = voltage_t(v_avg.value() + v_ac_abc[phase].value());
        }
        v_avg = voltage_t(v_avg.value() / upper_conducting.size());

        // Set all commutating phases to average voltage
        for (int phase : upper_conducting) {
            _v_phase_abc[phase] = v_avg;
        }
    }

    // Lower rail commutation: Check if multiple lower thyristors (T4, T6, T2) are conducting
    std::vector<int> lower_conducting;
    if (_conducting_states[3]) lower_conducting.push_back(0);  // T4 -> Phase A
    if (_conducting_states[5]) lower_conducting.push_back(1);  // T6 -> Phase B
    if (_conducting_states[1]) lower_conducting.push_back(2);  // T2 -> Phase C

    if (lower_conducting.size() > 1) {
        // Multiple phases connected to negative rail -> voltage notch
        // Terminal voltages collapse to average (phases shorted through DC link)
        voltage_t v_avg{0.0};
        for (int phase : lower_conducting) {
            v_avg = voltage_t(v_avg.value() + v_ac_abc[phase].value());
        }
        v_avg = voltage_t(v_avg.value() / lower_conducting.size());

        // Set all commutating phases to average voltage
        for (int phase : lower_conducting) {
            _v_phase_abc[phase] = v_avg;
        }
    }

    // Calculate line-to-line voltages from actual terminal phase voltages
    // These will show voltage notches during commutation
    _v_line_abc.a() = _v_phase_abc.a() - _v_phase_abc.b();  // V_ab
    _v_line_abc.b() = _v_phase_abc.b() - _v_phase_abc.c();  // V_bc
    _v_line_abc.c() = _v_phase_abc.c() - _v_phase_abc.a();  // V_ca
}

void thyristor_bridge::force_natural_turn_off(const abc<current_t>& i_ac_abc, current_t i_dc)
{
    auto is_anode_cathode_voltage_negative = [&](std::size_t idx) -> bool {
        return (_anode_voltages[idx].value() - _cathode_voltages[idx].value()) <= 0.0;
    };

    // For RECTIFIER mode: Line commutation (not load commutation)
    // A thyristor naturally turns off when another thyristor in the SAME leg turns on
    // and the AC voltage favors the new thyristor, causing current transfer

    // Upper leg (T1, T3, T5): Only ONE should conduct at a time
    // Turn off all except the one with highest positive current
    int upper_conducting_count = 0;
    int upper_max_idx = -1;
    real_t upper_max_current = -1e9;

    if (_conducting_states[0] && i_ac_abc.a().value() > upper_max_current) {
        upper_max_idx = 0;
        upper_max_current = i_ac_abc.a().value();
    }
    if (_conducting_states[2] && i_ac_abc.b().value() > upper_max_current) {
        upper_max_idx = 2;
        upper_max_current = i_ac_abc.b().value();
    }
    if (_conducting_states[4] && i_ac_abc.c().value() > upper_max_current) {
        upper_max_idx = 4;
        upper_max_current = i_ac_abc.c().value();
    }

    // Turn off upper thyristors that aren't carrying maximum current
    // Only turn off if current is low AND it is reverse biased (or commutation complete)
    if (_conducting_states[0] && 0 != upper_max_idx && i_ac_abc.a().value() < i_dc.value() * 0.1 &&
        is_anode_cathode_voltage_negative(0)) {
        _conducting_states[0] = false;
    }
    if (_conducting_states[2] && 2 != upper_max_idx && i_ac_abc.b().value() < i_dc.value() * 0.1 &&
        is_anode_cathode_voltage_negative(2)) {
        _conducting_states[2] = false;
    }
    if (_conducting_states[4] && 4 != upper_max_idx && i_ac_abc.c().value() < i_dc.value() * 0.1 &&
        is_anode_cathode_voltage_negative(4)) {
        _conducting_states[4] = false;
    }

    // Absolute turn-off if current is negative (reverse blocking)
    // Upper thyristors source positive current. If current < 0, they must turn off.
    if (_conducting_states[0] && i_ac_abc.a().value() < -1e-3) _conducting_states[0] = false;
    if (_conducting_states[2] && i_ac_abc.b().value() < -1e-3) _conducting_states[2] = false;
    if (_conducting_states[4] && i_ac_abc.c().value() < -1e-3) _conducting_states[4] = false;

    // Lower leg (T2, T4, T6): Only ONE should conduct at a time
    // Turn off all except the one with highest negative current (most negative)
    int lower_max_idx = -1;
    real_t lower_max_current = 1e9;  // Looking for most negative

    if (_conducting_states[3] && i_ac_abc.a().value() < lower_max_current) {
        lower_max_idx = 3;
        lower_max_current = i_ac_abc.a().value();
    }
    if (_conducting_states[5] && i_ac_abc.b().value() < lower_max_current) {
        lower_max_idx = 5;
        lower_max_current = i_ac_abc.b().value();
    }
    if (_conducting_states[1] && i_ac_abc.c().value() < lower_max_current) {
        lower_max_idx = 1;
        lower_max_current = i_ac_abc.c().value();
    }

    // Turn off lower thyristors that aren't carrying maximum current
    if (_conducting_states[3] && 3 != lower_max_idx && i_ac_abc.a().value() > -i_dc.value() * 0.1 &&
        is_anode_cathode_voltage_negative(3)) {
        _conducting_states[3] = false;
    }
    if (_conducting_states[5] && 5 != lower_max_idx && i_ac_abc.b().value() > -i_dc.value() * 0.1 &&
        is_anode_cathode_voltage_negative(5)) {
        _conducting_states[5] = false;
    }
    if (_conducting_states[1] && 1 != lower_max_idx && i_ac_abc.c().value() > -i_dc.value() * 0.1 &&
        is_anode_cathode_voltage_negative(1)) {
        _conducting_states[1] = false;
    }

    // Absolute turn-off if current is positive (reverse blocking for lower thyristors)
    // Lower thyristors sink current (negative). If current > 0, they must turn off.
    if (_conducting_states[3] && i_ac_abc.a().value() > 1e-3) _conducting_states[3] = false;
    if (_conducting_states[5] && i_ac_abc.b().value() > 1e-3) _conducting_states[5] = false;
    if (_conducting_states[1] && i_ac_abc.c().value() > 1e-3) _conducting_states[1] = false;

    // Note: We do NOT force turn-off just because i_dc is zero.
    // This allows the bridge to start conducting from zero current (start-up).
    // Thyristors will turn off if they become reverse biased (checked above).
}

abc<current_t> thyristor_bridge::calculate_ac_currents(current_t i_dc_fixed) const
{
    abc<current_t> i_ac{current_t{0.0}, current_t{0.0}, current_t{0.0}};

    // Count conducting thyristors in each leg
    int upper_count =
        (_conducting_states[0] ? 1 : 0) + (_conducting_states[2] ? 1 : 0) + (_conducting_states[4] ? 1 : 0);
    int lower_count =
        (_conducting_states[3] ? 1 : 0) + (_conducting_states[5] ? 1 : 0) + (_conducting_states[1] ? 1 : 0);

    // During commutation overlap, current is shared between conducting thyristors
    // This provides a simple first-order approximation of commutation
    current_t upper_share =
        (upper_count > 0) ? current_t(i_dc_fixed.value() / static_cast<real_t>(upper_count)) : current_t{0.0};
    current_t lower_share =
        (lower_count > 0) ? current_t(i_dc_fixed.value() / static_cast<real_t>(lower_count)) : current_t{0.0};

    // Phase A: T1 (upper, index 0) sources, T4 (lower, index 3) sinks
    if (_conducting_states[0])
        i_ac.a() = current_t(i_ac.a().value() + upper_share.value());  // T1 conducting → positive current
    if (_conducting_states[3])
        i_ac.a() = current_t(i_ac.a().value() - lower_share.value());  // T4 conducting → negative current

    // Phase B: T3 (upper, index 2) sources, T6 (lower, index 5) sinks
    if (_conducting_states[2]) i_ac.b() = current_t(i_ac.b().value() + upper_share.value());  // T3 conducting
    if (_conducting_states[5]) i_ac.b() = current_t(i_ac.b().value() - lower_share.value());  // T6 conducting

    // Phase C: T5 (upper, index 4) sources, T2 (lower, index 1) sinks
    if (_conducting_states[4]) i_ac.c() = current_t(i_ac.c().value() + upper_share.value());  // T5 conducting
    if (_conducting_states[1]) i_ac.c() = current_t(i_ac.c().value() - lower_share.value());  // T2 conducting

    return i_ac;
}

voltage_t thyristor_bridge::calculate_dc_voltage(
    const abc<voltage_t>& v_ac_abc, voltage_t v_dc_positive, voltage_t v_dc_negative) const
{
    voltage_t v_positive = v_dc_negative;  // Default to negative rail
    voltage_t v_negative = v_dc_negative;

    // Find the AC voltage connected to positive DC rail (through upper thyristors)
    if (_conducting_states[0]) v_positive = max(v_positive, v_ac_abc.a());  // T1
    if (_conducting_states[2]) v_positive = max(v_positive, v_ac_abc.b());  // T3
    if (_conducting_states[4]) v_positive = max(v_positive, v_ac_abc.c());  // T5

    // Find the AC voltage connected to negative DC rail (through lower thyristors)
    if (_conducting_states[3]) v_negative = min(v_negative, v_ac_abc.a());  // T4
    if (_conducting_states[5]) v_negative = min(v_negative, v_ac_abc.b());  // T6
    if (_conducting_states[1]) v_negative = min(v_negative, v_ac_abc.c());  // T2

    return v_positive - v_negative;
}

void thyristor_bridge::update_commutation_state(
    current_t i_dc, inductance_t commutation_inductance, duration_t dt, const abc<voltage_t>& v_ac_abc)
{
    if (_commutation_incoming_thyristor_idx >= 0 && _commutation_outgoing_thyristor_idx >= 0) {
        // Determine phase voltages for incoming and outgoing thyristors
        voltage_t v_incoming{0.0};
        voltage_t v_outgoing{0.0};

        // Map thyristor index to phase voltage
        // T1 (0) -> A, T3 (2) -> B, T5 (4) -> C (upper)
        // T4 (3) -> A, T6 (5) -> B, T2 (1) -> C (lower)
        auto get_voltage = [&](int idx) -> voltage_t {
            if (idx == 0 || idx == 3) return v_ac_abc.a();
            if (idx == 2 || idx == 5) return v_ac_abc.b();
            if (idx == 4 || idx == 1) return v_ac_abc.c();
            return voltage_t{0.0};
        };

        v_incoming = get_voltage(_commutation_incoming_thyristor_idx);
        v_outgoing = get_voltage(_commutation_outgoing_thyristor_idx);

        // Sign convention: positive di_comm/dt when incoming current rises
        // Upper thyristors: current flows from phase to DC+, so V_incoming > V_outgoing drives commutation
        // Lower thyristors: current flows from DC- to phase, so V_outgoing > V_incoming drives commutation
        bool incoming_is_upper = (_commutation_incoming_thyristor_idx == 0 ||
                                  _commutation_incoming_thyristor_idx == 2 || _commutation_incoming_thyristor_idx == 4);

        voltage_t v_diff = incoming_is_upper ? (v_incoming - v_outgoing) : (v_outgoing - v_incoming);

        // Forward Euler integration: di/dt = V/L
        _commutation_current =
            current_t(_commutation_current.value() + (v_diff / commutation_inductance).value() * dt.value());

        // Clamp to valid range [0, i_dc]
        _commutation_current = current_t(std::max(real_t{0.0}, _commutation_current.value()));

        // Check for completion
        if (_commutation_current.value() >= i_dc.value()) {
            _commutation_current = i_dc;
            // Commutation complete: outgoing thyristor turns off
            _conducting_states[_commutation_outgoing_thyristor_idx] = false;

            // Reset indices
            _commutation_incoming_thyristor_idx = -1;
            _commutation_outgoing_thyristor_idx = -1;
        }
    }
    else {
        _commutation_current = current_t{0.0};
    }
}

void thyristor_bridge::update_thyristor_currents(current_t i_dc)
{
    _thyristor_currents.fill(current_t{0.0});

    current_t upper_current_used{0.0};
    current_t lower_current_used{0.0};

    // If commutation is active
    if (_commutation_incoming_thyristor_idx >= 0 && _commutation_outgoing_thyristor_idx >= 0) {
        _thyristor_currents[_commutation_incoming_thyristor_idx] = _commutation_current;
        _thyristor_currents[_commutation_outgoing_thyristor_idx] = i_dc - _commutation_current;

        // Check which leg they belong to (Even = Upper, Odd = Lower)
        bool incoming_upper = (_commutation_incoming_thyristor_idx % 2 == 0);
        if (incoming_upper)
            upper_current_used = i_dc;
        else
            lower_current_used = i_dc;
    }

    // Distribute remaining current to others
    int upper_others = 0;
    int lower_others = 0;
    for (int i = 0; i < 6; ++i) {
        if (_conducting_states[i]) {
            if (i == _commutation_incoming_thyristor_idx || i == _commutation_outgoing_thyristor_idx) continue;
            if (i % 2 == 0)
                upper_others++;
            else
                lower_others++;
        }
    }

    current_t upper_remaining = i_dc - upper_current_used;
    current_t lower_remaining = i_dc - lower_current_used;

    // Avoid division by zero and negative currents (if commutation overshot)
    if (upper_remaining.value() < 0) upper_remaining = current_t{0.0};
    if (lower_remaining.value() < 0) lower_remaining = current_t{0.0};

    if (upper_others > 0) {
        current_t share = current_t(upper_remaining.value() / upper_others);
        for (int i = 0; i < 6; i += 2) {  // 0, 2, 4
            if (_conducting_states[i] && i != _commutation_incoming_thyristor_idx &&
                i != _commutation_outgoing_thyristor_idx) {
                _thyristor_currents[i] = share;
            }
        }
    }

    if (lower_others > 0) {
        current_t share = current_t(lower_remaining.value() / lower_others);
        for (int i = 1; i < 6; i += 2) {  // 1, 3, 5
            if (_conducting_states[i] && i != _commutation_incoming_thyristor_idx &&
                i != _commutation_outgoing_thyristor_idx) {
                _thyristor_currents[i] = share;
            }
        }
    }
}

voltage_t thyristor_bridge::get_instantaneous_dc_voltage(const abc<voltage_t>& v_ac_abc) const
{
    voltage_t v_pos{0.0};
    voltage_t v_neg{0.0};
    int upper_count = 0;
    int lower_count = 0;

    // Upper rail (T1, T3, T5)
    if (_conducting_states[0]) {
        v_pos = voltage_t(v_pos.value() + v_ac_abc.a().value());
        upper_count++;
    }
    if (_conducting_states[2]) {
        v_pos = voltage_t(v_pos.value() + v_ac_abc.b().value());
        upper_count++;
    }
    if (_conducting_states[4]) {
        v_pos = voltage_t(v_pos.value() + v_ac_abc.c().value());
        upper_count++;
    }

    // Lower rail (T4, T6, T2)
    if (_conducting_states[3]) {
        v_neg = voltage_t(v_neg.value() + v_ac_abc.a().value());
        lower_count++;
    }
    if (_conducting_states[5]) {
        v_neg = voltage_t(v_neg.value() + v_ac_abc.b().value());
        lower_count++;
    }
    if (_conducting_states[1]) {
        v_neg = voltage_t(v_neg.value() + v_ac_abc.c().value());
        lower_count++;
    }

    if (upper_count > 0) v_pos = voltage_t(v_pos.value() / upper_count);
    if (lower_count > 0) v_neg = voltage_t(v_neg.value() / lower_count);

    return v_pos - v_neg;
}

}  // namespace yeet::control