#include "yeet_control/thyristor_bridge.hpp"

void thyristor_bridge::update_conduction_state(
    const std::array<double, 3>& v_ac_abc, const std::array<double, 2>& v_dc_rails, const std::array<bool, 6>& pulses)
{
    // Correct physical connections for a standard 6-pulse bridge
    const std::array<double, 6> anode_voltages = {
        v_ac_abc[0],    // T1 anode is on Phase A
        v_dc_rails[1],  // T2 anode is on Negative DC Rail
        v_ac_abc[1],    // T3 anode is on Phase B
        v_dc_rails[1],  // T4 anode is on Negative DC Rail
        v_ac_abc[2],    // T5 anode is on Phase C
        v_dc_rails[1]   // T6 anode is on Negative DC Rail
    };

    const std::array<double, 6> cathode_voltages = {
        v_dc_rails[0],  // T1 cathode is on Positive DC Rail
        v_ac_abc[2],    // T2 cathode is on Phase C
        v_dc_rails[0],  // T3 cathode is on Positive DC Rail
        v_ac_abc[0],    // T4 cathode is on Phase A
        v_dc_rails[0],  // T5 cathode is on Positive DC Rail
        v_ac_abc[1]     // T6 cathode is on Phase B
    };

    for (int i = 0; i < 6; ++i) {
        if (!_conducting_states_[i]) {
            bool has_gate_pulse = pulses[i];
            bool is_forward_biased = (anode_voltages[i] > cathode_voltages[i]);
            if (has_gate_pulse && is_forward_biased) {
                _conducting_states_[i] = true;
            }
        }
    }
}

void thyristor_bridge::force_natural_turn_off(const std::array<double, 3>& i_ac_abc, double i_dc)
{
    // For RECTIFIER mode: Line commutation (not load commutation)
    // A thyristor naturally turns off when another thyristor in the SAME leg turns on
    // and the AC voltage favors the new thyristor, causing current transfer

    // Upper leg (T1, T3, T5): Only ONE should conduct at a time
    // Turn off all except the one with highest positive current
    int upper_conducting_count = 0;
    int upper_max_idx = -1;
    double upper_max_current = -1e9;

    if (_conducting_states_[0] && i_ac_abc[0] > upper_max_current) {
        upper_max_idx = 0;
        upper_max_current = i_ac_abc[0];
    }
    if (_conducting_states_[2] && i_ac_abc[1] > upper_max_current) {
        upper_max_idx = 2;
        upper_max_current = i_ac_abc[1];
    }
    if (_conducting_states_[4] && i_ac_abc[2] > upper_max_current) {
        upper_max_idx = 4;
        upper_max_current = i_ac_abc[2];
    }

    // Turn off upper thyristors that aren't carrying maximum current
    if (_conducting_states_[0] && 0 != upper_max_idx && i_ac_abc[0] < i_dc * 0.1) _conducting_states_[0] = false;
    if (_conducting_states_[2] && 2 != upper_max_idx && i_ac_abc[1] < i_dc * 0.1) _conducting_states_[2] = false;
    if (_conducting_states_[4] && 4 != upper_max_idx && i_ac_abc[2] < i_dc * 0.1) _conducting_states_[4] = false;

    // Lower leg (T2, T4, T6): Only ONE should conduct at a time
    // Turn off all except the one with highest negative current (most negative)
    int lower_max_idx = -1;
    double lower_max_current = 1e9;  // Looking for most negative

    if (_conducting_states_[3] && i_ac_abc[0] < lower_max_current) {
        lower_max_idx = 3;
        lower_max_current = i_ac_abc[0];
    }
    if (_conducting_states_[5] && i_ac_abc[1] < lower_max_current) {
        lower_max_idx = 5;
        lower_max_current = i_ac_abc[1];
    }
    if (_conducting_states_[1] && i_ac_abc[2] < lower_max_current) {
        lower_max_idx = 1;
        lower_max_current = i_ac_abc[2];
    }

    // Turn off lower thyristors that aren't carrying maximum current
    if (_conducting_states_[3] && 3 != lower_max_idx && i_ac_abc[0] > -i_dc * 0.1) _conducting_states_[3] = false;
    if (_conducting_states_[5] && 5 != lower_max_idx && i_ac_abc[1] > -i_dc * 0.1) _conducting_states_[5] = false;
    if (_conducting_states_[1] && 1 != lower_max_idx && i_ac_abc[2] > -i_dc * 0.1) _conducting_states_[1] = false;

    // If DC current ceases, all thyristors must turn off
    if (i_dc <= 1e-6) {
        _conducting_states_.fill(false);
    }
}

std::array<double, 3> thyristor_bridge::calculate_ac_currents(double i_dc_fixed) const
{
    std::array<double, 3> i_ac = {0.0, 0.0, 0.0};

    // Count conducting thyristors in each leg
    int upper_count =
        (_conducting_states_[0] ? 1 : 0) + (_conducting_states_[2] ? 1 : 0) + (_conducting_states_[4] ? 1 : 0);
    int lower_count =
        (_conducting_states_[3] ? 1 : 0) + (_conducting_states_[5] ? 1 : 0) + (_conducting_states_[1] ? 1 : 0);

    // During commutation overlap, current is shared between conducting thyristors
    // This provides a simple first-order approximation of commutation
    double upper_share = (upper_count > 0) ? (i_dc_fixed / static_cast<double>(upper_count)) : 0.0;
    double lower_share = (lower_count > 0) ? (i_dc_fixed / static_cast<double>(lower_count)) : 0.0;

    // Phase A: T1 (upper, index 0) sources, T4 (lower, index 3) sinks
    if (_conducting_states_[0]) i_ac[0] += upper_share;  // T1 conducting → positive current
    if (_conducting_states_[3]) i_ac[0] -= lower_share;  // T4 conducting → negative current

    // Phase B: T3 (upper, index 2) sources, T6 (lower, index 5) sinks
    if (_conducting_states_[2]) i_ac[1] += upper_share;  // T3 conducting
    if (_conducting_states_[5]) i_ac[1] -= lower_share;  // T6 conducting

    // Phase C: T5 (upper, index 4) sources, T2 (lower, index 1) sinks
    if (_conducting_states_[4]) i_ac[2] += upper_share;  // T5 conducting
    if (_conducting_states_[1]) i_ac[2] -= lower_share;  // T2 conducting

    return i_ac;
}

double thyristor_bridge::calculate_dc_voltage(
    const std::array<double, 3>& v_ac_abc, const std::array<double, 2>& v_dc_rails) const
{
    double v_positive = v_dc_rails[1];  // Default to negative rail
    double v_negative = v_dc_rails[1];

    // Find the AC voltage connected to positive DC rail (through upper thyristors)
    if (_conducting_states_[0]) v_positive = std::max(v_positive, v_ac_abc[0]);  // T1
    if (_conducting_states_[2]) v_positive = std::max(v_positive, v_ac_abc[1]);  // T3
    if (_conducting_states_[4]) v_positive = std::max(v_positive, v_ac_abc[2]);  // T5

    // Find the AC voltage connected to negative DC rail (through lower thyristors)
    if (_conducting_states_[3]) v_negative = std::min(v_negative, v_ac_abc[0]);  // T4
    if (_conducting_states_[5]) v_negative = std::min(v_negative, v_ac_abc[1]);  // T6
    if (_conducting_states_[1]) v_negative = std::min(v_negative, v_ac_abc[2]);  // T2

    return v_positive - v_negative;
}