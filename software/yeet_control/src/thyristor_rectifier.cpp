#include "yeet_control/thyristor_rectifier.hpp"

#include <algorithm>

namespace yeet::control {

thyristor_rectifier::thyristor_rectifier(double grid_resistance,
    double grid_inductance,
    double load_resistance,
    double load_inductance,
    double sampling_time)
    : _grid_resistance(grid_resistance),
      _grid_inductance(grid_inductance),
      _load_resistance(load_resistance),
      _load_inductance(load_inductance),
      _sampling_time(sampling_time)
{
}

void thyristor_rectifier::update(
    const abc<double>& v_ac_source, const std::array<bool, 6>& pulses, double v_dc_back_emf)
{
    // Step 1: Update thyristor turn-on based on gate pulses and forward bias
    update_bridge_conduction(v_ac_source, pulses);

    // Step 2: Detect commutation (new thyristor turning on)
    detect_commutation();

    // Step 3: RK4 integration of DC link and commutating currents
    State current_state = {_i_dc, _i_commutating};
    auto k1 = calculate_derivatives(current_state, v_ac_source, v_dc_back_emf);
    auto k2 = calculate_derivatives(current_state + k1 * (_sampling_time * 0.5), v_ac_source, v_dc_back_emf);
    auto k3 = calculate_derivatives(current_state + k2 * (_sampling_time * 0.5), v_ac_source, v_dc_back_emf);
    auto k4 = calculate_derivatives(current_state + k3 * _sampling_time, v_ac_source, v_dc_back_emf);

    State new_state = current_state + (k1 + k2 * 2.0 + k3 * 2.0 + k4) * (_sampling_time / 6.0);

    // Apply new state - clamp currents to non-negative values
    _i_dc = std::max(0.0, new_state.i_dc);
    _i_commutating = std::max(0.0, new_state.i_commutating);

    // Step 4: If commutation is complete (outgoing current ≈ 0), reset commutation state
    if (_outgoing_thyristor >= 0 && _incoming_thyristor >= 0) {
        double i_outgoing = _i_dc - _i_commutating;
        if (i_outgoing < 1e-6 || _i_commutating >= _i_dc * 0.999) {
            // Commutation complete - incoming thyristor now carries full DC current
            _i_commutating = 0.0;
            _outgoing_thyristor = -1;
            _incoming_thyristor = -1;
        }
    }

    // Step 5: Compute individual thyristor currents from i_dc and i_commutating
    update_thyristor_currents();

    // Step 6: Natural turn-off - thyristors turn off when current drops to zero
    abc<double> i_grid_currents = get_grid_currents();
    std::array<double, 3> i_ac_array = {i_grid_currents.a(), i_grid_currents.b(), i_grid_currents.c()};
    double i_dc = get_dc_current();
    _bridge.force_natural_turn_off(i_ac_array, i_dc);
}

void thyristor_rectifier::reset()
{
    _i_dc = 0.0;
    _i_commutating = 0.0;
    _i_thyristor.fill(0.0);
    _v_dc_output = 0.0;
    _outgoing_thyristor = -1;
    _incoming_thyristor = -1;
    _bridge.reset();
}

abc<double> thyristor_rectifier::get_grid_currents() const noexcept
{
    // Phase currents are the difference between upper and lower thyristor currents
    // Phase A: i_a = i_T1 - i_T4
    // Phase B: i_b = i_T3 - i_T6
    // Phase C: i_c = i_T5 - i_T2
    double i_a = _i_thyristor[0] - _i_thyristor[3];
    double i_b = _i_thyristor[2] - _i_thyristor[5];
    double i_c = _i_thyristor[4] - _i_thyristor[1];
    return abc<double>(i_a, i_b, i_c);
}

double thyristor_rectifier::get_dc_current() const noexcept
{
    // DC current is the main state variable
    return _i_dc;
}

thyristor_rectifier::State thyristor_rectifier::calculate_derivatives(
    const State& state, const abc<double>& v_ac_source, double v_dc_back_emf)
{
    const auto& conducting = _bridge.get_conduction_states();
    State derivatives{0.0, 0.0};

    // Calculate phase currents from thyristor currents (from cached _i_thyristor)
    double i_a = _i_thyristor[0] - _i_thyristor[3];  // i_T1 - i_T4
    double i_b = _i_thyristor[2] - _i_thyristor[5];  // i_T3 - i_T6
    double i_c = _i_thyristor[4] - _i_thyristor[1];  // i_T5 - i_T2

    // AC terminal voltages after grid impedance drops
    double v_term_a = v_ac_source.a() - _grid_resistance * i_a;
    double v_term_b = v_ac_source.b() - _grid_resistance * i_b;
    double v_term_c = v_ac_source.c() - _grid_resistance * i_c;

    // Find which phase voltages are active (conducting thyristors)
    std::array<double, 3> v_phases = {v_term_a, v_term_b, v_term_c};
    double v_upper = -1e9;  // DC+ rail voltage
    double v_lower = 1e9;   // DC- rail voltage
    int upper_phase = -1;   // Phase index for upper thyristor
    int lower_phase = -1;   // Phase index for lower thyristor

    // Find conducting upper thyristor (T1=phase A, T3=phase B, T5=phase C)
    if (conducting[0]) {
        v_upper = v_term_a;
        upper_phase = 0;
    }  // T1 on Phase A
    if (conducting[2]) {
        v_upper = v_term_b;
        upper_phase = 1;
    }  // T3 on Phase B
    if (conducting[4]) {
        v_upper = v_term_c;
        upper_phase = 2;
    }  // T5 on Phase C

    // Find conducting lower thyristor (T4=phase A, T6=phase B, T2=phase C)
    if (conducting[3]) {
        v_lower = v_term_a;
        lower_phase = 0;
    }  // T4 on Phase A
    if (conducting[5]) {
        v_lower = v_term_b;
        lower_phase = 1;
    }  // T6 on Phase B
    if (conducting[1]) {
        v_lower = v_term_c;
        lower_phase = 2;
    }  // T2 on Phase C

    // Calculate DC link voltage output (for telemetry)
    double v_dc_link = (upper_phase >= 0 && lower_phase >= 0) ? (v_upper - v_lower) : 0.0;
    _v_dc_output = v_dc_link;

    // Total series inductance and resistance
    double L_total = 2.0 * _grid_inductance + _load_inductance;
    double R_total = 2.0 * _grid_resistance + _load_resistance;

    // === DC Link Current Differential Equation ===
    // L_total * di_dc/dt = V_line_to_line - R_total*i_dc - V_back_emf
    if (upper_phase >= 0 && lower_phase >= 0) {
        double v_line_to_line = v_upper - v_lower;
        derivatives.i_dc = (v_line_to_line - R_total * state.i_dc - v_dc_back_emf) / L_total;
    }
    else {
        derivatives.i_dc = 0.0;  // No conduction path
    }

    // === Commutation Current Differential Equation ===
    // During commutation overlap: L_comm * di_comm/dt = (V_incoming - V_outgoing)
    // L_comm is the commutation inductance (typically grid inductance)
    if (_incoming_thyristor >= 0 && _outgoing_thyristor >= 0) {
        // Determine phase voltages for incoming and outgoing thyristors
        double v_incoming = 0.0;
        double v_outgoing = 0.0;

        // Map thyristor index to phase voltage
        // T1 (0) → A, T3 (2) → B, T5 (4) → C (upper thyristors)
        // T4 (3) → A, T6 (5) → B, T2 (1) → C (lower thyristors)
        if (_incoming_thyristor == 0 || _incoming_thyristor == 3)
            v_incoming = v_term_a;
        else if (_incoming_thyristor == 2 || _incoming_thyristor == 5)
            v_incoming = v_term_b;
        else if (_incoming_thyristor == 4 || _incoming_thyristor == 1)
            v_incoming = v_term_c;

        if (_outgoing_thyristor == 0 || _outgoing_thyristor == 3)
            v_outgoing = v_term_a;
        else if (_outgoing_thyristor == 2 || _outgoing_thyristor == 5)
            v_outgoing = v_term_b;
        else if (_outgoing_thyristor == 4 || _outgoing_thyristor == 1)
            v_outgoing = v_term_c;

        // Sign convention: positive di_comm/dt when incoming current rises
        // Upper thyristors: current flows from phase to DC+, so V_incoming > V_outgoing drives commutation
        // Lower thyristors: current flows from DC- to phase, so V_outgoing > V_incoming drives commutation
        bool incoming_is_upper = (_incoming_thyristor == 0 || _incoming_thyristor == 2 || _incoming_thyristor == 4);

        double v_diff = incoming_is_upper ? (v_incoming - v_outgoing) : (v_outgoing - v_incoming);

        double commutation_inductance = 2.0f * _grid_inductance;
        derivatives.i_commutating = v_diff / commutation_inductance;
    }
    else {
        derivatives.i_commutating = 0.0;  // No active commutation
    }

    return derivatives;
}

void thyristor_rectifier::update_bridge_conduction(const abc<double>& v_ac_source, const std::array<bool, 6>& pulses)
{
    // Calculate phase currents from thyristor currents
    double i_a = _i_thyristor[0] - _i_thyristor[3];
    double i_b = _i_thyristor[2] - _i_thyristor[5];
    double i_c = _i_thyristor[4] - _i_thyristor[1];

    // Estimate AC terminal voltages after grid impedance
    std::array<double, 3> v_ac_terminals = {v_ac_source.a() - _grid_resistance * i_a,
        v_ac_source.b() - _grid_resistance * i_b, v_ac_source.c() - _grid_resistance * i_c};

    // Estimate DC rail voltages for forward bias check
    // When no thyristors conducting (_i_dc ≈ 0), use conservative estimate (rails at 0V)
    // This allows initial turn-on when gate pulses arrive
    // When thyristors conducting, estimate rails from DC link voltage
    double v_p_rail = 0.0;
    double v_n_rail = 0.0;

    if (_i_dc > 1e-6) {
        // Thyristors are conducting - estimate rail voltages from DC output
        // This is the voltage across the DC link (positive rail - negative rail)
        v_p_rail = _v_dc_output * 0.5;  // Assume symmetric around zero
        v_n_rail = -_v_dc_output * 0.5;
    }

    // Update thyristor conduction
    _bridge.update_conduction_state(v_ac_terminals, {v_p_rail, v_n_rail}, pulses);
}

void thyristor_rectifier::detect_commutation()
{
    // Detect when a new thyristor turns on while another is conducting
    // This indicates the start of commutation overlap
    const auto& conducting = _bridge.get_conduction_states();
    static std::array<bool, 6> prev_conducting = {false, false, false, false, false, false};

    // Check each thyristor for a transition from OFF → ON
    for (int i = 0; i < 6; ++i) {
        if (conducting[i] && !prev_conducting[i]) {
            // Thyristor i just turned on
            // Determine if this is a commutation event (replacing another thyristor on same rail)
            bool is_upper_thyristor = (i == 0 || i == 2 || i == 4);  // T1, T3, T5

            if (is_upper_thyristor) {
                // Find previously conducting upper thyristor
                for (int j = 0; j < 6; j += 2) {  // Check T1 (0), T3 (2), T5 (4)
                    if (j != i && prev_conducting[j] && conducting[j]) {
                        // Commutation from T_j to T_i
                        _outgoing_thyristor = j;
                        _incoming_thyristor = i;
                        _i_commutating = 0.0;  // Reset commutation current
                        break;
                    }
                }
            }
            else {
                // Find previously conducting lower thyristor
                int lower_indices[] = {3, 5, 1};  // T4, T6, T2
                for (int j : lower_indices) {
                    if (j != i && prev_conducting[j] && conducting[j]) {
                        // Commutation from T_j to T_i
                        _outgoing_thyristor = j;
                        _incoming_thyristor = i;
                        _i_commutating = 0.0;  // Reset commutation current
                        break;
                    }
                }
            }
        }
    }

    // Update previous state
    prev_conducting = conducting;
}

void thyristor_rectifier::update_thyristor_currents()
{
    // Compute individual thyristor currents from i_dc and i_commutating
    const auto& conducting = _bridge.get_conduction_states();

    // Initialize all currents to zero
    _i_thyristor.fill(0.0);

    if (_outgoing_thyristor >= 0 && _incoming_thyristor >= 0) {
        // Commutation in progress
        // Incoming thyristor current rises: 0 → i_dc
        // Outgoing thyristor current falls: i_dc → 0
        _i_thyristor[_incoming_thyristor] = _i_commutating;
        _i_thyristor[_outgoing_thyristor] = _i_dc - _i_commutating;

        // Find the conducting thyristor on the opposite rail (not commutating)
        bool incoming_is_upper = (_incoming_thyristor == 0 || _incoming_thyristor == 2 || _incoming_thyristor == 4);
        if (incoming_is_upper) {
            // Commutation on upper rail, find lower thyristor
            for (int i : {3, 5, 1}) {  // T4, T6, T2
                if (conducting[i]) {
                    _i_thyristor[i] = _i_dc;
                    break;
                }
            }
        }
        else {
            // Commutation on lower rail, find upper thyristor
            for (int i : {0, 2, 4}) {  // T1, T3, T5
                if (conducting[i]) {
                    _i_thyristor[i] = _i_dc;
                    break;
                }
            }
        }
    }
    else {
        // No commutation - normal operation
        // One upper and one lower thyristor conducting, both carry i_dc
        for (int i = 0; i < 6; ++i) {
            if (conducting[i]) {
                _i_thyristor[i] = _i_dc;
            }
        }
    }
}

}  // namespace yeet::control
