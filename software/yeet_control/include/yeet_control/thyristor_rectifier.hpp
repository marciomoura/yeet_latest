#pragma once

#include "yeet_control/reference_frame.hpp"
#include "yeet_control/thyristor_bridge.hpp"

namespace yeet::control {

/**
 * @brief Complete dynamic model of a 6-pulse thyristor rectifier system.
 *
 * Models a three-phase AC voltage source → grid RL impedance → thyristor bridge → DC RL load with back-EMF.
 *
 * System topology:
 *   V_source_abc → [R_grid, L_grid] → Thyristor Bridge → [R_load, L_load, V_dc_back_emf]
 *
 * State variables (integrated over time):
 *   - i_dc: DC link current through the load
 *   - i_commutating: Current through the incoming thyristor during commutation overlap
 *
 * Commutation model:
 *   - Normally: One upper + one lower thyristor conduct, each carries i_dc
 *   - During commutation: Incoming thyristor current rises from 0 → i_dc
 *                         Outgoing thyristor current = i_dc - i_commutating
 *   - After commutation: i_commutating resets to 0, new thyristor carries full i_dc
 *
 * AC phase currents = difference between upper and lower thyristor currents in that phase.
 *
 * Architecture:
 *   - Call update() each time step with AC voltages, gate pulses, and DC back-EMF
 *   - Internal Runge-Kutta 4th order integration for accurate dynamics
 *   - Natural commutation handled automatically based on current transfer completion
 */
class thyristor_rectifier {
public:
    /**
     * @brief Constructs the thyristor rectifier model.
     * @param grid_resistance Grid equivalent series resistance per phase [Ohms]
     * @param grid_inductance Grid equivalent series inductance per phase [Henry]
     * @param load_resistance DC load resistance [Ohms]
     * @param load_inductance DC load inductance [Henry]
     * @param sampling_time Integration time step [seconds]
     */
    thyristor_rectifier(double grid_resistance,
        double grid_inductance,
        double load_resistance,
        double load_inductance,
        double sampling_time);

    /**
     * @brief Updates the rectifier model for one time step.
     *
     * Performs RK4 integration of the system differential equations:
     *   - DC link equation: L_total * di_dc/dt = V_line_to_line - R_total*i_dc - V_dc_back_emf
     *   - Commutation overlap: L_comm * di_comm/dt = (V_incoming_phase - V_outgoing_phase)
     *
     * where L_total = L_grid + L_load, R_total = R_grid + R_load
     *
     * @param v_ac_source Three-phase source voltages (line-to-neutral) [V]
     * @param pulses Gate signals for thyristors T1-T6
     * @param v_dc_back_emf DC back-voltage from load (motor EMF, battery, etc.) [V]
     */
    void update(const abc<double>& v_ac_source, const std::array<bool, 6>& pulses, double v_dc_back_emf);

    /**
     * @brief Resets all state variables to zero.
     */
    void reset();

    /**
     * @brief Gets the three-phase grid currents.
     * @return Grid currents in abc frame [A]
     */
    abc<double> get_grid_currents() const noexcept;

    /**
     * @brief Gets the DC load current (sum of upper thyristor currents).
     * @return DC current [A]
     */
    double get_dc_current() const noexcept;

    /**
     * @brief Gets the DC voltage output from the thyristor bridge.
     * @return DC voltage [V]
     */
    double get_dc_voltage() const noexcept { return _v_dc_output; }

    /**
     * @brief Gets the thyristor bridge conduction states.
     * @return Array of 6 booleans (true = conducting)
     */
    const std::array<bool, 6>& get_conduction_states() const noexcept { return _bridge.get_conduction_states(); }

    /**
     * @brief Gets individual thyristor currents.
     * @return Array of 6 currents for T1-T6 [A]
     */
    const std::array<double, 6>& get_thyristor_currents() const noexcept { return _i_thyristor; }

private:
    // Circuit parameters
    double _grid_resistance;
    double _grid_inductance;
    double _load_resistance;
    double _load_inductance;
    double _sampling_time;

    // State variables - DC link current and commutating current
    double _i_dc{0.0};           // DC link current through load [A]
    double _i_commutating{0.0};  // Incoming thyristor current during commutation [A]

    // Commutation tracking
    int _outgoing_thyristor{-1};  // Index of outgoing thyristor (0-5), -1 if no commutation
    int _incoming_thyristor{-1};  // Index of incoming thyristor (0-5), -1 if no commutation

    // Cached outputs
    double _v_dc_output{0.0};                                          // DC voltage from bridge [V]
    std::array<double, 6> _i_thyristor{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};  // Individual thyristor currents [A]

    // Internal components
    thyristor_bridge _bridge;

    // Internal state structure for RK4 integration
    struct State {
        double i_dc;           // DC link current
        double i_commutating;  // Commutating current

        State operator+(const State& other) const { return {i_dc + other.i_dc, i_commutating + other.i_commutating}; }

        State operator*(double scalar) const { return {i_dc * scalar, i_commutating * scalar}; }
    };

    // RK4 helper methods
    State calculate_derivatives(const State& state, const abc<double>& v_ac_source, double v_dc_back_emf);
    void update_bridge_conduction(const abc<double>& v_ac_source, const std::array<bool, 6>& pulses);

    // Commutation detection and current distribution
    void detect_commutation();
    void update_thyristor_currents();
};

}  // namespace yeet::control
