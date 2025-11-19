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
 *   - i_dc: DC current through the bridge and load
 *
 * Note: AC grid currents are determined by i_dc and thyristor conduction states,
 * not separate state variables. Total inductance = L_grid/2 + L_load (series connection).
 *
 * Architecture:
 *   - Call update() each time step with AC voltages, gate pulses, and DC back-EMF
 *   - Internal Runge-Kutta 4th order integration for accurate dynamics
 *   - Natural commutation handled automatically based on current reversal
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
     *   - Grid currents: L_grid * di_grid/dt = v_source - R_grid*i_grid - v_bridge_ac
     *   - DC current: L_load * di_dc/dt = v_bridge_dc - R_load*i_dc - v_dc_back_emf
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
     * @brief Gets the DC load current.
     * @return DC current [A]
     */
    double get_dc_current() const noexcept { return _i_dc; }

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

private:
    // Circuit parameters
    double _grid_resistance;
    double _grid_inductance;
    double _load_resistance;
    double _load_inductance;
    double _sampling_time;

    // State variables
    double _i_dc{0.0};  // DC current through bridge and load [A]

    // Cached outputs
    double _v_dc_output{0.0};  // DC voltage from bridge [V]

    // Internal components
    thyristor_bridge _bridge;

    // Internal state structure for RK4 integration
    struct State {
        double i_dc;

        State operator+(const State& other) const { return {i_dc + other.i_dc}; }

        State operator*(double scalar) const { return {i_dc * scalar}; }
    };

    // RK4 helper methods
    State calculate_derivatives(const State& state, const abc<double>& v_ac_source, double v_dc_back_emf);
    void update_bridge_conduction(const abc<double>& v_ac_source, const std::array<bool, 6>& pulses);
};

}  // namespace yeet::control
