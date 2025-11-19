#pragma once

#include <array>

#include "yeet_control/reference_frame.hpp"
#include "yeet_control/thyristor_bridge.hpp"

namespace yeet::control {

/**
 * @brief Complete dynamic model of a back-to-back thyristor LCI system.
 *
 * Models a three-phase grid → rectifier bridge → DC link → inverter bridge → three-phase machine load.
 *
 * System topology:
 *   Grid_abc → [R_grid, L_grid] → Rectifier Bridge → [R_dc, L_dc] → Inverter Bridge → [R_mach, L_mach] → Machine_abc
 *
 * State variables (integrated over time):
 *   - i_dc: DC link current
 *   - i_comm_rect: Current through incoming rectifier thyristor during commutation
 *   - i_comm_inv: Current through incoming inverter thyristor during commutation
 *
 * The DC link equation couples both bridges:
 *   L_total * di_dc/dt = V_rect - V_inv - R_total * i_dc
 *
 * where:
 *   L_total = 2*L_grid + L_dc + 2*L_mach
 *   R_total = 2*R_grid + R_dc + 2*R_mach
 *
 * Each bridge handles commutation independently with its respective AC-side inductance.
 */
class thyristor_back_to_back {
public:
    /**
     * @brief Constructs the back-to-back thyristor LCI model.
     * @param grid_res Grid equivalent series resistance per phase [Ohms]
     * @param grid_ind Grid equivalent series inductance per phase [Henry]
     * @param dc_res DC link resistance [Ohms]
     * @param dc_ind DC link inductance [Henry]
     * @param mach_res Machine equivalent series resistance per phase [Ohms]
     * @param mach_ind Machine equivalent series inductance per phase [Henry]
     * @param ts Integration time step [seconds]
     */
    thyristor_back_to_back(
        double grid_res, double grid_ind, double dc_res, double dc_ind, double mach_res, double mach_ind, double ts);

    /**
     * @brief Updates the complete LCI system for one time step.
     *
     * Performs RK4 integration of:
     *   - DC link equation coupling both bridges
     *   - Rectifier commutation overlap equation
     *   - Inverter commutation overlap equation
     *
     * @param v_grid Grid source voltages (line-to-neutral) [V]
     * @param v_machine Machine terminal voltages (EMF, line-to-neutral) [V]
     * @param p_rect Gate signals for rectifier thyristors T1-T6
     * @param p_inv Gate signals for inverter thyristors T1-T6
     */
    void update(const abc<double>& v_grid,
        const abc<double>& v_machine,
        const std::array<bool, 6>& p_rect,
        const std::array<bool, 6>& p_inv);

    /**
     * @brief Resets all state variables to zero.
     */
    void reset();

    /**
     * @brief Gets the three-phase grid currents.
     * @return Grid currents in abc frame [A]
     */
    abc<double> get_grid_currents() const;

    /**
     * @brief Gets the three-phase machine currents.
     * @return Machine currents in abc frame [A]
     */
    abc<double> get_machine_currents() const;

    /**
     * @brief Gets the DC link current.
     * @return DC current [A]
     */
    double get_dc_current() const noexcept { return _i_dc; }

    /**
     * @brief Gets the rectifier DC output voltage.
     * @return Rectifier DC voltage [V]
     */
    double get_rectifier_voltage() const noexcept { return _v_rect_dc; }

    /**
     * @brief Gets the inverter DC input voltage.
     * @return Inverter DC voltage [V]
     */
    double get_inverter_voltage() const noexcept { return _v_inv_dc; }

    /**
     * @brief Gets the rectifier bridge conduction states.
     * @return Array of 6 booleans (true = conducting)
     */
    const std::array<bool, 6>& get_rectifier_states() const noexcept { return _rect_bridge.get_conduction_states(); }

    /**
     * @brief Gets the inverter bridge conduction states.
     * @return Array of 6 booleans (true = conducting)
     */
    const std::array<bool, 6>& get_inverter_states() const noexcept { return _inv_bridge.get_conduction_states(); }

    /**
     * @brief Gets individual rectifier thyristor currents.
     * @return Array of 6 currents for T1-T6 [A]
     */
    const std::array<double, 6>& get_rectifier_thyristor_currents() const noexcept { return _i_thy_rect; }

    /**
     * @brief Gets individual inverter thyristor currents.
     * @return Array of 6 currents for T1-T6 [A]
     */
    const std::array<double, 6>& get_inverter_thyristor_currents() const noexcept { return _i_thy_inv; }

    /**
     * @brief Gets the line-to-line voltages at the grid-side AC terminals.
     * @return Array [v_ab, v_bc, v_ca] at rectifier AC terminals [V]
     */
    const std::array<double, 3>& get_grid_line_voltages() const noexcept { return _rect_bridge.get_ac_line_voltages(); }

    /**
     * @brief Gets the line-to-line voltages at the machine-side AC terminals.
     * @return Array [v_ab, v_bc, v_ca] at inverter AC terminals [V]
     */
    const std::array<double, 3>& get_machine_line_voltages() const noexcept
    {
        return _inv_bridge.get_ac_line_voltages();
    }

private:
    // Circuit parameters
    double _grid_resistance;
    double _grid_inductance;
    double _dc_resistance;
    double _dc_inductance;
    double _machine_resistance;
    double _machine_inductance;
    double _sampling_time;

    // State variables
    double _i_dc{0.0};         // DC link current [A]
    double _i_comm_rect{0.0};  // Rectifier commutating current [A]
    double _i_comm_inv{0.0};   // Inverter commutating current [A]

    // Rectifier commutation tracking
    int _outgoing_rect{-1};  // Outgoing rectifier thyristor index (0-5), -1 if no commutation
    int _incoming_rect{-1};  // Incoming rectifier thyristor index (0-5), -1 if no commutation

    // Inverter commutation tracking
    int _outgoing_inv{-1};  // Outgoing inverter thyristor index (0-5), -1 if no commutation
    int _incoming_inv{-1};  // Incoming inverter thyristor index (0-5), -1 if no commutation

    // Cached outputs
    mutable double _v_rect_dc{0.0};                                   // Rectifier DC voltage [V]
    mutable double _v_inv_dc{0.0};                                    // Inverter DC voltage [V]
    std::array<double, 6> _i_thy_rect{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};  // Rectifier thyristor currents [A]
    std::array<double, 6> _i_thy_inv{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};   // Inverter thyristor currents [A]

    // Internal components
    thyristor_bridge _rect_bridge;
    thyristor_bridge _inv_bridge;

    // Internal state structure for RK4 integration
    struct State {
        double i_dc;         // DC link current
        double i_comm_rect;  // Rectifier commutating current
        double i_comm_inv;   // Inverter commutating current

        State operator+(const State& other) const
        {
            return {i_dc + other.i_dc, i_comm_rect + other.i_comm_rect, i_comm_inv + other.i_comm_inv};
        }

        State operator*(double scalar) const { return {i_dc * scalar, i_comm_rect * scalar, i_comm_inv * scalar}; }
    };

    // RK4 and dynamics methods
    State calculate_derivatives(const State& state, const abc<double>& v_grid, const abc<double>& v_machine) const;

    // Bridge conduction update methods
    void update_rectifier_conduction(const abc<double>& v_grid, const std::array<bool, 6>& pulses);
    void update_inverter_conduction(const abc<double>& v_machine, const std::array<bool, 6>& pulses);

    // Commutation detection methods
    void detect_rectifier_commutation();
    void detect_inverter_commutation();

    // Current distribution
    void update_thyristor_currents();
};

}  // namespace yeet::control