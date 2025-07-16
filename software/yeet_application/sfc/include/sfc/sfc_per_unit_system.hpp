#pragma once

#include <cassert>

#include "yeet_control/constants.hpp"
// Note: Per-unit and reference frame headers are likely not needed here anymore,
// as this class IS the per-unit definition.

namespace yeet::application::sfc {

/**
 * @brief Per-Unit System for Load-Commutated Inverter (LCI) Drives
 *
 * This class implements the per-unit system as defined in the LCI system
 * documentation and summary table. The system is designed around a single
 * system-wide base power, with distinct base quantities for the grid-side,
 * DC link, and machine-side.
 *
 * This structure explicitly separates the different parts of the LCI drive,
 * making the per-unit conversions clear and unambiguous.
 */
class sfc_per_unit_system {
public:
    /**
     * @brief Struct containing all per-unit base quantities for the Grid Side (LV).
     */
    struct grid_side_bases {
        double base_voltage_ll_rms_v;         ///< Base line-to-line RMS voltage [V]
        double base_voltage_peak_phase_v;     ///< Base peak phase voltage [V]
        double base_current_rms_a;            ///< Base line RMS current [A]
        double base_current_peak_phase_a;     ///< Base peak phase current [A]
        double base_impedance_ohm;            ///< Base impedance [Ohm]
        double base_frequency_hz;             ///< Base frequency [Hz]
        double base_angular_frequency_rad_s;  ///< Base angular frequency [rad/s]
        double base_inductance_h;             ///< Base inductance [H]
    };

    /**
     * @brief Struct containing all per-unit base quantities for the DC Link.
     */
    struct dc_link_bases {
        double base_voltage_v;      ///< Base DC voltage [V]
        double base_current_a;      ///< Base DC current [A]
        double base_impedance_ohm;  ///< Base DC impedance [Ohm]
        double base_inductance_h;   ///< Base inductance [H]
    };

    /**
     * @brief Struct containing all per-unit base quantities for the Machine Side (CMS).
     */
    struct machine_side_bases {
        double base_power_va;                 ///< Base 3-phase apparent power [VA]
        double base_voltage_ll_rms_v;         ///< Base line-to-line RMS voltage [V]
        double base_voltage_peak_phase_v;     ///< Base peak phase voltage [V]
        double base_current_rms_a;            ///< Base line RMS current [A]
        double base_current_peak_phase_a;     ///< Base peak phase current [A]
        double base_impedance_ohm;            ///< Base impedance [Ohm]
        double base_frequency_hz;             ///< Base frequency [Hz]
        double base_angular_frequency_rad_s;  ///< Base angular frequency [rad/s]
        double base_inductance_h;             ///< Base inductance [H]
        double base_torque_nm;                ///< Base mechanical torque [Nm]
        double base_flux_linkage_vs_rad;      ///< Base flux linkage [Vs/rad]
    };

    /**
     * @brief Constructor for the LCI per-unit system.
     *
     * @param grid_voltage_ll_rms Rated line-to-line RMS voltage on the grid-side (LV) [V].
     * @param grid_frequency_hz Rated frequency on the grid-side [Hz].
     * @param dc_link_current_rated_a Rated nominal DC link current [A].
     * @param machine_power_va Rated 3-phase apparent power of the machine [VA].
     * @param machine_voltage_ll_rms Rated line-to-line RMS voltage on the machine-side [V].
     * @param machine_frequency_hz Rated frequency on the machine-side [Hz].
     * @param machine_num_poles Number of poles of the synchronous machine.
     */
    sfc_per_unit_system(double grid_voltage_ll_rms, double grid_frequency_hz, double dc_link_current_rated_a,
                        double machine_power_va, double machine_voltage_ll_rms, double machine_frequency_hz,
                        int machine_num_poles)
        : _grid_voltage_ll_rms_v(grid_voltage_ll_rms),
          _grid_frequency_hz(grid_frequency_hz),
          _dc_link_current_rated_a(dc_link_current_rated_a),
          _machine_power_va(machine_power_va),
          _machine_voltage_ll_rms_v(machine_voltage_ll_rms),
          _machine_frequency_hz(machine_frequency_hz),
          _machine_num_poles(machine_num_poles) {
        assert(grid_voltage_ll_rms > 0.0 && "Grid voltage must be positive");
        assert(grid_frequency_hz > 0.0 && "Grid frequency must be positive");
        assert(dc_link_current_rated_a > 0.0 && "DC link current must be positive");
        assert(machine_power_va > 0.0 && "Machine power must be positive");
        assert(machine_voltage_ll_rms > 0.0 && "Machine voltage must be positive");
        assert(machine_frequency_hz > 0.0 && "Machine frequency must be positive");
        assert(machine_num_poles > 0 && machine_num_poles % 2 == 0 && "Machine poles must be a positive even number");

        calculate_base_values();
    }

    grid_side_bases const& get_grid_bases() const { return _grid_side; }
    dc_link_bases const& get_dc_link_bases() const { return _dc_link; }
    machine_side_bases const& get_machine_bases() const { return _machine_side; }

private:
    // Input parameters
    const double _grid_voltage_ll_rms_v;
    const double _grid_frequency_hz;
    const double _dc_link_current_rated_a;
    const double _machine_power_va;
    const double _machine_voltage_ll_rms_v;
    const double _machine_frequency_hz;
    const double _machine_num_poles;

    grid_side_bases _grid_side{};
    dc_link_bases _dc_link{};
    machine_side_bases _machine_side{};

    grid_side_bases calculate_grid_side_base_values() const;
    dc_link_bases calculate_dc_link_base_values() const;
    machine_side_bases calculate_machine_side_base_values() const;

    /**
     * @brief Calculate all derived base values and populate the public structs.
     */
    void calculate_base_values();
};

}  // namespace yeet::application::sfc