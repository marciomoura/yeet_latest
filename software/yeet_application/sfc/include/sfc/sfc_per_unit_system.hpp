#pragma once

#include <cassert>

#include "yeet_control/precomputed_divisor.hpp"
#include "yeet_control/units.hpp"

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
        precomputed_divisor<voltage_t> base_voltage_peak_phase;           ///< Base peak phase voltage [V]
        precomputed_divisor<current_t> base_current_peak_phase;           ///< Base peak phase current [A]
        precomputed_divisor<impedance_t> base_impedance;                  ///< Base impedance [Ohm]
        precomputed_divisor<frequency_t> base_frequency;                  ///< Base frequency [Hz]
        precomputed_divisor<angular_frequency_t> base_angular_frequency;  ///< Base angular frequency [rad/s]
        precomputed_divisor<inductance_t> base_inductance;                ///< Base inductance [H]
    };

    /**
     * @brief Struct containing all per-unit base quantities for the DC Link.
     */
    struct dc_link_bases {
        precomputed_divisor<voltage_t> base_voltage;        ///< Base DC voltage [V]
        precomputed_divisor<current_t> base_current;        ///< Base DC current [A]
        precomputed_divisor<impedance_t> base_impedance;    ///< Base DC impedance [Ohm]
        precomputed_divisor<inductance_t> base_inductance;  ///< Base inductance [H]
    };

    /**
     * @brief Struct containing all per-unit base quantities for the Machine Side (CMS).
     */
    struct machine_side_bases {
        precomputed_divisor<power_t> base_power;                          ///< Base 3-phase apparent power [VA]
        precomputed_divisor<voltage_t> base_voltage_peak_phase;           ///< Base peak phase voltage [V]
        precomputed_divisor<current_t> base_current_peak_phase;           ///< Base peak phase current [A]
        precomputed_divisor<impedance_t> base_impedance;                  ///< Base impedance [Ohm]
        precomputed_divisor<frequency_t> base_frequency;                  ///< Base frequency [Hz]
        precomputed_divisor<angular_frequency_t> base_angular_frequency;  ///< Base angular frequency [rad/s]
        precomputed_divisor<inductance_t> base_inductance;                ///< Base inductance [H]
        precomputed_divisor<torque_t> base_torque;                        ///< Base mechanical torque [Nm]
        precomputed_divisor<magnetic_flux_t> base_flux_linkage;           ///< Base flux linkage [Vs/rad]
    };

    /**
     * @brief Constructor for the LCI per-unit system.
     *
     * @param grid_voltage_ll_rms Rated line-to-line RMS voltage on the grid-side (LV) [V].
     * @param grid_frequency Rated frequency on the grid-side [Hz].
     * @param dc_link_current_rated Rated nominal DC link current [A].
     * @param machine_power Rated 3-phase apparent power of the machine [VA].
     * @param machine_voltage_ll_rms Rated line-to-line RMS voltage on the machine-side [V].
     * @param machine_frequency Rated frequency on the machine-side [Hz].
     * @param machine_num_poles Number of poles of the synchronous machine.
     */
    sfc_per_unit_system(voltage_t grid_voltage_ll_rms, frequency_t grid_frequency, current_t dc_link_current_rated,
                        power_t machine_power, voltage_t machine_voltage_ll_rms, frequency_t machine_frequency,
                        int machine_num_poles)
        : _grid_voltage_ll_rms(grid_voltage_ll_rms),
          _grid_frequency(grid_frequency),
          _dc_link_current_rated(dc_link_current_rated),
          _machine_power(machine_power),
          _machine_voltage_ll_rms(machine_voltage_ll_rms),
          _machine_frequency(machine_frequency),
          _machine_num_poles(machine_num_poles) {
        assert(grid_voltage_ll_rms.value() > 0.0 && "Grid voltage must be positive");
        assert(grid_frequency.value() > 0.0 && "Grid frequency must be positive");
        assert(dc_link_current_rated.value() > 0.0 && "DC link current must be positive");
        assert(machine_power.value() > 0.0 && "Machine power must be positive");
        assert(machine_voltage_ll_rms.value() > 0.0 && "Machine voltage must be positive");
        assert(machine_frequency.value() > 0.0 && "Machine frequency must be positive");
        assert(machine_num_poles > 0 && machine_num_poles % 2 == 0 && "Machine poles must be a positive even number");

        calculate_base_values();
    }

    grid_side_bases const& get_grid_bases() const { return _grid_side; }
    dc_link_bases const& get_dc_link_bases() const { return _dc_link; }
    machine_side_bases const& get_machine_bases() const { return _machine_side; }

private:
    // Input parameters
    voltage_t _grid_voltage_ll_rms;
    frequency_t _grid_frequency;
    current_t _dc_link_current_rated;
    power_t _machine_power;
    voltage_t _machine_voltage_ll_rms;
    frequency_t _machine_frequency;
    int _machine_num_poles;

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