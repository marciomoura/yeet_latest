#pragma once

#include <cassert>

#include "yeet_control/precomputed_divisor.hpp"
#include "yeet_control/reference_frame.hpp"
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

    /**
     * @brief Reflects a voltage from machine-side per-unit to grid-side per-unit.
     *
     * This conversion accounts for the different voltage bases between the machine-side
     * and grid-side per-unit systems. The reflection uses the ratio of the base voltages.
     *
     * @param machine_side_voltage_pu Voltage in machine-side per-unit [pu]
     * @return Voltage reflected to grid-side per-unit [pu]
     */
    voltage_pu_t reflect_machine_to_grid_voltage_pu(voltage_pu_t machine_side_voltage_pu) const {
        // V_grid_pu = V_machine_pu * (V_base_machine / V_base_grid)
        const double voltage_reflection_ratio = _machine_voltage_ll_rms.value() / _grid_voltage_ll_rms.value();
        return voltage_pu_t{machine_side_voltage_pu.value() * voltage_reflection_ratio};
    }

    /**
     * @brief Reflects a voltage from grid-side per-unit to machine-side per-unit.
     *
     * This conversion accounts for the different voltage bases between the grid-side
     * and machine-side per-unit systems. The reflection uses the ratio of the base voltages.
     *
     * @param grid_side_voltage_pu Voltage in grid-side per-unit [pu]
     * @return Voltage reflected to machine-side per-unit [pu]
     */
    voltage_pu_t reflect_grid_to_machine_voltage_pu(voltage_pu_t grid_side_voltage_pu) const {
        // V_machine_pu = V_grid_pu * (V_base_grid / V_base_machine)
        const double voltage_reflection_ratio = _grid_voltage_ll_rms.value() / _machine_voltage_ll_rms.value();
        return voltage_pu_t{grid_side_voltage_pu.value() * voltage_reflection_ratio};
    }

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

/**
 * @brief Generic function to reflect per-unit quantities between different base systems.
 *
 * This function converts a per-unit quantity from one base system to another by
 * applying the appropriate scaling factor based on the ratio of the base values.
 *
 * @tparam T The type of the per-unit quantity (e.g., voltage_pu_t, abc<voltage_pu_t>)
 * @tparam BaseType The type of the base quantity (e.g., voltage_t)
 * @param source_pu_quantity The per-unit quantity in the source base system [pu]
 * @param source_base_value The base value of the source system
 * @param target_base_value The base value of the target system (as precomputed divisor for efficiency)
 * @return The per-unit quantity reflected to the target base system [pu]
 */
template <typename T, typename BaseType>
inline T reflect_per_unit_base_to_another_base(const T& source_pu_quantity, const BaseType& source_base_value,
                                               const precomputed_divisor<BaseType>& target_base_value) {
    // Reflection ratio = source_base / target_base
    const real_t reflection_ratio = source_base_value / target_base_value;
    return reflection_ratio * source_pu_quantity;
}

/**
 * @brief Reflects voltages from machine-side per-unit to grid-side per-unit.
 *
 * @tparam T The type of the voltage quantity (e.g., voltage_pu_t, abc<voltage_pu_t>)
 * @param machine_side_voltage_pu Voltage in machine-side per-unit [pu]
 * @param machine_side_base_voltage The machine-side voltage base [V]
 * @param grid_side_base_voltage The grid-side voltage base (precomputed divisor) [V]
 * @return Voltage reflected to grid-side per-unit [pu]
 */
template <typename T>
inline T reflect_machine_to_grid_voltage_pu(const T& machine_side_voltage_pu,
                                            const voltage_t& machine_side_base_voltage,
                                            const precomputed_divisor<voltage_t>& grid_side_base_voltage) {
    return reflect_per_unit_base_to_another_base(machine_side_voltage_pu, machine_side_base_voltage,
                                                 grid_side_base_voltage);
}

/**
 * @brief Reflects voltages from grid-side per-unit to machine-side per-unit.
 *
 * @tparam T The type of the voltage quantity (e.g., voltage_pu_t, abc<voltage_pu_t>)
 * @param grid_side_voltage_pu Voltage in grid-side per-unit [pu]
 * @param grid_side_base_voltage The grid-side voltage base [V]
 * @param machine_side_base_voltage The machine-side voltage base (precomputed divisor) [V]
 * @return Voltage reflected to machine-side per-unit [pu]
 */
template <typename T>
inline T reflect_grid_to_machine_voltage_pu(const T& grid_side_voltage_pu, const voltage_t& grid_side_base_voltage,
                                            const precomputed_divisor<voltage_t>& machine_side_base_voltage) {
    return reflect_per_unit_base_to_another_base(grid_side_voltage_pu, grid_side_base_voltage,
                                                 machine_side_base_voltage);
}

}  // namespace yeet::application::sfc