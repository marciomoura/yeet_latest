#pragma once

#include <cassert>
#include <cmath>
#include <type_traits>

#include "yeet_control/constants.hpp"
#include "yeet_control/per_unit.hpp"
#include "yeet_control/reference_frame.hpp"

namespace yeet::application::sfc {

/**
 * @brief Per-Unit System for Load-Commutated Inverter (LCI) Drives
 *
 * This class implements the per-unit system as defined in the SFC documentation.
 * The system is designed around the synchronous machine ratings as the primary
 * base values, with DC link base quantities derived to ensure power invariance.
 *
 * Reference: software/yeet_application/sfc/docs/per_unit_system.md
 */
class sfc_per_unit_system {
public:
    /**
     * @brief Constructor for the per-unit system
     *
     * @param rated_power_va Rated 3-phase apparent power of the synchronous machine [VA]
     * @param rated_voltage_ll_rms Rated line-to-line RMS voltage of the synchronous machine [V]
     * @param rated_frequency_hz Rated electrical frequency of the machine [Hz]
     */
    sfc_per_unit_system(double rated_power_va, double rated_voltage_ll_rms, double rated_frequency_hz)
        : _base_apparent_power_va(rated_power_va),
          _base_voltage_ll_rms_v(rated_voltage_ll_rms),
          _base_frequency_hz(rated_frequency_hz) {
        assert(rated_power_va > 0.0 && "Rated power must be positive");
        assert(rated_voltage_ll_rms > 0.0 && "Rated voltage must be positive");
        assert(rated_frequency_hz > 0.0 && "Rated frequency must be positive");
        calculate_base_values();
    }

    // AC Side Base Quantities (Machine)
    /**
     * @brief Get base apparent power [VA]
     * @return Base power (S_base = S_rated,M)
     */
    double get_base_power() const { return _base_apparent_power_va; }

    /**
     * @brief Get base line-to-line RMS voltage [V]
     * @return Base voltage (V_base = V_rated,LL,M)
     */
    double get_base_voltage() const { return _base_voltage_ll_rms_v; }

    /**
     * @brief Get base line current RMS [A]
     * @return Base current (I_base = S_base / (sqrt(3) * V_base))
     */
    double get_base_current() const { return _base_current_rms_a; }

    /**
     * @brief Get base impedance [Ω]
     * @return Base impedance (Z_base = V_base^2 / S_base)
     */
    double get_base_impedance() const { return _base_impedance_ohm; }

    /**
     * @brief Get base electrical frequency [Hz]
     * @return Base frequency (f_base = f_rated,M)
     */
    double get_base_frequency() const { return _base_frequency_hz; }

    /**
     * @brief Get base electrical angular frequency [rad/s]
     * @return Base angular frequency (ω_base = 2π * f_base)
     */
    double get_base_angular_frequency() const { return _base_angular_frequency_rad_per_s; }

    /**
     * @brief Get base peak phase voltage [V]
     * @return Base peak phase voltage
     */
    double get_base_voltage_peak() const { return _base_voltage_phase_peak_v; }

    /**
     * @brief Get base peak phase current [A]
     * @return Base peak phase current
     */
    double get_base_current_peak() const { return _base_current_phase_peak_a; }

    /**
     * @brief Get base peak flux linkage [Vs]
     * @return Base peak flux linkage
     */
    double get_base_flux_linkage_peak() const { return _base_flux_linkage_peak_vs; }

    // DC Link Base Quantities
    /**
     * @brief Get base DC voltage [V]
     * @return Base DC voltage (V_d,base = (3*sqrt(2)/π) * V_base)
     */
    double get_base_dc_voltage() const { return _base_dc_voltage_v; }

    /**
     * @brief Get base DC current [A]
     * @return Base DC current (I_d,base = S_base / V_d,base)
     */
    double get_base_dc_current() const { return _base_dc_current_a; }

    /**
     * @brief Get base DC impedance [Ω]
     * @return Base DC impedance (Z_d,base = V_d,base / I_d,base)
     */
    double get_base_dc_impedance() const { return _base_dc_impedance_ohm; }

    /**
     * @brief Get the conversion factor from AC to DC voltage base
     * @return AC to DC voltage conversion factor (3*sqrt(2)/π)
     */
    static constexpr double get_ac_to_dc_voltage_factor() { return 3.0 * control::sqrt_2 / yeet::control::pi<double>; }

    /**
     * @brief Get the conversion factor from DC to AC current base
     * @return DC to AC current conversion factor (π/(3*sqrt(2)))
     */
    static constexpr double get_dc_to_ac_current_factor() {
        return yeet::control::pi<double> / (3.0 * control::sqrt_2);
    }

private:
    // Input parameters
    double _base_apparent_power_va;  ///< Base apparent power [VA]
    double _base_voltage_ll_rms_v;   ///< Base line-to-line RMS voltage [V]
    double _base_frequency_hz;       ///< Base electrical frequency [Hz]

    // Calculated AC base quantities
    double _base_current_rms_a;                ///< Base line current RMS [A]
    double _base_impedance_ohm;                ///< Base impedance [Ω]
    double _base_angular_frequency_rad_per_s;  ///< Base electrical angular frequency [rad/s]

    // Peak base quantities for frame conversions
    double _base_voltage_phase_peak_v;  ///< Base peak phase voltage [V]
    double _base_current_phase_peak_a;  ///< Base peak phase current [A]
    double _base_flux_linkage_peak_vs;  ///< Base peak flux linkage [Vs]

    // Calculated DC base quantities
    double _base_dc_voltage_v;      ///< Base DC voltage [V]
    double _base_dc_current_a;      ///< Base DC current [A]
    double _base_dc_impedance_ohm;  ///< Base DC impedance [Ω]

    /**
     * @brief Calculate all derived base values from the input parameters
     */
    void calculate_base_values() {
        // AC base quantities
        _base_current_rms_a = _base_apparent_power_va / (yeet::control::sqrt_3 * _base_voltage_ll_rms_v);
        _base_impedance_ohm = (_base_voltage_ll_rms_v * _base_voltage_ll_rms_v) / _base_apparent_power_va;
        _base_angular_frequency_rad_per_s = yeet::control::two_pi<double> * _base_frequency_hz;

        // Peak base quantities for frame conversions
        _base_voltage_phase_peak_v =
            _base_voltage_ll_rms_v * yeet::control::line_to_phase_rms_factor * yeet::control::rms_to_peak_factor;
        _base_current_phase_peak_a = _base_current_rms_a * yeet::control::rms_to_peak_factor;
        _base_flux_linkage_peak_vs = _base_voltage_phase_peak_v / _base_angular_frequency_rad_per_s;

        // DC base quantities (ensuring power invariance)
        _base_dc_voltage_v = get_ac_to_dc_voltage_factor() * _base_voltage_ll_rms_v;
        _base_dc_current_a = _base_apparent_power_va / _base_dc_voltage_v;
        _base_dc_impedance_ohm = _base_dc_voltage_v / _base_dc_current_a;
    }
};

}  // namespace yeet::application::sfc