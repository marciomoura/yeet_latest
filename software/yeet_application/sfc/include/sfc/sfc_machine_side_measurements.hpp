#pragma once

#include "sfc_per_unit_system.hpp"
#include "yeet_control/adc_value_t.hpp"
#include "yeet_control/reference_frame.hpp"
#include "yeet_control/units.hpp"

namespace yeet::application::sfc {

/**
 * @brief Handles ideal measurement conversion for the machine side of the SFC.
 *
 * This class takes raw three-phase voltage and current measurements and
 * converts them into the per-unit system defined for the machine side.
 * It does not perform any filtering or ADC simulation.
 */
class sfc_machine_side_measurements_ideal {
public:
    /**
     * @brief Updates the per-unit values from new raw measurements.
     * @param bases The per-unit base values for the machine side.
     * @param raw_voltages The raw three-phase voltage measurements [V].
     * @param raw_currents The raw three-phase current measurements [A].
     */
    void update(sfc_per_unit_system::machine_side_bases const& bases,
        const abc<voltage_t>& raw_voltages,
        const abc<current_t>& raw_currents);

    const abc<voltage_pu_t>& get_voltage_pu() const { return _voltage_pu; }
    const abc<current_pu_t>& get_current_pu() const { return _current_pu; }

private:
    abc<voltage_pu_t> _voltage_pu{};
    abc<current_pu_t> _current_pu{};
};

/**
 * @brief Handles measurement conversion for the machine side of the SFC using ADC simulation.
 *
 * This class takes ADC counts from the hardware mock, converts them back to real values,
 * and then converts them into the per-unit system defined for the machine side.
 * It simulates the complete measurement chain including ADC quantization effects.
 *
 * Note: Machine side only measures voltages (Vab, Vbc), no current measurements.
 */
class sfc_machine_side_measurements {
public:
    /**
     * @brief Constructs the machine-side measurements processor.
     */
    sfc_machine_side_measurements() = default;

    /**
     * @brief Configures the ADC conversion parameters for voltage channels.
     * @param bits The number of bits of the voltage ADCs.
     * @param full_scale_voltage The full-scale voltage range of the ADCs.
     */
    void configure_voltage_conversion(int bits, real_t full_scale_voltage);

    /**
     * @brief Updates the per-unit values from ADC counts.
     * @param bases The per-unit base values for the machine side.
     * @param v_ab_counts ADC counts for line voltage Vab.
     * @param v_bc_counts ADC counts for line voltage Vbc.
     */
    void update(sfc_per_unit_system::machine_side_bases const& bases,
        const yeet::control::adc_value_t& v_ab_counts,
        const yeet::control::adc_value_t& v_bc_counts);

    /**
     * @brief Resets all internal states.
     */
    void reset();

    const abc<voltage_pu_t>& get_voltage_pu() const { return _voltage_pu; }
    const abc<current_pu_t>& get_current_pu() const { return _current_pu; }

    // Individual raw measurements after ADC conversion
    real_t get_raw_v_ab() const { return _v_ab_counts.to_real(); }
    real_t get_raw_v_bc() const { return _v_bc_counts.to_real(); }

private:
    // ADC values for voltage measurement channels
    yeet::control::adc_value_t _v_ab_counts;
    yeet::control::adc_value_t _v_bc_counts;

    // Per-unit outputs
    abc<voltage_pu_t> _voltage_pu{};
    abc<current_pu_t> _current_pu{};  // Zero for machine side (no current measurements)
};

}  // namespace yeet::application::sfc