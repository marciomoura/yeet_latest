#pragma once

#include "sfc_per_unit_system.hpp"
#include "yeet_control/adc_value.hpp"
#include "yeet_control/precomputed_divisor.hpp"
#include "yeet_control/reference_frame.hpp"
#include "yeet_control/units.hpp"

namespace yeet::application::sfc {

/**
 * @brief Handles ideal measurement conversion for the grid side of the SFC.
 *
 * This class takes raw three-phase voltage and current measurements and
 * converts them into the per-unit system defined for the grid side.
 * It does not perform any filtering or ADC simulation.
 */
class sfc_grid_side_measurements_ideal {
public:
    /**
     * @brief Updates the per-unit values from new raw measurements.
     * @param bases The per-unit base values for the grid side.
     * @param raw_voltages The raw three-phase voltage measurements [V].
     * @param raw_currents The raw three-phase current measurements [A].
     */
    void update(sfc_per_unit_system::grid_side_bases const& bases,
        const abc<voltage_t>& raw_voltages,
        const abc<current_t>& raw_currents);

    const abc<voltage_pu_t>& get_voltage_pu() const { return _voltage_pu; }
    const abc<current_pu_t>& get_current_pu() const { return _current_pu; }

private:
    abc<voltage_pu_t> _voltage_pu{};
    abc<current_pu_t> _current_pu{};
};

/**
 * @brief Handles measurement conversion for the grid side of the SFC using ADC simulation.
 *
 * This class takes ADC counts from the hardware mock, converts them back to real values,
 * and then converts them into the per-unit system defined for the grid side.
 * It simulates the complete measurement chain including ADC quantization effects.
 */
class sfc_grid_side_measurements {
public:
    /**
     * @brief Constructs the grid-side measurements processor.
     */
    sfc_grid_side_measurements() = default;

    /**
     * @brief Configures the ADC conversion parameters for voltage channels.
     * @param bits The number of bits of the voltage ADCs.
     * @param full_scale_voltage The full-scale voltage range of the ADCs.
     */
    void configure_voltage_conversion(int bits, real_t full_scale_voltage);

    /**
     * @brief Configures the ADC conversion parameters for current channels.
     * @param bits The number of bits of the current ADCs.
     * @param full_scale_current The full-scale current range of the ADCs.
     */
    void configure_current_conversion(int bits, real_t full_scale_current);

    /**
     * @brief Configure the voltage transducer gain (secondary transformer output to ADC card input).
     * Gain defined as V_card / V_secondary_LL. Must be > 0. Stored internally as divisor for fast inversion.
     */
    void configure_voltage_transducer_gain(real_t gain) noexcept;

    /**
     * @brief Configure the current transducer gain (ADC card input volts per secondary ampere).
     * Gain defined as V_card / I_secondary. Must be > 0. Stored as divisor for fast inversion.
     */
    void configure_current_transducer_gain(real_t gain) noexcept;

    /**
     * @brief Configures potential transformer (PT) and current transformer (CT) ratios.
     * Ratios defined as Primary/Secondary. Provided as raw scalar ratios.
     * Stored internally as precomputed divisors for fast primary reconstruction.
     * @param pt_ratio Primary line-line voltage / secondary line-line voltage.
     * @param ct_ratio Primary current / secondary current.
     */
    void configure_transformer_ratios(real_t pt_ratio, real_t ct_ratio) noexcept;

    /**
     * @brief Configure PT using primary & secondary voltages directly.
     */
    void configure_voltage_transformer(real_t primary_ll_voltage, real_t secondary_ll_voltage) noexcept;

    /**
     * @brief Configure CT using primary & secondary currents directly.
     */
    void configure_current_transformer(real_t primary_current, real_t secondary_current) noexcept;

    /**
     * @brief Updates the per-unit values from ADC counts.
     * @param bases The per-unit base values for the grid side.
     * @param v_ab_counts ADC counts for line voltage Vab.
     * @param v_bc_counts ADC counts for line voltage Vbc.
     * @param i_a_counts ADC counts for line current Ia.
     * @param i_c_counts ADC counts for line current Ic.
     */
    void update(sfc_per_unit_system::grid_side_bases const& bases,
        yeet::control::adc_counts v_ab_counts,
        yeet::control::adc_counts v_bc_counts,
        yeet::control::adc_counts i_a_counts,
        yeet::control::adc_counts i_c_counts);

    /**
     * @brief Resets all internal states.
     */
    void reset();

    const abc<voltage_pu_t>& get_voltage_pu() const { return _voltage_pu; }
    const abc<current_pu_t>& get_current_pu() const { return _current_pu; }

    // Individual raw measurements after ADC conversion
    real_t get_raw_v_ab() const { return _voltage_converter.to_real(_v_ab_counts); }
    real_t get_raw_v_bc() const { return _voltage_converter.to_real(_v_bc_counts); }
    real_t get_raw_i_a() const { return _current_converter.to_real(_i_a_counts); }
    real_t get_raw_i_c() const { return _current_converter.to_real(_i_c_counts); }

    // Reconstructed primary (pre-transformer) quantities using configured ratios
    real_t get_primary_v_ab() const { return get_raw_v_ab() * _pt_ratio.value(); }
    real_t get_primary_v_bc() const { return get_raw_v_bc() * _pt_ratio.value(); }
    real_t get_primary_i_a() const { return get_raw_i_a() * _ct_ratio.value(); }
    real_t get_primary_i_c() const { return get_raw_i_c() * _ct_ratio.value(); }

    real_t get_voltage_transducer_gain() const { return _voltage_transducer_gain.value(); }
    real_t get_current_transducer_gain() const { return _current_transducer_gain.value(); }

    real_t get_pt_ratio() const { return _pt_ratio.value(); }
    real_t get_ct_ratio() const { return _ct_ratio.value(); }

private:
    // ADC values for each measurement channel
    yeet::control::adc_counts _v_ab_counts{};
    yeet::control::adc_counts _v_bc_counts{};
    yeet::control::adc_counts _i_a_counts{};
    yeet::control::adc_counts _i_c_counts{};

    yeet::control::adc_converter _voltage_converter{};
    yeet::control::adc_converter _current_converter{};

    // Transformer ratios (Primary / Secondary) stored as divisors for fast secondary->primary reconstruction
    yeet::precomputed_divisor<real_t> _pt_ratio{real_t{1.0}};
    yeet::precomputed_divisor<real_t> _ct_ratio{real_t{1.0}};
    // Transducer gains (card input volts per secondary quantity)
    yeet::precomputed_divisor<real_t> _voltage_transducer_gain{real_t{1.0}};  // V_card / V_secondary_LL
    yeet::precomputed_divisor<real_t> _current_transducer_gain{real_t{1.0}};  // V_card / I_secondary

    // Per-unit outputs
    abc<voltage_pu_t> _voltage_pu{};
    abc<current_pu_t> _current_pu{};
};

}  // namespace yeet::application::sfc