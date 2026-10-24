#pragma once

#include "sfc_per_unit_system.hpp"
#include "yeet_control/adc_value.hpp"
#include "yeet_control/precomputed_divisor.hpp"
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

    const line_voltage<voltage_t>& get_voltage_line() const { return _voltage_line; }

private:
    abc<voltage_pu_t> _voltage_pu{};
    abc<current_pu_t> _current_pu{};
    line_voltage<voltage_t> _voltage_line{};
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
     * @brief Configure PT ratio directly (Primary/Secondary).
     */
    void configure_pt_ratio(real_t pt_ratio) noexcept;

    /**
     * @brief Configure voltage transducer gain (card volts per secondary line-line volt).
     * Gain > 0. Stored internally as divisor for fast inversion during reconstruction.
     */
    void configure_voltage_transducer_gain(real_t gain) noexcept;

    /**
     * @brief Configure voltage transformer using primary & secondary line-line voltages.
     */
    void configure_voltage_transformer(real_t primary_ll_voltage, real_t secondary_ll_voltage) noexcept;

    /**
     * @brief Updates the per-unit values from ADC counts.
     * @param bases The per-unit base values for the machine side.
     * @param v_ab_counts ADC counts for line voltage Vab.
     * @param v_bc_counts ADC counts for line voltage Vbc.
     */
    void update(sfc_per_unit_system::machine_side_bases const& bases,
        yeet::control::adc_counts v_ab_counts,
        yeet::control::adc_counts v_bc_counts);

    /**
     * @brief Resets all internal states.
     */
    void reset();

    const abc<voltage_pu_t>& get_voltage_pu() const { return _voltage_pu; }
    const abc<current_pu_t>& get_current_pu() const { return _current_pu; }

    const line_voltage<voltage_t>& get_voltage_line() const { return _voltage_line; }

    /**
     * @brief Resets only the peak count detectors.
     */
    void clear_peaks() noexcept;

    // Peak counts detection results
    int32_t get_v_ab_counts_peak() const noexcept { return _v_ab_counts_peak; }
    int32_t get_v_bc_counts_peak() const noexcept { return _v_bc_counts_peak; }

    // Individual raw measurements after ADC conversion
    real_t get_raw_v_ab() const { return _voltage_converter.to_real(_v_ab_counts); }
    real_t get_raw_v_bc() const { return _voltage_converter.to_real(_v_bc_counts); }

    // Primary (pre-transformer) reconstructed voltages
    real_t get_primary_v_ab() const { return get_raw_v_ab() * _pt_ratio.value(); }
    real_t get_primary_v_bc() const { return get_raw_v_bc() * _pt_ratio.value(); }
    real_t get_pt_ratio() const { return _pt_ratio.value(); }

private:
    // ADC values for voltage measurement channels
    yeet::control::adc_counts _v_ab_counts{};
    yeet::control::adc_counts _v_bc_counts{};

    // Peak counts detection
    int32_t _v_ab_counts_peak{0};
    int32_t _v_bc_counts_peak{0};

    yeet::control::adc_converter _voltage_converter{};

    // Transformer ratio (Primary/Secondary)
    yeet::precomputed_divisor<real_t> _pt_ratio{real_t{1.0}};
    yeet::precomputed_divisor<real_t> _voltage_transducer_gain{real_t{1.0}};  // V_card / V_secondary_LL

    // Per-unit outputs
    abc<voltage_pu_t> _voltage_pu{};
    abc<current_pu_t> _current_pu{};  // Zero for machine side (no current measurements)

    // SI outputs
    line_voltage<voltage_t> _voltage_line{};
};

}  // namespace yeet::application::sfc