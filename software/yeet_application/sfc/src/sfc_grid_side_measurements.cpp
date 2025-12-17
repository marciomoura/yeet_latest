#include "sfc/sfc_grid_side_measurements.hpp"

namespace yeet::application::sfc {

// Implementation for ideal measurements (no ADC simulation)
void sfc_grid_side_measurements_ideal::update(sfc_per_unit_system::grid_side_bases const& bases,
    const abc<voltage_t>& raw_voltages,
    const abc<current_t>& raw_currents)
{
    // Convert to per-unit using peak phase base values
    _voltage_pu = to_pu(raw_voltages, bases.base_voltage_peak_phase);
    _current_pu = to_pu(raw_currents, bases.base_current_peak_phase);

    // Store SI values
    _voltage_line = yeet::to_line_voltage(raw_voltages);
    _current = raw_currents;
}

// Implementation for realistic measurements (with ADC simulation)
void sfc_grid_side_measurements::configure_voltage_conversion(int bits, real_t full_scale_voltage)
{
    _voltage_converter.configure(bits, full_scale_voltage);
}

void sfc_grid_side_measurements::configure_current_conversion(int bits, real_t full_scale_current)
{
    _current_converter.configure(bits, full_scale_current);
}

void sfc_grid_side_measurements::configure_voltage_transducer_gain(real_t gain) noexcept
{
    if (gain > real_t{0}) {
        _voltage_transducer_gain = yeet::precomputed_divisor<real_t>{gain};
    }
}

void sfc_grid_side_measurements::configure_current_transducer_gain(real_t gain) noexcept
{
    if (gain > real_t{0}) {
        _current_transducer_gain = yeet::precomputed_divisor<real_t>{gain};
    }
}

void sfc_grid_side_measurements::configure_transformer_ratios(real_t pt_ratio, real_t ct_ratio) noexcept
{
    if (pt_ratio > real_t{0}) {
        _pt_ratio = yeet::precomputed_divisor<real_t>{pt_ratio};
    }
    if (ct_ratio > real_t{0}) {
        _ct_ratio = yeet::precomputed_divisor<real_t>{ct_ratio};
    }
}

void sfc_grid_side_measurements::configure_voltage_transformer(
    real_t primary_ll_voltage, real_t secondary_ll_voltage) noexcept
{
    if (secondary_ll_voltage > real_t{0}) {
        _pt_ratio = yeet::precomputed_divisor<real_t>{primary_ll_voltage / secondary_ll_voltage};
    }
}

void sfc_grid_side_measurements::configure_current_transformer(
    real_t primary_current, real_t secondary_current) noexcept
{
    if (secondary_current > real_t{0}) {
        _ct_ratio = yeet::precomputed_divisor<real_t>{primary_current / secondary_current};
    }
}

void sfc_grid_side_measurements::update(sfc_per_unit_system::grid_side_bases const& bases,
    yeet::control::adc_counts voltage_ab_counts,
    yeet::control::adc_counts voltage_bc_counts,
    yeet::control::adc_counts current_a_counts,
    yeet::control::adc_counts current_c_counts)
{
    // Store the ADC values
    _v_ab_counts = voltage_ab_counts;
    _v_bc_counts = voltage_bc_counts;
    _i_a_counts = current_a_counts;
    _i_c_counts = current_c_counts;

    // --- Voltage Measurement Processing ---
    // Step 1: ADC counts -> card input volts
    const voltage_t voltage_ab_card{_voltage_converter.to_real(_v_ab_counts)};
    const voltage_t voltage_bc_card{_voltage_converter.to_real(_v_bc_counts)};
    // Step 2: Remove transducer gain (secondary LL voltage = card volts / gain)
    const voltage_t voltage_ab_secondary = voltage_ab_card / _voltage_transducer_gain.value();
    const voltage_t voltage_bc_secondary = voltage_bc_card / _voltage_transducer_gain.value();
    // Step 3: Apply PT ratio to get primary side line-line volts
    const voltage_t voltage_ab_primary = voltage_ab_secondary * _pt_ratio.value();
    const voltage_t voltage_bc_primary = voltage_bc_secondary * _pt_ratio.value();
    // Step 4: Reconstruct primary phase voltages from primary line voltages
    const abc<voltage_t> raw_voltages = from_line<voltage_t>(voltage_ab_primary, voltage_bc_primary);

    // --- Current Measurement Processing ---
    // Get converted real values
    const voltage_t current_a_card_voltage{_current_converter.to_real(_i_a_counts)};
    const voltage_t current_c_card_voltage{_current_converter.to_real(_i_c_counts)};
    // Remove current transducer gain (secondary current = card volts / gain)
    const current_t current_a_secondary{current_a_card_voltage.value() / _current_transducer_gain.value()};
    const current_t current_c_secondary{current_c_card_voltage.value() / _current_transducer_gain.value()};
    // Scale back to primary using CT ratio
    const real_t ct_ratio_value = _ct_ratio.value();
    const current_t current_a_primary = current_a_secondary * ct_ratio_value;
    const current_t current_c_primary = current_c_secondary * ct_ratio_value;
    // Reconstruct phase b current assuming a balanced system (Ia + Ib + Ic = 0)
    const current_t current_b_primary = -(current_a_primary + current_c_primary);
    const abc<current_t> raw_currents{current_a_primary, current_b_primary, current_c_primary};

    // Convert to per-unit using peak phase base values
    _voltage_pu = to_pu(raw_voltages, bases.base_voltage_peak_phase);
    _current_pu = to_pu(raw_currents, bases.base_current_peak_phase);

    // Store SI values
    _voltage_line = yeet::to_line_voltage(raw_voltages);
    _current = raw_currents;
}

void sfc_grid_side_measurements::reset()
{
    _v_ab_counts = {};
    _v_bc_counts = {};
    _i_a_counts = {};
    _i_c_counts = {};
    _voltage_pu = abc<voltage_pu_t>{};
    _current_pu = abc<current_pu_t>{};
    _voltage_line = line_voltage<voltage_t>{};
    _current = abc<current_t>{};
}

}  // namespace yeet::application::sfc