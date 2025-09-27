#include "sfc/sfc_machine_side_measurements.hpp"

namespace yeet::application::sfc {

// Implementation for ideal measurements (no ADC simulation)
void sfc_machine_side_measurements_ideal::update(sfc_per_unit_system::machine_side_bases const& bases,
    const abc<voltage_t>& raw_voltages,
    const abc<current_t>& raw_currents)
{
    // Convert to per-unit using peak phase base values
    _voltage_pu = to_pu(raw_voltages, bases.base_voltage_peak_phase);
    _current_pu = to_pu(raw_currents, bases.base_current_peak_phase);
}

// Implementation for realistic measurements (with ADC simulation)
void sfc_machine_side_measurements::configure_voltage_conversion(int bits, real_t full_scale_voltage)
{
    _voltage_converter.configure(bits, full_scale_voltage);
}

void sfc_machine_side_measurements::configure_pt_ratio(real_t pt_ratio) noexcept
{
    if (pt_ratio > real_t{0.0}) {
        _pt_ratio = yeet::precomputed_divisor<real_t>{pt_ratio};
    }
}

void sfc_machine_side_measurements::configure_voltage_transducer_gain(real_t gain) noexcept
{
    if (gain > real_t{0}) {
        _voltage_transducer_gain = yeet::precomputed_divisor<real_t>{gain};
    }
}

void sfc_machine_side_measurements::configure_voltage_transformer(
    real_t primary_ll_voltage, real_t secondary_ll_voltage) noexcept
{
    if (secondary_ll_voltage > real_t{0}) {
        _pt_ratio = yeet::precomputed_divisor<real_t>{primary_ll_voltage / secondary_ll_voltage};
    }
}

void sfc_machine_side_measurements::update(sfc_per_unit_system::machine_side_bases const& bases,
    yeet::control::adc_counts voltage_ab_counts,
    yeet::control::adc_counts voltage_bc_counts)
{
    // Store adc counts
    _v_ab_counts = voltage_ab_counts;
    _v_bc_counts = voltage_bc_counts;

    // Step 1: ADC counts -> card input volts
    const voltage_t voltage_ab_card{_voltage_converter.to_real(_v_ab_counts)};
    const voltage_t voltage_bc_card{_voltage_converter.to_real(_v_bc_counts)};

    // Step 2: Remove transducer gain to obtain secondary side line-line volts
    const voltage_t voltage_ab_secondary = voltage_ab_card / _voltage_transducer_gain;
    const voltage_t voltage_bc_secondary = voltage_bc_card / _voltage_transducer_gain;

    // Step 3: Apply PT ratio to get primary side line-line volts
    const voltage_t voltage_ab_primary = voltage_ab_secondary * _pt_ratio.value();
    const voltage_t voltage_bc_primary = voltage_bc_secondary * _pt_ratio.value();

    // Step 4: Reconstruct primary phase voltages from primary line voltages
    const abc<voltage_t> raw_voltages = from_line<voltage_t>(voltage_ab_primary, voltage_bc_primary);

    // Convert to per-unit using peak phase base values
    _voltage_pu = to_pu(raw_voltages, bases.base_voltage_peak_phase);

    // Machine side has no current measurements, so create zero currents
    const abc<current_t> raw_currents{current_t{0.0}, current_t{0.0}, current_t{0.0}};

    _current_pu = to_pu(raw_currents, bases.base_current_peak_phase);
}

void sfc_machine_side_measurements::reset()
{
    _v_ab_counts = {};
    _v_bc_counts = {};
    _voltage_pu = abc<voltage_pu_t>{};
    _current_pu = abc<current_pu_t>{};
}

}  // namespace yeet::application::sfc