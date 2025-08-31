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
    _v_ab_counts = yeet::control::adc_value_t(bits, full_scale_voltage);
    _v_bc_counts = yeet::control::adc_value_t(bits, full_scale_voltage);
}

void sfc_machine_side_measurements::update(sfc_per_unit_system::machine_side_bases const& bases,
    const yeet::control::adc_value_t& v_ab_counts,
    const yeet::control::adc_value_t& v_bc_counts)
{
    // Store the ADC values
    _v_ab_counts = v_ab_counts;
    _v_bc_counts = v_bc_counts;

    // Get converted real values
    const real_t v_ab = _v_ab_counts.to_real();
    const real_t v_bc = _v_bc_counts.to_real();

    // Calculate the third phase (assuming balanced system)
    const real_t v_ca = -(v_ab + v_bc);  // Vca = -(Vab + Vbc)

    // Create abc structures with proper units
    const abc<voltage_t> raw_voltages{voltage_t{v_ab}, voltage_t{v_bc}, voltage_t{v_ca}};

    // Machine side has no current measurements, so create zero currents
    const abc<current_t> raw_currents{current_t{0.0}, current_t{0.0}, current_t{0.0}};

    // Convert to per-unit using peak phase base values
    _voltage_pu = to_pu(raw_voltages, bases.base_voltage_peak_phase);
    _current_pu = to_pu(raw_currents, bases.base_current_peak_phase);
}

void sfc_machine_side_measurements::reset()
{
    _v_ab_counts = yeet::control::adc_value_t{};
    _v_bc_counts = yeet::control::adc_value_t{};
    _voltage_pu = abc<voltage_pu_t>{};
    _current_pu = abc<current_pu_t>{};
}

}  // namespace yeet::application::sfc