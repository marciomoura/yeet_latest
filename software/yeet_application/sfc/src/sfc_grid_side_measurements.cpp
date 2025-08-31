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
}

// Implementation for realistic measurements (with ADC simulation)
void sfc_grid_side_measurements::configure_voltage_conversion(int bits, real_t full_scale_voltage)
{
    _v_ab_counts = yeet::control::adc_value_t(bits, full_scale_voltage);
    _v_bc_counts = yeet::control::adc_value_t(bits, full_scale_voltage);
}

void sfc_grid_side_measurements::configure_current_conversion(int bits, real_t full_scale_current)
{
    _i_a_counts = yeet::control::adc_value_t(bits, full_scale_current);
    _i_c_counts = yeet::control::adc_value_t(bits, full_scale_current);
}

void sfc_grid_side_measurements::update(sfc_per_unit_system::grid_side_bases const& bases,
    const yeet::control::adc_value_t& v_ab_counts,
    const yeet::control::adc_value_t& v_bc_counts,
    const yeet::control::adc_value_t& i_a_counts,
    const yeet::control::adc_value_t& i_c_counts)
{
    // Store the ADC values
    _v_ab_counts = v_ab_counts;
    _v_bc_counts = v_bc_counts;
    _i_a_counts = i_a_counts;
    _i_c_counts = i_c_counts;

    // Get converted real values
    const real_t v_ab = _v_ab_counts.to_real();
    const real_t v_bc = _v_bc_counts.to_real();
    const real_t i_a = _i_a_counts.to_real();
    const real_t i_c = _i_c_counts.to_real();

    // Calculate the third phase (assuming balanced system)
    const real_t v_ca = -(v_ab + v_bc);  // Vca = -(Vab + Vbc)
    const real_t i_b = -(i_a + i_c);     // Ib = -(Ia + Ic)

    // Create abc structures with proper units
    const abc<voltage_t> raw_voltages{voltage_t{v_ab}, voltage_t{v_bc}, voltage_t{v_ca}};

    const abc<current_t> raw_currents{current_t{i_a}, current_t{i_b}, current_t{i_c}};

    // Convert to per-unit using peak phase base values
    _voltage_pu = to_pu(raw_voltages, bases.base_voltage_peak_phase);
    _current_pu = to_pu(raw_currents, bases.base_current_peak_phase);
}

void sfc_grid_side_measurements::reset()
{
    _v_ab_counts = yeet::control::adc_value_t{};
    _v_bc_counts = yeet::control::adc_value_t{};
    _i_a_counts = yeet::control::adc_value_t{};
    _i_c_counts = yeet::control::adc_value_t{};
    _voltage_pu = abc<voltage_pu_t>{};
    _current_pu = abc<current_pu_t>{};
}

}  // namespace yeet::application::sfc