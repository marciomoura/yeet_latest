#include "sfc/sfc_machine_side_observer.hpp"

#include "yeet_control/units.hpp"

namespace yeet::application::sfc {

sfc_machine_side_observer::sfc_machine_side_observer(duration_t sampling_period)
    : _pll(sampling_period),
      _filtered_voltage_filter(real_t{sampling_period.value()}),
      _filtered_frequency(real_t{sampling_period.value()}),
      _nominal_frequency{default_nominal_frequency}
{
    configure_voltage_filter(default_voltage_estimation_filter_cutoff);

    const auto gains = _pll.parameters_30_hz_crossover_frequency;
    configure_pll(gains.kp, gains.ti, default_nominal_frequency);

    configure_frequency_filter(default_frequency_estimation_filter_cutoff);

    reset();
}

void sfc_machine_side_observer::configure_pll(real_t kp, duration_t ti, frequency_t nominal_frequency)
{
    assert(nominal_frequency.value() > 0.0f && kp > 0.0f && ti.value() > 0.0f &&
           "Nominal frequency, kp, and ti must be positive values");

    _nominal_frequency = nominal_frequency;
    // Configure PLL with specified gains
    _pll.configure_nominal_frequency(nominal_frequency);
    _pll.configure_pi_controller(kp, ti);
}

void sfc_machine_side_observer::configure_voltage_filter(frequency_t cutoff_frequency, real_t damping_ratio)
{
    _filtered_voltage_filter.configure(cutoff_frequency, damping_ratio);
}

void sfc_machine_side_observer::configure_frequency_filter(frequency_t cutoff_frequency, real_t damping_ratio)
{
    _filtered_frequency.configure(cutoff_frequency, damping_ratio);
}

void sfc_machine_side_observer::update(const abc<voltage_pu_t>& grid_voltage_abc)
{
    // Convert machine voltage to alpha-beta for PLL
    alphabeta<voltage_pu_t> const grid_voltage_alphabeta = grid_voltage_abc.to_alphabeta();

    // Update PLL
    _pll.update(grid_voltage_alphabeta);

    // Update output states
    _estimated_angle = _pll.get_estimated_angle_aligned_phase_a();
    _filtered_frequency.update(_pll.get_estimated_frequency_pu());
    _estimated_frequency_pu = frequency_pu_t{_filtered_frequency.get_output()};
    _estimated_frequency = to_si(_estimated_frequency_pu, _nominal_frequency);

    // Use the transformed d-q voltages to calculate magnitude
    const auto& dq_voltages = _pll.get_dq_vector();
    _filtered_voltage_filter.update(dq_voltages.magnitude());
    _estimated_voltage_magnitude = voltage_pu_t{_filtered_voltage_filter.get_output()};

    // Store the input alpha-beta as filtered output (the PLL internally filters)
    _filtered_voltage_alphabeta = _pll.get_alphabeta();
}

void sfc_machine_side_observer::reset(frequency_pu_t initial_frequency)
{
    _pll.reset(initial_frequency);
    _filtered_voltage_filter.reset();
    _filtered_frequency.reset();
    _estimated_angle = angle_wrapped{0.0f};
    _estimated_frequency = to_si(initial_frequency, _nominal_frequency);
    _estimated_frequency_pu = initial_frequency;
    _estimated_voltage_magnitude = voltage_pu_t{0.0f};
    _filtered_voltage_alphabeta = alphabeta<voltage_pu_t>{voltage_pu_t{0.0f}, voltage_pu_t{0.0f}};
}

void sfc_machine_side_observer::preset(angle_wrapped preset_angle, frequency_pu_t preset_frequency)
{
    // Preset the PLL integrator with the specified angle and frequency
    _pll.preset(preset_angle, preset_frequency);

    // Update the estimated outputs to match the preset values
    _estimated_angle = preset_angle;
    _estimated_frequency_pu = preset_frequency;
    _estimated_frequency = to_si(preset_frequency, _nominal_frequency);

    // Reset the frequency filter to the preset frequency
    _filtered_frequency.reset(_estimated_frequency_pu.value());
}

const angle_wrapped& sfc_machine_side_observer::get_estimated_angle() const { return _estimated_angle; }

frequency_t sfc_machine_side_observer::get_estimated_frequency() const { return _estimated_frequency; }

frequency_pu_t sfc_machine_side_observer::get_estimated_frequency_pu() const { return _estimated_frequency_pu; }

voltage_pu_t sfc_machine_side_observer::get_estimated_voltage_magnitude() const { return _estimated_voltage_magnitude; }

alphabeta<voltage_pu_t> sfc_machine_side_observer::get_filtered_voltage_alphabeta() const
{
    return _filtered_voltage_alphabeta;
}

}  // namespace yeet::application::sfc
