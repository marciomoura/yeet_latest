#include "sfc/sfc_machine_side_observer.hpp"

#include "yeet_control/six_pulse_generator.hpp"

namespace yeet::application::sfc {

sfc_machine_side_observer::sfc_machine_side_observer(duration_t sampling_period)
    : _pll(sampling_period),
      _filtered_voltage_filter(real_t{sampling_period.value()}),
      _nominal_frequency{default_nominal_frequency}
{
    // _commutation_mask.
    _filtered_voltage_filter.configure(10.0, 0.707);

    auto gains = _pll.parameters_30_hz_crossover_frequency;
    configure_pll(gains.kp, gains.ti, default_nominal_frequency);
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

void sfc_machine_side_observer::set_initial_speed(frequency_pu_t initial_frequency) { _pll.reset(initial_frequency); }

void sfc_machine_side_observer::update(const abc<voltage_pu_t>& grid_voltage_pu)
{
    // Convert machine voltage to alpha-beta for PLL
    alphabeta<voltage_pu_t> const grid_voltage_alphabeta = grid_voltage_pu.to_alphabeta();

    // Update PLL
    _pll.update(grid_voltage_alphabeta);

    // Update output states
    _estimated_angle = _pll.get_estimated_angle_aligned_phase_a();
    _estimated_frequency_pu = _pll.get_estimated_frequency_pu();
    _estimated_frequency = to_si(_estimated_frequency_pu, _nominal_frequency);

    // Use the transformed d-q voltages to calculate magnitude
    const auto& dq_voltages = _pll.get_dq_vector();
    _filtered_voltage_filter.update(dq_voltages.magnitude());
    _estimated_voltage_magnitude = voltage_pu_t{_filtered_voltage_filter.get_output()};

    // Store the input alpha-beta as filtered output (the PLL internally filters)
    _filtered_voltage_alphabeta = _pll.get_alphabeta();
}

void sfc_machine_side_observer::reset()
{
    _pll.reset();  // Reset to nominal frequency
    _estimated_angle = angle_wrapped{0.0f};
    _estimated_frequency = _nominal_frequency;
    _estimated_frequency_pu = frequency_pu_t{1.0f};
    _estimated_voltage_magnitude = voltage_pu_t{0.0f};
    _filtered_voltage_alphabeta = alphabeta<voltage_pu_t>{voltage_pu_t{0.0f}, voltage_pu_t{0.0f}};
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
