#include "sfc/sfc_grid_side_observer.hpp"

namespace yeet::application::sfc {

sfc_grid_side_observer::sfc_grid_side_observer(duration_t sampling_period)
    : _pll(sampling_period), _nominal_frequency{default_nominal_frequency}
{
    configure_pll(default_pll_kp, default_pll_ti, default_nominal_frequency);
    reset();
}

void sfc_grid_side_observer::configure_pll(real_t kp, duration_t ti, frequency_t nominal_frequency)
{
    assert(
        nominal_frequency > 0.0f && kp > 0.0f && ti > 0.0f && "Nominal frequency, kp, and ti must be positive values");

    _nominal_frequency = nominal_frequency;
    // Configure PLL with default gains
    _pll.configure_nominal_frequency(nominal_frequency);
    _pll.configure_pi_controller(kp, ti);
    _pll.configure_frequency_limits(frequency_t{10.0f}, frequency_t{100.0f});
    _pll.configure_sequence_extractor(1.414, 1.0 / nominal_frequency, true);
    _pll.configure_pi_controller(kp, ti);
}

void sfc_grid_side_observer::update(const abc<voltage_pu_t>& grid_voltage_pu)
{
    // Convert masked voltage to dimensionless alpha-beta for PLL
    alphabeta<voltage_pu_t> const machine_voltage_alphabeta = grid_voltage_pu.to_alphabeta();

    // Update PLL
    _pll.update(machine_voltage_alphabeta);

    // Update output states
    _estimated_angle = _pll.get_estimated_angle();
    _estimated_frequency = frequency_t{_pll.get_estimated_frequency()};
    _estimated_frequency_pu = to_pu(_pll.get_estimated_frequency(), _nominal_frequency);

    // Get the positive sequence voltage from the PLL
    auto positive_sequence = _pll.get_positive_sequence();
    _filtered_voltage_alphabeta =
        alphabeta<voltage_pu_t>{voltage_pu_t{positive_sequence.alpha()}, voltage_pu_t{positive_sequence.beta()}};

    // Calculate voltage magnitude from the filtered positive sequence
    _estimated_voltage_magnitude = _filtered_voltage_alphabeta.magnitude();
}

void sfc_grid_side_observer::reset()
{
    _pll.reset();
    _estimated_angle = angle_wrapped{0.0f};
    _estimated_frequency = _nominal_frequency;
    _estimated_frequency_pu = frequency_pu_t{1.0f};
    _estimated_voltage_magnitude = voltage_pu_t{0.0f};
    _filtered_voltage_alphabeta = alphabeta<voltage_pu_t>{voltage_pu_t{0.0f}, voltage_pu_t{0.0f}};
}

const angle_wrapped& sfc_grid_side_observer::get_estimated_angle() const { return _estimated_angle; }

frequency_t sfc_grid_side_observer::get_estimated_frequency() const { return _estimated_frequency; }

frequency_pu_t sfc_grid_side_observer::get_estimated_frequency_pu() const { return _estimated_frequency_pu; }

voltage_pu_t sfc_grid_side_observer::get_estimated_voltage_magnitude() const { return _estimated_voltage_magnitude; }

alphabeta<voltage_pu_t> sfc_grid_side_observer::get_filtered_voltage_alphabeta() const
{
    return _filtered_voltage_alphabeta;
}

}  // namespace yeet::application::sfc
