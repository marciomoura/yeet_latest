#include "sfc/sfc_grid_side_observer.hpp"

#include "yeet_control/six_pulse_generator.hpp"

namespace yeet::application::sfc {

sfc_grid_side_observer::sfc_grid_side_observer(duration_t sampling_period)
    : _pll(sampling_period),
      _commutation_monitor(sfc_commutation_monitor::Config{.commutation_start_threshold = default_commutation_threshold,
                                                           .commutation_end_threshold = default_commutation_threshold}),
      _commutation_mask(sampling_period.value()),
      _nominal_frequency{default_nominal_frequency} {
    configure_pll(default_pll_kp, default_pll_ti, default_nominal_frequency);
    reset();
}

void sfc_grid_side_observer::configure_pll(real_t kp, duration_t ti, frequency_t nominal_frequency) {
    assert(nominal_frequency > 0.0f && kp > 0.0f && ti > 0.0f &&
           "Nominal frequency, kp, and ti must be positive values");

    _nominal_frequency = nominal_frequency;
    // Configure PLL with default gains
    _pll.configure_nominal_frequency(nominal_frequency);
    _pll.configure_pi_controller(kp, ti);
    _pll.configure_frequency_limits(frequency_t{10.0f}, frequency_t{100.0f});
    _pll.configure_sequence_extractor(1.414, 1.0 / nominal_frequency, true);
    _pll.configure_pi_controller(kp, ti);
}

void sfc_grid_side_observer::configure_commutation_failure_threshold(voltage_pu_t threshold_pu) {
    // Update the commutation monitor configuration
    sfc_commutation_monitor::Config config{.commutation_start_threshold = threshold_pu,
                                           .commutation_end_threshold = threshold_pu};
    _commutation_monitor = sfc_commutation_monitor(config);
}

void sfc_grid_side_observer::update(const abc<voltage_pu_t>& grid_voltage_pu,
                                    const control::six_pulse_generator::pulses& grid_side_pulses) {
    // Convert to dimensionless for PLL input
    auto voltage_alpha_beta = grid_voltage_pu.to_alphabeta();
    alphabeta<real_t> voltage_dimensionless{voltage_alpha_beta.alpha().value(), voltage_alpha_beta.beta().value()};

    // Update commutation monitor
    _is_commuting = _commutation_monitor.update(grid_voltage_pu, grid_side_pulses);

    abc<voltage_pu_t> masked_grid_voltage_pu = _commutation_mask.update(
        grid_voltage_pu, _pll.get_estimated_angle(), frequency_t{_pll.get_estimated_frequency()}, _is_commuting);

    // Convert masked voltage to dimensionless alpha-beta for PLL
    alphabeta<voltage_pu_t> const masked_voltage = masked_grid_voltage_pu.to_alphabeta();

    // Update PLL
    _pll.update(masked_voltage);

    // Update output states
    _estimated_angle = _pll.get_estimated_angle();
    _estimated_frequency = frequency_t{_pll.get_estimated_frequency()};
    _estimated_frequency_pu = frequency_pu_t{_pll.get_estimated_frequency() / _nominal_frequency.value()};

    // Get the positive sequence voltage from the PLL
    auto positive_sequence = _pll.get_positive_sequence();
    _filtered_voltage_alphabeta =
        alphabeta<voltage_pu_t>{voltage_pu_t{positive_sequence.alpha()}, voltage_pu_t{positive_sequence.beta()}};

    // Calculate voltage magnitude from the filtered positive sequence
    _estimated_voltage_magnitude = _filtered_voltage_alphabeta.magnitude();
}

void sfc_grid_side_observer::reset() {
    _pll.reset();
    _estimated_angle = angle<zero_to_two_pi>{0.0f};
    _estimated_frequency = _nominal_frequency;
    _estimated_frequency_pu = frequency_pu_t{1.0f};
    _estimated_voltage_magnitude = voltage_pu_t{0.0f};
    _filtered_voltage_alphabeta = alphabeta<voltage_pu_t>{voltage_pu_t{0.0f}, voltage_pu_t{0.0f}};
    _is_commuting = false;
}

const angle<zero_to_two_pi>& sfc_grid_side_observer::get_estimated_angle() const { return _estimated_angle; }

frequency_t sfc_grid_side_observer::get_estimated_frequency() const { return _estimated_frequency; }

frequency_pu_t sfc_grid_side_observer::get_estimated_frequency_pu() const { return _estimated_frequency_pu; }

voltage_pu_t sfc_grid_side_observer::get_estimated_voltage_magnitude() const { return _estimated_voltage_magnitude; }

alphabeta<voltage_pu_t> sfc_grid_side_observer::get_filtered_voltage_alphabeta() const {
    return _filtered_voltage_alphabeta;
}

bool sfc_grid_side_observer::is_commuting() const { return _is_commuting; }

}  // namespace yeet::application::sfc
