#include "sfc/sfc_plant_internal_model.hpp"

#include <algorithm>
#include <cmath>

namespace yeet::application::sfc {

sfc_plant_internal_model::sfc_plant_internal_model(const sfc_plant_model_electrical_parameters& params)
    : _converter(params.grid_side_resistance,
          params.grid_side_inductance,
          params.dc_link_resistance,
          params.dc_link_inductance,
          params.machine_side_resistance,
          params.machine_side_inductance,
          duration_t(kSamplingTime)),
      _grid_waveform(kSamplingTime),
      _machine_waveform(kSamplingTime)
{
    _grid_waveform.disable_noise();
    _grid_waveform.set_fundamental_negative_sequence_signal_amplitude(0.0f);
    _machine_waveform.disable_noise();
    _machine_waveform.set_fundamental_negative_sequence_signal_amplitude(0.0f);
}

void sfc_plant_internal_model::configure_electrical_parameters(const sfc_plant_model_electrical_parameters& params)
{
    _converter = control::thyristor_back_to_back(params.grid_side_resistance, params.grid_side_inductance,
        params.dc_link_resistance, params.dc_link_inductance, params.machine_side_resistance,
        params.machine_side_inductance, duration_t(kSamplingTime));
    _converter.reset();
}

void sfc_plant_internal_model::initialize()
{
    _converter.reset();
    _grid_waveform = control::three_phase_waveform_generator(kSamplingTime);
    _grid_waveform.disable_noise();
    _grid_waveform.set_fundamental_negative_sequence_signal_amplitude(0.0f);
    _machine_waveform = control::three_phase_waveform_generator(kSamplingTime);
    _machine_waveform.disable_noise();
    _machine_waveform.set_fundamental_negative_sequence_signal_amplitude(0.0f);

    _grid_source_voltages = abc<voltage_t>{voltage_t{0.0}, voltage_t{0.0}, voltage_t{0.0}};
    _machine_source_voltages = abc<voltage_t>{voltage_t{0.0}, voltage_t{0.0}, voltage_t{0.0}};
    _grid_source_angle = angle_wrapped{0.0};
    _machine_source_angle = angle_wrapped{0.0};
}

void sfc_plant_internal_model::reset() { initialize(); }

void sfc_plant_internal_model::terminate() { _converter.reset(); }

void sfc_plant_internal_model::update(const std::array<bool, 6>& grid_pulses,
    const std::array<bool, 6>& machine_pulses,
    frequency_t grid_freq,
    voltage_t grid_volt_rms,
    frequency_t machine_freq,
    voltage_t machine_volt_rms)
{
    // Update waveforms
    configure_waveform_generator(_grid_waveform, grid_freq, grid_volt_rms);
    configure_waveform_generator(_machine_waveform, machine_freq, machine_volt_rms);

    _grid_waveform.update();
    _machine_waveform.update();

    _grid_source_voltages = to_dimension_abc<voltage_t>(_grid_waveform.get_signal_abc());
    _machine_source_voltages = to_dimension_abc<voltage_t>(_machine_waveform.get_signal_abc());

    _grid_source_angle = _grid_waveform.get_fundamental_signal_angle().a();
    _machine_source_angle = _machine_waveform.get_fundamental_signal_angle().a();

    // Update converter
    const control::thyristor_pulses g_pulses = control::thyristor_pulses::make_pulses(grid_pulses);
    const control::thyristor_pulses m_pulses = control::thyristor_pulses::make_pulses(machine_pulses);

    _converter.update(_grid_source_voltages, _machine_source_voltages, g_pulses, m_pulses);
}

void sfc_plant_internal_model::configure_waveform_generator(
    control::three_phase_waveform_generator& generator, frequency_t frequency, voltage_t voltage_rms) noexcept
{
    const real_t freq_value = std::max(static_cast<real_t>(frequency.value()), real_t{0.1f});
    generator.set_signal_frequency(freq_value);

    const real_t phase_peak = static_cast<real_t>(std::abs(voltage_rms.value()) * std::sqrt(2.0 / 3.0));
    generator.set_fundamental_positive_sequence_signal_amplitude(abc<real_t>{phase_peak, phase_peak, phase_peak});
}

abc<voltage_t> sfc_plant_internal_model::get_grid_voltages() const noexcept
{
    return _converter.get_rectifier_terminal_voltages();
}

abc<current_t> sfc_plant_internal_model::get_grid_currents() const noexcept { return _converter.get_grid_currents(); }

abc<voltage_t> sfc_plant_internal_model::get_machine_voltages() const noexcept
{
    return _converter.get_inverter_terminal_voltages();
}

abc<current_t> sfc_plant_internal_model::get_machine_currents() const noexcept
{
    return _converter.get_machine_currents();
}

current_t sfc_plant_internal_model::get_dc_link_current() const noexcept { return _converter.get_dc_current(); }

angle_wrapped sfc_plant_internal_model::get_grid_source_angle() const noexcept { return _grid_source_angle; }

angle_wrapped sfc_plant_internal_model::get_machine_source_angle() const noexcept { return _machine_source_angle; }

voltage_t sfc_plant_internal_model::get_rectifier_dc_voltage() const noexcept
{
    return _converter.get_rectifier_dc_voltage();
}

voltage_t sfc_plant_internal_model::get_inverter_dc_voltage() const noexcept
{
    return _converter.get_inverter_dc_voltage();
}

std::array<bool, 6> sfc_plant_internal_model::get_grid_bridge_conduction_states() const noexcept
{
    return _converter.get_rectifier_bridge().get_conduction_states();
}

std::array<bool, 6> sfc_plant_internal_model::get_machine_bridge_conduction_states() const noexcept
{
    return _converter.get_inverter_bridge().get_conduction_states();
}

}  // namespace yeet::application::sfc
