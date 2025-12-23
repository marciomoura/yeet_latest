#include "sfc_integration/sfc_machine_side_observer_runner.hpp"

namespace yeet::application::sfc::integration {

void sfc_machine_side_observer_runner::register_datamodel(yeet::telemetry::parameter_handler& handler)
{
    // Delegate to the datamodel's registration method
    sfc_machine_side_observer_datamodel datamodel(&_inputs, &_outputs, &_parameters);
    datamodel.register_data_model(handler);
}

void sfc_machine_side_observer_runner::initialize()
{
    set_parameters();
    set_parameter_callback();

    // Set initial speed
    _sfc_machine_side_observer.reset(_parameters.initial_frequency);
}

void sfc_machine_side_observer_runner::set_parameters()
{
    _sfc_machine_side_observer.configure_pll(_parameters.kp, _parameters.ti, _parameters.nominal_frequency);
    _sfc_machine_side_observer.configure_frequency_filter(_parameters.frequency_filter_cutoff);
}

void sfc_machine_side_observer_runner::set_parameter_callback()
{
    _parameters.kp.register_callback([&]() {
        _sfc_machine_side_observer.configure_pll(_parameters.kp, _parameters.ti, _parameters.nominal_frequency);
    });
    _parameters.ti.register_callback([&]() {
        _sfc_machine_side_observer.configure_pll(_parameters.kp, _parameters.ti, _parameters.nominal_frequency);
    });
    _parameters.nominal_frequency.register_callback([&]() {
        _sfc_machine_side_observer.configure_pll(_parameters.kp, _parameters.ti, _parameters.nominal_frequency);
    });
    _parameters.frequency_filter_cutoff.register_callback(
        [&]() { _sfc_machine_side_observer.configure_frequency_filter(_parameters.frequency_filter_cutoff); });
}

void sfc_machine_side_observer_runner::run()
{
    if (!_inputs.release_observer.read()) {
        // Observer disabled - reset estimation to zero
        _sfc_machine_side_observer.reset(frequency_pu_t{0.0});
        _previous_preset_observer = false;
        // Write zero outputs
        _outputs.estimated_voltage_magnitude = voltage_pu_t{0.0};
        _outputs.estimated_frequency = frequency_t{0.0};
        _outputs.estimated_speed = speed_pu_t{0.0};
        _outputs.estimated_frequency_pu = frequency_pu_t{0.0};
        _outputs.estimated_reference_angle = angle_wrapped{0.0};
        _outputs.estimated_voltage_alphabeta_filtered = alphabeta<voltage_pu_t>{voltage_pu_t{0.0}, voltage_pu_t{0.0}};
        return;
    }

    const bool preset_requested = _inputs.preset_observer.read();
    const bool preset_rising_edge = preset_requested && !_previous_preset_observer;
    _previous_preset_observer = preset_requested;

    if (preset_rising_edge) {
        _sfc_machine_side_observer.preset_for_bumpless_transfer(
            _inputs.preset_angle.read(), _inputs.preset_frequency.read(), _inputs.machine_voltage_pu.read());
    }

    // Always execute the observer update when enabled, even during the preset window.
    _sfc_machine_side_observer.update(_inputs.machine_voltage_pu.read());

    _outputs.estimated_voltage_magnitude = _sfc_machine_side_observer.get_estimated_voltage_magnitude();

    if (preset_requested) {
        const auto preset_frequency_pu = _inputs.preset_frequency.read();
        _outputs.estimated_reference_angle = _inputs.preset_angle.read();
        _outputs.estimated_frequency_pu = preset_frequency_pu;
        _outputs.estimated_frequency = to_si(preset_frequency_pu, _parameters.nominal_frequency.read());
        _outputs.estimated_speed = speed_pu_t{preset_frequency_pu.value()};
    }
    else {
        _outputs.estimated_frequency = _sfc_machine_side_observer.get_estimated_frequency();
        _outputs.estimated_speed = speed_pu_t{_sfc_machine_side_observer.get_estimated_frequency_pu().value()};
        _outputs.estimated_frequency_pu = _sfc_machine_side_observer.get_estimated_frequency_pu();
        _outputs.estimated_reference_angle = _sfc_machine_side_observer.get_estimated_angle();
    }

    _outputs.estimated_voltage_alphabeta_filtered = _sfc_machine_side_observer.get_filtered_voltage_alphabeta();
    _outputs.observer_error = _sfc_machine_side_observer.get_observer_error();
}
void sfc_machine_side_observer_runner::stop() { _sfc_machine_side_observer.reset(); }

}  // namespace yeet::application::sfc::integration