#include "sfc_integration/sfc_machine_side_observer_runner.hpp"

namespace yeet::application::sfc::integration {

void sfc_machine_side_observer_runner::register_datamodel(yeet::model_registry& registry)
{
    // Delegate to the datamodel's registration method
    sfc_machine_side_observer_datamodel datamodel(&_inputs, &_outputs, &_parameters);
    datamodel.register_data_model(registry);
}

void sfc_machine_side_observer_runner::initialize()
{
    // Configure PLL with parameters from datamodel
    _sfc_machine_side_observer.configure_pll(_parameters.kp, _parameters.ti, _parameters.nominal_frequency);

    // Configure frequency filter
    _sfc_machine_side_observer.configure_frequency_filter(_parameters.frequency_filter_cutoff);

    // Set initial speed
    _sfc_machine_side_observer.reset(_parameters.initial_frequency);

    // Initialize edge detector state
    _preset_previous = false;
}

void sfc_machine_side_observer_runner::run()
{
    const bool release_observer = _inputs.release_observer.read();
    const bool preset_observer = _inputs.preset_observer.read();
    const angle_wrapped preset_angle = _inputs.preset_angle.read();

    // Edge detection: detect rising edge on preset signal
    const bool preset_rising_edge = preset_observer && !_preset_previous;
    _preset_previous = preset_observer;

    if (!release_observer) {
        // Observer disabled - reset estimation to zero
        _sfc_machine_side_observer.reset(frequency_pu_t{0.0});
        // Write zero outputs
        _outputs.estimated_voltage_magnitude = voltage_pu_t{0.0};
        _outputs.estimated_frequency = frequency_t{0.0};
        _outputs.estimated_speed = speed_pu_t{0.0};
        _outputs.estimated_frequency_pu = frequency_pu_t{0.0};
        _outputs.estimated_reference_angle = angle_wrapped{0.0};
        _outputs.estimated_voltage_alphabeta_filtered = alphabeta<voltage_pu_t>{voltage_pu_t{0.0}, voltage_pu_t{0.0}};
        // Reset edge detector
        _preset_previous = false;
        return;
    }

    // Check for preset rising edge - execute preset before normal update
    if (preset_rising_edge) {
        _sfc_machine_side_observer.preset(preset_angle);
    }

    // Normal observer execution when enabled
    _sfc_machine_side_observer.update(_inputs.machine_voltage_pu.read());

    _outputs.estimated_voltage_magnitude = _sfc_machine_side_observer.get_estimated_voltage_magnitude();
    _outputs.estimated_frequency = _sfc_machine_side_observer.get_estimated_frequency();
    _outputs.estimated_speed = speed_pu_t{_sfc_machine_side_observer.get_estimated_frequency_pu().value()};
    _outputs.estimated_frequency_pu = _sfc_machine_side_observer.get_estimated_frequency_pu();
    _outputs.estimated_reference_angle = _sfc_machine_side_observer.get_estimated_angle();
    _outputs.estimated_voltage_alphabeta_filtered = _sfc_machine_side_observer.get_filtered_voltage_alphabeta();
}
void sfc_machine_side_observer_runner::stop()
{
    _sfc_machine_side_observer.reset();
    _preset_previous = false;
}

}  // namespace yeet::application::sfc::integration