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
}

void sfc_machine_side_observer_runner::run()
{
    _sfc_machine_side_observer.update(_inputs.machine_voltage_pu.read());

    _outputs.estimated_voltage_magnitude = _sfc_machine_side_observer.get_estimated_voltage_magnitude();
    _outputs.estimated_frequency = _sfc_machine_side_observer.get_estimated_frequency();
    _outputs.estimated_speed = speed_pu_t{_sfc_machine_side_observer.get_estimated_frequency_pu().value()};
    _outputs.estimated_frequency_pu = _sfc_machine_side_observer.get_estimated_frequency_pu();
    _outputs.estimated_position = _sfc_machine_side_observer.get_estimated_angle();
    _outputs.filtered_voltage_alphabeta = _sfc_machine_side_observer.get_filtered_voltage_alphabeta();
}

void sfc_machine_side_observer_runner::stop() { _sfc_machine_side_observer.reset(); }

}  // namespace yeet::application::sfc::integration