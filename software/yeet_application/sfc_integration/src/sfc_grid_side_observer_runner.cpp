#include "sfc_integration/sfc_grid_side_observer_runner.hpp"

namespace yeet::application::sfc::integration {

void sfc_grid_side_observer_runner::register_datamodel(yeet::model_registry& registry)
{
    // Delegate to the datamodel's registration method
    sfc_grid_side_observer_datamodel datamodel(&_inputs, &_outputs, &_parameters);
    datamodel.register_data_model(registry);
}

void sfc_grid_side_observer_runner::initialize()
{
    set_parameters();
    set_parameter_callback();
    _sfc_grid_side_observer.reset(_parameters.initial_frequency);
}

void sfc_grid_side_observer_runner::run()
{
    _sfc_grid_side_observer.update(_inputs.grid_voltage_abc);

    _outputs.estimated_reference_angle = _sfc_grid_side_observer.get_estimated_angle();
    _outputs.estimated_frequency = _sfc_grid_side_observer.get_estimated_frequency();
    _outputs.estimated_frequency_pu = _sfc_grid_side_observer.get_estimated_frequency_pu();
    _outputs.estimated_voltage_magnitude = _sfc_grid_side_observer.get_estimated_voltage_magnitude();
    _outputs.estimated_voltage_alphabeta_filtered = _sfc_grid_side_observer.get_filtered_voltage_alphabeta();
    _outputs.observer_error = _sfc_grid_side_observer.get_observer_error();
}

void sfc_grid_side_observer_runner::stop() { _sfc_grid_side_observer.reset(); }

void sfc_grid_side_observer_runner::set_parameters()
{
    _sfc_grid_side_observer.configure_pll(_parameters.kp, _parameters.ti, _parameters.nominal_frequency);
    _sfc_grid_side_observer.configure_frequency_filter(_parameters.frequency_filter_cutoff);
}

void sfc_grid_side_observer_runner::set_parameter_callback()
{
    _parameters.kp.register_callback([&]() {
        _sfc_grid_side_observer.configure_pll(_parameters.kp, _parameters.ti, _parameters.nominal_frequency);
    });
    _parameters.ti.register_callback([&]() {
        _sfc_grid_side_observer.configure_pll(_parameters.kp, _parameters.ti, _parameters.nominal_frequency);
    });
    _parameters.nominal_frequency.register_callback([&]() {
        _sfc_grid_side_observer.configure_pll(_parameters.kp, _parameters.ti, _parameters.nominal_frequency);
    });
    _parameters.frequency_filter_cutoff.register_callback(
        [&]() { _sfc_grid_side_observer.configure_frequency_filter(_parameters.frequency_filter_cutoff); });
}

}  // namespace yeet::application::sfc::integration