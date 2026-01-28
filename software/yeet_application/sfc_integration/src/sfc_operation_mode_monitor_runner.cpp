#include "sfc_integration/sfc_operation_mode_monitor_runner.hpp"

namespace yeet::application::sfc::integration {

sfc_operation_mode_monitor_runner::sfc_operation_mode_monitor_runner()
    : base_application(sampling_time), _discontinuous_conduction_detector(sampling_time)
{
}

void sfc_operation_mode_monitor_runner::register_datamodel(yeet::telemetry::parameter_handler& handler)
{
    // Delegate to the datamodel's registration method
    sfc_operation_mode_monitor_datamodel datamodel(&_inputs, &_outputs, &_parameters);
    datamodel.register_data_model(handler);
}

void sfc_operation_mode_monitor_runner::initialize()
{
    _mode_detector.reset();
    _discontinuous_conduction_detector.reset();

    set_parameters();
    set_parameter_callback();
}

void sfc_operation_mode_monitor_runner::run()
{
    const bool enable = _inputs.enable.read();

    // Update the detectors with the latest inputs
    _mode_detector.update(enable, _inputs.measured_speed.read());

    // current_pu_t current_pu = max(_inputs.dc_link_current.read() - current_pu_t{0.05}, current_pu_t{0.0});

    _discontinuous_conduction_detector.update(_inputs.dc_link_current.read());

    // Write the results to the output signals
    _outputs.operating_mode = _mode_detector.get_operating_mode();
    _outputs.has_mode_changed = _mode_detector.has_mode_changed();
    _outputs.current_is_zero = _discontinuous_conduction_detector.get_is_current_zero();
    _outputs.is_discontinuous_mode = _discontinuous_conduction_detector.get_is_discontinuous_mode();
}

void sfc_operation_mode_monitor_runner::stop()
{
    _mode_detector.reset();
    _discontinuous_conduction_detector.reset();
}

void sfc_operation_mode_monitor_runner::set_parameters()
{
    // Configure mode detector
    _mode_detector.configure_mode_handover_speed_pu(_parameters.mode_handover_speed_pu.read());
    _mode_detector.configure_hysteresis_factor(_parameters.mode_hysteresis_factor.read());
    _mode_detector.configure_force_natural_commutation(_parameters.force_to_natural_commutation.read());
    _mode_detector.configure_force_forced_commutation(_parameters.force_to_forced_commutation.read());

    // Configure discontinuous conduction detector
    _discontinuous_conduction_detector.configure(_parameters.zero_current_threshold_pu.read(),
        _parameters.is_zero_on_delay_s.read(), _parameters.discontinuous_on_delay_s.read(),
        _parameters.discontinuous_off_delay_s.read());
}

void sfc_operation_mode_monitor_runner::set_parameter_callback()
{
    // Register callbacks for mode detector parameters
    _parameters.mode_handover_speed_pu.register_callback(
        [&]() { _mode_detector.configure_mode_handover_speed_pu(_parameters.mode_handover_speed_pu.read()); });
    _parameters.mode_hysteresis_factor.register_callback(
        [&]() { _mode_detector.configure_hysteresis_factor(_parameters.mode_hysteresis_factor.read()); });
    _parameters.force_to_natural_commutation.register_callback(
        [&]() { _mode_detector.configure_force_natural_commutation(_parameters.force_to_natural_commutation.read()); });
    _parameters.force_to_forced_commutation.register_callback(
        [&]() { _mode_detector.configure_force_forced_commutation(_parameters.force_to_forced_commutation.read()); });

    // Register callbacks for discontinuous conduction detector parameters
    auto configure_dcm = [&]() {
        _discontinuous_conduction_detector.configure(_parameters.zero_current_threshold_pu.read(),
            _parameters.is_zero_on_delay_s.read(), _parameters.discontinuous_on_delay_s.read(),
            _parameters.discontinuous_off_delay_s.read());
    };
    _parameters.zero_current_threshold_pu.register_callback(configure_dcm);
    _parameters.is_zero_on_delay_s.register_callback(configure_dcm);
    _parameters.discontinuous_on_delay_s.register_callback(configure_dcm);
    _parameters.discontinuous_off_delay_s.register_callback(configure_dcm);
}

}  // namespace yeet::application::sfc::integration