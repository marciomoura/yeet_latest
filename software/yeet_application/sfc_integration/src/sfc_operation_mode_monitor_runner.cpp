#include "sfc_integration/sfc_operation_mode_monitor_runner.hpp"

namespace yeet::application::sfc::integration {

sfc_operation_mode_monitor_runner::sfc_operation_mode_monitor_runner()
    : base_application(sampling_time), _discontinuous_conduction_detector(sampling_time)
{
}

void sfc_operation_mode_monitor_runner::register_datamodel(yeet::model_registry& registry)
{
    // Delegate to the datamodel's registration method
    sfc_operation_mode_monitor_datamodel datamodel(&_inputs, &_outputs, &_parameters);
    datamodel.register_data_model(registry);
}

void sfc_operation_mode_monitor_runner::initialize()
{
    set_parameters();
    set_parameter_callback();

    _mode_detector.reset();
    _discontinuous_conduction_detector.reset();
}

void sfc_operation_mode_monitor_runner::run()
{
    const bool enable = _inputs.enable.read();

    // Update the detectors with the latest inputs
    _mode_detector.update(enable, _inputs.measured_speed_pu.read());

    if (enable) {
        _discontinuous_conduction_detector.update(_inputs.dc_link_current_pu.read());
    }
    else {
        _discontinuous_conduction_detector.reset();
    }

    // Write the results to the output signals
    _outputs.operating_mode = _mode_detector.get_operating_mode();
    _outputs.has_mode_changed = _mode_detector.has_mode_changed();
    _outputs.is_current_zero = _discontinuous_conduction_detector.get_is_current_zero();
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
    _mode_detector.configure_mode_handover_speed_pu(_parameters.mode_handover_speed_pu);
    _mode_detector.configure_hysteresis_factor(_parameters.mode_hysteresis_factor);

    // Configure discontinuous conduction detector
    _discontinuous_conduction_detector.configure(_parameters.zero_current_threshold_pu, _parameters.is_zero_on_delay_s,
        _parameters.discontinuous_on_delay_s, _parameters.discontinuous_off_delay_s);
}

void sfc_operation_mode_monitor_runner::set_parameter_callback()
{
    // For this implementation, we'll handle parameter changes through periodic calls to set_parameters()
    // rather than using callbacks. This is simpler and sufficient for most use cases.
}

}  // namespace yeet::application::sfc::integration