#include "sfc_integration/sfc_grid_side_hardware_mock_runner.hpp"

namespace yeet::application::sfc::integration {

void sfc_grid_side_hardware_mock_runner::register_datamodel(yeet::telemetry::parameter_handler& handler)
{
    // Delegate to the datamodel's registration method
    sfc_grid_side_hardware_mock_datamodel datamodel(&_inputs, &_outputs, &_parameters);
    datamodel.register_data_model(handler);
}

sfc_grid_side_hardware_mock_runner::sfc_grid_side_hardware_mock_runner(duration_t sampling_period)
    : base_application(sampling_period), _hardware_mock(sampling_period)
{
}

void sfc_grid_side_hardware_mock_runner::initialize()
{
    _hardware_mock.reset();
    set_parameters();
    set_parameter_callbacks();
}

void sfc_grid_side_hardware_mock_runner::run()
{
    // Read raw analog signals from the inputs
    const auto grid_voltage_abc = _inputs.grid_voltage_abc.read();
    const auto grid_current_abc = _inputs.grid_current_abc.read();

    // Convert line-to-neutral voltages to line-to-line for the mock hardware interface
    const real_t v_ab = grid_voltage_abc.a().value() - grid_voltage_abc.b().value();
    const real_t v_bc = grid_voltage_abc.b().value() - grid_voltage_abc.c().value();

    // Pass the raw analog values to the hardware mock for processing
    _hardware_mock.update(v_ab, v_bc, grid_current_abc.a().value(), grid_current_abc.c().value());

    // Write the quantized ADC counts to the outputs
    _outputs.v_ab_counts = _hardware_mock.get_v_ab_counts();
    _outputs.v_bc_counts = _hardware_mock.get_v_bc_counts();
    _outputs.i_a_counts = _hardware_mock.get_i_a_counts();
    _outputs.i_c_counts = _hardware_mock.get_i_c_counts();
}

void sfc_grid_side_hardware_mock_runner::stop() { _hardware_mock.reset(); }

void sfc_grid_side_hardware_mock_runner::set_parameters()
{
    _hardware_mock.configure_voltage_adc(_parameters.voltage_adc_gain.read(),
        _parameters.voltage_adc_cutoff_freq.read(), _parameters.voltage_adc_bits.read(),
        _parameters.voltage_adc_full_scale.read());

    _hardware_mock.configure_current_adc(_parameters.current_adc_gain.read(),
        _parameters.current_adc_cutoff_freq.read(), _parameters.current_adc_bits.read(),
        _parameters.current_adc_full_scale.read());

    // Apply transformer ratios
    _hardware_mock.configure_transformer_ratios(_parameters.pt_ratio.read(), _parameters.ct_ratio.read());
}

void sfc_grid_side_hardware_mock_runner::set_parameter_callbacks()
{
    // Register a callback to update hardware settings if parameters change during runtime
    auto callback = [&]() { this->set_parameters(); };

    _parameters.voltage_adc_gain.register_callback(callback);
    _parameters.voltage_adc_cutoff_freq.register_callback(callback);
    _parameters.voltage_adc_bits.register_callback(callback);
    _parameters.voltage_adc_full_scale.register_callback(callback);

    _parameters.current_adc_gain.register_callback(callback);
    _parameters.current_adc_cutoff_freq.register_callback(callback);
    _parameters.current_adc_bits.register_callback(callback);
    _parameters.current_adc_full_scale.register_callback(callback);
    _parameters.pt_ratio.register_callback(callback);
    _parameters.ct_ratio.register_callback(callback);
}

}  // namespace yeet::application::sfc::integration