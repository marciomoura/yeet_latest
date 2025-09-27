#include "sfc_integration/sfc_machine_side_hardware_mock_runner.hpp"

namespace yeet::application::sfc::integration {

sfc_machine_side_hardware_mock_runner::sfc_machine_side_hardware_mock_runner(duration_t sampling_period)
    : base_application(sampling_period), _hardware_mock(sampling_period)
{
}

void sfc_machine_side_hardware_mock_runner::initialize()
{
    _hardware_mock.reset();
    set_parameters();
    set_parameter_callbacks();
}

void sfc_machine_side_hardware_mock_runner::run()
{
    // Read raw analog signals from the inputs
    const auto machine_voltage = _inputs.machine_voltage.read();

    // Convert line-to-neutral voltages to line-to-line for the mock hardware interface
    const real_t v_ab = machine_voltage.a().value() - machine_voltage.b().value();
    const real_t v_bc = machine_voltage.b().value() - machine_voltage.c().value();

    // Pass the raw analog values to the hardware mock for processing
    _hardware_mock.update(v_ab, v_bc);

    // Write the quantized ADC counts to the outputs
    _outputs.v_ab_counts = _hardware_mock.get_v_ab_counts();
    _outputs.v_bc_counts = _hardware_mock.get_v_bc_counts();
}

void sfc_machine_side_hardware_mock_runner::stop() { _hardware_mock.reset(); }

void sfc_machine_side_hardware_mock_runner::set_parameters()
{
    _hardware_mock.configure_voltage_adc(_parameters.voltage_adc_gain.read(),
        _parameters.voltage_adc_cutoff_freq.read(), _parameters.voltage_adc_bits.read(),
        _parameters.voltage_adc_full_scale.read());
    _hardware_mock.configure_transformer_ratio(_parameters.pt_ratio.read());
}

void sfc_machine_side_hardware_mock_runner::set_parameter_callbacks()
{
    // Register a callback to update hardware settings if parameters change during runtime
    auto callback = [&]() { this->set_parameters(); };

    _parameters.voltage_adc_gain.register_callback(callback);
    _parameters.voltage_adc_cutoff_freq.register_callback(callback);
    _parameters.voltage_adc_bits.register_callback(callback);
    _parameters.voltage_adc_full_scale.register_callback(callback);
    _parameters.pt_ratio.register_callback(callback);
}

}  // namespace yeet::application::sfc::integration