#include "sfc_integration/sfc_dc_link_measurements_runner.hpp"

namespace yeet::application::sfc::integration {

void sfc_dc_link_measurements_runner::initialize()
{
    set_parameters();
    set_parameter_callback();
}

void sfc_dc_link_measurements_runner::run()
{
    _sfc_dc_link_measurements.update(_per_unit_system->get_dc_link_bases(), {}, _inputs.dc_current);

    // _outputs.dc_voltage_pu = _sfc_dc_link_measurements.get_voltage_pu();
    _outputs.dc_current_pu = _sfc_dc_link_measurements.get_current_pu();

    _sfc_dc_link_filtering.update(
        _sfc_dc_link_measurements.get_voltage_pu(), _sfc_dc_link_measurements.get_current_pu());
    _outputs.dc_current_filtered_pu = _sfc_dc_link_filtering.get_filtered_current_pu();

    _outputs.dc_current_heavy_filtered_pu = _sfc_dc_link_filtering.get_heavy_filtered_current_pu();
}

void sfc_dc_link_measurements_runner::stop() {}

void sfc_dc_link_measurements_runner::set_parameters()
{
    _sfc_dc_link_filtering.configure_current_filter(_parameters.filter_cutoff_frequency);
    _sfc_dc_link_filtering.configure_heavy_current_filter(_parameters.heavy_filter_cutoff_frequency);
}

void sfc_dc_link_measurements_runner::set_parameter_callback()
{
    _parameters.filter_cutoff_frequency.register_callback([&]() { set_parameters(); });
    _parameters.heavy_filter_cutoff_frequency.register_callback([&]() { set_parameters(); });
}

}  // namespace yeet::application::sfc::integration