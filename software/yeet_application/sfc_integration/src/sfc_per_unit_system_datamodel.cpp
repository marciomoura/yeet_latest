/**
 * @file sfc_per_unit_system_datamodel.cpp
 * @brief Implementation of per-unit system datamodel
 */

#include "sfc_integration/sfc_per_unit_system_datamodel.hpp"

#include <cmath>

#include "sfc/sfc_per_unit_system.hpp"
#include "yeet/parameter_handler.hpp"
#include "yeet/utils/parameter_accessors.hpp"

namespace yeet::application::sfc::integration {

void sfc_per_unit_system_datamodel::register_datamodel(
    yeet::telemetry::parameter_handler& handler, std::shared_ptr<sfc_per_unit_system> pu_system)
{
    static constexpr std::string_view component_name = "per_unit_system";
    auto configuration_lambda = [this, pu_system]() { *pu_system = this->get_sfc_per_unit_system(); };

    _parameters.grid_voltage_base_rms.register_callback(configuration_lambda);
    _parameters.grid_frequency_base.register_callback(configuration_lambda);
    _parameters.dc_link_current_base.register_callback(configuration_lambda);
    _parameters.machine_voltage_base_rms.register_callback(configuration_lambda);
    _parameters.machine_frequency_base.register_callback(configuration_lambda);
    _parameters.machine_power_base.register_callback(configuration_lambda);
    _parameters.machine_num_poles.register_callback(configuration_lambda);

    // Register grid-side base quantities
    handler.register_parameter(
        component_name, "grid_voltage_base_rms", yeet::utils::make_units_accessor(_parameters.grid_voltage_base_rms));
    handler.register_parameter(
        component_name, "grid_frequency_base", yeet::utils::make_units_accessor(_parameters.grid_frequency_base));

    // Register DC link base quantities
    handler.register_parameter(
        component_name, "dc_link_current_base", yeet::utils::make_units_accessor(_parameters.dc_link_current_base));

    // Register machine-side base quantities
    handler.register_parameter(component_name, "machine_voltage_base_rms",
        yeet::utils::make_units_accessor(_parameters.machine_voltage_base_rms));
    handler.register_parameter(
        component_name, "machine_frequency_base", yeet::utils::make_units_accessor(_parameters.machine_frequency_base));
    handler.register_parameter(
        component_name, "machine_power_base", yeet::utils::make_units_accessor(_parameters.machine_power_base));
    handler.register_parameter(
        component_name, "machine_num_poles", yeet::utils::make_units_accessor(_parameters.machine_num_poles));
}

sfc_per_unit_system sfc_per_unit_system_datamodel::get_sfc_per_unit_system()
{
    return sfc_per_unit_system(this->_parameters.grid_voltage_base_rms.read(),
        this->_parameters.grid_frequency_base.read(), this->_parameters.dc_link_current_base.read(),
        this->_parameters.machine_power_base.read(), this->_parameters.machine_voltage_base_rms.read(),
        this->_parameters.machine_frequency_base.read(), this->_parameters.machine_num_poles.read());
}

}  // namespace yeet::application::sfc::integration
