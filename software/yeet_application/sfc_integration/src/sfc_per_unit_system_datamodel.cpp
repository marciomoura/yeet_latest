/**
 * @file sfc_per_unit_system_datamodel.cpp
 * @brief Implementation of per-unit system datamodel
 */

#include "sfc_integration/sfc_per_unit_system_datamodel.hpp"

#include <cmath>

#include "sfc/sfc_per_unit_system.hpp"
#include "yeet/utils/parameter_accessors.hpp"

namespace yeet::application::sfc::integration {

void sfc_per_unit_system_datamodel::register_datamodel(
    yeet::model_registry& registry, std::shared_ptr<sfc_per_unit_system> pu_system)
{
    auto configuration_lambda = [this, pu_system]() { *pu_system = this->get_sfc_per_unit_system(); };

    _parameters.grid_voltage_base_rms.register_callback(configuration_lambda);
    _parameters.grid_frequency_base.register_callback(configuration_lambda);
    _parameters.dc_link_current_base.register_callback(configuration_lambda);
    _parameters.machine_voltage_base_rms.register_callback(configuration_lambda);
    _parameters.machine_frequency_base.register_callback(configuration_lambda);
    _parameters.machine_power_base.register_callback(configuration_lambda);
    _parameters.machine_num_poles.register_callback(configuration_lambda);

    yeet::model_registry_entry entry{yeet::signal_registry{}, yeet::parameter_registry{}};

    // Register grid-side base quantities
    entry.parameters.insert(
        "grid_voltage_base_rms", yeet::utils::make_units_accessor(_parameters.grid_voltage_base_rms));
    entry.parameters.insert("grid_frequency_base", yeet::utils::make_units_accessor(_parameters.grid_frequency_base));

    // Register DC link base quantities
    entry.parameters.insert("dc_link_current_base", yeet::utils::make_units_accessor(_parameters.dc_link_current_base));

    // Register machine-side base quantities
    entry.parameters.insert(
        "machine_voltage_base_rms", yeet::utils::make_units_accessor(_parameters.machine_voltage_base_rms));
    entry.parameters.insert(
        "machine_frequency_base", yeet::utils::make_units_accessor(_parameters.machine_frequency_base));
    entry.parameters.insert("machine_power_base", yeet::utils::make_units_accessor(_parameters.machine_power_base));
    entry.parameters.insert("machine_num_poles", yeet::utils::make_units_accessor(_parameters.machine_num_poles));

    // Register the entry with the model registry
    registry.insert("per_unit_system", std::move(entry));
}

sfc_per_unit_system sfc_per_unit_system_datamodel::get_sfc_per_unit_system()
{
    return sfc_per_unit_system(this->_parameters.grid_voltage_base_rms.read(),
        this->_parameters.grid_frequency_base.read(), this->_parameters.dc_link_current_base.read(),
        this->_parameters.machine_power_base.read(), this->_parameters.machine_voltage_base_rms.read(),
        this->_parameters.machine_frequency_base.read(), this->_parameters.machine_num_poles.read());
}

}  // namespace yeet::application::sfc::integration
