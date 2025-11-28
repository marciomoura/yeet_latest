#include "sfc/sfc_plant_model_builder.hpp"

#include "sfc/sfc_plant_internal_model.hpp"
#include "sfc/sfc_plant_simulink_model.hpp"

namespace yeet::application::sfc {

sfc_plant_model_builder& sfc_plant_model_builder::with_topology(sfc_topology_type type)
{
    _topology_type = type;
    return *this;
}

sfc_plant_model_builder& sfc_plant_model_builder::with_electrical_parameters(
    const sfc_plant_model_electrical_parameters& params)
{
    _electrical_params = params;
    _electrical_params_set = true;
    return *this;
}

sfc_plant_model_builder& sfc_plant_model_builder::with_mechanical_parameters(
    const sfc_plant_model_mechanical_parameters& params)
{
    _mechanical_params = params;
    _mechanical_params_set = true;
    return *this;
}

sfc_plant_model_builder& sfc_plant_model_builder::with_voltage_control_parameters(
    const sfc_plant_model_voltage_control_parameters& params)
{
    _voltage_control_params = params;
    _voltage_control_params_set = true;
    return *this;
}

std::unique_ptr<sfc_plant_model_voltage_behind_reactance> sfc_plant_model_builder::build()
{
    if (!_electrical_params_set) {
        _electrical_params = sfc_plant_model_voltage_behind_reactance::create_default_electrical_parameters();
    }
    if (!_mechanical_params_set) {
        _mechanical_params = sfc_plant_model_voltage_behind_reactance::create_default_mechanical_parameters();
    }
    if (!_voltage_control_params_set) {
        _voltage_control_params = sfc_plant_model_voltage_behind_reactance::create_default_voltage_control_parameters();
    }

    std::unique_ptr<sfc_topology_interface> topology;
    if (_topology_type == sfc_topology_type::internal_model) {
        topology = std::make_unique<sfc_plant_internal_model>(_electrical_params);
    }
    else {
        topology = std::make_unique<sfc_plant_simulink_model>(_electrical_params);
    }

    return std::make_unique<sfc_plant_model_voltage_behind_reactance>(
        std::move(topology), _electrical_params, _mechanical_params, _voltage_control_params);
}

}  // namespace yeet::application::sfc
