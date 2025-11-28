#pragma once

#include <memory>

#include "sfc_plant_model_voltage_behind_reactance.hpp"

namespace yeet::application::sfc {

enum class sfc_topology_type { internal_model, simulink_generated };

class sfc_plant_model_builder {
public:
    sfc_plant_model_builder& with_topology(sfc_topology_type type);
    sfc_plant_model_builder& with_electrical_parameters(const sfc_plant_model_electrical_parameters& params);
    sfc_plant_model_builder& with_mechanical_parameters(const sfc_plant_model_mechanical_parameters& params);
    sfc_plant_model_builder& with_voltage_control_parameters(const sfc_plant_model_voltage_control_parameters& params);

    std::unique_ptr<sfc_plant_model_voltage_behind_reactance> build();

private:
    sfc_topology_type _topology_type = sfc_topology_type::internal_model;
    sfc_plant_model_electrical_parameters _electrical_params;
    sfc_plant_model_mechanical_parameters _mechanical_params;
    sfc_plant_model_voltage_control_parameters _voltage_control_params;
    bool _electrical_params_set = false;
    bool _mechanical_params_set = false;
    bool _voltage_control_params_set = false;
};

}  // namespace yeet::application::sfc
