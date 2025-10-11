#pragma once

#include "yeet_control/units.hpp"

namespace yeet::application::sfc {

/**
 * @brief Defines the operational mode for the machine side of the plant.
 */
enum class plant_model_machine_operation_mode {
    fixed_speed,    // Machine runs at a constant, predefined frequency.
    variable_speed  // Machine speed is determined by the mechanical load model.
};

/**
 * @brief Defines the voltage control mode for the machine side.
 */
enum class plant_model_voltage_control_mode {
    fixed_voltage,    // Machine voltage is fixed at nominal value.
    speed_dependent,  // Machine voltage varies with speed using speed-voltage relationship.
    external_control  // Machine voltage is set externally (e.g., from excitation controller).
};

/**
 * @brief Electrical parameters configuration structure for SFC plant models
 */
struct sfc_plant_model_electrical_parameters {
    frequency_t grid_frequency;
    frequency_t machine_frequency;
    voltage_t grid_voltage_rms;
    voltage_t machine_voltage_rms;
    angle_t grid_firing_angle_deg;
    angle_t machine_firing_angle_deg;
    resistance_t grid_side_resistance;
    inductance_t grid_side_inductance;
    resistance_t machine_side_resistance;
    inductance_t machine_side_inductance;
    current_t base_dc_link_current;
    power_t machine_rated_power_va;
    int machine_num_poles;
};

/**
 * @brief Mechanical parameters configuration structure
 */
struct sfc_plant_model_mechanical_parameters {
    inertia_constant_t inertia_constant_h;
    damping_coefficient_pu_t damping_factor_d;
};

/**
 * @brief Voltage control parameters configuration structure
 */
struct sfc_plant_model_voltage_control_parameters {
    real_t voltage_speed_coefficient;
    real_t voltage_offset;
};

}  // namespace yeet::application::sfc