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

/**
 * @brief Transformer parameters for input grid-side transformer
 */
struct sfc_plant_model_transformer_parameters {
    power_t nominal_power_va;
    frequency_t nominal_frequency_hz;
    voltage_t hv_phase_to_phase_rms_voltage;
    resistance_pu_t hv_resistance_pu;
    inductance_pu_t hv_inductance_pu;
    voltage_t lv_phase_to_phase_rms_voltage;
    resistance_pu_t lv_resistance_pu;
    inductance_pu_t lv_inductance_pu;
    resistance_pu_t magnetization_resistance_pu;
    inductance_pu_t magnetization_inductance_pu;
};

/**
 * @brief DC link parameters
 */
struct sfc_plant_model_dc_link_parameters {
    resistance_t resistance;
    inductance_t inductance;
};

/**
 * @brief Synchronous machine electrical parameters
 */
struct sfc_plant_model_machine_electrical_parameters {
    power_t nominal_power_va;
    voltage_t nominal_phase_to_phase_voltage;
    frequency_t nominal_frequency_hz;
    resistance_t stator_resistance;
    inductance_t stator_leakage_inductance;
    inductance_t magnetizing_inductance_d;
    inductance_t magnetizing_inductance_q;
    inductance_t canay_inductance;
    resistance_t damper_resistance_d;
    inductance_t damper_leakage_inductance_d;
    resistance_t damper_resistance_q;
    inductance_t damper_leakage_inductance_q;
    resistance_t field_resistance;
    inductance_t field_leakage_inductance;
};

/**
 * @brief Synchronous machine mechanical parameters
 */
struct sfc_plant_model_machine_mechanical_parameters {
    real_t inertia_coefficient;  // J in kg⋅m²
    int pole_pairs;
};

/**
 * @brief Measurement scaling ratios for PT/CT transformers
 */
struct sfc_plant_model_measurement_parameters {
    real_t grid_pt_ratio;   // Potential transformer ratio (grid side)
    real_t grid_ct_ratio;   // Current transformer ratio (grid side)
    real_t motor_pt_ratio;  // Potential transformer ratio (motor side)
    real_t motor_ct_ratio;  // Current transformer ratio (motor side)
};

/**
 * @brief Machine state outputs grouped by category
 */
struct machine_mechanical_state {
    speed_t speed;               // Mechanical speed
    torque_t electrical_torque;  // Electrical torque
    angle_t mechanical_angle;    // Mechanical rotor angle
    torque_t load_torque;        // Load torque
    power_t electrical_power;    // Electrical power
};

/**
 * @brief Machine stator electrical state in dq frame
 */
struct machine_stator_state_dq {
    current_t current_d;
    current_t current_q;
    voltage_t voltage_d;
    voltage_t voltage_q;
};

/**
 * @brief Machine rotor electrical state
 */
struct machine_rotor_state {
    magnetic_flux_t flux_d;
    magnetic_flux_t flux_q;
    current_t field_current_d;
    current_t damper_current_q1;
    current_t damper_current_q2;
    current_t damper_current_d;
};

}  // namespace yeet::application::sfc