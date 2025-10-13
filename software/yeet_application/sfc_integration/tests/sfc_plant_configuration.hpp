#pragma once

#include "sfc/sfc_plant_model_parameters.hpp"
#include "sfc_integration/sfc_plant_model_datamodel.hpp"
#include "sfc_integration/sfc_plant_model_runner.hpp"
#include "sfc_integration/sfc_speed_controller_runner.hpp"
#include "yeet_control/units.hpp"

/**
 * @file sfc_plant_configuration.hpp
 * @brief Common plant model configuration functions for SFC integration tests
 */

namespace yeet::application::sfc::integration::test {

/**
 * @brief Configuration parameters for variable speed plant operation
 */
struct variable_speed_config {
    /// Motor damping factor in per-unit
    damping_coefficient_pu_t damping_factor{0.01f};

    /// Motor inertia constant
    inertia_constant_t inertia_constant{0.5f};

    /// Speed-voltage relationship parameters
    real_t voltage_at_base_speed{0.8f};
    real_t voltage_slope{0.2f};
    real_t voltage_offset{0.0f};

    /// Initial mechanical speed in per-unit
    speed_pu_t initial_speed{1.0f};

    /// Load torque in per-unit
    torque_pu_t load_torque{0.3f};
};

/**
 * @brief Default variable speed configuration for most tests
 */
static constexpr variable_speed_config default_variable_speed_config{};

/**
 * @brief Configures a plant model for variable speed operation with standard test parameters
 *
 * This function applies a common configuration pattern used across multiple SFC tests.
 * It configures both the plant datamodel parameters and the underlying plant model
 * for variable speed operation with speed-dependent voltage control.
 *
 * @param plant_runner The plant runner to configure
 * @param config Configuration parameters (uses default if not specified)
 */
inline void configure_variable_speed_plant(
    sfc_plant_model_runner& plant_runner, const variable_speed_config& config = default_variable_speed_config)
{
    // Configure underlying plant model
    plant_runner.get_plant_model().set_operation_mode(plant_model_machine_operation_mode::variable_speed);
    plant_runner.get_plant_model().set_voltage_control_mode(plant_model_voltage_control_mode::speed_dependent);
    plant_runner.get_plant_model().configure_speed_voltage_relationship(
        config.voltage_at_base_speed, config.voltage_slope, config.voltage_offset);
    plant_runner.get_plant_model().set_initial_mechanical_speed_pu(config.initial_speed);
    plant_runner.get_plant_model().set_load_torque_pu(config.load_torque);

    plant_runner.get_parameters().operation_mode.write(
        sfc_plant_model_datamodel::machine_operation_mode::variable_speed);
    plant_runner.get_parameters().motor_damping_factor.write(config.damping_factor);
    plant_runner.get_parameters().motor_inertia_constant.write(config.inertia_constant);
}

/**
 * @brief Configuration parameters for fixed speed plant operation
 */
struct fixed_speed_config {
    /// Motor damping factor in per-unit
    damping_coefficient_pu_t damping_factor{0.01f};

    /// Motor inertia constant
    inertia_constant_t inertia_constant{0.5f};

    /// Fixed machine frequency
    frequency_t machine_frequency{50.0f};

    /// Fixed machine voltage
    voltage_t machine_voltage{2200.0f};

    /// Load torque in per-unit
    torque_pu_t load_torque{0.3f};
};

/**
 * @brief Default fixed speed configuration
 */
static constexpr fixed_speed_config default_fixed_speed_config{};

/**
 * @brief Configures a plant model for fixed speed operation
 *
 * @param plant_runner The plant runner to configure
 * @param config Configuration parameters (uses default if not specified)
 */
inline void configure_fixed_speed_plant(
    sfc_plant_model_runner& plant_runner, const fixed_speed_config& config = default_fixed_speed_config)
{
    // Configure plant datamodel parameters
    plant_runner.get_parameters().operation_mode.write(sfc_plant_model_datamodel::machine_operation_mode::fixed_speed);

    plant_runner.get_parameters().motor_damping_factor.write(config.damping_factor);
    plant_runner.get_parameters().motor_inertia_constant.write(config.inertia_constant);

    // Configure underlying plant model
    plant_runner.get_plant_model().set_operation_mode(plant_model_machine_operation_mode::fixed_speed);
    plant_runner.get_plant_model().set_machine_parameters(config.machine_frequency, config.machine_voltage);
    plant_runner.get_plant_model().set_load_torque_pu(config.load_torque);
}

}  // namespace yeet::application::sfc::integration::test
