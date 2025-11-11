#pragma once

#include <optional>

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

    /// Initial mechanical speed in per-unit
    speed_pu_t initial_speed{0.01f};

    /// Load torque in per-unit
    torque_pu_t load_torque{0.2f};

    /// Linear load coefficient k1 [pu/pu] for T_load = k1*omega
    real_t linear_load_coefficient{0.0f};

    /// Quadratic load coefficient k2 [pu/pu^2] for T_load = k2*omega^2
    /// For gas turbine compressors, typical: 0.2-0.4
    real_t quadratic_load_coefficient{0.0f};

    /// Voltage control mode (set to nullopt to preserve existing mode)
    std::optional<plant_model_voltage_control_mode> voltage_control_mode{
        plant_model_voltage_control_mode::variable_speed};
};

/**
 * @brief Default variable speed configuration for most tests
 */
static constexpr variable_speed_config default_variable_speed_config{};

/**
 * @brief Gas turbine starter configuration with realistic compressor load characteristics
 *
 * This configuration models a large gas turbine with:
 * - High inertia (3.0 s): Large rotor and compressor assembly
 * - Low damping (0.02 pu): Low friction bearings
 * - Dominant quadratic load (0.3 pu/pu^2): Compressor aerodynamic drag proportional to speed^2
 * - Low external load (0.0 pu): No mechanical load at standstill
 * - Variable speed voltage control: Back-EMF proportional to speed and flux
 */
inline variable_speed_config create_gas_turbine_config()
{
    variable_speed_config config{};
    config.inertia_constant = inertia_constant_t{3.0f};      // Large rotational mass
    config.damping_factor = damping_coefficient_pu_t{0.1f};  // Medium friction
    config.linear_load_coefficient = 0.0f;                   // Minimal viscous losses
    config.quadratic_load_coefficient = 0.3f;                // Compressor load ∝ speed²
    config.load_torque = torque_pu_t{0.0f};                  // No constant load
    config.initial_speed = speed_pu_t{0.001f};               // Near standstill
    config.voltage_control_mode = plant_model_voltage_control_mode::variable_speed;
    return config;
}

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

    // Set voltage control mode in both datamodel parameter AND plant model
    // If config.voltage_control_mode is nullopt, preserve the builder's setting (variable_speed)
    if (config.voltage_control_mode.has_value()) {
        // Update both the plant model AND the datamodel parameter for consistency
        plant_runner.get_plant_model().set_voltage_control_mode(config.voltage_control_mode.value());

        // Map to datamodel enum
        if (config.voltage_control_mode.value() == plant_model_voltage_control_mode::fixed_voltage) {
            plant_runner.get_parameters().voltage_control_mode.write(
                sfc_plant_model_datamodel::machine_voltage_control_mode::fixed_voltage);
        }
        else if (config.voltage_control_mode.value() == plant_model_voltage_control_mode::variable_speed) {
            plant_runner.get_parameters().voltage_control_mode.write(
                sfc_plant_model_datamodel::machine_voltage_control_mode::variable_speed);
        }
    }
    // else: preserve whatever voltage_control_mode was set by the builder in both places

    // Note: configure_speed_voltage_relationship removed - back-EMF calculated directly from E = Φ × ω
    // The voltage_slope and voltage_offset parameters are no longer used

    plant_runner.get_plant_model().set_initial_mechanical_speed_pu(config.initial_speed);
    plant_runner.get_plant_model().set_load_torque_pu(config.load_torque);

    plant_runner.get_parameters().operation_mode.write(
        sfc_plant_model_datamodel::machine_operation_mode::variable_speed);
    plant_runner.get_parameters().motor_damping_factor.write(config.damping_factor);
    plant_runner.get_parameters().motor_inertia_constant.write(config.inertia_constant);
    plant_runner.get_parameters().motor_linear_load_coefficient.write(config.linear_load_coefficient);
    plant_runner.get_parameters().motor_quadratic_load_coefficient.write(config.quadratic_load_coefficient);
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
    voltage_t machine_voltage_abc{2200.0f};

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
    plant_runner.get_plant_model().set_machine_parameters(config.machine_frequency, config.machine_voltage_abc);
    plant_runner.get_plant_model().set_load_torque_pu(config.load_torque);
}

}  // namespace yeet::application::sfc::integration::test
