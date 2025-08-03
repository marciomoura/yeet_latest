#pragma once

#include "sfc_6p_6p_plant_model.hpp"
#include "sfc_per_unit_system.hpp"
#include "yeet_control/mechanical_load.hpp"
#include "yeet_control/speed_voltage_relationship.hpp"
#include "yeet_control/units.hpp"

// to be removed from yeet_application
#include "yeet_testing/simulator.hpp"

using namespace yeet;

namespace yeet::application::sfc {

/**
 * @brief A comprehensive plant model simulator for SFC (Static Frequency Converter) applications
 *
 * This class provides a complete simulation environment for SFC systems, including:
 * - 6-pulse to 6-pulse LCI plant model
 * - Mechanical load dynamics with variable inertia and damping
 * - Speed-dependent voltage control for V/f operation
 * - Comprehensive logging capabilities for all electrical and mechanical quantities
 * - Support for both fixed and variable speed operation modes
 */
class sfc_plant_model_simulator {
public:
    // Frequency and electrical parameters with proper units
    static constexpr frequency_t kNominalGridFrequency = frequency_t(50.0);     // 50 Hz grid frequency
    static constexpr frequency_t kNominalMachineFrequency = frequency_t(50.0);  // 50 Hz machine frequency
    static constexpr voltage_t kNominalGridVoltageRMS = voltage_t(2090.0);      // 2090V RMS line voltage
    static constexpr voltage_t kNominalMachineVoltageRMS = voltage_t(1980.0);   // 1980V RMS line voltage

    // Firing angles (dimensionless angles in degrees)
    static constexpr angle_t kNominalGridFiringAngleDeg = angle_t(13.0);  // Nominal firing angle for grid side
    static constexpr angle_t kNominalMachineFiringAngleDeg =
        angle_t(180.0 - 26.84);  // Nominal firing angle for motor side

    // Circuit parameters with proper units
    static constexpr resistance_t kGridSideResistance = resistance_t(0.1552);     // 0.1552 Ohm grid side resistance
    static constexpr inductance_t kGridSideInductance = inductance_t(0.4941e-3);  // 0.4941 mH grid side inductance
    static constexpr resistance_t kMachineSideResistance = resistance_t(0.0788);  // 0.0788 Ohm machine side resistance
    static constexpr inductance_t kMachineSideInductance =
        inductance_t(0.2508e-3);  // 0.2508 mH machine side inductance

    // Power and current ratings with proper units
    static constexpr current_t kBaseDCLinkCurrent = current_t(1044.0);  // 1044 A base DC link current
    static constexpr power_t kMachineRatedPowerVA = power_t(2950e3);    // 2.95 MVA machine rated power
    static constexpr int kMachineNumPoles = 2;                          // Number of poles (dimensionless)

    // Default Mechanical Parameters with proper units
    static constexpr inertia_constant_t kDefaultInertiaConstantH = inertia_constant_t(2.5);  // 2.5 s inertia constant
    static constexpr damping_coefficient_pu_t kDefaultDampingFactorD =
        damping_coefficient_pu_t(0.01);  // 0.01 pu damping factor

    // Default Speed-Voltage Parameters
    static constexpr real_t kDefaultVoltageSpeedCoefficient = 0.9f;  // 0.9 pu/pu voltage-speed coefficient
    static constexpr real_t kDefaultVoltageOffset = 0.1f;            // 0.1 pu voltage offset

    /**
     * @brief Defines the operational mode for the machine side of the plant.
     */
    enum class machine_operation_mode {
        fixed_speed,    // Machine runs at a constant, predefined frequency.
        variable_speed  // Machine speed is determined by the mechanical load model.
    };

    /**
     * @brief Defines the voltage control mode for the machine side.
     */
    enum class voltage_control_mode {
        fixed_voltage,   // Machine voltage is fixed at nominal value.
        speed_dependent  // Machine voltage varies with speed using speed-voltage relationship.
    };

    /**
     * @brief Constructor initializes the plant model simulator with default parameters.
     *
     * Sets up the mechanical load, speed-voltage relationship, and per-unit system
     * with reasonable default values for typical SFC applications.
     */
    sfc_plant_model_simulator();

    /**
     * @brief Sets the machine's operational mode (fixed or variable speed).
     * @param mode The desired operation mode.
     */
    void set_operation_mode(machine_operation_mode mode);

    /**
     * @brief Sets the voltage control mode for the machine side.
     * @param mode The desired voltage control mode.
     */
    void set_voltage_control_mode(voltage_control_mode mode);

    /**
     * @brief Configures the parameters of the mechanical load model.
     * @param inertia_constant_seconds The inertia constant in seconds.
     * @param damping_coefficient The damping factor in per-unit.
     */
    void configure_mechanical_load(
        inertia_constant_t inertia_constant_seconds, damping_coefficient_pu_t damping_coefficient);

    /**
     * @brief Configures the speed-voltage relationship parameters.
     * @param coefficient The voltage-speed coefficient [pu/pu].
     * @param offset_pu The voltage offset in per-unit.
     * @param time_constant_seconds The time constant for dynamics [s]. Use 0.0 for algebraic response.
     */
    void configure_speed_voltage_relationship(
        real_t coefficient, real_t offset_pu, real_t time_constant_seconds = 0.0f);

    /**
     * @brief Configures the voltage limits for the speed-voltage relationship.
     * @param min_voltage_pu Minimum voltage magnitude [pu].
     * @param max_voltage_pu Maximum voltage magnitude [pu].
     */
    void configure_voltage_limits_pu(real_t min_voltage_pu, real_t max_voltage_pu);

    /**
     * @brief Sets the initial mechanical speed for the load.
     * @param initial_speed_pu The initial mechanical speed in per-unit.
     *
     * This method should be called before starting the simulation to establish
     * the initial conditions for the mechanical system. It is particularly useful
     * for soft-starter applications where the machine may already be rotating.
     */
    void set_initial_mechanical_speed_pu(speed_pu_t initial_speed_pu);

    /**
     * @brief Sets the load torque in per-unit for the mechanical load model.
     * @param load_torque_pu The load torque in per-unit.
     *
     * This method allows setting the load torque that the mechanical load model will apply
     * during the simulation. It is useful for simulating different loading conditions.
     */
    void set_load_torque_pu(torque_pu_t load_torque_pu);

    /**
     * @brief Gets a reference to the mechanical load model.
     * @return A reference to the mechanical_load.
     */
    control::mechanical_load& mechanical_load();

    /**
     * @brief Gets a reference to the speed-voltage relationship model.
     * @return A reference to the speed_voltage_relationship.
     */
    control::speed_voltage_relationship& speed_voltage_relationship();

    /**
     * @brief Gets a reference to the per-unit system.
     * @return A reference to the sfc_per_unit_system.
     */
    const sfc_per_unit_system& per_unit_system() const;

    /**
     * @brief Gets a reference to the underlying plant model.
     * @return A reference to the sfc_6p_6p_plant_model.
     */
    sfc_6p_6p_plant_model& plant();

    /**
     * @brief Registers the simulator's step function with the simulator framework.
     * @param simulator The testing simulator instance.
     */
    void setup_simulator(yeet::simulator& simulator);

    /**
     * @brief Sets up comprehensive logging for all plant model quantities.
     * @param simulator The testing simulator instance.
     */
    void setup_all_logging(yeet::simulator& simulator);

    /**
     * @brief Sets up logging for grid-side electrical quantities.
     * @param simulator The testing simulator instance.
     */
    void setup_grid_side_logging(yeet::simulator& simulator);

    /**
     * @brief Sets up logging for machine-side electrical quantities.
     * @param simulator The testing simulator instance.
     */
    void setup_machine_side_logging(yeet::simulator& simulator);

    /**
     * @brief Sets up logging for DC link quantities.
     * @param simulator The testing simulator instance.
     */
    void setup_dc_link_logging(yeet::simulator& simulator);

    /**
     * @brief Sets up logging for mechanical load quantities.
     * @param simulator The testing simulator instance.
     */
    void setup_mechanical_load_logging(yeet::simulator& simulator);

    /**
     * @brief Executes one simulation step, handling both fixed and variable speed modes.
     *
     * This method coordinates the execution of:
     * - Plant model electrical dynamics
     * - Mechanical load dynamics (if variable speed mode)
     * - Speed-voltage relationship (if speed-dependent voltage mode)
     * - Plant parameter updates
     */
    void step();

private:
    /**
     * @brief Steps the mechanical load dynamics if in variable speed mode.
     */
    void step_mechanical_dynamics();

    /**
     * @brief Steps the speed-voltage relationship if voltage control is enabled.
     */
    void step_voltage_control();

    /**
     * @brief Updates the plant model parameters with computed frequency and voltage.
     */
    void update_plant_parameters();

    /**
     * @brief Gets the current machine voltage based on voltage control mode.
     * @return The machine voltage in RMS volts.
     */
    voltage_t get_machine_voltage() const;

private:
    torque_pu_t _load_torque_pu = torque_pu_t{0.0};  // Default load torque in per-unit

    sfc_6p_6p_plant_model _plant;
    control::mechanical_load _mechanical_load;
    control::speed_voltage_relationship _speed_voltage_relationship;
    sfc_per_unit_system _pu_system;
    machine_operation_mode _mode = machine_operation_mode::fixed_speed;
    voltage_control_mode _voltage_mode = voltage_control_mode::fixed_voltage;
};

}  // namespace yeet::application::sfc
