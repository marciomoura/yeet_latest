#pragma once

#include "sfc_6p_6p_plant_model.hpp"
#include "sfc_per_unit_system.hpp"
#include "yeet_control/mechanical_load.hpp"
#include "yeet_control/reference_frame.hpp"
#include "yeet_control/speed_voltage_relationship.hpp"
#include "yeet_control/units.hpp"
#include "yeet_testing/simulator.hpp"

using namespace yeet;

namespace yeet::application::sfc {

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

    sfc_plant_model_simulator()
        : _mechanical_load(sfc_6p_6p_plant_model::kSamplingTime),
          _speed_voltage_relationship(sfc_6p_6p_plant_model::kSamplingTime),
          _pu_system(kNominalGridVoltageRMS, kNominalGridFrequency, kBaseDCLinkCurrent, kMachineRatedPowerVA,
                     kNominalMachineVoltageRMS, kNominalMachineFrequency, kMachineNumPoles) {
        // Configure with default mechanical parameters
        _plant.initialize();
        _mechanical_load.configure(kDefaultInertiaConstantH, kDefaultDampingFactorD);

        // Configure with default speed-voltage parameters
        _speed_voltage_relationship.configure_voltage_speed_coefficient(kDefaultVoltageSpeedCoefficient);
        _speed_voltage_relationship.configure_voltage_offset_pu(kDefaultVoltageOffset);
        _speed_voltage_relationship.configure_voltage_limits_pu(0.1f, 1.5f);  // Reasonable voltage limits
    }

    /**
     * @brief Sets the machine's operational mode (fixed or variable speed).
     * @param mode The desired operation mode.
     */
    void set_operation_mode(machine_operation_mode mode) { _mode = mode; }

    /**
     * @brief Sets the voltage control mode for the machine side.
     * @param mode The desired voltage control mode.
     */
    void set_voltage_control_mode(voltage_control_mode mode) { _voltage_mode = mode; }

    /**
     * @brief Configures the parameters of the mechanical load model.
     * @param inertia_constant_seconds The inertia constant in seconds.
     * @param damping_coefficient The damping factor in per-unit.
     */
    void configure_mechanical_load(inertia_constant_t inertia_constant_seconds,
                                   damping_coefficient_pu_t damping_coefficient) {
        _mechanical_load.configure(inertia_constant_seconds, damping_coefficient);
    }

    /**
     * @brief Configures the speed-voltage relationship parameters.
     * @param coefficient The voltage-speed coefficient [pu/pu].
     * @param offset_pu The voltage offset in per-unit.
     * @param time_constant_seconds The time constant for dynamics [s]. Use 0.0 for algebraic response.
     */
    void configure_speed_voltage_relationship(real_t coefficient, real_t offset_pu,
                                              real_t time_constant_seconds = 0.0f) {
        _speed_voltage_relationship.configure_voltage_speed_coefficient(coefficient);
        _speed_voltage_relationship.configure_voltage_offset_pu(offset_pu);
        _speed_voltage_relationship.configure_time_constant(time_constant_seconds);
    }

    /**
     * @brief Configures the voltage limits for the speed-voltage relationship.
     * @param min_voltage_pu Minimum voltage magnitude [pu].
     * @param max_voltage_pu Maximum voltage magnitude [pu].
     */
    void configure_voltage_limits_pu(real_t min_voltage_pu, real_t max_voltage_pu) {
        _speed_voltage_relationship.configure_voltage_limits_pu(min_voltage_pu, max_voltage_pu);
    }

    /**
     * @brief Sets the initial mechanical speed for the load.
     * @param initial_speed_pu The initial mechanical speed in per-unit.
     *
     * This method should be called before starting the simulation to establish
     * the initial conditions for the mechanical system. It is particularly useful
     * for soft-starter applications where the machine may already be rotating.
     */
    void set_initial_mechanical_speed_pu(speed_pu_t initial_speed_pu) {
        _mechanical_load.reset(initial_speed_pu);

        _speed_voltage_relationship.reset(1.0, 1.0);  // initial_speed_pu.value());
        // Initialize speed-voltage relationship with corresponding voltage
        // if (_voltage_mode == voltage_control_mode::speed_dependent) {
        // }

        // If in variable speed mode, also update the machine frequency to match
        if (_mode == machine_operation_mode::variable_speed) {
            frequency_pu_t initial_frequency_pu = per_unit_cast<frequency_pu_t>(initial_speed_pu);
            frequency_t machine_frequency_hz = yeet::to_si(initial_frequency_pu, kNominalMachineFrequency);

            voltage_t machine_voltage = kNominalMachineVoltageRMS;
            if (_voltage_mode == voltage_control_mode::speed_dependent) {
                real_t voltage_magnitude_pu = _speed_voltage_relationship.get_voltage_magnitude_pu();
                machine_voltage = voltage_t(voltage_magnitude_pu * kNominalMachineVoltageRMS.value());
            }

            _plant.set_machine_parameters(machine_frequency_hz, machine_voltage);
        }
    }

    /**
     * @brief Sets the load torque in per-unit for the mechanical load model.
     * @param load_torque_pu The load torque in per-unit.
     *
     * This method allows setting the load torque that the mechanical load model will apply
     * during the simulation. It is useful for simulating different loading conditions.
     */
    void set_load_torque_pu(torque_pu_t load_torque_pu) {
        // Set the load torque in the mechanical load model
        _load_torque_pu = load_torque_pu;
    }

    /**
     * @brief Gets a reference to the mechanical load model.
     * @return A reference to the mechanical_load.
     */
    control::mechanical_load& mechanical_load() { return _mechanical_load; }

    /**
     * @brief Gets a reference to the speed-voltage relationship model.
     * @return A reference to the speed_voltage_relationship.
     */
    control::speed_voltage_relationship& speed_voltage_relationship() { return _speed_voltage_relationship; }

    /**
     * @brief Gets a reference to the per-unit system.
     * @return A reference to the sfc_per_unit_system.
     */
    const sfc_per_unit_system& per_unit_system() const { return _pu_system; }

    /**
     * @brief Gets a reference to the underlying plant model.
     * @return A reference to the sfc_6p_6p_plant_model.
     */
    sfc_6p_6p_plant_model& plant() { return _plant; }

    /**
     * @brief Registers the simulator's step function with the simulator framework.
     * @param simulator The testing simulator instance.
     */
    void setup_simulator(yeet::testing::simulator& simulator) {
        simulator.register_lambda_runner(sfc_6p_6p_plant_model::kSamplingTime, [&]() { this->step(); });
    }

    void setup_all_logging(yeet::testing::simulator& simulator) {
        setup_grid_side_logging(simulator);
        setup_machine_side_logging(simulator);
        setup_dc_link_logging(simulator);
        setup_mechanical_load_logging(simulator);
    }

    void setup_grid_side_logging(yeet::testing::simulator& simulator) {
        // Log grid-side voltages (abc)
        simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "grid_voltage_a", [&]() -> double { return _plant.get_grid_side_voltages_abc().a(); }));
        simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "grid_voltage_b", [&]() -> double { return _plant.get_grid_side_voltages_abc().b(); }));
        simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "grid_voltage_c", [&]() -> double { return _plant.get_grid_side_voltages_abc().c(); }));

        // Log grid-side currents (abc)
        simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "grid_current_a", [&]() -> double { return _plant.get_grid_side_currents_abc().a(); }));
        simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "grid_current_b", [&]() -> double { return _plant.get_grid_side_currents_abc().b(); }));
        simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "grid_current_c", [&]() -> double { return _plant.get_grid_side_currents_abc().c(); }));

        // Log grid-side angle
        simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "grid_angle_rad", [&]() -> double { return _plant.get_grid_voltage_source_angle().get_radians(); }));
    }

    void setup_machine_side_logging(yeet::testing::simulator& simulator) {
        // Log machine-side voltages (abc)
        simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "machine_voltage_a", [&]() -> double { return _plant.get_machine_side_voltages_abc().a(); }));
        simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "machine_voltage_b", [&]() -> double { return _plant.get_machine_side_voltages_abc().b(); }));
        simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "machine_voltage_c", [&]() -> double { return _plant.get_machine_side_voltages_abc().c(); }));

        // Log machine-side currents (abc)
        simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "machine_current_a", [&]() -> double { return _plant.get_machine_side_currents_abc().a(); }));
        simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "machine_current_b", [&]() -> double { return _plant.get_machine_side_currents_abc().b(); }));
        simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "machine_current_c", [&]() -> double { return _plant.get_machine_side_currents_abc().c(); }));

        // Log machine-side angle
        simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "machine_angle_rad", [&]() -> double { return _plant.get_machine_voltage_source_angle().get_radians(); }));
    }

    void setup_dc_link_logging(yeet::testing::simulator& simulator) {
        // Log DC link current
        simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "dc_link_current", [&]() -> double { return _plant.get_dc_link_current(); }));
    }

    void setup_mechanical_load_logging(yeet::testing::simulator& simulator) {
        // Log mechanical speed
        simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "mechanical_speed_pu", [&]() -> double { return _mechanical_load.get_speed_pu(); }));

        // Log machine frequency
        simulator.register_sampler_entry(
            yeet::testing::make_lambda_sampler<double>("machine_frequency_hz", [&]() -> double {
                if (_mode == machine_operation_mode::variable_speed) {
                    return _mechanical_load.get_speed_pu() * kNominalMachineFrequency.value();
                } else {
                    return kNominalMachineFrequency.value();
                }
            }));

        // Log electromagnetic torque
        simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "electromagnetic_torque_pu",
            [&]() -> double { return _plant.get_dc_link_current() / kBaseDCLinkCurrent.value(); }));
    }

    /**
     * @brief Executes one simulation step, handling both fixed and variable speed modes.
     */
    void step() {
        // Always step the plant model first
        _plant.step();

        // Step the mechanical dynamics (if enabled)
        step_mechanical_dynamics();

        // Step the voltage control (if enabled)
        step_voltage_control();

        // Update plant parameters with the computed values
        update_plant_parameters();
    }

private:
    /**
     * @brief Steps the mechanical load dynamics if in variable speed mode.
     */
    void step_mechanical_dynamics() {
        if (_mode != machine_operation_mode::variable_speed) {
            return;  // No mechanical dynamics in fixed speed mode
        }

        // Calculate electromagnetic torque from DC link current
        // Convert DC current to per-unit using the base DC current
        const current_pu_t dc_link_current = to_pu(_plant.get_dc_link_current(), kBaseDCLinkCurrent);

        // Calculate electromagnetic torque in per-unit
        // For LCI drives: T_e = k * I_dc where k is proportional to machine flux
        // In per-unit: T_e_pu = I_dc_pu (assuming unit flux)
        const torque_pu_t electromagnetic_torque_pu = torque_pu_t{dc_link_current};

        // Step the mechanical load with electromagnetic torque
        _mechanical_load.update(electromagnetic_torque_pu, _load_torque_pu);
    }

    /**
     * @brief Steps the speed-voltage relationship if voltage control is enabled.
     */
    void step_voltage_control() {
        if (_voltage_mode != voltage_control_mode::speed_dependent) {
            return;  // No voltage control in fixed voltage mode
        }

        // Determine the speed input for voltage calculation
        const speed_pu_t speed_input_pu = _mechanical_load.get_speed_pu();

        // Update the speed-voltage relationship
        _speed_voltage_relationship.update(speed_input_pu);
    }

    /**
     * @brief Updates the plant model parameters with computed frequency and voltage.
     */
    void update_plant_parameters() {
        const frequency_t machine_frequency_hz =
            (_mode == machine_operation_mode::variable_speed)
                ? to_si(frequency_pu_t{_mechanical_load.get_speed_pu()}, kNominalMachineFrequency)
                : _plant.get_machine_frequency();

        const voltage_t machine_voltage =
            (_voltage_mode == voltage_control_mode::speed_dependent)
                ? voltage_t(_speed_voltage_relationship.get_voltage_magnitude_pu() * kNominalMachineVoltageRMS.value())
                : kNominalMachineVoltageRMS;

        _plant.set_machine_parameters(machine_frequency_hz, machine_voltage);
    }

    /**
     * @brief Gets the current machine voltage based on voltage control mode.
     * @return The machine voltage in RMS volts.
     */
    voltage_t get_machine_voltage() const {
        if (_voltage_mode == voltage_control_mode::speed_dependent) {
            // Use voltage from speed-voltage relationship
            const real_t voltage_magnitude_pu = _speed_voltage_relationship.get_voltage_magnitude_pu();
            return voltage_t(voltage_magnitude_pu * kNominalMachineVoltageRMS.value());
        } else {
            // Use nominal voltage for fixed voltage mode
            return kNominalMachineVoltageRMS;
        }
    }

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
