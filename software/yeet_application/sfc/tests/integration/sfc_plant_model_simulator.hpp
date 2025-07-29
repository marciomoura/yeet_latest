#pragma once

#include "sfc/sfc_per_unit_system.hpp"
#include "sfc_6p_6p_plant_model.hpp"
#include "yeet_control/mechanical_load.hpp"
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
    static constexpr damping_coefficient_t kDefaultDampingFactorD =
        damping_coefficient_t(0.01);  // 0.01 pu damping factor

    /**
     * @brief Defines the operational mode for the machine side of the plant.
     */
    enum class machine_operation_mode {
        fixed_speed,    // Machine runs at a constant, predefined frequency.
        variable_speed  // Machine speed is determined by the mechanical load model.
    };

    sfc_plant_model_simulator()
        : _mechanical_load(sfc_6p_6p_plant_model::kSamplingTime),
          _pu_system(kNominalGridVoltageRMS, kNominalGridFrequency, kBaseDCLinkCurrent, kMachineRatedPowerVA,
                     kNominalMachineVoltageRMS, kNominalMachineFrequency, kMachineNumPoles) {
        // Configure with default mechanical parameters
        _plant.initialize();
        _mechanical_load.configure(kDefaultInertiaConstantH.value(), kDefaultDampingFactorD.value());
    }

    /**
     * @brief Sets the machine's operational mode (fixed or variable speed).
     * @param mode The desired operation mode.
     */
    void set_operation_mode(machine_operation_mode mode) { _mode = mode; }

    /**
     * @brief Configures the parameters of the mechanical load model.
     * @param inertia_H The inertia constant in seconds.
     * @param damping_D The damping factor in per-unit.
     */
    void configure_mechanical_load(double inertia_H, double damping_D) {
        _mechanical_load.configure(inertia_H, damping_D);
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

        // If in variable speed mode, also update the machine frequency to match
        if (_mode == machine_operation_mode::variable_speed) {
            frequency_pu_t initial_frequency_pu = per_unit_cast<frequency_pu_t>(initial_speed_pu);
            frequency_t machine_frequency_hz = yeet::to_si(initial_frequency_pu, kNominalMachineFrequency);
            _plant.set_machine_parameters(machine_frequency_hz, kNominalMachineVoltageRMS);
        }
    }

    /**
     * @brief Gets a reference to the mechanical load model.
     * @return A reference to the mechanical_load.
     */
    control::mechanical_load& mechanical_load() { return _mechanical_load; }

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
    void step(double load_torque_pu = 0.75) {
        // Always step the plant model first
        _plant.step();

        if (_mode == machine_operation_mode::variable_speed) {
            // Calculate electromagnetic torque from DC link current
            // Convert DC current to per-unit using the base DC current
            double i_dc_pu = _plant.get_dc_link_current() / kBaseDCLinkCurrent.value();

            // Calculate electromagnetic torque in per-unit
            // For LCI drives: T_e = k * I_dc where k is proportional to machine flux
            // In per-unit: T_e_pu = I_dc_pu (assuming unit flux)
            double electromagnetic_torque_pu = i_dc_pu;

            // Step the mechanical load with electromagnetic torque
            double omega_pu = _mechanical_load.update(electromagnetic_torque_pu, load_torque_pu);

            // Convert mechanical speed back to machine frequency
            frequency_t machine_frequency_hz = to_si(frequency_pu_t(omega_pu), kNominalMachineFrequency);

            // Update the plant model's machine side frequency
            _plant.set_machine_parameters(machine_frequency_hz, kNominalMachineVoltageRMS);
        }
    }

private:
    sfc_6p_6p_plant_model _plant;
    control::mechanical_load _mechanical_load;
    sfc_per_unit_system _pu_system;
    machine_operation_mode _mode = machine_operation_mode::fixed_speed;
};

}  // namespace yeet::application::sfc
