#pragma once

#include "yeet/model_registry.hpp"
#include "yeet/parameter.hpp"
#include "yeet/signal.hpp"
#include "yeet/utils/parameter_accessors.hpp"
#include "yeet_control/angle_wrapped.hpp"
#include "yeet_control/reference_frame.hpp"
#include "yeet_control/six_pulse_generator.hpp"
#include "yeet_control/units.hpp"

/**
 * @file sfc_plant_model_datamodel.hpp
 * @brief Datamodel specification for SFC Plant Model component
 */

namespace yeet::application::sfc::integration {

class sfc_plant_model_datamodel {
public:
    /**
     * @brief Defines the operational mode for the machine side of the plant.
     */
    enum class machine_operation_mode {
        fixed_speed,    ///< Machine runs at a constant, predefined frequency.
        variable_speed  ///< Machine speed is determined by the mechanical load model.
    };

    /**
     * @brief Defines the voltage control mode for the machine side excitation.
     */
    enum class machine_voltage_control_mode {
        fixed_voltage,  ///< Machine voltage is fixed (no V/Hz control).
        variable_speed  ///< Machine voltage controlled by field current regulator (E = Φ × ω).
    };

    /**
     * @brief Defines the back-to-back topology implementation to use.
     */
    enum class topology_type {
        simulink,  ///< Use Simulink-generated model wrapper (default)
        internal,  ///< Use internal C++ thyristor model
        rectifier  ///< Use rectifier-only topology (machine-side short-circuited)
    };

    /**
     * @brief Input signals for the SFC plant model component
     */
    struct inputs {
        /// Grid-side firing pulses (6-pulse rectifier)
        signal_input<control::six_pulse_generator::pulses> grid_firing_pulses;

        /// Machine-side firing pulses (6-pulse inverter)
        signal_input<control::six_pulse_generator::pulses> machine_firing_pulses;

        /// Field current reference from external excitation controller [pu]
        signal_input<current_pu_t> field_current_reference;
    };

    /**
     * @brief Output signals from the SFC plant model component
     */
    struct outputs {
        /// Grid-side voltage measurements (line-to-neutral) [V]
        signal_output<abc<voltage_t>> grid_voltage_abc;

        /// Grid-side current measurements [A]
        signal_output<abc<current_t>> grid_current_abc;

        /// DC link voltage [V]
        signal_output<voltage_t> dc_link_voltage;

        /// DC link current [A]
        signal_output<current_t> dc_link_current;

        /// Machine-side voltage measurements (line-to-neutral) [V]
        signal_output<abc<voltage_t>> machine_voltage_abc;

        /// Machine-side current measurements [A]
        signal_output<abc<current_t>> machine_current;

        /// Grid voltage angular position [rad]
        signal_output<angle_wrapped> grid_voltage_source_angle;

        /// Machine voltage angular position [rad]
        signal_output<angle_wrapped> machine_voltage_source_angle;

        /// Machine frequency [Hz]
        signal_output<frequency_t> machine_frequency;

        /// Machine speed [pu]
        signal_output<speed_pu_t> machine_speed;

        /// Grid frequency (Hz)
        signal_output<frequency_t> grid_frequency;

        /// Machine flux [pu]
        signal_output<real_t> machine_flux_pu;

        /// Field current [pu]
        signal_output<current_pu_t> field_current_pu;
    };

    /**
     * @brief Configuration parameters for the SFC plant model
     *
     * These parameters reflect the configurable aspects of the sfc_plant_model_simulator.
     */
    struct parameters {
        /// Initial machine speed [pu]
        /// Default: 1.0 pu (synchronous speed)
        parameter<real_t> initial_speed = real_t(1.0);

        /// Defines if the machine runs at a fixed speed or if speed is determined by the load model.
        parameter<machine_operation_mode> operation_mode = machine_operation_mode::fixed_speed;

        /// Defines the voltage control mode for machine-side excitation
        parameter<machine_voltage_control_mode> voltage_control_mode = machine_voltage_control_mode::fixed_voltage;

        /// Motor inertia constant (H).
        /// Default: 2.5 s
        parameter<inertia_constant_t> motor_inertia_constant = inertia_constant_t(2.5);

        /// Motor damping factor (D) in per-unit.
        /// Default: 0.01 pu
        parameter<damping_coefficient_pu_t> motor_damping_factor = damping_coefficient_pu_t(0.01);

        /// Linear load coefficient k1 [pu/pu] for speed-dependent load torque
        /// T_load = T_external + k1*omega + k2*omega^2
        /// Default: 0.0 (no linear load)
        parameter<real_t> motor_linear_load_coefficient = real_t(0.0);

        /// Quadratic load coefficient k2 [pu/pu^2] for speed-dependent load torque
        /// Typical for gas turbine compressors: 0.2-0.4
        /// Default: 0.0 (no quadratic load)
        parameter<real_t> motor_quadratic_load_coefficient = real_t(0.0);

        /// Constant torque load [pu]
        /// This parameter sets a constant torque load on the machine shaft
        /// Positive values represent motor load, negative values represent generating load
        /// Default: 0.0 (no constant torque load)
        parameter<torque_pu_t> constant_torque = torque_pu_t(0.0);

        /// Grid voltage source frequency in per-unit [pu]
        /// This parameter sets the frequency of the grid voltage source relative to rated frequency
        /// Used to simulate grid frequency variations or off-nominal conditions
        /// Default: 1.0 pu (nominal frequency)
        parameter<frequency_pu_t> grid_frequency_pu = frequency_pu_t(1.0);

        /// Grid voltage source RMS voltage in per-unit [pu]
        /// This parameter sets the RMS line voltage of the grid source relative to rated voltage
        /// Used to simulate voltage sags, swells, or off-nominal conditions
        /// Default: 1.0 pu (nominal voltage)
        parameter<voltage_pu_t> grid_voltage_pu = voltage_pu_t(1.0);

        /// Topology implementation to use for the back-to-back converter model
        /// simulink: Use Simulink-generated model wrapper (default, validated against Matlab)
        /// internal: Use internal C++ thyristor model (faster, no external dependencies)
        /// Note: This parameter must be set before initialize() is called
        parameter<topology_type> topology = topology_type::simulink;
    };

private:
    inputs* _model_inputs;
    outputs* _model_outputs;
    parameters* _model_parameters;

public:
    sfc_plant_model_datamodel(inputs* model_inputs, outputs* model_outputs, parameters* model_parameters)
        : _model_inputs(model_inputs), _model_outputs(model_outputs), _model_parameters(model_parameters)
    {
        assert(_model_inputs != nullptr);
        assert(_model_outputs != nullptr);
        assert(_model_parameters != nullptr);
    }

    void register_data_model(model_registry& registry)
    {
        model_registry_entry entry{signal_registry{}, parameter_registry{}};

        entry.parameters.insert("initial_speed", yeet::utils::make_units_accessor(_model_parameters->initial_speed));

        // // Register operation_mode enum parameter
        // auto& operation_mode_param = _model_parameters->operation_mode;
        // entry.parameters.insert("operation_mode",
        //     low_level_access([&operation_mode_param]()
        //                          -> float { return static_cast<float>(static_cast<int>(operation_mode_param.read()));
        //                          },
        //         [&operation_mode_param](float value) {
        //             operation_mode_param.write(static_cast<machine_operation_mode>(static_cast<int>(value)));
        //         }));

        // // Register voltage_control_mode enum parameter
        // auto& voltage_control_mode_param = _model_parameters->voltage_control_mode;
        // entry.parameters.insert("voltage_control_mode",
        //     low_level_access(
        //         [&voltage_control_mode_param]() -> float {
        //             return static_cast<float>(static_cast<int>(voltage_control_mode_param.read()));
        //         },
        //         [&voltage_control_mode_param](float value) {
        //             voltage_control_mode_param.write(
        //                 static_cast<machine_voltage_control_mode>(static_cast<int>(value)));
        //         }));

        entry.parameters.insert(
            "motor_inertia_constant", yeet::utils::make_units_accessor(_model_parameters->motor_inertia_constant));
        entry.parameters.insert(
            "motor_damping_factor", yeet::utils::make_units_accessor(_model_parameters->motor_damping_factor));
        entry.parameters.insert("motor_linear_load_coefficient",
            yeet::utils::make_units_accessor(_model_parameters->motor_linear_load_coefficient));
        entry.parameters.insert("motor_quadratic_load_coefficient",
            yeet::utils::make_units_accessor(_model_parameters->motor_quadratic_load_coefficient));
        entry.parameters.insert(
            "constant_torque", yeet::utils::make_units_accessor(_model_parameters->constant_torque));
        entry.parameters.insert(
            "grid_frequency_pu", yeet::utils::make_units_accessor(_model_parameters->grid_frequency_pu));
        entry.parameters.insert(
            "grid_voltage_pu", yeet::utils::make_units_accessor(_model_parameters->grid_voltage_pu));
        entry.parameters.insert("plant_topology", yeet::utils::make_enum_accessor(_model_parameters->topology));

        registry.insert("plant_model", std::move(entry));
    }

    inputs* get_inputs() { return _model_inputs; }
    outputs* get_outputs() { return _model_outputs; }
    parameters* get_parameters() { return _model_parameters; }
};

}  // namespace yeet::application::sfc::integration
