#pragma once

#include "yeet/model_registry.hpp"
#include "yeet/parameter.hpp"
#include "yeet/signal.hpp"
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
     * @brief Input signals for the SFC plant model component
     */
    struct inputs {
        /// Grid-side firing pulses (6-pulse rectifier)
        signal_input<control::six_pulse_generator::pulses> grid_side_firing_pulses;

        /// Machine-side firing pulses (6-pulse inverter)
        signal_input<control::six_pulse_generator::pulses> machine_side_firing_pulses;

        /// Mechanical load torque on motor shaft
        /// Range: [-2.0, 2.0] pu
        signal_input<torque_pu_t> load_torque_pu;

        /// Grid voltage (line-to-line RMS)
        /// Range: [0.5, 1.2] pu
        signal_input<abc<voltage_pu_t>> grid_voltage_ll_rms_pu;

        /// Grid impedance (for dynamic studies)
        signal_input<abc<impedance_t>> grid_impedance;
    };

    /**
     * @brief Output signals from the SFC plant model component
     */
    struct outputs {
        /// Grid-side voltage measurements (line-to-neutral)
        signal_output<abc<voltage_t>> grid_voltage_ln;

        /// Grid-side current measurements
        signal_output<abc<current_t>> grid_current;

        /// DC link voltage
        signal_output<voltage_t> dc_link_voltage;

        /// DC link current
        signal_output<current_t> dc_link_current;

        /// Machine-side voltage measurements (line-to-neutral)
        signal_output<abc<voltage_t>> machine_voltage_ln;

        /// Machine-side current measurements
        signal_output<abc<current_t>> machine_current;

        /// Motor rotor position (mechanical)
        /// Range: [0.0, 2π] radians
        signal_output<angle_t> rotor_position_mechanical;

        /// Motor rotor speed (mechanical)
        /// Range: [-3.0, 3.0] pu
        signal_output<speed_pu_t> rotor_speed_mechanical;

        /// Motor electromagnetic torque
        /// Range: [-2.0, 2.0] pu
        signal_output<torque_pu_t> electromagnetic_torque;

        /// Total harmonic distortion - grid current
        /// Range: [0.0, 1.0] (fraction)
        signal_output<real_t> grid_current_thd;

        /// Power factor at grid connection
        /// Range: [-1.0, 1.0]
        signal_output<real_t> power_factor;

        /// Total active power (grid side)
        /// Range: [-2.0, 2.0] pu
        signal_output<power_pu_t> active_power_pu;
    };

    /**
     * @brief Configuration parameters for the SFC plant model
     */
    struct parameters {
        /// Motor rated power
        /// Range: [0.1, 100.0] MW
        parameter<power_t> motor_rated_power = power_t(1.0e6);  // 1 MW

        /// Motor rated voltage (line-to-line RMS)
        /// Range: [100.0, 10000.0] V
        parameter<voltage_t> motor_rated_voltage = voltage_t(690.0);  // 690V

        /// Motor rated frequency
        /// Range: [10.0, 100.0] Hz
        parameter<frequency_t> motor_rated_frequency = frequency_t(50.0);

        /// Motor pole pairs
        /// Range: [1, 12]
        parameter<int> motor_pole_pairs = 2;

        /// Motor stator resistance
        /// Range: [0.001, 0.1] pu
        parameter<resistance_pu_t> motor_stator_resistance = resistance_pu_t(0.02);

        /// Motor rotor resistance (referred to stator)
        /// Range: [0.001, 0.1] pu
        parameter<resistance_pu_t> motor_rotor_resistance = resistance_pu_t(0.015);

        /// Motor stator leakage inductance
        /// Range: [0.05, 0.2] pu
        parameter<inductance_pu_t> motor_stator_leakage_inductance = inductance_pu_t(0.08);

        /// Motor rotor leakage inductance (referred to stator)
        /// Range: [0.05, 0.2] pu
        parameter<inductance_pu_t> motor_rotor_leakage_inductance = inductance_pu_t(0.08);

        /// Motor magnetizing inductance
        /// Range: [3.0, 8.0] pu
        parameter<inductance_pu_t> motor_magnetizing_inductance = inductance_pu_t(4.5);

        /// Motor inertia constant
        /// Range: [0.5, 10.0] s
        parameter<duration_t> motor_inertia_constant = duration_t(2.0);

        /// DC link inductor
        /// Range: [0.01, 1.0] H
        parameter<inductance_t> dc_link_inductance = inductance_t(0.1);

        /// DC link resistance
        /// Range: [0.001, 0.1] Ω
        parameter<resistance_t> dc_link_resistance = resistance_t(0.01);

        /// Grid side transformer turns ratio
        /// Range: [0.1, 10.0]
        parameter<real_t> grid_transformer_ratio = real_t(1.0);

        /// Machine side transformer turns ratio
        /// Range: [0.1, 10.0]
        parameter<real_t> machine_transformer_ratio = real_t(1.0);

        /// Converter commutation inductance (per phase)
        /// Range: [0.001, 0.1] H
        parameter<inductance_t> commutation_inductance = inductance_t(0.005);

        /// Thyristor forward voltage drop
        /// Range: [0.5, 3.0] V
        parameter<voltage_t> thyristor_forward_drop = voltage_t(1.2);

        /// Grid short circuit power (for impedance calculation)
        /// Range: [10.0, 10000.0] MVA
        parameter<power_t> grid_short_circuit_power = power_t(100.0e6);  // 100 MVA
    };

private:
    inputs* _model_inputs;
    outputs* _model_outputs;
    parameters* _model_parameters;

public:
    sfc_plant_model_datamodel(inputs* model_inputs, outputs* model_outputs, parameters* model_parameters)
        : _model_inputs(model_inputs), _model_outputs(model_outputs), _model_parameters(model_parameters) {
        assert(_model_inputs != nullptr);
        assert(_model_outputs != nullptr);
        assert(_model_parameters != nullptr);
    }

    void register_data_model(model_registry& registry) {
        model_registry_entry entry{signal_registry{}, parameter_registry{}};

        // Register outputs in signal registry
        entry.signal_registry.insert("grid_voltage_ln", _model_outputs->grid_voltage_ln);
        entry.signal_registry.insert("grid_current", _model_outputs->grid_current);
        entry.signal_registry.insert("dc_link_voltage", _model_outputs->dc_link_voltage);
        entry.signal_registry.insert("dc_link_current", _model_outputs->dc_link_current);
        entry.signal_registry.insert("machine_voltage_ln", _model_outputs->machine_voltage_ln);
        entry.signal_registry.insert("machine_current", _model_outputs->machine_current);
        entry.signal_registry.insert("rotor_position_mechanical", _model_outputs->rotor_position_mechanical);
        entry.signal_registry.insert("rotor_speed_mechanical", _model_outputs->rotor_speed_mechanical);
        entry.signal_registry.insert("electromagnetic_torque", _model_outputs->electromagnetic_torque);
        entry.signal_registry.insert("grid_current_thd", _model_outputs->grid_current_thd);
        entry.signal_registry.insert("power_factor", _model_outputs->power_factor);
        entry.signal_registry.insert("active_power_pu", _model_outputs->active_power_pu);

        // Register parameters in parameter registry
        entry.parameter_registry.insert("motor_rated_power", _model_parameters->motor_rated_power);
        entry.parameter_registry.insert("motor_rated_voltage", _model_parameters->motor_rated_voltage);
        entry.parameter_registry.insert("motor_rated_frequency", _model_parameters->motor_rated_frequency);
        entry.parameter_registry.insert("motor_pole_pairs", _model_parameters->motor_pole_pairs);
        entry.parameter_registry.insert("motor_stator_resistance", _model_parameters->motor_stator_resistance);
        entry.parameter_registry.insert("motor_rotor_resistance", _model_parameters->motor_rotor_resistance);
        entry.parameter_registry.insert("motor_stator_leakage_inductance",
                                        _model_parameters->motor_stator_leakage_inductance);
        entry.parameter_registry.insert("motor_rotor_leakage_inductance",
                                        _model_parameters->motor_rotor_leakage_inductance);
        entry.parameter_registry.insert("motor_magnetizing_inductance",
                                        _model_parameters->motor_magnetizing_inductance);
        entry.parameter_registry.insert("motor_inertia_constant", _model_parameters->motor_inertia_constant);
        entry.parameter_registry.insert("dc_link_inductance", _model_parameters->dc_link_inductance);
        entry.parameter_registry.insert("dc_link_resistance", _model_parameters->dc_link_resistance);
        entry.parameter_registry.insert("grid_transformer_ratio", _model_parameters->grid_transformer_ratio);
        entry.parameter_registry.insert("machine_transformer_ratio", _model_parameters->machine_transformer_ratio);
        entry.parameter_registry.insert("commutation_inductance", _model_parameters->commutation_inductance);
        entry.parameter_registry.insert("thyristor_forward_drop", _model_parameters->thyristor_forward_drop);
        entry.parameter_registry.insert("grid_short_circuit_power", _model_parameters->grid_short_circuit_power);

        registry.insert("sfc_plant_model", std::move(entry));
    }

    inputs* get_inputs() { return _model_inputs; }
    outputs* get_outputs() { return _model_outputs; }
    parameters* get_parameters() { return _model_parameters; }
};

}  // namespace yeet::application::sfc::integration
