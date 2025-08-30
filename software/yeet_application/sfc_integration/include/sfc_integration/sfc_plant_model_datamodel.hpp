#pragma once

#include "yeet/model_registry.hpp"
#include "yeet/parameter.hpp"
#include "yeet/signal.hpp"
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
    };

    /**
     * @brief Output signals from the SFC plant model component
     */
    struct outputs {
        /// Grid-side voltage measurements (line-to-neutral)
        signal_output<abc<voltage_t>> grid_voltage;

        /// Grid-side current measurements
        signal_output<abc<current_t>> grid_current;

        /// DC link voltage
        signal_output<voltage_t> dc_link_voltage;

        /// DC link current
        signal_output<current_t> dc_link_current;

        /// Machine-side voltage measurements (line-to-neutral)
        signal_output<abc<voltage_t>> machine_voltage;

        /// Machine-side current measurements
        signal_output<abc<current_t>> machine_current;

        /// Grid voltage angular position
        /// Range: [0.0, 2π] radians
        signal_output<angle_wrapped> grid_voltage_source_angle;

        /// Grid voltage angular position
        /// Range: [0.0, 2π] radians
        signal_output<angle_wrapped> machine_voltage_source_angle;

        /// Machine frequency (Hz)
        signal_output<frequency_t> machine_frequency;

        /// Machine speed pu
        signal_output<speed_pu_t> machine_speed;

        /// Grid frequency (Hz)
        signal_output<frequency_t> grid_frequency;
    };

    /**
     * @brief Configuration parameters for the SFC plant model
     *
     * These parameters reflect the configurable aspects of the sfc_plant_model_simulator.
     */
    struct parameters {
        parameter<real_t> initial_speed = real_t(1.0);

        /// Defines if the machine runs at a fixed speed or if speed is determined by the load model.
        parameter<machine_operation_mode> operation_mode = machine_operation_mode::fixed_speed;

        /// Motor inertia constant (H).
        /// Default: 2.5 s
        parameter<inertia_constant_t> motor_inertia_constant = inertia_constant_t(2.5);

        /// Motor damping factor (D) in per-unit.
        /// Default: 0.01 pu
        parameter<damping_coefficient_pu_t> motor_damping_factor = damping_coefficient_pu_t(0.01);
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
        // model_registry_entry entry{signal_registry{}, parameter_registry{}};

        // // Register outputs in signal registry
        // entry.signal_registry.insert("grid_voltage_ln", _model_outputs->grid_voltage_ln);
        // entry.signal_registry.insert("grid_current", _model_outputs->grid_current);
        // entry.signal_registry.insert("dc_link_voltage", _model_outputs->dc_link_voltage);
        // entry.signal_registry.insert("dc_link_current", _model_outputs->dc_link_current);
        // entry.signal_registry.insert("machine_voltage_ln", _model_outputs->machine_voltage_ln);
        // entry.signal_registry.insert("machine_current", _model_outputs->machine_current);
        // entry.signal_registry.insert("rotor_position_mechanical", _model_outputs->rotor_position_mechanical);
        // entry.signal_registry.insert("rotor_speed_mechanical", _model_outputs->rotor_speed_mechanical);
        // entry.signal_registry.insert("electromagnetic_torque", _model_outputs->electromagnetic_torque);
        // entry.signal_registry.insert("grid_current_thd", _model_outputs->grid_current_thd);
        // entry.signal_registry.insert("power_factor", _model_outputs->power_factor);
        // entry.signal_registry.insert("active_power_pu", _model_outputs->active_power_pu);

        // // Register parameters in parameter registry
        // entry.parameter_registry.insert("operation_mode", _model_parameters->operation_mode);
        // entry.parameter_registry.insert("motor_inertia_constant", _model_parameters->motor_inertia_constant);
        // entry.parameter_registry.insert("motor_damping_factor", _model_parameters->motor_damping_factor);

        // registry.insert("sfc_plant_model", std::move(entry));
    }

    inputs* get_inputs() { return _model_inputs; }
    outputs* get_outputs() { return _model_outputs; }
    parameters* get_parameters() { return _model_parameters; }
};

}  // namespace yeet::application::sfc::integration
