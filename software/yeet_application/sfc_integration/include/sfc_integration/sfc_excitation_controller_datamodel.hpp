#pragma once

#include "yeet/model_registry.hpp"
#include "yeet/parameter.hpp"
#include "yeet/signal.hpp"
#include "yeet/utils/parameter_accessors.hpp"
#include "yeet_control/units.hpp"

/**
 * @file sfc_excitation_controller_datamodel.hpp
 * @brief Datamodel specification for SFC Excitation Controller component
 */

namespace yeet::application::sfc::integration {

class sfc_excitation_controller_datamodel {
public:
    /**
     * @brief Input signals for the excitation controller component
     */
    struct inputs {
        /// Machine rotor speed in per-unit
        signal_input<speed_pu_t> machine_rotor_speed_pu;

        /// Exciter ready status from external excitation controller
        signal_input<bool> exciter_ready;

        /// Command to start excitation sequence
        signal_input<bool> start_command;

        /// Command to stop excitation sequence
        signal_input<bool> stop_command;
    };

    /**
     * @brief Output signals from the excitation controller component
     */
    struct outputs {
        /// Excitation voltage setpoint to send to exciter [pu]
        signal_output<voltage_pu_t> excitation_voltage_setpoint;

        /// Enable command to external exciter
        signal_output<bool> enable_excitation;

        /// Field established status flag
        signal_output<bool> is_field_established;

        /// Controller running status
        signal_output<bool> is_running;

        /// Pre-fluxing elapsed time [s]
        signal_output<duration_t> prefluxing_elapsed_time;
    };

    /**
     * @brief Configuration parameters for the excitation controller
     */
    struct parameters {
        /// Pre-fluxing duration [seconds]
        parameter<duration_t> prefluxing_duration = duration_t{2.0};

        /// Field established timeout [seconds]
        parameter<duration_t> field_established_timeout = duration_t{5.0};

        /// Excitation ramp-down time during shutdown [seconds]
        parameter<duration_t> excitation_ramp_down_time = duration_t{1.0};

        /// Speed axis for lookup table [pu]
        parameter<std::array<real_t, 6>> speed_axis_pu = std::array<real_t, 6>{0.0, 0.05, 0.80, 0.95, 1.00, 1.05};

        /// Excitation voltage axis for lookup table [pu]
        parameter<std::array<real_t, 6>> excitation_voltage_axis_pu =
            std::array<real_t, 6>{0.30, 0.30, 1.00, 0.80, 0.73, 0.70};
    };

private:
    inputs* _model_inputs;
    outputs* _model_outputs;
    parameters* _model_parameters;

public:
    sfc_excitation_controller_datamodel(inputs* model_inputs, outputs* model_outputs, parameters* model_parameters)
        : _model_inputs(model_inputs), _model_outputs(model_outputs), _model_parameters(model_parameters)
    {
        assert(_model_inputs != nullptr);
        assert(_model_outputs != nullptr);
        assert(_model_parameters != nullptr);
    }

    void register_data_model(model_registry& registry)
    {
        model_registry_entry entry{signal_registry{}, parameter_registry{}};

        entry.parameters.insert(
            "prefluxing_duration", yeet::utils::make_units_accessor(_model_parameters->prefluxing_duration));
        entry.parameters.insert("field_established_timeout",
            yeet::utils::make_units_accessor(_model_parameters->field_established_timeout));
        entry.parameters.insert("excitation_ramp_down_time",
            yeet::utils::make_units_accessor(_model_parameters->excitation_ramp_down_time));

        // Register speed axis lookup table entries individually
        auto& speed_axis = _model_parameters->speed_axis_pu;
        for (std::size_t i = 0; i < 6; ++i) {
            std::string param_name = "speed_axis_pu[" + std::to_string(i) + "]";
            entry.parameters.insert(
                param_name, low_level_access([&speed_axis, i]() -> float { return speed_axis.read()[i]; },
                                [&speed_axis, i](float value) {
                                    auto table = speed_axis.read();
                                    table[i] = real_t(value);
                                    speed_axis.write(table);
                                }));
        }

        // Register excitation voltage axis lookup table entries individually
        auto& excitation_voltage_axis = _model_parameters->excitation_voltage_axis_pu;
        for (std::size_t i = 0; i < 6; ++i) {
            std::string param_name = "excitation_voltage_axis_pu[" + std::to_string(i) + "]";
            entry.parameters.insert(param_name,
                low_level_access([&excitation_voltage_axis, i]() -> float { return excitation_voltage_axis.read()[i]; },
                    [&excitation_voltage_axis, i](float value) {
                        auto table = excitation_voltage_axis.read();
                        table[i] = real_t(value);
                        excitation_voltage_axis.write(table);
                    }));
        }

        registry.insert("excitation_controller", std::move(entry));
    }

    inputs* get_inputs() { return _model_inputs; }
    outputs* get_outputs() { return _model_outputs; }
    parameters* get_parameters() { return _model_parameters; }
};

}  // namespace yeet::application::sfc::integration
