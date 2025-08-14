#pragma once

#include "yeet/model_registry.hpp"
#include "yeet/parameter.hpp"
#include "yeet/signal.hpp"
#include "yeet_control/reference_frame.hpp"
#include "yeet_control/six_pulse_generator.hpp"  // for pulses
#include "yeet_control/units.hpp"

/**
 * @file sfc_machine_side_measurements_datamodel.hpp
 * @brief Integrated datamodel for SFC Machine-Side Measurements with filtering
 *
 * This component handles both raw measurement conditioning and signal filtering
 * for machine-side electrical and mechanical quantities.
 */
namespace yeet::application::sfc::integration {

class sfc_machine_side_measurements_datamodel {
public:
    /**
     * @brief Input signals for the machine-side measurements component
     */
    struct inputs {
        signal_input<abc<voltage_t>> machine_voltage_measurements;
        signal_input<abc<current_t>> machine_current_measurements;
        signal_input<control::six_pulse_generator::pulses> machine_pulses;
        signal_input<frequency_t> machine_side_frequency;
        signal_input<angle_wrapped> machine_side_angle;
    };

    /**
     * @brief Output signals from the machine-side measurements component
     */
    struct outputs {
        signal_output<speed_pu_t> machine_side_speed;
        signal_output<alphabeta<voltage_pu_t>> machine_voltage_alphabeta;
        signal_output<alphabeta<voltage_pu_t>> machine_voltage_alphabeta_filtered;
        signal_output<abc<voltage_pu_t>> machine_voltage_abc;
        signal_output<abc<voltage_pu_t>> machine_voltage_abc_unmasked;
        signal_output<abc<voltage_pu_t>> machine_voltage_abc_filtered;
        signal_output<abc<current_pu_t>> machine_current_abc;
        signal_output<voltage_pu_t> machine_voltage_magnitude;
        signal_output<voltage_pu_t> machine_voltage_magnitude_filtered;
        signal_output<bool> is_commuting;
    };

    /**
     * @brief Configuration parameters for measurements and filtering
     */
    struct parameters {};

private:
    inputs* _model_inputs;
    outputs* _model_outputs;
    parameters* _model_parameters;

public:
    sfc_machine_side_measurements_datamodel(inputs* model_inputs, outputs* model_outputs, parameters* model_parameters)
        : _model_inputs(model_inputs), _model_outputs(model_outputs), _model_parameters(model_parameters)
    {
        assert(_model_inputs != nullptr);
        assert(_model_outputs != nullptr);
        assert(_model_parameters != nullptr);
    }

    void register_data_model(model_registry& registry)
    {
        model_registry_entry entry{signal_registry{}, parameter_registry{}};

        // Register outputs in signal registry
        // entry.signal_registry.insert("machine_voltage_alphabeta_pu", _model_outputs->machine_voltage_alphabeta_pu);
        // entry.signal_registry.insert("machine_current_alphabeta_pu", _model_outputs->machine_current_alphabeta_pu);
        // entry.signal_registry.insert("machine_voltage_abc_pu", _model_outputs->machine_voltage_abc_pu);
        // entry.signal_registry.insert("machine_current_abc_pu", _model_outputs->machine_current_abc_pu);
        // entry.signal_registry.insert("machine_speed_pu", _model_outputs->machine_speed_pu);
        // entry.signal_registry.insert("rotor_position_electrical", _model_outputs->rotor_position_electrical);
        // entry.signal_registry.insert("machine_voltage_magnitude_pu", _model_outputs->machine_voltage_magnitude_pu);
        // entry.signal_registry.insert("machine_current_magnitude_pu", _model_outputs->machine_current_magnitude_pu);
        // entry.signal_registry.insert("back_emf_magnitude_pu", _model_outputs->back_emf_magnitude_pu);
        // entry.signal_registry.insert("current_ripple_factor", _model_outputs->current_ripple_factor);
        // entry.signal_registry.insert("is_valid", _model_outputs->is_valid);
        // entry.signal_registry.insert("filtering_active", _model_outputs->filtering_active);

        // // Register parameters in parameter registry
        // entry.parameter_registry.insert("voltage_scale_factor", _model_parameters->voltage_scale_factor);
        // entry.parameter_registry.insert("current_scale_factor", _model_parameters->current_scale_factor);
        // entry.parameter_registry.insert("speed_scale_factor", _model_parameters->speed_scale_factor);
        // entry.parameter_registry.insert("pole_pairs", _model_parameters->pole_pairs);
        // entry.parameter_registry.insert("voltage_lpf_cutoff", _model_parameters->voltage_lpf_cutoff);
        // entry.parameter_registry.insert("current_lpf_cutoff", _model_parameters->current_lpf_cutoff);
        // entry.parameter_registry.insert("speed_lpf_cutoff", _model_parameters->speed_lpf_cutoff);
        // entry.parameter_registry.insert("filter_order", _model_parameters->filter_order);
        // entry.parameter_registry.insert("current_hpf_cutoff", _model_parameters->current_hpf_cutoff);
        // entry.parameter_registry.insert("anti_aliasing_frequency", _model_parameters->anti_aliasing_frequency);
        // entry.parameter_registry.insert("ripple_calculation_window", _model_parameters->ripple_calculation_window);

        // registry.insert("sfc_machine_side_measurements", std::move(entry));
    }

    inputs* get_inputs() { return _model_inputs; }
    outputs* get_outputs() { return _model_outputs; }
    parameters* get_parameters() { return _model_parameters; }
};

}  // namespace yeet::application::sfc::integration
