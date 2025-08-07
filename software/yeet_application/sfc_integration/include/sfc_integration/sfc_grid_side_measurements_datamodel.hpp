#pragma once

#include "yeet/model_registry.hpp"
#include "yeet/parameter.hpp"
#include "yeet/signal.hpp"
#include "yeet_control/angle_wrapped.hpp"
#include "yeet_control/reference_frame.hpp"
#include "yeet_control/six_pulse_generator.hpp"  // for pulses
#include "yeet_control/units.hpp"

//     _is_commuting = _commutation_monitor.update(grid_voltage_pu, grid_side_pulses);

//  abc<voltage_pu_t> masked_grid_voltage_pu = _commutation_mask.update(
//      grid_voltage_pu, _pll.get_estimated_angle(), frequency_t{_pll.get_estimated_frequency()}, _is_commuting);

//  // Convert masked voltage to dimensionless alpha-beta for PLL
//  alphabeta<voltage_pu_t> const masked_voltage = masked_grid_voltage_pu.to_alphabeta();

/**
 * @file sfc_grid_side_measurements_datamodel.hpp
 * @brief Integrated datamodel for SFC Grid-Side Measurements with filtering
 *
 * This component handles both raw measurement conditioning and signal filtering
 * to provide clean, filtered measurement signals for control algorithms.
 */
namespace yeet::application::sfc::integration {

class sfc_grid_side_measurements_datamodel {
public:
    /**
     * @brief Input signals for the grid-side measurements component
     */
    struct inputs {
        signal_input<abc<voltage_t>> grid_voltage_measurements;
        signal_input<abc<current_t>> grid_current_measurements;

        signal_input<control::six_pulse_generator::pulses> grid_pulses;
        signal_input<frequency_t> grid_side_frequency;
        signal_input<angle_wrapped> grid_side_angle;
    };

    /**
     * @brief Output signals from the grid-side measurements component
     */
    struct outputs {
        signal_output<alphabeta<voltage_pu_t>> grid_voltage_alphabeta;
        signal_output<alphabeta<voltage_pu_t>> grid_voltage_alphabeta_filtered;
        signal_output<abc<voltage_pu_t>> grid_voltage_abc;
        signal_output<abc<voltage_pu_t>> grid_voltage_abc_unmasked;
        signal_output<abc<voltage_pu_t>> grid_voltage_abc_filtered;
        signal_output<abc<current_pu_t>> grid_current_abc;
        signal_output<voltage_pu_t> grid_voltage_magnitude;
        signal_output<voltage_pu_t> grid_voltage_magnitude_filtered;
        signal_output<bool> is_commuting;
    };

    /**
     * @brief Configuration parameters for measurements and filtering
     */
    struct parameters {
        // Scaling parameters
        parameter<frequency_t> lpf_cutoff_frequency = frequency_t(200.0);
    };

private:
    inputs* _model_inputs;
    outputs* _model_outputs;
    parameters* _model_parameters;

public:
    sfc_grid_side_measurements_datamodel(inputs* model_inputs, outputs* model_outputs, parameters* model_parameters)
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
        // entry.signal_registry.insert("grid_voltage_alphabeta_pu", _model_outputs->grid_voltage_alphabeta_pu);
        // entry.signal_registry.insert("grid_current_alphabeta_pu", _model_outputs->grid_current_alphabeta_pu);
        // entry.signal_registry.insert("grid_voltage_abc_pu", _model_outputs->grid_voltage_abc_pu);
        // entry.signal_registry.insert("grid_current_abc_pu", _model_outputs->grid_current_abc_pu);
        // entry.signal_registry.insert("grid_voltage_magnitude_pu", _model_outputs->grid_voltage_magnitude_pu);
        // entry.signal_registry.insert("grid_current_magnitude_pu", _model_outputs->grid_current_magnitude_pu);
        // entry.signal_registry.insert("dc_voltage_pu", _model_outputs->dc_voltage_pu);
        // entry.signal_registry.insert("voltage_thd", _model_outputs->voltage_thd);
        // entry.signal_registry.insert("current_thd", _model_outputs->current_thd);
        // entry.signal_registry.insert("is_valid", _model_outputs->is_valid);
        // entry.signal_registry.insert("filtering_active", _model_outputs->filtering_active);

        // // Register parameters in parameter registry
        // entry.parameter_registry.insert("voltage_scale_factor", _model_parameters->voltage_scale_factor);
        // entry.parameter_registry.insert("current_scale_factor", _model_parameters->current_scale_factor);
        // entry.parameter_registry.insert("lpf_cutoff_frequency", _model_parameters->lpf_cutoff_frequency);
        // entry.parameter_registry.insert("lpf_filter_order", _model_parameters->lpf_filter_order);
        // entry.parameter_registry.insert("notch_frequency_5th", _model_parameters->notch_frequency_5th);
        // entry.parameter_registry.insert("notch_frequency_7th", _model_parameters->notch_frequency_7th);
        // entry.parameter_registry.insert("notch_frequency_11th", _model_parameters->notch_frequency_11th);
        // entry.parameter_registry.insert("notch_frequency_13th", _model_parameters->notch_frequency_13th);
        // entry.parameter_registry.insert("notch_q_factor", _model_parameters->notch_q_factor);
        // entry.parameter_registry.insert("anti_aliasing_frequency", _model_parameters->anti_aliasing_frequency);
        // entry.parameter_registry.insert("thd_calculation_window", _model_parameters->thd_calculation_window);

        // registry.insert("sfc_grid_side_measurements", std::move(entry));
    }

    inputs* get_inputs() { return _model_inputs; }
    outputs* get_outputs() { return _model_outputs; }
    parameters* get_parameters() { return _model_parameters; }
};

}  // namespace yeet::application::sfc::integration
