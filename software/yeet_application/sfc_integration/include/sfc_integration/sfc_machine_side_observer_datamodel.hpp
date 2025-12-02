#pragma once

#include "yeet/model_registry.hpp"
#include "yeet/parameter.hpp"
#include "yeet/signal.hpp"
#include "yeet/utils/parameter_accessors.hpp"
#include "yeet_control/reference_frame.hpp"
#include "yeet_control/units.hpp"

/**
 * @file sfc_machine_side_observer_datamodel.hpp
 * @brief Datamodel specification for SFC Machine-Side Observer component
 */

namespace yeet::application::sfc::integration {

class sfc_machine_side_observer_datamodel {
public:
    /**
     * @brief Input signals for the machine-side observer component
     */
    struct inputs {
        /// Three-phase machine voltage measurements in per-unit (machine-side base)
        yeet::signal_input<yeet::abc<yeet::voltage_pu_t>> machine_voltage_pu;

        /// Enable signal for observer execution (from startup sequencer)
        yeet::signal_input<bool> release_observer;

        /// Preset signal to initialize observer angle integrator (from startup sequencer)
        yeet::signal_input<bool> preset_observer;

        /// Preset angle value to initialize observer integrator [rad]
        yeet::signal_input<angle_wrapped> preset_angle;
    };

    /**
     * @brief Output signals from the machine-side observer component
     */
    struct outputs {
        /// Estimated voltage magnitude [pu]
        yeet::signal_output<yeet::voltage_pu_t> estimated_voltage_magnitude;

        /// Estimated frequency [Hz]
        yeet::signal_output<yeet::frequency_t> estimated_frequency;

        /// Estimated frequency in per-unit [pu]
        yeet::signal_output<yeet::frequency_pu_t> estimated_frequency_pu;

        /// Estimated machine speed [pu]
        yeet::signal_output<yeet::speed_pu_t> estimated_speed;

        /// Estimated rotor phase angle [rad]
        yeet::signal_output<angle_wrapped> estimated_reference_angle;

        /// Filtered voltage in alpha-beta stationary frame [pu]
        yeet::signal_output<yeet::alphabeta<yeet::voltage_pu_t>> estimated_voltage_alphabeta_filtered;
    };

    /**
     * @brief Configuration parameters for the machine-side observer
     */
    struct parameters {
        /// PLL proportional gain
        yeet::parameter<real_t> kp = 0.4;

        /// PLL integral time constant [s]
        yeet::parameter<duration_t> ti = duration_t{ms(31.8)};

        /// Nominal machine frequency [Hz]
        yeet::parameter<frequency_t> nominal_frequency = frequency_t{50.0};

        /// Initial frequency estimate for startup [pu]
        yeet::parameter<frequency_pu_t> initial_frequency = frequency_pu_t{0.01};

        /// Initial position for integrator preset [rad]
        yeet::parameter<angle_t> initial_position = angle_t{0.0};

        /// Frequency estimation filter cutoff frequency [Hz]
        yeet::parameter<frequency_t> frequency_filter_cutoff = frequency_t{10.0};

        /// Commutation detection voltage threshold [pu]
        yeet::parameter<real_t> commutation_detection_threshold = 0.15;
    };

private:
    inputs* _model_inputs;
    outputs* _model_outputs;
    parameters* _model_parameters;

public:
    sfc_machine_side_observer_datamodel(inputs* model_inputs, outputs* model_outputs, parameters* model_parameters)
        : _model_inputs(model_inputs), _model_outputs(model_outputs), _model_parameters(model_parameters)
    {
        assert(_model_inputs != nullptr);
        assert(_model_outputs != nullptr);
        assert(_model_parameters != nullptr);
    }

    void register_data_model(yeet::model_registry& registry)
    {
        model_registry_entry entry{signal_registry{}, parameter_registry{}};

        entry.parameters.insert("kp", yeet::utils::make_units_accessor(_model_parameters->kp));
        entry.parameters.insert("ti", yeet::utils::make_units_accessor(_model_parameters->ti));
        entry.parameters.insert(
            "nominal_frequency", yeet::utils::make_units_accessor(_model_parameters->nominal_frequency));
        entry.parameters.insert(
            "initial_frequency", yeet::utils::make_units_accessor(_model_parameters->initial_frequency));
        entry.parameters.insert(
            "initial_position", yeet::utils::make_units_accessor(_model_parameters->initial_position));
        entry.parameters.insert(
            "frequency_filter_cutoff", yeet::utils::make_units_accessor(_model_parameters->frequency_filter_cutoff));
        entry.parameters.insert("commutation_detection_threshold",
            yeet::utils::make_units_accessor(_model_parameters->commutation_detection_threshold));

        registry.insert("observer", std::move(entry));
    }

    inputs* get_inputs() { return _model_inputs; }
    outputs* get_outputs() { return _model_outputs; }
    parameters* get_parameters() { return _model_parameters; }
};

}  // namespace yeet::application::sfc::integration
