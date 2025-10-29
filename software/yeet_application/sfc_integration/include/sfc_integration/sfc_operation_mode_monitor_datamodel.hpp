#pragma once

#include "sfc/sfc_types.hpp"
#include "yeet/model_registry.hpp"
#include "yeet/parameter.hpp"
#include "yeet/signal.hpp"
#include "yeet/utils/parameter_accessors.hpp"
#include "yeet_control/units.hpp"

namespace yeet::application::sfc::integration {

/**
 * @class sfc_operation_mode_monitor_datamodel
 * @brief Defines the data interface for the SFC Operation Mode Monitor.
 *
 * This datamodel aggregates all signals and parameters required for detecting
 * the primary LCI operating mode (pulse vs. load-commutated) and for monitoring
 * discontinuous conduction mode based on DC link current.
 */
class sfc_operation_mode_monitor_datamodel {
public:
    /**
     * @brief Input signals for the operation mode monitor.
     */
    struct inputs {
        /// @brief System enable signal. When false, detectors are reset to a safe state.
        signal_input<bool> enable;

        /// @brief Measured machine speed in per-unit for mode detection.
        signal_input<speed_pu_t> measured_speed_pu;

        /// @brief Measured DC link current in per-unit for discontinuous conduction detection.
        signal_input<current_pu_t> dc_link_current_pu;
    };

    /**
     * @brief Output signals from the operation mode monitor.
     */
    struct outputs {
        /// @brief The current operating mode (forced_commutation or natural_commutation).
        signal_output<sfc_operating_mode_t> operating_mode;

        /// @brief True for one cycle immediately after an operating mode transition.
        signal_output<bool> has_mode_changed;

        /// @brief True if the DC link current is considered zero (with on-delay).
        signal_output<bool> is_current_zero;

        /// @brief True if the system is determined to be in discontinuous conduction mode.
        signal_output<bool> is_discontinuous_mode;
    };

    /**
     * @brief Configuration parameters for the operation mode monitor.
     */
    struct parameters {
        // Mode Detector Parameters
        /// @brief Base speed for switching between pulse and load-commutated modes.
        parameter<speed_pu_t> mode_handover_speed_pu{speed_pu_t{0.25f}};

        /// @brief Hysteresis factor for mode switching to prevent chattering.
        parameter<real_t> mode_hysteresis_factor{0.15f};

        // Discontinuous Conduction Detector Parameters
        /// @brief Current level below which DC link current is considered zero.
        parameter<current_pu_t> zero_current_threshold_pu{current_pu_t{0.02f}};

        /// @brief On-delay in seconds to confirm current is zero.
        parameter<duration_t> is_zero_on_delay_s{duration_t{(1.0 / 50.0 / 6.0)}};

        parameter<duration_t> discontinuous_on_delay_s{duration_t{0.002f}};
        parameter<duration_t> discontinuous_off_delay_s{duration_t{0.05f}};
    };

private:
    inputs* _model_inputs;
    outputs* _model_outputs;
    parameters* _model_parameters;

public:
    sfc_operation_mode_monitor_datamodel(inputs* model_inputs, outputs* model_outputs, parameters* model_parameters)
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
            "mode_handover_speed_pu", yeet::utils::make_units_accessor(_model_parameters->mode_handover_speed_pu));
        entry.parameters.insert("mode_hysteresis_factor", _model_parameters->mode_hysteresis_factor);
        entry.parameters.insert("zero_current_threshold_pu",
            yeet::utils::make_units_accessor(_model_parameters->zero_current_threshold_pu));
        entry.parameters.insert(
            "is_zero_on_delay_s", yeet::utils::make_units_accessor(_model_parameters->is_zero_on_delay_s));
        entry.parameters.insert(
            "discontinuous_on_delay_s", yeet::utils::make_units_accessor(_model_parameters->discontinuous_on_delay_s));
        entry.parameters.insert("discontinuous_off_delay_s",
            yeet::utils::make_units_accessor(_model_parameters->discontinuous_off_delay_s));

        registry.insert("operation_mode_monitor", std::move(entry));
    }

    inputs* get_inputs() { return _model_inputs; }
    outputs* get_outputs() { return _model_outputs; }
    parameters* get_parameters() { return _model_parameters; }
};

}  // namespace yeet::application::sfc::integration
