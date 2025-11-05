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
     * @brief Input signals for the operation mode monitor
     */
    struct inputs {
        /// System enable signal [bool]
        signal_input<bool> enable;

        /// Measured machine speed for mode detection [pu]
        signal_input<speed_pu_t> measured_speed;

        /// Measured DC link current for discontinuous conduction detection [pu]
        signal_input<current_pu_t> dc_link_current;
    };

    /**
     * @brief Output signals from the operation mode monitor
     */
    struct outputs {
        /// Current operating mode (forced vs natural commutation)
        signal_output<sfc_operating_mode_t> operating_mode;

        /// True for one cycle after mode transition [bool]
        signal_output<bool> has_mode_changed;

        /// DC link current is considered zero [bool]
        signal_output<bool> current_is_zero;

        /// System is in discontinuous conduction mode [bool]
        signal_output<bool> is_discontinuous_mode;
    };

    /**
     * @brief Configuration parameters for the operation mode monitor
     */
    struct parameters {
        /// Speed threshold for mode switching [pu]
        parameter<speed_pu_t> mode_handover_speed_pu{speed_pu_t{0.15f}};

        /// Hysteresis factor for mode switching [dimensionless]
        parameter<real_t> mode_hysteresis_factor{0.15f};

        /// Current threshold for zero current detection [pu]
        parameter<current_pu_t> zero_current_threshold_pu{current_pu_t{0.02f}};

        /// On-delay for zero current detection [s]
        parameter<duration_t> is_zero_on_delay_s{duration_t{(1.0 / 50.0 / 6.0)}};

        /// On-delay for discontinuous conduction detection [s]
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
