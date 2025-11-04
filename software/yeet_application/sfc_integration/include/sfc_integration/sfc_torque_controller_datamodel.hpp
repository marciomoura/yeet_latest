#pragma once

#include "yeet/model_registry.hpp"
#include "yeet/parameter.hpp"
#include "yeet/signal.hpp"
#include "yeet/utils/parameter_accessors.hpp"
#include "yeet_control/units.hpp"

/**
 * @file sfc_torque_controller_datamodel.hpp
 * @brief Datamodel specification for SFC Torque Controller component
 *
 * The torque controller sits between the speed controller and current controller,
 * managing torque-level operations including breakaway boost, dynamic limiting,
 * and torque-to-current conversion.
 */

namespace yeet::application::sfc {
// Forward declaration for control_mode enum
class sfc_torque_controller;
}  // namespace yeet::application::sfc

namespace yeet::application::sfc::integration {

class sfc_torque_controller_datamodel {
public:
    /**
     * @brief Input signals for the torque controller component
     */
    struct inputs {
        /// Torque reference from speed controller [pu]
        signal_input<torque_pu_t> torque_reference;

        /// Maximum dynamic torque limit from speed generator [pu]
        signal_input<torque_pu_t> dynamic_max_torque_limit;

        /// Minimum dynamic torque limit from speed generator [pu]
        signal_input<torque_pu_t> dynamic_min_torque_limit;

        /// Enable signal for breakaway boost from speed generator
        signal_input<bool> enable_breakaway_boost;

        /// Saturation feedback from downstream current controller
        signal_input<bool> is_saturated;
    };

    /**
     * @brief Output signals from the torque controller component
     */
    struct outputs {
        /// Current reference output to current controller [pu]
        signal_output<current_pu_t> current_reference;

        /// Base torque reference before boost [pu]
        signal_output<torque_pu_t> base_torque_reference;

        /// Total torque reference after boost injection [pu]
        signal_output<torque_pu_t> total_torque_reference;

        /// Limited torque reference after all limits applied [pu]
        signal_output<torque_pu_t> limited_torque_reference;

        /// Local saturation status (torque limits exceeded)
        signal_output<bool> is_saturated;

        /// Downstream saturation status (from current controller)
        signal_output<bool> downstream_saturation;

        /// Composite saturation status (local OR downstream)
        signal_output<bool> composite_saturation;

        /// Breakaway boost active status
        signal_output<bool> is_breakaway_boost_active;

        /// Control loop active status
        signal_output<bool> is_active;
    };

    /**
     * @brief Configuration parameters for the torque controller
     */
    struct parameters {
        parameter<bool> override_torque_reference = false;

        /// Overrides normal control operation for testing or specific control modes
        parameter<torque_pu_t> forced_torque_reference = torque_pu_t{0.0};

        /// Absolute minimum torque limit [pu]
        /// Hard limit representing physical machine capability
        parameter<torque_pu_t> absolute_min_torque = torque_pu_t{-1.2};

        /// Absolute maximum torque limit [pu]
        /// Hard limit representing physical machine capability
        parameter<torque_pu_t> absolute_max_torque = torque_pu_t{1.2};

        /// Torque-to-current conversion factor [dimensionless]
        /// Typically close to 1.0 in a well-designed per-unit system
        parameter<real_t> torque_to_current_factor = real_t(1.0);

        /// Torque rate limit [pu/s]
        /// Limits how fast torque can change to prevent mechanical shock
        parameter<real_t> torque_rate_limit = real_t(10.0);

        /// Enable/disable torque rate limiting
        parameter<bool> enable_torque_rate_limiting = true;

        /// Breakaway boost torque magnitude [pu]
        /// Extra torque added during startup to overcome static friction
        parameter<torque_pu_t> breakaway_boost_torque = torque_pu_t{0.0};

        /// Breakaway boost duration [s]
        /// How long the boost is applied
        parameter<duration_t> breakaway_boost_duration = duration_t{1.0};
    };

private:
    inputs* _model_inputs;
    outputs* _model_outputs;
    parameters* _model_parameters;

public:
    sfc_torque_controller_datamodel(inputs* model_inputs, outputs* model_outputs, parameters* model_parameters)
        : _model_inputs(model_inputs), _model_outputs(model_outputs), _model_parameters(model_parameters)
    {
        assert(_model_inputs != nullptr);
        assert(_model_outputs != nullptr);
        assert(_model_parameters != nullptr);
    }

    void register_data_model(model_registry& registry)
    {
        model_registry_entry entry{signal_registry{}, parameter_registry{}};

        entry.parameters.insert("override_torque_reference",
            yeet::utils::make_units_accessor(_model_parameters->override_torque_reference));
        entry.parameters.insert(
            "forced_torque_reference", yeet::utils::make_units_accessor(_model_parameters->forced_torque_reference));
        entry.parameters.insert(
            "absolute_min_torque", yeet::utils::make_units_accessor(_model_parameters->absolute_min_torque));
        entry.parameters.insert(
            "absolute_max_torque", yeet::utils::make_units_accessor(_model_parameters->absolute_max_torque));
        entry.parameters.insert("torque_to_current_factor", _model_parameters->torque_to_current_factor);
        entry.parameters.insert("torque_rate_limit", _model_parameters->torque_rate_limit);
        entry.parameters.insert("enable_torque_rate_limiting", _model_parameters->enable_torque_rate_limiting);
        entry.parameters.insert(
            "breakaway_boost_torque", yeet::utils::make_units_accessor(_model_parameters->breakaway_boost_torque));
        entry.parameters.insert(
            "breakaway_boost_duration", yeet::utils::make_units_accessor(_model_parameters->breakaway_boost_duration));

        registry.insert("torque_controller", std::move(entry));
    }

    inputs* get_inputs() { return _model_inputs; }
    outputs* get_outputs() { return _model_outputs; }
    parameters* get_parameters() { return _model_parameters; }
};

}  // namespace yeet::application::sfc::integration
