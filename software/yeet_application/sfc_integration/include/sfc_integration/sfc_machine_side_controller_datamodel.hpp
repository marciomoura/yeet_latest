#pragma once

#include "sfc/sfc_machine_side_lookup_table.hpp"
#include "yeet/model_registry.hpp"
#include "yeet/parameter.hpp"
#include "yeet/signal.hpp"
#include "yeet_control/angle_wrapped.hpp"
#include "yeet_control/units.hpp"

namespace yeet::application::sfc::integration {

class sfc_machine_side_controller_datamodel {
public:
    /**
     * @brief Input signals for the machine side controller component
     */
    struct inputs {
        /// Measured machine electrical frequency in per-unit
        signal_input<speed_pu_t> machine_speed;

        /// Measured DC link current in Amperes
        signal_input<current_pu_t> dc_link_current;
    };

    /**
     * @brief Output signals from the machine side controller component
     */
    struct outputs {
        /// Calculated firing angle
        signal_output<angle_wrapped> machine_side_firing_angle;
    };

    /**
     * @brief Configuration parameters for the machine side controller
     */
    struct parameters {
        static constexpr std::size_t NUM_POINTS = sfc_machine_side_firing_angle_lookup_table::NUM_POINTS;

        parameter<std::array<current_pu_t, NUM_POINTS>> dc_current_pu_lookup_table =
            sfc_machine_side_firing_angle_lookup_table::kDefaultDcCurrentAxisPu;

        parameter<std::array<real_t, NUM_POINTS>> firing_angle_lookup_table =
            sfc_machine_side_firing_angle_lookup_table::kDefaultFiringAngleDeg;
    };

private:
    inputs* _model_inputs;
    outputs* _model_outputs;
    parameters* _model_parameters;

public:
    sfc_machine_side_controller_datamodel(inputs* model_inputs, outputs* model_outputs, parameters* model_parameters)
        : _model_inputs(model_inputs), _model_outputs(model_outputs), _model_parameters(model_parameters)
    {
        assert(_model_inputs != nullptr);
        assert(_model_outputs != nullptr);
        assert(_model_parameters != nullptr);
    }

    void register_data_model(model_registry& registry) {}

    inputs* get_inputs() { return _model_inputs; }
    outputs* get_outputs() { return _model_outputs; }
    parameters* get_parameters() { return _model_parameters; }
};

}  // namespace yeet::application::sfc::integration
