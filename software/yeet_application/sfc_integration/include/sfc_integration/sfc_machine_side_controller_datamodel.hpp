#pragma once

#include "sfc/sfc_machine_side_lookup_table.hpp"
#include "yeet/model_registry.hpp"
#include "yeet/parameter.hpp"
#include "yeet/signal.hpp"
#include "yeet/utils/parameter_accessors.hpp"
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
        signal_output<angle_wrapped> firing_angle;
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

    void register_data_model(model_registry& registry)
    {
        model_registry_entry entry{signal_registry{}, parameter_registry{}};

        // Register DC current lookup table entries individually
        static constexpr std::string_view dc_current_names[] = {"dc_current_pu_lookup_table[0]",
            "dc_current_pu_lookup_table[1]", "dc_current_pu_lookup_table[2]", "dc_current_pu_lookup_table[3]",
            "dc_current_pu_lookup_table[4]", "dc_current_pu_lookup_table[5]"};

        for (std::size_t i = 0; i < parameters::NUM_POINTS; ++i) {
            entry.parameters.insert(std::string(dc_current_names[i]),
                yeet::utils::make_array_element_accessor(_model_parameters->dc_current_pu_lookup_table, i));
        }

        // Register firing angle lookup table entries individually (real_t doesn't have .value() method)
        static constexpr std::string_view firing_angle_names[] = {"firing_angle_lookup_table_deg[0]",
            "firing_angle_lookup_table_deg[1]", "firing_angle_lookup_table_deg[2]", "firing_angle_lookup_table_deg[3]",
            "firing_angle_lookup_table_deg[4]", "firing_angle_lookup_table_deg[5]"};

        // real_t is a float type, so we can use direct casting
        for (std::size_t i = 0; i < parameters::NUM_POINTS; ++i) {
            // Create static context storage for each index
            static struct context_storage_t {
                parameter<std::array<real_t, parameters::NUM_POINTS>>* param;
                std::size_t index;
            } contexts[parameters::NUM_POINTS];

            contexts[i] = {&_model_parameters->firing_angle_lookup_table, i};

            entry.parameters.insert(
                std::string(firing_angle_names[i]), low_level_access(
                                                        +[](void* ctx) -> float {
                                                            auto* c = static_cast<context_storage_t*>(ctx);
                                                            return static_cast<float>(c->param->read()[c->index]);
                                                        },
                                                        +[](void* ctx, float value) {
                                                            auto* c = static_cast<context_storage_t*>(ctx);
                                                            auto table = c->param->read();
                                                            table[c->index] = real_t(value);
                                                            c->param->write(table);
                                                        },
                                                        &contexts[i]));
        }

        registry.insert("stator_voltage_controller", std::move(entry));
    }

    inputs* get_inputs() { return _model_inputs; }
    outputs* get_outputs() { return _model_outputs; }
    parameters* get_parameters() { return _model_parameters; }
};

}  // namespace yeet::application::sfc::integration
