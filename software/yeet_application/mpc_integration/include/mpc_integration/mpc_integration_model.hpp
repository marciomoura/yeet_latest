#pragma once

#include <cmath>
#include <cstdint>

#include "yeet/model_registry.hpp"
#include "yeet/parameter.hpp"
#include "yeet/signal.hpp"
#include "yeet_control/constants.hpp"

namespace yeet::application::mpc {

class mpc_integration_datamodel {
public:
    struct inputs {
        yeet::signal_input<double> wm;
        yeet::signal_input<double> ifd;
        yeet::signal_input<double> it;
        yeet::signal_input<double> id;
        yeet::signal_input<double> iq;
        yeet::signal_input<double> vt;
        yeet::signal_input<double> vd;
        yeet::signal_input<double> vq;
        yeet::signal_input<double> vg;
        yeet::signal_input<double> load_angle_deg;
    };

    struct parameters {
        yeet::parameter<double> l_s = 0.15f;
        yeet::parameter<double> l_ad = 1.35f;
        yeet::parameter<double> l_aq = 0.85f;
        yeet::parameter<double> l_fd = 0.4725f;
        yeet::parameter<double> r_s = 0.003f;
        yeet::parameter<double> r_fd = 0.000537147932935147f;
        yeet::parameter<double> w_base = 2.0f * yeet::control::pi * 60.0f;
        yeet::parameter<double> h = 3.5f;
        yeet::parameter<double> kd = 0.5f;
        yeet::parameter<double> l_g = 0.1f;
        yeet::parameter<double> r_g = 0.1f / 20.0f;

        yeet::parameter<double> mpc_feedforward_output = 1.0f;
        yeet::parameter<double> mpc_lambda = 1e-4f;
        yeet::parameter<double> mpc_terminal_voltage_reference = 1.0f;
        yeet::parameter<double> mpc_speed_deviation_penalization = 1000.0f;
        yeet::parameter<double> mpc_umax = 15.0f;
        yeet::parameter<double> mpc_umin = -15.0f;
    };

    struct outputs {
        yeet::signal_output<double> efd;
        yeet::signal_output<std::int32_t> nwsr;
        yeet::signal_output<std::uint32_t> model_update_counter;
    };

    mpc_integration_datamodel(inputs* model_inputs, outputs* model_outputs, parameters* model_parameters)
        : _model_inputs(model_inputs), _model_outputs(model_outputs), _model_parameters(model_parameters) {
        assert(_model_inputs != nullptr);
        assert(_model_outputs != nullptr);
        assert(_model_parameters != nullptr);
    }

    void register_data_model(yeet::model_registry& registry) {
        yeet::model_registry_entry entry{yeet::signal_registry{}, yeet::parameter_registry{}};

        // entry.parameter_registry.insert("l_s", _model_parameters->l_s);
        // entry.parameter_registry.insert("l_ad", _model_parameters->l_ad);
        // entry.parameter_registry.insert("l_aq", _model_parameters->l_aq);
        // entry.parameter_registry.insert("l_fd", _model_parameters->l_fd);
        // entry.parameter_registry.insert("r_s", _model_parameters->r_s);
        // entry.parameter_registry.insert("r_fd", _model_parameters->r_fd);
        // entry.parameter_registry.insert("w_base", _model_parameters->w_base);
        // entry.parameter_registry.insert("h", _model_parameters->h);
        // entry.parameter_registry.insert("kd", _model_parameters->kd);
        // entry.parameter_registry.insert("l_g", _model_parameters->l_g);
        // entry.parameter_registry.insert("r_g", _model_parameters->r_g);

        entry.signal_registry.insert("efd", _model_outputs->efd);
        entry.signal_registry.insert("nwsr", _model_outputs->nwsr);
        entry.signal_registry.insert("model_counter", _model_outputs->model_update_counter);

        registry.insert("control", std::move(entry));
    }

    inputs* get_inputs() { return _model_inputs; }
    outputs* get_outputs() { return _model_outputs; }
    parameters* get_parameters() { return _model_parameters; }

private:
    inputs* _model_inputs{};
    outputs* _model_outputs{};
    parameters* _model_parameters{};
};

// entry.signal_registry.insert("rotor_speed", model_inputs->rotor_speed);
// entry.signal_registry.insert("field_current", model_inputs->field_current);
// entry.signal_registry.insert("terminal_current_magnitude", model_inputs->terminal_current_magnitude);
// entry.signal_registry.insert("terminal_current_d", model_inputs->terminal_current_d);
// entry.signal_registry.insert("terminal_current_q", model_inputs->terminal_current_q);
// entry.signal_registry.insert("terminal_voltage_magnitude", model_inputs->terminal_voltage_magnitude);
// entry.signal_registry.insert("terminal_voltage_d", model_inputs->terminal_voltage_d);
// entry.signal_registry.insert("terminal_voltage_q", model_inputs->terminal_voltage_q);
// entry.signal_registry.insert("grid_voltage", model_inputs->grid_voltage);
// entry.signal_registry.insert("delta", model_inputs->delta);
}  // namespace yeet::application::mpc
