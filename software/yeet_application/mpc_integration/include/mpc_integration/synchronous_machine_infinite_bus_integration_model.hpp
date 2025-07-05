#pragma once

#include <Eigen/Dense>

#include "yeet/model_registry.hpp"
#include "yeet/parameter.hpp"
#include "yeet/signal.hpp"

namespace yeet::application::mpc {

class synchronous_machine_infinite_bus_integration_datamodel {
public:
    struct inputs {
        yeet::signal_input<double> efd;
    };

    struct parameters {
        yeet::parameter<double> pmech = 0.75f;
        //     parameter<double> l_s;
        //     parameter<double> l_ad;
        //     parameter<double> l_aq;
        //     parameter<double> l_fd;
        //     parameter<double> r_s;
        //     parameter<double> r_fd;
        //     parameter<double> w_base;
        //     parameter<double> h;
        //     parameter<double> kd;
        //     parameter<double> l_g;
        //     parameter<double> r_g;
    };

    struct outputs {
        yeet::signal_output<double> wm;
        yeet::signal_output<double> wt;
        yeet::signal_output<double> ifd;
        yeet::signal_output<double> it;
        yeet::signal_output<double> id;
        yeet::signal_output<double> iq;
        yeet::signal_output<double> vt;
        yeet::signal_output<double> vd;
        yeet::signal_output<double> vq;
        yeet::signal_output<double> vg;
        yeet::signal_output<double> load_angle_deg;
        yeet::signal_output<double> voltage_alpha;
        yeet::signal_output<double> voltage_beta;
        yeet::signal_output<double> current_alpha;
        yeet::signal_output<double> current_beta;
    };

    synchronous_machine_infinite_bus_integration_datamodel(inputs* model_inputs, outputs* model_outputs,
                                                           parameters* model_parameters)
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

        entry.signal_registry.insert("wm", _model_outputs->wm);
        // entry.signal_registry.insert("wt", _model_outputs->wt);
        entry.signal_registry.insert("ifd", _model_outputs->ifd);
        entry.signal_registry.insert("it", _model_outputs->it);
        entry.signal_registry.insert("id", _model_outputs->id);
        entry.signal_registry.insert("iq", _model_outputs->iq);
        entry.signal_registry.insert("vt", _model_outputs->vt);
        entry.signal_registry.insert("vd", _model_outputs->vd);
        entry.signal_registry.insert("vq", _model_outputs->vq);
        entry.signal_registry.insert("vg", _model_outputs->vg);
        entry.signal_registry.insert("load_angle_deg", _model_outputs->load_angle_deg);
        // entry.signal_registry.insert("voltage_alpha", _model_outputs->voltage_alpha);
        // entry.signal_registry.insert("voltage_beta", _model_outputs->voltage_beta);
        // entry.signal_registry.insert("current_alpha", _model_outputs->current_alpha);
        // entry.signal_registry.insert("current_beta", _model_outputs->current_beta);

        registry.insert("plant", std::move(entry));
    }

    inputs* get_inputs() { return _model_inputs; }
    outputs* get_outputs() { return _model_outputs; }
    parameters* get_parameters() { return _model_parameters; }

private:
    inputs* _model_inputs{};
    outputs* _model_outputs{};
    parameters* _model_parameters{};
};

}  // namespace yeet::application::mpc