#include "mpc_integration/mpc_integration_runner.hpp"

#include "yeet_control/constants.hpp"

namespace yeet::application::mpc {

void mpc_integration_runner::initialize() {
    _unconstrained_excitation_mpc.set_sampling_time(sampling_time);

    _unconstrained_excitation_mpc.set_terminal_voltage_reference(_parameters.mpc_terminal_voltage_reference);
    _unconstrained_excitation_mpc.set_feedforward_output(_parameters.mpc_feedforward_output);
    _unconstrained_excitation_mpc.set_speed_deviation_penalization(_parameters.mpc_speed_deviation_penalization);

    set_parameters();
    set_parameter_callback();

    _unconstrained_excitation_mpc.update(get_initial_operating_point(), true);

    _outputs.efd = _unconstrained_excitation_mpc.get_output();
}

void mpc_integration_runner::run() {
    _unconstrained_excitation_mpc.update(get_operating_point(), false);

    _outputs.efd = _unconstrained_excitation_mpc.get_output();
    _outputs.nwsr = _unconstrained_excitation_mpc.get_nwsr();
    _outputs.model_update_counter = _unconstrained_excitation_mpc.get_model_update_counter();
}

void mpc_integration_runner::stop() {}

void mpc_integration_runner::set_parameters() {
    machine_parameters machine_parameters{};
    machine_parameters.l_s = _parameters.l_s;
    machine_parameters.l_ad = _parameters.l_ad;
    machine_parameters.l_aq = _parameters.l_aq;
    machine_parameters.l_fd = _parameters.l_fd;
    machine_parameters.r_s = _parameters.r_s;
    machine_parameters.r_fd = _parameters.r_fd;

    system_parameters system_parameters{};
    system_parameters.w_base = _parameters.w_base;
    system_parameters.h = _parameters.h;
    system_parameters.kd = _parameters.kd;
    system_parameters.l_g = _parameters.l_g;
    system_parameters.r_g = _parameters.r_g;

    _unconstrained_excitation_mpc.set_parameters(machine_parameters, system_parameters);
    _unconstrained_excitation_mpc.set_lambda(_parameters.mpc_lambda);
    _unconstrained_excitation_mpc.set_umax(_parameters.mpc_umax);
    _unconstrained_excitation_mpc.set_umin(_parameters.mpc_umin);
}

void mpc_integration_runner::set_parameter_callback() {
    _parameters.mpc_lambda.register_callback(
        [&]() { _unconstrained_excitation_mpc.set_lambda(_parameters.mpc_lambda); });

    _parameters.mpc_terminal_voltage_reference.register_callback([&]() {
        _unconstrained_excitation_mpc.set_terminal_voltage_reference(_parameters.mpc_terminal_voltage_reference);
    });

    _parameters.mpc_speed_deviation_penalization.register_callback([&]() {
        _unconstrained_excitation_mpc.set_speed_deviation_penalization(_parameters.mpc_speed_deviation_penalization);
    });

    _parameters.mpc_umax.register_callback([&]() { _unconstrained_excitation_mpc.set_umax(_parameters.mpc_umax); });
    _parameters.mpc_umin.register_callback([&]() { _unconstrained_excitation_mpc.set_umin(_parameters.mpc_umin); });

    _parameters.l_s.register_callback([&]() { set_parameters(); });
    _parameters.l_ad.register_callback([&]() { set_parameters(); });
    _parameters.l_aq.register_callback([&]() { set_parameters(); });
    _parameters.l_fd.register_callback([&]() { set_parameters(); });
    _parameters.r_s.register_callback([&]() { set_parameters(); });
    _parameters.r_fd.register_callback([&]() { set_parameters(); });
    _parameters.w_base.register_callback([&]() { set_parameters(); });
    _parameters.h.register_callback([&]() { set_parameters(); });
    _parameters.kd.register_callback([&]() { set_parameters(); });
    _parameters.l_g.register_callback([&]() { set_parameters(); });
    _parameters.r_g.register_callback([&]() { set_parameters(); });
}

operating_point_type mpc_integration_runner::get_operating_point() {
    operating_point_type operating_point{};

    operating_point.w0_pu = _inputs.wm;
    operating_point.ifd_0 = _inputs.ifd;
    operating_point.it_0 = _inputs.it;
    operating_point.id_0 = _inputs.id;
    operating_point.iq_0 = _inputs.iq;
    operating_point.vt_0 = _inputs.vt;
    operating_point.vd_0 = _inputs.vd;
    operating_point.vq_0 = _inputs.vq;
    operating_point.vg_0 = 1.0f;
    operating_point.delta_0 = _inputs.load_angle_deg * yeet::control::pi / 180.0f;

    return operating_point;
}

operating_point_type mpc_integration_runner::get_initial_operating_point() {
    operating_point_type operating_point;

    //            u_0: 1.508580000000000
    //            u_k: 1.508591712556988
    //           vt_k: 1.000000262106270
    //           ud_k: 0.562052744981866
    //           uq_k: 0.827101708419804
    //           it_k: 0.723065397435660
    //           id_k: 0.453199844445547
    //           iq_k: 0.563412344525146
    //          ifd_k: 1.508591712131481
    //           wm_k: 1.000000000000004
    //    delta_deg_k: 34.197867737913157
    //           pt_k: 0.720721529282184
    //           qt_k: 0.058174910807558

    operating_point.w0_pu = 1.000000000000004f;
    operating_point.ifd_0 = 1.508591712131481f;
    operating_point.it_0 = 0.723065397435660f;
    operating_point.id_0 = 0.453199844445547f;
    operating_point.iq_0 = 0.563412344525146f;
    operating_point.vt_0 = 1.000000262106270f;
    operating_point.vd_0 = 0.562052744981866f;
    operating_point.vq_0 = 0.827101708419804f;
    operating_point.vg_0 = 1.0f;
    operating_point.delta_0 = (yeet::control::pi / 180.0f) * 34.197867737913157f;

    return operating_point;
}

}  // namespace yeet::application::mpc
