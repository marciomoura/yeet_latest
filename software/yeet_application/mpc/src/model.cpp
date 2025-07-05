#include "mpc/model.h"

#include <cassert>

namespace yeet::application::mpc {

void synchronous_machine_tied_to_infinite_bus_model::set_machine_parameters(
    const machine_parameters& machine_parameters) {
    _machine_parameters = machine_parameters;

    update_internal_parameters();
}

void synchronous_machine_tied_to_infinite_bus_model::set_system_parameters(const system_parameters& system_parameters) {
    _system_parameters = system_parameters;

    update_internal_parameters();
}

Eigen::Matrix<double, 7, 3> const& synchronous_machine_tied_to_infinite_bus_model::get_output_matrix() const {
    return _output_matrix;
}

Eigen::Matrix<double, 3, 1> const& synchronous_machine_tied_to_infinite_bus_model::get_x0() const { return _x0; }

Eigen::Matrix<double, 3, 3> const& synchronous_machine_tied_to_infinite_bus_model::get_state_matrix() const {
    return _state_matrix;
}

Eigen::Matrix<double, 3, 1> const& synchronous_machine_tied_to_infinite_bus_model::get_input_matrix() const {
    return _input_matrix;
}

internal_machine_parameters const& synchronous_machine_tied_to_infinite_bus_model::get_internal_machine_parameters()
    const {
    return _internal_machine_parameters;
}

Eigen::Matrix<double, 2, 3> const& synchronous_machine_tied_to_infinite_bus_model::get_reduced_output_matrix() const {
    return _output_reduced_matrix;
}

void synchronous_machine_tied_to_infinite_bus_model::update_internal_parameters() {
    // ld = ls + lad;
    _internal_machine_parameters.l_d = _machine_parameters.l_s + _machine_parameters.l_ad;

    // lq = ls + m.Xaq;
    _internal_machine_parameters.l_q = _machine_parameters.l_s + _machine_parameters.l_aq;

    // lffd =  lad + lfd;
    _internal_machine_parameters.l_ffd = _machine_parameters.l_ad + _machine_parameters.l_fd;

    assert(_machine_parameters.l_fd + _machine_parameters.l_ad != 0);

    // ladd = (lfd * lad) / (lfd + lad);
    _internal_machine_parameters.l_add =
        (_machine_parameters.l_fd * _machine_parameters.l_ad) / (_machine_parameters.l_fd + _machine_parameters.l_ad);

    // ldd = ladd + ls;
    _internal_machine_parameters.l_dd = _internal_machine_parameters.l_add + _machine_parameters.l_s;

    // rt = rs + rg;
    _internal_machine_parameters.r_t = _machine_parameters.r_s + _system_parameters.r_g;

    // ltd = ldd + lg;
    _internal_machine_parameters.l_td = _internal_machine_parameters.l_dd + _system_parameters.l_g;

    // ltq = lq + lg;
    _internal_machine_parameters.l_tq = _internal_machine_parameters.l_q + _system_parameters.l_g;

    // d = rt^2 + ltq * ltd;
    _internal_machine_parameters.d = _internal_machine_parameters.r_t * _internal_machine_parameters.r_t +
                                     _internal_machine_parameters.l_tq * _internal_machine_parameters.l_td;
}

void synchronous_machine_tied_to_infinite_bus_model::update_model(operating_point_type const& operating_point) {
    // psiq_0 = -iq_0*lq;
    double psiq_0 = -operating_point.iq_0 * _internal_machine_parameters.l_q;

    // psid_0 = -id_0*ld + ifd_0*lad;
    double psid_0 =
        -operating_point.id_0 * _internal_machine_parameters.l_d + operating_point.ifd_0 * _machine_parameters.l_ad;

    // psifd_0 = ifd_0*lffd - id_0*lad;
    double psifd_0 =
        operating_point.ifd_0 * _internal_machine_parameters.l_ffd - operating_point.id_0 * _machine_parameters.l_ad;

    // kid_delta = (ltq*vg_0*sin(delta_0)-rt*vg_0*cos(delta_0))/d;
    double kid_delta = (_internal_machine_parameters.l_tq * operating_point.vg_0 * sin(operating_point.delta_0) -
                        _internal_machine_parameters.r_t * operating_point.vg_0 * cos(operating_point.delta_0)) /
                       _internal_machine_parameters.d;

    // kid_psifd = ladd*ltq/(d*lfd);
    double kid_psifd = _internal_machine_parameters.l_add * _internal_machine_parameters.l_tq /
                       (_internal_machine_parameters.d * _machine_parameters.l_fd);

    // kiq_delta = (ltd*vg_0*cos(delta_0)+rt*vg_0*sin(delta_0))/d;
    double kiq_delta = (_internal_machine_parameters.l_td * operating_point.vg_0 * cos(operating_point.delta_0) +
                        _internal_machine_parameters.r_t * operating_point.vg_0 * sin(operating_point.delta_0)) /
                       _internal_machine_parameters.d;

    // kiq_psifd = ladd*rt/(d*lfd);
    double kiq_psifd = _internal_machine_parameters.l_add * _internal_machine_parameters.r_t /
                       (_internal_machine_parameters.d * _machine_parameters.l_fd);

    // kifd_delta = (ladd*ltq*vg_0*sin(delta_0)-ladd*rt*vg_0*cos(delta_0))/(d*lfd);
    double kifd_delta = (_internal_machine_parameters.l_add * _internal_machine_parameters.l_tq * operating_point.vg_0 *
                             sin(operating_point.delta_0) -
                         _internal_machine_parameters.l_add * _internal_machine_parameters.r_t * operating_point.vg_0 *
                             cos(operating_point.delta_0)) /
                        (_internal_machine_parameters.d * _machine_parameters.l_fd);

    // kifd_psifd = (1/lffd + lad^2 * ltq/(d*lffd^2));
    double kifd_psifd = (1.0f / _internal_machine_parameters.l_ffd +
                         _machine_parameters.l_ad * _machine_parameters.l_ad * _internal_machine_parameters.l_tq /
                             (_internal_machine_parameters.d * _internal_machine_parameters.l_ffd *
                              _internal_machine_parameters.l_ffd));

    // kh = 0.5/s.H;
    double kh = 0.5f / _system_parameters.h;

    // kw_w = -s.Kd*kh;
    double kw_w = -_system_parameters.kd * kh;

    // kw_delta =kh*(-psid_0*kiq_delta + psiq_0*kid_delta - id_0*kiq_delta*lq + iq_0*kid_delta*ld -
    // iq_0*kifd_delta*lad);
    double kw_delta = kh * (-psid_0 * kiq_delta + psiq_0 * kid_delta -
                            operating_point.id_0 * kiq_delta * _internal_machine_parameters.l_q +
                            operating_point.iq_0 * kid_delta * _internal_machine_parameters.l_d -
                            operating_point.iq_0 * kifd_delta * _machine_parameters.l_ad);

    // kw_psifd =kh*(-psid_0*kiq_psifd + psiq_0*kid_psifd - id_0*kiq_psifd*lq + iq_0*kid_psifd*ld -
    // iq_0*kifd_psifd*lad);
    double kw_psifd = kh * (-psid_0 * kiq_psifd + psiq_0 * kid_psifd -
                            operating_point.id_0 * kiq_psifd * _internal_machine_parameters.l_q +
                            operating_point.iq_0 * kid_psifd * _internal_machine_parameters.l_d -
                            operating_point.iq_0 * kifd_psifd * _machine_parameters.l_ad);

    // kdelta_w = w_base;
    double kdelta_w = _system_parameters.w_base;
    static constexpr double kdelta_delta = 0.0f;
    static constexpr double kdelta_psifd = 0.0f;

    // kpsifd_w = 0;
    static constexpr double kpsifd_w = 0.0f;

    // kpsifd_delta = -w_0*kifd_delta*rfd;
    double kpsifd_delta = -_system_parameters.w_base * kifd_delta * _machine_parameters.r_fd;

    // kpsifd_psifd = -w_0*kifd_psifd*rfd;
    double kpsifd_psifd = -_system_parameters.w_base * kifd_psifd * _machine_parameters.r_fd;

    // kpsifd_efd = w_0*rfd/lad;
    double kpsifd_efd = _system_parameters.w_base * _machine_parameters.r_fd / _machine_parameters.l_ad;

    // States are \omega, \delta and \psi_fd
    _state_matrix << kw_w, kw_delta, kw_psifd,  // w
        kdelta_w, kdelta_delta, kdelta_psifd,   // delta
        kpsifd_w, kpsifd_delta, kpsifd_psifd;   // psi_fd

    _input_matrix << 0.0f,  //
        0.0f,               //
        kpsifd_efd;         // efd

    // x0 = [w;delta_0;psifd_0];
    _x0 << operating_point.w0_pu, operating_point.delta_0, psifd_0;

    //  kvt_delta = (-kid_delta*ld*w*vq_0 - kid_delta*rs*vd_0 + kifd_delta*lad*w*vq_0 + kiq_delta*lq*w*vd_0 -
    //  kiq_delta*rs*vq_0)/vt_0;
    double kvt_delta = (-kid_delta * _internal_machine_parameters.l_d * operating_point.w0_pu * operating_point.vq_0 -
                        kid_delta * _machine_parameters.r_s * operating_point.vd_0 +
                        kifd_delta * _machine_parameters.l_ad * operating_point.w0_pu * operating_point.vq_0 +
                        kiq_delta * _internal_machine_parameters.l_q * operating_point.w0_pu * operating_point.vd_0 -
                        kiq_delta * _machine_parameters.r_s * operating_point.vq_0) /
                       operating_point.vt_0;

    //  kvt_psifd = (-kid_psifd*ld*w*vq_0 - kid_psifd*rs*vd_0 + kifd_psifd*lad*w*vq_0 + kiq_psifd*lq*w*vd_0 -
    //  kiq_psifd*rs*vq_0)/vt_0;
    double kvt_psifd = (-kid_psifd * _internal_machine_parameters.l_d * operating_point.w0_pu * operating_point.vq_0 -
                        kid_psifd * _machine_parameters.r_s * operating_point.vd_0 +
                        kifd_psifd * _machine_parameters.l_ad * operating_point.w0_pu * operating_point.vq_0 +
                        kiq_psifd * _internal_machine_parameters.l_q * operating_point.w0_pu * operating_point.vd_0 -
                        kiq_psifd * _machine_parameters.r_s * operating_point.vq_0) /
                       operating_point.vt_0;

    //  kqt_delta=(-id_0*kid_delta*ld*w + id_0*kifd_delta*lad*w - id_0*kiq_delta*rs + iq_0*kid_delta*rs -
    //  iq_0*kiq_delta*lq*w + kid_delta*vq_0 - kiq_delta*vd_0);
    double kqt_delta = (-operating_point.id_0 * kid_delta * _internal_machine_parameters.l_d * operating_point.w0_pu +
                        operating_point.id_0 * kifd_delta * _machine_parameters.l_ad * operating_point.w0_pu -
                        operating_point.id_0 * kiq_delta * _machine_parameters.r_s +
                        operating_point.iq_0 * kid_delta * _machine_parameters.r_s -
                        operating_point.iq_0 * kiq_delta * _internal_machine_parameters.l_q * operating_point.w0_pu +
                        kid_delta * operating_point.vq_0 - kiq_delta * operating_point.vd_0);

    //  kqt_psifd=(-id_0*kid_psifd*ld*w + id_0*kifd_psifd*lad*w - id_0*kiq_psifd*rs + iq_0*kid_psifd*rs -
    //  iq_0*kiq_psifd*lq*w + kid_psifd*vq_0 - kiq_psifd*vd_0);
    double kqt_psifd = (-operating_point.id_0 * kid_psifd * _internal_machine_parameters.l_d * operating_point.w0_pu +
                        operating_point.id_0 * kifd_psifd * _machine_parameters.l_ad * operating_point.w0_pu -
                        operating_point.id_0 * kiq_psifd * _machine_parameters.r_s +
                        operating_point.iq_0 * kid_psifd * _machine_parameters.r_s -
                        operating_point.iq_0 * kiq_psifd * _internal_machine_parameters.l_q * operating_point.w0_pu +
                        kid_psifd * operating_point.vq_0 - kiq_psifd * operating_point.vd_0);

    //  kpt_delta=(-id_0*kid_delta*rs + id_0*kiq_delta*lq*w - iq_0*kid_delta*ld*w + iq_0*kifd_delta*lad*w -
    //  iq_0*kiq_delta*rs + kid_delta*vd_0 + kiq_delta*vq_0);
    double kpt_delta = (-operating_point.id_0 * kid_delta * _machine_parameters.r_s +
                        operating_point.id_0 * kiq_delta * _internal_machine_parameters.l_q * operating_point.w0_pu -
                        operating_point.iq_0 * kid_delta * _internal_machine_parameters.l_d * operating_point.w0_pu +
                        operating_point.iq_0 * kifd_delta * _machine_parameters.l_ad * operating_point.w0_pu -
                        operating_point.iq_0 * kiq_delta * _machine_parameters.r_s + kid_delta * operating_point.vd_0 +
                        kiq_delta * operating_point.vq_0);

    //  kpt_psifd=(-id_0*kid_psifd*rs + id_0*kiq_psifd*lq*w - iq_0*kid_psifd*ld*w + iq_0*kifd_psifd*lad*w -
    //  iq_0*kiq_psifd*rs + kid_psifd*vd_0 + kiq_psifd*vq_0);
    double kpt_psifd = (-operating_point.id_0 * kid_psifd * _machine_parameters.r_s +
                        operating_point.id_0 * kiq_psifd * _internal_machine_parameters.l_q * operating_point.w0_pu -
                        operating_point.iq_0 * kid_psifd * _internal_machine_parameters.l_d * operating_point.w0_pu +
                        operating_point.iq_0 * kifd_psifd * _machine_parameters.l_ad * operating_point.w0_pu -
                        operating_point.iq_0 * kiq_psifd * _machine_parameters.r_s + kid_psifd * operating_point.vd_0 +
                        kiq_psifd * operating_point.vq_0);

    _output_matrix << 1.0f, 0.0f, 0.0f,  // w
        0.0f, 1.0f, 0.0f,                // delta
        0.0f, 0.0f, 1.0f,                // psi_fd
        0.0f, kvt_delta, kvt_psifd,      // v_t
        0.0f, kqt_delta, kqt_psifd,      // q_t
        0.0f, kpt_delta, kpt_psifd,      // p_t
        0.0f, kifd_delta, kifd_psifd;    // i_fd

    _output_reduced_matrix << 0.0f, kvt_delta, kvt_psifd,  // v_t
        1.0f, 0.0f, 0.0;                                   // w
}

}  // namespace yeet::application::mpc