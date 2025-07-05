#pragma once

#include <Eigen/Dense>

namespace yeet::application::mpc {

struct machine_parameters {
    double l_s;   // Leakage reactance
    double l_ad;  //
    double l_aq;  //
    double l_fd;  // Field winding reactance
    double r_s;   // Leakage resistance
    double r_fd;  // Field winding resistance
};

struct internal_machine_parameters {
    double l_d;
    double l_q;
    double l_ffd;
    double l_add;
    double l_dd;
    double l_tq;
    double l_td;
    double r_t;
    double d;
};

struct system_parameters {
    double w_base;  // Base angular velocity
    double h;       // Inertia constant
    double kd;      // Damping constant
    double l_g;     // Grid impedance
    double r_g;     // Grid resistance
};

struct operating_point_type {
    double w0_pu;    // Angular velocity (per unit)
    double ifd_0;    // Field current (per unit)
    double it_0;     // Terminal current (per unit)
    double id_0;     // Terminal d-axis current (per unit)
    double iq_0;     // Terminal q-axis current (per unit)
    double vt_0;     // Terminal voltage magnitude (per unit)
    double vd_0;     // Terminal d-axis voltage (per unit)
    double vq_0;     // Terminal q-axis voltage (per unit)
    double vg_0;     // Estimated grid voltage (per unit)
    double delta_0;  // Rotor/load angle (radians)
};

class synchronous_machine_tied_to_infinite_bus_model {
public:
    void set_machine_parameters(const machine_parameters& machine_parameters);

    void set_system_parameters(const system_parameters& system_parameters);

    void update_model(operating_point_type const& operating_point);

    Eigen::Matrix<double, 7, 3> const& get_output_matrix() const;

    Eigen::Matrix<double, 2, 3> const& get_reduced_output_matrix() const;

    Eigen::Matrix<double, 3, 1> const& get_x0() const;

    Eigen::Matrix<double, 3, 3> const& get_state_matrix() const;

    Eigen::Matrix<double, 3, 1> const& get_input_matrix() const;

    internal_machine_parameters const& get_internal_machine_parameters() const;

private:
    void update_internal_parameters();

    machine_parameters _machine_parameters;

    internal_machine_parameters _internal_machine_parameters;

    system_parameters _system_parameters;

    Eigen::Matrix<double, 3, 1> _x0{};  // Initial condition

    Eigen::Matrix<double, 3, 3> _state_matrix{};  // State matrix

    Eigen::Matrix<double, 3, 1> _input_matrix{};  // Input matrix

    Eigen::Matrix<double, 7, 3> _output_matrix{};  // Output matrix

    Eigen::Matrix<double, 2, 3> _output_reduced_matrix{};  // Output matrix
};

}  // namespace yeet::application::mpc