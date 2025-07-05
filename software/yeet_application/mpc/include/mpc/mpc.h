#pragma once

#include <Eigen/Dense>

#include "incremental_format.h"
#include "model.h"
#include "mpc/quadratic_program.h"
#include "qp_solver.h"
#include "quadratic_program.h"

namespace yeet::application::mpc {

template <std::size_t Np_Size>
class excitation_mpc {
public:
    static constexpr std::size_t NumConstraints = 0;
    static constexpr std::size_t StatesN = 3;
    static constexpr std::size_t InputsN = 1;
    static constexpr std::size_t OutputsN = 2;
    static constexpr std::size_t Np = Np_Size;
    static constexpr std::size_t IncrementalStatesN = StatesN + OutputsN;
    static constexpr std::size_t IncrementalInputsN = InputsN;
    static constexpr std::size_t IncrementalOutputsN = OutputsN;
    static constexpr std::size_t SoftConstraintsN = 2;  // vt_max, vt_min

    excitation_mpc() { calculate_q_tilda(); }

    void set_parameters(const machine_parameters& machine_parameters, const system_parameters& system_parameters) {
        _model.set_machine_parameters(machine_parameters);

        _model.set_system_parameters(system_parameters);
    }

    void set_sampling_time(double sampling_time) { _sampling_time = sampling_time; }

    void set_feedforward_output(double u_0) { _u_k = u_0; }

    void set_lambda(double lambda) { _lambda = lambda; }

    void set_umax(double u_max) { _u_max = u_max; }

    void set_umin(double u_min) { _u_min = u_min; }

    double get_lambda() const { return _lambda; }

    void set_speed_deviation_penalization(double speed_deviation_penalization) {
        _speed_deviation_penalization = speed_deviation_penalization;

        calculate_q_tilda();
    }

    void update(operating_point_type const& operating_point, bool first_iteration) {
        if (first_iteration) {
            _operating_point_k = operating_point;
        } else {
            _operating_point_k = operating_point;
        }

        // n.x0_k = [w_0; delta_0; psifd_0];
        _model.update_model(operating_point);

        if (first_iteration) {
            _x0_k = _x0_km1 = _model.get_x0();
        } else {
            _x0_km1 = _x0_k;

            _x0_k = _model.get_x0();
        }

        // Compute the change in x0 from step [k-1] to step [k]
        // This term is multiplied by the Gamma matrix.
        // After the control execution, x0_km1 is updated with x0_k.
        // n.dx0_k = n.x0_k - n.x0_km1;
        _dx0_k = _x0_k - _x0_km1;

        // Algorithm to decide when to update the model.
        // requires_model_update(operating_point, _operating_point_linearization)
        bool initialize_solver = false;
        if (first_iteration || _model_update_counter == refresh_model_after_n_iterations) {
            _operating_point_linearization = operating_point;
            _model_update_counter = 0;

            calculate_gamma_ypsilon();

            calculate_qp_quadratic_term();

            initialize_solver = true;
        } else {
            _model_update_counter++;
        }

        // These are the system matrices that are dependent on the operating point.
        // Therefore, these can't be precomputed.

        // [n.dx0_k; n.Y], n.Yref,
        // Y in this case is [vt; wm]
        Eigen::Matrix<double, OutputsN, 1> const y0 = {operating_point.vt_0, operating_point.w0_pu};

        Eigen::Matrix<double, StatesN + OutputsN, 1> const x0 = incremental_format_x0<StatesN, OutputsN>(_dx0_k, y0);

        calculate_linear_term<IncrementalStatesN, IncrementalInputsN, IncrementalOutputsN, Np>(
            _ypsilon, _precomputed_q_tilda, _y_ref, _gamma, x0, _theta);

        // update soft constraints
        // {operating_point.vt_0, operating_point.vt_0}, {1.05f, 0.995f},
        // {soft_constraint_type::Max, soft_constraint_type::Min}

        _soft_constraints[0] = {operating_point.vt_0, 1.03f, soft_constraint_type::Max};
        _soft_constraints[1] = {operating_point.vt_0, 0.99f, soft_constraint_type::Min};

        // Every time is updated
        _augmented_b = create_min_max_soft_constraints_b<IncrementalStatesN, IncrementalInputsN, SoftConstraintsN, Np>(
            _u_k, _u_max, _u_min, _soft_constraints);

        _augmented_theta = augment_linear_term_with_slack_variables<IncrementalStatesN, IncrementalInputsN,
                                                                    IncrementalOutputsN, Np, SoftConstraintsN>(_theta);

        if (initialize_solver) {
            // using the horizon penalization
            auto horizon_penalties = create_horizon_weighted_penalties<SoftConstraintsN, Np>(100.0);
            _augmented_hessian =
                augment_quadratic_term_with_horizon_weighted_slack_variables<IncrementalStatesN, IncrementalInputsN,
                                                                             IncrementalOutputsN, Np, SoftConstraintsN>(
                    _hessian, horizon_penalties);

            // std::array<Eigen::Matrix<double, Np, 1>, SoftConstraintsN> create_horizon_weighted_penalties(1000.0f);
            //     double base_penalty);

            // std::array<double, SoftConstraintsN> soft_constraints_penalization_r = {100.0f, 100.0f};
            // constrained-control
            // _augmented_hessian =
            //     augment_quadratic_term_with_slack_variables<IncrementalStatesN, IncrementalInputsN,
            //                                                      IncrementalOutputsN, Np, SoftConstraintsN>(
            //         _hessian, soft_constraints_penalization_r);

            _augmented_a = create_min_max_soft_constraints_a<IncrementalStatesN, IncrementalInputsN, SoftConstraintsN,
                                                             Np, Eigen::RowMajor>(a_incremental, b_incremental,
                                                                                  _soft_constraints, _powers_of_a);

            // Initializes the solver with the matrices of the updated model.
            _solver.initialize(_augmented_hessian, _augmented_theta, _augmented_a, _augmented_b, _incremental_u_k);
        } else {
            // Hotstarts the solver with the new theta and b_sc.
            _solver.hotstart(_augmented_theta, _augmented_b, _incremental_u_k);
        }

        // Integrate the control action.
        // An accumulator class can be created for this component.
        _u_k += _incremental_u_k;
    }

    double get_output() const { return _u_k; }

    std::int32_t get_nwsr() const { return _solver.get_nwsr(); }

    void set_terminal_voltage_reference(double vt) {
        for (std::size_t i = 0; i < Np; ++i) {
            _y_ref.block(i * OutputsN, 0, OutputsN, 1) << vt, 1.0f;
        }
    }

    double get_sampling_time() const { return _sampling_time; }

    std::uint32_t get_model_update_counter() const { return _model_update_counter; }

private:
    // Another idea is to re-trigger based on the operating point.
    static bool requires_model_update(operating_point_type const& operating_point,
                                      operating_point_type const& operating_point_at_linearization) {
        // Compute the model and store the operating point that the model has been linearized.
        // Evaluate in new operating points if the model needs to be re-computed.
        // Compare against the operating point of the new linearization.
        bool w0_above_threshold = std::abs(operating_point_at_linearization.w0_pu - operating_point.w0_pu) > 0.01f;
        bool vt_above_threshold = std::abs(operating_point_at_linearization.vt_0 - operating_point.vt_0) > 0.1f;
        bool it_above_threshold = std::abs(operating_point_at_linearization.it_0 - operating_point.it_0) > 0.1f;

        return w0_above_threshold || vt_above_threshold || it_above_threshold;
    }

    void calculate_q_tilda() {
        Eigen::Matrix<double, IncrementalOutputsN, 1> penalization = {1.0f, _speed_deviation_penalization};
        _precomputed_q_tilda = precompute_q_tilda<IncrementalOutputsN, Np>(penalization);
    }

    void calculate_gamma_ypsilon() {
        Eigen::Matrix<double, StatesN, StatesN> discrete_a;
        Eigen::Matrix<double, StatesN, InputsN> discrete_b;
        state_space_forward_euler_discretization<StatesN, InputsN>(_model.get_state_matrix(), _model.get_input_matrix(),
                                                                   _sampling_time, discrete_a, discrete_b);

        incremental_format<StatesN, InputsN, OutputsN>(discrete_a, discrete_b, _model.get_reduced_output_matrix(),
                                                       a_incremental, b_incremental, c_incremental);

        _powers_of_a = compute_powers_of_a<IncrementalStatesN, Np>(a_incremental);

        // update soft constraints
        _soft_constraints[0].c = c_incremental.row(0);
        _soft_constraints[1].c = c_incremental.row(0);

        calculate_gamma_and_ypsilon<IncrementalStatesN, IncrementalInputsN, IncrementalOutputsN, Np>(
            a_incremental, b_incremental, c_incremental, _powers_of_a, _gamma, _ypsilon);
    }

    void calculate_qp_quadratic_term() {
        calculate_quadratic_term<IncrementalStatesN, IncrementalInputsN, IncrementalOutputsN, Np>(
            _ypsilon, _precomputed_q_tilda, _lambda, _hessian);
    }

    double _sampling_time = 50e-3;
    double _lambda = 1e-4;
    double _speed_deviation_penalization = 1000.0f;
    double _u_max = 15.0f;
    double _u_min = -10.0f;
    double _u_k = {};
    double _incremental_u_k = {};

    std::uint32_t const refresh_model_after_n_iterations = 20;
    std::uint32_t _model_update_counter = 0;

    operating_point_type _operating_point_linearization{};
    operating_point_type _operating_point_k{};

    Eigen::DiagonalMatrix<double, Np * IncrementalOutputsN> _precomputed_q_tilda;

    Eigen::Matrix<double, StatesN, 1> _dx0_k = Eigen::MatrixXd::Zero(StatesN, 1);

    Eigen::Matrix<double, StatesN, 1> _x0_k = Eigen::MatrixXd::Zero(StatesN, 1);

    Eigen::Matrix<double, StatesN, 1> _x0_km1 = Eigen::MatrixXd::Zero(StatesN, 1);

    Eigen::Matrix<double, Np * OutputsN, 1> _y_ref = Eigen::MatrixXd::Ones(Np * OutputsN, 1);

    Eigen::Matrix<double, StatesN + OutputsN, StatesN + OutputsN> a_incremental;
    Eigen::Matrix<double, StatesN + OutputsN, InputsN> b_incremental;
    Eigen::Matrix<double, OutputsN, StatesN + OutputsN> c_incremental;

    std::array<Eigen::Matrix<double, IncrementalStatesN, IncrementalStatesN>, Np + 1> _powers_of_a{};

    std::array<SoftConstraint<IncrementalStatesN>, SoftConstraintsN> _soft_constraints;

    // Ax <= b
    Eigen::Matrix<double, 2 * Np*(InputsN + NumConstraints), Np * InputsN, Eigen::RowMajor> _constraints_a;
    Eigen::Matrix<double, 2 * Np*(InputsN + NumConstraints), InputsN> _constraints_b;

    Eigen::Matrix<double, Np * IncrementalOutputsN, IncrementalStatesN> _gamma =
        Eigen::MatrixXd::Zero(Np * IncrementalOutputsN, IncrementalStatesN);

    Eigen::Matrix<double, Np * IncrementalOutputsN, Np * IncrementalInputsN> _ypsilon =
        Eigen::MatrixXd::Zero(Np * IncrementalOutputsN, Np* IncrementalInputsN);

    Eigen::Matrix<double, Np * IncrementalInputsN, Np * IncrementalInputsN, Eigen::RowMajor> _hessian =
        Eigen::MatrixXd::Zero(Np * IncrementalInputsN, Np* IncrementalInputsN);

    Eigen::Matrix<double, Np * IncrementalInputsN, 1> _theta = Eigen::MatrixXd::Zero(Np * IncrementalInputsN, 1);

    // augmented for constrained control
    Eigen::Matrix<double, Np*(InputsN + SoftConstraintsN), Np*(InputsN + SoftConstraintsN)> _augmented_hessian;
    Eigen::Matrix<double, Np * InputsN + SoftConstraintsN * Np, 1> _augmented_theta;
    Eigen::Matrix<double, Np * InputsN * (2 + SoftConstraintsN), Np * InputsN * (1 + SoftConstraintsN), Eigen::RowMajor>
        _augmented_a;
    Eigen::Matrix<double, Np * InputsN * (2 + SoftConstraintsN), InputsN> _augmented_b;

    synchronous_machine_tied_to_infinite_bus_model _model;

    QPSolver<StatesN, InputsN, Np, SoftConstraintsN> _solver;
};

}  // namespace yeet::application::mpc
