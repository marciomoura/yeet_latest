#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <memory>
#include <qpOASES.hpp>

namespace yeet::application::mpc {

template <std::size_t StatesN, std::size_t InputsN, std::size_t Np, std::size_t SoftConstraintsN>
class QPSolver {
public:
    QPSolver() {
        // Set options
        options.setToMPC();
        options.printLevel = qpOASES::PL_NONE;
        options.enableRegularisation = qpOASES::BT_FALSE;
        _qp->setOptions(options);
    }

    void initialize(
        const Eigen::Matrix<double, Np*(InputsN + SoftConstraintsN), Np*(InputsN + SoftConstraintsN)>& hessian,
        const Eigen::Matrix<double, Np * InputsN + SoftConstraintsN * Np, 1>& theta,
        const Eigen::Matrix<double, Np * InputsN * (2 + SoftConstraintsN), Np * InputsN * (1 + SoftConstraintsN),
                            Eigen::RowMajor>& a_sc,
        const Eigen::Matrix<double, Np * InputsN * (2 + SoftConstraintsN), InputsN>& b_sc, double& delta_u) {
        int nwsr = max_nwsr;  // Maximum number of working set recalculations

        const qpOASES::returnValue status =
            _qp->init(hessian.data(), theta.data(), a_sc.data(), nullptr, nullptr, nullptr, b_sc.data(), nwsr);

        delta_u = update_solution(status);
        _actual_nwsr = nwsr;
    }

    void hotstart(const Eigen::Matrix<double, Np * InputsN + SoftConstraintsN * Np, 1>& new_theta,
                  const Eigen::Matrix<double, Np * InputsN * (2 + SoftConstraintsN), InputsN>& new_b_sc,
                  double& delta_u) {
        std::int32_t nwsr = max_nwsr;

        const qpOASES::returnValue status = _qp->hotstart(new_theta.data(),  // new gradient
                                                          nullptr,           // new lower bounds (none)
                                                          nullptr,           // new upper bounds (none)
                                                          nullptr,           // new lower constraint bounds (none)
                                                          new_b_sc.data(),   // new upper constraint bounds
                                                          nwsr);  // working set recalculations (input/output)
        _actual_nwsr = nwsr;

        delta_u = update_solution(status);
    }

    std::int32_t get_nwsr() const { return _actual_nwsr; }

private:
    double update_solution(qpOASES::returnValue status) {
        if (status == qpOASES::SUCCESSFUL_RETURN) {
            _qp->getPrimalSolution(_x_opt.data());
            return _x_opt[0];
        } else {
            return 0.0;
        }
    }

    // Problem dimensions
    static constexpr int max_nwsr = 10;                               // Maximum number of working set recalculations
    static constexpr int nV = (1 + SoftConstraintsN) * Np;            // Number of variables
    static constexpr int nC = Np * InputsN * (2 + SoftConstraintsN);  // Number of constraints
    std::int32_t _actual_nwsr = 0;

    qpOASES::Options options = {};
    std::array<qpOASES::real_t, nV> _x_opt{};
    std::unique_ptr<qpOASES::QProblem> _qp = std::make_unique<qpOASES::QProblem>(nV, nC);

    // Get solution
};

}  // namespace yeet::application::mpc