#pragma once

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Sparse"
#include <OsqpEigen/OsqpEigen.h>


class QP {
    OsqpEigen::Solver solver;
public:
    ~QP() {
        clear();
    }
    void set_number_of_variables(int n) {
        solver.data()->setNumberOfVariables(n);
    }
    void set_number_of_constraints(int m) {
        solver.data()->setNumberOfConstraints(m);
    }
    bool set_bounds(Eigen::VectorXd& lb, Eigen::VectorXd& ub) {
        return solver.data()->setBounds(lb, ub);
    }
    bool set_lower_bound(Eigen::VectorXd& lb) {
        return solver.data()->setLowerBound(lb);
    }
    bool set_upper_bound(Eigen::VectorXd& ub) {
        return solver.data()->setLowerBound(ub);
    }
    bool set_hessian(Eigen::SparseMatrix<double>& P) {
        return solver.data()->setHessianMatrix(P);
    }
    bool set_gradient(Eigen::VectorXd& q) {
        return solver.data()->setGradient(q);
    }
    bool set_linear_constraints(Eigen::SparseMatrix<double>& A) {
        return solver.data()->setLinearConstraintsMatrix(A);
    }
    void set_warm_start(bool on) {
        solver.settings()->setWarmStart(on);
    }
    void set_timelimit(double sec) {
        // solver.settings()->setTimeLimit(sec);
        solver.settings()->getSettings()->time_limit = sec;
    }
    void set_maxiter(int max_iter) {
        solver.settings()->setMaxIteration(max_iter);
    }
    void set_verbosity(bool on) {
        solver.settings()->setVerbosity(on);
    }
    bool init() {
        return solver.initSolver();
    }
    void clear() {
        solver.clearSolver();
    }

    // argmin(x)  J=1/2 x'Px + qx
    // s.t.        lb <= A <= ub
    bool solveProblem(int* perror=nullptr) {
        OsqpEigen::ErrorExitFlag res = solver.solveProblem();
        if (perror)
            *perror = (int)res;
        return res == OsqpEigen::ErrorExitFlag::NoError;
    }

    Eigen::VectorXd get_solution() {
        return solver.getSolution();
    }

    bool set_initial_value(Eigen::VectorXd& x0) {
        return solver.setPrimalVariable(x0);
    }

    bool update_bounds(Eigen::VectorXd& lb, Eigen::VectorXd& ub) {
        return solver.updateBounds(lb, ub);
    }
    bool update_lower_bound(Eigen::VectorXd& lb) {
        return solver.updateLowerBound(lb);
    }
    bool update_upper_bound(Eigen::VectorXd& ub) {
        return solver.updateLowerBound(ub);
    }
    bool update_hessian(Eigen::SparseMatrix<double>& P) {
        return solver.updateHessianMatrix(P);
    }
    bool update_gradient(Eigen::VectorXd& q) {
        return solver.updateGradient(q);
    }
    bool update_linear_constraints(Eigen::SparseMatrix<double>& A) {
        return solver.updateLinearConstraintsMatrix(A);
    }
};
