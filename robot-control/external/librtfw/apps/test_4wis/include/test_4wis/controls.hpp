#pragma once

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Sparse"

// #include <autodiff/forward/dual.hpp>
// #include <autodiff/forward/dual/eigen.hpp>


// using namespace autodiff;

// namespace autodiff {
//     template<int N>
//     using VectorNdual = Eigen::Matrix<autodiff::dual1st, N, 1, 0, N, 1>;
// }


namespace state_space {
    namespace continuous {
        template <int n, int m, int r>
        struct  Linear {
            Eigen::Matrix<double, m, m> A;
            Eigen::Matrix<double, m, r> B;
            Eigen::Matrix<double, n, m> C;
            Eigen::Matrix<double, n, r> D;

        };

        // template <int n, int m, int r>
        // struct Nonlinear {
        //     VectorNdual<m> x;
        //     VectorNdual<r> u;
        //     // VectorNdual<m> (*f)(const VectorNdual<m>& x, const VectorNdual<r>& u, void* param);
        //     // VectorNdual<n> (*g)(const VectorNdual<m>& x, const VectorNdual<r>& u, void* param);
        //     std::function<VectorNdual<m>(const VectorNdual<m>& x, const VectorNdual<r>& u)> f;
        //     std::function<VectorNdual<n>(const VectorNdual<m>& x, const VectorNdual<r>& u)> g;

        //     void linearize(Linear<n, m, r>& res) {
        //         VectorXdual F;

        //         res.A = jacobian(f, wrt(x), at(x, u), F);
        //         res.B = jacobian(f, wrt(u), at(x, u), F);
        //         res.C = jacobian(g, wrt(x), at(x, u), F);
        //         res.D = jacobian(g, wrt(u), at(x, u), F);
        //     }
        // };
    };
    namespace discrete {
        template <int n, int m, int r>
        struct  Linear {
            continuous::Linear<n, m, r> system;
            double T;
        };

    };

    template <int n, int m, int r>
    void discretize(discrete::Linear<n, m, r>& res, continuous::Linear<n, m, r> cont, double T, int order=1) {
        res.system.A = Eigen::Matrix<double, m, m>::Identity();
        Eigen::Matrix<double, m, m> temp = Eigen::Matrix<double, m, m>::Identity() * T;
        Eigen::Matrix<double, m, m> AT = Eigen::Matrix<double, m, m>::Identity();
        for (auto i=0; i<order; i++) {
            AT = AT * cont.A*T;
            res.system.A += AT/(i+1);
            temp += AT/((i+1)*(i+2));
        }
        res.system.B = temp * cont.B * T;
    //    x(kT + T) =  e^AT x(kT) + integrl (kT, kT+T, e^A(kT+T - t)Bu) (tau=kT+T-t, t=kT+T-tau)
    //    x(kT + T) =  {I+(AT)+(AT)^2/2+(AT)^3/3+(AT)^4/4} x(kT) + integrl (0, T, e^At)Bu
    //    x(kT + T) =  {I+(AT)+(AT)^2/2+(AT)^3/3+(AT)^4/4} x(kT) + {T + (AT) T/2+(AT)^2/2 T/3+(AT)^3/3 T/4+(AT)^4/5 T/5}Bu
        res.system.C = cont.C;
        res.system.D = cont.D;
        res.T = T;
    }

    template <int n, int m, int r>
    void discretize_taylor(discrete::Linear<n, m, r>& res, continuous::Linear<n, m, r> cont, double T, int order=1) {
        res.system.A = Eigen::Matrix<double, m, m>::Identity();
        Eigen::Matrix<double, m, m> temp = Eigen::Matrix<double, m, m>::Zero();
        Eigen::Matrix<double, m, m> AT = Eigen::Matrix<double, m, m>::Identity();
        for (auto i=0; i<order; i++) {
            if (i != 0)
                temp += AT/(i*(i+1));
            else
                temp += AT;
            AT = AT * cont.A*T;
            res.system.A += AT/(i+1);
        }
        res.system.B = temp * cont.B * T;
    //    x(kT + T) =  e^AT x(kT) + integrl (kT, kT+T, e^A(kT+T - t)Bu) (tau=kT+T-t, t=kT+T-tau)
    //    x(kT + T) =  {I+(AT)+(AT)^2/2+(AT)^3/3+(AT)^4/4} x(kT) + integrl (0, T, e^At)Bu
    //    x(kT + T) =  {I+(AT)+(AT)^2/2+(AT)^3/3+(AT)^4/4} x(kT) + {T + (AT) T/2+(AT)^2/2 T/3+(AT)^3/3 T/4}Bu
        res.system.C = cont.C;
        res.system.D = cont.D;
        res.T = T;
    }

    template <int n, int m, int r>
    void minreal(state_space::continuous::Linear<n, m, r>& res, state_space::continuous::Linear<n, m, r> ls) {
        
    }

};