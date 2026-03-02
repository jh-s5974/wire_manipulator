#include <eigen3/Eigen/Dense>


namespace mecanum {
    // l_x, l_y : wheel distances from center

    // forward kinematics
    // v_x = (w_fl + w_fr + w_rl + w_rr) r/4
    // v_y = (-w_fl + w_fr + w_rl - w_rr) r/4
    // w_z = (-w_fl + w_fr - w_rl + w_rr) r/[4(l_x + l_y)]

    // inverse kinematics
    // w_fl = 1/r [v_x - v_y - (l_x + l_y)w_z]
    // w_fr = 1/r [v_x + v_y + (l_x + l_y)w_z]
    // w_rl = 1/r [v_x + v_y - (l_x + l_y)w_z]
    // w_rr = 1/r [v_x - v_y + (l_x + l_y)w_z]


    // wheel speed(fl, fr, rl, rr) -> frame twist(vx, vy, wz)
    // m, rad, sec
    void fk(Eigen::Matrix<double, 3, 4>& mat, double lx, double ly, double radius) {
        auto a = radius*0.25;
        auto b = a/(lx + ly);
        mat.row(0) << a, a, a, a;
        mat.row(1) << -a, a, a, -a;
        mat.row(2) << -b, b, -b, b;
    }

    // frame twist(vx, vy, wz) -> wheel speed(fl, fr, rl, rr)
    // m, rad, sec
    void ik(Eigen::Matrix<double, 4, 3>& mat, double lx, double ly, double radius) {
        auto a = 1.0/radius;
        auto b = (lx + ly)*a;
        mat.col(0) << a, a, a, a;
        mat.col(1) << -a, a, a, -a;
        mat.col(2) << -b, b, -b, b;
    }

    // wheel speed(fl, fr, rl, rr) -> frame twist(vx, vy, wz)
    // m, rad, sec
    void fk_min(Eigen::Matrix<double, 3, 4>& mat, double lx, double ly, double radius) {
        Eigen::Matrix<double, 4, 3> ik_mat;
        ik(ik_mat, lx, ly, radius);
        mat = ik_mat.transpose() * (ik_mat * ik_mat.transpose()).inverse();
    }

    // wheel speed(fl, fr, rl, rr) -> frame twist(vx, vy, wz)
    // m, rad, sec
    Eigen::Vector<double, 3> fk(Eigen::Vector<double, 4> vel, double lx, double ly, double radius) {
        Eigen::Matrix<double, 3, 4> mat;
        fk(mat, lx, ly, radius);
        return mat*vel;
    }

    // frame twist(vx, vy, wz) -> wheel speed(fl, fr, rl, rr)
    // m, rad, sec
    Eigen::Vector<double, 4> ik(Eigen::Vector<double, 3> vel, double lx, double ly, double radius) {
        Eigen::Matrix<double, 4, 3> mat;
        ik(mat, lx, ly, radius);
        return mat*vel;
    }
    

};