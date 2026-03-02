#include <eigen3/Eigen/Dense>


namespace fourwis {
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


    // wheel speed(fl_x, fl_y, fr_x, fr_y, rl_x, rl_y, rr_x, rr_y) -> frame twist(vx, vy, wz)
    // m, rad, sec
    void fk(Eigen::Matrix<double, 3, 8>& mat, double lx, double ly, double radius) {
        mat.row(0) << 0.25, 0, 0.25, 0, 0.25, 0, 0.25, 0;
        mat.row(1) << 0, 0.25, 0, 0.25, 0, 0.25, 0, 0.25;
        mat.row(2) << -ly, lx, ly, lx, -ly, -lx, ly, -lx;
        mat.row(2) = mat.row(2) * 0.25 / (lx*lx+ly*ly);
        mat = mat * radius;
    }

    // frame twist(vx, vy, wz) -> wheel speed(fl_x, fl_y, fr_x, fr_y, rl_x, rl_y, rr_x, rr_y)
    // m, rad, sec
    void ik(Eigen::Matrix<double, 8, 3>& mat, double lx, double ly, double radius) {

        mat.col(0) << 1, 0, 1, 0, 1, 0, 1, 0;
        mat.col(1) << 0, 1, 0, 1, 0, 1, 0, 1;
        mat.col(2) << -ly, lx, ly, lx, -ly, -lx, ly, -lx;
        mat = mat/radius;
    }

    // wheel speed(fl, fr, rl, rr) -> frame twist(vx, vy, wz)
    // m, rad, sec
    void fk_min(Eigen::Matrix<double, 3, 8>& mat, double lx, double ly, double radius) {
        Eigen::Matrix<double, 8, 3> ik_mat;
        ik(ik_mat, lx, ly, radius);
        mat = (ik_mat.transpose() * ik_mat).inverse() * ik_mat.transpose();
    }

    // wheel speed(fl, fr, rl, rr) -> frame twist(vx, vy, wz)
    // m, rad, sec
    Eigen::Vector<double, 3> fk(Eigen::Vector<double, 8> vel, double lx, double ly, double radius) {
        Eigen::Matrix<double, 3, 8> mat;
        fk(mat, lx, ly, radius);
        return mat*vel;
    }

    // frame twist(vx, vy, wz) -> wheel speed(fl, fr, rl, rr)
    // m, rad, sec
    Eigen::Vector<double, 8> ik(Eigen::Vector<double, 3> vel, double lx, double ly, double radius) {
        Eigen::Matrix<double, 8, 3> mat;
        ik(mat, lx, ly, radius);
        return mat*vel;
    }
    

    // wheel speed(fl, fr, rl, rr) -> frame twist(vx, vy, wz)
    // m, rad, sec
    Eigen::Vector<double, 3> fk_min(Eigen::Vector<double, 8> vel, double lx, double ly, double radius) {
        Eigen::Matrix<double, 3, 8> mat;
        fk_min(mat, lx, ly, radius);
        return mat*vel;
    }
};