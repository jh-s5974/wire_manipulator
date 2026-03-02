#pragma once

#include <rtfw/task.h>
// #include "frame.hpp"
#include <manif/manif.h>
#include "lpf.hpp"

using namespace rtfw;
using namespace rtfw::rt;


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
static void mecanum_fk(Eigen::Matrix<double, 3, 4>& mat, double lx, double ly, double radius) {
    auto a = radius*0.25;
    auto b = a/(lx + ly);
    mat.row(0) << a, a, a, a;
    mat.row(1) << -a, a, a, -a;
    mat.row(2) << -b, b, -b, b;
}

// frame twist(vx, vy, wz) -> wheel speed(fl, fr, rl, rr)
// m, rad, sec
static void mecanum_ik(Eigen::Matrix<double, 4, 3>& mat, double lx, double ly, double radius) {
    auto a = 1.0/radius;
    auto b = (lx + ly)*a;
    mat.col(0) << a, a, a, a;
    mat.col(1) << -a, a, a, -a;
    mat.col(2) << -b, b, -b, b;
}

// wheel speed(fl, fr, rl, rr) -> frame twist(vx, vy, wz)
// m, rad, sec
static Eigen::Vector<double, 3> mecanum_fk(Eigen::Vector<double, 4> vel, double lx, double ly, double radius) {
    Eigen::Matrix<double, 3, 4> mat;
    mecanum_fk(mat, lx, ly, radius);
    return mat*vel;
}

// frame twist(vx, vy, wz) -> wheel speed(fl, fr, rl, rr)
// m, rad, sec
static Eigen::Vector<double, 4> mecanum_ik(Eigen::Vector<double, 3> vel, double lx, double ly, double radius) {
    Eigen::Matrix<double, 4, 3> mat;
    mecanum_ik(mat, lx, ly, radius);
    return mat*vel;
}

namespace task_pool {

    class MecanumFK : public ITask {
    public:
        MecanumFK(double max_rpm=60): rpm_max(max_rpm) {
            Eigen::Matrix<double, 3, 4> fk;
            mecanum_fk(fk, width, length, radius);

            Eigen::Vector4d rpm;
            rpm << rpm_max, rpm_max, rpm_max, rpm_max;
            rpm *= 2*M_PI/60;    // rpm -> rad/s
            Eigen::Vector3d velx = fk * rpm;

            rpm << -rpm_max, rpm_max, rpm_max, -rpm_max;
            rpm *= 2*M_PI/60;    // rpm -> rad/s
            Eigen::Vector3d vely = fk * rpm;

            rpm << -rpm_max, rpm_max, -rpm_max, rpm_max;
            rpm *= 2*M_PI/60;    // rpm -> rad/s
            Eigen::Vector3d rot = fk * rpm;

            vel_max.lin().x() = velx[0];
            vel_max.lin().y() = vely[1];
            vel_max.ang().z() = rot[2];
        }
        const char* getName() const override { return "MECANUM_FK"; }
        void setup(TaskRegistry& r) override {
            r.add_dependency(dw_velocity_pv);
            r.add_dependency(dr_wheel_pv);
        }
        void execute() override {
            dr_wheel_pv.on_update([this](const std::array<double, 4>& data){
                Eigen::Vector4d rpm;
                rpm << data[0], data[1], data[2], data[3];
                rpm *= 2*M_PI/60;    // rpm -> rad/s
                Eigen::Matrix<double, 3, 4> fk;
                // mecanum::fk_min(fk, width, length, radius);
                mecanum_fk(fk, width, length, radius);

                Eigen::Vector3d vel = fk * rpm;
                
                vel_pv.lin().x() = vel[0];
                vel_pv.lin().y() = vel[1];
                vel_pv.ang().z() = vel[2];
                dw_velocity_pv.write(vel_pv);
            });
        }

    private:
        double L[2];
        double R;

        double width = 0.58;      // m
        double length = 0.25357;  // m
        double radius = 0.1;       // m
        double rpm_max = 60;        // rpm
        manif::SE3Tangentd vel_max;

        manif::SE3Tangentd vel_pv;
    private:
        DataWriter<manif::SE3Tangentd> dw_velocity_pv{"velocity_pv"};
        DataReader<std::array<double, 4>> dr_wheel_pv{"wheel_pv"};

    private:
    };


    class MecanumIK : public ITask {
    public:
        MecanumIK(double max_rpm=60): rpm_max(max_rpm) {
            Eigen::Matrix<double, 3, 4> fk;
            mecanum_fk(fk, width, length, radius);

            Eigen::Vector4d rpm;
            rpm << rpm_max, rpm_max, rpm_max, rpm_max;
            rpm *= 2*M_PI/60;    // rpm -> rad/s
            Eigen::Vector3d velx = fk * rpm;

            rpm << -rpm_max, rpm_max, rpm_max, -rpm_max;
            rpm *= 2*M_PI/60;    // rpm -> rad/s
            Eigen::Vector3d vely = fk * rpm;

            rpm << -rpm_max, rpm_max, -rpm_max, rpm_max;
            rpm *= 2*M_PI/60;    // rpm -> rad/s
            Eigen::Vector3d rot = fk * rpm;

            vel_max.lin().x() = velx[0];
            vel_max.lin().y() = vely[1];
            vel_max.ang().z() = rot[2];
            std::cout << "max vel=" << velx[0] << ", " << "max rot= " << rot[2] << std::endl;
        }
        const char* getName() const override { return "MECANUM_IK"; }
        void setup(TaskRegistry& r) override {
            r.add_dependency(dw_wheel_sv);
            r.add_dependency(dr_velocity_sv);
        }
        void initialize() override {
            vel_sv.setZero();
            vel_slew.setZero();
        }
        void execute() override {
            dr_velocity_sv.on_update([this](const manif::SE3Tangentd& data) {          
                const double lpf = lpf_gain(getFrequency(), 10); // exp(-2pi*fc/fs)
                vel_sv.lin() = lpf*vel_sv.lin() + (1-lpf) * data.lin();
                vel_sv.ang() = lpf*vel_sv.ang() + (1-lpf) * data.ang();
                auto dvel = vel_sv - vel_slew;
                auto hz = getFrequency();
                Eigen::Vector3d mag;
                mag << dvel.lin().x(), dvel.lin().y(), dvel.ang().z();
                double norm = mag.norm();
                if (norm > 0.3/hz) {
                    dvel *= 0.3/hz/norm;
                }
                // dvel.lin().x() = std::clamp(dvel.lin().x(), -0.3/hz, 0.3/hz);
                // dvel.lin().y() = std::clamp(dvel.lin().y(), -0.3/hz, 0.3/hz);
                // dvel.ang().z() = std::clamp(dvel.ang().z(), -0.3/hz, 0.3/hz);
                vel_slew += dvel;

                Eigen::Vector3d vel;
                vel << vel_slew.lin().x(), vel_slew.lin().y(), vel_slew.ang().z();
                Eigen::Matrix<double, 4, 3> ik;
                mecanum_ik(ik, width, length, radius);

                Eigen::Vector4d rpm = ik * vel;
                rpm *= 60/2/M_PI;    // rad/s -> rpm
                double rpm_norm = rpm.norm();
                double max_coeff = rpm.cwiseAbs().maxCoeff();
                if (max_coeff > rpm_max) {
                // rpm scaling
                rpm = rpm * rpm_max/max_coeff;
                }
                // for (auto i=0; i<4; i++) {
                //   if (rpm[i] > rpm_max) rpm[i] = rpm_max;
                //   if (rpm[i] < -rpm_max) rpm[i] = -rpm_max;
                // }
                std::array<double, 4> msg_rpm;
                for (auto i=0; i<4; i++)
                msg_rpm[i] = rpm[i];
                dw_wheel_sv.write(msg_rpm);
            });
          
        }

    private:
        double L[2];
        double R;

        double width = 0.58;      // m
        double length = 0.25357;  // m
        double radius = 0.1;       // m
        double rpm_max = 60;        // rpm
        manif::SE3Tangentd vel_max;

        manif::SE3Tangentd vel_sv;
        manif::SE3Tangentd vel_slew;
    private:
        DataWriter<std::array<double, 4>> dw_wheel_sv{"wheel_sv"};
        DataReader<manif::SE3Tangentd> dr_velocity_sv{"velocity_sv"};
    private:
    };
};