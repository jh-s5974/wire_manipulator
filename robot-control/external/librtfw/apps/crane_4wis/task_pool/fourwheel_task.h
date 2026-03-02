#pragma once

#include <rtfw/task.h>
// #include "frame.hpp"
#include <manif/manif.h>
#include "lpf.hpp"

using namespace rtfw;
using namespace rtfw::rt;

namespace Eigen {
    using Vector8d = Eigen::Vector<double, 8>;
}

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
static void fourwheel_fk(Eigen::Matrix<double, 3, 8>& mat, double lx, double ly, double radius) {    
    auto a = radius*0.25;
    auto b = a*lx;
    auto c = a*ly;
    mat.row(0) << a, 0, a, 0, a, 0, a, 0;
    mat.row(1) << 0, a, 0, a, 0, a, 0, a;
    mat.row(2) << -c*1, b*1, -c*-1, b*1, -c*1, b*-1, -c*-1, b*-1;
}

// frame twist(vx, vy, wz) -> wheel speed(fl, fr, rl, rr)
// m, rad, sec
static void fourwheel_ik(Eigen::Matrix<double, 8, 3>& mat, double lx, double ly, double radius) {
    // self.ik_vel = np.array([
    //     [1, 0, -self.ly[0]],
    //     [0, 1, self.lx[0]],
    //     [1, 0, -self.ly[1]],
    //     [0, 1, self.lx[1]],
    //     [1, 0, -self.ly[2]],
    //     [0, 1, self.lx[2]],
    //     [1, 0, -self.ly[3]],
    //     [0, 1, self.lx[3]]
    // ]) / self.rw

    auto a = 1.0/radius;
    auto b = lx*a;
    auto c = ly*a;
    mat.col(0) << a, 0, a, 0, a, 0, a, 0;
    mat.col(1) << 0, a, 0, a, 0, a, 0, a;
    mat.col(2) << -c*1, b*1, -c*-1, b*1, -c*1, b*-1, -c*-1, b*-1;
}

// wheel speed(fl, fr, rl, rr) -> frame twist(vx, vy, wz)
// m, rad, sec
static Eigen::Vector<double, 3> fourwheel_fk(Eigen::Vector<double, 8> vel, double lx, double ly, double radius) {
    Eigen::Matrix<double, 3, 8> mat;
    fourwheel_fk(mat, lx, ly, radius);
    return mat*vel;
}

// frame twist(vx, vy, wz) -> wheel speed(fl, fr, rl, rr)
// m, rad, sec
static Eigen::Vector<double, 8> fourwheel_ik(Eigen::Vector<double, 3> vel, double lx, double ly, double radius) {
    Eigen::Matrix<double, 8, 3> mat;
    fourwheel_ik(mat, lx, ly, radius);
    return mat*vel;
}

namespace task_pool {

    class FK : public ITask {
    public:
        const char* getName() const override { return "fourwheel_FK"; }
        void setup(TaskRegistry& r) override {
            r.add_dependency(dw_velocity_pv);
            r.add_dependency(dr_motor_pv);
        }
        void execute() override {
            dr_motor_pv.on_update([this](const std::array<double, 8>& data){
                Eigen::Vector8d vi;
                vi <<   data[0]*std::cos(data[4]), data[0]*std::sin(data[4]), 
                        data[1]*std::cos(data[5]), data[1]*std::sin(data[5]), 
                        data[2]*std::cos(data[6]), data[2]*std::sin(data[6]), 
                        data[3]*std::cos(data[7]), data[3]*std::sin(data[7]);
                vi *= 2*M_PI/60;    // rpm -> rad/s
                Eigen::Matrix<double, 3, 8> fk;
                // mecanum::fk_min(fk, width, length, radius);
                fourwheel_fk(fk, length, width, radius);

                Eigen::Vector3d vel = fk * vi;
                
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
        DataReader<std::array<double, 8>> dr_motor_pv{"motor_pv"};

    private:
    };


    class IK : public ITask {
    public:
        const char* getName() const override { return "4WIS_IK"; }
        void setup(TaskRegistry& r) override {
            r.add_dependency(dw_motor_sv);
            r.add_dependency(dr_motor_pv);
            r.add_dependency(dr_velocity_sv);
        }
        void initialize() override {
            vel_sv.setZero();
            vel_slew.setZero();
        }
        void execute() override {

            dr_motor_pv.on_update([this](const std::array<double, 8>& data) {
                prev_steer(0) = data[4];
                prev_steer(1) = data[5];
                prev_steer(2) = data[6];
                prev_steer(3) = data[7];
            });


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
                Eigen::Matrix<double, 8, 3> ik;
                fourwheel_ik(ik, length, width, radius);

                Eigen::Vector<double, 8> vi_flat = ik * vel;
                vi_flat *= 60/2/M_PI;    // rad/s -> rpm

                // reshape to (4, 2) - 각 휠별로 [vx, vy] 2개씩
                Eigen::Matrix<double, 4, 2> vi;
                for (int i = 0; i < 4; ++i) {
                    vi(i, 0) = vi_flat(2 * i);      // vx
                    vi(i, 1) = vi_flat(2 * i + 1);  // vy
                }
                
                // wheel_mag = norm(vi, axis=1) - 각 휠의 속도 크기
                Eigen::Vector4d wheel_mag;
                for (int i = 0; i < 4; ++i) {
                    wheel_mag(i) = vi.row(i).norm();
                }
                
                // steer_base = atan2(vi[:, 1], vi[:, 0] + eps)
                double eps = 1e-5;
                Eigen::Vector4d steer_base;
                for (int i = 0; i < 4; ++i) {
                    steer_base(i) = std::atan2(vi(i, 1), vi(i, 0) + eps);
                }
                
                // steer_pi = atan2(sin(steer_base + π), cos(steer_base + π))
                Eigen::Vector4d steer_pi;
                for (int i = 0; i < 4; ++i) {
                    float angle = steer_base(i) + M_PI;
                    steer_pi(i) = std::atan2(std::sin(angle), std::cos(angle));
                }
                
                // err_base_raw = steer_base - prev_steer[:4]
                Eigen::Vector4d err_base_raw = steer_base - prev_steer;
                
                // err_base = abs(atan2(sin(err_base_raw), cos(err_base_raw)))
                Eigen::Vector4d err_base;
                for (int i = 0; i < 4; ++i) {
                    err_base(i) = std::abs(std::atan2(std::sin(err_base_raw(i)), 
                                                    std::cos(err_base_raw(i))));
                }
                
                // err_pi_raw = steer_pi - prev_steer[:4]
                Eigen::Vector4d err_pi_raw = steer_pi - prev_steer.head(4);
                
                // err_pi = abs(atan2(sin(err_pi_raw), cos(err_pi_raw)))
                Eigen::Vector4d err_pi;
                for (int i = 0; i < 4; ++i) {
                    err_pi(i) = std::abs(std::atan2(std::sin(err_pi_raw(i)), 
                                                    std::cos(err_pi_raw(i))));
                }
                
                // use_pi_solution = err_pi < err_base (boolean vector)
                Eigen::Vector4i use_pi_solution;
                for (int i = 0; i < 4; ++i) {
                    use_pi_solution(i) = (err_pi(i) < err_base(i)) ? 1 : 0;
                }
                
                // final_steer = where(use_pi_solution, steer_pi, steer_base)
                Eigen::Vector4d final_steer;
                for (int i = 0; i < 4; ++i) {
                    final_steer(i) = use_pi_solution(i) ? steer_pi(i) : steer_base(i);
                }
                
                // wheel_sign = where(use_pi_solution, -1.0, 1.0)
                Eigen::Vector4d wheel_sign;
                for (int i = 0; i < 4; ++i) {
                    wheel_sign(i) = use_pi_solution(i) ? -1.0f : 1.0f;
                }
                
                // final_wheel = wheel_sign * wheel_mag
                Eigen::Vector4d rpm = wheel_sign.cwiseProduct(wheel_mag);
                


                double max_coeff = rpm.cwiseAbs().maxCoeff();
                if (max_coeff > rpm_max) {
                    // rpm scaling
                    rpm = rpm * rpm_max/max_coeff;
                }
                // for (auto i=0; i<4; i++) {
                //   if (rpm[i] > rpm_max) rpm[i] = rpm_max;
                //   if (rpm[i] < -rpm_max) rpm[i] = -rpm_max;
                // }
                std::array<double, 8> msg_motor;
                for (auto i=0; i<4; i++)
                    msg_motor[i] = rpm[i];
                for (auto i=0; i<4; i++)
                    msg_motor[i+4] = final_steer[i];
                dw_motor_sv.write(msg_motor);
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
        Eigen::Vector4d prev_steer;
    private:
        DataWriter<std::array<double, 8>> dw_motor_sv{"motor_sv"};
        DataReader<std::array<double, 8>> dr_motor_pv{"motor_pv"};
        DataReader<manif::SE3Tangentd> dr_velocity_sv{"velocity_sv"};
    private:
    };
};