#pragma once

#include <rtfw/task.h>
#include "util.hpp"
#include "eigen3/Eigen/Dense"
#include <manif/manif.h>

using namespace rtfw;
using namespace rtfw::rt;


namespace task_pool {

    class TrackingControl : public ITask {
    public:
        const char* getName() const override { return "TrackingControl"; }
        void setup(TaskRegistry& r) override {
            r.add_dependency(dw_state);
            r.add_dependency(dw_mobile_vel);
            r.add_dependency(dw_target_vel);
            r.add_dependency(dr_start);
            r.add_dependency(dr_stop);
            r.add_dependency(dr_error);
            r.add_dependency(dr_mobile_vel);
            r.add_dependency(dr_vel_target);
            r.add_dependency(dr_odom);
            r.add_dependency(dr_local_target);
            r.add_dependency(p_Q_pos);
            r.add_dependency(p_Q_vel);
            r.add_dependency(p_R_vel);
            r.add_dependency(p_ff_gain);
            r.add_dependency(p_ff_lpf);
            r.add_dependency(p_deadband);
            r.add_dependency(p_slew);
            r.add_dependency(p_tau);
        }
        void initialize() override {
            _dt = 1.0/getFrequency();
            double tau = p_tau.read();
            double alpha = std::exp(-2*M_PI * _dt / tau);
            Ad.setZero();
            Bd.setZero();
            Q.setZero();
            R.setZero();
            Ad.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
            Ad.block<3, 3>(0, 3) = _dt*Eigen::Matrix3d::Identity();
            Ad.block<3, 3>(3, 3) = alpha*Eigen::Matrix3d::Identity();
            Bd.block<3, 3>(3, 0) = (1-alpha)*Eigen::Matrix3d::Identity();
            auto Q_pos_gain = p_Q_pos.read();
            auto Q_vel_gain = p_Q_vel.read();
            auto R_vel_gain = p_R_vel.read();
            Q.diagonal() << Q_pos_gain[0], Q_pos_gain[1], Q_pos_gain[2], 
                            Q_vel_gain[0], Q_vel_gain[1], Q_vel_gain[2];
            R.diagonal() << R_vel_gain[0], R_vel_gain[1], R_vel_gain[2];
            Eigen::Matrix<double, 6, 6> P;
            P.setZero();
            if(solve_dare_static<6, 3>(Ad, Bd, Q, R, P, 1e-6, 1e4)) {
                K = (R+Bd.transpose()*P*Bd).inverse()*Bd.transpose()*P;
            } else {
                getLogger()->error("[{}] discrete ARE solver failed", getName());
                K.setZero();
            }
            odom.setIdentity();
            odom_global.setIdentity();
            vel_pv.setZero();
            target_vel.setZero();
        }
        void execute() override {
            dr_start.on_update([this]() {
                if (!active) {
                    active = true;
                    perr.setZero();
                    vel_sv = vel_pv;
                    odom_global = odom;
                    vel_pv.setZero();
                    target_vel.setZero();
                    getLogger()->info("[{}] state=on", getName());
                    // printf("tracking control: state=on\n");
                }
            });
            dr_stop.on_update([this]() {
                if (active) {
                    active = false;
                    vel_sv.setZero();
                    manif::SE3Tangentd msg_vel;
                    msg_vel.lin().x() = 0;
                    msg_vel.lin().y() = 0;
                    msg_vel.ang().z() = 0;
                    dw_mobile_vel.write(msg_vel);
                    getLogger()->info("[{}] state=off", getName());
                    // printf("tracking control: state=off\n");
                }
            });

            dr_mobile_vel.on_update([this](const manif::SE3Tangentd& data) {
                Eigen::Vector3d vel;
                auto lpf = lpf_gain(getFrequency(), 10);
                vel << data.lin().x(), data.lin().y(), data.ang().z();
                vel_pv = vel_pv*lpf + vel*(1-lpf);
            });
            dr_odom.on_update([this](const manif::SE2d& data) {
                odom = data;
            });


            dr_vel_target.on_update([&](const manif::SE3Tangentd& data) {
                auto lpf = lpf_gain(100, p_ff_lpf.read());
                manif::SE2Tangentd world_vel(data.lin().x(), data.lin().y(), data.ang().z());
                auto target_vel_est = odom.inverse().adj() * world_vel;
                auto target_vel_lpf = target_vel_est * (1-lpf) + target_vel * lpf;
                auto dval = target_vel_lpf - target_vel;
                auto slew = p_slew.read();
                manif::SE2Tangentd dmax(slew[0]*_dt, slew[1]*_dt, slew[2]*_dt);
                for (auto i=0; i<3; i++) {
                    dval.coeffs()[i] = std::clamp(dval.coeffs()[i], -dmax.coeffs()[i], dmax.coeffs()[i]);
                }
                target_vel += dval;
                vel_ff.x() = target_vel.x();
                vel_ff.y() = target_vel.y();
                vel_ff.z() = target_vel.angle();
            });

            dr_error.on_update([this](const manif::SE3Tangentd& data) { 
                odom_global = odom;
                pose_error = data;       
            });

            dr_local_target.on_update([this](const manif::SE3d& data) { 
                local_target = data;
            });

            if (active) {
                manif::SE2Tangentd error(pose_error.lin().x(), pose_error.lin().y(), pose_error.ang().z());
                // error = ((odom_global + error) - odom);
                // error -= local_pos;
        
                update_controller(error);
            }
            
            dw_state.write(active);            
        }
    private:
        // DataWriter<interface::MOTOR> motor[12];
        DataWriter<bool> dw_state{"tc_state"};
        DataWriter<manif::SE3Tangentd> dw_mobile_vel{"tc/velocity_sv"};
        DataWriter<manif::SE2Tangentd> dw_target_vel{"tc/target_vel_est"};
        
        DataReader<Signal> dr_start{"tracking_start"};
        DataReader<Signal> dr_stop{"tracking_stop"};
        DataReader<manif::SE3Tangentd> dr_error{"pose_error_filtered"};
        // DataReader<manif::SE3Tangentd> dr_error{"pose_error"};
        DataReader<manif::SE3Tangentd> dr_mobile_vel{"velocity_pv"};
        DataReader<manif::SE3Tangentd> dr_vel_target{"marker_iekf/vel_target"};
        DataReader<manif::SE3d> dr_local_target{"local_target"};
        // DataReader<manif::SE3Tangentd> dr_vel_target{"marker_iekf/vel_body"};
        DataReader<manif::SE2d> dr_odom{"odometry"};
        Parameter<double> p_tau{"param.LQR.tau", 0.06};
        Parameter<std::array<double, 3>> p_ff_gain{"param.LQR.ff_gain", {0, 0, 0}};
        Parameter<double> p_ff_lpf{"param.LQR.ff_lpf", 5.0};
        Parameter<std::array<double, 3>> p_Q_pos{"param.LQR.Q_pos_gain", {1, 1, 1}};
        Parameter<std::array<double, 3>> p_Q_vel{"param.LQR.Q_vel_gain", {1, 1, 1}};
        Parameter<std::array<double, 3>> p_R_vel{"param.LQR.R_vel_gain", {1, 1, 1}};
        Parameter<std::array<double, 3>> p_slew{"param.LQR.ff_slew", {0.1, 0.1, 0.1}};
        Parameter<std::array<double, 3>> p_deadband{"param.LQR.deadband", {0.0, 0.0, 0.0}};

    private:
        bool update_controller(manif::SE2Tangentd error) {
            if (!active)
                return false;

            manif::SE3Tangentd msg_vel;

            // LQR Control
            
            {
            // auto derr = (error - perr)/_dt;
            
            auto dV = target_vel - vel_pv;
            auto ad = error.smallAdj();
            auto derr = dV - 0.5*ad*dV + 1.0/12*ad*ad*dV;
                
            Eigen::Vector<double, 6> state;
            state.head(3) = -error.coeffs();
            state.tail(3) = -derr.coeffs();

            auto deadband = p_deadband.read();
            for (auto i=0; i<3; i++) {
                if (state.head(3)(i) < -deadband[i]) 
                    state.head(3)(i) += deadband[i];
                else if (state.head(3)(i) > deadband[i]) 
                    state.head(3)(i) -= deadband[i];
                else 
                    state.head(3)(i) = 0;

                // if (std::abs(state.head(3)(i)) < dead_zone[i])
                // state.head(3)(i) = 0;
            }
            
            // state.tail(3) = vel_pv;
            Eigen::Vector3d ff_a(p_ff_gain.read().data());
            vel_sv = -this->K*state + vel_ff.cwiseProduct(ff_a);

            // if (p_deadband.read()) {
            //     if (std::abs(vel_sv.x()) < 0.03) vel_sv.x() = 0;
            //     if (std::abs(vel_sv.y()) < 0.03) vel_sv.y() = 0;
            //     if (std::abs(vel_sv.z()) < 0.03) vel_sv.z() = 0;
            //     // if (vel_sv.norm() < 0.02) vel_sv.setZero();
            // }
            
            msg_vel.lin().x() = vel_sv[0];
            msg_vel.lin().y() = vel_sv[1];
            msg_vel.ang().z() = vel_sv[2];
            }
            dw_mobile_vel.write(msg_vel);
            PERIODIC_CALL(
                getLogger()->info("[{}] vx={:.02f}, vy={:.02f}, wz={:.02f}", getName(), msg_vel.lin().x(), msg_vel.lin().y(), msg_vel.ang().z());
            , 1s);

            perr = error;
            return true;
        }
        
        std::array<double, 3> kp;
        std::array<double, 3> kv;
        
        double v_max = 0.63;
        double w_max = 0.75;

        double _dt;
        manif::SE3Tangentd pose_error;
        manif::SE3d local_target;
        Eigen::Matrix<double, 6, 6> Ad;
        Eigen::Matrix<double, 6, 3> Bd;
        Eigen::Matrix<double, 6, 6> Q;
        Eigen::Matrix<double, 3, 3> R;
        Eigen::Matrix<double, 3, 6> K;
    private:
        bool active = false;
        
        manif::SE2Tangentd perr;

        Eigen::Vector3d vel_ff;
        Eigen::Vector<double, 3> vel_pv;
        Eigen::Vector<double, 3> vel_sv;

        manif::SE2d odom_global;
        manif::SE2d odom;
        manif::SE2Tangentd target_vel;
    };
};