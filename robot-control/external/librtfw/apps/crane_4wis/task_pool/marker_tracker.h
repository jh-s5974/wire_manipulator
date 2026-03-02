#pragma once

#include <rtfw/task.h>
#include <eigen3/Eigen/Dense>
#include <deque>
#include "util.hpp"
#include "lpf.hpp"
#include "maf.hpp"
#include <manif/manif.h>

using namespace rtfw;
using namespace rtfw::rt;


namespace task_pool {

    class MarkerTracker : public ITask {
    public:
        const char* getName() const override { return "MarkerTracker"; }
        void setup(TaskRegistry& r) override {
            r.add_dependency(dw_spatial_error);
            r.add_dependency(dw_body_error);
            r.add_dependency(dw_error);
            r.add_dependency(dw_ferror);
            r.add_dependency(dw_target);
            r.add_dependency(dw_ref);
            r.add_dependency(dw_local_target);
            r.add_dependency(dr_odom);
            r.add_dependency(dr_pose_target);
            r.add_dependency(dr_start);
            r.add_dependency(dr_stop);
        }
        void initialize() {
            W_X_n.setIdentity();
            W_X_r.setIdentity();
        }
        void execute() override {
            dr_start.on_update([this](){
                if (!active) {
                    result.ref = W_X_n.inverse()*W_X_r;
                    result.error.setZero();
                    result.target.setIdentity();
                    
                    double fc = 0.5;
                    double fs = getFrequency();
                    lpf_trs.reset();
                    lpf_trs.set_freq(fc, fs);
                    lpf_rot.reset();
                    lpf_rot.set_freq(fc, fs);
                    getLogger()->info("[{}] active", getName());
                    // std::cout <<"MarkerTracker: active" << std::endl;
                    active = true;
                }
            });

            dr_stop.on_update([this](){
                if (active) {
                    getLogger()->info("[{}] deactive", getName());
                    // std::cout <<"MarkerTracker: deactive" << std::endl;
                    active = false;
                }
            });

            dr_odom.on_update([&](const manif::SE2d& data) {
                W_X_n = manif::SE3d(Eigen::Vector3d(data.x(), data.y(), 0), Eigen::AngleAxisd(data.angle(), Eigen::Vector3d::UnitZ()));
            });

            dr_pose_target.on_update([this](const manif::SE3d& data) {
                // n_X_r
                W_X_r = data;
            });

            manif::SE3d n_X_c;
            Eigen::Matrix3d n_R_c;
            n_R_c << 0, 0, 1,
                        -1, 0, 0,
                        0, -1, 0;
            n_X_c.quat(Eigen::Quaterniond(n_R_c));
            n_X_c.translation(Eigen::Vector3d(-(0.24), 0.0, 0));

            auto n_X_r = W_X_n.inverse()*W_X_r;
            dw_target.write(n_X_c.inverse() * n_X_r);

            if (!active) return;

            auto r_X_n = n_X_r.inverse();
            auto r_X_ref = result.ref.inverse();


            // result.error = diff(data, result.ref);
            // right invariant error
            // auto spatial_error = (r_X_ref*r_X_n.inverse()).log();
            // left invariant error
            // auto body_error = (r_X_n.inverse()*r_X_ref).log();


            auto n_X_ref = n_X_r * r_X_ref;

            manif::SE3Tangentd body_error, spatial_error;
            
            spatial_error = (r_X_ref * r_X_n.inverse()).log();
            spatial_error.lin().z() = 0;
            spatial_error.ang().x() = 0;
            spatial_error.ang().y() = 0;
            // if (std::abs(spatial_error.ang().z()) < 0.1) 
                // spatial_error.ang().z() = 0;
            // body_error.lin() = n_X_r.asSO3().quat().matrix() * spatial_error.lin();
            // body_error.ang() = n_X_r.asSO3().quat().matrix() * spatial_error.ang();
            body_error = n_X_r.adj() * spatial_error;



            // result.error = spatial_error;
            result.error = body_error;
            result.ferror.lin() = lpf_trs.update(result.error.lin());
            result.ferror.ang() = lpf_rot.update(result.error.ang());
            // result.error_f.linear = maf_trs.update(result.error_f.linear);
            // result.error_f.angular = maf_rot.update(result.error_f.angular);

            dw_error.write(result.error);
            dw_ferror.write(result.ferror);
            dw_ref.write(n_X_c.inverse()*r_X_ref.inverse());
            // dw_spatial_error.write(spatial_error);
            dw_body_error.write(body_error);
            dw_local_target.write(n_X_r);
        }
    private:
        DataReader<Signal> dr_start{"tracking_start"};
        DataReader<Signal> dr_stop{"tracking_stop"};
        DataReader<manif::SE2d> dr_odom{"odometry"};
        // DataReader<manif::SE3d> dr_pose_target{"lpf/g_pose_target"};
        DataReader<manif::SE3d> dr_pose_target{"marker_iekf/pose_target"};
        
        DataWriter<manif::SE3Tangentd> dw_spatial_error{"spatial_error"};
        DataWriter<manif::SE3Tangentd> dw_body_error{"body_error"};
        DataWriter<manif::SE3Tangentd> dw_error{"pose_error"};
        DataWriter<manif::SE3Tangentd> dw_ferror{"pose_error_filtered"};
        DataWriter<manif::SE3d> dw_local_target{"local_target"};
        DataWriter<manif::SE3d> dw_target{"mt/target"};
        DataWriter<manif::SE3d> dw_ref{"mt/reference"};

    private:
        struct {
            manif::SE3Tangentd error;
            manif::SE3Tangentd ferror;
            manif::SE3d ref;
            manif::SE3d target;
        } result;

    private:
        bool active=false;
        VectorLPF<3> lpf_trs;
        VectorLPF<3> lpf_rot;
        manif::SE3d W_X_n;
        manif::SE3d W_X_r;
        // se3 ml_x_mr;
    };
};
