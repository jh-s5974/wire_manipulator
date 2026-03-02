#pragma once

#include "rtfw/task.h"
#include "eigen3/Eigen/Dense"
#include <manif/manif.h>
#include <cmath>
#include <vector>
#include <numeric>
#include <interfaces/CvResultData.h>


using namespace rtfw::rt;

namespace task_pool {
    class PostProcessTask : public ITask {
    private:
        enum class FilterState {
            WAITING_FOR_FIRST_SIGHTING,
            RUNNING
        };

    public:
        const char* getName() const override { return "PostProcess"; }

        void setup(TaskRegistry& r) override {
            r.add_dependency(dr_data);
            r.add_dependency(dr_odom);
            for (int i=0; i<4; i++) {
                r.add_dependency(dw_pose[i]);
                r.add_dependency(dw_gpose[i]);
            }
        }

        void initialize() override {
            
        }

        void execute() override {
            manif::SE3d odom_pose;
            bool odom_updated = false;
            dr_odom.on_update([&](const manif::SE2d& data) {
                odom_pose.quat(Eigen::AngleAxisd(data.angle(), Eigen::Vector3d::UnitZ()));
                odom_pose.translation(Eigen::Vector3d(data.x(), data.y(), 0));
                odom_updated = true;
            });
            
            // --- Vision 측정 ---
            manif::SE3d marker_meas[3];
            bool marker_updated[3] = {false, false, false};
            dr_data.on_update([&](const custom_types::CvResultData& data) {
                for (int i=0; i<3; i++) {
                    marker_meas[i] = data.marker[i];
                    marker_updated[i] = data.detect[i];
                }
            });

            manif::SE3d left_focus(Eigen::Vector3d(0, -0.18, 0), Eigen::Quaterniond::Identity());
            manif::SE3d right_focus(Eigen::Vector3d(0, -0.18, 0), Eigen::Quaterniond::Identity());
            manif::SE3d body_focus(Eigen::Vector3d(0, 0, -0.2), Eigen::Quaterniond::Identity());


            manif::SE3d r_X_m;
            manif::SE3d n_X_c;
            
            Eigen::Matrix3d r_R_m;
            r_R_m <<  0,  0, -1,
                     -1,  0,  0,
                      0,  1,  0;
            r_X_m.quat(Eigen::Quaterniond(r_R_m));
            r_X_m.translation(Eigen::Vector3d::Zero());

            Eigen::Matrix3d n_R_c;
            n_R_c <<  0,  0,  1,
                     -1,  0,  0,
                      0, -1,  0;
            n_X_c.quat(Eigen::Quaterniond(n_R_c));
            n_X_c.translation(Eigen::Vector3d(-(0.36 - 0.12), 0, 0));

            if (marker_updated[0]) marker_meas[0] = n_X_c*marker_meas[0] * body_focus * r_X_m.inverse();
            if (marker_updated[1]) marker_meas[1] = n_X_c*marker_meas[1] * left_focus * r_X_m.inverse();
            if (marker_updated[2]) marker_meas[2] = n_X_c*marker_meas[2] * right_focus * r_X_m.inverse();

            for (auto i=0; i<3; i++) {
                if (marker_updated[i]) {
                    dw_pose[i].write(marker_meas[i]);
                    if (odom_updated)
                        dw_gpose[i].write(odom_pose*marker_meas[i]);
                }
            }

            if (marker_updated[1] || marker_updated[2]) {
                manif::SE3d N_X_left = marker_meas[1];
                manif::SE3d N_X_right= marker_meas[2];
    
                manif::SE3d target(
                    0.5*(N_X_left.translation()+N_X_right.translation()),
                    N_X_left.quat().slerp(0.5, N_X_right.quat()));
                dw_pose[3].write(target);
                if (odom_updated)
                    dw_gpose[3].write(odom_pose*target);
            }                
        }

    private:
        DataReader<custom_types::CvResultData> dr_data{"cv_data"};
        DataReader<manif::SE2d> dr_odom{"odometry"};

        DataWriter<manif::SE3d> dw_pose[4] = {
            DataWriter<manif::SE3d>{"raw/l_pose_body"},
            DataWriter<manif::SE3d>{"raw/l_pose_left_foot"},
            DataWriter<manif::SE3d>{"raw/l_pose_right_foot"},
            DataWriter<manif::SE3d>{"raw/l_pose_target"}
        };

        DataWriter<manif::SE3d> dw_gpose[4] = {
            DataWriter<manif::SE3d>{"raw/g_pose_body"},
            DataWriter<manif::SE3d>{"raw/g_pose_left_foot"},
            DataWriter<manif::SE3d>{"raw/g_pose_right_foot"},
            DataWriter<manif::SE3d>{"raw/g_pose_target"}
        };

    private:
    };
};