#pragma once

#include "rtfw/task.h"
#include "interfaces/CvResultData.h"
// #include "frame.hpp"
#include "util.hpp"


#include <eigen3/Eigen/Dense>
#include <manif/manif.h>
#include "lpf.hpp"


using namespace rtfw::rt;

namespace task_pool {

    class MarkerLPF : public ITask {
    public:
        const char* getName() const override { return "MarkerLpf"; }
        void setup(TaskRegistry& r) override {
            for (auto i=0; i<4; i++) {
                r.add_dependency(dr_pose[i]);
                r.add_dependency(dr_gpose[i]);
                r.add_dependency(dw_fpose[i]);
                r.add_dependency(dw_fgpose[i]);
            }
        }

        void initialize() override {
            gain = lpf_gain(getFrequency(), 1);
            // gain = 0;
            for (auto i=0; i<4; i++) {
                fpose[i].setIdentity();
                fgpose[i].setIdentity();
                lost[i] = (uint64_t)-1;
                glost[i] = (uint64_t)-1;
            }
        }

        void execute() override {
            for (auto i=0; i<4; i++) {
                lost[i]++;
                dr_pose[i].on_update([&](const manif::SE3d& data) {
                    if (lost[i] > 10) {
                        fpose[i] = data;
                    } else {
                        fpose[i].translation(gain * fpose[i].translation() + (1-gain) * data.translation());
                        fpose[i].quat(data.quat().slerp(gain, Eigen::Quaterniond(fpose[i].quat())));
                    }
                    dw_fpose[i].write(fpose[i]);
                    lost[i] = 0;
                });


                glost[i]++;
                dr_gpose[i].on_update([&](const manif::SE3d& data) {
                    if (glost[i] > 10) {
                        fgpose[i] = data;
                    } else {
                        fgpose[i].translation(gain * fgpose[i].translation() + (1-gain) * data.translation());
                        fgpose[i].quat(data.quat().slerp(gain, Eigen::Quaterniond(fgpose[i].quat())));
                    }
                    dw_fgpose[i].write(fgpose[i]);
                    glost[i] = 0;
                });
                
            }
        }
    private:
        DataReader<manif::SE3d> dr_pose[4] = {
            DataReader<manif::SE3d>{"raw/l_pose_body"},
            DataReader<manif::SE3d>{"raw/l_pose_left_foot"},
            DataReader<manif::SE3d>{"raw/l_pose_right_foot"},
            DataReader<manif::SE3d>{"raw/l_pose_target"}
        };

        DataReader<manif::SE3d> dr_gpose[4] = {
            DataReader<manif::SE3d>{"raw/g_pose_body"},
            DataReader<manif::SE3d>{"raw/g_pose_left_foot"},
            DataReader<manif::SE3d>{"raw/g_pose_right_foot"},
            DataReader<manif::SE3d>{"raw/g_pose_target"}
        };
    
        DataWriter<manif::SE3d> dw_fpose[4] = {
            DataWriter<manif::SE3d>{"lpf/l_pose_body"},
            DataWriter<manif::SE3d>{"lpf/l_pose_left_foot"},
            DataWriter<manif::SE3d>{"lpf/l_pose_right_foot"},
            DataWriter<manif::SE3d>{"lpf/l_pose_target"},
        };

        DataWriter<manif::SE3d> dw_fgpose[4] = {
            DataWriter<manif::SE3d>{"lpf/g_pose_body"},
            DataWriter<manif::SE3d>{"lpf/g_pose_left_foot"},
            DataWriter<manif::SE3d>{"lpf/g_pose_right_foot"},
            DataWriter<manif::SE3d>{"lpf/g_pose_target"},
        };

    private:
        double gain;
        std::array<manif::SE3d, 4> fpose;
        std::array<manif::SE3d, 4> fgpose;
        uint64_t lost[4];
        uint64_t glost[4];
    };
};