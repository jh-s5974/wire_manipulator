#pragma once

#include "rtfw/task.h"
#include "interfaces/CvResultData.h"
// #include "frame.hpp"
#include "util.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <deque>
#include <thread>
#include <future>
#include <fstream>

#include <eigen3/Eigen/Dense>
// #include <opencv2/objdetect.hpp>
#include "json.hpp"


enum class SM_State {
    IDLE,
    CONF,
    ACTIVE,
    INACTIVE,
    UNCONF,
};

using namespace rtfw;

namespace task_pool {

    class VisionTask : public rt::ITask {
    public:
        const char* getName() const override { return "Vision_Task"; }
        void setup(rt::TaskRegistry& r) override {
            r.add_dependency(dr_cmd_state);
            r.add_dependency(dr_cmd_rec_start);
            r.add_dependency(dr_cmd_rec_stop);
            r.add_dependency(dr_cmd_setup);
            r.add_dependency(dr_target);
            r.add_dependency(dw_state);
            r.add_dependency(dw_detect);
            r.add_dependency(dw_data);
        }
        void execute() override {
            dr_cmd_state.on_update([this](const bool& data) {
                if (data && !active) {
                    active = data;
                }
                if (!data && active) {
                    active = data;
                }
            });

            dr_cmd_rec_start.on_update([this](const bool& data) {

            });

            dr_cmd_rec_stop.on_update([this](const bool& data) {
                
            });

            dr_cmd_setup.on_update([&](const double& data) {
                if (data && !active) {
                    active = data;
                }
                if (!data && active) {
                    active = data;
                }
            });

            dr_target.on_update([this](const manif::SE3d& data) {
                
            });

            dw_state.write(active);
        }
    private:
        rt::DataReader<bool> dr_cmd_state{"cmd_cv_state"};
        rt::DataReader<bool> dr_cmd_rec_start{"js/btn_10"};
        rt::DataReader<bool> dr_cmd_rec_stop{"js/btn_11"};
        rt::DataReader<double> dr_cmd_setup{"js/axis_7"};
        rt::DataReader<manif::SE3d> dr_target{"mt/target"};
        rt::DataWriter<bool> dw_state{"cv_state", ArchiveOption::Enable};
        rt::DataWriter<bool> dw_detect{"marker_detection", ArchiveOption::Enable};
        rt::DataWriter<custom_types::CvResultData> dw_data{"cv_data", ArchiveOption::Enable};

    private:
        bool active = false;
    };
};