#pragma once

#include <rtfw/task.h>
#include <manif/manif.h>

using namespace rtfw;
using namespace rtfw::rt;


namespace task_pool {

    class VelocitySelector : public ITask {
    public:
        const char* getName() const override { return "velocity_selector"; }
        void setup(TaskRegistry& r) override {
            r.add_dependency(dr_switch);
            r.add_dependency(dr_input_1);
            r.add_dependency(dr_input_2);
            r.add_dependency(dw_output);
        }
        void execute() override {
            if (dr_switch.read()) {
                dr_input_2.on_update([this](const manif::SE3Tangentd& data) {
                    dw_output.write(data);
                });
            } else {
                dr_input_1.on_update([this](const manif::SE3Tangentd& data) {
                    dw_output.write(data);
                });
            }
        }
    private:
        // DataWriter<interface::MOTOR> motor[12];
        DataReader<bool> dr_switch{"tc_state"};
        DataReader<manif::SE3Tangentd> dr_input_1{"sm/velocity_sv"};
        DataReader<manif::SE3Tangentd> dr_input_2{"tc/velocity_sv"};
        DataWriter<manif::SE3Tangentd> dw_output{"velocity_sv"};
    };


    class CraneSpdSelector : public ITask {
    public:
        const char* getName() const override { return "crane_spd_selector"; }
        void setup(TaskRegistry& r) override {
            r.add_dependency(dr_switch);
            r.add_dependency(dr_input_1);
            r.add_dependency(dr_input_2);
            r.add_dependency(dw_output);
        }
        void execute() override {
            if (dr_switch.read()) {
                dr_input_2.on_update([this](const double& data) {
                    dw_output.write(data);
                });
            } else {
                dr_input_1.on_update([this](const double& data) {
                    dw_output.write(data);
                });
            }            
        }
    private:
        // DataWriter<interface::MOTOR> motor[12];
        DataReader<bool> dr_switch{"cc_state"};
        DataReader<double> dr_input_1{"sm/crane_spd_sv"};
        DataReader<double> dr_input_2{"cc/crane_spd_sv"};
        DataWriter<double> dw_output{"crane_spd_sv"};
    };
};