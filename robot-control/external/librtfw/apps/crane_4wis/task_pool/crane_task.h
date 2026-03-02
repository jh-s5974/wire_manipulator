#pragma once

#include <rtfw/task.h>

using namespace rtfw;
using namespace rtfw::rt;

namespace task_pool {

    class CraneTask : public ITask {
    public:
        const char* getName() const override { return "CRANE_TASK"; }
        void setup(TaskRegistry& r) override {
            r.add_dependency(dw_state);
            r.add_dependency(dw_speed);
            r.add_dependency(dw_stop);
            r.add_dependency(dr_cmd_state);
            r.add_dependency(dr_pos_sv);
            r.add_dependency(dr_pos_pv);
        }
        void execute() override {
            
            dr_cmd_state.on_update([this](const bool& data) {
                if (data && !active) {
                    active = data;
                    getLogger()->info("[{}] state=on", getName());
                    // printf("crane controller: state=on\n");
                }
                if (!data && active) {
                    active = data;
                    busy = false;
                    getLogger()->info("[{}] state=off", getName());
                    // printf("crane controller: state=off\n");
                }
            });

            dr_pos_pv.on_update([this](const double& data) {
                pos_cache = data;
                if (!busy) return;

                auto error = pos_sv - pos_cache;
                if (std::abs(error) > 0.001) {
                    auto spd = error * 0.1;
                    if (spd > ref_speed) spd = ref_speed;
                    if (spd < -ref_speed) spd = -ref_speed;
                    dw_speed.write(spd);
                    PERIODIC_CALL(
                        getLogger()->info("[{}] current({:.04f} m) >> target({:.04f} m)", getName(), pos_cache, pos_sv);
                        // printf("crane control: current(%lf m) >> target(%lf m)\n", pos_cache, pos_sv)
                    , 1s);
                } else {
                    getLogger()->info("[{}] target position reached", getName());
                    // printf("crane control: target position reached\n");
                    dw_stop.write();
                    busy = false;
                }
            });

            
            dr_pos_sv.on_update([this](const double& data) {
                if (!active)
                return;

                getLogger()->info("[{}] target position={:.04f}", getName(), data);
                // printf("crane contrl: target position=%lf\n", data);
                pos_sv = data;
                busy = true;
            });

            dw_state.write(busy);
        }
    private:        
          DataWriter<bool> dw_state{"cc_state"};
          DataWriter<double> dw_speed{"cc/crane_spd_sv"};
          DataWriter<Signal> dw_stop{"crane_stop"};
          DataReader<bool> dr_cmd_state{"cmd_cc_state"};
          DataReader<double> dr_pos_sv{"crane_pos_sv"};
          DataReader<double> dr_pos_pv{"crane_pos_pv"};

    private:
        bool active = false;
        bool busy = false;
        double ref_speed = 10e-3;        // m/s
        // double acc_limit = 0.02;        // m/s^2
        // double jerk_limit = 0.05;        // m/s^3
        
        double pos_cache = 0;
        double pos_sv = 0;
    };

};