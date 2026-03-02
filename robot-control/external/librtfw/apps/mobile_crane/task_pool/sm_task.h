#pragma once

#include <rtfw/task.h>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <thread>

#include <eigen3/Eigen/Dense>

#include "util.hpp"
// #include "frame.hpp"
#include <manif/manif.h>

using namespace std::chrono_literals;
using namespace rtfw;
using namespace rtfw::rt;


/******************************************
 * State Machine
 * 
 * 1. State
 *   1) Manual: 
 *      - Joystick available      
 *   2) Tracking
 *      - Crane lock
 *      - Joystick lock
 *   3) Crane  
 *      - Wheel slow
 *   4) Recover
 *      - Wheel lock
 *      - Crane lock
 * 
 *  2. State-Transition:
 * 
 *        Crane <---> Manual <---> Tracking
 *          ^                          |
 *          ㄴ------ Recover <----------
 * 
 *  3. Safety
 *   1) slow move during Crane up
 *   2) while moving, Crane locked
 * 
*********************************************/


namespace task_pool {

    class StateMachine : public ITask {
    public:
        const char* getName() const override { return "StateMachine"; }
        void setup(TaskRegistry& r) override {
            wdt = {0};
            states = {0};

            r.add_dependency(dw_cmd_cc_state);
            r.add_dependency(dw_cmd_cv_state);
            r.add_dependency(dw_origin_reset);
            r.add_dependency(dw_height_sv);
            r.add_dependency(dw_vel_sv);
            r.add_dependency(dw_crane_spd_sv);
            r.add_dependency(dw_track_start);
            r.add_dependency(dw_track_stop);
            r.add_dependency(dr_ad_state);
            r.add_dependency(dr_tc_state);
            r.add_dependency(dr_cc_state);
            r.add_dependency(dr_wm_state);
            r.add_dependency(dr_cv_state);
            r.add_dependency(dr_js_state);
            r.add_dependency(dr_cmd_vx);
            r.add_dependency(dr_cmd_vy);
            r.add_dependency(dr_cmd_wz);
            r.add_dependency(dr_cmd_vc_up);
            r.add_dependency(dr_cmd_vc_down);
            r.add_dependency(dr_cmd_restore);
            r.add_dependency(dr_cmd_tracking_on);
            r.add_dependency(dr_cmd_tracking_off);
            r.add_dependency(dr_cmd_origin_reset);
            r.add_dependency(dr_cmd_soft_emg);
            r.add_dependency(dr_cmd_crane_teach);
            r.add_dependency(dr_cmd_vxy_reverse);
            r.add_dependency(dr_marker_detection);
            r.add_dependency(dr_height_pv);
            r.add_dependency(dr_abnormal);
            
        }
        void execute() override {

            dr_ad_state.on_update([this](const bool& data){
                if (states.ad != data) // printf("state machine: ad state changed(%s)\n", data? "true": "false");
                    getLogger()->info("[{}] ad state changed({})\n", getName(), data? "true": "false");
                states.ad = data;
                wdt.ad = 0;
            });
            dr_tc_state.on_update([this](const bool& data){
                if (states.tc != data) // printf("state machine: tc state changed(%s)\n", data? "true": "false");
                    getLogger()->info("[{}] tc state changed({})\n", getName(), data? "true": "false");
                states.tc = data;
                wdt.tc = 0;
            });
            dr_cc_state.on_update([this](const bool& data){
                if (states.cc != data) // printf("state machine: cc state changed(%s)\n", data? "true": "false");
                    getLogger()->info("[{}] cc state changed({})\n", getName(), data? "true": "false");
                states.cc = data;
                wdt.cc = 0;
            });
            dr_wm_state.on_update([this](const bool& data){
                if (states.wm != data) // printf("state machine: wm state changed(%s)\n", data? "true": "false");
                    getLogger()->info("[{}] wm state changed({})\n", getName(), data? "true": "false");
                states.wm = data;
                wdt.wm = 0;
            });
            dr_cv_state.on_update([this](const bool& data){
                if (states.cv != data) // printf("state machine: cv state changed(%s)\n", data? "true": "false");
                    getLogger()->info("[{}] cv state changed({})\n", getName(), data? "true": "false");
                states.cv = data;
                wdt.cv = 0;
            });
            dr_js_state.on_update([this](const bool& data){
                if (states.js != data) // printf("state machine: js state changed(%s)\n", data? "true": "false");
                    getLogger()->info("[{}] js state changed({})\n", getName(), data? "true": "false");
                states.js = data;
                wdt.js = 0;
            });
            
            dr_cmd_vx.on_update([this](const double& data){
                req.vx = -data;
                if (std::abs(req.vx) < 0.02) req.vx = 0;
                if (is_vxy_reverse)
                    req.vx = -req.vx;
            });
            dr_cmd_vy.on_update([this](const double& data){
                req.vy = -data;
                if (std::abs(req.vy) < 0.02) req.vy = 0;
                if (is_vxy_reverse)
                    req.vy = -req.vy;
            });
            dr_cmd_wz.on_update([this](const double& data){
                req.wz = -data;
                if (std::abs(req.wz) < 0.02) req.wz = 0;
            });
            dr_cmd_vc_up.on_update([this](const double& data){
                if (data < 0.2)
                    req.vc = (1 + data) * 0.5;
                if (data >= 0.2)
                req.vc = (1 + data) * 1;
                if (std::abs(req.vc) < 0.02) req.vc = 0;
            });
            dr_cmd_vc_down.on_update([this](const double& data){
                if (data < 0.8)
                    req.vc = -(1 + data) * 0.25;
                if (data >= 0.8)
                    req.vc = -(1 + data) * 0.5;
                if (std::abs(req.vc) < 0.02) req.vc = 0;
            });
            dr_cmd_restore.on_update([this](const double& data){
                if (data > 0) {
                    // Force Enter RESTORE MODE
                    if (elevation < 0)
                        // std::cout << "Error: Target Position=" << elevation<< std::endl;
                        getLogger()->error("[{}] Target Position={:.04f}", getName(), elevation);
                    else {
                        // std::cout << "Force RESTORE" << std::endl;
                        getLogger()->info("[{}] Force RESTORE", getName());
                        start_restore_process(elevation);
                    }
                }
            });
            
            dr_cmd_tracking_on.on_update([this](const bool& data) {
                if (!data) return;

                if (state == STATE::Manual) {
                // if (state == STATE::Manual || state == STATE::Crane) {
                    if (!states.cv) {
                        getLogger()->info("[{}] can't enter tracking mode(cv offline)", getName());
                        return;
                    }
                    if (marker_lost > 0) {
                        getLogger()->info("[{}] can't enter tracking mode(marker lost)", getName());
                        return;
                    }
                    if (elevation < 0) {
                        getLogger()->info("[{}] can't enter tracking mode(crane target not aligned)", getName());
                        return;
                    }
                    getLogger()->info("[{}] system enter tracking mode", getName());
                    state = STATE::Tracking;
                    sig_ad = false;
                    dw_track_start.write();
                }
            });            
            dr_cmd_tracking_off.on_update([this](const bool& data) {
                if (!data) return;
                
                dw_track_stop.write();
                if (state == STATE::Tracking) {
                    state = STATE::Manual;
                    getLogger()->info("[{}] system enter manual mode", getName());
                }
            });
            dr_cmd_origin_reset.on_update([this](const bool& data) {
                if (data && (state == STATE::Manual || state == STATE::Crane)) {    
                    dw_origin_reset.write();
                    elevation = -1;
                }
            });
            dr_cmd_soft_emg.on_update([this](const bool& data) {
                if (data) {
                    getLogger()->warn("[{}] ***emergency***", getName());

                    
                    state = STATE::Emergency;
                    dw_track_stop.write();
                    dw_cmd_cc_state.write(false);
                    // dw_cmd_cm_state.write(false);
                    dw_cmd_cv_state.write(false);

                    cmd.vx = req.vx = 0;
                    cmd.vy = req.vy = 0;
                    cmd.wz = req.wz = 0;
                    cmd.vc = req.vc = 0;
                } else {
                    getLogger()->warn("[{}] ***system start***", getName());
                    state = STATE::Manual;

                    // dw_cmd_cv_state.write(true);
                }
            });
            dr_cmd_crane_teach.on_update([this](const bool& data) {
                if (data) {
                    elevation = crane_pos;
                    getLogger()->info("[{}] set elevation height={:.04f}", getName(), elevation);
                } else {

                } 
            });
            dr_cmd_vxy_reverse.on_update([this](const bool& data) {
                if (data) {
                    is_vxy_reverse = true;
                    getLogger()->info("[{}] vxy reverse on", getName());
                } else {
                    is_vxy_reverse = false;
                    getLogger()->info("[{}] vxy reverse off", getName());
                } 
            });
            dr_marker_detection.on_update([this](const bool& data) {
                wdt.cv = 0;
                if (data) {
                    if (marker_lost > 0)
                        getLogger()->info("[{}] marker tracking restore", getName());
                    marker_lost = 0;
                } else {
                    if (marker_lost == 0)
                        getLogger()->info("[{}] marker tracking lost", getName());

                    if (marker_lost < marker_lost_max) {
                        marker_lost++;
                        if (state == STATE::Tracking)
                            getLogger()->info("[{}] marker lost={}", getName(), marker_lost);
                    }
                }
            });
            dr_height_pv.on_update([this](const double& data) {
                crane_pos = data;
            });
            dr_abnormal.on_update([this](const bool& data) {
                if (data) {
                    if (state == STATE::Tracking) {
                        getLogger()->info("[{}] abnormal detected", getName());
                        dw_track_stop.write();
                        sig_ad = true;
                    }
                } else {

                }
            });
            

            // if (getCurrentTick() == 0) {
            //     dw_track_stop.write();
            //     dw_cmd_cc_state.write(false);
            //     // dw_cmd_cm_state.write(true);
            //     dw_cmd_wm_state.write(true);
            //     dw_cmd_cv_state.write(false);
            //     dw_cmd_js_state.write(true);
            // }

            {
                watchdog_task();               

                if (!states.js) {
                    PERIODIC_CALL(
                        getLogger()->warn("[{}] Joystick not connected", getName());
                    , 3s);

                    if (state == STATE::Manual || state == STATE::Crane) {
                        cmd.vx = req.vx = 0;
                        cmd.vy = req.vy = 0;
                        cmd.wz = req.wz = 0;
                        cmd.vc = req.vc = 0;
                        state = STATE::Idle;
                        // dw_track_stop.write();
                        // dw_cmd_cc_state.write(false);
                        // // dw_cmd_cm_state.write(true);
                        // dw_cmd_wm_state.write(true);
                        // // dw_cmd_cv_state.write(false);
                        // dw_cmd_js_state.write(true);
                    }

                } else {
                    if (state == STATE::Idle) {
                    getLogger()->info("[{}] state initialize", getName());
                    state = STATE::Manual;

                    // dw_cmd_cm_state.write(true);
                    // dw_cmd_cv_state.write(false);


                    // Debug!
                    // dw_cmd_ad_state.write(true);

                    }
                }

                double factor = 0.95;
                cmd.vx = req.vx * (1-factor) + cmd.vx * factor;
                cmd.vy = req.vy * (1-factor) + cmd.vy * factor;
                cmd.wz = req.wz * (1-factor) + cmd.wz * factor;
                cmd.vc = req.vc * (1-factor) + cmd.vc * factor;

                if (std::abs(cmd.vx) < 0.01 && std::abs(req.vx) < 0.01) cmd.vx = 0;
                if (std::abs(cmd.vy) < 0.01 && std::abs(req.vy) < 0.01) cmd.vy = 0;
                if (std::abs(cmd.wz) < 0.01 && std::abs(req.wz) < 0.01) cmd.wz = 0;
                if (std::abs(cmd.vc) < 0.01 && std::abs(req.vc) < 0.01) cmd.vc = 0;

                switch (state) {
                    case STATE::Manual: {
                        manif::SE3Tangentd msg_mvel;
                        msg_mvel.lin().x() = v_max * cmd.vx;
                        msg_mvel.lin().y() = v_max * cmd.vy;
                        msg_mvel.ang().z() = w_max * cmd.wz;
                        dw_vel_sv.write(msg_mvel);

                        double msg_cvel;
                        msg_cvel = vc_max * cmd.vc;
                        dw_crane_spd_sv.write(msg_cvel);

                        PERIODIC_CALL(
                            getLogger()->info("[{}] MANUAL: vx={:.02f}, vy={:.02f}, wz={:.02f}, vc={:.04f}, h={:.04f}", getName(),
                                msg_mvel.lin().x(), msg_mvel.lin().y(), msg_mvel.ang().z(), msg_cvel, crane_pos);
                        , 1s);

                        if (crane_pos > 0) {
                            state = STATE::Crane;
                            getLogger()->info("[{}] system enter crane mode", getName());
                        }


            // // debug!!!
            // last = this->get_clock()->now();
            // crane_pos = -1;

                        break;
                    }
                    case STATE::Crane: {
                        manif::SE3Tangentd msg_mvel;
                        msg_mvel.lin().x() = slow_factor * v_max * cmd.vx;
                        msg_mvel.lin().y() = slow_factor * v_max * cmd.vy;
                        msg_mvel.ang().z() = slow_factor * w_max * cmd.wz;
                        dw_vel_sv.write(msg_mvel);

                        double msg_cvel;
                        msg_cvel = vc_max * cmd.vc;
                        dw_crane_spd_sv.write(msg_cvel);

                        PERIODIC_CALL(
                            getLogger()->info("[{}] CRANE: vx={:.02f}, vy={:.02f}, wz={:.02f}, vc={:.04f}, h={:.04f}", getName(),
                            msg_mvel.lin().x(), msg_mvel.lin().y(), msg_mvel.ang().z(), msg_cvel, crane_pos);
                            // printf("CRANE: vx=%lf, vy=%lf, wz=%lf, vc=%f, h=%lf\n", 
                            // msg_mvel.lin().x(), msg_mvel.lin().y(), msg_mvel.ang().z(), msg_cvel, crane_pos);
                        , 1s);

            // // debug!!
            // crane_pos += vc_max * vc * 0.001;
                        if (crane_pos <= 0) {
                            state = STATE::Manual;
                            getLogger()->info("[{}] system enter manual mode", getName());
                        }
                        
                        break;
                    }
                    case STATE::Tracking: {
                        bool recover = false;
                        if (sig_ad) {
                            getLogger()->info("[{}] state machine: abnormal detected", getName());
                            recover = true;
                        }
                        if (marker_lost >= marker_lost_max) {
                            getLogger()->info("[{}] state machine: marker lost", getName());
                            recover = true;
                        }
                        if (!states.cv) {
                            getLogger()->info("[{}] state machine: cv state bad", getName());
                            recover = true;
                        }
                        
                        if(!states.js) {
                            getLogger()->info("[{}] state machine: js state bad", getName());
                            recover = true;                            
                        }
                        // recover = false;
                        if (recover) {
                            start_restore_process(elevation);                
                        }
                        
                        double msg_cvel;
                        msg_cvel = vc_max * cmd.vc;
                        dw_crane_spd_sv.write(msg_cvel);
                        break;
                    }
                    case STATE::Recover: {
            // // debug!!!
            // crane_pos += 0.001;
                        if (!states.cc) {
                            state = STATE::Crane;
                            getLogger()->info("[{}] system enter crane mode", getName());
                            dw_cmd_cc_state.write(false);
                        } else {

                            PERIODIC_CALL(
                                getLogger()->info("[{}] RECOVER: height={:.04f} -> {:.04f}", getName(), crane_pos, elevation);
                            , 1s);
                        }
                        break;
                    }
                    default: {
                        manif::SE3Tangentd msg_mvel;
                        msg_mvel.lin().x() = 0;
                        msg_mvel.lin().y() = 0;
                        msg_mvel.ang().z() = 0;
                        dw_vel_sv.write(msg_mvel);

                        double msg_cvel;
                        msg_cvel = 0;
                        dw_crane_spd_sv.write(msg_cvel);

                        break;
                    }
                }
            }
        }


        void watchdog_task() {
            if (++wdt.ad > 100) {
                PERIODIC_CALL(getLogger()->warn("[{}] abnormal detection no signal", getName()), 10s);
                wdt.ad = 0;
                states.ad = false;
            }
            if (++wdt.cc > 100) {
                PERIODIC_CALL(getLogger()->warn("[{}] crane control no signal", getName()), 10s);
                wdt.cc = 0;
                states.cc = false;
            }
            if (++wdt.cm > 100) {
                PERIODIC_CALL(getLogger()->warn("[{}] crane motor no signal", getName()), 10s);
                wdt.cm = 0;
                states.cm = false;
            }
            if (++wdt.tc > 100) {
                PERIODIC_CALL(getLogger()->warn("[{}] tracking control no signal", getName()), 10s);
                wdt.tc = 0;
                states.tc = false;
            }
            if (++wdt.wm > 100) {
                PERIODIC_CALL(getLogger()->warn("[{}] wheel motor no signal", getName()), 10s);
                wdt.wm = 0;
                states.wm = false;
            }
            if (++wdt.cv > 100) {
                PERIODIC_CALL(getLogger()->warn("[{}] vision no signal", getName()), 10s);
                wdt.cv = 0;
                states.cv = false;
            }
            if (++wdt.js > 100) {
                PERIODIC_CALL(getLogger()->warn("[{}] joystick no signal", getName()), 10s);
                wdt.js = 0;
                states.js = false;
            }
        }
        
        void start_restore_process(double elevation) {
            manif::SE3Tangentd msg_mvel;
            msg_mvel.lin().x() = 0;
            msg_mvel.lin().y() = 0;
            msg_mvel.ang().z() = 0;
            dw_vel_sv.write(msg_mvel);
            dw_track_stop.write();

            double msg = elevation;
            dw_cmd_cc_state.write(true);
            states.cc = true;
            dw_height_sv.write(msg);
            getLogger()->info("[{}] system enter recovery mode (to {:.04f} m)", getName(), elevation);
            state = STATE::Recover;
            // signal.tx.cmd_wm_state.send(false);
        }

    private:
        DataWriter<Signal> dw_track_start{"tracking_start"};
        DataWriter<Signal> dw_track_stop{"tracking_stop"};
        DataWriter<bool> dw_cmd_cc_state{"cmd_cc_state"};
        DataWriter<bool> dw_cmd_cv_state{"cmd_cv_state"};
        DataWriter<Signal> dw_origin_reset{"crane_pos_reset"};
        DataWriter<double> dw_height_sv{"crane_pos_sv"};
        DataWriter<manif::SE3Tangentd> dw_vel_sv{"sm/velocity_sv"};
        DataWriter<double> dw_crane_spd_sv{"sm/crane_spd_sv"};

        DataReader<bool> dr_ad_state{"ad_state", DependencyType::Weak};
        DataReader<bool> dr_tc_state{"tc_state", DependencyType::Weak};
        DataReader<bool> dr_cc_state{"cc_state", DependencyType::Weak};
        DataReader<bool> dr_wm_state{"wm_state", DependencyType::Weak};
        DataReader<bool> dr_cv_state{"cv_state", DependencyType::Weak};
        DataReader<bool> dr_js_state{"js_state", DependencyType::Weak};

        DataReader<double> dr_cmd_vx{"js/axis_1"};
        DataReader<double> dr_cmd_vy{"js/axis_0"};
        DataReader<double> dr_cmd_wz{"js/axis_2"};
        DataReader<double> dr_cmd_vc_up{"js/axis_4"};
        DataReader<double> dr_cmd_vc_down{"js/axis_5"};
        DataReader<double> dr_cmd_restore{"js/axis_6"};
        DataReader<bool> dr_cmd_tracking_on{"js/btn_0"};
        DataReader<bool> dr_cmd_tracking_off{"js/btn_1"};
        DataReader<bool> dr_cmd_origin_reset{"js/btn_3"};
        DataReader<bool> dr_cmd_soft_emg{"js/btn_4"};
        DataReader<bool> dr_cmd_crane_teach{"js/btn_6"};
        DataReader<bool> dr_cmd_vxy_reverse{"js/btn_7"};
        DataReader<bool> dr_marker_detection{"marker_detection"};
        DataReader<double> dr_height_pv{"crane_pos_pv"};
        DataReader<bool> dr_abnormal{"abnormal", DependencyType::Weak};
    private:
        enum STATE {
        Idle,
        Manual,
        Crane,
        Tracking,
        Recover,
        Emergency,
        } state = STATE::Idle;

        struct {
            int ad; // abnormal detector
            int tc; // tracking control
            int cc; // crane control
            int wm; // wheel motor
            int cm; // crane motor
            int cv; // computer vision
            int mk; // mecanum kinematics
            int js; // joystick
        } wdt, states;

        double slow_factor = 0.2;
        double v_max = 0.63*0.2;
        double w_max = 0.75*0.2;
        double vc_max = 0.002;
        double elevation = -1;

    private:
        bool active;
        bool run = true;
        std::chrono::steady_clock::time_point last;

        double crane_pos = 0;
        int marker_lost = 0;
        const int marker_lost_max = 20;
        bool sig_ad = false;
        bool is_vxy_reverse = false;
        
        struct {
        double vx;
        double vy;
        double wz;
        double vc;
        } req, cmd;
    };
};