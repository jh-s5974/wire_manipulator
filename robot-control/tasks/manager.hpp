#pragma once

#include <rtfw/task.h>
#include "util.hpp"

#include "custom_types.hpp"
#include <manif/manif.h>
#include <Eigen/Dense>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <queue>
#include <iostream>
#include <array>

#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <unistd.h>
#include <linux/joystick.h>
#include <thread>


using namespace std::chrono_literals;
using namespace rtfw;
using namespace rtfw::rt;

namespace task_pool {

    enum SafetyLevel {
        SAFETY_ESSENTIAL = 0,
        SAFETY_STRICT = 1,
    };

    
    enum State {
        IDLE,
        MANU,
        READY,
        RL_WALK,        
    };
    
    struct ManagerState {
        State state_, prev_state_;

        Eigen::Vector3d cmd_vel;
        bool gui_control_requested = false;
        bool gui_control_request_event = false;
        bool gui_control_request_value = false;
        bool gui_control_granted = false;
        int gui_mode_request = -1;
        bool gui_mode_request_online = false;
        std::array<custom_types::MotorCmd, 12> gui_motor_cmd{};
        std::array<bool, 12> gui_motor_cmd_online{};
        std::array<bool, 12> gui_motor_on{};
        std::array<bool, 12> gui_motor_on_online{};

        struct MotorInterpState {
            bool active = false;
            double start_ms = 0.0;
            double duration_ms = 0.0;
            double start_pos = 0.0;
            custom_types::MotorCmd target_cmd{};
        };
        std::array<MotorInterpState, 12> gui_motor_interp{};
        std::array<bool, 12> gui_motor_cmd_new{};

        struct {
            uint64_t motor[12];
            uint64_t imu;
            uint64_t joystick;
        } wdt;

        struct {
            custom_types::MotorState motor[12];
            Eigen::Quaterniond base_quat;
            uint64_t tick_ms;
            struct {
                bool motor[12];
                bool imu;
                bool joystick;
            } online;
            bool req_rl_control;
            bool walk_ready;
        } robot_state;
    };

    std::unordered_map<int, int> joint_id_mapping;

    class Manager : public Task<ManagerState> {
    public:
        const char* getName() const override { return "Manager"; }

        void initialize(ManagerState& s) override {
            s.state_ = State::IDLE;
            s.prev_state_ = State::IDLE;
            s.cmd_vel.setZero();
            s.gui_control_requested = false;
            s.gui_control_request_event = false;
            s.gui_control_request_value = false;
            s.gui_control_granted = false;
            s.gui_mode_request = -1;
            s.gui_mode_request_online = false;
            s.gui_motor_cmd_online.fill(false);
            s.gui_motor_on_online.fill(false);
            for (auto i = 0; i < 12; i++) s.gui_motor_interp[i] = {};
            s.gui_motor_cmd_new.fill(false);
            s.gui_motor_on.fill(true);


            for (auto i=0; i<12; i++) {
                // wdt.motor[i] = 0;
                s.robot_state.motor[i].pos = 0.0;
                s.robot_state.motor[i].vel = 0.0;
                s.robot_state.motor[i].torque = 0.0;
            }
            s.robot_state.tick_ms = 0;
            s.robot_state.req_rl_control = false;
            s.robot_state.walk_ready = false;
        }

        void execute(ManagerState& s) override {

            for (auto i = 0; i < 12; i++) s.gui_motor_cmd_new[i] = false;

            for (auto i=0; i<12; i++) {
                dr_mtr_stat[i].on_update([&, i](const custom_types::MotorState& data) {
                        s.wdt.motor[i] = getCurrentTick();
                        s.robot_state.motor[i].pos = data.pos;
                        s.robot_state.motor[i].vel = data.vel;
                        s.robot_state.motor[i].torque = data.torque;
                        s.robot_state.motor[i].status = data.status;
                });
            }

            dr_imu.on_update([&](const custom_types::Imu& data) {
                s.wdt.imu = getCurrentTick();
                s.robot_state.base_quat = Eigen::Quaterniond(data.orientation.w, data.orientation.x, data.orientation.y, data.orientation.z);
            });

            dr_gui_control_request.on_update([&](const bool& req) {
                s.gui_control_request_value = req;
                s.gui_control_request_event = true;
                getLogger()->info("[{}] GUI control request updated: {}", getName(), req ? "true" : "false");
            });

            dr_gui_mode_request.on_update([&](const int& mode) {
                s.gui_mode_request = mode;
                s.gui_mode_request_online = true;
            });

            
            auto& joint_kp = p_joint_kp.read();
            auto& joint_kd = p_joint_kd.read();

            for (auto i=0; i<12; i++) {
                dr_gui_mtr_cmd[i].on_update([&, i](const custom_types::MotorCmd& data) {
                    s.gui_motor_cmd[i] = data;
                    s.gui_motor_cmd_online[i] = true;
                    s.gui_motor_cmd_new[i] = true;
                });

                dr_gui_motor_on[i].on_update([&, i](const bool& on) {
                    s.gui_motor_on[i] = on;
                    s.gui_motor_on_online[i] = true;

                    if (on && s.gui_control_granted) {
                        // Initialize to safe cmd when turning motor on: hold current position with kp=0, kd=30
                        custom_types::MotorCmd safe_cmd{};
                        safe_cmd.pos = s.robot_state.motor[i].pos;
                        safe_cmd.vel = 0.0;
                        safe_cmd.torque = 0.0;
                        // safe_cmd.kp = 0.0;
                        safe_cmd.kp = joint_kp[i];
                        // safe_cmd.kd = joint_kd[i];
                        safe_cmd.kd = 3.0;
                        dw_mtr_cmd[i].write(safe_cmd);
                    }
                });
            }


            auto vx = -dr_joy_vx.read();
            auto vy = -dr_joy_vy.read();
            auto wz = -dr_joy_wz.read();
            if (std::abs(vx) < 1e-1) vx = 0.0;
            if (std::abs(vy) < 1e-1) vy = 0.0;
            if (std::abs(wz) < 1e-1) wz = 0.0;

            const double gain = joy_gain.read();
            s.cmd_vel << s.cmd_vel.x()*gain + vx*joy_vx_scale.read()*(1.0 - gain),
                         s.cmd_vel.y()*gain + vy*joy_vy_scale.read()*(1.0 - gain),
                         s.cmd_vel.z()*gain + wz*joy_wz_scale.read()*(1.0 - gain);


            // dr_joy_X.on_update([&](const bool& data) {
            
            // });
            // dr_joy_Y.on_update([&](const bool& data) {
            
            // });

            dr_joy_A.on_update([&](const bool& data) {                
                if (data == 1 && s.state_ == State::READY){
                    s.robot_state.req_rl_control = true;
                }
            });
            dr_joy_B.on_update([&](const bool& data) {
                if (data == 1 && s.state_ == State::RL_WALK){
                    s.robot_state.req_rl_control = false;
                }
            });

            
            wdt_process(s);
            process_gui_command_bridge(s);
            process_gui_mode_request(s);
            
            switch (s.state_) {
                case State::IDLE: // initial step
                {
                    // motor check
                    // imu check
                    // robot state initialization
                    task_initial(s);
                    break;
                }
                case State::MANU: // neutral step
                {
                    // leg stretching, high stiffness, low damping
                    // joint control
                    task_neutral(s);
                    break;
                }
                case State::READY: // ready step
                {
                    // walking ready, low stiffness, high damping
                    // joint control -> RL control transition
                    task_ready(s);
                    break;
                }
                case State::RL_WALK: // RL control step
                {
                    // cmd_vel에 따라 RL 제어 명령 발행
                    task_rl_control(s);
                    break;
                }
                default:
                    getLogger()->warn("Unknown state!");
                    break;
            }

            dw_gui_mode_current.write(gui_mode_from_state(s.state_));
            dw_safety_level.write(safety_level_from_state(s.state_));

            s.robot_state.tick_ms = getExecutionLocalTick()*1000.0/getFrequency();
            PERIODIC_CALL(
                getLogger()->info("[{}] Mode={}", getName(), robot_mode_to_string(s.state_));
            , 5s);
        }    
    private:

    DataReader<double> dr_joy_vx{"js/axis_1"};
    DataReader<double> dr_joy_vy{"js/axis_0"};
    DataReader<double> dr_joy_wz{"js/axis_2"};

    DataReader<bool> dr_joy_A{"js/btn_0"};
    DataReader<bool> dr_joy_B{"js/btn_1"};
    DataReader<bool> dr_joy_X{"js/btn_3"};
    DataReader<bool> dr_joy_Y{"js/btn_4"};
    DataReader<bool> dr_gui_control_request{"gui/motor/control_request", DependencyType::Weak};
    DataReader<int> dr_gui_mode_request{"gui/robot/mode_request", DependencyType::Weak};

    DataReader<custom_types::MotorCmd> dr_gui_mtr_cmd[12] = {
        DataReader<custom_types::MotorCmd>{"gui/hip_yaw_left/cmd", DependencyType::Weak},
        DataReader<custom_types::MotorCmd>{"gui/hip_yaw_right/cmd", DependencyType::Weak},
        DataReader<custom_types::MotorCmd>{"gui/hip_roll_left/cmd", DependencyType::Weak},
        DataReader<custom_types::MotorCmd>{"gui/hip_roll_right/cmd", DependencyType::Weak},
        DataReader<custom_types::MotorCmd>{"gui/hip_pitch_left/cmd", DependencyType::Weak},
        DataReader<custom_types::MotorCmd>{"gui/hip_pitch_right/cmd", DependencyType::Weak},
        DataReader<custom_types::MotorCmd>{"gui/knee_left/cmd", DependencyType::Weak},
        DataReader<custom_types::MotorCmd>{"gui/knee_right/cmd", DependencyType::Weak},
        DataReader<custom_types::MotorCmd>{"gui/ankle_pitch_left/cmd", DependencyType::Weak},
        DataReader<custom_types::MotorCmd>{"gui/ankle_pitch_right/cmd", DependencyType::Weak},
        DataReader<custom_types::MotorCmd>{"gui/ankle_roll_left/cmd", DependencyType::Weak},
        DataReader<custom_types::MotorCmd>{"gui/ankle_roll_right/cmd", DependencyType::Weak},
    };

    DataReader<bool> dr_gui_motor_on[12] = {
        DataReader<bool>{"gui/hip_yaw_left/on", DependencyType::Weak},
        DataReader<bool>{"gui/hip_yaw_right/on", DependencyType::Weak},
        DataReader<bool>{"gui/hip_roll_left/on", DependencyType::Weak},
        DataReader<bool>{"gui/hip_roll_right/on", DependencyType::Weak},
        DataReader<bool>{"gui/hip_pitch_left/on", DependencyType::Weak},
        DataReader<bool>{"gui/hip_pitch_right/on", DependencyType::Weak},
        DataReader<bool>{"gui/knee_left/on", DependencyType::Weak},
        DataReader<bool>{"gui/knee_right/on", DependencyType::Weak},
        DataReader<bool>{"gui/ankle_pitch_left/on", DependencyType::Weak},
        DataReader<bool>{"gui/ankle_pitch_right/on", DependencyType::Weak},
        DataReader<bool>{"gui/ankle_roll_left/on", DependencyType::Weak},
        DataReader<bool>{"gui/ankle_roll_right/on", DependencyType::Weak},
    };

    DataReader<custom_types::MotorState> dr_mtr_stat[12] = {
        DataReader<custom_types::MotorState>{"hip_yaw_left/state", DependencyType::Weak},
        DataReader<custom_types::MotorState>{"hip_yaw_right/state", DependencyType::Weak},
        DataReader<custom_types::MotorState>{"hip_roll_left/state", DependencyType::Weak},
        DataReader<custom_types::MotorState>{"hip_roll_right/state", DependencyType::Weak},
        DataReader<custom_types::MotorState>{"hip_pitch_left/state", DependencyType::Weak},
        DataReader<custom_types::MotorState>{"hip_pitch_right/state", DependencyType::Weak},
        DataReader<custom_types::MotorState>{"knee_left/state", DependencyType::Weak},
        DataReader<custom_types::MotorState>{"knee_right/state", DependencyType::Weak},
        DataReader<custom_types::MotorState>{"ankle_pitch_left/state", DependencyType::Weak},
        DataReader<custom_types::MotorState>{"ankle_pitch_right/state", DependencyType::Weak},
        DataReader<custom_types::MotorState>{"ankle_roll_left/state", DependencyType::Weak},
        DataReader<custom_types::MotorState>{"ankle_roll_right/state", DependencyType::Weak},
    };
    DataReader<custom_types::Imu> dr_imu{"imu_data", DependencyType::Weak};

    DataWriter<custom_types::MotorCmd> dw_mtr_cmd[12] = {
        DataWriter<custom_types::MotorCmd>{"manager/hip_yaw_left/cmd"},
        DataWriter<custom_types::MotorCmd>{"manager/hip_yaw_right/cmd"},
        DataWriter<custom_types::MotorCmd>{"manager/hip_roll_left/cmd"},
        DataWriter<custom_types::MotorCmd>{"manager/hip_roll_right/cmd"},
        DataWriter<custom_types::MotorCmd>{"manager/hip_pitch_left/cmd"},
        DataWriter<custom_types::MotorCmd>{"manager/hip_pitch_right/cmd"},
        DataWriter<custom_types::MotorCmd>{"manager/knee_left/cmd"},
        DataWriter<custom_types::MotorCmd>{"manager/knee_right/cmd"},
        DataWriter<custom_types::MotorCmd>{"manager/ankle_pitch_left/cmd"},
        DataWriter<custom_types::MotorCmd>{"manager/ankle_pitch_right/cmd"},
        DataWriter<custom_types::MotorCmd>{"manager/ankle_roll_left/cmd"},
        DataWriter<custom_types::MotorCmd>{"manager/ankle_roll_right/cmd"},
    };
    DataWriter<bool> dw_motor_on[12] = {
        DataWriter<bool>{"hip_yaw_left/on"},
        DataWriter<bool>{"hip_yaw_right/on"},
        DataWriter<bool>{"hip_roll_left/on"},
        DataWriter<bool>{"hip_roll_right/on"},
        DataWriter<bool>{"hip_pitch_left/on"},
        DataWriter<bool>{"hip_pitch_right/on"},
        DataWriter<bool>{"knee_left/on"},
        DataWriter<bool>{"knee_right/on"},
        DataWriter<bool>{"ankle_pitch_left/on"},
        DataWriter<bool>{"ankle_pitch_right/on"},
        DataWriter<bool>{"ankle_roll_left/on"},
        DataWriter<bool>{"ankle_roll_right/on"},
    };
    DataWriter<bool> dw_gui_control_requested{"gui/motor/control_requested"};
    DataWriter<bool> dw_gui_control_granted{"gui/motor/control_granted"};
    DataWriter<int> dw_gui_mode_current{"gui/robot/mode_current"};
    DataWriter<int> dw_safety_level{"manager/safety_level"};
    DataWriter<manif::SE2Tangentd> dw_cmd_vel{"manager/cmd_vel"};
    DataWriter<bool> dw_rl_signal{"manager/rl_signal"};


    Parameter<std::vector<std::string>> p_joint_names{"joints", std::vector<std::string>()};
    Parameter<std::vector<double>> p_default_joint_positions{"default_joint_positions"};
    Parameter<std::vector<double>> p_joint_zero_positions{"joint_zero_positions"};
    Parameter<std::vector<double>> p_joint_kp{"joint_kp"};
    Parameter<std::vector<double>> p_joint_kd{"joint_kd"};
    Parameter<double> joy_vx_scale{"joy.vx_scale", 1.0};
    Parameter<double> joy_vy_scale{"joy.vy_scale", 1.0};
    Parameter<double> joy_wz_scale{"joy.wz_scale", 1.0};
    Parameter<double> joy_gain{"joy.gain", 0.99};

        

        // DataWriter<bool> dw_btn[12] = {
        //     DataWriter<bool>{"js/btn_0", ArchiveOption::Enable}, // A
        //     DataWriter<bool>{"js/btn_1", ArchiveOption::Enable}, // B
        //     DataWriter<bool>{"js/btn_2", ArchiveOption::Enable},
        //     DataWriter<bool>{"js/btn_3", ArchiveOption::Enable}, // X
        //     DataWriter<bool>{"js/btn_4", ArchiveOption::Disable}, // Y, emg btn
        //     DataWriter<bool>{"js/btn_5", ArchiveOption::Enable},
        //     DataWriter<bool>{"js/btn_6", ArchiveOption::Enable},
        //     DataWriter<bool>{"js/btn_7", ArchiveOption::Enable},
        //     DataWriter<bool>{"js/btn_8", ArchiveOption::Enable},
        //     DataWriter<bool>{"js/btn_9", ArchiveOption::Enable},
        //     DataWriter<bool>{"js/btn_10", ArchiveOption::Enable},
        //     DataWriter<bool>{"js/btn_11", ArchiveOption::Enable},
        // };
    private:

        void task_initial(ManagerState& s) {
            auto& joint_names = p_joint_names.read();
            bool succeed = true;
            for (auto i=0; i<joint_names.size(); i++) {
                if (!s.robot_state.online.motor[i]) {
                    PERIODIC_CALL(
                        getLogger()->warn("Motor(%d) %s is offline!", i, joint_names[i].c_str());
                    , 1s);
                    succeed = false;
                }
            }
            if (!s.robot_state.online.imu) {
                PERIODIC_CALL(
                    getLogger()->warn("IMU is offline!");
                , 1s);
                succeed = false;
            }
            if (!s.robot_state.online.joystick) {
                PERIODIC_CALL(
                    getLogger()->warn("Joystick is offline!");
                , 1s);
                succeed = false;
            }

            if (succeed) {
                getLogger()->info("All components are online. Transitioning to NEUTRAL state.");
                state_commit(s, State::MANU);
            }
            else {
                PERIODIC_CALL(
                    getLogger()->warn("Waiting for all components to be online...");
                , 1s);
                state_commit(s, State::IDLE);
            }   
        }

        void task_neutral(ManagerState& s) {

            
            dw_rl_signal.write(false); // RL 제어 시작 신호 (false)
            s.robot_state.walk_ready = false;
        }

        void task_ready(ManagerState& s) {
            auto& joint_names = p_joint_names.read();

            dw_cmd_vel.write(manif::SE2Tangentd(0, 0, 0));



            static int start_tick;
            static std::vector<double> init_q;
            if (s.state_ != s.prev_state_) {
                s.robot_state.walk_ready = false;
                s.robot_state.req_rl_control = false;
                dw_rl_signal.write(false); // RL 제어 시작 신호 (false)
                getLogger()->info("[{}] READY state. Initializing joint positions...", getName());
                start_tick = s.robot_state.tick_ms;
                init_q.clear();
                for (auto i=0; i<joint_names.size(); i++) {
                    init_q.push_back(s.robot_state.motor[i].pos);
                }
            }

            auto& joint_target_positions = p_default_joint_positions.read();
            auto& joint_kp = p_joint_kp.read();
            auto& joint_kd = p_joint_kd.read();

            double duration_ms = 3000.0; // 3 seconds
            double ratio = (s.robot_state.tick_ms - start_tick) / duration_ms;
            ratio = std::min(std::max(ratio, 0.0), 1.0); // Clamp ratio to [0, 1]
            for (auto i=0; i<12; i++) {
                custom_types::MotorCmd mtr_cmd;
                strncpy(mtr_cmd.name, joint_names[i].c_str(), joint_names[i].size()+1);
                mtr_cmd.pos = init_q[i] * (1-ratio) + joint_target_positions[i]*0.3 * ratio;
                mtr_cmd.vel = 0.0;
                mtr_cmd.torque = 0.0;
                mtr_cmd.kp = joint_kp[i];
                mtr_cmd.kd = joint_kd[i];

                dw_mtr_cmd[i].write(mtr_cmd);
            }

            // double max_err = 0;
            // for (auto i=0; i<cfg_.joint_names.size(); i++) {
            //     double err = std::abs(s.robot_state.q[i] - joint_zero_positions[i]);
            //     if (err > max_err) {
            //         max_err = err;
            //     }
            // }

            // if (s.robot_state.tick_ms - start_tick > duration_ms && max_err < 0.1) { // 3 second delay
            if (s.robot_state.tick_ms - start_tick > duration_ms) { // 3 second delay
                // getLogger()->info("Transitioning to READY state.");
                // state_commit(s, State::READY);
                s.robot_state.walk_ready = true;
            } else {
                s.robot_state.walk_ready = false;
            }

            if (s.robot_state.walk_ready) {
                if (s.robot_state.req_rl_control) {
                    getLogger()->info("Mode changed to: [Reinforcement Learning]");
                    state_commit(s, State::RL_WALK);
                    return;
                }
                

            }


            
            state_commit(s, State::READY);
        }

        void task_rl_control(ManagerState& s) {

            // s.cmd_vel.setZero();
            dw_cmd_vel.write(manif::SE2Tangentd(s.cmd_vel[0], s.cmd_vel[1], s.cmd_vel[2]));
            
            dw_rl_signal.write(true); // RL 제어 시작 신호 (true)

            
            state_commit(s, State::RL_WALK);
        }

        void state_commit(ManagerState& s, State new_state) {
            s.prev_state_ = s.state_;
            s.state_ = new_state;
        }

        void process_gui_command_bridge(ManagerState& s) {
            if (s.gui_control_granted && s.state_ != State::MANU) {
                s.gui_control_granted = false;
                for (int i = 0; i < 12; ++i) s.gui_motor_interp[i].active = false;
                getLogger()->info("GUI motor control REVOKED (left MANU state)");
            }

            bool requested_this_cycle = false;
            if (s.gui_control_request_event) {
                s.gui_control_request_event = false;
                requested_this_cycle = s.gui_control_request_value;

                if (s.gui_control_request_value) {
                    if (s.state_ == State::MANU) {
                        if (!s.gui_control_granted) {
                            s.gui_control_granted = true;
                            getLogger()->info("GUI motor control GRANTED (state=NEUTRAL)");
                        }
                    } else {
                        s.gui_control_granted = false;
                        getLogger()->warn("GUI motor control request REJECTED (state={})",
                                          s.state_ == State::IDLE ? "INITIAL" :
                                          s.state_ == State::READY ? "READY" : "RL_CONTROL");
                    }
                } else {
                    if (s.gui_control_granted) {
                        s.gui_control_granted = false;
                        getLogger()->info("GUI motor control RELEASED by request");
                    }
                }
            }

            s.gui_control_requested = requested_this_cycle;

            dw_gui_control_requested.write(s.gui_control_requested);
            dw_gui_control_granted.write(s.gui_control_granted);

            if (!s.gui_control_granted) {
                return;
            }

            for (int i = 0; i < 12; ++i) {
                if (s.gui_motor_cmd_new[i]) {
                    s.gui_motor_cmd_new[i] = false;
                    const auto& new_cmd = s.gui_motor_cmd[i];
                    if (new_cmd.duration_ms > 0.0) {
                        auto& interp = s.gui_motor_interp[i];
                        interp.active = true;
                        interp.start_ms = getExecutionLocalTick() * 1000.0 / getFrequency();
                        interp.duration_ms = new_cmd.duration_ms;
                        interp.start_pos = s.robot_state.motor[i].pos;
                        interp.target_cmd = new_cmd;
                    } else {
                        s.gui_motor_interp[i].active = false;
                    }
                }

                auto& interp = s.gui_motor_interp[i];
                if (interp.active) {
                    double current_ms = getExecutionLocalTick() * 1000.0 / getFrequency();
                    double elapsed = current_ms - interp.start_ms;
                    double ratio = std::min(elapsed / interp.duration_ms, 1.0);
                    custom_types::MotorCmd cmd = interp.target_cmd;
                    cmd.pos = interp.start_pos + ratio * (interp.target_cmd.pos - interp.start_pos);
                    dw_mtr_cmd[i].write(cmd);
                    if (ratio >= 1.0) {
                        interp.active = false;
                        s.gui_motor_cmd[i] = cmd;
                    }
                } else if (s.gui_motor_cmd_online[i]) {
                    dw_mtr_cmd[i].write(s.gui_motor_cmd[i]);
                }
            }

            for (int i = 0; i < 12; ++i) {
                if (!s.gui_motor_on_online[i]) {
                    continue;
                }
                dw_motor_on[i].write(s.gui_motor_on[i]);
            }
        }

        static int gui_mode_from_state(State state) {
            switch (state) {
            case State::IDLE:
                return 0; // IDLE
            case State::MANU:
                return 1; // MANU
            case State::READY:
                return 2; // READY
            case State::RL_WALK:
                return 3; // RL WALK
            default:
                return 0;
            }
        }

        void process_gui_mode_request(ManagerState& s) {
            if (!s.gui_mode_request_online) {
                return;
            }
            s.gui_mode_request_online = false;

            switch (s.gui_mode_request) {
            case 0: // IDLE
                s.robot_state.req_rl_control = false;
                state_commit(s, State::IDLE);
                getLogger()->info("[{}] GUI mode request accepted: IDLE", getName());
                break;

            case 1: // MANU
                s.robot_state.req_rl_control = false;
                state_commit(s, State::MANU);
                getLogger()->info("[{}] GUI mode request accepted: MANU", getName());
                break;

            case 2: // READY
                if (s.state_ != State::MANU && s.state_ != State::READY && s.state_ != State::RL_WALK) {
                    getLogger()->warn("[{}] GUI mode request READY rejected: robot is in INITIAL", getName());
                    break;
                }
                s.robot_state.req_rl_control = false;
                state_commit(s, State::READY);
                getLogger()->info("[{}] GUI mode request accepted: READY", getName());
                break;

            case 3: // RL WALK
                if (s.state_ != State::READY && s.state_ != State::RL_WALK) {
                    getLogger()->warn("[{}] GUI mode request RL WALK rejected: state is not READY/RL_CONTROL", getName());
                    break;
                }
                if (s.robot_state.walk_ready == false) {
                    getLogger()->warn("[{}] GUI mode request RL WALK rejected: walk not ready", getName());
                    break;
                }
                s.robot_state.req_rl_control = true;
                state_commit(s, State::RL_WALK);
                getLogger()->info("[{}] GUI mode request accepted: RL WALK", getName());
                break;

            default:
                getLogger()->warn("[{}] Unknown GUI mode request: {}", getName(), s.gui_mode_request);
                break;
            }
        }

        void wdt_process(ManagerState& s) {
            auto now = getCurrentTick();
            for (int i = 0; i < 12; i++) {
                s.robot_state.online.motor[i] = now - s.wdt.motor[i] <= 1 * getFrequency();
            }
            s.robot_state.online.imu = now - s.wdt.imu <= 1 * getFrequency();
            s.robot_state.online.joystick = now - s.wdt.joystick <= 1 * getFrequency();
        }

        static std::string robot_mode_to_string(int mode) {
            switch (mode) {
            case 0: return "IDLE";
            case 1: return "MANU";
            case 2: return "READY";
            case 3: return "RL WALK";
            default: return "IDLE";
            }
        }

        static int safety_level_from_state(State state) {
            switch (state) {
            case State::IDLE:
            case State::MANU:
            case State::READY:
                return SAFETY_ESSENTIAL;
            case State::RL_WALK:
                return SAFETY_STRICT;
            default:
                return SAFETY_ESSENTIAL;
            }
        }
    };
};