#pragma once

#include <rtfw/task.h>
#include "../util.hpp"
#include "../custom_types.hpp"
#include "motor.impl.hpp"
#include "../kin_2rsu.hpp"

#include <chrono>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <inttypes.h>
#include <cmath>
#include <chrono>
#include <thread>
#include <vector>
#include <memory>
#include <csignal>
#include <fcntl.h>
#include <arpa/inet.h> // ntohs 사용을 위해 추가


using namespace std::chrono_literals;
using namespace rtfw;
using namespace rtfw::rt;

namespace task_pool {

    class CanBus1 : public ITask {
    public:
        const char* getName() const override { return "CanBus1"; }

        void initialize(void*) override {
            for (auto& flag: on_flag) flag = true;
        }

        void execute(void*) override {
            bool cmd_updated[6] = {false};
            for (int i=0; i<cmds.size(); i++) {
                dr_mtr_cmd[i].on_update([&, i](const custom_types::MotorCmd& data) {
                    wdt[i] = getCurrentTick();
                    cmds[i].pos = data.pos;
                    cmds[i].vel = data.vel;
                    cmds[i].torque = data.torque;
                    cmds[i].kp = data.kp;
                    cmds[i].kd = data.kd;
                    cmd_updated[i] = true;
                });

                dr_motor_on[i].on_update([&, i](const bool& on) {
                    if (!on_flag[i] && on) {
                        getLogger()->info("[{}] Motor {} ON", getName(), i);
                        on_flag[i] = on;
                    }
                    if (on_flag[i] && !on) {
                        getLogger()->info("[{}] Motor {} OFF", getName(), i);
                        on_flag[i] = on;
                    }
                });
            }

            if (so > 0) {
                struct can_frame rx;
                bool stat_updated[6] = {false};
                while (read(so, &rx, sizeof(rx)) > 0) {
                    for (auto i=0; i<motors.size(); i++) {
                        if (motors[i]->isMyFrame(&rx)) {
                            motors[i]->parseFeedback(&rx);

                            wdt[i] = getExecutionLocalTick();
                            stat_updated[i] = true;
                            break;
                        }
                    }
                }

                for (auto i=0; i<motors.size(); i++) {
                    if (stat_updated[i]) {
                        custom_types::MotorState state{};
                        state.pos = motors[i]->state.pos;
                        state.vel = motors[i]->state.vel;
                        state.torque = motors[i]->state.torque;
                        state.status = motors[i]->state.status;
                        dw_mtr_stat[i].write(state);
                    }
                }


                // 2. 제어 명령 생성 및 송신
                int offline = 0;
                for (auto i=0; i<motors.size(); i++) {
                    auto tick = getExecutionLocalTick();
                    if (tick - wdt[i] > 1 * getFrequency()) {
                        PERIODIC_CALL(
                            getLogger()->warn("[{}] Motor {} seems offline. Sending start command.", getName(), i);
                        , 1s);
                        motors[i]->state.online = false;
                        motors[i]->Start(so, true);
                        offline++;
                    }

                    const bool motor_on = on_flag[i];
                    const MotorCommand applied_cmd = motor_on ? cmds[i] : MotorCommand{};

                    custom_types::MotorCmd applied{};
                    applied.pos = applied_cmd.pos;
                    applied.vel = applied_cmd.vel;
                    applied.torque = applied_cmd.torque;
                    applied.kp = applied_cmd.kp;
                    applied.kd = applied_cmd.kd;
                    dw_mtr_cmd_applied[i].write(applied);

                    if (motor_on) {
                        motors[i]->Control(so, cmds[i]);
                    } else {
                        MotorCommand zero_cmd{};
                        motors[i]->Control(so, zero_cmd);
                    }
                }

                if (offline < cmds.size()) {
                    can_wdt = getExecutionLocalTick();
                }
                if (getExecutionLocalTick() - can_wdt > 5 * getFrequency()) {
                    PERIODIC_CALL(
                        getLogger()->warn("[{}] All motors seem offline. Check connections!", getName());
                        can_close();
                    , 1s);
                }
            }
            else {
                PERIODIC_CALL(
                    so = can_open(const_cast<char*>(p_port.read().c_str()));
                    if (so > 0) {
                        for (auto& t: wdt)  t = getCurrentTick();
                        for (auto& mtr: motors) mtr->Start(so, true);
                    }
                , 1s);
            }

            dw_state.write(so > 0);
        }

        ~CanBus1() {
            if (so > 0) {
                for (auto& motor : motors) {
                    motor->Stop(so);
                }
                can_close();
            }
        }
    private:

        DataWriter<bool> dw_state{"can1_state", ArchiveOption::Enable};
            
        DataWriter<custom_types::MotorState> dw_mtr_stat[6] = {
            DataWriter<custom_types::MotorState>{"hip_yaw_right/state", ArchiveOption::Enable},
            DataWriter<custom_types::MotorState>{"hip_roll_right/state", ArchiveOption::Enable},
            DataWriter<custom_types::MotorState>{"hip_pitch_right/state", ArchiveOption::Enable},
            DataWriter<custom_types::MotorState>{"knee_right/state", ArchiveOption::Enable},
            DataWriter<custom_types::MotorState>{"ankle_pitch_right/state", ArchiveOption::Enable},
            DataWriter<custom_types::MotorState>{"ankle_roll_right/state", ArchiveOption::Enable},
        };

        DataReader<custom_types::MotorCmd> dr_mtr_cmd[6] = {
            DataReader<custom_types::MotorCmd>{"hip_yaw_right/cmd"},
            DataReader<custom_types::MotorCmd>{"hip_roll_right/cmd"},
            DataReader<custom_types::MotorCmd>{"hip_pitch_right/cmd"},
            DataReader<custom_types::MotorCmd>{"knee_right/cmd"},
            DataReader<custom_types::MotorCmd>{"ankle_pitch_right/cmd"},
            DataReader<custom_types::MotorCmd>{"ankle_roll_right/cmd"},
        };
        DataWriter<custom_types::MotorCmd> dw_mtr_cmd_applied[6] = {
            DataWriter<custom_types::MotorCmd>{"hip_yaw_right/cmd_applied", ArchiveOption::Enable},
            DataWriter<custom_types::MotorCmd>{"hip_roll_right/cmd_applied", ArchiveOption::Enable},
            DataWriter<custom_types::MotorCmd>{"hip_pitch_right/cmd_applied", ArchiveOption::Enable},
            DataWriter<custom_types::MotorCmd>{"knee_right/cmd_applied", ArchiveOption::Enable},
            DataWriter<custom_types::MotorCmd>{"ankle_pitch_right/cmd_applied", ArchiveOption::Enable},
            DataWriter<custom_types::MotorCmd>{"ankle_roll_right/cmd_applied", ArchiveOption::Enable},
        };
        DataReader<bool> dr_motor_on[6] = {
            DataReader<bool>{"hip_yaw_right/on"},
            DataReader<bool>{"hip_roll_right/on"},
            DataReader<bool>{"hip_pitch_right/on"},
            DataReader<bool>{"knee_right/on"},
            DataReader<bool>{"ankle_pitch_right/on"},
            DataReader<bool>{"ankle_roll_right/on"},
        };

        Parameter<std::string> p_port{"can1.port", "can1"};
        

    private:

        int can_open(char* port) {
            int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);

            if (s <= 0) {
                getLogger()->error("[{}] Failed to open CAN interface on {}", getName(), port);
                return s;
                // std::cerr << "Failed to open CAN interface on " << port << std::endl;
            }

            getLogger()->info("[{}] CanBus connection established on {}", getName(), port);
            struct ifreq ifr; strcpy(ifr.ifr_name, port);
            ioctl(s, SIOCGIFINDEX, &ifr);
            struct sockaddr_can addr = {0};
            addr.can_family = AF_CAN; addr.can_ifindex = ifr.ifr_ifindex;
            bind(s, (struct sockaddr *)&addr, sizeof(addr));

            // 수신 Non-blocking 설정
            int flags = fcntl(s, F_GETFL, 0); fcntl(s, F_SETFL, flags | O_NONBLOCK);

            return s;
        }

        void can_close() {

            if (so > 0) {
                close(so);
                getLogger()->info("[{}] CanBus connection closed", getName());
                // std::cout << "CanBus connection closed" << std::endl;
            }
            so = -1;
        }

        

    private:
        int so=-1;
        int wdt[6] = {0,};
        int can_wdt = 0;
        bool on_flag[6] = {false,};

        std::array<MotorCommand, 6> cmds;
        std::vector<std::shared_ptr<Motor>> motors = {
            std::make_shared<RmdX6P36>(0), // hip_yaw_right
            std::make_shared<RmdX6P36>(1), // hip_roll_right
            std::make_shared<RmdX6P36>(2), // hip_pitch_right
            std::make_shared<RmdX6P36>(3), // knee_right
            std::make_shared<RobStride03>(4), // ankle_pitch_right
            std::make_shared<RobStride03>(5), // ankle_roll_right
        };

    };
};