#pragma once

#include <rtfw/task.h>
#include "util.hpp"
#include "../custom_types.hpp"
#include "motor.impl.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <queue>
#include <iostream>

#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <unistd.h>
#include <linux/can.h>
#include <arpa/inet.h> // ntohs 사용을 위해 추가
#include <thread>


using namespace std::chrono_literals;
using namespace rtfw;
using namespace rtfw::rt;

namespace task_pool {

    class CanBus2 : public ITask {
    public:
        const char* getName() const override { return "CanBus2"; }

        void initialize(void*) override {

        }

        void execute(void*) override {
            for (int i=0; i<cmds.size(); i++) {
                dr_mtr_stat.on_update([&, i](const custom_types::MotorState& data) {
                    wdt[i] = getCurrentTick();
                    cmds[i] = data;
                });
            }

            if (so > 0) {
                struct can_frame rx;
                while (read(so, &rx, sizeof(rx)) > 0) {
                    for (auto i=0; i<motors.size(); i++) {
                        if (motors[i]->isMyFrame(&rx)) {
                            motors[i]->parseFeedback(&rx);

                            custom_types::MotorState state
                            state.pos = motors[i]->state.pos;
                            state.vel = motors[i]->state.vel;
                            state.torque = motors[i]->state.torque;
                            state.status = motors[i]->state.status;

                            dr_mtr_stat[i].write(state);
                            wdt[i] = getExecutionLocalTick();
                            break;
                        }
                    }
                }


                // 2. 제어 명령 생성 및 송신
                for (auto i=0; i<motors.size(); i++) {
                    auto tick = getExecutionLocalTick();
                    if (tick - wdt[i] > 1 * getFrequency()) {
                        motors[i]->Start(so, true);
                    }
                    motors[i]->Control(so, cmds[i]);
                }
            }
            else {
                PERIODIC_CALL(
                    so = can_open(const_cast<char*>(p_port.read().c_str()));
                , 1s);
            }

            dw_state.write(so > 0);
        }

        ~CanBus2() {
            if (so > 0) {
                for (auto& motor : motors) {
                    motor->Stop(so);
                }
                can_close();
            }
        }
    private:

        DataWriter<bool> dw_state{"can2_state", ArchiveOption::Enable};
            
        DataWriter<custom_types::MotorState> dr_mtr_stat[1] = {
            DataWriter<custom_types::MotorState>{"waist_yaw/state", ArchiveOption::Enable},
        };

        DataReader<custom_types::MotorCmd> dw_mtr_cmd[1] = {
            DataReader<custom_types::MotorCmd>{"waist_yaw/cmd"},
        };

        Parameter<std::string> p_port{"can2.port", "can2"};
        

    private:

        int can_open(char* port) {
            int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);

            if (s <= 0) {
                getLogger()->error("[{}] Failed to open CAN interface on {}", getName(), port);
                return s;
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
                getLogger()->info("[{}] connection closed", getName());
                // std::cout << "CanBus connection closed" << std::endl;
            }
            so = -1;
            init = false;
        }

        

    private:
        int so=-1;
        int wdt[1] = {0};

        std::array<custom_types::MotorCmd, 1> cmds;
        std::vector<std::shared_ptr<Motor>> motors = {
            std::make_shared<RobStride03>(0), // waist_yaw
        };

    };
};