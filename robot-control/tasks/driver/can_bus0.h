#pragma once

// CAN Bus 0 — Joint 0 & Joint 1 (직접 구동)
// Joint 0: base_yaw   (RobStride03 0x01) — revolute
// Joint 1: pitch       (RobStride03 0x02) — revolute

#include <rtfw/task.h>
#include "../util.hpp"
#include "../custom_types.hpp"
#include "motor.impl.hpp"

#include <chrono>
#include <cmath>
#include <cstring>
#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <linux/can.h>
#include <linux/can/error.h>
#include <linux/can/raw.h>
#include <memory>
#include <net/if.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>
#include <vector>

using namespace std::chrono_literals;
using namespace rtfw;
using namespace rtfw::rt;

namespace task_pool {

class CanBus0 : public ITask {
public:
    const char* getName() const override { return "CanBus0"; }

    static constexpr int N = 2; // 물리 모터 수 = 가상 조인트 수 (직접 구동)

    void initialize(void*) override {
        for (auto& f : on_flag)       f = false;
        for (auto& s : motor_sync_)   s = false;
        for (auto& p : joint_fb_pos_) p = 0.0;
        for (auto& c : cmds)          c = {};
        for (auto& i : cmd_interp)    i = {};
    }

    void execute(void*) override {
        // 1. 상위 명령 수신
        for (int i = 0; i < N; i++) {
            dr_joint_cmd[i].on_update([&, i](const custom_types::MotorCmd& data) {
                cmd_wdt[i] = getExecutionLocalTick();
                if (data.duration_ms > 0.0) {
                    if (!cmd_interp[i].active ||
                        std::abs(data.pos - cmd_interp[i].target_pos) > 1e-4) {
                        cmd_interp[i].start_ref   = joint_fb_pos_[i];
                        cmd_interp[i].target_pos  = data.pos;
                        cmd_interp[i].duration_ms = data.duration_ms;
                        cmd_interp[i].elapsed_ms  = 0.0;
                        cmd_interp[i].active       = true;
                    }
                } else {
                    cmd_interp[i].active = false;
                    cmds[i].pos = data.pos;
                }
                cmds[i].vel    = data.vel;
                cmds[i].torque = data.torque;
                cmds[i].kp     = data.kp;
                cmds[i].kd     = data.kd;
            });

            dr_joint_on[i].on_update([&, i](const bool& on) {
                if (!on_flag[i] && on) {
                    getLogger()->info("[{}] Joint {} ON", getName(), i);
                    on_flag[i]     = true;
                    motor_sync_[i] = false;
                    cmd_interp[i]  = {};
                    if (so > 0) motors[i]->Start(so);
                }
                if (on_flag[i] && !on) {
                    getLogger()->info("[{}] Joint {} OFF", getName(), i);
                    on_flag[i]     = false;
                    motor_sync_[i] = false;
                    cmd_interp[i]  = {};
                    if (so > 0) motors[i]->Stop(so);
                }
            });
        }

        if (so > 0) {
            // 2. CAN 수신 및 피드백 처리
            struct can_frame rx;
            while (read(so, &rx, sizeof(rx)) > 0) {
                if (rx.can_id & CAN_ERR_FLAG) {
                    if (rx.can_id & CAN_ERR_BUSOFF) {
                        getLogger()->error("[{}] CAN bus-off! Restarting...", getName());
                        restart_required = true;
                    } else if (rx.can_id & CAN_ERR_BUSERROR) {
                        getLogger()->warn("[{}] CAN bus error", getName());
                    }
                    can_close();
                    break;
                }
                for (int i = 0; i < N; i++) {
                    if (motors[i]->isMyFrame(&rx) && motors[i]->parseFeedback(&rx)) {
                        com_wdt[i] = getExecutionLocalTick();

                        if (!motor_sync_[i]) {
                            joint_fb_pos_[i] = motors[i]->state.pos;
                            cmds[i].pos      = joint_fb_pos_[i];
                            cmd_interp[i]    = {};
                            motor_sync_[i]   = true;
                            getLogger()->info("[{}] Joint {} synced pos={:.3f}", getName(), i, motors[i]->state.pos);
                        }

                        custom_types::MotorState st{};
                        st.pos     = motors[i]->state.pos;
                        st.vel     = motors[i]->state.vel;
                        st.torque  = motors[i]->state.torque;
                        st.status  = motors[i]->state.status;
                        st.enabled = on_flag[i];
                        dw_joint_state[i].write(st);

                        joint_fb_pos_[i] = motors[i]->state.pos;
                        break;
                    }
                }
            }

            // 3. Interpolation 진행 및 모터 명령 송신
            const double dt_ms = 1000.0 / getFrequency();
            auto tick = getExecutionLocalTick();
            int offline = 0;

            for (int i = 0; i < N; i++) {
                if (tick - com_wdt[i] > 1 * getFrequency()) {
                    motors[i]->state.online = false;
                    offline++;
                    if (motor_sync_[i]) {
                        motor_sync_[i] = false;
                        cmd_interp[i]  = {};
                        getLogger()->warn("[{}] Joint {} timeout, re-sync required", getName(), i);
                    }
                }

                if (cmd_interp[i].active) {
                    cmd_interp[i].elapsed_ms += dt_ms;
                    const double ratio = std::min(cmd_interp[i].elapsed_ms / cmd_interp[i].duration_ms, 1.0);
                    cmds[i].pos = cmd_interp[i].start_ref + ratio * (cmd_interp[i].target_pos - cmd_interp[i].start_ref);
                    if (ratio >= 1.0) cmd_interp[i].active = false;
                }

                const bool ready = on_flag[i] && motor_sync_[i] && (tick - cmd_wdt[i] <= 1 * getFrequency());
                const MotorCommand applied = ready ? cmds[i] : MotorCommand{};

                custom_types::MotorCmd applied_msg{};
                applied_msg.pos    = applied.pos;
                applied_msg.vel    = applied.vel;
                applied_msg.torque = applied.torque;
                applied_msg.kp     = applied.kp;
                applied_msg.kd     = applied.kd;
                dw_joint_cmd_applied[i].write(applied_msg);

                if (ready) motors[i]->Control(so, cmds[i]);
                else       motors[i]->Control(so, MotorCommand{});
            }

            PERIODIC_CALL(
                for (int i = 0; i < N; i++) {
                    if (!motors[i]->state.online)
                        getLogger()->warn("[{}] Motor {} offline", getName(), motors[i]->id);
                }
            , 1s);

            if (offline < N) can_wdt = getExecutionLocalTick();
            if (getExecutionLocalTick() - can_wdt > 3 * getFrequency()) {
                PERIODIC_CALL(getLogger()->warn("[{}] All motors offline!", getName());, 1s);
                can_close();
            }
        } else {
            // 재연결 시도 (1초마다)
            if (getExecutionLocalTick() % getFrequency() == 0) {
                getLogger()->info("[{}] try connection", getName());
                so = can_open(const_cast<char*>(p_port.read().c_str()));
                if (so > 0) {
                    can_wdt = getExecutionLocalTick();
                    for (auto& t : cmd_wdt) t = getExecutionLocalTick();
                    for (auto& t : com_wdt) t = getExecutionLocalTick();
                    for (auto& m : motors)  m->Start(so, true);
                    for (auto& f : on_flag) f = false;
                }
            }
        }

        dw_state.write(so > 0);
    }

    bool onOverrun() override { return true; }

    ~CanBus0() {
        if (so > 0) {
            for (auto& m : motors) m->Stop(so);
            can_close();
        }
    }

private:
    struct MotorCmdInterp {
        bool   active      = false;
        double start_ref   = 0.0;
        double target_pos  = 0.0;
        double duration_ms = 0.0;
        double elapsed_ms  = 0.0;
    };

    // ── 데이터 채널 ──
    DataWriter<bool> dw_state{"can0_state", ArchiveOption::Enable};

    DataWriter<custom_types::MotorState> dw_joint_state[N] = {
        DataWriter<custom_types::MotorState>{"joint0/state", ArchiveOption::Enable},
        DataWriter<custom_types::MotorState>{"joint1/state", ArchiveOption::Enable},
    };
    DataWriter<custom_types::MotorCmd> dw_joint_cmd_applied[N] = {
        DataWriter<custom_types::MotorCmd>{"joint0/cmd_applied", ArchiveOption::Enable},
        DataWriter<custom_types::MotorCmd>{"joint1/cmd_applied", ArchiveOption::Enable},
    };
    DataReader<custom_types::MotorCmd> dr_joint_cmd[N] = {
        DataReader<custom_types::MotorCmd>{"joint0/cmd"},
        DataReader<custom_types::MotorCmd>{"joint1/cmd"},
    };
    DataReader<bool> dr_joint_on[N] = {
        DataReader<bool>{"joint0/on"},
        DataReader<bool>{"joint1/on"},
    };

    Parameter<std::string> p_port{"can0.port", "can0"};

    // ── 모터 ──
    std::vector<std::shared_ptr<Motor>> motors = {
        std::make_shared<RobStride03>(0x01), // joint0: base_yaw
        std::make_shared<RobStride03>(0x02), // joint1: pitch
    };

    // ── 내부 상태 ──
    std::array<MotorCmdInterp, N> cmd_interp{};
    std::array<MotorCommand, N>   cmds{};
    std::array<double, N>         joint_fb_pos_{};
    bool   on_flag[N]     = {};
    bool   motor_sync_[N] = {};
    int    cmd_wdt[N]     = {};
    int    com_wdt[N]     = {};
    int    can_wdt        = 0;
    int    so             = -1;
    bool   link_initialized = false;
    bool   restart_required = false;

    // ── CAN 유틸리티 ──
    bool run_ip_command(const char* port, const char* action, bool log_fail = true) {
        char cmd[256];
        snprintf(cmd, sizeof(cmd), "ip link set dev %s %s", port, action);
        int ret = system(cmd);
        if (ret == -1 || !WIFEXITED(ret) || WEXITSTATUS(ret) != 0) {
            if (log_fail) getLogger()->error("[{}] '{}' failed", getName(), cmd);
            return false;
        }
        return true;
    }

    bool is_link_up(const char* port) {
        int fd = socket(AF_INET, SOCK_DGRAM, 0);
        if (fd < 0) return false;
        struct ifreq ifr{};
        strncpy(ifr.ifr_name, port, IFNAMSIZ - 1);
        bool up = (ioctl(fd, SIOCGIFFLAGS, &ifr) == 0) && (ifr.ifr_flags & IFF_UP);
        close(fd);
        return up;
    }

    void reset_can_state() {
        can_wdt = 0;
        for (auto& t : cmd_wdt)   t = 0;
        for (auto& t : com_wdt)   t = 0;
        for (auto& m : motors)    m->state.online = false;
        for (auto& s : motor_sync_) s = false;
        for (auto& i : cmd_interp)  i = {};
    }

    bool can_ifup(const char* port) {
        reset_can_state();
        const char* attempts[] = {
            "up type can bitrate 1000000 restart-ms 100 berr-reporting on",
            "up type can bitrate 1000000 restart-ms 100",
            "up type can bitrate 1000000"
        };
        for (auto* a : attempts) {
            if (run_ip_command(port, a, false)) {
                getLogger()->info("[{}] {} up: '{}'", getName(), port, a);
                return true;
            }
        }
        getLogger()->error("[{}] Failed to configure {}", getName(), port);
        return false;
    }

    int can_open(char* port) {
        if (!link_initialized) {
            if (is_link_up(port)) {
                link_initialized = true;
                getLogger()->info("[{}] {} already up", getName(), port);
            } else {
                if (!can_ifup(port)) return -1;
                link_initialized = true;
            }
        } else if (restart_required) {
            run_ip_command(port, "restart", false);
        }

        int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (s <= 0) { getLogger()->error("[{}] socket() failed on {}", getName(), port); return s; }

        can_err_mask_t em = CAN_ERR_TX_TIMEOUT | CAN_ERR_BUSOFF | CAN_ERR_BUSERROR | CAN_ERR_RESTARTED;
        setsockopt(s, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &em, sizeof(em));

        struct ifreq ifr; strcpy(ifr.ifr_name, port);
        if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) { close(s); return -1; }

        struct sockaddr_can addr{};
        addr.can_family = AF_CAN; addr.can_ifindex = ifr.ifr_ifindex;
        if (bind(s, (struct sockaddr*)&addr, sizeof(addr)) < 0) { close(s); return -1; }

        int flags = fcntl(s, F_GETFL, 0); fcntl(s, F_SETFL, flags | O_NONBLOCK);
        restart_required = false;
        getLogger()->info("[{}] Connected on {}", getName(), port);
        return s;
    }

    void can_close() {
        if (so > 0) { close(so); getLogger()->info("[{}] CAN closed", getName()); }
        so = -1;
        reset_can_state();
    }
};

} // namespace task_pool
