#pragma once

// CAN Bus 1 — Joint 2, 3, 4 (와이어/볼스크류 구동)
//
// 물리 모터 배치:
//   motors[0] = MyActuatorX6 0x03  →  Joint 2 (하단링크길이, 볼스크류)
//   motors[1] = MyActuatorX6 0x04  →  Joint 3 (팔꿈치 피치, 와이어 A)
//   motors[2] = MyActuatorX6 0x05  →  Joint 3 (팔꿈치 피치, 와이어 B)
//   motors[3] = MyActuatorX6 0x06  →  Joint 4 (상단링크길이, 와이어 A)
//   motors[4] = MyActuatorX6 0x07  →  Joint 4 (상단링크길이, 와이어 B)
//
// 가상 조인트 인덱스 (this file 내부):
//   ji=0 → joint2,  ji=1 → joint3,  ji=2 → joint4

#include <rtfw/task.h>
#include "../util.hpp"
#include "../custom_types.hpp"
#include "motor.impl.hpp"
#include "../kin_manipulator.hpp"

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

class CanBus1 : public ITask {
public:
    const char* getName() const override { return "CanBus1"; }

    static constexpr int NM = 5; // 물리 모터 수
    static constexpr int NJ = 3; // 가상 조인트 수 (joint2=ji0, joint3=ji1, joint4=ji2)

    // 조인트 ji의 가상 조인트 이름 (데이터 채널용)
    // ji0=joint2, ji1=joint3, ji2=joint4
    static constexpr const char* joint_name[NJ] = {"joint2", "joint3", "joint4"};

    void initialize(void*) override {
        for (auto& f : joint_on)     f = false;
        for (auto& s : motor_sync_)  s = false;
        for (auto& p : joint_fb_pos_) p = 0.0;
        for (auto& c : joint_cmds)   c = {};
        for (auto& i : cmd_interp)   i = {};
    }

    void execute(void*) override {
        // 1. 상위 조인트 명령 수신
        for (int ji = 0; ji < NJ; ji++) {
            dr_joint_cmd[ji].on_update([&, ji](const custom_types::MotorCmd& data) {
                cmd_wdt[ji] = getExecutionLocalTick();
                if (data.duration_ms > 0.0) {
                    if (!cmd_interp[ji].active ||
                        std::abs(data.pos - cmd_interp[ji].target_pos) > 1e-4) {
                        cmd_interp[ji].start_ref   = joint_fb_pos_[ji];
                        cmd_interp[ji].target_pos  = data.pos;
                        cmd_interp[ji].duration_ms = data.duration_ms;
                        cmd_interp[ji].elapsed_ms  = 0.0;
                        cmd_interp[ji].active       = true;
                    }
                } else {
                    cmd_interp[ji].active = false;
                    joint_cmds[ji].pos = data.pos;
                }
                joint_cmds[ji].vel    = data.vel;
                joint_cmds[ji].torque = data.torque;
                joint_cmds[ji].kp     = data.kp;
                joint_cmds[ji].kd     = data.kd;
            });

            // 조인트 on/off → 해당 물리 모터 제어
            dr_joint_on[ji].on_update([&, ji](const bool& on) {
                if (!joint_on[ji] && on) {
                    getLogger()->info("[{}] Joint {} ON", getName(), ji + 2);
                    joint_on[ji] = true;
                    cmd_interp[ji] = {};
                    for (int mi : joint_motors(ji)) {
                        motor_sync_[mi] = false;
                        if (so > 0) motors[mi]->Start(so);
                    }
                }
                if (joint_on[ji] && !on) {
                    getLogger()->info("[{}] Joint {} OFF", getName(), ji + 2);
                    joint_on[ji] = false;
                    cmd_interp[ji] = {};
                    for (int mi : joint_motors(ji)) {
                        motor_sync_[mi] = false;
                        if (so > 0) motors[mi]->Stop(so);
                    }
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
                for (int mi = 0; mi < NM; mi++) {
                    if (motors[mi]->isMyFrame(&rx) && motors[mi]->parseFeedback(&rx)) {
                        com_wdt[mi] = getExecutionLocalTick();
                        // 해당 물리 모터의 조인트가 아직 동기화 안 된 경우 처리
                        try_sync_joint(motor_joint(mi));
                        break;
                    }
                }
            }

            // 피드백 FK → 가상 조인트 상태 발행
            publish_joint_states();

            // 3. Interpolation 진행 및 모터 명령 송신
            const double dt_ms = 1000.0 / getFrequency();
            auto tick = getExecutionLocalTick();
            int offline = 0;

            for (int mi = 0; mi < NM; mi++) {
                if (tick - com_wdt[mi] > 1 * getFrequency()) {
                    motors[mi]->state.online = false;
                    offline++;
                    if (motor_sync_[mi]) {
                        motor_sync_[mi] = false;
                        int ji = motor_joint(mi);
                        cmd_interp[ji] = {};
                        getLogger()->warn("[{}] Motor[{}] timeout, re-sync required", getName(), mi);
                    }
                }
            }

            // Interpolation: 가상 조인트 공간에서 진행
            for (int ji = 0; ji < NJ; ji++) {
                if (cmd_interp[ji].active) {
                    cmd_interp[ji].elapsed_ms += dt_ms;
                    const double ratio = std::min(cmd_interp[ji].elapsed_ms / cmd_interp[ji].duration_ms, 1.0);
                    joint_cmds[ji].pos = cmd_interp[ji].start_ref + ratio * (cmd_interp[ji].target_pos - cmd_interp[ji].start_ref);
                    if (ratio >= 1.0) cmd_interp[ji].active = false;
                }
            }

            // IK → 물리 모터 명령 송신
            send_motor_commands(tick, offline);

            PERIODIC_CALL(
                for (int mi = 0; mi < NM; mi++) {
                    if (!motors[mi]->state.online)
                        getLogger()->warn("[{}] Motor[{}] (0x{:02X}) offline", getName(), mi, motors[mi]->id);
                }
            , 1s);

            if (offline < NM) can_wdt = getExecutionLocalTick();
            if (getExecutionLocalTick() - can_wdt > 3 * getFrequency()) {
                PERIODIC_CALL(getLogger()->warn("[{}] All motors offline!", getName());, 1s);
                can_close();
            }
        } else {
            if (getExecutionLocalTick() % getFrequency() == 0) {
                getLogger()->info("[{}] try connection", getName());
                so = can_open(const_cast<char*>(p_port.read().c_str()));
                if (so > 0) {
                    can_wdt = getExecutionLocalTick();
                    for (auto& t : cmd_wdt) t = getExecutionLocalTick();
                    for (auto& t : com_wdt) t = getExecutionLocalTick();
                    for (auto& m : motors)  m->Start(so, true);
                    for (auto& f : joint_on) f = false;
                }
            }
        }

        dw_state.write(so > 0);
    }

    bool onOverrun() override { return true; }

    ~CanBus1() {
        if (so > 0) {
            for (auto& m : motors) m->Stop(so);
            can_close();
        }
    }

private:
    // ── 조인트↔모터 인덱스 매핑 헬퍼 ──

    // 가상 조인트 ji에 속한 물리 모터 인덱스 목록
    static std::vector<int> joint_motors(int ji) {
        switch (ji) {
            case 0: return {0};        // joint2: motors[0]
            case 1: return {1, 2};     // joint3: motors[1], motors[2]
            case 2: return {3, 4};     // joint4: motors[3], motors[4]
            default: return {};
        }
    }

    // 물리 모터 mi가 속한 가상 조인트 ji
    static int motor_joint(int mi) {
        if (mi == 0) return 0;
        if (mi == 1 || mi == 2) return 1;
        return 2; // mi == 3 || mi == 4
    }

    // ji의 모든 모터가 동기화됐는지 확인
    bool joint_fully_synced(int ji) const {
        for (int mi : joint_motors(ji))
            if (!motor_sync_[mi]) return false;
        return true;
    }

    // ── 피드백 FK ──

    double compute_joint_fk(int ji) const {
        switch (ji) {
            case 0: return kin_manip::joint2_motor_to_joint(motors[0]->state.pos);
            case 1: return kin_manip::joint3_motors_to_joint(motors[1]->state.pos, motors[2]->state.pos);
            case 2: return kin_manip::joint4_motors_to_joint(motors[3]->state.pos, motors[4]->state.pos);
            default: return 0.0;
        }
    }

    double compute_joint_vel_fk(int ji) const {
        switch (ji) {
            case 0: return kin_manip::joint2_vel_motor_to_joint(motors[0]->state.vel);
            case 1: return kin_manip::joint3_vel_motors_to_joint(motors[1]->state.vel, motors[2]->state.vel);
            case 2: return kin_manip::joint4_vel_motors_to_joint(motors[3]->state.vel, motors[4]->state.vel);
            default: return 0.0;
        }
    }

    double compute_joint_torque_fk(int ji) const {
        switch (ji) {
            case 0: return kin_manip::joint2_torque_motor_to_joint(motors[0]->state.torque);
            case 1: return kin_manip::joint3_torque_motors_to_joint(motors[1]->state.torque, motors[2]->state.torque);
            case 2: return kin_manip::joint4_torque_motors_to_joint(motors[3]->state.torque, motors[4]->state.torque);
            default: return 0.0;
        }
    }

    // 피드백 수신 시 조인트 동기화 시도
    void try_sync_joint(int ji) {
        bool all_synced_before = joint_fully_synced(ji);
        // 방금 수신된 모터를 sync 표시
        for (int mi : joint_motors(ji)) {
            if (motors[mi]->state.online && !motor_sync_[mi]) {
                motor_sync_[mi] = true;
                getLogger()->info("[{}] Motor[{}] synced pos={:.3f}", getName(), mi, motors[mi]->state.pos);
            }
        }
        // 이 조인트의 모든 모터가 동기화 완료됐을 때 초기 명령을 현재 위치로 설정
        if (!all_synced_before && joint_fully_synced(ji)) {
            joint_fb_pos_[ji] = compute_joint_fk(ji);
            joint_cmds[ji].pos = joint_fb_pos_[ji];
            cmd_interp[ji] = {};
            getLogger()->info("[{}] Joint{} synced at pos={:.4f}", getName(), ji + 2, joint_fb_pos_[ji]);
        }
    }

    // 가상 조인트 상태 발행 (FK)
    void publish_joint_states() {
        for (int ji = 0; ji < NJ; ji++) {
            if (!joint_fully_synced(ji)) continue;

            joint_fb_pos_[ji] = compute_joint_fk(ji);

            custom_types::MotorState st{};
            st.pos     = joint_fb_pos_[ji];
            st.vel     = compute_joint_vel_fk(ji);
            st.torque  = compute_joint_torque_fk(ji);
            st.status  = 0;
            st.enabled = joint_on[ji];
            dw_joint_state[ji].write(st);
        }
    }

    // IK → 물리 모터 명령 송신
    void send_motor_commands(uint64_t tick, int /*offline*/) {
        for (int ji = 0; ji < NJ; ji++) {
            const bool ready = joint_on[ji] && joint_fully_synced(ji) &&
                               (tick - cmd_wdt[ji] <= 1 * getFrequency());

            if (!ready) {
                // 오프라인 or 준비 안 됨 → 영 명령
                for (int mi : joint_motors(ji))
                    motors[mi]->Control(so, MotorCommand{});
                continue;
            }

            // IK: 가상 조인트 명령 → 물리 모터 명령
            apply_ik_and_send(ji);

            // applied cmd 발행 (가상 조인트 공간)
            custom_types::MotorCmd applied{};
            applied.pos    = joint_cmds[ji].pos;
            applied.vel    = joint_cmds[ji].vel;
            applied.torque = joint_cmds[ji].torque;
            applied.kp     = joint_cmds[ji].kp;
            applied.kd     = joint_cmds[ji].kd;
            dw_joint_cmd_applied[ji].write(applied);
        }
    }

    void apply_ik_and_send(int ji) {
        const auto& cmd = joint_cmds[ji];
        switch (ji) {
            case 0: { // joint2: 볼스크류 1:1 IK
                MotorCommand mc{};
                mc.pos    = kin_manip::joint2_joint_to_motor(cmd.pos);
                mc.vel    = kin_manip::joint2_vel_joint_to_motor(cmd.vel);
                mc.torque = kin_manip::joint2_force_joint_to_motor(cmd.torque);
                mc.kp = cmd.kp;
                mc.kd = cmd.kd;
                motors[0]->Control(so, mc);
                break;
            }
            case 1: { // joint3: 팔꿈치 와이어 2모터 IK
                auto [pa, pb] = kin_manip::joint3_joint_to_motors(cmd.pos);
                auto [va, vb] = kin_manip::joint3_vel_joint_to_motors(cmd.vel);
                auto [ta, tb] = kin_manip::joint3_torque_joint_to_motors(cmd.torque);
                MotorCommand mca{}, mcb{};
                mca.pos = pa; mca.vel = va; mca.torque = ta; mca.kp = cmd.kp; mca.kd = cmd.kd;
                mcb.pos = pb; mcb.vel = vb; mcb.torque = tb; mcb.kp = cmd.kp; mcb.kd = cmd.kd;
                motors[1]->Control(so, mca);
                motors[2]->Control(so, mcb);
                break;
            }
            case 2: { // joint4: 상단링크 와이어 2모터 IK
                auto [pa, pb] = kin_manip::joint4_joint_to_motors(cmd.pos);
                auto [va, vb] = kin_manip::joint4_vel_joint_to_motors(cmd.vel);
                auto [fa, fb] = kin_manip::joint4_force_joint_to_motors(cmd.torque);
                MotorCommand mca{}, mcb{};
                mca.pos = pa; mca.vel = va; mca.torque = fa; mca.kp = cmd.kp; mca.kd = cmd.kd;
                mcb.pos = pb; mcb.vel = vb; mcb.torque = fb; mcb.kp = cmd.kp; mcb.kd = cmd.kd;
                motors[3]->Control(so, mca);
                motors[4]->Control(so, mcb);
                break;
            }
        }
    }

    // ── 데이터 채널 ──
    DataWriter<bool> dw_state{"can1_state", ArchiveOption::Enable};

    DataWriter<custom_types::MotorState> dw_joint_state[NJ] = {
        DataWriter<custom_types::MotorState>{"joint2/state", ArchiveOption::Enable},
        DataWriter<custom_types::MotorState>{"joint3/state", ArchiveOption::Enable},
        DataWriter<custom_types::MotorState>{"joint4/state", ArchiveOption::Enable},
    };
    DataWriter<custom_types::MotorCmd> dw_joint_cmd_applied[NJ] = {
        DataWriter<custom_types::MotorCmd>{"joint2/cmd_applied", ArchiveOption::Enable},
        DataWriter<custom_types::MotorCmd>{"joint3/cmd_applied", ArchiveOption::Enable},
        DataWriter<custom_types::MotorCmd>{"joint4/cmd_applied", ArchiveOption::Enable},
    };
    DataReader<custom_types::MotorCmd> dr_joint_cmd[NJ] = {
        DataReader<custom_types::MotorCmd>{"joint2/cmd"},
        DataReader<custom_types::MotorCmd>{"joint3/cmd"},
        DataReader<custom_types::MotorCmd>{"joint4/cmd"},
    };
    DataReader<bool> dr_joint_on[NJ] = {
        DataReader<bool>{"joint2/on"},
        DataReader<bool>{"joint3/on"},
        DataReader<bool>{"joint4/on"},
    };

    Parameter<std::string> p_port{"can1.port", "can1"};

    // ── 모터 ──
    std::vector<std::shared_ptr<Motor>> motors = {
        std::make_shared<MyActuatorX6>(0x03), // joint2: 볼스크류
        std::make_shared<MyActuatorX6>(0x04), // joint3: 와이어 A
        std::make_shared<MyActuatorX6>(0x05), // joint3: 와이어 B
        std::make_shared<MyActuatorX6>(0x06), // joint4: 와이어 A
        std::make_shared<MyActuatorX6>(0x07), // joint4: 와이어 B
    };

    // ── 내부 상태 ──
    struct MotorCmdInterp {
        bool   active      = false;
        double start_ref   = 0.0;
        double target_pos  = 0.0;
        double duration_ms = 0.0;
        double elapsed_ms  = 0.0;
    };

    std::array<MotorCmdInterp, NJ> cmd_interp{};
    std::array<MotorCommand, NJ>   joint_cmds{};
    std::array<double, NJ>         joint_fb_pos_{};
    bool   joint_on[NJ]    = {};
    bool   motor_sync_[NM] = {};
    int    cmd_wdt[NJ]     = {};
    int    com_wdt[NM]     = {};
    int    can_wdt         = 0;
    int    so              = -1;
    bool   link_initialized = false;
    bool   restart_required = false;

    // ── CAN 유틸리티 (can_bus0.h와 동일 구조) ──
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
        for (auto& t : cmd_wdt)    t = 0;
        for (auto& t : com_wdt)    t = 0;
        for (auto& m : motors)     m->state.online = false;
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
