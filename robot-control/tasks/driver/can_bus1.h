#pragma once

// CAN Bus 1 — Joint 3, Joint 4 (와이어 구동)
//
// 물리 모터 배치:
//   motors[0] = MyActuatorX6 0x04  →  Joint 3 (팔꿈치 피치, 와이어 A, 장력)
//   motors[1] = MyActuatorX6 0x05  →  Joint 3 (팔꿈치 피치, 와이어 B, 위치)
//   motors[2] = MyActuatorX6 0x06  →  Joint 4 (상단링크길이, 와이어 A)
//   motors[3] = MyActuatorX6 0x07  →  Joint 4 (상단링크길이, 와이어 B)
//
// 가상 조인트 인덱스 (this file 내부):
//   ji=0 → joint3,  ji=1 → joint4
//
// lower_link 커플링: motor 0x03 (CanBus0 소유, joint2)의 raw 위치가 필요하다.
// "phys_motor/m03/state" 를 구독해 motor03_pos_로 캐시한다 (CanBus0가 모터 공간 raw 값을 publish).
//
// 모터 회전 방향 보정: config/robotnl.yaml 의 motor_direction_sign[3..6]
// (m04~m07) 을 실제 배선/장착에 맞춰 +1/-1 로 설정. mpos()/mvel()/mtorque()로
// 모터 상태를 읽고, send_motor_cmd()로 명령을 보낼 때 일괄 적용되며,
// 그 외 모든 코드(kin_manipulator.hpp 포함)는 보정된 값을 "정방향"으로 다룬다.
//
// 모터 명령 모드 (gui/motor_raw_mode == true):
//   gui/phys_motor/m0X/cmd, gui/phys_motor/m0X/on 을 직접 구독하여 와이어 장력/커플링 IK를
//   완전히 우회하고, 4개 모터를 각각 독립적으로 raw 위치 제어한다 (실 모터 단위 테스트용).

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

    static constexpr int NM = 4; // 물리 모터 수
    static constexpr int NJ = 2; // 가상 조인트 수 (joint3=ji0, joint4=ji1)

    // 조인트 ji의 가상 조인트 이름 (데이터 채널용)
    static constexpr const char* joint_name[NJ] = {"joint3", "joint4"};

    void initialize(void*) override {
        for (auto& f : joint_on_flag_) f = false;
        for (auto& f : phys_on_flag_)  f = false;
        for (auto& f : effective_on_)  f = false;
        for (auto& s : motor_sync_)    s = false;
        for (auto& p : joint_fb_pos_)  p = 0.0;
        for (auto& c : joint_cmds)     c = {};
        for (auto& i : cmd_interp)     i = {};
        for (auto& c : phys_cmd_cache_) c = {};

        // motor_direction_sign 7개 중 m04~m07 (index 3~6) 사용
        const auto& sign = p_motor_sign.read();
        for (int mi = 0; mi < NM; mi++)
            motor_sign_[mi] = ((int)sign.size() > mi + 3) ? sign[mi + 3] : 1.0;
    }

    void execute(void*) override {
        // 0. 모터 명령 모드 플래그 + lower_link 커플링용 motor 0x03 위치 갱신
        dr_motor_raw_mode_.on_update([&](const bool& v) { motor_raw_mode_ = v; });
        dr_motor03_state_.on_update([&](const custom_types::MotorState& d) {
            motor03_pos_ = d.pos;
            motor03_synced_ = true;
        });

        // 1. 상위 명령 수신 (조인트 경로 + 물리 모터 직접 경로 둘 다 항상 수신)
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

            dr_joint_on[ji].on_update([&, ji](const bool& on) { joint_on_flag_[ji] = on; });
        }

        for (int mi = 0; mi < NM; mi++) {
            dr_phys_cmd_[mi].on_update([&, mi](const custom_types::MotorCmd& data) {
                phys_cmd_wdt_[mi] = getExecutionLocalTick();
                phys_cmd_cache_[mi] = data;
            });
            dr_phys_on_[mi].on_update([&, mi](const bool& on) { phys_on_flag_[mi] = on; });
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
                        rx_count_[mi]++;
                        // 해당 물리 모터의 조인트가 아직 동기화 안 된 경우 처리
                        try_sync_joint(motor_joint(mi));
                        // 물리 모터 상태 publish (GUI 모터 뷰용, 방향 보정 적용됨)
                        custom_types::MotorState phys_st{};
                        phys_st.pos     = mpos(mi);
                        phys_st.vel     = mvel(mi);
                        phys_st.torque  = mtorque(mi);
                        phys_st.enabled = effective_on_[mi];
                        dw_phys_state_[mi].write(phys_st);
                        break;
                    }
                }
            }

            // 피드백 FK → 가상 조인트 상태 발행 (조인트 모드에서만 의미 있음, raw 모드에서도 계속 publish)
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

            // 모드 전환 시 on/off 추적: 현재 모드 기준 effective on 계산 (모터별)
            for (int mi = 0; mi < NM; mi++) {
                const bool target_on = motor_raw_mode_ ? phys_on_flag_[mi] : joint_on_flag_[motor_joint(mi)];
                if (!effective_on_[mi] && target_on) {
                    getLogger()->info("[{}] Motor[{}] ON ({})", getName(), mi, motor_raw_mode_ ? "raw" : "joint");
                    effective_on_[mi] = true;
                    motor_sync_[mi]   = false;
                    cmd_interp[motor_joint(mi)] = {};
                    if (so > 0) motors[mi]->Start(so);
                } else if (effective_on_[mi] && !target_on) {
                    getLogger()->info("[{}] Motor[{}] OFF ({})", getName(), mi, motor_raw_mode_ ? "raw" : "joint");
                    effective_on_[mi] = false;
                    motor_sync_[mi]   = false;
                    cmd_interp[motor_joint(mi)] = {};
                    if (so > 0) motors[mi]->Stop(so);
                }
            }

            if (motor_raw_mode_) {
                // 모터 명령 모드: 와이어 텐션/커플링 IK 완전 우회, 4개 모터 독립 raw 위치 제어
                for (int mi = 0; mi < NM; mi++) {
                    const bool ready = effective_on_[mi] && motor_sync_[mi] &&
                                        (tick - phys_cmd_wdt_[mi] <= 1 * getFrequency());
                    MotorCommand applied{};
                    if (ready) {
                        applied.pos    = phys_cmd_cache_[mi].pos;
                        applied.vel    = phys_cmd_cache_[mi].vel;
                        applied.torque = phys_cmd_cache_[mi].torque;
                        applied.kp     = phys_cmd_cache_[mi].kp;
                        applied.kd     = phys_cmd_cache_[mi].kd;
                    }
                    send_motor_cmd(mi, applied);
                }
            } else {
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
                send_motor_commands(tick);
            }

            update_io_stats();

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
                    for (auto& t : cmd_wdt)       t = getExecutionLocalTick();
                    for (auto& t : com_wdt)       t = getExecutionLocalTick();
                    for (auto& t : phys_cmd_wdt_) t = getExecutionLocalTick();
                    for (auto& m : motors)        m->Start(so, true);
                    for (auto& f : effective_on_) f = false;
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
        can_ifdown_if_ours();
    }

private:
    // ── 조인트↔모터 인덱스 매핑 헬퍼 ──

    // 가상 조인트 ji에 속한 물리 모터 인덱스 목록
    static std::vector<int> joint_motors(int ji) {
        switch (ji) {
            case 0: return {0, 1};     // joint3: motors[0](A,장력), motors[1](B,위치)
            case 1: return {2, 3};     // joint4: motors[2](A), motors[3](B)
            default: return {};
        }
    }

    // 물리 모터 mi가 속한 가상 조인트 ji
    static int motor_joint(int mi) {
        return (mi == 0 || mi == 1) ? 0 : 1; // mi==2||mi==3 → joint4
    }

    // ji의 모든 모터가 동기화됐는지 확인
    // joint3/4 모두 motor 0x03(lower_link, CanBus0 소유) 위치로 커플링 보정을 받으므로,
    // motor03_pos_가 최소 한 번 갱신되기 전에는 동기화 완료로 보지 않는다.
    // (그렇지 않으면 motor03_pos_ 기본값(0)으로 sync된 뒤 실제 값이 늦게 도착할 때
    //  커플링 보정 항이 갑자기 점프해 큰 위치오차/토크가 발생할 수 있음)
    bool joint_fully_synced(int ji) const {
        if (!motor03_synced_) return false;
        for (int mi : joint_motors(ji))
            if (!motor_sync_[mi]) return false;
        return true;
    }

    // 모터 배선/장착 방향 보정 (motor_direction_sign) 적용된 모터 상태 접근자
    double mpos(int mi)    const { return motor_sign_[mi] * motors[mi]->state.pos; }
    double mvel(int mi)    const { return motor_sign_[mi] * motors[mi]->state.vel; }
    double mtorque(int mi) const { return motor_sign_[mi] * motors[mi]->state.torque; }

    // ── 피드백 FK ──

    double compute_joint_fk(int ji) const {
        switch (ji) {
            case 0: // motor B (0x05, motors[1]) 기준, lower_link 커플링 보정
                return kin_manip::joint3_motor_to_joint(mpos(1), motor03_pos_);
            case 1: { // 방향별 active 모터 기준, lower_link 커플링 보정
                double active = joint4_dir_extend_ ? mpos(3) : mpos(2);
                return kin_manip::joint4_motor_to_joint(active, motor03_pos_);
            }
            default: return 0.0;
        }
    }

    double compute_joint_vel_fk(int ji) const {
        switch (ji) {
            case 0: return kin_manip::joint3_vel_motor_to_joint(mvel(1));
            case 1: {
                double active_vel = joint4_dir_extend_ ? mvel(3) : mvel(2);
                return kin_manip::joint4_vel_motor_to_joint(active_vel);
            }
            default: return 0.0;
        }
    }

    double compute_joint_torque_fk(int ji) const {
        switch (ji) {
            case 0: return kin_manip::joint3_torque_motor_to_joint(mtorque(1));
            case 1: {
                double active_tau = joint4_dir_extend_ ? mtorque(3) : mtorque(2);
                return kin_manip::joint4_torque_motor_to_joint(active_tau);
            }
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
                phys_cmd_cache_[mi].pos = mpos(mi); // raw 모드 초기 목표 = 현재 위치
                getLogger()->info("[{}] Motor[{}] synced pos={:.3f}", getName(), mi, mpos(mi));
            }
        }
        // 이 조인트의 모든 모터가 동기화 완료됐을 때 초기 명령을 현재 위치로 설정
        if (!all_synced_before && joint_fully_synced(ji)) {
            joint_fb_pos_[ji] = compute_joint_fk(ji);
            joint_cmds[ji].pos = joint_fb_pos_[ji];
            cmd_interp[ji] = {};
            getLogger()->info("[{}] Joint{} synced at pos={:.4f}", getName(), ji + 3, joint_fb_pos_[ji]);
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
            st.enabled = joint_on_flag_[ji];
            dw_joint_state[ji].write(st);
        }
    }

    // IK → 물리 모터 명령 송신 (조인트 모드)
    void send_motor_commands(uint64_t tick) {
        for (int ji = 0; ji < NJ; ji++) {
            const bool ready = joint_on_flag_[ji] && joint_fully_synced(ji) &&
                               (tick - cmd_wdt[ji] <= 1 * getFrequency());

            if (!ready) {
                // 오프라인 or 준비 안 됨 → 영 명령
                for (int mi : joint_motors(ji))
                    send_motor_cmd(mi, MotorCommand{});
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
            case 0: { // joint3: 팔꿈치 와이어 IK
                // motor A (0x04, motors[0]): 항상 장력 제어 — kp=kd=0, torque=tension
                // motor B (0x05, motors[1]): 항상 위치 제어 — lower_link 커플링 보정 포함
                MotorCommand mc_b{};
                mc_b.pos    = kin_manip::joint3_joint_to_motor_pos(cmd.pos, motor03_pos_);
                mc_b.vel    = kin_manip::joint3_vel_joint_to_motor(cmd.vel);
                mc_b.torque = kin_manip::joint3_torque_joint_to_motor(cmd.torque);
                mc_b.kp     = cmd.kp;
                mc_b.kd     = cmd.kd;

                MotorCommand mc_a{};
                mc_a.pos    = mpos(0);
                mc_a.vel    = 0.0;
                mc_a.torque = kin_manip::ELBOW_TENSION_TORQUE;
                mc_a.kp     = 0.0;
                mc_a.kd     = 0.0;

                send_motor_cmd(0, mc_a);
                send_motor_cmd(1, mc_b);
                break;
            }
            case 1: { // joint4: 상단링크 와이어 IK — 방향별 모터 전환
                // extending (q4↑): motor B (0x07, motors[3]) 위치, motor A (0x06, motors[2]) 장력
                // retracting (q4↓): motor A (0x06, motors[2]) 위치, motor B (0x07, motors[3]) 장력
                bool extending = (cmd.pos >= joint_fb_pos_[1]);
                joint4_dir_extend_ = extending;

                MotorCommand mc_active{};
                mc_active.pos    = kin_manip::joint4_joint_to_motor_pos(cmd.pos, motor03_pos_);
                mc_active.vel    = kin_manip::joint4_vel_joint_to_motor(cmd.vel);
                mc_active.torque = kin_manip::joint4_torque_joint_to_motor(cmd.torque);
                mc_active.kp     = cmd.kp;
                mc_active.kd     = cmd.kd;

                MotorCommand mc_tension{};
                mc_tension.pos    = extending ? mpos(2) : mpos(3);
                mc_tension.vel    = 0.0;
                mc_tension.torque = kin_manip::UPPER_TENSION_TORQUE;
                mc_tension.kp     = 0.0;
                mc_tension.kd     = 0.0;

                if (extending) {
                    send_motor_cmd(2, mc_tension);
                    send_motor_cmd(3, mc_active);
                } else {
                    send_motor_cmd(2, mc_active);
                    send_motor_cmd(3, mc_tension);
                }
                break;
            }
        }
    }

    // ── 데이터 채널 ──
    DataWriter<bool> dw_state{"can1_state", ArchiveOption::Enable};

    // 물리 모터 raw 상태 (GUI 모터 뷰용) — motors[0..3] = m04..m07
    DataWriter<custom_types::MotorState> dw_phys_state_[NM] = {
        DataWriter<custom_types::MotorState>{"phys_motor/m04/state"},
        DataWriter<custom_types::MotorState>{"phys_motor/m05/state"},
        DataWriter<custom_types::MotorState>{"phys_motor/m06/state"},
        DataWriter<custom_types::MotorState>{"phys_motor/m07/state"},
    };

    // 물리 모터 CAN tx/rx 통계 (GUI 모터 뷰 진단용)
    DataWriter<custom_types::MotorIoStats> dw_phys_io_[NM] = {
        DataWriter<custom_types::MotorIoStats>{"phys_motor/m04/io"},
        DataWriter<custom_types::MotorIoStats>{"phys_motor/m05/io"},
        DataWriter<custom_types::MotorIoStats>{"phys_motor/m06/io"},
        DataWriter<custom_types::MotorIoStats>{"phys_motor/m07/io"},
    };
    // 물리 모터에 실제로 적용된 명령 echo (모터 공간) — GUI Sync가 "현재 값"을 보여주는 데 사용
    DataWriter<custom_types::MotorCmd> dw_phys_cmd_applied_[NM] = {
        DataWriter<custom_types::MotorCmd>{"phys_motor/m04/cmd_applied"},
        DataWriter<custom_types::MotorCmd>{"phys_motor/m05/cmd_applied"},
        DataWriter<custom_types::MotorCmd>{"phys_motor/m06/cmd_applied"},
        DataWriter<custom_types::MotorCmd>{"phys_motor/m07/cmd_applied"},
    };

    DataWriter<custom_types::MotorState> dw_joint_state[NJ] = {
        DataWriter<custom_types::MotorState>{"joint3/state", ArchiveOption::Enable},
        DataWriter<custom_types::MotorState>{"joint4/state", ArchiveOption::Enable},
    };
    DataWriter<custom_types::MotorCmd> dw_joint_cmd_applied[NJ] = {
        DataWriter<custom_types::MotorCmd>{"joint3/cmd_applied", ArchiveOption::Enable},
        DataWriter<custom_types::MotorCmd>{"joint4/cmd_applied", ArchiveOption::Enable},
    };
    DataReader<custom_types::MotorCmd> dr_joint_cmd[NJ] = {
        DataReader<custom_types::MotorCmd>{"joint3/cmd"},
        DataReader<custom_types::MotorCmd>{"joint4/cmd"},
    };
    DataReader<bool> dr_joint_on[NJ] = {
        DataReader<bool>{"joint3/on"},
        DataReader<bool>{"joint4/on"},
    };

    // ── lower_link 커플링용 motor 0x03 raw 위치 (CanBus0 소유) ──
    DataReader<custom_types::MotorState> dr_motor03_state_{"phys_motor/m03/state", DependencyType::Weak};
    double motor03_pos_ = 0.0;
    bool   motor03_synced_ = false; // CanBus0로부터 m03 raw 위치를 한 번이라도 받았는지

    // ── 모터 명령 모드 (IK 우회, 모터 공간 직접 명령) ──
    DataReader<bool> dr_motor_raw_mode_{"gui/motor_raw_mode", DependencyType::Weak};
    DataReader<custom_types::MotorCmd> dr_phys_cmd_[NM] = {
        DataReader<custom_types::MotorCmd>{"gui/phys_motor/m04/cmd", DependencyType::Weak},
        DataReader<custom_types::MotorCmd>{"gui/phys_motor/m05/cmd", DependencyType::Weak},
        DataReader<custom_types::MotorCmd>{"gui/phys_motor/m06/cmd", DependencyType::Weak},
        DataReader<custom_types::MotorCmd>{"gui/phys_motor/m07/cmd", DependencyType::Weak},
    };
    DataReader<bool> dr_phys_on_[NM] = {
        DataReader<bool>{"gui/phys_motor/m04/on", DependencyType::Weak},
        DataReader<bool>{"gui/phys_motor/m05/on", DependencyType::Weak},
        DataReader<bool>{"gui/phys_motor/m06/on", DependencyType::Weak},
        DataReader<bool>{"gui/phys_motor/m07/on", DependencyType::Weak},
    };

    Parameter<std::string> p_port{"can1.port", "can1"};
    // 모터 방향 보정 (m04~m07) — 실제 배선/장착에 맞춰 +1/-1
    Parameter<std::vector<double>> p_motor_sign{"motor_direction_sign"};

    // ── 모터 ──
    std::vector<std::shared_ptr<Motor>> motors = {
        std::make_shared<MyActuatorX6>(0x04), // joint3: 와이어 A (장력)
        std::make_shared<MyActuatorX6>(0x05), // joint3: 와이어 B (위치)
        std::make_shared<MyActuatorX6>(0x06), // joint4: 와이어 A
        std::make_shared<MyActuatorX6>(0x07), // joint4: 와이어 B
    };

    double motor_sign_[NM] = {1.0, 1.0, 1.0, 1.0};

    // ── 내부 상태 ──
    struct MotorCmdInterp {
        bool   active      = false;
        double start_ref   = 0.0;
        double target_pos  = 0.0;
        double duration_ms = 0.0;
        double elapsed_ms  = 0.0;
    };

    bool   motor_raw_mode_ = false; // gui/motor_raw_mode 캐시
    std::array<MotorCmdInterp, NJ>           cmd_interp{};
    std::array<MotorCommand, NJ>             joint_cmds{};       // 조인트 모드 명령 (joint 공간)
    std::array<custom_types::MotorCmd, NM>   phys_cmd_cache_{}; // raw 모드 명령 (모터 공간)
    std::array<double, NJ>                   joint_fb_pos_{};
    bool   joint_on_flag_[NJ] = {}; // joint3~4/on 캐시
    bool   phys_on_flag_[NM]  = {}; // gui/phys_motor/m0X/on 캐시
    bool   effective_on_[NM]  = {}; // 현 모드 기준 실제 적용된 on 상태
    bool   motor_sync_[NM]    = {};
    bool   joint4_dir_extend_ = true;  // true: extending (motor B active), false: retracting (motor A active)
    int    cmd_wdt[NJ]        = {};
    int    phys_cmd_wdt_[NM]  = {};
    int    com_wdt[NM]        = {};
    int    can_wdt            = 0;
    int    so                 = -1;
    bool   link_initialized   = false;
    bool   restart_required   = false;
    bool   link_brought_up_by_us_ = false; // 이 프로세스가 직접 'ip link up' 했는지
    std::string opened_port_;

    // ── CAN tx/rx 통계 (GUI 모터 뷰 진단용) ──
    uint32_t tx_count_[NM]      = {};
    uint32_t rx_count_[NM]      = {};
    uint32_t tx_count_prev_[NM] = {};
    uint32_t rx_count_prev_[NM] = {};
    double   tx_hz_[NM]         = {};
    double   rx_hz_[NM]         = {};
    std::chrono::steady_clock::time_point io_window_start_ = std::chrono::steady_clock::now();

    // cmd는 모터 공간(raw) 기준 — 모터로 보내기 직전 motor_sign_ 으로 되돌림
    void send_motor_cmd(int mi, const MotorCommand& cmd) {
        custom_types::MotorCmd echo{};
        echo.pos = cmd.pos; echo.vel = cmd.vel; echo.torque = cmd.torque; echo.kp = cmd.kp; echo.kd = cmd.kd;
        dw_phys_cmd_applied_[mi].write(echo);

        MotorCommand raw = cmd;
        raw.pos    *= motor_sign_[mi];
        raw.vel    *= motor_sign_[mi];
        raw.torque *= motor_sign_[mi];
        motors[mi]->Control(so, raw);
        tx_count_[mi]++;
    }

    void update_io_stats() {
        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - io_window_start_).count();
        if (elapsed >= 1.0) {
            for (int mi = 0; mi < NM; mi++) {
                tx_hz_[mi] = (tx_count_[mi] - tx_count_prev_[mi]) / elapsed;
                rx_hz_[mi] = (rx_count_[mi] - rx_count_prev_[mi]) / elapsed;
                tx_count_prev_[mi] = tx_count_[mi];
                rx_count_prev_[mi] = rx_count_[mi];
            }
            io_window_start_ = now;
        }
        for (int mi = 0; mi < NM; mi++) {
            custom_types::MotorIoStats io{};
            io.tx_count = tx_count_[mi];
            io.rx_count = rx_count_[mi];
            io.tx_hz    = tx_hz_[mi];
            io.rx_hz    = rx_hz_[mi];
            dw_phys_io_[mi].write(io);
        }
    }

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
        for (auto& t : cmd_wdt)       t = 0;
        for (auto& t : com_wdt)       t = 0;
        for (auto& t : phys_cmd_wdt_) t = 0;
        for (auto& m : motors)        m->state.online = false;
        for (auto& s : motor_sync_)   s = false;
        for (auto& i : cmd_interp)    i = {};
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
                link_initialized       = true;
                link_brought_up_by_us_ = true;
            }
        } else if (restart_required) {
            run_ip_command(port, "restart", false);
        }
        opened_port_ = port;

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

    // 이 프로세스가 직접 활성화했던 인터페이스만 종료 시 비활성화
    void can_ifdown_if_ours() {
        if (link_brought_up_by_us_ && !opened_port_.empty()) {
            run_ip_command(opened_port_.c_str(), "down", false);
            getLogger()->info("[{}] {} down", getName(), opened_port_);
            link_brought_up_by_us_ = false;
        }
    }
};

} // namespace task_pool
