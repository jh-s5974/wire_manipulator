#pragma once

// ═══════════════════════════════════════════════════════════════════════════
// CanBus0 태스크 — Joint 0(base_yaw) · Joint 1(pitch) · Joint 2(lower_link), CAN0 60Hz
// ═══════════════════════════════════════════════════════════════════════════
//
// 역할: joint0~2 의 가상 조인트 명령(jointN/cmd)을 받아 모터 명령으로 바꿔 CAN0에 송신하고,
//       모터 피드백을 가상 조인트 상태(jointN/state)로 발행한다. GUI "모터 뷰"용 raw 모드도 지원.
//
// 조인트 ↔ 모터 (1:1, joint2만 볼스크류 변환):
//   joint0: base_yaw   — RobStride03  0x01, revolute,  모터=조인트 1:1
//   joint1: pitch      — RobStride03  0x02, revolute,  모터=조인트 1:1
//   joint2: lower_link — MyActuatorX6 0x03, prismatic, 볼스크류 변환 (kin_manip::joint2_*)
//
// 호스트 보간: joint0/1(RobStride, MIT류)은 모터 자체 속도제한이 없어 호스트가 duration 보간을
//   돌린다. joint2(MyActuatorX6)는 0xA4 위치모드가 자체 속도제한을 돌리므로 보간 없이 즉시 적용.
//
// 모터 방향 보정: config/robotnl.yaml 의 motor_direction_sign[0..2](m01/m02/m03)를 배선/장착에
//   맞춰 +1/-1 로 설정. 상태를 읽는 시점과 send_motor_cmd() 송신 시점에 적용되며, 그 외 모든
//   코드는 보정된 값을 "정방향"으로 다룬다.
//
// phys_motor/m03/state: 항상 "모터 공간"(raw, 볼스크류 변환 전) 위치를 publish —
//   CanBus1(joint3/4)이 lower_link 커플링 보정에 이 값을 사용한다.
//
// raw 모드 (gui/motor_raw_mode == true):
//   gui/phys_motor/m0X/cmd · /on 을 직접 구독해 조인트 IK(볼스크류 변환)를 우회하고
//   모터 raw 위치/속도/토크 명령을 그대로 송신한다 (실 모터 단위 테스트용).

#include <rtfw/task.h>
#include "../util.hpp"
#include "../custom_types.hpp"
#include "../kin_manipulator.hpp"
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

    static constexpr int N = 3; // 물리 모터 수 = 가상 조인트 수 (직접 구동, joint2만 볼스크류 변환)

    void initialize(void*) override {
        for (auto& f : joint_on_flag_)   f = false;
        for (auto& f : phys_on_flag_)    f = false;
        for (auto& f : effective_on_)    f = false;
        for (auto& s : motor_sync_)      s = false;
        for (auto& p : joint_fb_pos_)    p = 0.0;
        for (auto& c : cmds)             c = {};
        for (auto& i : cmd_interp)       i = {};
        for (auto& i : phys_cmd_interp_) i = {};
        for (auto& c : phys_cmd_cache_)  c = {};

        const auto& sign = p_motor_sign.read();
        for (int i = 0; i < N; i++)
            motor_sign_[i] = ((int)sign.size() > i) ? sign[i] : 1.0;
    }

    void execute(void*) override {
        // 0. 모터 명령 모드 플래그 갱신
        dr_motor_raw_mode_.on_update([&](const bool& v) {
            if (v != motor_raw_mode_) {
                for (auto& i : cmd_interp)       i = {};
                for (auto& i : phys_cmd_interp_) i = {};
            }
            motor_raw_mode_ = v;
        });

        // 1. 상위 명령 수신 (조인트 경로 + 물리 모터 직접 경로 둘 다 항상 수신)
        for (int i = 0; i < N; i++) {
            dr_joint_cmd[i].on_update([&, i](const custom_types::MotorCmd& data) {
                cmd_wdt[i] = getExecutionLocalTick();
                // joint2(i==2, MyActuatorX6)는 0xA4 자체가 maxSpeed 기반 속도제한 PI를 돌기 때문에
                // 호스트에서 또 보간하면 두 속도제한이 간섭해 목표가 들쭉날쭉해진다 — 그래서
                // 목표값을 즉시 적용하고 속도는 joint_default_speed(yaml)로 조절한다.
                // joint0/1(RobStride, MIT류)는 자체 속도제한이 없어 호스트 보간을 그대로 사용.
                //
                // 보간 시작점(start_ref)은 "피드백 위치"가 아니라 "마지막 명령값(cmds[i].pos)"이다.
                // SafetyLayer가 SL_MANUAL에서 같은 명령을 200Hz로 재전달하므로(보간 안 함),
                // start_ref를 피드백으로 두면: 보간 완료 후에도 재전달 때마다 피드백→목표 램프가
                // 재시작되어 게인이 낮아 모터가 0.9에 멈추면 명령이 0.9→1.0을 무한 반복한다.
                // start_ref=마지막 명령값이면: 같은 목표 재전달 시 start==target이라 램프가 상수가
                // 되어(움직임 없음) 반복이 사라지고, 진짜 새 목표일 때만 마지막 명령값에서 램프한다.
                if (i != 2 && data.duration_ms > 0.0) {
                    if (!cmd_interp[i].active ||
                        std::abs(data.pos - cmd_interp[i].target_pos) > 1e-4) {
                        cmd_interp[i].start_ref   = cmds[i].pos;
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

            dr_joint_on[i].on_update([&, i](const bool& on) { joint_on_flag_[i] = on; });

            dr_phys_cmd_[i].on_update([&, i](const custom_types::MotorCmd& data) {
                phys_cmd_wdt_[i] = getExecutionLocalTick();
                if (data.duration_ms > 0.0) {
                    if (!phys_cmd_interp_[i].active ||
                        std::abs(data.pos - phys_cmd_interp_[i].target_pos) > 1e-4) {
                        // start_ref은 피드백이 아니라 마지막 명령값(phys_cmd_cache_)에서 출발 —
                        // 같은 명령 200Hz 재전달 시 start==target이라 보간이 상수가 되어
                        // 게인이 낮아 모터가 목표에 못 미쳐도 명령이 반복 진동하지 않는다.
                        phys_cmd_interp_[i].start_ref   = phys_cmd_cache_[i].pos;
                        phys_cmd_interp_[i].target_pos  = data.pos;
                        phys_cmd_interp_[i].duration_ms = data.duration_ms;
                        phys_cmd_interp_[i].elapsed_ms  = 0.0;
                        phys_cmd_interp_[i].active       = true;
                    }
                } else {
                    phys_cmd_interp_[i].active = false;
                    phys_cmd_cache_[i].pos = data.pos;
                }
                phys_cmd_cache_[i].vel    = data.vel;
                phys_cmd_cache_[i].torque = data.torque;
                phys_cmd_cache_[i].kp     = data.kp;
                phys_cmd_cache_[i].kd     = data.kd;
            });

            dr_phys_on_[i].on_update([&, i](const bool& on) { phys_on_flag_[i] = on; });

            // 영점 설정 트리거(one-shot) — 모터가 OFF 일 때만 허용(ON 상태에서 0점을 바꾸면
            // 현재 명령이 새 0점 기준으로 재해석되어 급격히 움직일 수 있다). 시퀀스 진행은 아래 송신부.
            dr_set_zero_[i].on_update([&, i]() {
                if (effective_on_[i]) {
                    getLogger()->warn("[{}] Motor {} 영점 무시: 모터 ON 상태 (먼저 OFF 하세요)", getName(), i);
                    return;
                }
                if (zero_step_[i] == 0) {
                    zero_step_[i] = 1;
                    getLogger()->info("[{}] Motor {} 영점 설정 시작", getName(), i);
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
                        rx_count_[i]++;

                        // 모터 배선/장착 방향 보정 (motor_direction_sign) — 모터 공간(raw) 값
                        const double raw_pos   = motor_sign_[i] * motors[i]->state.pos;
                        const double raw_vel   = motor_sign_[i] * motors[i]->state.vel;
                        const double raw_torque= motor_sign_[i] * motors[i]->state.torque;
                        const double joint_pos = motor_to_joint_pos(i, raw_pos);

                        // hasValidPosition(): MyActuatorX6는 0x92 절대위치 응답을 받기 전엔 false —
                        // MIT의 windowed(±12.5rad) 값으로 잘못 동기화되는 것을 방지
                        if (!motor_sync_[i] && motors[i]->hasValidPosition()) {
                            joint_fb_pos_[i] = joint_pos;
                            cmds[i].pos      = joint_fb_pos_[i];
                            cmd_interp[i]    = {};
                            phys_cmd_cache_[i].pos = raw_pos; // raw 모드 초기 목표 = 현재 위치
                            motor_sync_[i]   = true;
                            getLogger()->info("[{}] Joint {} synced pos={:.3f}", getName(), i, joint_pos);
                        }

                        // 가상 조인트 상태 (joint 공간, 볼스크류 변환 적용)
                        custom_types::MotorState st{};
                        st.pos     = joint_pos;
                        st.vel     = motor_to_joint_vel(i, raw_vel);
                        st.torque  = motor_to_joint_torque(i, raw_torque);
                        st.status  = motors[i]->state.status;
                        st.enabled = effective_on_[i];
                        dw_joint_state[i].write(st);

                        // 물리 모터 raw 상태 (GUI 모터 뷰 + CanBus1 lower_link 커플링용, 모터 공간)
                        custom_types::MotorState phys_st{};
                        phys_st.pos     = raw_pos;
                        phys_st.vel     = raw_vel;
                        phys_st.torque  = raw_torque;
                        phys_st.status  = motors[i]->state.status;
                        phys_st.enabled = effective_on_[i];
                        dw_phys_state_[i].write(phys_st);

                        joint_fb_pos_[i] = joint_pos;
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
                        phys_cmd_interp_[i] = {};
                        getLogger()->warn("[{}] Joint {} timeout, re-sync required", getName(), i);
                    }
                }

                // 모드 전환 시 on/off 추적: 현재 모드 기준 effective on 계산
                const bool target_on = motor_raw_mode_ ? phys_on_flag_[i] : joint_on_flag_[i];
                if (!effective_on_[i] && target_on) {
                    getLogger()->info("[{}] Motor {} ON ({})", getName(), i, motor_raw_mode_ ? "raw" : "joint");
                    effective_on_[i] = true;
                    motor_sync_[i]   = false;
                    cmd_interp[i]    = {};
                    phys_cmd_interp_[i] = {};
                    if (so > 0) motors[i]->Start(so);
                } else if (effective_on_[i] && !target_on) {
                    getLogger()->info("[{}] Motor {} OFF ({})", getName(), i, motor_raw_mode_ ? "raw" : "joint");
                    effective_on_[i] = false;
                    motor_sync_[i]   = false;
                    cmd_interp[i]    = {};
                    phys_cmd_interp_[i] = {};
                    if (so > 0) motors[i]->Stop(so);
                }

                MotorCommand applied{}; // 모터 공간(raw) 명령

                if (motor_raw_mode_) {
                    if (phys_cmd_interp_[i].active) {
                        phys_cmd_interp_[i].elapsed_ms += dt_ms;
                        const double ratio = std::min(phys_cmd_interp_[i].elapsed_ms / phys_cmd_interp_[i].duration_ms, 1.0);
                        phys_cmd_cache_[i].pos = phys_cmd_interp_[i].start_ref +
                            ratio * (phys_cmd_interp_[i].target_pos - phys_cmd_interp_[i].start_ref);
                        if (ratio >= 1.0) phys_cmd_interp_[i].active = false;
                    }

                    const bool ready = effective_on_[i] && motor_sync_[i] &&
                                        (tick - phys_cmd_wdt_[i] <= 1 * getFrequency());
                    if (ready) {
                        applied.pos    = phys_cmd_cache_[i].pos;
                        applied.vel    = phys_cmd_cache_[i].vel;
                        applied.torque = phys_cmd_cache_[i].torque;
                        applied.kp     = phys_cmd_cache_[i].kp;
                        applied.kd     = phys_cmd_cache_[i].kd;
                    }
                } else {
                    if (cmd_interp[i].active) {
                        cmd_interp[i].elapsed_ms += dt_ms;
                        const double ratio = std::min(cmd_interp[i].elapsed_ms / cmd_interp[i].duration_ms, 1.0);
                        cmds[i].pos = cmd_interp[i].start_ref + ratio * (cmd_interp[i].target_pos - cmd_interp[i].start_ref);
                        if (ratio >= 1.0) cmd_interp[i].active = false;
                    }

                    const bool ready = effective_on_[i] && motor_sync_[i] && (tick - cmd_wdt[i] <= 1 * getFrequency());
                    if (ready) {
                        // joint2(i==2)는 MyActuatorX6 위치모드(0xA4)를 쓰는데, velocity=0이면
                        // 프로토콜상 "속도제한 없음"이 되어 위험하다 — GUI가 안 줬으면 yaml 기본값으로 대체
                        double vel_cmd = cmds[i].vel;
                        if (i == 2 && vel_cmd == 0.0) {
                            const auto& def_speed = p_default_speed.read();
                            if ((int)def_speed.size() > i) vel_cmd = def_speed[i];
                        }
                        applied.pos    = joint_to_motor_pos(i, cmds[i].pos);
                        applied.vel    = joint_to_motor_vel(i, vel_cmd);
                        applied.torque = joint_to_motor_torque(i, cmds[i].torque);
                        applied.kp     = cmds[i].kp;
                        applied.kd     = cmds[i].kd;
                    }

                    custom_types::MotorCmd applied_msg{};
                    applied_msg.pos    = ready ? cmds[i].pos    : 0.0;
                    applied_msg.vel    = ready ? cmds[i].vel    : 0.0;
                    applied_msg.torque = ready ? cmds[i].torque : 0.0;
                    applied_msg.kp     = ready ? cmds[i].kp     : 0.0;
                    applied_msg.kd     = ready ? cmds[i].kd     : 0.0;
                    dw_joint_cmd_applied[i].write(applied_msg);
                }

                send_motor_cmd(i, applied); // 모터 방향 보정은 send_motor_cmd 내부에서 처리
            }

            // 영점 설정 시퀀스(모터별) — zero_step_>0인 모터만 동작. send_motor_cmd 는 zero_step_>0이면
            // 일반 제어 프레임을 보내지 않으므로(아래 가드), 해당 모터엔 영점/리셋 프레임만 나간다.
            for (int i = 0; i < N; i++) run_zero_sequence(i);

            // 0x92 위치조회: N대 제어명령 전송 완료 후 라운드로빈으로 1대씩 전송.
            // A4 제어명령 직후 0x92를 같은 모터에 보내면 펌웨어가 무시하므로, 제어 프레임들
            // 뒤로 분리해 간격을 확보한다. RobStride(0x01/0x02)의 RequestExtra는 no-op이라
            // 실제 0x92는 MyActuatorX6(joint2/0x03)에만 전송된다. (can_bus1과 동일 방식)
            {
                int q_i = (int)(tick % (uint64_t)N);
                if (!motors[q_i]->RequestExtra(so)) tx_fail_count_[q_i]++;
            }

            update_io_stats();

            PERIODIC_CALL(
                for (int i = 0; i < N; i++) {
                    if (!motors[i]->state.online)
                        getLogger()->warn("[{}] Motor {} offline", getName(), motors[i]->id);
                    else if (effective_on_[i] && !motors[i]->hasValidPosition())
                        getLogger()->warn("[{}] Motor {} online이지만 0x92 위치 응답을 못 받음 — 동기화 불가", getName(), motors[i]->id);
                    if (tx_fail_count_[i] > 0) {
                        getLogger()->warn("[{}] Motor {} CAN TX 드랍 {}회 (버퍼 과부하 가능성)", getName(), i, tx_fail_count_[i]);
                        tx_fail_count_[i] = 0;
                    }
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
                    for (auto& t : cmd_wdt)      t = getExecutionLocalTick();
                    for (auto& t : com_wdt)      t = getExecutionLocalTick();
                    for (auto& t : phys_cmd_wdt_) t = getExecutionLocalTick();
                    for (auto& m : motors)       m->Start(so, true);
                    for (auto& f : effective_on_) f = false;
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
        can_ifdown_if_ours();
    }

private:
    struct MotorCmdInterp {
        bool   active      = false;
        double start_ref   = 0.0;
        double target_pos  = 0.0;
        double duration_ms = 0.0;
        double elapsed_ms  = 0.0;
    };

    // ── joint2(인덱스 2)만 볼스크류 단위 변환, 그 외(joint0/1)는 모터=조인트 1:1 ──
    static double motor_to_joint_pos(int i, double v)    { return (i == 2) ? kin_manip::joint2_motor_to_joint(v)    : v; }
    static double joint_to_motor_pos(int i, double v)    { return (i == 2) ? kin_manip::joint2_joint_to_motor(v)    : v; }
    static double motor_to_joint_vel(int i, double v)    { return (i == 2) ? kin_manip::joint2_vel_motor_to_joint(v): v; }
    static double joint_to_motor_vel(int i, double v)    { return (i == 2) ? kin_manip::joint2_vel_joint_to_motor(v): v; }
    static double motor_to_joint_torque(int i, double v) { return (i == 2) ? kin_manip::joint2_torque_motor_to_joint(v) : v; }
    static double joint_to_motor_torque(int i, double v) { return (i == 2) ? kin_manip::joint2_force_joint_to_motor(v)  : v; }

    // ── 데이터 채널 ──
    DataWriter<bool> dw_state{"can0_state", ArchiveOption::Enable};

    DataWriter<custom_types::MotorState> dw_joint_state[N] = {
        DataWriter<custom_types::MotorState>{"joint0/state", ArchiveOption::Enable},
        DataWriter<custom_types::MotorState>{"joint1/state", ArchiveOption::Enable},
        DataWriter<custom_types::MotorState>{"joint2/state", ArchiveOption::Enable},
    };
    // 물리 모터 raw 상태 (GUI 모터 뷰용, 모터 공간)
    DataWriter<custom_types::MotorState> dw_phys_state_[N] = {
        DataWriter<custom_types::MotorState>{"phys_motor/m01/state"},
        DataWriter<custom_types::MotorState>{"phys_motor/m02/state"},
        DataWriter<custom_types::MotorState>{"phys_motor/m03/state"},
    };
    // 물리 모터 CAN tx/rx 통계 (GUI 모터 뷰 진단용)
    DataWriter<custom_types::MotorIoStats> dw_phys_io_[N] = {
        DataWriter<custom_types::MotorIoStats>{"phys_motor/m01/io"},
        DataWriter<custom_types::MotorIoStats>{"phys_motor/m02/io"},
        DataWriter<custom_types::MotorIoStats>{"phys_motor/m03/io"},
    };
    // 물리 모터에 실제로 적용된 명령 echo (모터 공간) — GUI Sync가 "현재 값"을 보여주는 데 사용
    DataWriter<custom_types::MotorCmd> dw_phys_cmd_applied_[N] = {
        DataWriter<custom_types::MotorCmd>{"phys_motor/m01/cmd_applied"},
        DataWriter<custom_types::MotorCmd>{"phys_motor/m02/cmd_applied"},
        DataWriter<custom_types::MotorCmd>{"phys_motor/m03/cmd_applied"},
    };
    DataWriter<custom_types::MotorCmd> dw_joint_cmd_applied[N] = {
        DataWriter<custom_types::MotorCmd>{"joint0/cmd_applied", ArchiveOption::Enable},
        DataWriter<custom_types::MotorCmd>{"joint1/cmd_applied", ArchiveOption::Enable},
        DataWriter<custom_types::MotorCmd>{"joint2/cmd_applied", ArchiveOption::Enable},
    };
    DataReader<custom_types::MotorCmd> dr_joint_cmd[N] = {
        DataReader<custom_types::MotorCmd>{"joint0/cmd"},
        DataReader<custom_types::MotorCmd>{"joint1/cmd"},
        DataReader<custom_types::MotorCmd>{"joint2/cmd"},
    };
    DataReader<bool> dr_joint_on[N] = {
        DataReader<bool>{"joint0/on"},
        DataReader<bool>{"joint1/on"},
        DataReader<bool>{"joint2/on"},
    };

    // ── 모터 명령 모드 (IK 우회, 모터 공간 직접 명령) ──
    DataReader<bool> dr_motor_raw_mode_{"gui/motor_raw_mode", DependencyType::Weak};
    DataReader<custom_types::MotorCmd> dr_phys_cmd_[N] = {
        DataReader<custom_types::MotorCmd>{"gui/phys_motor/m01/cmd", DependencyType::Weak},
        DataReader<custom_types::MotorCmd>{"gui/phys_motor/m02/cmd", DependencyType::Weak},
        DataReader<custom_types::MotorCmd>{"gui/phys_motor/m03/cmd", DependencyType::Weak},
    };
    DataReader<bool> dr_phys_on_[N] = {
        DataReader<bool>{"gui/phys_motor/m01/on", DependencyType::Weak},
        DataReader<bool>{"gui/phys_motor/m02/on", DependencyType::Weak},
        DataReader<bool>{"gui/phys_motor/m03/on", DependencyType::Weak},
    };
    // 영점 설정 트리거(one-shot) — 인덱스 = motors[] 순서 (m01, m02, m03)
    DataReader<rt::Signal> dr_set_zero_[N] = {
        DataReader<rt::Signal>{"gui/phys_motor/m01/set_zero", DependencyType::Weak},
        DataReader<rt::Signal>{"gui/phys_motor/m02/set_zero", DependencyType::Weak},
        DataReader<rt::Signal>{"gui/phys_motor/m03/set_zero", DependencyType::Weak},
    };

    Parameter<std::string> p_port{"can0.port", "can0"};
    // 모터 방향 보정 (m01, m02, m03) — 실제 배선/장착에 맞춰 +1/-1
    Parameter<std::vector<double>> p_motor_sign{"motor_direction_sign"};
    // joint2 위치모드(0xA4)에서 velocity 미지정(0) 시 대체할 기본 속도 [rad/s 또는 m/s]
    Parameter<std::vector<double>> p_default_speed{"joint_default_speed"};

    // ── 모터 ──
    std::vector<std::shared_ptr<Motor>> motors = {
        std::make_shared<RobStride03>(0x01),   // joint0: base_yaw
        std::make_shared<RobStride03>(0x02),   // joint1: pitch
        std::make_shared<MyActuatorX6>(0x03),  // joint2: lower_link (볼스크류)
    };

    double motor_sign_[N] = {1.0, 1.0, 1.0};

    // ── 내부 상태 ──
    bool   motor_raw_mode_ = false; // gui/motor_raw_mode 캐시
    std::array<MotorCmdInterp, N>            cmd_interp{};
    std::array<MotorCmdInterp, N>            phys_cmd_interp_{}; // raw 모드 보간 (모터 공간)
    std::array<MotorCommand, N>              cmds{};            // 조인트 모드 명령 (joint 공간)
    std::array<custom_types::MotorCmd, N>    phys_cmd_cache_{}; // raw 모드 명령 (모터 공간)
    std::array<double, N>                    joint_fb_pos_{};
    bool   joint_on_flag_[N] = {}; // joint0~2/on 캐시
    bool   phys_on_flag_[N]  = {}; // gui/phys_motor/m0X/on 캐시
    bool   effective_on_[N]  = {}; // 현 모드 기준 실제 적용된 on 상태
    bool   motor_sync_[N]    = {};
    int    cmd_wdt[N]        = {};
    int    phys_cmd_wdt_[N]  = {};
    int    com_wdt[N]        = {};
    int    can_wdt           = 0;
    int    so                = -1;
    bool   link_initialized   = false;
    bool   restart_required   = false;
    bool   link_brought_up_by_us_ = false; // 이 프로세스가 직접 'ip link up' 했는지
    std::string opened_port_;

    // ── CAN tx/rx 통계 (GUI 모터 뷰 진단용) ──
    uint32_t tx_count_[N]      = {};
    uint32_t tx_fail_count_[N] = {}; // write() 실패(TX 버퍼 과부하로 드랍) 누적 카운트
    uint32_t rx_count_[N]      = {};
    uint32_t tx_count_prev_[N] = {};
    uint32_t rx_count_prev_[N] = {};
    double   tx_hz_[N]         = {};
    double   rx_hz_[N]         = {};
    std::chrono::steady_clock::time_point io_window_start_ = std::chrono::steady_clock::now();

    // 영점 설정 진행 상태(모터별). 0=대기, >0=시퀀스 진행 카운터(send_motor_cmd 억제).
    int zero_step_[N] = {};

    // 영점 설정 시퀀스(모터별 상태머신). 60Hz 기준 step 1에서 영점 저장 프레임을 보내고,
    // MyActuatorX6는 ROM 기록이 안정되도록 ~100ms 뒤 리셋(0x76)을 보낸다(RobStride는 즉시 적용이라 no-op).
    // 완료 시 motor_sync_=false 로 강제 재동기화 — 0점 기준이 바뀌었으니 캐시된 명령을 재시드해야 한다.
    static constexpr int kZeroResetStep = 7;  // step 1(0x64) 후 ~100ms 뒤 0x76
    static constexpr int kZeroDoneStep  = 9;  // 시퀀스 종료
    void run_zero_sequence(int i) {
        if (zero_step_[i] == 0) return;
        if (zero_step_[i] == 1) {
            if (motors[i]->SetZero(so)) { tx_count_[i]++; getLogger()->info("[{}] Motor {} SetZero 프레임 전송", getName(), i); }
            else getLogger()->warn("[{}] Motor {} SetZero 미지원/전송 실패", getName(), i);
        } else if (zero_step_[i] == kZeroResetStep) {
            if (motors[i]->Reset(so)) { tx_count_[i]++; getLogger()->info("[{}] Motor {} Reset(재시작) 프레임 전송", getName(), i); }
        }
        if (++zero_step_[i] > kZeroDoneStep) {
            zero_step_[i] = 0;
            motor_sync_[i]      = false; // 0점 기준 변경 → 재동기화(명령 재시드)
            cmd_interp[i]       = {};
            phys_cmd_interp_[i] = {};
            getLogger()->info("[{}] Motor {} 영점 설정 완료", getName(), i);
        }
    }

    // cmd는 모터 공간(raw) 기준 — 모터로 보내기 직전 motor_sign_ 으로 되돌림
    void send_motor_cmd(int i, const MotorCommand& cmd) {
        if (zero_step_[i] > 0) return; // 영점 시퀀스 중에는 일반 제어 프레임을 보내지 않음
        custom_types::MotorCmd echo{};
        echo.pos = cmd.pos; echo.vel = cmd.vel; echo.torque = cmd.torque; echo.kp = cmd.kp; echo.kd = cmd.kd;
        dw_phys_cmd_applied_[i].write(echo);

        MotorCommand raw = cmd;
        raw.pos    *= motor_sign_[i];
        raw.vel    *= motor_sign_[i];
        raw.torque *= motor_sign_[i];
        // kp>0(위치 추종 의도)이면 멀티턴 절대위치 모드, kp==0(순수 토크)면 MIT 모드로 자동 전환
        // (MyActuatorX6 한정. RobStride03는 이 필드를 무시함)
        raw.position_mode = cmd.kp > 0.0;
        if (!motors[i]->Control(so, raw)) tx_fail_count_[i]++;
        tx_count_[i]++;
        // 0x92 위치조회는 여기서 보내지 않는다 — 제어 명령 직후 같은 모터에 0x92를 보내면
        // 펌웨어가 무시하므로, 제어 루프 종료 후 execute()에서 라운드로빈으로 분리 송신한다.
    }

    void update_io_stats() {
        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - io_window_start_).count();
        if (elapsed >= 1.0) {
            for (int i = 0; i < N; i++) {
                tx_hz_[i] = (tx_count_[i] - tx_count_prev_[i]) / elapsed;
                rx_hz_[i] = (rx_count_[i] - rx_count_prev_[i]) / elapsed;
                tx_count_prev_[i] = tx_count_[i];
                rx_count_prev_[i] = rx_count_[i];
            }
            io_window_start_ = now;
        }
        for (int i = 0; i < N; i++) {
            custom_types::MotorIoStats io{};
            io.tx_count = tx_count_[i];
            io.rx_count = rx_count_[i];
            io.tx_hz    = tx_hz_[i];
            io.rx_hz    = rx_hz_[i];
            dw_phys_io_[i].write(io);
        }
    }

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
