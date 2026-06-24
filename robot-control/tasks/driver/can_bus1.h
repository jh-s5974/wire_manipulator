#pragma once

// ═══════════════════════════════════════════════════════════════════════════
// CanBus1 태스크 — Joint 3(팔꿈치) · Joint 4(상단링크), 와이어 구동 (CAN1, 60Hz)
// ═══════════════════════════════════════════════════════════════════════════
//
// 역할: 두 와이어 구동 조인트의 가상 조인트 명령(joint3/4/cmd)을 받아 IK로 물리 모터
//       명령으로 바꿔 CAN1에 송신하고, 모터 피드백을 FK로 가상 조인트 상태(joint3/4/state)로
//       발행한다. GUI "모터 뷰"용 raw 모드(IK 우회 직접 제어)도 지원한다.
//
// 물리 모터 배치 (motors[] 배열은 "joint 그룹 순서" — CAN ID 오름차순이 아님에 주의):
//   motors[0] = 0x06 → joint3 와이어 A : 장력(토크) 제어 (고정)
//   motors[1] = 0x07 → joint3 와이어 B : 위치 제어 (고정)
//   motors[2] = 0x04 → joint4 m04 : 길이 기준 모터. 길어질 때 위치제어 / 줄어들 때 장력
//   motors[3] = 0x05 → joint4 m05 :              줄어들 때 위치제어 / 길어질 때 장력
//   → joint3 은 역할 고정(B=위치, A=장력). joint4 는 이동 방향에 따라 m04↔m05 역할 교대
//     (목표>현재 길어짐→m04 위치, 목표<현재 줄어듦→m05 위치). 길이 FK 는 항상 m04 기준.
//
// 이 파일 내부의 가상 조인트 인덱스(ji): ji=0 → joint3,  ji=1 → joint4.
//
// lower_link 커플링: joint3/4 IK·FK 는 motor 0x03(lower_link, CanBus0 소유)의 raw 위치가
//   필요하다. "phys_motor/m03/state" 를 구독해 motor03_pos_ 로 캐시한다.
//
// 모터 방향 보정: config/robotnl.yaml 의 motor_direction_sign 을 CAN ID 기준(m04~m07)으로
//   설정한다. mpos()/mvel()/mtorque() 로 상태를 읽고 send_motor_cmd() 로 명령을 보낼 때
//   일괄 적용되며, 그 외 모든 코드(kin_manipulator.hpp 포함)는 보정된 값을 "정방향"으로 다룬다.
//
// raw 모드 (gui/motor_raw_mode == true):
//   gui/phys_motor/m0X/cmd · /on 을 직접 구독해 와이어 장력/커플링 IK를 완전히 우회하고
//   4개 모터를 각각 독립 raw 제어한다 (실 모터 단위 테스트용).

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
        for (auto& c : phys_cmd_cache_) c = {};

        // motor_direction_sign 은 CAN ID 기준 (m01~m07 = index 0~6).
        // 배열 순서가 CAN ID 순서와 다르므로 motors[mi]->id 로 직접 인덱싱한다.
        // (0x04→index3, 0x05→index4, 0x06→index5, 0x07→index6)
        const auto& sign = p_motor_sign.read();
        for (int mi = 0; mi < NM; mi++) {
            int idx = (int)motors[mi]->id - 1;
            motor_sign_[mi] = ((int)sign.size() > idx && idx >= 0) ? sign[idx] : 1.0;
        }
    }

    void execute(void*) override {
        // 0. 모터 명령 모드 플래그 + lower_link 커플링용 motor 0x03 위치 갱신
        dr_motor_raw_mode_.on_update([&](const bool& v) {
            motor_raw_mode_ = v;
        });
        dr_motor03_state_.on_update([&](const custom_types::MotorState& d) {
            motor03_pos_ = d.pos;
            motor03_synced_ = true;
        });

        // 1. 상위 명령 수신 — 가상 조인트 경로(joint3/4)와 물리 모터 직접 경로(raw)를 항상 둘 다 구독.
        //    MyActuatorX6는 모터가 0xA4 위치모드에서 자체 속도제한을 돌리므로 호스트 측 보간은
        //    하지 않고 마지막 명령을 그대로 캐시했다가 매 주기 IK를 거쳐 재전송한다.
        for (int ji = 0; ji < NJ; ji++) {
            dr_joint_cmd[ji].on_update([&, ji](const custom_types::MotorCmd& data) {
                cmd_wdt[ji] = getExecutionLocalTick();   // 명령 워치독 갱신
                joint_cmds[ji].pos    = data.pos;         // 마지막 조인트 명령 캐시
                joint_cmds[ji].vel    = data.vel;
                joint_cmds[ji].torque = data.torque;
                joint_cmds[ji].kp     = data.kp;
                joint_cmds[ji].kd     = data.kd;
            });

            dr_joint_on[ji].on_update([&, ji](const bool& on) { joint_on_flag_[ji] = on; });
        }

        for (int mi = 0; mi < NM; mi++) {
            dr_phys_cmd_[mi].on_update([&, mi](const custom_types::MotorCmd& data) {
                phys_cmd_wdt_[mi]      = getExecutionLocalTick();
                phys_cmd_cache_[mi].pos    = data.pos;
                phys_cmd_cache_[mi].vel    = data.vel;
                phys_cmd_cache_[mi].torque = data.torque;
                phys_cmd_cache_[mi].kp     = data.kp;
                phys_cmd_cache_[mi].kd     = data.kd;
            });
            dr_phys_on_[mi].on_update([&, mi](const bool& on) { phys_on_flag_[mi] = on; });

            // 영점 설정 트리거(one-shot) — 모터가 OFF 일 때만 허용(ON 상태에서 0점을 바꾸면
            // 현재 명령이 새 0점 기준으로 재해석되어 급격히 움직일 수 있다). 시퀀스 진행은 아래 송신부.
            dr_set_zero_[mi].on_update([&, mi]() {
                if (effective_on_[mi]) {
                    getLogger()->warn("[{}] Motor[{}] (0x{:02X}) 영점 무시: 모터 ON 상태 (먼저 OFF 하세요)",
                                      getName(), mi, motors[mi]->id);
                    return;
                }
                if (zero_step_[mi] == 0) {
                    zero_step_[mi] = 1;
                    getLogger()->info("[{}] Motor[{}] (0x{:02X}) 영점 설정 시작", getName(), mi, motors[mi]->id);
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

            // 3. 모터 명령 송신 (인터폴레이션 없음 — A4 위치/A1 토크 직접 명령)
            exec_tick_ = getExecutionLocalTick();
            auto tick = exec_tick_;
            int offline = 0;

            for (int mi = 0; mi < NM; mi++) {
                if (tick - com_wdt[mi] > 1 * getFrequency()) {
                    motors[mi]->state.online = false;
                    offline++;
                    if (motor_sync_[mi]) {
                        motor_sync_[mi] = false;
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
                    if (so > 0) motors[mi]->Start(so);
                } else if (effective_on_[mi] && !target_on) {
                    getLogger()->info("[{}] Motor[{}] OFF ({})", getName(), mi, motor_raw_mode_ ? "raw" : "joint");
                    effective_on_[mi] = false;
                    motor_sync_[mi]   = false;
                    if (so > 0) motors[mi]->Stop(so);
                }
            }

            if (motor_raw_mode_) {
                // 모터 명령 모드: 와이어 텐션/커플링 IK 완전 우회, 4개 모터 독립 raw 제어
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
                // 조인트 모드: IK → 물리 모터 명령 송신 (호스트 인터폴레이션 없음)
                send_motor_commands(tick);
            }

            // 영점 설정 시퀀스(모터별) — zero_step_>0인 모터만 동작. send_motor_cmd 는 zero_step_>0이면
            // 일반 제어 프레임을 보내지 않으므로(아래 가드), 해당 모터엔 영점/리셋 프레임만 나간다.
            for (int mi = 0; mi < NM; mi++) run_zero_sequence(mi);

            // 0x92 위치조회: 4대 제어명령 전송 완료 후 라운드로빈으로 1대씩 전송
            // A4/A1 제어명령 직후 0x92를 같은 모터에 보내면 펌웨어가 0x92를 무시함
            // → 4대 제어명령 사이에 ~3개 CAN 프레임 간격이 생겨 모터가 처리할 시간을 확보
            {
                int q_mi = (int)(exec_tick_ % (uint64_t)NM);
                if (!motors[q_mi]->RequestExtra(so)) tx_fail_count_[q_mi]++;
            }

            update_io_stats();

            PERIODIC_CALL(
                for (int mi = 0; mi < NM; mi++) {
                    if (!motors[mi]->state.online)
                        getLogger()->warn("[{}] Motor[{}] (0x{:02X}) offline", getName(), mi, motors[mi]->id);
                    else if (effective_on_[mi] && !motors[mi]->hasValidPosition())
                        getLogger()->warn("[{}] Motor[{}] online이지만 0x92 위치 응답을 못 받음 — 동기화 불가", getName(), mi);
                    if (tx_fail_count_[mi] > 0) {
                        getLogger()->warn("[{}] Motor[{}] CAN TX 드랍 {}회 (버퍼 과부하 가능성)", getName(), mi, tx_fail_count_[mi]);
                        tx_fail_count_[mi] = 0;
                    }
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
            case 0: return {0, 1};     // joint3: motors[0](A=0x06,장력), motors[1](B=0x07,위치)
            case 1: return {2, 3};     // joint4: motors[2](A=0x04,장력), motors[3](B=0x05,위치)
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

    // 위치 FK: 각 조인트의 위치 제어 모터(B) 위치 + lower_link 커플링 보정 → 조인트 위치
    double compute_joint_fk(int ji) const {
        switch (ji) {
            case 0: // joint3: 위치 모터 B(0x07, motors[1]) 기준
                return kin_manip::joint3_motor_to_joint(mpos(1), motor03_pos_);
            case 1: // joint4: 길이 기준 모터 m04(0x04, motors[2]) — 위치/장력 역할과 무관하게 길이 반영
                return kin_manip::joint4_motor4_to_joint(mpos(2), motor03_pos_);
            default: return 0.0;
        }
    }

    // 속도 FK: 위치 제어 모터(B)의 속도 → 조인트 속도
    double compute_joint_vel_fk(int ji) const {
        switch (ji) {
            case 0: return kin_manip::joint3_vel_motor_to_joint(mvel(1)); // 0x07
            case 1: return kin_manip::joint4_vel_motor4_to_joint(mvel(2)); // 0x04 (길이 기준 모터)
            default: return 0.0;
        }
    }

    // 토크 FK: 위치 제어 모터(B)의 토크 → 조인트 토크
    double compute_joint_torque_fk(int ji) const {
        switch (ji) {
            case 0: return kin_manip::joint3_torque_motor_to_joint(mtorque(1)); // 0x07
            // joint4: 길이 기준 모터 m04 의 토크로 근사(장력 역할일 땐 ~일정). 표시용 추정치.
            case 1: return kin_manip::joint4_torque_motor4_to_joint(mtorque(2)); // 0x04
            default: return 0.0;
        }
    }

    // 피드백 수신 시 조인트 동기화 시도
    void try_sync_joint(int ji) {
        bool all_synced_before = joint_fully_synced(ji);
        // 방금 수신된 모터를 sync 표시
        for (int mi : joint_motors(ji)) {
            // hasValidPosition(): 0x92 절대위치 응답을 받기 전엔 false — MIT windowed 값으로
            // 잘못 동기화되는 것을 방지
            if (motors[mi]->state.online && !motor_sync_[mi] && motors[mi]->hasValidPosition()) {
                motor_sync_[mi] = true;
                phys_cmd_cache_[mi].pos = mpos(mi); // raw 모드 초기 목표 = 현재 위치
                getLogger()->info("[{}] Motor[{}] synced pos={:.3f}", getName(), mi, mpos(mi));
            }
        }
        // 이 조인트의 모든 모터가 동기화 완료됐을 때 초기 명령을 현재 위치로 설정
        if (!all_synced_before && joint_fully_synced(ji)) {
            joint_fb_pos_[ji] = compute_joint_fk(ji);
            joint_cmds[ji].pos = joint_fb_pos_[ji];
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

    // 조인트 모드 송신 진입점: 각 가상 조인트마다 준비 상태를 확인하고 IK 송신을 호출,
    // GUI 에코용으로 실제 적용된 조인트 명령(joint3/4/cmd_applied)을 발행한다.
    void send_motor_commands(uint64_t tick) {
        for (int ji = 0; ji < NJ; ji++) {
            // ready 조건: 조인트 ON + 모든 모터 동기화 완료 + 명령이 1초 내 살아있음
            const bool ready = joint_on_flag_[ji] && joint_fully_synced(ji) &&
                               (tick - cmd_wdt[ji] <= 1 * getFrequency());

            if (!ready) {
                // 오프라인 or 준비 안 됨 → 두 모터 모두 영(0) 명령으로 안전하게 정지
                for (int mi : joint_motors(ji))
                    send_motor_cmd(mi, MotorCommand{});
                if (ji == 1) j4_active_pos_motor_ = -1; // 다음 ready 때 현재 위치 기준 재결정
                continue;
            }

            // IK: 가상 조인트 명령 → 두 물리 모터 명령(위치 + 장력)으로 변환 후 송신
            apply_ik_and_send(ji);

            // GUI Sync가 "현재 적용된 값"을 표시하도록 조인트 공간 명령을 그대로 에코
            custom_types::MotorCmd applied{};
            applied.pos    = joint_cmds[ji].pos;
            applied.vel    = joint_cmds[ji].vel;
            applied.torque = joint_cmds[ji].torque;
            applied.kp     = joint_cmds[ji].kp;
            applied.kd     = joint_cmds[ji].kd;
            dw_joint_cmd_applied[ji].write(applied);
        }
    }

    // 와이어 조인트 IK: 가상 조인트 명령 → 위치 모터(B) + 장력 모터(A) 명령으로 분해해 송신.
    // 두 조인트 모두 모터 역할은 고정 (B=위치, A=장력) — 이동 방향에 따라 바뀌지 않는다.
    void apply_ik_and_send(int ji) {
        const auto& cmd = joint_cmds[ji];
        switch (ji) {
            case 0: { // joint3: 팔꿈치 와이어 IK
                // motor A (0x06, motors[0]): 항상 장력 제어 — kp=kd=0, torque=tension
                // motor B (0x07, motors[1]): 항상 위치 제어 — lower_link 커플링 보정 포함
                // motor B는 MyActuatorX6 위치모드(0xA4)를 쓰는데, velocity=0이면 프로토콜상
                // "속도제한 없음"이 되어 위험하다 — GUI가 안 줬으면 yaml 기본값으로 대체
                double vel3 = cmd.vel;
                if (vel3 == 0.0) {
                    const auto& def_speed = p_default_speed.read();
                    if ((int)def_speed.size() > 3) vel3 = def_speed[3];
                }

                MotorCommand mc_b{};
                mc_b.pos    = kin_manip::joint3_joint_to_motor_pos(cmd.pos, motor03_pos_);
                mc_b.vel    = kin_manip::joint3_vel_joint_to_motor(vel3);
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
            case 1: { // joint4: 상단링크 와이어 IK — 이동 방향에 따라 위치/장력 모터 역할 교대
                // 방향(어느 모터가 위치제어냐)은 '직전 조인트 명령'을 기준으로 정한다 (모터 상태값 X):
                //   목표>직전명령 : 길어져야 함 → m04(0x04) 위치, m05(0x05) 장력
                //   목표<직전명령 : 줄어들어야 함 → m05 위치, m04 장력
                // 모터 상태(FK)로 판단하면 장력제어 모터가 예상보다 더/덜 움직였을 때 방향을 오판하므로
                // 명령 기준으로 바꿨다(첫 명령만 예외로 측정 길이 사용 — 아래 결정 로직 참조).
                // 데드밴드(UPPER_DIR_DEADBAND) 이내 미세 명령 변화로는 역할을 안 바꾼다(히스테리시스).
                // 위치 모터는 0xA4 위치모드라 velocity=0이면 "속도제한 없음"이 되어 위험 —
                // GUI가 안 줬으면 yaml 기본값(joint_default_speed[4])으로 대체.
                double vel4 = cmd.vel;
                if (vel4 == 0.0) {
                    const auto& def_speed = p_default_speed.read();
                    if ((int)def_speed.size() > 4) vel4 = def_speed[4];
                }

                const double L_tgt = cmd.pos;

                // ── 위치제어(능동) 모터 선택: 2=m04(길어짐), 3=m05(줄어듦) ──
                // 기준은 '직전 조인트 명령 길이'(j4_prev_cmd_len_). 토크제어 중인 반대편 모터가
                // 예상보다 더/덜 돌아 측정 각도(FK)가 틀어져도, 명령 기준이라 방향을 오판하지 않는다.
                //   목표 > 직전명령 → 길어짐 → m04 위치 / 목표 < 직전명령 → 줄어듦 → m05 위치.
                // 명령 이력이 없을 때(첫 명령/재동기화 직후, active==-1)만 막 동기화된 실제 길이(FK)로
                // 첫 방향을 정한다(이때는 토크제어 누적이 없어 측정값을 신뢰할 수 있다).
                int active = j4_active_pos_motor_;
                if (active != 2 && active != 3) {
                    // 첫 결정: 측정 길이(FK) 기준
                    const double L_cur = kin_manip::joint4_motor4_to_joint(mpos(2), motor03_pos_);
                    active = (L_tgt - L_cur >= 0.0) ? 2 : 3; // 동률→m04(길어짐)
                    j4_prev_cmd_len_ = L_tgt;
                } else {
                    // 이후: 직전 '명령' 기준. 데드밴드 이상 바뀐 경우에만 역할·기준 갱신.
                    const double err = L_tgt - j4_prev_cmd_len_;
                    if (err >  UPPER_DIR_DEADBAND)      { active = 2; j4_prev_cmd_len_ = L_tgt; } // 길어짐→m04
                    else if (err < -UPPER_DIR_DEADBAND) { active = 3; j4_prev_cmd_len_ = L_tgt; } // 줄어듦→m05
                    // 데드밴드 내: 직전 역할·기준 그대로 유지(채터링 방지)
                }
                j4_active_pos_motor_ = active;

                // 장력 모터 명령(공통) — pos 는 현재값 hold, 순수 토크(kp=kd=0).
                // +UPPER_TENSION_TORQUE 는 각 모터의 '능동(와이어 감기)' 방향으로 당겨 와이어를
                // 팽팽히 유지한다(motor_direction_sign 이 +=능동방향이 되도록 보정돼 있다는 전제).
                MotorCommand mc_ten{};
                mc_ten.vel = 0.0; mc_ten.torque = kin_manip::UPPER_TENSION_TORQUE;
                mc_ten.kp = 0.0;  mc_ten.kd = 0.0;

                MotorCommand mc_pos{};
                mc_pos.kp = cmd.kp; mc_pos.kd = cmd.kd;

                if (active == 2) {
                    // 길어짐: m04(2) 위치, m05(3) 장력
                    mc_pos.pos    = kin_manip::joint4_joint_to_motor4_pos(L_tgt, motor03_pos_);
                    mc_pos.vel    = kin_manip::joint4_vel_joint_to_motor4(vel4);
                    mc_pos.torque = kin_manip::joint4_torque_joint_to_motor4(cmd.torque);
                    mc_ten.pos    = mpos(3);
                    send_motor_cmd(2, mc_pos);
                    send_motor_cmd(3, mc_ten);
                } else {
                    // 줄어듦: m05(3) 위치, m04(2) 장력
                    mc_pos.pos    = kin_manip::joint4_joint_to_motor_pos(L_tgt, motor03_pos_);
                    mc_pos.vel    = kin_manip::joint4_vel_joint_to_motor(vel4);
                    mc_pos.torque = kin_manip::joint4_torque_joint_to_motor(cmd.torque);
                    mc_ten.pos    = mpos(2);
                    send_motor_cmd(3, mc_pos);
                    send_motor_cmd(2, mc_ten);
                }
                break;
            }
        }
    }

    // ── 데이터 채널 ──
    DataWriter<bool> dw_state{"can1_state", ArchiveOption::Enable};

    // 물리 모터 raw 상태 (GUI 모터 뷰용) — 채널명(mNN)은 CAN ID 기준, 배열 순서는 motors[] 와 동일
    // motors[0]=0x06, [1]=0x07, [2]=0x04, [3]=0x05
    DataWriter<custom_types::MotorState> dw_phys_state_[NM] = {
        DataWriter<custom_types::MotorState>{"phys_motor/m06/state"},
        DataWriter<custom_types::MotorState>{"phys_motor/m07/state"},
        DataWriter<custom_types::MotorState>{"phys_motor/m04/state"},
        DataWriter<custom_types::MotorState>{"phys_motor/m05/state"},
    };

    // 물리 모터 CAN tx/rx 통계 (GUI 모터 뷰 진단용)
    DataWriter<custom_types::MotorIoStats> dw_phys_io_[NM] = {
        DataWriter<custom_types::MotorIoStats>{"phys_motor/m06/io"},
        DataWriter<custom_types::MotorIoStats>{"phys_motor/m07/io"},
        DataWriter<custom_types::MotorIoStats>{"phys_motor/m04/io"},
        DataWriter<custom_types::MotorIoStats>{"phys_motor/m05/io"},
    };
    // 물리 모터에 실제로 적용된 명령 echo (모터 공간) — GUI Sync가 "현재 값"을 보여주는 데 사용
    DataWriter<custom_types::MotorCmd> dw_phys_cmd_applied_[NM] = {
        DataWriter<custom_types::MotorCmd>{"phys_motor/m06/cmd_applied"},
        DataWriter<custom_types::MotorCmd>{"phys_motor/m07/cmd_applied"},
        DataWriter<custom_types::MotorCmd>{"phys_motor/m04/cmd_applied"},
        DataWriter<custom_types::MotorCmd>{"phys_motor/m05/cmd_applied"},
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

    // joint4 방향별 위치/장력 역할 교대 상태
    //   현재 위치 제어를 맡은 모터 인덱스(2=m04 길어짐, 3=m05 줄어듦). -1=미결정(다음 ready 때 결정)
    int j4_active_pos_motor_ = -1;
    // 방향 결정 기준이 되는 '직전 joint4 명령 길이'[m]. active==-1(첫 명령)일 때 FK로 초기화되고,
    // 이후 데드밴드 이상 명령이 바뀔 때마다 갱신된다. (모터 측정값이 아니라 명령 이력을 기준으로 함)
    double j4_prev_cmd_len_ = 0.0;
    // '명령'이 이 값 이상 바뀌어야 위치/장력 역할을 교대한다(미세 변화 채터링 방지) [m].
    // 의도한 최소 이동(사용자 예: 1mm)보다 작아야 그 이동이 방향 전환을 일으킨다.
    static constexpr double UPPER_DIR_DEADBAND = 0.0002; // 0.2mm

    // ── 모터 명령 모드 (IK 우회, 모터 공간 직접 명령) ──
    DataReader<bool> dr_motor_raw_mode_{"gui/motor_raw_mode", DependencyType::Weak};
    DataReader<custom_types::MotorCmd> dr_phys_cmd_[NM] = {
        DataReader<custom_types::MotorCmd>{"gui/phys_motor/m06/cmd", DependencyType::Weak},
        DataReader<custom_types::MotorCmd>{"gui/phys_motor/m07/cmd", DependencyType::Weak},
        DataReader<custom_types::MotorCmd>{"gui/phys_motor/m04/cmd", DependencyType::Weak},
        DataReader<custom_types::MotorCmd>{"gui/phys_motor/m05/cmd", DependencyType::Weak},
    };
    DataReader<bool> dr_phys_on_[NM] = {
        DataReader<bool>{"gui/phys_motor/m06/on", DependencyType::Weak},
        DataReader<bool>{"gui/phys_motor/m07/on", DependencyType::Weak},
        DataReader<bool>{"gui/phys_motor/m04/on", DependencyType::Weak},
        DataReader<bool>{"gui/phys_motor/m05/on", DependencyType::Weak},
    };
    // 영점 설정 트리거(one-shot) — 인덱스 = motors[] 순서 (m06, m07, m04, m05; CAN ID 순서 아님!)
    DataReader<rt::Signal> dr_set_zero_[NM] = {
        DataReader<rt::Signal>{"gui/phys_motor/m06/set_zero", DependencyType::Weak},
        DataReader<rt::Signal>{"gui/phys_motor/m07/set_zero", DependencyType::Weak},
        DataReader<rt::Signal>{"gui/phys_motor/m04/set_zero", DependencyType::Weak},
        DataReader<rt::Signal>{"gui/phys_motor/m05/set_zero", DependencyType::Weak},
    };

    Parameter<std::string> p_port{"can1.port", "can1"};
    // 모터 방향 보정 (m04~m07) — 실제 배선/장착에 맞춰 +1/-1
    Parameter<std::vector<double>> p_motor_sign{"motor_direction_sign"};
    // joint3/4 위치모드(0xA4)에서 velocity 미지정(0) 시 대체할 기본 속도 [rad/s 또는 m/s]
    Parameter<std::vector<double>> p_default_speed{"joint_default_speed"};

    // ── 모터 ──
    // 배열 인덱스는 joint 그룹 순서 (CAN ID 순서가 아님):
    //   [0],[1] = joint3(elbow) 0x06/0x07,  [2],[3] = joint4(upper) 0x04/0x05
    std::vector<std::shared_ptr<Motor>> motors = {
        std::make_shared<MyActuatorX6>(0x06), // motors[0] joint3 elbow: 와이어 A (장력)
        std::make_shared<MyActuatorX6>(0x07), // motors[1] joint3 elbow: 와이어 B (위치)
        std::make_shared<MyActuatorX6>(0x04), // motors[2] joint4 upper: 와이어 A
        std::make_shared<MyActuatorX6>(0x05), // motors[3] joint4 upper: 와이어 B
    };

    double motor_sign_[NM] = {1.0, 1.0, 1.0, 1.0};

    // ── 내부 상태 ──
    bool   motor_raw_mode_ = false; // gui/motor_raw_mode 캐시
    std::array<MotorCommand, NJ>             joint_cmds{};       // 조인트 모드 명령 (joint 공간, 마지막 명령 캐시)
    std::array<custom_types::MotorCmd, NM>   phys_cmd_cache_{}; // raw 모드 명령 (모터 공간, 마지막 명령 캐시)
    std::array<double, NJ>                   joint_fb_pos_{};
    bool   joint_on_flag_[NJ] = {}; // joint3~4/on 캐시
    bool   phys_on_flag_[NM]  = {}; // gui/phys_motor/m0X/on 캐시
    bool   effective_on_[NM]  = {}; // 현 모드 기준 실제 적용된 on 상태
    bool   motor_sync_[NM]    = {};
    int    zero_step_[NM]     = {}; // 영점 설정 진행 상태(0=대기, >0=시퀀스 진행 카운터)
    uint64_t exec_tick_ = 0; // 0x92 라운드로빈 스태거링에 사용
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
    uint32_t tx_fail_count_[NM] = {}; // write() 실패(TX 버퍼 과부하로 드랍) 누적 카운트
    uint32_t rx_count_[NM]      = {};
    uint32_t tx_count_prev_[NM] = {};
    uint32_t rx_count_prev_[NM] = {};
    double   tx_hz_[NM]         = {};
    double   rx_hz_[NM]         = {};
    std::chrono::steady_clock::time_point io_window_start_ = std::chrono::steady_clock::now();

    // 영점 설정 시퀀스(모터별 상태머신). 60Hz 기준 step 1에서 영점 저장 프레임(0x64)을 보내고,
    // ROM 기록이 안정되도록 ~100ms 뒤 리셋(0x76)을 보낸다(can_bus1 은 전부 MyActuatorX6).
    // 완료 시 motor_sync_=false 로 강제 재동기화 — 0점 기준이 바뀌었으니 현재 위치로 명령을 재시드한다.
    static constexpr int kZeroResetStep = 7;  // step 1(0x64) 후 ~100ms 뒤 0x76
    static constexpr int kZeroDoneStep  = 9;  // 시퀀스 종료
    void run_zero_sequence(int mi) {
        if (zero_step_[mi] == 0) return;
        if (zero_step_[mi] == 1) {
            if (motors[mi]->SetZero(so)) { tx_count_[mi]++; getLogger()->info("[{}] Motor[{}] (0x{:02X}) SetZero 프레임 전송", getName(), mi, motors[mi]->id); }
            else getLogger()->warn("[{}] Motor[{}] SetZero 미지원/전송 실패", getName(), mi);
        } else if (zero_step_[mi] == kZeroResetStep) {
            if (motors[mi]->Reset(so)) { tx_count_[mi]++; getLogger()->info("[{}] Motor[{}] (0x{:02X}) Reset(재시작) 프레임 전송", getName(), mi, motors[mi]->id); }
        }
        if (++zero_step_[mi] > kZeroDoneStep) {
            zero_step_[mi] = 0;
            motor_sync_[mi] = false; // 0점 기준 변경 → 온라인+유효위치 복귀 시 현재 위치로 재시드
            getLogger()->info("[{}] Motor[{}] (0x{:02X}) 영점 설정 완료", getName(), mi, motors[mi]->id);
        }
    }

    // cmd는 모터 공간(raw) 기준 — 모터로 보내기 직전 motor_sign_ 으로 되돌림
    void send_motor_cmd(int mi, const MotorCommand& cmd) {
        if (zero_step_[mi] > 0) return; // 영점 시퀀스 중에는 일반 제어 프레임을 보내지 않음
        custom_types::MotorCmd echo{};
        echo.pos = cmd.pos; echo.vel = cmd.vel; echo.torque = cmd.torque; echo.kp = cmd.kp; echo.kd = cmd.kd;
        dw_phys_cmd_applied_[mi].write(echo);

        MotorCommand raw = cmd;
        raw.pos    *= motor_sign_[mi];
        raw.vel    *= motor_sign_[mi];
        raw.torque *= motor_sign_[mi];
        // kp>0 → 0xA4 절대위치 모드(joint_default_speed가 maxSpeed로 적용됨)
        // kp==0 → 0xA1 토크(전류) 제어 모드
        raw.position_mode = cmd.kp > 0.0;
        if (!motors[mi]->Control(so, raw)) tx_fail_count_[mi]++;
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
