#pragma once

// Safety Layer — 매니퓰레이터용 (5 조인트, IMU/RL 없음)
//
// 모드: MANUAL (기본) / LOCK (안전 위반 시) / RESTORE (잠금 해제 복구)
//
// 세이프티 수치 조정은 config/robotnl.yaml 의
//   safety_layer.*  키들을 수정하면 됩니다.
//
// 조인트 위치 한계 순서 (joint0 ~ joint4):
//   joint0: base_yaw   (revolute, rad)
//   joint1: pitch       (revolute, rad)
//   joint2: lower_link  (prismatic, m)
//   joint3: elbow_pitch (revolute, rad)
//   joint4: upper_link  (prismatic, m)

#include <rtfw/task.h>
#include "custom_types.hpp"
#include "util.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <string>

using namespace std::chrono_literals;
using namespace rtfw;
using namespace rtfw::rt;

namespace task_pool {

static constexpr int SL_N = 5; // 조인트 수

enum SLMode { SL_MANUAL, SL_LOCK, SL_RESTORE };

struct SafetyLayerState {
    std::array<custom_types::MotorCmd,   SL_N> manager_cmd{};
    std::array<custom_types::MotorState, SL_N> motor_state{};
    std::array<bool, SL_N> manager_cmd_online{};
    std::array<bool, SL_N> motor_state_online{};
    std::array<uint64_t, SL_N> manager_cmd_tick{};
    std::array<uint64_t, SL_N> motor_state_tick{};
    SLMode   mode                = SL_MANUAL;
    uint64_t restore_enter_tick  = 0;
};

class SafetyLayer : public Task<SafetyLayerState> {
public:
    const char* getName() const override { return "SafetyLayer"; }

    void initialize(SafetyLayerState& s) override {
        s.mode = SL_MANUAL;
        s.manager_cmd_online.fill(false);
        s.motor_state_online.fill(false);
        s.manager_cmd_tick.fill(0);
        s.motor_state_tick.fill(0);
        s.restore_enter_tick = getExecutionLocalTick();
    }

    void execute(SafetyLayerState& s) override {
        // 잠금 해제 요청
        dr_reset.on_update([&]() {
            if (s.mode == SL_LOCK) {
                s.mode = SL_RESTORE;
                s.restore_enter_tick = getExecutionLocalTick();
                getLogger()->info("[{}] Restore requested from LOCK", getName());
                for (int i = 0; i < SL_N; i++) {
                    auto cmd = s.manager_cmd[i];
                    cmd.duration_ms = p_restore_duration_sec.read() * 1000.0;
                    dw_mtr_cmd[i].write(clamp_cmd(cmd));
                }
            }
        });

        // Manager 명령 수신
        for (int i = 0; i < SL_N; i++) {
            dr_manager_cmd[i].on_update([&, i](const custom_types::MotorCmd& d) {
                s.manager_cmd[i]        = d;
                s.manager_cmd_online[i] = true;
                s.manager_cmd_tick[i]   = getExecutionLocalTick();
            });
            dr_mtr_stat[i].on_update([&, i](const custom_types::MotorState& d) {
                s.motor_state[i]        = d;
                s.motor_state_online[i] = true;
                s.motor_state_tick[i]   = getExecutionLocalTick();
            });
        }

        // 안전 검사
        std::string fault_reason;
        if (s.mode == SL_MANUAL && !safety_check(s, fault_reason))
            enter_lock(s, fault_reason);

        switch (s.mode) {
            case SL_MANUAL:
                for (int i = 0; i < SL_N; i++)
                    dw_mtr_cmd[i].write(clamp_cmd(s.manager_cmd[i]));
                break;

            case SL_LOCK:
                for (int i = 0; i < SL_N; i++) {
                    custom_types::MotorCmd lock{};
                    lock.pos        = s.motor_state_online[i] ? s.motor_state[i].pos : 0.0;
                    lock.kp         = p_lock_kp.read();
                    lock.kd         = p_lock_kd.read();
                    lock.duration_ms = 0.0;
                    dw_mtr_cmd[i].write(lock);
                }
                break;

            case SL_RESTORE: {
                double elapsed = static_cast<double>(getExecutionLocalTick() - s.restore_enter_tick)
                                 / getFrequency();
                if (elapsed >= p_restore_duration_sec.read()) {
                    s.mode = SL_MANUAL;
                    getLogger()->info("[{}] Restore complete → MANUAL", getName());
                }
                break;
            }
        }

        dw_lock_state.write(s.mode == SL_LOCK);
        dw_restore_state.write(s.mode == SL_RESTORE);
    }

private:
    // ── 데이터 채널 ──
    DataReader<rt::Signal> dr_reset{"manager/reset_signal", DependencyType::Weak};

    DataReader<custom_types::MotorCmd> dr_manager_cmd[SL_N] = {
        DataReader<custom_types::MotorCmd>{"manager/joint0/cmd", DependencyType::Weak},
        DataReader<custom_types::MotorCmd>{"manager/joint1/cmd", DependencyType::Weak},
        DataReader<custom_types::MotorCmd>{"manager/joint2/cmd", DependencyType::Weak},
        DataReader<custom_types::MotorCmd>{"manager/joint3/cmd", DependencyType::Weak},
        DataReader<custom_types::MotorCmd>{"manager/joint4/cmd", DependencyType::Weak},
    };
    DataReader<custom_types::MotorState> dr_mtr_stat[SL_N] = {
        DataReader<custom_types::MotorState>{"joint0/state", DependencyType::Weak},
        DataReader<custom_types::MotorState>{"joint1/state", DependencyType::Weak},
        DataReader<custom_types::MotorState>{"joint2/state", DependencyType::Weak},
        DataReader<custom_types::MotorState>{"joint3/state", DependencyType::Weak},
        DataReader<custom_types::MotorState>{"joint4/state", DependencyType::Weak},
    };
    DataWriter<custom_types::MotorCmd> dw_mtr_cmd[SL_N] = {
        DataWriter<custom_types::MotorCmd>{"joint0/cmd"},
        DataWriter<custom_types::MotorCmd>{"joint1/cmd"},
        DataWriter<custom_types::MotorCmd>{"joint2/cmd"},
        DataWriter<custom_types::MotorCmd>{"joint3/cmd"},
        DataWriter<custom_types::MotorCmd>{"joint4/cmd"},
    };
    DataWriter<bool> dw_lock_state{"safety/lock_state"};
    DataWriter<bool> dw_restore_state{"safety/restore_state"};

    // ── 파라미터 (config/robotnl.yaml에서 수정) ──
    Parameter<double> p_restore_duration_sec{"safety_layer.restore_duration_sec", 10.0};
    Parameter<double> p_cmd_timeout_sec{"safety_layer.cmd_timeout_sec", 1.0};
    Parameter<double> p_state_timeout_sec{"safety_layer.state_timeout_sec", 1.0};

    // 조인트 위치 한계 [joint0~4]: rad (revolute) / m (prismatic)
    // URDF 한계값 기본 적용 — yaml에서 덮어쓸 수 있음
    Parameter<std::vector<double>> p_joint_pos_limit_lower{
        "safety_layer.joint_pos_limit_lower",
        {-2.094395, -0.523599, -0.09, -1.570796, -0.095}
    };
    Parameter<std::vector<double>> p_joint_pos_limit_upper{
        "safety_layer.joint_pos_limit_upper",
        { 2.094395,  1.570796,  0.0,  1.570796,  0.0}
    };

    // 위치 한계 허용 오차 [rad / m]
    // 시뮬레이션: ~0.002 (부동소수점 드리프트 흡수)
    // 실제 하드웨어: ~0.001 (엔코더 노이즈 흡수)
    Parameter<double> p_pos_tolerance     {"safety_layer.pos_tolerance",       0.002};

    Parameter<double> p_joint_vel_limit   {"safety_layer.joint_vel_limit",    30.0};
    Parameter<double> p_joint_torque_limit{"safety_layer.joint_torque_limit", 30.0};
    Parameter<double> p_joint_temp_limit_c{"safety_layer.joint_temp_limit_c", 95.0};

    Parameter<double> p_cmd_pos_limit   {"safety_layer.cmd_pos_limit",    10.0};
    Parameter<double> p_cmd_vel_limit   {"safety_layer.cmd_vel_limit",    30.0};
    Parameter<double> p_cmd_torque_limit{"safety_layer.cmd_torque_limit", 30.0};
    Parameter<double> p_cmd_kp_limit    {"safety_layer.cmd_kp_limit",    600.0};
    Parameter<double> p_cmd_kd_limit    {"safety_layer.cmd_kd_limit",      5.0};
    Parameter<double> p_lock_kp         {"safety_layer.lock_kp",         100.0};
    Parameter<double> p_lock_kd         {"safety_layer.lock_kd",           3.0};

    // ── 내부 유틸리티 ──
    void enter_lock(SafetyLayerState& s, const std::string& reason) {
        s.mode = SL_LOCK;
        getLogger()->error("[{}] LOCK: {}", getName(), reason);
    }

    bool timeout_exceeded(uint64_t now, uint64_t stamp, double timeout_sec) const {
        if (stamp == 0) return true;
        return (now - stamp) > static_cast<uint64_t>(timeout_sec * getFrequency());
    }

    static bool is_finite_cmd(const custom_types::MotorCmd& c) {
        return std::isfinite(c.pos) && std::isfinite(c.vel) &&
               std::isfinite(c.torque) && std::isfinite(c.kp) && std::isfinite(c.kd);
    }
    static bool is_finite_state(const custom_types::MotorState& st) {
        return std::isfinite(st.pos) && std::isfinite(st.vel) &&
               std::isfinite(st.torque) && std::isfinite(st.temp);
    }

    custom_types::MotorCmd clamp_cmd(const custom_types::MotorCmd& in) {
        auto san = [](double v) { return std::isfinite(v) ? v : 0.0; };
        custom_types::MotorCmd out{};
        out.pos        = std::clamp(san(in.pos),    -p_cmd_pos_limit.read(),    p_cmd_pos_limit.read());
        out.vel        = std::clamp(san(in.vel),    -p_cmd_vel_limit.read(),    p_cmd_vel_limit.read());
        out.torque     = std::clamp(san(in.torque), -p_cmd_torque_limit.read(), p_cmd_torque_limit.read());
        out.kp         = std::clamp(san(in.kp),     0.0,                        p_cmd_kp_limit.read());
        out.kd         = std::clamp(san(in.kd),     0.0,                        p_cmd_kd_limit.read());
        out.duration_ms = (std::isfinite(in.duration_ms) && in.duration_ms >= 0.0) ? in.duration_ms : 0.0;
        return out;
    }

    bool safety_check(SafetyLayerState& s, std::string& reason) {
        const uint64_t now = getExecutionLocalTick();
        const auto& pos_lo = p_joint_pos_limit_lower.read();
        const auto& pos_hi = p_joint_pos_limit_upper.read();

        for (int i = 0; i < SL_N; i++) {
            // 명령 타임아웃
            if (s.manager_cmd_online[i] &&
                timeout_exceeded(now, s.manager_cmd_tick[i], p_cmd_timeout_sec.read())) {
                reason = "manager cmd timeout (joint " + std::to_string(i) + ")";
                return false;
            }
            // 상태 타임아웃
            if (s.motor_state_online[i] &&
                timeout_exceeded(now, s.motor_state_tick[i], p_state_timeout_sec.read())) {
                reason = "motor state timeout (joint " + std::to_string(i) + ")";
                return false;
            }

            if (!s.motor_state_online[i]) continue;

            // NaN/Inf 검사
            if (!is_finite_cmd(s.manager_cmd[i])) {
                reason = "manager cmd non-finite (joint " + std::to_string(i) + ")";
                return false;
            }
            if (!is_finite_state(s.motor_state[i])) {
                reason = "motor state non-finite (joint " + std::to_string(i) + ")";
                return false;
            }

            // 위치 한계 (tolerance 만큼 여유 허용)
            double pos = s.motor_state[i].pos;
            if ((int)pos_lo.size() > i && (int)pos_hi.size() > i) {
                const double tol = p_pos_tolerance.read();
                if (pos < pos_lo[i] - tol || pos > pos_hi[i] + tol) {
                    reason = "joint " + std::to_string(i) + " pos=" + std::to_string(pos)
                             + " out of [" + std::to_string(pos_lo[i] - tol) + ","
                             + std::to_string(pos_hi[i] + tol) + "]";
                    return false;
                }
            }

            // 속도 한계
            if (std::abs(s.motor_state[i].vel) > p_joint_vel_limit.read()) {
                reason = "joint " + std::to_string(i) + " vel=" +
                         std::to_string(s.motor_state[i].vel) + " exceeds limit";
                return false;
            }

            // 토크 한계
            if (std::abs(s.motor_state[i].torque) > p_joint_torque_limit.read()) {
                reason = "joint " + std::to_string(i) + " torque=" +
                         std::to_string(s.motor_state[i].torque) + " exceeds limit";
                return false;
            }

            // 온도 한계
            if (s.motor_state[i].temp > p_joint_temp_limit_c.read()) {
                reason = "joint " + std::to_string(i) + " temp=" +
                         std::to_string(s.motor_state[i].temp) + " exceeds limit";
                return false;
            }
        }

        return true;
    }
};

} // namespace task_pool
