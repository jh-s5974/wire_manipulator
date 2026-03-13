#pragma once

#include <rtfw/task.h>
#include "custom_types.hpp"
#include "util.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

using namespace std::chrono_literals;
using namespace rtfw;
using namespace rtfw::rt;

namespace task_pool {

    enum SafetyLayerLevel {
        SL_ESSENTIAL = 0,
        SL_STRICT = 1,
    };

    enum Mode {
        MANUAL,
        RL,
        LOCK,
        RESTORE,
    };

struct SafetyLayerState {
    std::array<custom_types::MotorCmd, 12> manager_cmd{};
    std::array<custom_types::MotorCmd, 12> policy_action{};
    std::array<custom_types::MotorState, 12> motor_state{};
    custom_types::Imu imu{};
    std::array<bool, 12> motor_on{};
    std::array<bool, 12> manager_cmd_online{};
    std::array<bool, 12> policy_action_online{};
    std::array<bool, 12> motor_state_online{};
    bool imu_online = false;
    bool use_policy_action = false;
    Mode mode = MANUAL;
    uint64_t restore_enter_tick = 0;
    std::array<uint64_t, 12> manager_cmd_tick{};
    std::array<uint64_t, 12> policy_action_tick{};
    std::array<uint64_t, 12> motor_state_tick{};
    uint64_t imu_tick = 0;
    // bool safety_armed = false;
    int safety_level = SL_ESSENTIAL;
    int prev_safety_level = SL_ESSENTIAL;

};

class SafetyLayer : public Task<SafetyLayerState> {
public:
    const char* getName() const override { return "SafetyLayer"; }

    void initialize(SafetyLayerState& s) override {
        s.mode = Mode::MANUAL;
        s.use_policy_action = false;
        s.manager_cmd_online.fill(false);
        s.policy_action_online.fill(false);
        s.motor_state_online.fill(false);
        s.motor_on.fill(true);

        s.restore_enter_tick = getExecutionLocalTick();
        s.manager_cmd_tick.fill(0);
        s.policy_action_tick.fill(0);
        s.motor_state_tick.fill(0);
        s.imu_tick = 0;
        // s.safety_armed = false;
        s.safety_level = SL_ESSENTIAL;
        s.prev_safety_level = SL_ESSENTIAL;

    }

    void execute(SafetyLayerState& s) override {

        dr_reset.on_update([&]() {
            if (s.mode == Mode::LOCK) {
                s.mode = Mode::RESTORE;
                s.restore_enter_tick = getExecutionLocalTick();
                getLogger()->info("[{}] Received reset signal, attempting to restore from LOCK mode", getName());
            }
        });

        dr_rl_signal_.on_update([&](const bool& on) {
            // if (s.mode == Mode::LOCK || s.mode == Mode::RESTORE) {
            //     s.use_policy_action = on;
            //     return;
            // }

            if (!on) {
                if (s.mode == Mode::RL) {
                    s.mode = Mode::MANUAL;
                    getLogger()->info("[{}] Switching to MANUAL mode", getName());

                    for (int i = 0; i < 12; ++i) {
                        s.manager_cmd_online[i] = true;
                        s.manager_cmd_tick[i] = getExecutionLocalTick();
                    }
                }
            } else {
                if (s.mode == Mode::MANUAL) {
                    s.mode = Mode::RL;
                    getLogger()->info("[{}] Switching to RL mode", getName());

                    for (int i = 0; i < 12; ++i) {
                        s.policy_action_online[i] = true;
                        s.policy_action_tick[i] = getExecutionLocalTick();
                    }
                }
            }
            s.use_policy_action = on;
        });

        dr_safety_level_.on_update([&](const int& level) {
            if (level == 1) {
                s.safety_level = SL_STRICT;
            } else {
                s.safety_level = SL_ESSENTIAL;
            }
        });

         for (int i = 0; i < 12; ++i) {
            dr_manager_cmd_[i].on_update([&, i](const custom_types::MotorCmd& data) {
                s.manager_cmd[i] = data;
                s.manager_cmd_online[i] = true;
                s.manager_cmd_tick[i] = getExecutionLocalTick();
            });

            dr_policy_action_[i].on_update([&, i](const custom_types::MotorCmd& data) {
                s.policy_action[i] = data;
                s.policy_action_online[i] = true;
                s.policy_action_tick[i] = getExecutionLocalTick();
            });

            dr_mtr_stat_[i].on_update([&, i](const custom_types::MotorState& data) {
                s.motor_state[i] = data;
                s.motor_state_online[i] = true;
                s.motor_state_tick[i] = getExecutionLocalTick();
            });
        }

        dr_imu_.on_update([&](const custom_types::Imu& data) {
            s.imu = data;
            s.imu_online = true;
            s.imu_tick = getExecutionLocalTick();
        });

        std::string fault_reason;
        if (s.mode != Mode::LOCK && s.mode != Mode::RESTORE && !safety_check(s, fault_reason)) {
            enter_lock(s, fault_reason);
        }
        
        switch(s.mode) {
            case MANUAL:
                for (int i=0; i<12; i++) {
                    dw_mtr_cmd_[i].write(clamp_cmd(s.manager_cmd[i]));
                }
                break;
            case RL:
                for (int i=0; i<12; i++) {
                    dw_mtr_cmd_[i].write(clamp_cmd(s.policy_action[i]));
                }
                break;
            case LOCK: {
                for (int i=0; i<12; i++) {
                    custom_types::MotorCmd lock_cmd;
                    lock_cmd.pos = s.motor_state_online[i] ? s.motor_state[i].pos : 0.0;
                    lock_cmd.vel = 0.0;
                    lock_cmd.torque = 0.0;
                    lock_cmd.kp = p_lock_kp.read();
                    lock_cmd.kd = p_lock_kd.read();
                    dw_mtr_cmd_[i].write(lock_cmd);
                }
                break;
            }
            case RESTORE: {
                double elapsed = getExecutionLocalTick() - s.restore_enter_tick;
                elapsed /= getFrequency();
                const double transition_duration = std::max(1e-3, p_restore_duration_sec.read());
                double ratio = elapsed / transition_duration;
                if (ratio >= 1.0) {
                    ratio = 1.0;
                    s.mode = Mode::MANUAL;
                    getLogger()->info("[{}] Restore complete, switching to MANUAL mode", getName());
                }
                for (int i=0; i<12; i++) {
                    const custom_types::MotorCmd cmd = blend_cmd(make_zero_cmd(), s.manager_cmd[i], ratio);
                    dw_mtr_cmd_[i].write(clamp_cmd(cmd));
                }
                break;
            }
        }

        dw_lock_state_.write(s.mode == Mode::LOCK);
    }


private:
    DataReader<bool> dr_rl_signal_{"manager/rl_signal", DependencyType::Weak};
    DataReader<int> dr_safety_level_{"manager/safety_level", DependencyType::Weak};
    DataReader<rt::Signal> dr_reset{"manager/reset_signal", DependencyType::Weak};
    DataWriter<bool> dw_lock_state_{"safety/lock_state"};

    Parameter<double> p_restore_duration_sec{"safety_layer.restore_duration_sec", 3.0};
    Parameter<double> p_cmd_timeout_sec{"safety_layer.cmd_timeout_sec", 1.0};
    Parameter<double> p_state_timeout_sec{"safety_layer.state_timeout_sec", 1.0};
    Parameter<double> p_imu_timeout_sec{"safety_layer.imu_timeout_sec", 1.0};

    Parameter<std::vector<double>> p_joint_pos_limit_lower{"safety_layer.joint_pos_limit_lower",
        {-0.698132, -1.5708, -0.698132, -1.5708, -1.8326, -1.8326, 0.0, 0.0, -1.22173, -1.22173, -0.663225, -0.523599}};
    Parameter<std::vector<double>> p_joint_pos_limit_upper{"safety_layer.joint_pos_limit_upper",
        {1.5708, 0.698132, 1.5708, 0.872665, 0.523599, 0.523599, 2.1293, 2.1293, 1.0472, 1.0472, 0.523599, 0.663225}};
    Parameter<double> p_joint_vel_limit{"safety_layer.joint_vel_limit", 20.0};
    Parameter<double> p_joint_torque_limit{"safety_layer.joint_torque_limit", 60.0};
    Parameter<double> p_joint_temp_limit_c{"safety_layer.joint_temp_limit_c", 95.0};

    Parameter<double> p_imu_tilt_limit_deg{"safety_layer.imu_tilt_limit_deg", 60.0};
    Parameter<double> p_imu_quat_norm_tol{"safety_layer.imu_quat_norm_tol", 0.2};

    Parameter<double> p_cmd_pos_limit{"safety_layer.cmd_pos_limit", 3.2};
    Parameter<double> p_cmd_vel_limit{"safety_layer.cmd_vel_limit", 40.0};
    Parameter<double> p_cmd_torque_limit{"safety_layer.cmd_torque_limit", 120.0};
    Parameter<double> p_cmd_kp_limit{"safety_layer.cmd_kp_limit", 600.0};
    Parameter<double> p_cmd_kd_limit{"safety_layer.cmd_kd_limit", 5.0};
    Parameter<double> p_lock_kp{"safety_layer.lock_kp", 100.0};
    Parameter<double> p_lock_kd{"safety_layer.lock_kd", 3.0};


    DataReader<custom_types::MotorCmd> dr_manager_cmd_[12] = {
        DataReader<custom_types::MotorCmd>{"manager/hip_yaw_left/cmd", DependencyType::Weak},
        DataReader<custom_types::MotorCmd>{"manager/hip_yaw_right/cmd", DependencyType::Weak},
        DataReader<custom_types::MotorCmd>{"manager/hip_roll_left/cmd", DependencyType::Weak},
        DataReader<custom_types::MotorCmd>{"manager/hip_roll_right/cmd", DependencyType::Weak},
        DataReader<custom_types::MotorCmd>{"manager/hip_pitch_left/cmd", DependencyType::Weak},
        DataReader<custom_types::MotorCmd>{"manager/hip_pitch_right/cmd", DependencyType::Weak},
        DataReader<custom_types::MotorCmd>{"manager/knee_left/cmd", DependencyType::Weak},
        DataReader<custom_types::MotorCmd>{"manager/knee_right/cmd", DependencyType::Weak},
        DataReader<custom_types::MotorCmd>{"manager/ankle_pitch_left/cmd", DependencyType::Weak},
        DataReader<custom_types::MotorCmd>{"manager/ankle_pitch_right/cmd", DependencyType::Weak},
        DataReader<custom_types::MotorCmd>{"manager/ankle_roll_left/cmd", DependencyType::Weak},
        DataReader<custom_types::MotorCmd>{"manager/ankle_roll_right/cmd", DependencyType::Weak},
    };

    DataReader<custom_types::MotorCmd> dr_policy_action_[12] = {
        DataReader<custom_types::MotorCmd>{"hip_yaw_left/action"},
        DataReader<custom_types::MotorCmd>{"hip_yaw_right/action"},
        DataReader<custom_types::MotorCmd>{"hip_roll_left/action"},
        DataReader<custom_types::MotorCmd>{"hip_roll_right/action"},
        DataReader<custom_types::MotorCmd>{"hip_pitch_left/action"},
        DataReader<custom_types::MotorCmd>{"hip_pitch_right/action"},
        DataReader<custom_types::MotorCmd>{"knee_left/action"},
        DataReader<custom_types::MotorCmd>{"knee_right/action"},
        DataReader<custom_types::MotorCmd>{"ankle_pitch_left/action"},
        DataReader<custom_types::MotorCmd>{"ankle_pitch_right/action"},
        DataReader<custom_types::MotorCmd>{"ankle_roll_left/action"},
        DataReader<custom_types::MotorCmd>{"ankle_roll_right/action"},
    };

    DataWriter<custom_types::MotorCmd> dw_mtr_cmd_[12] = {
        DataWriter<custom_types::MotorCmd>{"hip_yaw_left/cmd"},
        DataWriter<custom_types::MotorCmd>{"hip_yaw_right/cmd"},
        DataWriter<custom_types::MotorCmd>{"hip_roll_left/cmd"},
        DataWriter<custom_types::MotorCmd>{"hip_roll_right/cmd"},
        DataWriter<custom_types::MotorCmd>{"hip_pitch_left/cmd"},
        DataWriter<custom_types::MotorCmd>{"hip_pitch_right/cmd"},
        DataWriter<custom_types::MotorCmd>{"knee_left/cmd"},
        DataWriter<custom_types::MotorCmd>{"knee_right/cmd"},
        DataWriter<custom_types::MotorCmd>{"ankle_pitch_left/cmd"},
        DataWriter<custom_types::MotorCmd>{"ankle_pitch_right/cmd"},
        DataWriter<custom_types::MotorCmd>{"ankle_roll_left/cmd"},
        DataWriter<custom_types::MotorCmd>{"ankle_roll_right/cmd"},
    };    

    DataReader<custom_types::MotorState> dr_mtr_stat_[12] = {
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

    DataReader<custom_types::Imu> dr_imu_{"imu_data", DependencyType::Weak};
private:
    static bool is_finite(double v) {
        return std::isfinite(v);
    }

    static bool is_finite_cmd(const custom_types::MotorCmd& cmd) {
        return is_finite(cmd.pos) && is_finite(cmd.vel) && is_finite(cmd.torque) && is_finite(cmd.kp) && is_finite(cmd.kd);
    }

    static bool is_finite_state(const custom_types::MotorState& state) {
        return is_finite(state.pos) && is_finite(state.vel) && is_finite(state.torque) && is_finite(state.temp);
    }

    custom_types::MotorCmd make_zero_cmd() const {
        custom_types::MotorCmd cmd{};
        cmd.pos = 0.0;
        cmd.vel = 0.0;
        cmd.torque = 0.0;
        cmd.kp = 0.0;
        cmd.kd = 0.0;
        return cmd;
    }

    custom_types::MotorCmd blend_cmd(const custom_types::MotorCmd& from, const custom_types::MotorCmd& to, double ratio) const {
        const double alpha = std::clamp(ratio, 0.0, 1.0);
        custom_types::MotorCmd cmd{};
        cmd.pos = alpha * to.pos + (1.0 - alpha) * from.pos;
        cmd.vel = alpha * to.vel + (1.0 - alpha) * from.vel;
        cmd.torque = alpha * to.torque + (1.0 - alpha) * from.torque;
        cmd.kp = alpha * to.kp + (1.0 - alpha) * from.kp;
        cmd.kd = alpha * to.kd + (1.0 - alpha) * from.kd;
        return cmd;
    }

    custom_types::MotorCmd clamp_cmd(const custom_types::MotorCmd& in) {
        custom_types::MotorCmd out{};
        const auto sanitize = [](double v) {
            return std::isfinite(v) ? v : 0.0;
        };

        out.pos = std::clamp(sanitize(in.pos), -p_cmd_pos_limit.read(), p_cmd_pos_limit.read());
        out.vel = std::clamp(sanitize(in.vel), -p_cmd_vel_limit.read(), p_cmd_vel_limit.read());
        out.torque = std::clamp(sanitize(in.torque), -p_cmd_torque_limit.read(), p_cmd_torque_limit.read());
        out.kp = std::clamp(sanitize(in.kp), 0.0, p_cmd_kp_limit.read());
        out.kd = std::clamp(sanitize(in.kd), 0.0, p_cmd_kd_limit.read());
        return out;
    }

    bool timeout_exceeded(uint64_t now_tick, uint64_t stamp_tick, double timeout_sec) const {
        if (stamp_tick == 0) {
            return true;
        }
        const uint64_t timeout_tick = static_cast<uint64_t>(std::max(0.0, timeout_sec) * static_cast<double>(getFrequency()));
        return (now_tick - stamp_tick) > timeout_tick;
    }

    bool all_inputs_ready(const SafetyLayerState& s, bool need_policy_cmd) const {
        for (int i = 0; i < 12; ++i) {
            if (!s.manager_cmd_online[i]) {
                return false;
            }
            if (need_policy_cmd && !s.policy_action_online[i]) {
                return false;
            }
            if (!s.motor_state_online[i]) {
                return false;
            }
        }
        return s.imu_online;
    }

    void enter_lock(SafetyLayerState& s, const std::string& reason) {
        s.mode = Mode::LOCK;
        getLogger()->error("[{}] Enter LOCK mode: {}", getName(), reason);
    }

    bool safety_check(SafetyLayerState& s, std::string& reason) {
        // return true;    // TEMP: Disable safety checks for now to prevent accidental lockouts during development. Implement proper checks before enabling.
        const uint64_t now = getExecutionLocalTick();
        const bool need_policy_cmd = (s.mode == Mode::RL || s.use_policy_action);
        const bool require_manager_fresh = (s.mode != Mode::RL);
        const bool strict_level = (s.safety_level == SL_STRICT);

        if (s.prev_safety_level != s.safety_level) {
            if (strict_level) {
                // s.safety_armed = false;
                getLogger()->info("[{}] Safety level -> STRICT", getName());
            } else {
                getLogger()->info("[{}] Safety level -> ESSENTIAL", getName());
            }
            s.prev_safety_level = s.safety_level;
        }

        // if (strict_level) {
        //     if (!s.safety_armed) {
        //         // if (!all_inputs_ready(s, need_policy_cmd)) {
        //         //     return true;
        //         // }
        //         s.safety_armed = true;
        //         getLogger()->info("[{}] Strict safety checks armed", getName());
        //     }
        // }

        if (strict_level) {
            for (int i = 0; i < 12; ++i) {
                if (require_manager_fresh) {
                    if (!s.manager_cmd_online[i] || timeout_exceeded(now, s.manager_cmd_tick[i], p_cmd_timeout_sec.read())) {
                        reason = "manager command timeout (joint " + std::to_string(i) + ")";
                        return false;
                    }
                }

                if (need_policy_cmd && (!s.policy_action_online[i] || timeout_exceeded(now, s.policy_action_tick[i], p_cmd_timeout_sec.read()))) {
                    reason = "policy action timeout (joint " + std::to_string(i) + ")";
                    return false;
                }

                if (!s.motor_state_online[i] || timeout_exceeded(now, s.motor_state_tick[i], p_state_timeout_sec.read())) {
                    reason = "motor state timeout (joint " + std::to_string(i) + ")";
                    return false;
                }
            }

            if (!s.imu_online || timeout_exceeded(now, s.imu_tick, p_imu_timeout_sec.read())) {
                reason = "imu timeout";
                return false;
            }
        }

        for (int i = 0; i < 12; ++i) {
            if (!s.motor_state_online[i]) {
                return true;
            }
        }

        if (strict_level) {
            if (!s.imu_online) {
                return true;
            }
        }

        for (int i = 0; i < 12; ++i) {
            if (!is_finite_cmd(s.manager_cmd[i])) {
                reason = "manager command has non-finite value (joint " + std::to_string(i) + ")";
                return false;
            }

            if (need_policy_cmd && !is_finite_cmd(s.policy_action[i])) {
                reason = "policy action has non-finite value (joint " + std::to_string(i) + ")";
                return false;
            }

            if (!is_finite_state(s.motor_state[i])) {
                reason = "motor state has non-finite value (joint " + std::to_string(i) + ")";
                return false;
            }

            {
                const auto& pos_lower = p_joint_pos_limit_lower.read();
                const auto& pos_upper = p_joint_pos_limit_upper.read();
                const double pos = s.motor_state[i].pos;
                if (pos < pos_lower[i] || pos > pos_upper[i]) {
                    reason = "joint position limit exceeded (joint " + std::to_string(i) + "): " +
                             std::to_string(pos_lower[i]) + " <= " +
                             std::to_string(pos) + " <= " +
                             std::to_string(pos_upper[i]) + " violated";
                    return false;
                }
            }

            if (std::abs(s.motor_state[i].vel) > p_joint_vel_limit.read()) {
                reason = "joint velocity limit exceeded (joint " + std::to_string(i) + "): |" +
                         std::to_string(s.motor_state[i].vel) + "| > " +
                         std::to_string(p_joint_vel_limit.read());
                return false;
            }

            if (std::abs(s.motor_state[i].torque) > p_joint_torque_limit.read()) {
                reason = "joint torque limit exceeded (joint " + std::to_string(i) + "): |" +
                         std::to_string(s.motor_state[i].torque) + "| > " +
                         std::to_string(p_joint_torque_limit.read());
                return false;
            }

            if (s.motor_state[i].temp > p_joint_temp_limit_c.read()) {
                reason = "joint temperature limit exceeded (joint " + std::to_string(i) + "): " +
                         std::to_string(s.motor_state[i].temp) + " > " +
                         std::to_string(p_joint_temp_limit_c.read());
                return false;
            }
        }

        if (!strict_level) {
            return true;
        }

        const double qw = s.imu.orientation.w;
        const double qx = s.imu.orientation.x;
        const double qy = s.imu.orientation.y;
        const double qz = s.imu.orientation.z;

        if (!is_finite(qw) || !is_finite(qx) || !is_finite(qy) || !is_finite(qz)) {
            reason = "imu quaternion has non-finite value";
            return false;
        }

        const double qnorm = std::sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
        if (qnorm < std::numeric_limits<double>::epsilon() || std::abs(qnorm - 1.0) > p_imu_quat_norm_tol.read()) {
            reason = "imu quaternion norm invalid";
            return false;
        }

        const Eigen::Quaterniond q(qw / qnorm, qx / qnorm, qy / qnorm, qz / qnorm);
        // 로봇 body z축을 월드 프레임으로 회전 → 월드 up [0,0,1]과의 내적 = cos(tilt)
        // eulerAngles()는 특이점(gimbal lock)에서 ±π로 점프하는 문제가 있으므로
        // 중력 벡터 내적 기반 tilt 각도로 판정한다.
        const Eigen::Vector3d body_up_in_world = q * Eigen::Vector3d::UnitZ();
        const double cos_tilt = std::clamp(body_up_in_world.z(), -1.0, 1.0);
        const double tilt_rad = std::acos(cos_tilt);
        const double max_tilt_rad = p_imu_tilt_limit_deg.read() * M_PI / 180.0;

        if (tilt_rad > max_tilt_rad) {
            reason = "imu tilt limit exceeded (tilt=" + std::to_string(tilt_rad * 180.0 / M_PI) + " deg)";
            return false;
        }

        return true;
    }
};

} // namespace task_pool
