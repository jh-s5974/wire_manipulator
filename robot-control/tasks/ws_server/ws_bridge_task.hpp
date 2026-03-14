#pragma once

#include "ws_bridge.hpp"
#include "custom_types.hpp"
#include <rtfw/task.h>

#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <sstream>
#include <string>

using namespace std::chrono_literals;
using namespace rtfw;
using namespace rtfw::rt;

namespace task_pool {

class WsBridgeTask : public ITask {
public:
    WsBridgeTask() {
        motor_power_on_.fill(false);
    }

    const char* getName() const override { return "WsBridgeTask"; }

    void initialize(void* /*state_ptr*/) override {
        bridge_.set_password(p_password.read());
        bridge_.start();
    }

    void execute(void* /*state_ptr*/) override {
        update_inputs();
        bridge_.publish_state(build_snapshot());

        wsbridge::Event ev;
        while (bridge_.try_pop_event(ev)) {
            handle_event(ev);
        }
    }

private:
    static constexpr int kMotorCount = 16;

    struct MotorIo {
        const char* name;
        DataReader<custom_types::MotorState> state;
        DataReader<custom_types::MotorCmd> safety_cmd;
        DataReader<custom_types::MotorCmd> applied_cmd;
        DataWriter<custom_types::MotorCmd> cmd;
        DataWriter<bool> on;
    };

    std::array<MotorIo, kMotorCount> motors_ = {{
        {"hip_yaw_left", DataReader<custom_types::MotorState>{"hip_yaw_left/state", DependencyType::Weak}, DataReader<custom_types::MotorCmd>{"hip_yaw_left/cmd", DependencyType::Weak}, DataReader<custom_types::MotorCmd>{"hip_yaw_left/cmd_applied", DependencyType::Weak}, DataWriter<custom_types::MotorCmd>{"gui/hip_yaw_left/cmd"}, DataWriter<bool>{"gui/hip_yaw_left/on"}},
        {"hip_yaw_right", DataReader<custom_types::MotorState>{"hip_yaw_right/state", DependencyType::Weak}, DataReader<custom_types::MotorCmd>{"hip_yaw_right/cmd", DependencyType::Weak}, DataReader<custom_types::MotorCmd>{"hip_yaw_right/cmd_applied", DependencyType::Weak}, DataWriter<custom_types::MotorCmd>{"gui/hip_yaw_right/cmd"}, DataWriter<bool>{"gui/hip_yaw_right/on"}},
        {"hip_roll_left", DataReader<custom_types::MotorState>{"hip_roll_left/state", DependencyType::Weak}, DataReader<custom_types::MotorCmd>{"hip_roll_left/cmd", DependencyType::Weak}, DataReader<custom_types::MotorCmd>{"hip_roll_left/cmd_applied", DependencyType::Weak}, DataWriter<custom_types::MotorCmd>{"gui/hip_roll_left/cmd"}, DataWriter<bool>{"gui/hip_roll_left/on"}},
        {"hip_roll_right", DataReader<custom_types::MotorState>{"hip_roll_right/state", DependencyType::Weak}, DataReader<custom_types::MotorCmd>{"hip_roll_right/cmd", DependencyType::Weak}, DataReader<custom_types::MotorCmd>{"hip_roll_right/cmd_applied", DependencyType::Weak}, DataWriter<custom_types::MotorCmd>{"gui/hip_roll_right/cmd"}, DataWriter<bool>{"gui/hip_roll_right/on"}},
        {"hip_pitch_left", DataReader<custom_types::MotorState>{"hip_pitch_left/state", DependencyType::Weak}, DataReader<custom_types::MotorCmd>{"hip_pitch_left/cmd", DependencyType::Weak}, DataReader<custom_types::MotorCmd>{"hip_pitch_left/cmd_applied", DependencyType::Weak}, DataWriter<custom_types::MotorCmd>{"gui/hip_pitch_left/cmd"}, DataWriter<bool>{"gui/hip_pitch_left/on"}},
        {"hip_pitch_right", DataReader<custom_types::MotorState>{"hip_pitch_right/state", DependencyType::Weak}, DataReader<custom_types::MotorCmd>{"hip_pitch_right/cmd", DependencyType::Weak}, DataReader<custom_types::MotorCmd>{"hip_pitch_right/cmd_applied", DependencyType::Weak}, DataWriter<custom_types::MotorCmd>{"gui/hip_pitch_right/cmd"}, DataWriter<bool>{"gui/hip_pitch_right/on"}},
        {"knee_left", DataReader<custom_types::MotorState>{"knee_left/state", DependencyType::Weak}, DataReader<custom_types::MotorCmd>{"knee_left/cmd", DependencyType::Weak}, DataReader<custom_types::MotorCmd>{"knee_left/cmd_applied", DependencyType::Weak}, DataWriter<custom_types::MotorCmd>{"gui/knee_left/cmd"}, DataWriter<bool>{"gui/knee_left/on"}},
        {"knee_right", DataReader<custom_types::MotorState>{"knee_right/state", DependencyType::Weak}, DataReader<custom_types::MotorCmd>{"knee_right/cmd", DependencyType::Weak}, DataReader<custom_types::MotorCmd>{"knee_right/cmd_applied", DependencyType::Weak}, DataWriter<custom_types::MotorCmd>{"gui/knee_right/cmd"}, DataWriter<bool>{"gui/knee_right/on"}},
        {"ankle_pitch_left", DataReader<custom_types::MotorState>{"ankle_pitch_left/state", DependencyType::Weak}, DataReader<custom_types::MotorCmd>{"ankle_pitch_left/cmd", DependencyType::Weak}, DataReader<custom_types::MotorCmd>{"ankle_pitch_left/cmd_applied", DependencyType::Weak}, DataWriter<custom_types::MotorCmd>{"gui/ankle_pitch_left/cmd"}, DataWriter<bool>{"gui/ankle_pitch_left/on"}},
        {"ankle_pitch_right", DataReader<custom_types::MotorState>{"ankle_pitch_right/state", DependencyType::Weak}, DataReader<custom_types::MotorCmd>{"ankle_pitch_right/cmd", DependencyType::Weak}, DataReader<custom_types::MotorCmd>{"ankle_pitch_right/cmd_applied", DependencyType::Weak}, DataWriter<custom_types::MotorCmd>{"gui/ankle_pitch_right/cmd"}, DataWriter<bool>{"gui/ankle_pitch_right/on"}},
        {"ankle_roll_left", DataReader<custom_types::MotorState>{"ankle_roll_left/state", DependencyType::Weak}, DataReader<custom_types::MotorCmd>{"ankle_roll_left/cmd", DependencyType::Weak}, DataReader<custom_types::MotorCmd>{"ankle_roll_left/cmd_applied", DependencyType::Weak}, DataWriter<custom_types::MotorCmd>{"gui/ankle_roll_left/cmd"}, DataWriter<bool>{"gui/ankle_roll_left/on"}},
        {"ankle_roll_right", DataReader<custom_types::MotorState>{"ankle_roll_right/state", DependencyType::Weak}, DataReader<custom_types::MotorCmd>{"ankle_roll_right/cmd", DependencyType::Weak}, DataReader<custom_types::MotorCmd>{"ankle_roll_right/cmd_applied", DependencyType::Weak}, DataWriter<custom_types::MotorCmd>{"gui/ankle_roll_right/cmd"}, DataWriter<bool>{"gui/ankle_roll_right/on"}},
        {"ankle_upper_left", DataReader<custom_types::MotorState>{"ankle_upper_left/state", DependencyType::Weak}, DataReader<custom_types::MotorCmd>{"ankle_upper_left/cmd", DependencyType::Weak}, DataReader<custom_types::MotorCmd>{"ankle_upper_left/cmd_applied", DependencyType::Weak}, DataWriter<custom_types::MotorCmd>{"gui/ankle_upper_left/cmd"}, DataWriter<bool>{"gui/ankle_upper_left/on"}},
        {"ankle_upper_right", DataReader<custom_types::MotorState>{"ankle_upper_right/state", DependencyType::Weak}, DataReader<custom_types::MotorCmd>{"ankle_upper_right/cmd", DependencyType::Weak}, DataReader<custom_types::MotorCmd>{"ankle_upper_right/cmd_applied", DependencyType::Weak}, DataWriter<custom_types::MotorCmd>{"gui/ankle_upper_right/cmd"}, DataWriter<bool>{"gui/ankle_upper_right/on"}},
        {"ankle_lower_left", DataReader<custom_types::MotorState>{"ankle_lower_left/state", DependencyType::Weak}, DataReader<custom_types::MotorCmd>{"ankle_lower_left/cmd", DependencyType::Weak}, DataReader<custom_types::MotorCmd>{"ankle_lower_left/cmd_applied", DependencyType::Weak}, DataWriter<custom_types::MotorCmd>{"gui/ankle_lower_left/cmd"}, DataWriter<bool>{"gui/ankle_lower_left/on"}},
        {"ankle_lower_right", DataReader<custom_types::MotorState>{"ankle_lower_right/state", DependencyType::Weak}, DataReader<custom_types::MotorCmd>{"ankle_lower_right/cmd", DependencyType::Weak}, DataReader<custom_types::MotorCmd>{"ankle_lower_right/cmd_applied", DependencyType::Weak}, DataWriter<custom_types::MotorCmd>{"gui/ankle_lower_right/cmd"}, DataWriter<bool>{"gui/ankle_lower_right/on"}}
    }};

    DataReader<custom_types::Imu> dr_imu_{"imu_data", DependencyType::Weak};
    DataReader<bool> dr_motor_on_actual_[kMotorCount] = {
        DataReader<bool>{"hip_yaw_left/on", DependencyType::Weak},
        DataReader<bool>{"hip_yaw_right/on", DependencyType::Weak},
        DataReader<bool>{"hip_roll_left/on", DependencyType::Weak},
        DataReader<bool>{"hip_roll_right/on", DependencyType::Weak},
        DataReader<bool>{"hip_pitch_left/on", DependencyType::Weak},
        DataReader<bool>{"hip_pitch_right/on", DependencyType::Weak},
        DataReader<bool>{"knee_left/on", DependencyType::Weak},
        DataReader<bool>{"knee_right/on", DependencyType::Weak},
        DataReader<bool>{"ankle_pitch_left/on", DependencyType::Weak},
        DataReader<bool>{"ankle_pitch_right/on", DependencyType::Weak},
        DataReader<bool>{"ankle_roll_left/on", DependencyType::Weak},
        DataReader<bool>{"ankle_roll_right/on", DependencyType::Weak},
        DataReader<bool>{"ankle_pitch_left/on", DependencyType::Weak},
        DataReader<bool>{"ankle_pitch_right/on", DependencyType::Weak},
        DataReader<bool>{"ankle_roll_left/on", DependencyType::Weak},
        DataReader<bool>{"ankle_roll_right/on", DependencyType::Weak},
    };
    DataReader<bool> dr_motor_control_requested_{"gui/motor/control_requested", DependencyType::Weak};
    DataReader<bool> dr_motor_control_granted_{"gui/motor/control_granted", DependencyType::Weak};
    DataReader<int> dr_robot_mode_current_{"gui/robot/mode_current", DependencyType::Weak};
    DataReader<int> dr_safety_level_{"manager/safety_level", DependencyType::Weak};
    DataReader<bool> dr_safety_lock_{"safety/lock_state", DependencyType::Weak};
    DataReader<bool> dr_safety_restore_{"safety/restore_state", DependencyType::Weak};
    DataReader<bool> dr_walk_ready_{"manager/walk_ready", DependencyType::Weak};
    DataWriter<bool> dw_motor_control_request_{"gui/motor/control_request"};
    DataWriter<int> dw_robot_mode_request_{"gui/robot/mode_request"};
    DataWriter<rt::Signal> dw_safety_reset_{"manager/reset_signal"};
    Parameter<std::string> p_password{"gui/control_password", ""};
    Parameter<std::vector<std::string>> p_joint_names_{"joint_states/names"};
    wsbridge::WebsocketBridge bridge_{8080};

    std::array<custom_types::MotorState, kMotorCount> motor_cache_{};
    std::array<custom_types::MotorCmd, kMotorCount> safety_cmd_cache_{};
    std::array<bool, kMotorCount> safety_cmd_online_{};
    std::array<custom_types::MotorCmd, kMotorCount> applied_cmd_cache_{};
    std::array<bool, kMotorCount> applied_cmd_online_{};
    std::array<bool, kMotorCount> motor_online_{};
    std::array<bool, kMotorCount> motor_power_on_{};
    std::array<bool, kMotorCount> motor_on_actual_{};
    std::array<custom_types::MotorCmd, kMotorCount> cmd_cache_{};

    static constexpr int kMaxJointCount = custom_types::kMaxJointCount;

    custom_types::Imu imu_cache_{};
    bool imu_online_ = false;
    int joint_count_ = 0;
    bool control_requested_ = false;
    bool control_granted_ = false;
    int robot_mode_current_ = 0;
    int safety_level_ = 0;
    bool safety_locked_ = false;
    bool safety_restoring_ = false;
    bool walk_ready_ = false;

    static wsbridge::Vec3 quat_to_rpy(double w, double x, double y, double z) {
        const double sinr_cosp = 2.0 * (w * x + y * z);
        const double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
        const double roll = std::atan2(sinr_cosp, cosr_cosp);

        const double sinp = 2.0 * (w * y - z * x);
        const double pitch = std::abs(sinp) >= 1.0 ? std::copysign(M_PI / 2.0, sinp) : std::asin(sinp);

        const double siny_cosp = 2.0 * (w * z + x * y);
        const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
        const double yaw = std::atan2(siny_cosp, cosy_cosp);

        return {roll, pitch, yaw};
    }

    static std::string mode_from_status(int status) {
        if (status == 0) return "IDLE";
        return "POSITION";
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

    static std::optional<int> robot_mode_from_string(const std::string& mode) {
        if (mode == "IDLE") return 0;
        if (mode == "MANU") return 1;
        if (mode == "READY") return 2;
        if (mode == "RL WALK") return 3;
        return std::nullopt;
    }

    static std::string safety_level_to_string(int level) {
        return level == 1 ? "STRICT" : "ESSENTIAL";
    }

    void update_inputs() {
        for (int index = 0; index < kMotorCount; ++index) {
            motors_[index].state.on_update([&, index](const custom_types::MotorState& data) {
                motor_cache_[index] = data;
                motor_online_[index] = true;
            });

            motors_[index].safety_cmd.on_update([&, index](const custom_types::MotorCmd& data) {
                safety_cmd_cache_[index] = data;
                safety_cmd_online_[index] = true;
            });

            motors_[index].applied_cmd.on_update([&, index](const custom_types::MotorCmd& data) {
                applied_cmd_cache_[index] = data;
                applied_cmd_online_[index] = true;
            });

            dr_motor_on_actual_[index].on_update([&, index](const bool& on) {
                motor_on_actual_[index] = on;
            });
        }

        dr_imu_.on_update([&](const custom_types::Imu& data) {
            imu_cache_ = data;
            imu_online_ = true;
        });

        dr_motor_control_requested_.on_update([&](const bool& requested) {
            control_requested_ = requested;
        });

        dr_motor_control_granted_.on_update([&](const bool& granted) {
            control_granted_ = granted;
        });

        dr_robot_mode_current_.on_update([&](const int& mode) {
            robot_mode_current_ = mode;
        });

        dr_safety_level_.on_update([&](const int& level) {
            safety_level_ = level;
        });

        dr_safety_lock_.on_update([&](const bool& locked) {
            safety_locked_ = locked;
        });

        dr_safety_restore_.on_update([&](const bool& restoring) {
            safety_restoring_ = restoring;
        });

        dr_walk_ready_.on_update([&](const bool& ready) {
            walk_ready_ = ready;
        });
    }

    wsbridge::BridgeState build_snapshot() {
        wsbridge::BridgeState state;
        state.control.requested = control_requested_;
        state.control.granted = control_granted_;
        state.robot_mode.current = robot_mode_to_string(robot_mode_current_);
        state.safety.level = safety_level_to_string(safety_level_);
        state.safety.locked = safety_locked_;
        state.safety.restoring = safety_restoring_;
        state.robot_mode.walk_ready = walk_ready_;
        state.motors.reserve(kMotorCount);
        state.joint_states.reserve(12);

        for (int index = 0; index < kMotorCount; ++index) {
            const auto& in = motor_cache_[index];
            wsbridge::MotorSnapshot out{};
            out.id = index;
            out.name = motors_[index].name;
            out.mode = mode_from_status(in.status);
            out.position = in.pos;
            out.velocity = in.vel;
            out.torque = in.torque;
            out.temperature = in.temp;
            out.error = false;
            out.warning = (in.status != 0);
            if (safety_cmd_online_[index]) {
                const auto& sc = safety_cmd_cache_[index];
                out.command_position = sc.pos;
                out.command_velocity = sc.vel;
                out.command_torque = sc.torque;
                out.command_kp = sc.kp;
                out.command_kd = sc.kd;
                out.kp = sc.kp;
                out.kd = sc.kd;
            } else {
                out.command_position = 0.0;
                out.command_velocity = 0.0;
                out.command_torque = 0.0;
                out.command_kp = 0.0;
                out.command_kd = 0.0;
                out.kp = 0.0;
                out.kd = 0.0;
            }
            if (applied_cmd_online_[index]) {
                const auto& applied = applied_cmd_cache_[index];
                out.driver_command_position = applied.pos;
                out.driver_command_velocity = applied.vel;
                out.driver_command_torque = applied.torque;
                out.driver_command_kp = applied.kp;
                out.driver_command_kd = applied.kd;
            } else {
                out.driver_command_position = 0.0;
                out.driver_command_velocity = 0.0;
                out.driver_command_torque = 0.0;
                out.driver_command_kp = 0.0;
                out.driver_command_kd = 0.0;
            }
            out.enabled = motor_on_actual_[index];
            state.motors.push_back(std::move(out));
        }

        if (imu_online_) {
            state.imu.rpy = quat_to_rpy(
                imu_cache_.orientation.w,
                imu_cache_.orientation.x,
                imu_cache_.orientation.y,
                imu_cache_.orientation.z
            );
            state.imu.gyro = {imu_cache_.gyro.x, imu_cache_.gyro.y, imu_cache_.gyro.z};
            state.imu.accel = {imu_cache_.acc.x, imu_cache_.acc.y, imu_cache_.acc.z};
        } else {
            state.imu.rpy = {0.0, 0.0, 0.0};
            state.imu.gyro = {0.0, 0.0, 0.0};
            state.imu.accel = {0.0, 0.0, 0.0};
        }

        // state.joint_states.clear();
        std::vector<std::string> joint_names = p_joint_names_.read();
        state.joint_states[joint_names[0]] = 0;//motor_cache_[0].pos;
        state.joint_states[joint_names[1]] = 0;//motor_cache_[2].pos;
        state.joint_states[joint_names[2]] = 0;//motor_cache_[4].pos;
        state.joint_states[joint_names[3]] = 0;//motor_cache_[6].pos;
        state.joint_states[joint_names[4]] = 0;//motor_cache_[8].pos;
        state.joint_states[joint_names[5]] = 0;//motor_cache_[10].pos;
        state.joint_states[joint_names[6]] = 0;//motor_cache_[1].pos;
        state.joint_states[joint_names[7]] = 0;//motor_cache_[3].pos;
        state.joint_states[joint_names[8]] = 0;//motor_cache_[5].pos;
        state.joint_states[joint_names[9]] = 0;//motor_cache_[7].pos;
        state.joint_states[joint_names[10]] = 0;//motor_cache_[9].pos;
        state.joint_states[joint_names[11]] = 0;//motor_cache_[11].pos;

        return state;
    }

    void handle_event(const wsbridge::Event& ev) {
        switch (ev.kind) {
        case wsbridge::Event::Kind::SubscribeState:
            break;

        case wsbridge::Event::Kind::MotorPower:
            handle_motor_power(ev.power);
            break;

        case wsbridge::Event::Kind::MotorCommand:
            handle_motor_command(ev.command);
            break;

        case wsbridge::Event::Kind::MotorControlRequest:
            handle_motor_control_request(ev.control_request);
            break;

        case wsbridge::Event::Kind::RobotModeRequest:
            handle_robot_mode_request(ev.robot_mode_request);
            break;

        case wsbridge::Event::Kind::SafetyReset:
            handle_safety_reset(ev.safety_reset);
            break;
        }
    }

    void handle_safety_reset(const wsbridge::Event::SafetyResetPayload& req) {
        if (!req.request) {
            return;
        }

        dw_safety_reset_.write();
        getLogger()->info("[{}] Safety reset requested from GUI", getName());
    }

    void handle_robot_mode_request(const wsbridge::Event::RobotModeRequestPayload& req) {
        const auto mode = robot_mode_from_string(req.mode);
        if (!mode.has_value()) {
            getLogger()->warn("[{}] Ignoring robot mode request: unsupported mode='{}'", getName(), req.mode);
            return;
        }

        dw_robot_mode_request_.write(*mode);
        getLogger()->info("[{}] Robot mode request={} received from GUI", getName(), req.mode);
    }

    void handle_motor_control_request(const wsbridge::Event::ControlRequestPayload& req) {
        control_requested_ = req.request;
        if (!req.request) {
            control_granted_ = false;
        }
        dw_motor_control_request_.write(req.request);
        getLogger()->info("[{}] Motor control request={} received from GUI", getName(), req.request ? "true" : "false");
    }

    void handle_motor_power(const wsbridge::Event::PowerPayload& power) {
        if (!control_granted_) {
            getLogger()->warn("[{}] Ignoring motor power from GUI: control not granted", getName());
            return;
        }

        if (power.is_all) {
            for (int index = 0; index < kMotorCount; ++index) {
                motors_[index].on.write(power.on);
                motor_power_on_[index] = power.on;
                getLogger()->debug("[{}] Set power {} for motor {} ({})", getName(), power.on ? "ON" : "OFF", index, motors_[index].name);
            }
            return;
        }

        if (power.motor_id < 0 || power.motor_id >= kMotorCount) {
            return;
        }

        motors_[power.motor_id].on.write(power.on);
        motor_power_on_[power.motor_id] = power.on;
        getLogger()->debug("[{}] Set power {} for motor {} ({})", getName(), power.on ? "ON" : "OFF", power.motor_id, motors_[power.motor_id].name);
    }

    void handle_motor_command(const wsbridge::Event::CommandPayload& command) {
        if (!control_granted_) {
            getLogger()->warn("[{}] Ignoring motor command from GUI: control not granted", getName());
            return;
        }

        auto apply = [&](int index) {
            if (index < 0 || index >= kMotorCount) {
                return;
            }

            auto& cmd = cmd_cache_[index];
            std::memset(cmd.name, 0, sizeof(cmd.name));
            std::snprintf(cmd.name, sizeof(cmd.name), "%s", motors_[index].name);

            if (command.cmd.position) cmd.pos = *command.cmd.position;
            if (command.cmd.velocity) cmd.vel = *command.cmd.velocity;
            if (command.cmd.torque) cmd.torque = *command.cmd.torque;
            if (command.cmd.kp) cmd.kp = *command.cmd.kp;
            if (command.cmd.kd) cmd.kd = *command.cmd.kd;
            cmd.duration_ms = command.cmd.duration_ms;

            motors_[index].cmd.write(cmd);
            getLogger()->debug("[{}] Set command for motor {} ({}): pos={}, vel={}, torque={}, kp={}, kd={}", getName(),
                               index, motors_[index].name, cmd.pos, cmd.vel, cmd.torque, cmd.kp, cmd.kd);
        };

        if (command.is_all) {
            for (int index = 0; index < kMotorCount; ++index) {
                apply(index);
            }
            return;
        }

        apply(command.motor_id);
    }
};

} // namespace task_pool
