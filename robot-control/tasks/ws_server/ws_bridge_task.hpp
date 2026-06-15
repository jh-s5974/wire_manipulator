#pragma once

// WsBridgeTask — 매니퓰레이터용
//
// GUI 화면 구성:
//   1. 비밀번호 입력 (auth_required → auth_ok)
//   2. 조인트 상태 표시 (joint0~4 위치/속도/토크)
//   3. 모터 on/off 및 위치 명령 (MANU 상태에서 control_granted 후 사용)
//   4. IDLE/MANU 모드 전환만 허용 (READY, RL WALK 제거)
//   5. 세이프티 LOCK 해제 버튼

#include "ws_bridge.hpp"
#include "custom_types.hpp"
#include <rtfw/task.h>

#include <array>
#include <cmath>
#include <cstring>
#include <string>

using namespace std::chrono_literals;
using namespace rtfw;
using namespace rtfw::rt;

namespace task_pool {

class WsBridgeTask : public ITask {
public:
    WsBridgeTask() { motor_power_on_.fill(false); }

    const char* getName() const override { return "WsBridgeTask"; }

    void initialize(void*) override {
        bridge_.set_password(p_password.read());
        bridge_.start();
    }

    void execute(void*) override {
        update_inputs();
        bridge_.publish_state(build_snapshot());

        wsbridge::Event ev;
        while (bridge_.try_pop_event(ev))
            handle_event(ev);
    }

private:
    // ── 조인트 수 ──
    static constexpr int kJointCount = 5;
    // ── 물리 모터 수 및 채널 이름 ──
    static constexpr int kPhysCount = 7;
    static constexpr const char* kPhysChannels[kPhysCount] = {
        "phys_motor/m01/state", "phys_motor/m02/state",
        "phys_motor/m03/state", "phys_motor/m04/state",
        "phys_motor/m05/state", "phys_motor/m06/state",
        "phys_motor/m07/state",
    };
    static constexpr const char* kPhysNames[kPhysCount] = {
        "m01","m02","m03","m04","m05","m06","m07"
    };

    struct JointIo {
        const char* name;
        DataReader<custom_types::MotorState> state;
        DataReader<custom_types::MotorCmd>   safety_cmd;
        DataReader<custom_types::MotorCmd>   applied_cmd;
        DataWriter<custom_types::MotorCmd>   cmd;
        DataWriter<bool>                     on;
    };

    std::array<JointIo, kJointCount> joints_ = {{
        {"joint0",
         DataReader<custom_types::MotorState>{"joint0/state",       DependencyType::Weak},
         DataReader<custom_types::MotorCmd>  {"joint0/cmd",         DependencyType::Weak},
         DataReader<custom_types::MotorCmd>  {"joint0/cmd_applied", DependencyType::Weak},
         DataWriter<custom_types::MotorCmd>  {"gui/joint0/cmd"},
         DataWriter<bool>                    {"gui/joint0/on"}},
        {"joint1",
         DataReader<custom_types::MotorState>{"joint1/state",       DependencyType::Weak},
         DataReader<custom_types::MotorCmd>  {"joint1/cmd",         DependencyType::Weak},
         DataReader<custom_types::MotorCmd>  {"joint1/cmd_applied", DependencyType::Weak},
         DataWriter<custom_types::MotorCmd>  {"gui/joint1/cmd"},
         DataWriter<bool>                    {"gui/joint1/on"}},
        {"joint2",
         DataReader<custom_types::MotorState>{"joint2/state",       DependencyType::Weak},
         DataReader<custom_types::MotorCmd>  {"joint2/cmd",         DependencyType::Weak},
         DataReader<custom_types::MotorCmd>  {"joint2/cmd_applied", DependencyType::Weak},
         DataWriter<custom_types::MotorCmd>  {"gui/joint2/cmd"},
         DataWriter<bool>                    {"gui/joint2/on"}},
        {"joint3",
         DataReader<custom_types::MotorState>{"joint3/state",       DependencyType::Weak},
         DataReader<custom_types::MotorCmd>  {"joint3/cmd",         DependencyType::Weak},
         DataReader<custom_types::MotorCmd>  {"joint3/cmd_applied", DependencyType::Weak},
         DataWriter<custom_types::MotorCmd>  {"gui/joint3/cmd"},
         DataWriter<bool>                    {"gui/joint3/on"}},
        {"joint4",
         DataReader<custom_types::MotorState>{"joint4/state",       DependencyType::Weak},
         DataReader<custom_types::MotorCmd>  {"joint4/cmd",         DependencyType::Weak},
         DataReader<custom_types::MotorCmd>  {"joint4/cmd_applied", DependencyType::Weak},
         DataWriter<custom_types::MotorCmd>  {"gui/joint4/cmd"},
         DataWriter<bool>                    {"gui/joint4/on"}},
    }};

    // ── 물리 모터 상태 채널 (GUI 모터 뷰용) ──
    DataReader<custom_types::MotorState> dr_phys_state_[kPhysCount] = {
        DataReader<custom_types::MotorState>{kPhysChannels[0], DependencyType::Weak},
        DataReader<custom_types::MotorState>{kPhysChannels[1], DependencyType::Weak},
        DataReader<custom_types::MotorState>{kPhysChannels[2], DependencyType::Weak},
        DataReader<custom_types::MotorState>{kPhysChannels[3], DependencyType::Weak},
        DataReader<custom_types::MotorState>{kPhysChannels[4], DependencyType::Weak},
        DataReader<custom_types::MotorState>{kPhysChannels[5], DependencyType::Weak},
        DataReader<custom_types::MotorState>{kPhysChannels[6], DependencyType::Weak},
    };

    // 공통 GUI 상태 채널
    DataReader<bool> dr_control_requested_{"gui/motor/control_requested", DependencyType::Weak};
    DataReader<bool> dr_control_granted_  {"gui/motor/control_granted",   DependencyType::Weak};
    DataReader<int>  dr_robot_mode_       {"gui/robot/mode_current",      DependencyType::Weak};
    DataReader<bool> dr_safety_lock_      {"safety/lock_state",           DependencyType::Weak};
    DataReader<bool> dr_safety_restore_   {"safety/restore_state",        DependencyType::Weak};

    // 데이터 로거 채널
    DataReader<custom_types::LoggerInfo> dr_logger_info_{"data_logger/info", DependencyType::Weak};
    DataWriter<bool>                     dw_record_cmd_  {"data_logger/record_cmd"};

    DataWriter<bool>       dw_control_request_{"gui/motor/control_request"};
    DataWriter<int>        dw_robot_mode_req_  {"gui/robot/mode_request"};
    DataWriter<rt::Signal> dw_safety_reset_    {"manager/reset_signal"};

    Parameter<std::string> p_password{"gui/control_password", ""};
    Parameter<double> p_default_duration_sec{"ws_bridge.default_duration_sec", 0.0};

    wsbridge::WebsocketBridge bridge_{8080};

    // ── 캐시 ──
    std::array<custom_types::MotorState, kPhysCount>  phys_state_cache_{};
    std::array<bool,                     kPhysCount>  phys_online_{};
    std::array<custom_types::MotorState, kJointCount> state_cache_{};
    std::array<custom_types::MotorCmd,   kJointCount> safety_cmd_cache_{};
    std::array<bool,                     kJointCount> safety_cmd_online_{};
    std::array<custom_types::MotorCmd,   kJointCount> applied_cmd_cache_{};
    std::array<bool,                     kJointCount> applied_cmd_online_{};
    std::array<bool,                     kJointCount> joint_online_{};
    std::array<bool,                     kJointCount> motor_power_on_{};
    std::array<custom_types::MotorCmd,   kJointCount> cmd_cache_{};

    bool control_requested_ = false;
    bool control_granted_   = false;
    int  robot_mode_        = 0;
    bool safety_locked_     = false;
    bool safety_restoring_  = false;

    custom_types::LoggerInfo logger_info_{};

    // ── 입력 갱신 ──
    void update_inputs() {
        for (int i = 0; i < kPhysCount; i++) {
            dr_phys_state_[i].on_update([&, i](const custom_types::MotorState& d) {
                phys_state_cache_[i] = d;
                phys_online_[i]      = true;
            });
        }
        for (int i = 0; i < kJointCount; i++) {
            joints_[i].state.on_update([&, i](const custom_types::MotorState& d) {
                state_cache_[i]  = d;
                joint_online_[i] = true;
            });
            joints_[i].safety_cmd.on_update([&, i](const custom_types::MotorCmd& d) {
                safety_cmd_cache_[i]  = d;
                safety_cmd_online_[i] = true;
            });
            joints_[i].applied_cmd.on_update([&, i](const custom_types::MotorCmd& d) {
                applied_cmd_cache_[i]  = d;
                applied_cmd_online_[i] = true;
            });
        }
        dr_control_requested_.on_update([&](const bool& v) { control_requested_ = v; });
        dr_control_granted_.on_update([&](const bool& v)   { control_granted_   = v; });
        dr_robot_mode_.on_update([&](const int& v)          { robot_mode_        = v; });
        dr_safety_lock_.on_update([&](const bool& v)        { safety_locked_     = v; });
        dr_safety_restore_.on_update([&](const bool& v)     { safety_restoring_  = v; });
        dr_logger_info_.on_update([&](const custom_types::LoggerInfo& v) { logger_info_ = v; });
    }

    // ── 상태 스냅샷 생성 ──
    wsbridge::BridgeState build_snapshot() {
        wsbridge::BridgeState st;
        st.control.requested = control_requested_;
        st.control.granted   = control_granted_;
        st.robot_mode.current = robot_mode_to_string(robot_mode_);
        st.safety.locked     = safety_locked_;
        st.safety.restoring  = safety_restoring_;
        st.safety.level      = "ESSENTIAL";

        for (int i = 0; i < kJointCount; i++) {
            const auto& s = state_cache_[i];
            wsbridge::MotorSnapshot snap{};
            snap.id      = i;
            snap.name    = joints_[i].name;
            snap.mode    = (s.status == 0) ? "IDLE" : "POSITION";
            snap.position    = s.pos;
            snap.velocity    = s.vel;
            snap.torque      = s.torque;
            snap.temperature = s.temp;
            snap.warning     = (s.status != 0);
            snap.enabled     = motor_power_on_[i];

            if (safety_cmd_online_[i]) {
                snap.command_position = safety_cmd_cache_[i].pos;
                snap.command_velocity = safety_cmd_cache_[i].vel;
                snap.command_torque   = safety_cmd_cache_[i].torque;
                snap.command_kp       = safety_cmd_cache_[i].kp;
                snap.command_kd       = safety_cmd_cache_[i].kd;
                snap.kp = safety_cmd_cache_[i].kp;
                snap.kd = safety_cmd_cache_[i].kd;
            }
            if (applied_cmd_online_[i]) {
                snap.driver_command_position = applied_cmd_cache_[i].pos;
                snap.driver_command_velocity = applied_cmd_cache_[i].vel;
                snap.driver_command_torque   = applied_cmd_cache_[i].torque;
                snap.driver_command_kp       = applied_cmd_cache_[i].kp;
                snap.driver_command_kd       = applied_cmd_cache_[i].kd;
            }
            st.motors.push_back(std::move(snap));

            // joint_states 맵에도 추가
            st.joint_states[joints_[i].name] = state_cache_[i].pos;
        }

        // 물리 모터 raw 상태 (GUI 모터 뷰용)
        for (int i = 0; i < kPhysCount; i++) {
            const auto& ps = phys_state_cache_[i];
            wsbridge::MotorSnapshot psnap{};
            psnap.id       = i + 1; // 1-based: m01=1, m02=2, ...
            psnap.name     = kPhysNames[i];
            psnap.mode     = phys_online_[i] ? "RAW" : "N/A";
            psnap.position = ps.pos;
            psnap.velocity = ps.vel;
            psnap.torque   = ps.torque;
            psnap.enabled  = ps.enabled;
            st.physical_motors.push_back(std::move(psnap));
        }

        // 데이터 로거 상태
        st.data_logger.recording    = logger_info_.recording;
        st.data_logger.sample_count = logger_info_.sample_count;
        st.data_logger.filename     = logger_info_.filename;

        return st;
    }

    // ── 이벤트 처리 ──
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
                handle_control_request(ev.control_request);
                break;
            case wsbridge::Event::Kind::RobotModeRequest:
                handle_mode_request(ev.robot_mode_request);
                break;
            case wsbridge::Event::Kind::SafetyReset:
                handle_safety_reset(ev.safety_reset);
                break;
            case wsbridge::Event::Kind::DataLogger:
                handle_data_logger(ev.data_logger);
                break;
        }
    }

    void handle_control_request(const wsbridge::Event::ControlRequestPayload& req) {
        control_requested_ = req.request;
        if (!req.request) control_granted_ = false;
        dw_control_request_.write(req.request);
        getLogger()->info("[{}] control_request={}", getName(), req.request);
    }

    // IDLE(0) / MANU(1) 만 허용
    void handle_mode_request(const wsbridge::Event::RobotModeRequestPayload& req) {
        int mode = -1;
        if (req.mode == "IDLE") mode = 0;
        else if (req.mode == "MANU") mode = 1;

        if (mode < 0) {
            getLogger()->warn("[{}] Mode '{}' not supported for manipulator", getName(), req.mode);
            return;
        }
        dw_robot_mode_req_.write(mode);
        getLogger()->info("[{}] mode_request={}", getName(), req.mode);
    }

    void handle_safety_reset(const wsbridge::Event::SafetyResetPayload& req) {
        if (!req.request) return;
        dw_safety_reset_.write();
        getLogger()->info("[{}] safety reset requested", getName());
    }

    void handle_motor_power(const wsbridge::Event::PowerPayload& pwr) {
        if (!control_granted_) {
            getLogger()->warn("[{}] motor power ignored: not granted", getName());
            return;
        }
        auto apply = [&](int idx) {
            if (idx < 0 || idx >= kJointCount) return;
            joints_[idx].on.write(pwr.on);
            motor_power_on_[idx] = pwr.on;
        };
        if (pwr.is_all) for (int i = 0; i < kJointCount; i++) apply(i);
        else apply(pwr.motor_id);
    }

    void handle_motor_command(const wsbridge::Event::CommandPayload& cmd) {
        if (!control_granted_) {
            getLogger()->warn("[{}] motor cmd ignored: not granted", getName());
            return;
        }
        auto apply = [&](int idx) {
            if (idx < 0 || idx >= kJointCount) return;
            auto& c = cmd_cache_[idx];
            std::snprintf(c.name, sizeof(c.name), "%s", joints_[idx].name);
            if (cmd.cmd.position)  c.pos    = *cmd.cmd.position;
            if (cmd.cmd.velocity)  c.vel    = *cmd.cmd.velocity;
            if (cmd.cmd.torque)    c.torque = *cmd.cmd.torque;
            if (cmd.cmd.kp)        c.kp     = *cmd.cmd.kp;
            if (cmd.cmd.kd)        c.kd     = *cmd.cmd.kd;
            // yaml 파라미터가 있으면 GUI 값 무시하고 적용
            const double yaml_dur_ms = p_default_duration_sec.read() * 1000.0;
            c.duration_ms = (yaml_dur_ms >= 0.0) ? yaml_dur_ms : cmd.cmd.duration_ms;
            joints_[idx].cmd.write(c);
        };
        if (cmd.is_all) for (int i = 0; i < kJointCount; i++) apply(i);
        else apply(cmd.motor_id);
    }

    void handle_data_logger(const wsbridge::Event::DataLoggerPayload& payload) {
        dw_record_cmd_.write(payload.start);
        getLogger()->info("[{}] data_logger record={}", getName(), payload.start);
    }

    // ── 유틸리티 ──
    static std::string robot_mode_to_string(int mode) {
        return mode == 1 ? "MANU" : "IDLE";
    }
};

} // namespace task_pool
