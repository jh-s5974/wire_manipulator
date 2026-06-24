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
    static constexpr const char* kPhysIoChannels[kPhysCount] = {
        "phys_motor/m01/io", "phys_motor/m02/io",
        "phys_motor/m03/io", "phys_motor/m04/io",
        "phys_motor/m05/io", "phys_motor/m06/io",
        "phys_motor/m07/io",
    };
    // 물리 모터에 실제로 적용된 명령 echo — GUI Sync가 "현재 값"을 보여주는 데 사용
    static constexpr const char* kPhysCmdAppliedChannels[kPhysCount] = {
        "phys_motor/m01/cmd_applied", "phys_motor/m02/cmd_applied",
        "phys_motor/m03/cmd_applied", "phys_motor/m04/cmd_applied",
        "phys_motor/m05/cmd_applied", "phys_motor/m06/cmd_applied",
        "phys_motor/m07/cmd_applied",
    };
    // 모터 명령 모드용 — 물리 모터 직접 명령/on 채널 (CanBus0/1이 구독, IK 우회)
    static constexpr const char* kPhysCmdChannels[kPhysCount] = {
        "gui/phys_motor/m01/cmd", "gui/phys_motor/m02/cmd",
        "gui/phys_motor/m03/cmd", "gui/phys_motor/m04/cmd",
        "gui/phys_motor/m05/cmd", "gui/phys_motor/m06/cmd",
        "gui/phys_motor/m07/cmd",
    };
    static constexpr const char* kPhysOnChannels[kPhysCount] = {
        "gui/phys_motor/m01/on", "gui/phys_motor/m02/on",
        "gui/phys_motor/m03/on", "gui/phys_motor/m04/on",
        "gui/phys_motor/m05/on", "gui/phys_motor/m06/on",
        "gui/phys_motor/m07/on",
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

    // ── 물리 모터 적용 명령 echo 채널 (GUI Sync용) ──
    DataReader<custom_types::MotorCmd> dr_phys_cmd_applied_[kPhysCount] = {
        DataReader<custom_types::MotorCmd>{kPhysCmdAppliedChannels[0], DependencyType::Weak},
        DataReader<custom_types::MotorCmd>{kPhysCmdAppliedChannels[1], DependencyType::Weak},
        DataReader<custom_types::MotorCmd>{kPhysCmdAppliedChannels[2], DependencyType::Weak},
        DataReader<custom_types::MotorCmd>{kPhysCmdAppliedChannels[3], DependencyType::Weak},
        DataReader<custom_types::MotorCmd>{kPhysCmdAppliedChannels[4], DependencyType::Weak},
        DataReader<custom_types::MotorCmd>{kPhysCmdAppliedChannels[5], DependencyType::Weak},
        DataReader<custom_types::MotorCmd>{kPhysCmdAppliedChannels[6], DependencyType::Weak},
    };

    // ── 물리 모터 CAN tx/rx 통계 채널 (GUI 모터 뷰 진단용) ──
    DataReader<custom_types::MotorIoStats> dr_phys_io_[kPhysCount] = {
        DataReader<custom_types::MotorIoStats>{kPhysIoChannels[0], DependencyType::Weak},
        DataReader<custom_types::MotorIoStats>{kPhysIoChannels[1], DependencyType::Weak},
        DataReader<custom_types::MotorIoStats>{kPhysIoChannels[2], DependencyType::Weak},
        DataReader<custom_types::MotorIoStats>{kPhysIoChannels[3], DependencyType::Weak},
        DataReader<custom_types::MotorIoStats>{kPhysIoChannels[4], DependencyType::Weak},
        DataReader<custom_types::MotorIoStats>{kPhysIoChannels[5], DependencyType::Weak},
        DataReader<custom_types::MotorIoStats>{kPhysIoChannels[6], DependencyType::Weak},
    };

    // ── ROS2 조인트 명령 수신값 (GUI 표시용) ──
    // Ros2CmdBridge 가 /joint_command 수신 시 발행하는 내부단위(rad/m) 채널을 구독한다.
    // ros_mode 와 무관하게 "현재 들어오는 ROS 명령"을 GUI 에 그대로 보여주기 위함.
    static constexpr const char* kRos2CmdChannels[kJointCount] = {
        "ros2/joint0/cmd", "ros2/joint1/cmd", "ros2/joint2/cmd",
        "ros2/joint3/cmd", "ros2/joint4/cmd",
    };
    DataReader<custom_types::MotorCmd> dr_ros2_cmd_[kJointCount] = {
        DataReader<custom_types::MotorCmd>{kRos2CmdChannels[0], DependencyType::Weak},
        DataReader<custom_types::MotorCmd>{kRos2CmdChannels[1], DependencyType::Weak},
        DataReader<custom_types::MotorCmd>{kRos2CmdChannels[2], DependencyType::Weak},
        DataReader<custom_types::MotorCmd>{kRos2CmdChannels[3], DependencyType::Weak},
        DataReader<custom_types::MotorCmd>{kRos2CmdChannels[4], DependencyType::Weak},
    };

    // ── 모터 명령 모드 — 물리 모터 직접 명령/on 채널 (CanBus0/1 구독) ──
    DataWriter<custom_types::MotorCmd> dw_phys_cmd_[kPhysCount] = {
        DataWriter<custom_types::MotorCmd>{kPhysCmdChannels[0]},
        DataWriter<custom_types::MotorCmd>{kPhysCmdChannels[1]},
        DataWriter<custom_types::MotorCmd>{kPhysCmdChannels[2]},
        DataWriter<custom_types::MotorCmd>{kPhysCmdChannels[3]},
        DataWriter<custom_types::MotorCmd>{kPhysCmdChannels[4]},
        DataWriter<custom_types::MotorCmd>{kPhysCmdChannels[5]},
        DataWriter<custom_types::MotorCmd>{kPhysCmdChannels[6]},
    };
    DataWriter<bool> dw_phys_on_[kPhysCount] = {
        DataWriter<bool>{kPhysOnChannels[0]},
        DataWriter<bool>{kPhysOnChannels[1]},
        DataWriter<bool>{kPhysOnChannels[2]},
        DataWriter<bool>{kPhysOnChannels[3]},
        DataWriter<bool>{kPhysOnChannels[4]},
        DataWriter<bool>{kPhysOnChannels[5]},
        DataWriter<bool>{kPhysOnChannels[6]},
    };
    // 모터 명령 모드 전역 플래그 — true: CanBus0/1이 조인트 IK를 우회하고 phys_motor cmd를 직접 적용
    DataWriter<bool> dw_motor_raw_mode_{"gui/motor_raw_mode"};
    // ROS 명령 모드 전역 플래그 — true: Manager가 조인트 명령 소스를 GUI 대신 ROS2로 전환
    DataWriter<bool> dw_ros_mode_{"gui/ros_mode"};

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
    std::array<custom_types::MotorIoStats, kPhysCount> phys_io_cache_{};
    std::array<custom_types::MotorState, kJointCount> state_cache_{};
    std::array<custom_types::MotorCmd,   kJointCount> safety_cmd_cache_{};
    std::array<bool,                     kJointCount> safety_cmd_online_{};
    std::array<custom_types::MotorCmd,   kJointCount> applied_cmd_cache_{};
    std::array<bool,                     kJointCount> applied_cmd_online_{};
    std::array<bool,                     kJointCount> joint_online_{};
    std::array<bool,                     kJointCount> motor_power_on_{};
    std::array<custom_types::MotorCmd,   kJointCount> cmd_cache_{};
    std::array<custom_types::MotorCmd,   kPhysCount> phys_cmd_cache_{};
    std::array<bool,                     kPhysCount> phys_power_on_{};
    std::array<custom_types::MotorCmd,   kPhysCount> phys_cmd_applied_cache_{};
    std::array<bool,                     kPhysCount> phys_cmd_applied_online_{};
    std::array<custom_types::MotorCmd,   kJointCount> ros2_cmd_cache_{};      // ROS2 수신값(내부단위)
    std::array<bool,                     kJointCount> ros2_cmd_online_{};     // 조인트별 수신 여부

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
            dr_phys_io_[i].on_update([&, i](const custom_types::MotorIoStats& d) {
                phys_io_cache_[i] = d;
            });
            dr_phys_cmd_applied_[i].on_update([&, i](const custom_types::MotorCmd& d) {
                phys_cmd_applied_cache_[i]  = d;
                phys_cmd_applied_online_[i] = true;
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
            dr_ros2_cmd_[i].on_update([&, i](const custom_types::MotorCmd& d) {
                ros2_cmd_cache_[i]  = d;
                ros2_cmd_online_[i] = true;
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
            if (phys_cmd_applied_online_[i]) {
                const auto& ac = phys_cmd_applied_cache_[i];
                psnap.command_position = ac.pos;
                psnap.command_velocity = ac.vel;
                psnap.command_torque   = ac.torque;
                psnap.command_kp       = ac.kp;
                psnap.command_kd       = ac.kd;
                psnap.kp = ac.kp;
                psnap.kd = ac.kd;
            }
            const auto& io = phys_io_cache_[i];
            psnap.tx_count = io.tx_count;
            psnap.rx_count = io.rx_count;
            psnap.tx_hz    = io.tx_hz;
            psnap.rx_hz    = io.rx_hz;
            st.physical_motors.push_back(std::move(psnap));
        }

        // 데이터 로거 상태
        st.data_logger.recording    = logger_info_.recording;
        st.data_logger.sample_count = logger_info_.sample_count;
        st.data_logger.filename     = logger_info_.filename;

        // ROS2 /joint_command 수신값 — 내부단위(rad/m) → 표시단위(deg/mm) 역변환
        // (revolute 0/1/3: rad→deg, prismatic 2/4: m→mm). 한 조인트라도 수신했으면 online.
        bool ros2_any = false;
        st.ros2_cmd.values.assign(kJointCount, 0.0);
        for (int i = 0; i < kJointCount; i++) {
            if (!ros2_cmd_online_[i]) continue;
            ros2_any = true;
            const bool prismatic = (i == 2 || i == 4);
            const double internal = ros2_cmd_cache_[i].pos;
            st.ros2_cmd.values[i] = prismatic ? (internal * 1000.0)         // m → mm
                                              : (internal * 180.0 / M_PI);  // rad → deg
        }
        st.ros2_cmd.online = ros2_any;

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
            case wsbridge::Event::Kind::PhysMotorPower:
                handle_phys_motor_power(ev.phys_power);
                break;
            case wsbridge::Event::Kind::PhysMotorCommand:
                handle_phys_motor_command(ev.phys_command);
                break;
            case wsbridge::Event::Kind::ViewMode:
                handle_view_mode(ev.view_mode);
                break;
            case wsbridge::Event::Kind::RosMode:
                handle_ros_mode(ev.ros_mode);
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

    // 물리 모터(m01~m07, 1-based) 직접 on/off — 모터 명령 모드 (CanBus0/1이 gui/motor_raw_mode=true일 때만 적용)
    void handle_phys_motor_power(const wsbridge::Event::PhysPowerPayload& pwr) {
        if (!control_granted_) {
            getLogger()->warn("[{}] phys motor power ignored: not granted", getName());
            return;
        }
        auto apply = [&](int idx /*0-based*/) {
            if (idx < 0 || idx >= kPhysCount) return;
            dw_phys_on_[idx].write(pwr.on);
            phys_power_on_[idx] = pwr.on;
        };
        if (pwr.is_all) for (int i = 0; i < kPhysCount; i++) apply(i);
        else apply(pwr.motor_id - 1);
    }

    // 물리 모터(m01~m07, 1-based) 직접 raw 명령 — 모터 명령 모드 (IK/텐션 커플링 우회)
    void handle_phys_motor_command(const wsbridge::Event::PhysCommandPayload& cmd) {
        if (!control_granted_) {
            getLogger()->warn("[{}] phys motor cmd ignored: not granted", getName());
            return;
        }
        auto apply = [&](int idx /*0-based*/) {
            if (idx < 0 || idx >= kPhysCount) return;
            auto& c = phys_cmd_cache_[idx];
            std::snprintf(c.name, sizeof(c.name), "%s", kPhysNames[idx]);
            if (cmd.cmd.position)  c.pos    = *cmd.cmd.position;
            if (cmd.cmd.velocity)  c.vel    = *cmd.cmd.velocity;
            if (cmd.cmd.torque)    c.torque = *cmd.cmd.torque;
            if (cmd.cmd.kp)        c.kp     = *cmd.cmd.kp;
            if (cmd.cmd.kd)        c.kd     = *cmd.cmd.kd;
            const double yaml_dur_ms = p_default_duration_sec.read() * 1000.0;
            c.duration_ms = (yaml_dur_ms >= 0.0) ? yaml_dur_ms : cmd.cmd.duration_ms;
            dw_phys_cmd_[idx].write(c);
        };
        if (cmd.is_all) for (int i = 0; i < kPhysCount; i++) apply(i);
        else apply(cmd.motor_id - 1);
    }

    // GUI 뷰 모드 전환 → CanBus0/1 제어 경로 전환 (joint IK ↔ phys_motor 직접 명령)
    void handle_view_mode(const wsbridge::Event::ViewModePayload& vm) {
        const bool raw = (vm.mode == "motor");
        dw_motor_raw_mode_.write(raw);
        getLogger()->info("[{}] view_mode={} (raw_mode={})", getName(), vm.mode, raw);
    }

    // ROS 명령 모드 토글 → Manager 가 조인트 명령 소스를 GUI ↔ ROS2 로 전환
    void handle_ros_mode(const wsbridge::Event::RosModePayload& rm) {
        dw_ros_mode_.write(rm.enabled);
        getLogger()->info("[{}] ros_mode={}", getName(), rm.enabled);
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
