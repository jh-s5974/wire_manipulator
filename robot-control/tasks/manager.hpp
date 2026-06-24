#pragma once

#include <rtfw/task.h>
#include "util.hpp"
#include "custom_types.hpp"

#include <array>
#include <chrono>
#include <cstring>
#include <string>

using namespace std::chrono_literals;
using namespace rtfw;
using namespace rtfw::rt;

namespace task_pool {

// ─────────────────────────────────────────────────────────────
// 매니퓰레이터 조인트 (0~4)
//   0: base_yaw   (revolute,  RobStride03 0x01, CAN0)
//   1: pitch       (revolute,  RobStride03 0x02, CAN0)
//   2: lower_link  (prismatic, MyActuatorX6 0x03, CAN1)
//   3: elbow_pitch (revolute,  MyActuatorX6 0x06+0x07, CAN1)
//   4: upper_link  (prismatic, MyActuatorX6 0x04+0x05, CAN1)
// ─────────────────────────────────────────────────────────────

static constexpr int N_JOINTS = 5;

enum State { IDLE, MANU };

struct ManagerState {
    State state_      = IDLE;
    State prev_state_ = IDLE;

    bool gui_control_request_event = false;
    bool gui_control_request_value = false;
    bool gui_control_granted       = false;
    int  gui_mode_request          = -1;
    bool gui_mode_request_online   = false;

    std::array<custom_types::MotorCmd, N_JOINTS> gui_motor_cmd{};
    std::array<bool, N_JOINTS>                   gui_motor_cmd_online{};
    std::array<bool, N_JOINTS>                   gui_motor_on{};
    std::array<bool, N_JOINTS>                   gui_motor_on_online{};

    // ROS 명령 소스 — ros_mode=true 일 때 GUI 대신 ros2/jointN/cmd 를 사용한다.
    // (GUI/모터/ROS 명령 소스를 GUI 토글로 전환 — gui/ros_mode 채널)
    bool ros_mode = false;
    std::array<custom_types::MotorCmd, N_JOINTS> ros_motor_cmd{};
    std::array<bool, N_JOINTS>                   ros_motor_cmd_online{};

    struct {
        uint64_t motor[N_JOINTS]{};
    } wdt;

    struct {
        custom_types::MotorState motor[N_JOINTS]{};
        uint64_t tick_ms = 0;
        bool online[N_JOINTS]{};
    } robot_state;
};

class Manager : public Task<ManagerState> {
public:
    const char* getName() const override { return "Manager"; }

    void initialize(ManagerState& s) override {
        s.state_      = IDLE;
        s.prev_state_ = IDLE;
        s.gui_control_granted     = false;
        s.gui_control_request_event = false;
        s.gui_mode_request        = -1;
        s.gui_mode_request_online = false;
        s.gui_motor_cmd_online.fill(false);
        s.gui_motor_on_online.fill(false);
        s.gui_motor_on.fill(true);
        s.ros_mode = false;
        s.ros_motor_cmd_online.fill(false);
        for (int i = 0; i < N_JOINTS; i++) {
            s.wdt.motor[i] = 0;
            s.robot_state.motor[i] = {};
            s.robot_state.online[i] = false;
        }
    }

    void execute(ManagerState& s) override {
        // 모터 상태 수신
        for (int i = 0; i < N_JOINTS; i++) {
            dr_mtr_stat[i].on_update([&, i](const custom_types::MotorState& data) {
                s.wdt.motor[i] = getCurrentTick();
                s.robot_state.motor[i] = data;
            });
        }

        // GUI 요청 수신
        dr_gui_control_request.on_update([&](const bool& req) {
            s.gui_control_request_value = req;
            s.gui_control_request_event = true;
        });

        dr_gui_mode_request.on_update([&](const int& mode) {
            s.gui_mode_request = mode;
            s.gui_mode_request_online = true;
        });

        for (int i = 0; i < N_JOINTS; i++) {
            dr_gui_mtr_cmd[i].on_update([&, i](const custom_types::MotorCmd& data) {
                s.gui_motor_cmd[i]        = data;
                s.gui_motor_cmd_online[i] = true;
            });

            dr_gui_motor_on[i].on_update([&, i](const bool& on) {
                s.gui_motor_on[i]        = on;
                s.gui_motor_on_online[i] = true;

                if (on && s.gui_control_granted) {
                    // 켜진 직후, GUI가 아직 Send하지 않은 상태: kp=0/torque=0 (힘 없음),
                    // kd=3.0(댐핑)만 걸어서 외부 충격에 의한 의도치 않은 흔들림만 막는다.
                    // Send가 오면 dr_gui_mtr_cmd 핸들러가 실제 명령으로 덮어쓴다.
                    custom_types::MotorCmd safe{};
                    safe.pos    = s.robot_state.motor[i].pos;
                    safe.kp     = 0.0;
                    safe.kd     = 3.0;
                    safe.torque = 0.0;
                    dw_mtr_cmd[i].write(safe);
                    // GUI가 아직 이 조인트에 명시적으로 Send한 적이 없으면(gui_motor_cmd_online=false)
                    // 이 safe 값을 시드로 깔아서 process_gui_command_bridge가 매 틱 계속 재전송하게 한다.
                    // 그렇지 않으면 켜진 직후 1회만 전송되고 끊겨, 1초 뒤 SafetyLayer가
                    // "manager cmd timeout"으로 LOCK을 거는 문제가 발생한다.
                    if (!s.gui_motor_cmd_online[i]) {
                        s.gui_motor_cmd[i]        = safe;
                        s.gui_motor_cmd_online[i] = true;
                    }
                    // 전원 ON 직후엔 직전에 받아둔(오래된) ROS 명령으로 갑자기 튀지 않도록
                    // ROS 소스를 잠시 비활성화한다. 새 ROS 프레임이 오면 다시 ROS가 구동한다.
                    s.ros_motor_cmd_online[i] = false;
                }
            });
        }

        // ROS 모드 토글 + ROS 조인트 명령 수신 (ros_mode=true 일 때 GUI 대신 사용)
        dr_ros_mode.on_update([&](const bool& on) {
            if (on != s.ros_mode)
                getLogger()->info("[{}] ROS command mode {}", getName(), on ? "ENABLED" : "DISABLED");
            s.ros_mode = on;
        });
        for (int i = 0; i < N_JOINTS; i++) {
            dr_ros_cmd[i].on_update([&, i](const custom_types::MotorCmd& data) {
                s.ros_motor_cmd[i]        = data;
                s.ros_motor_cmd_online[i] = true;
            });
        }

        wdt_process(s);
        process_gui_command_bridge(s);
        process_gui_mode_request(s);

        switch (s.state_) {
            case IDLE: task_idle(s); break;
            case MANU: task_manu(s); break;
        }

        dw_gui_mode_current.write(state_to_gui_mode(s.state_));
        dw_safety_level.write(0); // 매니퓰레이터는 항상 ESSENTIAL

        s.robot_state.tick_ms = getExecutionLocalTick() * 1000 / getFrequency();

        PERIODIC_CALL(
            getLogger()->info("[{}] State={}", getName(), state_to_string(s.state_));
        , 5s);

        // [DIAG] 임시 진단 — ROS 소스 전환 상태. 원인 확인 후 제거.
        //   ros_mode    : 토글이 Manager까지 도달했는지 (gui/ros_mode 채널)
        //   ros_online  : ROS 브리지 명령이 도착한 조인트 수 (ros2/jointN/cmd 채널)
        //   gui_online  : GUI 명령이 도착한 조인트 수
        //   granted     : 제어 권한
        //   src         : 현재 실제로 선택되는 명령 소스
        PERIODIC_CALL(
            int ros_on = 0;
            int gui_on = 0;
            for (int i = 0; i < N_JOINTS; i++) {
                if (s.ros_motor_cmd_online[i]) ros_on++;
                if (s.gui_motor_cmd_online[i]) gui_on++;
            }
            getLogger()->info("[{}] DIAG ros_mode={} ros_online={}/{} gui_online={}/{} granted={} src={}",
                              getName(), s.ros_mode, ros_on, N_JOINTS, gui_on, N_JOINTS,
                              s.gui_control_granted, (s.ros_mode && ros_on > 0) ? "ROS" : "GUI");
        , 1s);
    }

private:
    // ── 상태 천이 헬퍼 ──
    void state_commit(ManagerState& s, State ns) {
        s.prev_state_ = s.state_;
        s.state_      = ns;
    }

    // IDLE: 모든 조인트가 온라인이 되면 MANU로 전환
    void task_idle(ManagerState& s) {
        bool all_online = true;
        for (int i = 0; i < N_JOINTS; i++) {
            if (!s.robot_state.online[i]) {
                PERIODIC_CALL(
                    getLogger()->warn("[{}] Joint {} offline", getName(), i);
                , 1s);
                all_online = false;
            }
        }
        if (all_online) {
            getLogger()->info("[{}] All joints online. → MANU", getName());
            state_commit(s, MANU);
        } else {
            PERIODIC_CALL(
                getLogger()->info("[{}] Waiting for joints...", getName());
            , 2s);
        }
    }

    // MANU: GUI 명령을 통한 수동 제어 대기
    void task_manu(ManagerState& s) {
        // 정상 동작 — GUI 명령 브리지에서 이미 명령 전달
        (void)s;
    }

    // ── GUI 명령 브리지 ──
    void process_gui_command_bridge(ManagerState& s) {
        // MANU 상태를 벗어나면 control 권한 해제
        if (s.gui_control_granted && s.state_ != MANU) {
            s.gui_control_granted = false;
            getLogger()->info("[{}] GUI control REVOKED", getName());
        }

        if (s.gui_control_request_event) {
            s.gui_control_request_event = false;
            if (s.gui_control_request_value) {
                if (s.state_ == MANU) {
                    s.gui_control_granted = true;
                    getLogger()->info("[{}] GUI control GRANTED", getName());
                } else {
                    getLogger()->warn("[{}] GUI control rejected (not MANU)", getName());
                }
            } else {
                s.gui_control_granted = false;
                getLogger()->info("[{}] GUI control RELEASED", getName());
            }
        }

        dw_gui_control_requested.write(s.gui_control_request_value);
        dw_gui_control_granted.write(s.gui_control_granted);

        if (!s.gui_control_granted) return;

        for (int i = 0; i < N_JOINTS; i++) {
            // 명령 소스 선택:
            //   - ROS 모드이고 해당 조인트의 ROS 명령이 도착했으면 ROS 명령을 사용.
            //   - 그 외(GUI 모드, 또는 ROS 모드지만 아직 ROS 미수신)에는 GUI 명령
            //     (전원 ON 시 깔아둔 safe-hold 시드 포함)을 사용.
            // ROS 미수신 구간에도 매 틱 명령이 전달되어 SafetyLayer "manager cmd
            // timeout"(LOCK)을 방지하고, 모터는 그 자리에서 안전하게 홀드한다.
            if (s.ros_mode && s.ros_motor_cmd_online[i])
                dw_mtr_cmd[i].write(s.ros_motor_cmd[i]);
            else if (s.gui_motor_cmd_online[i])
                dw_mtr_cmd[i].write(s.gui_motor_cmd[i]);

            if (s.gui_motor_on_online[i])
                dw_motor_on[i].write(s.gui_motor_on[i]);
        }
    }

    // ── GUI 모드 요청 (IDLE/MANU만 허용) ──
    void process_gui_mode_request(ManagerState& s) {
        if (!s.gui_mode_request_online) return;
        s.gui_mode_request_online = false;

        switch (s.gui_mode_request) {
            case 0: // IDLE
                state_commit(s, IDLE);
                getLogger()->info("[{}] GUI mode → IDLE", getName());
                break;
            case 1: // MANU
                state_commit(s, MANU);
                getLogger()->info("[{}] GUI mode → MANU", getName());
                break;
            default:
                getLogger()->warn("[{}] GUI mode {} unsupported for manipulator", getName(), s.gui_mode_request);
                break;
        }
    }

    void wdt_process(ManagerState& s) {
        auto now = getCurrentTick();
        for (int i = 0; i < N_JOINTS; i++)
            s.robot_state.online[i] = (now - s.wdt.motor[i]) <= 1 * getFrequency();
    }

    static int  state_to_gui_mode(State st)       { return st == MANU ? 1 : 0; }
    static const char* state_to_string(State st)   { return st == MANU ? "MANU" : "IDLE"; }

    // ── 데이터 채널 ──
    DataReader<custom_types::MotorState> dr_mtr_stat[N_JOINTS] = {
        DataReader<custom_types::MotorState>{"joint0/state", DependencyType::Weak},
        DataReader<custom_types::MotorState>{"joint1/state", DependencyType::Weak},
        DataReader<custom_types::MotorState>{"joint2/state", DependencyType::Weak},
        DataReader<custom_types::MotorState>{"joint3/state", DependencyType::Weak},
        DataReader<custom_types::MotorState>{"joint4/state", DependencyType::Weak},
    };

    DataReader<custom_types::MotorCmd> dr_gui_mtr_cmd[N_JOINTS] = {
        DataReader<custom_types::MotorCmd>{"gui/joint0/cmd", DependencyType::Weak},
        DataReader<custom_types::MotorCmd>{"gui/joint1/cmd", DependencyType::Weak},
        DataReader<custom_types::MotorCmd>{"gui/joint2/cmd", DependencyType::Weak},
        DataReader<custom_types::MotorCmd>{"gui/joint3/cmd", DependencyType::Weak},
        DataReader<custom_types::MotorCmd>{"gui/joint4/cmd", DependencyType::Weak},
    };
    DataReader<bool> dr_gui_motor_on[N_JOINTS] = {
        DataReader<bool>{"gui/joint0/on", DependencyType::Weak},
        DataReader<bool>{"gui/joint1/on", DependencyType::Weak},
        DataReader<bool>{"gui/joint2/on", DependencyType::Weak},
        DataReader<bool>{"gui/joint3/on", DependencyType::Weak},
        DataReader<bool>{"gui/joint4/on", DependencyType::Weak},
    };

    // ROS 명령 소스 — GUI 토글(gui/ros_mode)과 ROS 브리지가 발행하는 조인트 명령
    DataReader<bool> dr_ros_mode{"gui/ros_mode", DependencyType::Weak};
    DataReader<custom_types::MotorCmd> dr_ros_cmd[N_JOINTS] = {
        DataReader<custom_types::MotorCmd>{"ros2/joint0/cmd", DependencyType::Weak},
        DataReader<custom_types::MotorCmd>{"ros2/joint1/cmd", DependencyType::Weak},
        DataReader<custom_types::MotorCmd>{"ros2/joint2/cmd", DependencyType::Weak},
        DataReader<custom_types::MotorCmd>{"ros2/joint3/cmd", DependencyType::Weak},
        DataReader<custom_types::MotorCmd>{"ros2/joint4/cmd", DependencyType::Weak},
    };

    DataWriter<custom_types::MotorCmd> dw_mtr_cmd[N_JOINTS] = {
        DataWriter<custom_types::MotorCmd>{"manager/joint0/cmd"},
        DataWriter<custom_types::MotorCmd>{"manager/joint1/cmd"},
        DataWriter<custom_types::MotorCmd>{"manager/joint2/cmd"},
        DataWriter<custom_types::MotorCmd>{"manager/joint3/cmd"},
        DataWriter<custom_types::MotorCmd>{"manager/joint4/cmd"},
    };
    DataWriter<bool> dw_motor_on[N_JOINTS] = {
        DataWriter<bool>{"joint0/on"},
        DataWriter<bool>{"joint1/on"},
        DataWriter<bool>{"joint2/on"},
        DataWriter<bool>{"joint3/on"},
        DataWriter<bool>{"joint4/on"},
    };

    DataReader<bool> dr_gui_control_request{"gui/motor/control_request", DependencyType::Weak};
    DataReader<int>  dr_gui_mode_request{"gui/robot/mode_request", DependencyType::Weak};

    DataWriter<bool> dw_gui_control_requested{"gui/motor/control_requested"};
    DataWriter<bool> dw_gui_control_granted{"gui/motor/control_granted"};
    DataWriter<int>  dw_gui_mode_current{"gui/robot/mode_current"};
    DataWriter<int>  dw_safety_level{"manager/safety_level"};
};

} // namespace task_pool
