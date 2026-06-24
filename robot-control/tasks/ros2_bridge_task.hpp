#pragma once

// ═══════════════════════════════════════════════════════════════════════════
// ROS2 조인트 명령 브리지 (Non-RT 태스크)
// ═══════════════════════════════════════════════════════════════════════════
//
// 역할: ROS2 토픽 "/joint_command"(std_msgs/Float64MultiArray, 5개)로 들어온 조인트
//       목표값을 내부 단위로 변환해 "ros2/jointN/cmd" 채널로 발행한다. Manager 가
//       ROS 모드(gui/ros_mode=true)일 때 이 채널을 GUI 명령 대신 사용한다.
//       (GUI/모터/ROS 세 가지 명령 소스를 토글로 전환 — manager.hpp 참조)
//
// 메시지 규약 (조인트 순서 · 입력 단위):
//   data[0] joint0 base_yaw    [deg]
//   data[1] joint1 pitch       [deg]
//   data[2] joint2 lower_link  [mm]
//   data[3] joint3 elbow_pitch [deg]
//   data[4] joint4 upper_link  [mm]
//   → 내부 단위로 변환: revolute(0,1,3) deg→rad, prismatic(2,4) mm→m.
//
// kp/kd: 메시지에는 위치만 담기므로 config/robotnl.yaml 의 joint_kp/joint_kd 를
//   게인으로 사용한다(GUI 기본 게인과 동일). 속도는 0으로 두면 드라이버가
//   joint_default_speed 로 대체한다(joint2/3/4).
//
// 주의: 실제 모터가 움직이려면 GUI 에서 ① MANU 모드 ② 제어 권한(grant) ③ 모터 ON +
//   ④ ROS 모드 토글 ON 이어야 한다. ROS 브리지는 위치 setpoint 만 흘려보낸다.

#include "rtfw/task.h"
#include "custom_types.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <array>
#include <cmath>
#include <cstdio>
#include <utility>
#include <vector>

using namespace rtfw::rt;

namespace task_pool {

class Ros2CmdBridge : public ITask {
public:
    explicit Ros2CmdBridge(rclcpp::Node::SharedPtr node) : node_(std::move(node)) {}

    const char* getName() const override { return "Ros2CmdBridge"; }

    void initialize(void*) override {
        // 5개 조인트 목표값 구독. 콜백은 execute()의 spin_some() 안에서 동기 실행되므로
        // (별도 스레드 없음) latest_/have_msg_ 접근에 별도 락이 필요 없다.
        sub_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/joint_command", rclcpp::QoS(10),
            [this](std_msgs::msg::Float64MultiArray::SharedPtr msg) {
                if (static_cast<int>(msg->data.size()) < kJointCount) {
                    bad_size_ = static_cast<int>(msg->data.size());
                    return;
                }
                for (int i = 0; i < kJointCount; i++) latest_[i] = msg->data[i];
                have_msg_ = true;
                rx_count_++;   // [DIAG] 구독 콜백 수신 누계
            });

        // executor 에 node 를 한 번만 등록한다. 매 틱 임시 executor 를 새로 만들면
        // (이전 rclcpp::spin_some(node) 방식) 풀 스레드에서 wait_set/엔티티 수집 타이밍
        // 문제로 구독이 서비스되지 않아 콜백이 한 번도 안 불리는 현상이 있었다.
        executor_.add_node(node_);
    }

    void execute(void*) override {
        // ROS2 이벤트 처리 (이 호출 안에서 위 구독 콜백이 동기 실행됨).
        // 영속 executor 를 재사용 — node 는 initialize()에서 한 번만 add 했다.
        executor_.spin_some();

        // [DIAG] 임시 진단 — 콜백 수신(rx)/채널 발행(writes) 누계. 원인 확인 후 제거.
        PERIODIC_CALL(
            getLogger()->info("[{}] DIAG rx={} writes={} have_msg={}",
                              getName(), rx_count_, write_count_, have_msg_);
        , std::chrono::seconds(1));

        // 잘못된 길이의 메시지가 들어왔으면 주기적으로 한 번만 경고
        if (bad_size_ >= 0) {
            PERIODIC_CALL(
                getLogger()->warn("[{}] /joint_command 길이 {} (필요 {}) — 무시됨",
                                  getName(), bad_size_, kJointCount);
            , std::chrono::seconds(1));
            bad_size_ = -1;
        }

        if (!have_msg_) return;
        have_msg_ = false;

        const auto& kp = p_joint_kp.read();
        const auto& kd = p_joint_kd.read();
        // ROS 메시지엔 duration이 없으므로 yaml 보간시간으로 채운다(GUI와 동일하게 부드럽게 이동).
        // 0 이하면 즉시 이동 → 큰 위치차에서 토크 폭주/LOCK 위험하므로 음수는 0으로 클램프.
        const double duration_ms = std::max(0.0, p_default_duration_sec.read() * 1000.0);

        for (int i = 0; i < kJointCount; i++) {
            custom_types::MotorCmd c{};
            std::snprintf(c.name, sizeof(c.name), "ros2_joint%d", i);
            c.pos    = to_internal(i, latest_[i]);                 // deg→rad or mm→m
            c.vel    = 0.0;                                        // 0 → 드라이버가 default_speed 대체
            c.torque = 0.0;
            c.kp     = (static_cast<int>(kp.size()) > i) ? kp[i] : 0.0;
            c.kd     = (static_cast<int>(kd.size()) > i) ? kd[i] : 0.0;
            c.duration_ms = duration_ms;
            dw_ros2_cmd_[i].write(c);
        }
        write_count_++;   // [DIAG] ros2/jointN/cmd 발행 틱 누계
    }

private:
    static constexpr int kJointCount = 5;

    // 조인트별 입력 단위 → 내부 단위:
    //   revolute(joint0/1/3): deg → rad,  prismatic(joint2/4): mm → m
    static double to_internal(int i, double v) {
        const bool prismatic = (i == 2 || i == 4);
        return prismatic ? (v * 0.001) : (v * M_PI / 180.0);
    }

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_;
    // 영속 executor — node 를 한 번만 add 하고 매 틱 spin_some 으로 콜백을 서비스한다.
    rclcpp::executors::SingleThreadedExecutor executor_;

    std::array<double, kJointCount> latest_{}; // 최근 수신한 5개 목표값(입력 단위)
    bool have_msg_ = false;                     // 새 메시지 도착 플래그
    int  bad_size_ = -1;                        // 잘못된 길이 메시지(>=0이면 경고 대기)
    long rx_count_    = 0;                       // [DIAG] 구독 콜백 수신 누계
    long write_count_ = 0;                       // [DIAG] ros2/jointN/cmd 발행 틱 누계

    // 변환 결과를 발행하는 내부 명령 채널 — Manager 가 ROS 모드에서 구독
    DataWriter<custom_types::MotorCmd> dw_ros2_cmd_[kJointCount] = {
        DataWriter<custom_types::MotorCmd>{"ros2/joint0/cmd"},
        DataWriter<custom_types::MotorCmd>{"ros2/joint1/cmd"},
        DataWriter<custom_types::MotorCmd>{"ros2/joint2/cmd"},
        DataWriter<custom_types::MotorCmd>{"ros2/joint3/cmd"},
        DataWriter<custom_types::MotorCmd>{"ros2/joint4/cmd"},
    };

    // 게인/보간 파라미터 — config/robotnl.yaml
    Parameter<std::vector<double>> p_joint_kp{"joint_kp"};
    Parameter<std::vector<double>> p_joint_kd{"joint_kd"};
    // ROS 전용 보간시간(기본 1.0s, GUI 기본값과 동일). GUI 의 ws_bridge.* 와 별개.
    Parameter<double> p_default_duration_sec{"ros2_bridge.default_duration_sec", 1.0};
};

} // namespace task_pool
