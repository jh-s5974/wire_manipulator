// examples/remote_minimal.cpp
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "rtfw_connect_ros2/remote_client.h"

// 새로운 아키텍처에서 필요한 using 선언들
using rtfw::connect::HandleOptions;
using rtfw::connect::ros2::RemoteClient;

int main(int argc, char** argv){
  // 1. ROS 2 노드 초기화
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("rtfw_remote_minimal");

  // 2. RemoteClient 생성 및 연결
  // create는 RemoteClient의 구체적인 shared_ptr를 반환합니다.
  auto rc = RemoteClient::create(node);
  if (!rc->connect()) {
    RCLCPP_WARN(node->get_logger(), "topic_list not received yet; continuing anyway");
  }

  // 3. 사용 가능한 토픽 목록 확인 (선택 사항)
  auto topics = rc->list_topics();
  RCLCPP_INFO(node->get_logger(), "Available topics (%zu):", topics.size());
  for (const auto& t : topics) {
    RCLCPP_INFO(node->get_logger(), "  %s", t.c_str());
  }

  // 4. 데이터 핸들 요청
  // get_handle<T>는 이제 std::shared_ptr<THandle<T>>를 반환합니다.
  // 이 shared_ptr의 생명주기가 토픽의 "활성(active)" 상태를 관리합니다.

  // 가벼운 데이터: 기본 옵션(Persistent) 사용.
  // 핸들(h_odom, h_batt)이 살아있는 동안 브릿지에 계속 활성 상태로 알려집니다.
  auto h_odom = rc->get_handle<nav_msgs::msg::Odometry>("state/odom");
  auto h_batt = rc->get_handle<std_msgs::msg::Float32>("diag/battery");

  // 무거운 데이터(영상): 'Strict' 정책으로 지정.
  // 이 정책은 마지막 핸들(h_img)이 사라졌을 때, 내부 ROS 2 구독까지 완전히 해제하여
  // 불필요한 네트워크 트래픽 수신을 원천 차단합니다.
  HandleOptions img_opt;
  img_opt.sub_policy = HandleOptions::SubPolicy::Strict;
  auto h_img  = rc->get_handle<sensor_msgs::msg::Image>("camera/front/image_raw", img_opt);

  // 5. On-demand 활성화
  // 현재 살아있는 핸들(h_odom, h_batt, h_img)에 해당하는 토픽들만
  // 구독하겠다고 브릿지에 알립니다.
  rc->send_heartbeat();

  // (선택) 1.5초마다 주기적으로 하트비트를 보내 활성 상태를 유지합니다.
  rc->set_keepalive_period(std::chrono::milliseconds(1500));

  // 6. ROS 2 Executor 설정 및 실행
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);

  // 7. 메인 루프: 핸들을 이용한 데이터 폴링
  auto next_print = std::chrono::steady_clock::now();
  while (rclcpp::ok()) {
    nav_msgs::msg::Odometry od;
    // 핸들이 shared_ptr이므로 '->' 연산자로 멤버 함수에 접근합니다.
    if (h_odom && h_odom->read_latest(od)) {
      // od.pose.pose.position.x 와 같이 데이터 사용
    }

    std_msgs::msg::Float32 batt;
    if (h_batt && h_batt->read_latest(batt)) {
      // batt.data 와 같이 데이터 사용
    }
    
    sensor_msgs::msg::Image img;
    if (h_img && h_img->read_latest(img)) {
      // img.width, img.height 등 데이터 사용
    }

    // 1초마다 수신 상태를 로그로 출력
    if (std::chrono::steady_clock::now() >= next_print) {
      // 핸들 포인터가 유효한지 먼저 확인 후 read_latest 호출
      RCLCPP_INFO(node->get_logger(), "tick: odom:%s batt:%s img:%s",
                  (h_odom && h_odom->read_latest(od)) ? "Y" : "N",
                  (h_batt && h_batt->read_latest(batt)) ? "Y" : "N",
                  (h_img && h_img->read_latest(img)) ? "Y" : "N");
      next_print += std::chrono::seconds(1);
    }

    exec.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  // 8. 종료 처리
  rc->shutdown();
  rclcpp::shutdown();
  return 0;
}