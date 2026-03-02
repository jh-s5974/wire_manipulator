// ros2_decoder.h (수정 완료)

#pragma once
#include "rtfw_connect/iclient.h"
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosidl_runtime_cpp/traits.hpp>
#include <cstring> // std::memcpy를 사용하기 위해 추가

namespace rtfw::connect::ros2 {

// T: ROS2 메시지 타입 (e.g., sensor_msgs::msg::Image)
template <typename T>
struct Ros2Decoder {
  static std::string schema() {
    // "pkg/msg/Type"
    return std::string(rosidl_generator_traits::name<T>());
  }

  bool operator()(const ::rtfw::connect::Sample& s, T& out) const {
    if (!s.bytes || s.bytes->empty()) return false;

    // [수정] std::vector로부터 rclcpp::SerializedMessage를 직접 생성할 수 없습니다.
    // 대신, 필요한 크기의 SerializedMessage를 생성하고 내용을 복사합니다.
    const auto& vec = *s.bytes;

    // 1. 벡터 크기만큼의 버퍼를 가진 SerializedMessage 객체 생성
    rclcpp::SerializedMessage sm(vec.size());

    // 2. 벡터의 데이터를 SerializedMessage의 내부 버퍼로 복사
    auto& rcl_sm = sm.get_rcl_serialized_message();
    std::memcpy(rcl_sm.buffer, vec.data(), vec.size());
    rcl_sm.buffer_length = vec.size(); // 실제 데이터 길이 설정

    // 3. 이제 유효한 SerializedMessage 객체로 역직렬화 수행
    rclcpp::Serialization<T> ser;
    ser.deserialize_message(&sm, &out);
    return true;
  }
};

} // namespace rtfw::connect::ros2