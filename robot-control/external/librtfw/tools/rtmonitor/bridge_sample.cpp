
#include "rtfw_common/shm_layout.h"

#include <std_msgs/msg/float32_multi_array.hpp>
#include <rclcpp/rclcpp.hpp>

namespace rtfw::connect::ros2 {

    std_msgs::msg::Float32MultiArray convert(const std::array<double, 4>& src) {
        std_msgs::msg::Float32MultiArray dst;
        for (auto i=0; i<src.size(); i++)
            dst.data.push_back(src[i]);
        return dst;
    };
};
#include "rtfw_connect_ros2/bridge.h"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto bridge = rtfw::connect::ros2::Bridge("rtfw/bridge", rtfw::common::SHM_NAME);
    bridge.add_publisher<std::array<double, 4>>("velocity_sv", "velocity/sv");
    // ...
    // velocity
    // wheel
    // target pose
    // body pose
    // left foot pose
    // right foot pose
    // odometry
    // imu
    // camera preview
    // crane
    // 
    // parameter
    // ...
    bridge.run(30);
    rclcpp::shutdown();
    return 0;
}