#pragma once

#include "rtfw/task.h"
#include "biped_data_types.h"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

using namespace rtfw::rt;

// ROS2 Bridge Task - exposes RTFW data to ROS2 topics
class RosBridgeTask : public ITask {
public:
    RosBridgeTask(rclcpp::Node::SharedPtr ros_node) : ros_node_(ros_node) {}
    
    const char* getName() const override { return "RosBridgeTask"; }
        
    void initialize(void*) override {
        // Create publishers for RTFW -> ROS2
        sensors_pub_ = ros_node_->create_publisher<std_msgs::msg::Float64MultiArray>("/sensors/high_freq", 10);
        state_pub_ = ros_node_->create_publisher<std_msgs::msg::Float64MultiArray>("/robot/state", 10);
        torque_pub_ = ros_node_->create_publisher<std_msgs::msg::Float64MultiArray>("/motor/torques", 10);
        gait_pub_ = ros_node_->create_publisher<std_msgs::msg::Float64MultiArray>("/robot/gait", 10);
        vision_pub_ = ros_node_->create_publisher<std_msgs::msg::Float64MultiArray>("/vision/object", 10);
    }
    
    void execute(void*) override {
        // Keep ROS2 event loop responsive
        rclcpp::spin_some(ros_node_);
        
        // Publish sensor data
        sensors_reader_.on_update([this](const HighFreqSensors& data) {
            auto msg = std::make_unique<std_msgs::msg::Float64MultiArray>();
            msg->data.clear();
            for (int i = 0; i < 12; ++i) {
                msg->data.push_back(data.joint_positions[i]);
            }
            for (int i = 0; i < 12; ++i) {
                msg->data.push_back(data.joint_velocities[i]);
            }
            sensors_pub_->publish(std::move(msg));
        });
        
        // Publish robot state
        state_reader_.on_update([this](const RobotState& data) {
            auto msg = std::make_unique<std_msgs::msg::Float64MultiArray>();
            msg->data.clear();
            for (int i = 0; i < 3; ++i) {
                msg->data.push_back(data.center_of_mass[i]);
            }
            for (int i = 0; i < 2; ++i) {
                msg->data.push_back(data.zero_moment_point[i]);
            }
            msg->data.push_back(data.is_stable ? 1.0 : 0.0);
            state_pub_->publish(std::move(msg));
        });
        
        // Publish torques
        torque_reader_.on_update([this](const MotorTorques& data) {
            auto msg = std::make_unique<std_msgs::msg::Float64MultiArray>();
            msg->data.clear();
            for (int i = 0; i < 12; ++i) {
                msg->data.push_back(data.joint_torques[i]);
            }
            torque_pub_->publish(std::move(msg));
        });
        
        // Publish gait
        gait_reader_.on_update([this](const WalkingGait& data) {
            auto msg = std::make_unique<std_msgs::msg::Float64MultiArray>();
            msg->data.clear();
            msg->data.push_back(data.step_length);
            msg->data.push_back(data.step_height);
            msg->data.push_back(data.step_frequency);
            gait_pub_->publish(std::move(msg));
        });
        
        // Publish vision data
        vision_reader_.on_update([this](const VisionObject& data) {
            auto msg = std::make_unique<std_msgs::msg::Float64MultiArray>();
            msg->data.clear();
            msg->data.push_back(data.id);
            for (int i = 0; i < 3; ++i) {
                msg->data.push_back(data.position_in_world[i]);
            }
            vision_pub_->publish(std::move(msg));
        });
    }
    
private:
    rclcpp::Node::SharedPtr ros_node_;
    
    DataReader<HighFreqSensors> sensors_reader_{"sensors.high_freq", DependencyType::Weak};
    DataReader<RobotState> state_reader_{"robot.state.estimated", DependencyType::Weak};
    DataReader<MotorTorques> torque_reader_{"motor.torques.target", DependencyType::Weak};
    DataReader<WalkingGait> gait_reader_{"robot.gait.planned", DependencyType::Weak};
    DataReader<VisionObject> vision_reader_{"vision.object.detected", DependencyType::Weak};
    
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr sensors_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr state_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr torque_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr gait_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr vision_pub_;
};
