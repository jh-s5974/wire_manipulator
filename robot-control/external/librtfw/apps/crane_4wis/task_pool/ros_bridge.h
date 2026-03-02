#pragma once

#include <rtfw/task.h>
#include <manif/manif.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>

#include <atomic>
#include <mutex> // fallback용

using namespace rtfw::rt;



namespace ros2 {
    geometry_msgs::msg::Twist convert(const manif::SE3Tangentd& src) {
        geometry_msgs::msg::Twist dst;
        dst.linear.x = src.lin().x();
        dst.linear.y = src.lin().y();
        dst.linear.z = src.lin().z();
        dst.angular.x = src.ang().x();
        dst.angular.y = src.ang().y();
        dst.angular.z = src.ang().z();
        return dst;
    };

    std_msgs::msg::Float32MultiArray convert(const std::array<double, 4>& src) {
        std_msgs::msg::Float32MultiArray dst;
        dst.data.clear();
        for (auto& val: src)
            dst.data.push_back(val);
            return dst;
    };

    geometry_msgs::msg::Pose2D convert(const manif::SE2d& src) {
        geometry_msgs::msg::Pose2D dst;
        dst.x = src.x();
        dst.y = src.y();
        dst.theta = src.angle();
        return dst;
    };

    geometry_msgs::msg::Pose convert(const manif::SE3d& src) {
        geometry_msgs::msg::Pose dst;
        dst.position.x = src.translation().x();
        dst.position.y = src.translation().y();
        dst.position.z = src.translation().z();
        dst.orientation.w = src.quat().w();
        dst.orientation.x = src.quat().x();
        dst.orientation.y = src.quat().y();
        dst.orientation.z = src.quat().z();
        return dst;
    };


    std::array<double, 4> convert(const std_msgs::msg::Float32MultiArray& src) {

        std::array<double, 4> dst;
        for (auto i=0; i<std::min(dst.size(), src.data.size()); i++)
            dst[i] = src.data[i];
        return dst;
    };
}


namespace ros2 {
    class BridgeHelper {
    private:
        // 각 연결 정보를 저장하는 내부 구조체
        struct PublisherMapping {
            std::function<void()> trigger_func;
        };
        struct SubscriptionMapping {
            std::function<void()> process_func; // execute()에서 호출될 함수
        };
        std::vector<PublisherMapping> publisher_mappings_;
        std::vector<SubscriptionMapping> subscription_mappings_;

        rclcpp::Node& node_; // 외부에서 생성된 노드를 참조
        std::vector<rclcpp::PublisherBase::SharedPtr> publishers_;
        std::vector<rclcpp::SubscriptionBase::SharedPtr> subscribers_;

        template<typename T>
        class SharedState {
        private:
            // 더블 버퍼
            std::array<T, 2> buffers_; 
            // 현재 쓰기 중인 버퍼의 인덱스 (ROS 콜백 스레드만 수정)
            std::atomic<int> write_index_{0};
            // 가장 최근에 쓰기가 완료된 데이터가 있는지 여부 (새 데이터 플래그)
            std::atomic<bool> new_data_available_{false};
            
            // 쓰기 접근을 위한 간단한 스핀락 (optional, std::mutex보다 가벼움)
            std::atomic_flag spinlock_ = ATOMIC_FLAG_INIT;

        public:
            // ROS 콜백 스레드에서 호출 (Non-RT)
            void write(T&& data) {
                // 간단한 스핀락으로 동시 쓰기 방지
                while (spinlock_.test_and_set(std::memory_order_acquire)) {} // lock
                
                int current_write_idx = write_index_.load(std::memory_order_relaxed);
                buffers_[current_write_idx] = std::move(data);
                new_data_available_.store(true, std::memory_order_release); // 쓰기 완료 후 플래그 설정
                
                spinlock_.clear(std::memory_order_release); // unlock
            }

            // RTFW의 execute() 스레드에서 호출 (RT/Non-RT)
            std::optional<T> read_latest() {
                // 새 데이터가 없다면 아무것도 하지 않음
                if (!new_data_available_.load(std::memory_order_acquire)) {
                    return std::nullopt;
                }

                while (spinlock_.test_and_set(std::memory_order_acquire)) {} // lock
                
                // 새 데이터 플래그를 내리고, 현재 쓰기 인덱스를 읽기 인덱스로 스왑
                new_data_available_.store(false, std::memory_order_relaxed);
                int read_idx = write_index_.load(std::memory_order_relaxed);
                // 다음 쓰기는 다른 버퍼에 하도록 인덱스 토글
                write_index_.store(1 - read_idx, std::memory_order_relaxed);
                
                spinlock_.clear(std::memory_order_release); // unlock

                // 방금 읽기로 전환된 버퍼의 데이터를 "이동"하여 반환
                // 이동 시키면 버퍼의 내용은 더 이상 유효하지 않지만, 다음 쓰기가 덮어쓸 것이므로 괜찮음.
                return std::move(buffers_[read_idx]);
            }
        };

    public:
        BridgeHelper(rclcpp::Node& node) : node_(node) {}

        template<typename RtfwType>
        void expose_interface(rtfw::rt::DataReader<RtfwType>& reader, const std::string& ros_topic) {
            using ros2::convert;
            using RosType = std::decay_t<
                decltype(convert(std::declval<const RtfwType&>()))
            >;

            static_assert(!std::is_void<RosType>::value,
                "RTFW_BRIDGE_ERROR: convert(const RtfwType&) 오버로드가 필요합니다.");
            
            // 이제 추론된 RosType을 사용하여 Publisher를 생성
            auto publisher = node_.create_publisher<RosType>(ros_topic, 10);
            publishers_.push_back(publisher);

            auto trigger_action = [reader, publisher]() mutable {
                reader.on_update([&](const RtfwType& data) {
                    auto msg = std::make_unique<RosType>(convert(data));                    
                    publisher->publish(std::move(msg));
                });
            };
            
            publisher_mappings_.push_back({trigger_action});
        }


        template<typename RosType, typename RtfwType>
        void expose_interface(rtfw::rt::DataWriter<RtfwType>& writer, const std::string& ros_topic) {
            using ros2::convert;
            auto shared_state = std::make_shared<SharedState<RtfwType>>();

            // Subscriber 콜백은 이제 SharedState에 최신 데이터를 덮어쓰기만 함
            auto callback = [shared_state](const typename RosType::SharedPtr msg) {
                shared_state->write(convert(*msg)); // Lock-Free 쓰기
            };

            auto subscriber = node_.create_subscription<RosType>(ros_topic, 1, callback);
            subscribers_.push_back(subscriber);

            // execute()에서 호출될 처리 함수
            auto process_action = [writer, shared_state]() mutable {
                // 가장 마지막에 들어온 데이터가 있다면 읽어서 처리
                if (auto latest_data = shared_state->read_latest()) {
                    writer.write(*latest_data);
                }
            };
            
            subscription_mappings_.push_back({process_action});
        }

        void execute_publishers() {
            for (auto& mapping : publisher_mappings_) {
                mapping.trigger_func();
            }
        }

        void execute_subscriptions() {
            for (auto& mapping : subscription_mappings_) {
                mapping.process_func();
            }
        }
    };
};





namespace task_pool {

    class RosBridge : public ITask {
    public:
        ~RosBridge() {
            helper_ = nullptr;
            _node = nullptr;
            rclcpp::shutdown();
        }
        const char* getName() const override { return "ros_bridge"; }
        void setup(TaskRegistry& r) override {
            r.add_dependency(dr_velocity_sv);
            r.add_dependency(dr_velocity_pv);
            r.add_dependency(dr_wheel_sv);
            r.add_dependency(dr_wheel_pv);
            r.add_dependency(dr_error);
            r.add_dependency(dr_vel_target);
            r.add_dependency(dr_odom);
            r.add_dependency(dr_pose_target);
            r.add_dependency(dr_local_target);
            r.add_dependency(dw_wheel_sv);
        }
        void initialize() override {
            rclcpp::init(0, nullptr);
            _node = std::make_shared<rclcpp::Node>("rtfw_bridge");
            helper_ = std::make_unique<ros2::BridgeHelper>(*_node);
            
            helper_->expose_interface(dr_velocity_sv, "/velocity/sv");
            helper_->expose_interface(dr_velocity_pv, "/velocity/pv");
            helper_->expose_interface(dr_wheel_sv, "/wheel/sv");
            helper_->expose_interface(dr_wheel_pv, "/wheel/pv");
            helper_->expose_interface(dr_error, std::string(dr_error.key()));
            helper_->expose_interface(dr_vel_target, std::string(dr_vel_target.key()));
            helper_->expose_interface(dr_odom, std::string(dr_odom.key()));
            helper_->expose_interface(dr_pose_target, std::string(dr_pose_target.key()));
            helper_->expose_interface(dr_local_target, std::string(dr_local_target.key()));
            helper_->expose_interface<std_msgs::msg::Float32MultiArray>(dw_wheel_sv, "/dummy_input");
        }
        void execute() override {            
            rclcpp::spin_some(_node);
            helper_->execute_publishers();
            helper_->execute_subscriptions();
        }
    private:
        DataReader<manif::SE3Tangentd> dr_velocity_sv{"velocity_sv"};
        DataReader<manif::SE3Tangentd> dr_velocity_pv{"velocity_pv"};
        DataReader<std::array<double, 4>> dr_wheel_sv{"wheel_sv"};
        DataReader<std::array<double, 4>> dr_wheel_pv{"wheel_pv"};
        DataReader<manif::SE3Tangentd> dr_error{"pose_error"};
        DataReader<manif::SE3Tangentd> dr_vel_target{"marker_iekf/vel_target"};
        DataReader<manif::SE2d> dr_odom{"odometry"};
        DataReader<manif::SE3d> dr_pose_target{"marker_iekf/pose_target"};
        DataReader<manif::SE3d> dr_local_target{"local_target"};
        DataWriter<std::array<double ,4>> dw_wheel_sv{"dummy"};

    private:
        rclcpp::Node::SharedPtr _node;
        std::unique_ptr<ros2::BridgeHelper> helper_;
    };
};