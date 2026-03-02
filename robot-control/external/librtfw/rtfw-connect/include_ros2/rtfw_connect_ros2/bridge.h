// In rtfw-connect/include/ros2/bridge.h
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rtfw_connect/client.h>
#include <functional>
#include <string>
#include <vector>
#include <memory>
#include <any>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int32_multi_array.hpp>

namespace rtfw::connect::ros2 {
    template<typename BaseType, typename ExportType>
    ExportType convert(const BaseType&);

    class Bridge {
    private:
        // --- 내부 헬퍼 클래스 정의 ---

        // Publisher 생성을 위한 타입 소거용 추상 인터페이스
        class IPublisherHelper {
        public:
            virtual ~IPublisherHelper() = default;
            // 활성화된 클라이언트가 있을 때만 주기적으로 호출될 함수
            virtual void check_and_publish() = 0;
            // 이 Publisher의 활성화 상태를 제어
            virtual void set_active(bool active) = 0;
        };

        // 실제 발행 로직을 담는 템플릿 구현체
        template<typename RtfwType, typename RosType>
        class PublisherHelper : public IPublisherHelper {
        public:
            PublisherHelper(
                std::shared_ptr<rclcpp::Node> node,
                const rtfw::connect::SharedMemoryQuerier& querier,
                const std::string& rtfw_key,
                const std::string& ros_topic)
                : node_(node), 
                reader_(querier.getDataReader<RtfwType>(rtfw_key)),
                last_read_tick_(0),
                is_active_(false)
            {
                publisher_ = node_->create_publisher<RosType>(ros_topic, 10);
            }

            void set_active(bool active) override {
                is_active_.store(active, std::memory_order_relaxed);
            }

            void check_and_publish() override {
                if (!is_active_.load(std::memory_order_relaxed)) return;

                // [수정] check_update()를 사용하여 데이터 변경 감지
                if (reader_.check_update()) {
                    const RtfwType* data_ptr = reader_.access();
                    if (data_ptr) {
                        auto msg = std::make_unique<RosType>(convert(*data_ptr));
                        publisher_->publish(std::move(msg));
                    }
                }
            }

        private:
            std::shared_ptr<rclcpp::Node> node_;
            rtfw::connect::ExternalDataReader<RtfwType> reader_;
            typename rclcpp::Publisher<RosType>::SharedPtr publisher_;
            uint64_t last_read_tick_;
            std::atomic<bool> is_active_;
        };
        
        // --- Bridge 멤버 변수 ---
        std::shared_ptr<rclcpp::Node> node_;
        std::unique_ptr<rtfw::connect::SharedMemoryConnector> connector_;
        std::unique_ptr<rtfw::connect::SharedMemoryQuerier> querier_;
        std::unique_ptr<rtfw::connect::SharedMemoryController> controller_;
        
        std::map<std::string, std::shared_ptr<IPublisherHelper>> publisher_helpers_;
        rclcpp::TimerBase::SharedPtr timer_;

        // --- 브로드캐스트 제어를 위한 멤버 ---
        std::mutex sub_manager_mutex_;
        // key: topic_name, value: set of client_ids
        std::map<std::string, std::set<uint32_t>> topic_subscribers_; 
        // key: client_id, value: last_seen_time
        std::map<uint32_t, std::chrono::steady_clock::time_point> client_heartbeats_;
        std::map<uint32_t, std::set<uint32_t>> client_subscriptions_;

        std::map<std::string, uint32_t> topic_to_index_map_;
        std::vector<std::string> index_to_topic_map_;

        // rclcpp::Service<rtfw_interfaces::srv::ManageSubscriptions>::SharedPtr sub_management_service_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr topic_list_pub_;
        rclcpp::Subscription<std_msgs::msg::UInt32MultiArray>::SharedPtr heartbeat_subscriber_;
        rclcpp::TimerBase::SharedPtr heartbeat_timeout_timer_;
        rclcpp::TimerBase::SharedPtr main_publisher_timer_;


    public:
        Bridge(const std::string& bridge_node_name, const std::string& shm_name);

        template<typename RtfwType>
        void add_publisher(const std::string& rtfw_key, const std::string& ros_topic);

        void run(int main_loop_rate_hz = 30);
            
    private:
        void setup_subscription_management();        
    };

    // --- 템플릿 멤버 함수 구현 ---

    template<typename RtfwType>
    void Bridge::add_publisher(const std::string& rtfw_key, const std::string& ros_topic) {
        // ADL을 활성화하기 위해 using 선언

        // 변환 함수의 결과 타입을 추론하여 RosType을 결정
        using RosType = std::decay_t<decltype(convert(std::declval<const RtfwType&>()))>;

        static_assert(!std::is_void_v<RosType>, 
                    "A 'convert(const YourRtfwType&)' function must be defined and visible.");

        // PublisherHelper를 생성하여 맵에 저장
        auto helper = std::make_shared<PublisherHelper<RtfwType, RosType>>(node_, *querier_, rtfw_key, ros_topic);
        publisher_helpers_[ros_topic] = helper;
    }

} // namespace rtfw::connect::ros2

// int main_sample(int argc, char** argv) {
//     rclcpp::init(argc, argv);
    
//     // 1. Bridge 객체 생성 (모든 초기화가 내부에서 일어남)
//     rtfw::connect::ros2::Bridge bridge("my_robot_bridge", "/rtfw_shm");

//     // 2. 설정: 어떤 데이터를 어떻게 노출할지 "선언"
//     bridge.add_publisher<HighFreqSensors>("/sensors/high_freq", "/robot/joint_states");
//     bridge.add_publisher<RobotVelocity>("/robot/velocity", "/robot/cmd_vel_echo");
    
//     bridge.add_parameter_service<double, ...>("/gait/pid/p", "/robot/set_p_gain");
//     // ...

//     // 3. 실행: 이 함수가 종료될 때까지 Bridge가 모든 것을 알아서 처리
//     bridge.run();

//     rclcpp::shutdown();
//     return 0;
// }