#pragma once

#include "rtfw_connect/iclient.h"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int32_multi_array.hpp>
#include <atomic>
#include <mutex>
#include <unordered_map>

namespace rtfw::connect::ros2 {

    class RemoteClient;

    struct RemoteClientOptions {
        std::string topic_list_topic = "/rtfw/topic_list";
        std::string heartbeat_topic  = "/rtfw/heartbeat";
        rclcpp::QoS topic_list_qos   = rclcpp::QoS(1).reliable().transient_local();
        rclcpp::QoS heartbeat_qos    = rclcpp::QoS(1).reliable();
        rclcpp::QoS data_qos         = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
    };

    // --- 내부 관리용 클래스 (Body) ---
    // 실제 ROS 구독과 데이터 저장을 담당하는 "본체(Body)" 클래스.
    class ISubscriptionProxy {
    public:
        virtual ~ISubscriptionProxy() = default;
        virtual const std::string& topic_name() const = 0;
        // 하트비트 전송 시 활성 상태를 확인하기 위한 가상 함수.
        // 참조 카운트가 1 초과면 (즉, 사용자 핸들이 존재하면) true를 반환.
        virtual bool is_active() const = 0; 
        virtual HandleOptions::SubPolicy get_policy() const = 0;
    };

    // --- 사용자에게 전달될 핸들의 실제 구현 (Proxy) ---
    template<typename T>
    class ROSTHandle final : public rtfw::connect::THandle<T> {
    public:
        // 생성자는 내부용 Body에 대한 shared_ptr를 받습니다.
        ROSTHandle(std::weak_ptr<RemoteClient> owner, std::shared_ptr<ISubscriptionProxy> proxy);
        ~ROSTHandle() override; // 소멸자에서 release_handle 호출

        const std::string& topic_name() const override;
        bool read_latest(T& out) override;
        std::shared_ptr<const T> read_latest_ptr() override;

    private:
        // 실제 데이터가 저장된 Body에 대한 포인터.
        // 이 포인터의 생명주기가 RAII의 핵심입니다.
        std::weak_ptr<RemoteClient> owner_;
        std::shared_ptr<ISubscriptionProxy> proxy_;
    };

    // --- ROS2 IClient 구현체 ---
    class RemoteClient final : public IClient,
                            public std::enable_shared_from_this<RemoteClient> {
    public:
        static std::shared_ptr<RemoteClient> create(rclcpp::Node::SharedPtr node, RemoteClientOptions opt = {});

        // IClient 인터페이스 구현
        bool connect() override;
        void shutdown() override;
        std::vector<std::string> list_topics() const override;
        std::optional<uint32_t> topic_index(const std::string& name) const override;
        void send_heartbeat() override;
        void set_keepalive_period(std::optional<std::chrono::milliseconds> period) override;

        // 핵심 API: 템플릿을 이용해 타입-안전한 핸들을 생성하고 반환
        template<typename T>
        std::shared_ptr<THandle<T>> get_handle(const std::string& topic_name, const HandleOptions& opt = {});

    private:
        template<typename T> friend class ROSTHandle; 

        RemoteClient(rclcpp::Node::SharedPtr node, RemoteClientOptions opt);

        void handle_topic_list(std_msgs::msg::String::SharedPtr msg);
        uint32_t make_client_id_() const;
        void release_handle(const std::string& topic_name);
        
        rclcpp::Node::SharedPtr node_;
        RemoteClientOptions opt_;
        mutable std::mutex mtx_;

        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr topic_list_sub_;
        std::vector<std::string> index_to_topic_;
        std::unordered_map<std::string, uint32_t> topic_to_index_;

        rclcpp::Publisher<std_msgs::msg::UInt32MultiArray>::SharedPtr hb_pub_;
        uint32_t client_id_{0};
        rclcpp::TimerBase::SharedPtr keepalive_timer_;

        // 모든 구독 "본체(Body)"를 관리하는 맵.
        // 이 맵이 각 프록시에 대한 최소 1의 참조 카운트를 유지합니다.
        std::unordered_map<std::string, std::shared_ptr<ISubscriptionProxy>> proxies_;

        std::atomic<bool> connected_{false};
    };

} // namespace rtfw::connect::ros2

#include "rtfw_connect_ros2/remote_client.tpp"