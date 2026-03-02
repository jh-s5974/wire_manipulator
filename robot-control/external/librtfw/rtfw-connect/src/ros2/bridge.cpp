// In rtfw-connect/src/ros2/bridge.h

#include "rtfw_connect_ros2/bridge.h"
#include <sstream> // for std::stringstream


using namespace std::chrono_literals;

namespace rtfw::connect::ros2 {

Bridge::Bridge(const std::string& bridge_node_name, const std::string& shm_name)
    : node_(std::make_shared<rclcpp::Node>(bridge_node_name)),
      connector_(std::make_unique<rtfw::connect::SharedMemoryConnector>()), // 멤버 이니셜라이저에서 기본 생성
      querier_(nullptr), // 아래에서 유효한 포인터로 초기화
      controller_(nullptr)
{
    void* shm_ptr = connector_->connect(shm_name.c_str());
    if (!shm_ptr) {
        throw std::runtime_error("Failed to connect to shared memory: " + shm_name);
    }
    // Querier와 Controller를 생성. 포인터가 아니므로 복사 발생.
    // (더 나은 방법은 unique_ptr를 사용하는 것일 수 있음)
    querier_ = std::make_unique<rtfw::connect::SharedMemoryQuerier>(shm_ptr);
    controller_ = std::make_unique<rtfw::connect::SharedMemoryController>(shm_ptr);
}

void Bridge::run(int main_loop_rate_hz) {
    // 1. On-Demand 구독 관리 서비스 및 Heartbeat 기능 활성화
    setup_subscription_management();


    // --- 2. 메인 발행 타이머 생성 ---
    main_publisher_timer_ = node_->create_wall_timer(1000000us / main_loop_rate_hz, [this]() {
        // 1. 활성화할 토픽 인덱스의 전체 합집합 계산
        std::set<uint32_t> topics_to_activate;
        {
            std::lock_guard<std::mutex> lock(this->sub_manager_mutex_);
            for (const auto& pair : this->client_subscriptions_) {
                topics_to_activate.insert(pair.second.begin(), pair.second.end());
            }
        } // 락 범위 최소화

        // 2. 모든 PublisherHelper를 순회하며 상태 설정 및 발행 시도
        for (uint32_t i = 0; i < this->index_to_topic_map_.size(); ++i) {
            const std::string& topic_name = this->index_to_topic_map_[i];
            auto& helper = this->publisher_helpers_.at(topic_name);
            
            bool should_be_active = topics_to_activate.count(i);
            helper->set_active(should_be_active);
            
            // check_and_publish는 내부적으로 active 플래그를 확인하므로 항상 호출 가능
            helper->check_and_publish();
        }
    });
    
    // 3. ROS2 메인 루프 실행
    rclcpp::spin(node_);
}


void Bridge::setup_subscription_management() {
    RCLCPP_INFO(node_->get_logger(), "Finalizing bridge setup...");
    
    // --- 1. 토픽 목록 생성 및 발행 ---
    std::vector<std::string> topic_names;
    for (const auto& pair : publisher_helpers_) {
        topic_names.push_back(pair.first);
    }
    // 안정적인 순서를 위해 정렬 (선택적이지만 권장)
    std::sort(topic_names.begin(), topic_names.end());

    // 인덱스 맵 생성 (이제 순서가 인덱스가 됨)
    std::stringstream ss;
    bool first = true;
    for (uint32_t i = 0; i < topic_names.size(); ++i) {
        const std::string& name = topic_names[i];
        topic_to_index_map_[name] = i;
        index_to_topic_map_.push_back(name); // 역방향 맵
        
        if (!first) ss << ",";
        ss << name;
        first = false;
    }

    // Latched Publisher 생성 및 발행
    topic_list_pub_ = node_->create_publisher<std_msgs::msg::String>(
        "/rtfw/topic_list", rclcpp::QoS(1).transient_local());
    auto msg = std::make_unique<std_msgs::msg::String>();
    msg->data = ss.str();
    topic_list_pub_->publish(std::move(msg));
    RCLCPP_INFO(node_->get_logger(), "Published %zu topics to /rtfw/topic_list", topic_names.size());


    // --- 2. Heartbeat/구독 마스크 구독자 생성 ---
    heartbeat_subscriber_ = node_->create_subscription<std_msgs::msg::UInt32MultiArray>(
        "/rtfw/heartbeat", 10,
        [this](const std_msgs::msg::UInt32MultiArray::SharedPtr msg) {
            if (msg->data.empty()) return;

            std::lock_guard<std::mutex> lock(this->sub_manager_mutex_);
            
            uint32_t client_id = msg->data[0];
            
            // Heartbeat 시간 갱신
            this->client_heartbeats_[client_id] = std::chrono::steady_clock::now();
            
            // 새로운 구독 마스크 생성
            std::set<uint32_t> new_mask;
            for (size_t i = 1; i < msg->data.size(); ++i) {
                new_mask.insert(msg->data[i]);
            }
            
            // 기존 마스크와 비교하여 변경 사항 로깅 (디버깅용)
            // if (this->client_subscriptions_[client_id] != new_mask) {
            //     RCLCPP_DEBUG(this->node_->get_logger(), "Client %u updated subscription mask.", client_id);
            // }
            
            // 맵에 새로운 마스크 덮어쓰기
            this->client_subscriptions_[client_id] = new_mask;
        });

    // --- 3. Heartbeat 타임아웃 체커 타이머 생성 ---
    heartbeat_timeout_timer_ = node_->create_wall_timer(1s, [this]() {
        std::lock_guard<std::mutex> lock(this->sub_manager_mutex_);
        const auto now = std::chrono::steady_clock::now();
        const auto timeout_duration = std::chrono::seconds(3);
        
        std::vector<uint32_t> clients_to_remove;
        for (const auto& pair : this->client_heartbeats_) {
            if (now - pair.second > timeout_duration) {
                clients_to_remove.push_back(pair.first);
            }
        }
        
        for (const auto& client_id : clients_to_remove) {
            RCLCPP_WARN(this->node_->get_logger(), "Client %u timed out. Removing subscriptions.", client_id);
            this->client_heartbeats_.erase(client_id);
            this->client_subscriptions_.erase(client_id);
        }
    });
}


} // namespace rtfw::connect::ros2