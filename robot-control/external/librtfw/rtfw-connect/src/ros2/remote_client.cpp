#include "rtfw_connect_ros2/remote_client.h"
#include <random>
#include <sstream>

// 지원할 메시지 타입 헤더들
#include <std_msgs/msg/float32.hpp>

namespace rtfw::connect::ros2 {

void RemoteClient::release_handle(const std::string& topic_name) {
    std::lock_guard lock(mtx_);
    auto it = proxies_.find(topic_name);
    if (it == proxies_.end()) return;

    auto& proxy = it->second;
    
    // use_count가 2인 경우: proxies_ 맵에 하나, 방금 소멸된 ROSTHandle에 하나.
    // 즉, 마지막 사용자 핸들이었음을 의미.
    if (proxy.use_count() <= 2) { 
        if (proxy->get_policy() == HandleOptions::SubPolicy::Strict) {
            // Strict 정책이면 구독 자체를맵에서 제거하여 파괴
            proxies_.erase(it);
        }
        // Persistent 정책이면 아무것도 하지 않음. 프록시는 맵에 계속 남아있음.
    }
}

// --- 나머지 RemoteClient 구현 (이전과 거의 동일) ---
std::shared_ptr<RemoteClient> RemoteClient::create(rclcpp::Node::SharedPtr node, RemoteClientOptions opt) {
    return std::shared_ptr<RemoteClient>(new RemoteClient(std::move(node), std::move(opt)));
}
RemoteClient::RemoteClient(rclcpp::Node::SharedPtr node, RemoteClientOptions opt) : node_(std::move(node)), opt_(std::move(opt)) { client_id_ = make_client_id_(); }

bool RemoteClient::connect() {
    topic_list_sub_ = node_->create_subscription<std_msgs::msg::String>(
        opt_.topic_list_topic, opt_.topic_list_qos,
        std::bind(&RemoteClient::handle_topic_list, this, std::placeholders::_1));

    hb_pub_ = node_->create_publisher<std_msgs::msg::UInt32MultiArray>(
        opt_.heartbeat_topic, opt_.heartbeat_qos);

    const auto start = std::chrono::steady_clock::now();
    while (!connected_.load() && rclcpp::ok()) {
        rclcpp::spin_some(node_);
        if (std::chrono::steady_clock::now() - start > std::chrono::seconds(2)) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return connected_.load();
}

void RemoteClient::shutdown() {
    std::lock_guard lock(mtx_);
    set_keepalive_period(std::nullopt);
    if(hb_pub_) hb_pub_.reset();
    proxies_.clear(); // 이 한 줄이 모든 구독을 정리합니다!
    if(topic_list_sub_) topic_list_sub_.reset();
    topic_to_index_.clear();
    index_to_topic_.clear();
    connected_.store(false);
}

uint32_t RemoteClient::make_client_id_() const {
  const auto t = static_cast<uint64_t>(
      std::chrono::steady_clock::now().time_since_epoch().count());
  std::mt19937_64 rng(t ^ 0x9e3779b97f4a7c15ull);
  return static_cast<uint32_t>(rng());
}

void RemoteClient::handle_topic_list(std_msgs::msg::String::SharedPtr msg) {
    std::vector<std::string> names;
    names.reserve(128);
    std::istringstream ss(msg->data);
    std::string tok;
    while (std::getline(ss, tok, ',')) {
        tok.erase(tok.begin(), std::find_if(tok.begin(), tok.end(), [](unsigned char ch){ return !std::isspace(ch); }));
        tok.erase(std::find_if(tok.rbegin(), tok.rend(), [](unsigned char ch){ return !std::isspace(ch); }).base(), tok.end());
        if (!tok.empty()) names.push_back(tok);
    }
    std::lock_guard lock(mtx_);
    index_to_topic_ = names;
    topic_to_index_.clear();
    for (uint32_t i=0; i<names.size(); ++i) topic_to_index_[names[i]] = i;
    connected_.store(true);
}

void RemoteClient::send_heartbeat() {
    if (!hb_pub_) return;
    
    std::vector<uint32_t> active_indices;
    {
        std::lock_guard lock(mtx_);
        active_indices.reserve(proxies_.size());
        for (const auto& [name, proxy] : proxies_) {
            // is_active() 호출로 참조 카운트 기반 활성 상태 확인!
            if (proxy->is_active()) {
                if (auto it_idx = topic_to_index_.find(name); it_idx != topic_to_index_.end()) {
                    active_indices.push_back(it_idx->second);
                }
            }
        }
    }

    std_msgs::msg::UInt32MultiArray msg;
    msg.data.reserve(active_indices.size() + 1);
    msg.data.push_back(client_id_);
    msg.data.insert(msg.data.end(), active_indices.begin(), active_indices.end());
    hb_pub_->publish(msg);
}

void RemoteClient::set_keepalive_period(std::optional<std::chrono::milliseconds> period) {
    if (keepalive_timer_) {
        keepalive_timer_->cancel();
        keepalive_timer_.reset();
    }
    if (period) {
        keepalive_timer_ = node_->create_wall_timer(*period, [this](){ this->send_heartbeat(); });
    }
}

std::vector<std::string> RemoteClient::list_topics() const {
    std::lock_guard lock(mtx_);
    return index_to_topic_;
}

std::optional<uint32_t> RemoteClient::topic_index(const std::string& name) const {
    std::lock_guard lock(mtx_);
    auto it = topic_to_index_.find(name);
    if (it == topic_to_index_.end()) return std::nullopt;
    return it->second;
}

} // namespace rtfw::connect::ros2