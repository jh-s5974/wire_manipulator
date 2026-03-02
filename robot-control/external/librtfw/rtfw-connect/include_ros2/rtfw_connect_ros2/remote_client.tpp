// remote_client.tpp
#pragma once

// 이 파일은 remote_client.h에 의해서만 포함되어야 합니다.

#include "rtfw_connect_ros2/remote_client.h"

// 여기에 지원할 메시지 타입 헤더들을 포함할 필요가 없습니다.
// 사용자의 코드에서 직접 포함하기 때문입니다.

namespace rtfw::connect::ros2 {

// --- 내부 Body 클래스 구현 ---
// 이 클래스는 get_handle<T>가 호출될 때만 필요하므로, tpp 파일에 함께 둡니다.
template<typename T>
class SubscriptionProxy final : public ISubscriptionProxy,
                                public std::enable_shared_from_this<SubscriptionProxy<T>> {
public:
    SubscriptionProxy(rclcpp::Node::SharedPtr node, const std::string& topic, const rtfw::connect::ros2::RemoteClientOptions& opt, HandleOptions::SubPolicy policy)
        : topic_name_(topic), policy_(policy) {
        
        auto callback = [this](std::shared_ptr<const T> msg) {
            std::atomic_store(&this->latest_msg_, msg);
        };
        sub_ = node->create_subscription<T>(topic, opt.data_qos, callback);
    }
    
    ~SubscriptionProxy() override {
        if (sub_) sub_.reset();
    }

    const std::string& topic_name() const override { return topic_name_; }
    HandleOptions::SubPolicy get_policy() const override { return policy_; }

    bool is_active() const override {
        // weak_from_this()로 생성된 shared_ptr의 use_count가 1보다 크면 사용자 핸들이 존재.
        return !this->weak_from_this().expired() && this->shared_from_this().use_count() > 1;
    }

    std::shared_ptr<const T> get_latest_ptr() const {
        return std::atomic_load(&latest_msg_);
    }

private:
    std::string topic_name_;
    HandleOptions::SubPolicy policy_;
    typename rclcpp::Subscription<T>::SharedPtr sub_;
    std::shared_ptr<const T> latest_msg_;
};



// --- ROSTHandle (Proxy) 구현 ---
template<typename T>
ROSTHandle<T>::ROSTHandle(std::weak_ptr<RemoteClient> owner, std::shared_ptr<ISubscriptionProxy> proxy)
    : owner_(std::move(owner)), proxy_(std::move(proxy)) {}

template<typename T>
ROSTHandle<T>::~ROSTHandle() {
    // 소멸 시 owner에게 알려 리소스 정리를 위임
    if (auto owner = owner_.lock()) {
        owner->release_handle(proxy_->topic_name());
    }
}

template<typename T>
const std::string& ROSTHandle<T>::topic_name() const {
    return proxy_->topic_name();
}

template<typename T>
bool ROSTHandle<T>::read_latest(T& out) {
    auto msg_ptr = read_latest_ptr();
    if (!msg_ptr) return false;
    out = *msg_ptr;
    return true;
}

template<typename T>
std::shared_ptr<const T> ROSTHandle<T>::read_latest_ptr() {
    // Body를 실제 타입으로 다운캐스팅하여 데이터에 접근
    if (auto concrete_proxy = std::dynamic_pointer_cast<SubscriptionProxy<T>>(proxy_)) {
        return concrete_proxy->get_latest_ptr();
    }
    return nullptr;
}



// --- RemoteClient의 핵심 템플릿 API 구현 ---
template<typename T>
std::shared_ptr<THandle<T>> RemoteClient::get_handle(const std::string& topic_name, const HandleOptions& opt) {
    std::lock_guard lock(mtx_);

    auto it = proxies_.find(topic_name);
    if (it != proxies_.end()) { // 이미 프록시가 존재할 경우
        if (auto existing_proxy = std::dynamic_pointer_cast<SubscriptionProxy<T>>(it->second)) {
            if (existing_proxy->get_policy() != opt.sub_policy) {
                RCLCPP_WARN(node_->get_logger(), "Topic '%s' was requested with a different subscription policy. The original policy will be used.", topic_name.c_str());
            }
            return std::make_shared<ROSTHandle<T>>(weak_from_this(), existing_proxy);
        } else {
            throw std::runtime_error("Topic '" + topic_name + "' requested with a new type, but it already exists with a different type.");
        }
    }

    auto new_proxy = std::make_shared<SubscriptionProxy<T>>(node_, topic_name, opt_, opt.sub_policy);
    proxies_[topic_name] = new_proxy;
    
    return std::make_shared<ROSTHandle<T>>(weak_from_this(), new_proxy);
}

} // namespace rtfw::connect::ros2