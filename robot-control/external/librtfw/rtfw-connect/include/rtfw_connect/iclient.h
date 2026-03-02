#pragma once

#include <optional>
#include <string>
#include <vector>
#include <chrono>
#include <memory>

namespace rtfw::connect {

struct HandleOptions {
  enum class SubPolicy {
    // 핸들이 없어져도 내부 구독은 유지. 다음 get_handle 시 즉시 데이터 수신 가능.
    Persistent,
    // 마지막 핸들이 사라지면 내부 구독도 함께 해제. 네트워크 트래픽을 완전히 차단.
    Strict
  };
  SubPolicy sub_policy = SubPolicy::Persistent;
};

/**
 * @brief 데이터에 접근하는 사용자용 핸들 인터페이스.
 * 구현체는 이 클래스를 상속하여 실제 동작을 정의합니다.
 */
template<typename T>
class THandle {
public:
    virtual ~THandle() = default;
    virtual const std::string& topic_name() const = 0;
    virtual bool read_latest(T& out) = 0;
    virtual std::shared_ptr<const T> read_latest_ptr() = 0;
};


/**
 * @brief 통신 클라이언트의 공통 인터페이스.
 */
class IClient {
public:
    virtual ~IClient() = default;

    virtual bool connect() = 0;
    virtual void shutdown() = 0;
    virtual std::vector<std::string> list_topics() const = 0;
    virtual std::optional<uint32_t> topic_index(const std::string& name) const = 0;

    /**
     * @brief 특정 토픽에 대한 타입-특화 핸들을 요청합니다.
     * 템플릿 함수는 가상이 될 수 없으므로, 이 함수는 구현체에서 직접 구현되어야 합니다.
     * 이로 인해 IClient 포인터로는 get_handle을 직접 호출할 수 없지만,
     * 실제 사용하는 클라이언트 타입(e.g., RemoteClient)으로는 호출 가능합니다.
     * (이것이 가장 순수한 형태이며, 필요 시 이전의 type-erasure 기법을 다시 도입할 수 있습니다)
     */
    // template<typename T>
    // std::shared_ptr<THandle<T>> get_handle(const std::string& topic_name); // 구현체에서 정의

    virtual void send_heartbeat() = 0;
    virtual void set_keepalive_period(std::optional<std::chrono::milliseconds> period) = 0;
};

}