// in rtfw-connect/src/local/local_client.cpp
#include "rtfw_connect/generalclient.h" // 구현할 헤더
#include "rtfw_connect/shm_querier.h"
#include "rtfw_connect/shm_controller.h"

class LocalClient : public IClient {
public:
    LocalClient(void* shm_ptr) 
        : querier_(shm_ptr), controller_(shm_ptr) {}
    
    // ... IClient 인터페이스의 모든 가상 함수를 Querier/Controller를 호출하여 구현 ...
    std::shared_ptr<ISubscriptionHandle> subscribe(const std::string& topic_name) override {
        // Local 모드에서는 데이터 키가 토픽 이름. 
        // LocalHandle을 만들어 반환
    }

protected:
    bool set_parameter_any(const std::string& key, const std::any& value) override {
        // std::any의 타입을 확인하고, 맞는 controller_.setParameter<T> 호출
    }
    // ...
private:
    SharedMemoryQuerier querier_;
    SharedMemoryController controller_;
};
