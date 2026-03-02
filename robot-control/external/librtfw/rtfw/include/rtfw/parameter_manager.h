// rtfw/ParameterManager.h

#pragma once
#include <vector>
#include <string>
#include <map>
#include <yaml-cpp/yaml.h> // YAML 파싱을 위해 추가

#include "rtfw_common/shm_layout.h"
#include "rtfw/task.h"


namespace rtfw::internal {
    class SharedMemoryContext;

class ParameterManager {
public:
    // [변경] 생성자는 이제 아무것도 하지 않음.
    ParameterManager(); 

    // --- 1. 초기화 단계 (RealTimeFramework가 순서대로 호출) ---
    
    // Task들로부터 Parameter<T> 요청을 수집하여 내부 목록에 저장.
    void collectRequests(const std::vector<rt::ITask*>& tasks);

    // YAML 파일을 로드하여, 수집된 요청과 비교/검증하고,
    // 최종 ParameterInfo 목록과 버퍼 사이즈를 계산.
    bool prepareLayout(const std::string& yaml_path);
    
    // RealTimeFramework가 공유 메모리 빌드를 끝낸 후 호출됨.
    // 공유 메모리의 관련 포인터들을 멤버 변수에 저장.
    void initializeFromShm(SharedMemoryContext& shm_context);

    // YAML 파일의 초기값을 공유 메모리의 두 버퍼에 모두 기록.
    void writeInitialValues();


    // --- 2. 연결 단계 (RealTimeFramework가 호출) ---

    // 수집된 요청과 ParameterInfo 목록을 바탕으로
    // 모든 Parameter 프록시에 wire_job을 실행.
    void wireParameterReaders(const std::vector<rt::ITask*>& tasks);


    // --- 3. 실행 단계 (RealTimeFramework가 호출) ---
    
    // Working -> Stable 버퍼 스왑 (필요 시)
    void swapBuffers();

    int getStableIndex() const {
        return _block_header->stable_index.load(std::memory_order_acquire);
    }

    uint64_t getVersion(int buffer_index) const {
        return _block_header->version[buffer_index].load(std::memory_order_acquire);
    }
    
    const char* getBuffer(int buffer_index) const {
        return (buffer_index == 0) ? _buffer0 : _buffer1;
    }

    size_t getBufferSize() const {
        return _total_buffer_size;
    }

    // --- 4. 외부 API (rtfw-client 연동을 위한 내부 로직) ---
    // (이 부분은 ParameterController가 직접 SHM을 조작하므로,
    //  Manager 클래스 자체에는 public API가 필요 없을 수 있음.
    //  다만, 일관성을 위해 Manager를 거치게 할 수도 있음)


    // --- 5. Getter (RealTimeFramework가 Layout 계산 시 필요) ---
    std::vector<common::ParameterInfo>& getParamInfos() { return _param_infos; }
    size_t getCalculatedBufferSize() const { return _total_buffer_size; }


private:
    // 빌드 과정에서 사용될 임시 정보
    struct ParamRequestInfo { // ITask의 ParamReadRequest와 유사한 정보
        std::string key;
        uint64_t key_hash;
        uint64_t type_hash;
        size_t size;
        size_t alignment;
        IParameter* parameter_proxy;
        std::function<void(void* shm_base_ptr, const common::ParameterInfo*)> wire_job;
    };
    std::vector<ParamRequestInfo> _collected_requests;
    std::map<uint64_t, size_t> _key_hash_to_req_idx;
    
    std::vector<common::ParameterInfo> _param_infos;
    size_t _total_buffer_size = 0;
    YAML::Node _yaml_root; // 파싱된 YAML 데이터를 임시 저장

    // 빌드 후 공유 메모리를 직접 가리킬 포인터들
    void* _shm_base_ptr = nullptr;
    common::ParameterBlockDescriptor* _block_desc = nullptr;
    common::ParameterBlockHeader* _block_header = nullptr;
    common::ParameterInfo* _shm_param_info_array = nullptr; // SHM에 있는 info 배열
    char* _buffer0 = nullptr;
    char* _buffer1 = nullptr;

    // YAML 파싱 헬퍼 함수
    void parseYamlNodeRecursive(const YAML::Node& node, const std::string& prefix, 
                                std::map<std::string, YAML::Node>& out_flat_map);
};

} // namespace rtfw