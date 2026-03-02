#include "rtfw/parameter_manager.h"
#include "rtfw/shm_context.h"
#include "rtfw_common/type_utils.h" // get_stable_type_signature
#include <fstream> // for file check

using namespace rtfw::internal;

ParameterManager::ParameterManager() {
    // 생성자는 이제 비어있음
}

// --- 1. 초기화 단계 ---

void ParameterManager::collectRequests(const std::vector<rt::ITask*>& tasks) {
    for (rt::ITask* task : tasks) {
        for (const auto& req : task->_param_read_requests) { // ITask에 추가한 _param_read_requests
            if (_key_hash_to_req_idx.count(req.key_hash)) {
                // 이미 다른 태스크에서 동일한 키를 요청했으므로, 타입만 검증
                const auto& existing_req = _collected_requests[_key_hash_to_req_idx[req.key_hash]];
                if (existing_req.type_hash != req.type_hash) {
                    throw std::logic_error("Type mismatch for parameter key '" + req.key + "'.");
                }
            } else {
                // 새로운 요청 추가
                _key_hash_to_req_idx[req.key_hash] = _collected_requests.size();
                _collected_requests.push_back({
                    req.key, req.key_hash, req.type_hash,
                    req.size, req.alignment, req.parameter_proxy, req.wire_job
                });
            }
        }
    }
}

bool ParameterManager::prepareLayout(const std::string& yaml_path) {
    std::ifstream file(yaml_path);
    if (!file.good()) {
        // throw std::runtime_error("Parameter file not found: " + yaml_path);
        return false;
    }
    // 1. YAML 파일 파싱
    _yaml_root = YAML::LoadFile(yaml_path);

    std::map<std::string, YAML::Node> flat_yaml_map;
    parseYamlNodeRecursive(_yaml_root, "", flat_yaml_map);

    // 2. 파싱된 YAML과 수집된 요청을 비교하여 최종 ParameterInfo 생성
    _param_infos.reserve(_collected_requests.size());
    size_t current_offset = 0;

    for (const auto& req : _collected_requests) {
        std::string param_key_without_prefix = req.key;
        if (param_key_without_prefix.rfind("p.", 0) == 0) { // "p." 접두사 제거
            param_key_without_prefix = param_key_without_prefix.substr(2);
        }

        if (flat_yaml_map.find(param_key_without_prefix) == flat_yaml_map.end()) {
            // throw std::runtime_error("Parameter '" + req.key + "' requested by a task, but not found in " + yaml_path);
            continue;
        }

        common::ParameterInfo info;
        strncpy(info.key, req.key.c_str(), sizeof(info.key) - 1);
        info.key[sizeof(info.key) - 1] = '\0';
        info.key_hash = req.key_hash;
        info.type_hash = req.type_hash;
        info.data_size = req.size;
        info.data_alignment = req.alignment;
        
        // 오프셋 계산
        current_offset = common::align_offset(current_offset, info.data_alignment);
        info.offset_in_buffer = current_offset;
        current_offset += info.data_size;

        _param_infos.push_back(info);
    }

    _total_buffer_size = current_offset;
    return true;
}

void ParameterManager::initializeFromShm(SharedMemoryContext& shm_context) {
    _shm_base_ptr = shm_context.getBasePtr();
    if (!_shm_base_ptr) throw std::runtime_error("SHM base pointer is null in ParameterManager.");

    auto* header = shm_context.getHeader();
    _block_desc = (common::ParameterBlockDescriptor*)((char*)_shm_base_ptr + header->param_block_descriptor_offset);
    _block_header = (common::ParameterBlockHeader*)((char*)_shm_base_ptr + _block_desc->block_header_offset);
    _shm_param_info_array = (common::ParameterInfo*)((char*)_shm_base_ptr + header->param_info_array_offset);
    
    _buffer0 = (char*)_shm_base_ptr + _block_desc->buffer_0_data_offset;
    _buffer1 = (char*)_shm_base_ptr + _block_desc->buffer_1_data_offset;
}

void ParameterManager::writeInitialValues() {
    if (!_yaml_root || !_buffer0 || !_buffer1) return;

    // 파싱된 YAML 맵을 다시 생성
    std::map<std::string, YAML::Node> flat_yaml_map;
    parseYamlNodeRecursive(_yaml_root, "", flat_yaml_map);


    for (const auto& req : _collected_requests) {
        std::string key_without_prefix = req.key;
        auto it = flat_yaml_map.find(key_without_prefix);
        if (it != flat_yaml_map.end()) {
            const YAML::Node& val_node = it->second;
            // [핵심] 그냥 IParameter 객체에게 "네가 알아서 초기화해" 라고 위임
            req.parameter_proxy->initializeFromYaml(_shm_base_ptr, val_node);
        }
    }
}


// --- 2. 연결 단계 ---

void ParameterManager::wireParameterReaders(const std::vector<rt::ITask*>& tasks) {
    std::map<uint64_t, const common::ParameterInfo*> info_map;
    for (const auto& info : _param_infos) {
        info_map[info.key_hash] = &info;
    }

    for (rt::ITask* task : tasks) {
        for (const auto& req : task->_param_read_requests) {
            auto it = info_map.find(req.key_hash);
            if (it != info_map.end()) {
                req.wire_job(_shm_base_ptr, it->second); // 캡처된 람다(wire_job)를 실행하여 연결
            }
        }
    }
}


// --- 3. 실행 단계 ---

void ParameterManager::swapBuffers() {
    // ParameterController가 Working 버퍼의 버전을 올렸을 것임.
    // Stable 버퍼와 Working 버퍼의 버전이 다른지 확인.
    int stable_idx = _block_header->stable_index.load(std::memory_order_acquire);
    int working_idx = 1 - stable_idx;
    
    uint64_t stable_ver = _block_header->version[stable_idx].load(std::memory_order_acquire);
    uint64_t working_ver = _block_header->version[working_idx].load(std::memory_order_acquire);

    if (stable_ver < working_ver) {
        // 버전이 다르다는 것은 파라미터가 수정되었다는 의미.
        // stable_index를 working_idx로 변경하여 스왑한다.
        _block_header->stable_index.store(working_idx, std::memory_order_release);
    }
}

// --- 헬퍼 함수 ---
void ParameterManager::parseYamlNodeRecursive(const YAML::Node& node, const std::string& prefix, 
                                            std::map<std::string, YAML::Node>& out_flat_map) 
{
    if (node.IsMap()) {
        for (auto it = node.begin(); it != node.end(); ++it) {
            std::string key = it->first.as<std::string>();
            std::string new_prefix = prefix.empty() ? key : prefix + "." + key;
            parseYamlNodeRecursive(it->second, new_prefix, out_flat_map);
        }
    } else if (node.IsScalar()) {
        out_flat_map[prefix] = node;
    } else if (node.IsSequence()) {
        bool all_scalar = true;
        for (const auto& item : node) {
            if (!item.IsScalar()) {
                all_scalar = false;
                break;
            }
        }
        if (all_scalar) {
            out_flat_map[prefix] = node;
        }
    }
    // IsSequence는 파라미터에서 지원하지 않는 것으로 가정
}