// rtfw-client/shm_querier.cpp
#include "rtfw_connect/shm_querier.h"
#include <stdexcept>


using namespace rtfw::connect;
using namespace rtfw::common;

SharedMemoryQuerier::SharedMemoryQuerier(void* shm_base_ptr)
    : _shm_base_ptr(shm_base_ptr), _header(nullptr) 
{
    if (_shm_base_ptr) {
        _header = static_cast<const common::SharedMemoryHeader*>(_shm_base_ptr);
        buildCache();
    }
}

bool SharedMemoryQuerier::isConnected() const {
    return _shm_base_ptr != nullptr;
}

void SharedMemoryQuerier::buildCache() {
    if (!isConnected()) return;
    _descriptor_cache.clear();
    const char* base = static_cast<const char*>(_shm_base_ptr);
    const DataBlockDescriptor* descriptors = (const DataBlockDescriptor*)(base + _header->descriptor_array_offset);
    for (size_t i = 0; i < _header->descriptor_count; ++i) {
        // key는 null-terminated string이므로 string_view로 캐싱 가능
        _descriptor_cache[descriptors[i].key] = &descriptors[i];
    }

    _param_info_cache.clear();
    const ParameterInfo* param_infos = (const ParameterInfo*)(base + _header->param_info_array_offset);
    for (size_t i = 0; i < _header->param_info_count; ++i) {
        _param_info_cache[param_infos[i].key] = &param_infos[i];
    }
}

const SharedMemoryHeader* SharedMemoryQuerier::getHeader() const {
    return _header;
}

const DataBlockDescriptor* SharedMemoryQuerier::getDescriptor(std::string_view key) const {
    auto it = _descriptor_cache.find(key);
    return (it != _descriptor_cache.end()) ? it->second : nullptr;
}

std::vector<DataBlockDescriptor> SharedMemoryQuerier::getAllDescriptors() const {
    if (!isConnected()) return {};
    const char* base = static_cast<const char*>(_shm_base_ptr);
    const DataBlockDescriptor* desc_ptr = (const DataBlockDescriptor*)(base + _header->descriptor_array_offset);
    return std::vector<DataBlockDescriptor>(desc_ptr, desc_ptr + _header->descriptor_count);
}

const void* SharedMemoryQuerier::accessRawData(std::string_view key) const {
    if (!isConnected()) {
        return nullptr;
    }

    const DataBlockDescriptor* desc = getDescriptor(key);
    if (!desc) {
        // 키가 없으면 nullptr 반환
        return nullptr;
    }
    
    auto header = get_buffer_header(_shm_base_ptr, desc);

    int idx = header->ready_index.load(std::memory_order_acquire);
    if (idx < 0) {
        return nullptr;
    }

    return get_data_buffer(_shm_base_ptr, desc, idx);
}

const TaskStats* SharedMemoryQuerier::getTaskStatsArray() const {
    if (!isConnected()) return nullptr;
    return (const TaskStats*)((const char*)_shm_base_ptr + _header->task_stats_array_offset);
}

size_t SharedMemoryQuerier::getTaskStatsCount() const {
    return isConnected() ? _header->task_stats_count : 0;
}

std::vector<TaskGraphNodeInfo> SharedMemoryQuerier::getGraphNodes() const {
    if (!isConnected()) return {};
    const char* base = static_cast<const char*>(_shm_base_ptr);
    const TaskGraphNodeInfo* nodes_ptr = (const TaskGraphNodeInfo*)(base + _header->graph_nodes_array_offset);
    return std::vector<TaskGraphNodeInfo>(nodes_ptr, nodes_ptr + _header->graph_node_count);
}

std::vector<GraphEdge> SharedMemoryQuerier::getGraphEdges() const {
    if (!isConnected()) return {};
    const char* base = static_cast<const char*>(_shm_base_ptr);
    const GraphEdge* edges_ptr = (const GraphEdge*)(base + _header->graph_edges_array_offset);
    return std::vector<GraphEdge>(edges_ptr, edges_ptr + _header->graph_edge_count);
}

std::vector<DataFlowInfo> SharedMemoryQuerier::getDataFlows() const {
    if (!isConnected()) return {};
    const char* base = static_cast<const char*>(_shm_base_ptr);
    const DataFlowInfo* flows_ptr = (const DataFlowInfo*)(base + _header->data_flows_array_offset);
    return std::vector<DataFlowInfo>(flows_ptr, flows_ptr + _header->data_flow_count);
}


const TimelineStats* SharedMemoryQuerier::getTimelineStatsArray() const {
    if (!isConnected()) return nullptr;
    return (const TimelineStats*)((const char*)_shm_base_ptr + _header->timeline_stats_array_offset);
}

size_t SharedMemoryQuerier::getTimelineStatsCount() const {
    return isConnected() ? _header->timeline_stats_count : 0;
}

const PoolStats* SharedMemoryQuerier::getPoolStatsArray() const {
    if (!isConnected()) return nullptr;
    return (const PoolStats*)((const char*)_shm_base_ptr + _header->pool_stats_array_offset);
}

size_t SharedMemoryQuerier::getPoolStatsCount() const {
    return isConnected() ? _header->pool_stats_count : 0;
}

std::vector<LogEntry> SharedMemoryQuerier::getAllLogEntries() const {
    std::vector<LogEntry> all_entries;
    
    // 헤더나 로그 버퍼 포인터가 유효하지 않으면 빈 벡터를 반환
    if (!_header) { // log_buffer가 포인터라고 가정. 직접 포함이면 _header.log_buffer
        return all_entries;
    }

    const SharedLogBuffer* log_buffer = &(_header->log_buffer);

    // 1. head와 tail의 현재 위치를 원자적으로 읽어와 스냅샷을 만듦
    //    head는 계속 변할 수 있으므로, acquire 시맨틱으로 최신 값을 보장받는 것이 좋음
    uint64_t head = log_buffer->head.load(std::memory_order_acquire);
    // tail은 head에 의해 밀려날 수 있으므로, 역시 acquire로 읽는 것이 안전
    uint64_t tail = log_buffer->tail.load(std::memory_order_acquire);

    // head가 tail보다 뒤쳐져 있다면 (초기 상태 또는 오류), 읽을 내용이 없음
    if (head <= tail) {
        return all_entries;
    }
    
    // 2. 읽어야 할 로그의 총 개수를 계산
    //    버퍼 크기(1024)를 초과할 수 없음
    uint64_t total_logs_in_buffer = head - tail;
    size_t num_entries_to_read = static_cast<size_t>(std::min((uint64_t)1024, total_logs_in_buffer));

    // 미리 메모리를 할당하여 push_back 성능 최적화
    all_entries.reserve(num_entries_to_read);

    // 3. tail부터 시작하여 head 직전까지 순서대로 로그를 복사
    for (size_t i = 0; i < num_entries_to_read; ++i) {
        // 읽어야 할 로그의 실제 시퀀스 번호
        uint64_t entry_sequence = tail + i;
        // 시퀀스 번호를 사용하여 배열의 실제 인덱스를 계산
        size_t array_index = entry_sequence % 1024;

        // 공유 메모리에서 클라이언트의 로컬 벡터로 데이터 복사
        all_entries.push_back(log_buffer->entries[array_index]);
    }
    
    return all_entries;
}

std::vector<ParameterInfo> SharedMemoryQuerier::getAllParameterInfos() const {
    if (!isConnected()) return {};
    const char* base = static_cast<const char*>(_shm_base_ptr);
    const ParameterInfo* info_ptr = (const ParameterInfo*)(base + _header->param_info_array_offset);
    return std::vector<ParameterInfo>(info_ptr, info_ptr + _header->param_info_count);
}