#include "rtfw/shm_context.h"
#include <numeric>
#include <cstring>
#include <algorithm> // for std::max

using namespace rtfw::internal;
using namespace rtfw::common;


SharedMemoryContext::SharedMemoryContext()
    : _shm_base_ptr(nullptr), _header(nullptr) {}

size_t SharedMemoryContext::calculateLayoutSize(
    const std::vector<DataBlockDescriptor>& descriptors,
    size_t task_count,
    size_t timeline_stats_count,
    size_t pool_stats_count,
    const std::vector<TaskGraphNodeInfo>& graph_nodes,
    const std::vector<GraphEdge>& graph_edges,
    const std::vector<DataFlowInfo>& data_flows,
    const std::vector<ParameterInfo>& param_infos) const 
{
    size_t total_size = sizeof(SharedMemoryHeader);

    total_size = align_offset(total_size, alignof(DataBlockDescriptor));
    total_size += sizeof(DataBlockDescriptor) * descriptors.size();

    total_size = align_offset(total_size, alignof(TaskStats));
    total_size += sizeof(TaskStats) * task_count;

    total_size = align_offset(total_size, alignof(TaskGraphNodeInfo));
    total_size += sizeof(TaskGraphNodeInfo) * graph_nodes.size();

    total_size = align_offset(total_size, alignof(GraphEdge));
    total_size += sizeof(GraphEdge) * graph_edges.size();

    total_size = align_offset(total_size, alignof(DataFlowInfo));
    total_size += sizeof(DataFlowInfo) * data_flows.size();

    total_size = align_offset(total_size, alignof(TimelineStats));
    total_size += sizeof(TimelineStats) * timeline_stats_count;
    
    total_size = align_offset(total_size, alignof(PoolStats));
    total_size += sizeof(PoolStats) * pool_stats_count;

    // 1. ParameterBlockDescriptor 자체의 크기
    total_size = align_offset(total_size, alignof(ParameterBlockDescriptor));
    total_size += sizeof(ParameterBlockDescriptor);

    // 2. ParameterInfo 배열의 크기
    total_size = align_offset(total_size, alignof(ParameterInfo));
    total_size += sizeof(ParameterInfo) * param_infos.size();

    // 3. 실제 파라미터 데이터 블록의 크기 계산
    size_t single_buffer_data_size = 0;
    for (const auto& p_info : param_infos) {
        single_buffer_data_size = align_offset(single_buffer_data_size, p_info.data_alignment);
        single_buffer_data_size += p_info.data_size;
    }

    // ParameterBlockHeader + (데이터 버퍼 * 2)
    size_t param_block_total_size = sizeof(ParameterBlockHeader);
    param_block_total_size = align_offset(param_block_total_size, 8); // 데이터 시작 위치 정렬
    param_block_total_size += single_buffer_data_size * 2;
    
    total_size = align_offset(total_size, 8); // 파라미터 블록 시작 위치 정렬
    total_size += param_block_total_size;


    total_size = align_offset(total_size, 8); // 데이터 블록 영역 정렬
    size_t current_data_offset = total_size;

    // DataBlock 자체의 오프셋은 desc에 저장되므로, 여기서는 전체 크기만 계산
    for (const auto& desc_template : descriptors) {
        // 1. 블록 시작 위치 정렬
        size_t block_alignment = std::max({desc_template.data_alignment, alignof(BufferHeader), alignof(BufferState)});
        current_data_offset = align_offset(current_data_offset, block_alignment);

        size_t block_start_offset = current_data_offset;
        size_t offset_within_block = 0;

        // 2. BufferHeader 크기 추가
        offset_within_block += sizeof(BufferHeader);

        // 3. BufferState 배열 위치 정렬 및 크기 추가
        offset_within_block = align_offset(offset_within_block, alignof(BufferState));
        offset_within_block += sizeof(BufferState) * desc_template.block_count;

        // 4. 데이터 영역 위치 정렬 및 크기 추가
        offset_within_block = align_offset(offset_within_block, desc_template.data_alignment);
        offset_within_block += desc_template.data_stride * desc_template.block_count;

        // 5. 현재 오프셋에 이 블록의 총 크기를 더함
        current_data_offset = block_start_offset + offset_within_block;
    }
    return current_data_offset;
}

void SharedMemoryContext::buildAndPopulate(
    void* base_ptr,
    const std::vector<DataBlockDescriptor>& descriptors,
    size_t task_count,
    size_t timeline_stats_count,
    size_t pool_stats_count,
    const std::vector<TaskGraphNodeInfo>& graph_nodes,
    const std::vector<GraphEdge>& graph_edges,
    const std::vector<DataFlowInfo>& data_flows,
    std::vector<ParameterInfo>& param_infos)
{
    _shm_base_ptr = base_ptr;
    _header = static_cast<SharedMemoryHeader*>(_shm_base_ptr);
    new (_header) SharedMemoryHeader(); // placement new로 헤더 초기화

    char* base_char_ptr = static_cast<char*>(_shm_base_ptr);
    size_t current_offset = sizeof(SharedMemoryHeader);

    // 1. 메타데이터 기록 및 헤더 채우기
    current_offset = align_offset(current_offset, alignof(DataBlockDescriptor));
    _header->descriptor_array_offset = current_offset;
    _header->descriptor_count = descriptors.size();
    memcpy(base_char_ptr + current_offset, descriptors.data(), sizeof(DataBlockDescriptor) * descriptors.size());
    current_offset += sizeof(DataBlockDescriptor) * descriptors.size();

    current_offset = align_offset(current_offset, alignof(TaskStats));
    _header->task_stats_array_offset = current_offset;
    _header->task_stats_count = task_count;
    TaskStats* stats_array = (TaskStats*)(base_char_ptr + current_offset);
    for(size_t i = 0; i < task_count; ++i) { new (&stats_array[i]) TaskStats(); }
    current_offset += sizeof(TaskStats) * task_count;

    current_offset = align_offset(current_offset, alignof(TaskGraphNodeInfo));
    _header->graph_nodes_array_offset = current_offset;
    _header->graph_node_count = graph_nodes.size();
    memcpy(base_char_ptr + current_offset, graph_nodes.data(), sizeof(TaskGraphNodeInfo) * graph_nodes.size());
    current_offset += sizeof(TaskGraphNodeInfo) * graph_nodes.size();

    current_offset = align_offset(current_offset, alignof(GraphEdge));
    _header->graph_edges_array_offset = current_offset;
    _header->graph_edge_count = graph_edges.size();
    memcpy(base_char_ptr + current_offset, graph_edges.data(), sizeof(GraphEdge) * graph_edges.size());
    current_offset += sizeof(GraphEdge) * graph_edges.size();
    
    current_offset = align_offset(current_offset, alignof(DataFlowInfo));
    _header->data_flows_array_offset = current_offset;
    _header->data_flow_count = data_flows.size();
    memcpy(base_char_ptr + current_offset, data_flows.data(), sizeof(DataFlowInfo) * data_flows.size());
    current_offset += sizeof(DataFlowInfo) * data_flows.size();

    current_offset = align_offset(current_offset, alignof(TimelineStats));
    _header->timeline_stats_array_offset = current_offset;
    _header->timeline_stats_count = timeline_stats_count;
    TimelineStats* timeline_stats_array = (TimelineStats*)(base_char_ptr + current_offset);
    for(size_t i = 0; i < timeline_stats_count; ++i) {
        new (&timeline_stats_array[i]) TimelineStats();
    }
    current_offset += sizeof(TimelineStats) * timeline_stats_count;

    current_offset = align_offset(current_offset, alignof(PoolStats));
    _header->pool_stats_array_offset = current_offset;
    _header->pool_stats_count = pool_stats_count;
    PoolStats* pool_stats_array = (PoolStats*)(base_char_ptr + current_offset);
    for(size_t i = 0; i < pool_stats_count; ++i) {
        new (&pool_stats_array[i]) PoolStats();
    }
    current_offset += sizeof(PoolStats) * pool_stats_count;



    // --- [신규] 파라미터 영역 빌드 ---
    // 1. ParameterBlockDescriptor 위치 설정
    current_offset = align_offset(current_offset, alignof(ParameterBlockDescriptor));
    _header->param_block_descriptor_offset = current_offset;
    auto* p_block_desc = (ParameterBlockDescriptor*)(base_char_ptr + current_offset);
    new (p_block_desc) ParameterBlockDescriptor(); // placement new
    current_offset += sizeof(ParameterBlockDescriptor);

    // 2. ParameterInfo 배열 위치 설정 및 기록
    current_offset = align_offset(current_offset, alignof(ParameterInfo));
    _header->param_info_array_offset = current_offset;
    _header->param_info_count = param_infos.size();
    
    // offset_in_buffer 채우기
    size_t current_param_offset_in_buffer = 0;
    for(auto& p_info : param_infos) {
        current_param_offset_in_buffer = align_offset(current_param_offset_in_buffer, p_info.data_alignment);
        p_info.offset_in_buffer = current_param_offset_in_buffer;
        current_param_offset_in_buffer += p_info.data_size;
    }
    const size_t single_buffer_size = current_param_offset_in_buffer;
    p_block_desc->buffer_size = single_buffer_size;

    // 채워진 정보 배열을 공유 메모리에 복사
    memcpy(base_char_ptr + _header->param_info_array_offset, param_infos.data(), sizeof(ParameterInfo) * param_infos.size());
    current_offset += sizeof(ParameterInfo) * param_infos.size();
    
    // 3. 실제 파라미터 데이터 블록 위치 설정 및 초기화
    current_offset = align_offset(current_offset, 8);
    
    // Header 위치 설정 및 초기화
    p_block_desc->block_header_offset = current_offset;
    auto* p_block_header = (ParameterBlockHeader*)(base_char_ptr + current_offset);
    new (p_block_header) ParameterBlockHeader();
    current_offset += sizeof(ParameterBlockHeader);
    current_offset = align_offset(current_offset, 8); // 데이터 정렬

    // Buffer 0 위치 설정
    p_block_desc->buffer_0_data_offset = current_offset;
    current_offset += single_buffer_size;

    // Buffer 1 위치 설정
    p_block_desc->buffer_1_data_offset = current_offset;
    current_offset += single_buffer_size;


    
    // 2. 데이터 블록 영역 초기화
    current_offset = align_offset(current_offset, 8);
    _header->data_blocks_area_offset = current_offset;

    DataBlockDescriptor* stored_descriptors = (DataBlockDescriptor*)(base_char_ptr + _header->descriptor_array_offset);
    for (size_t i = 0; i < _header->descriptor_count; ++i) {
        auto& desc = stored_descriptors[i];
        
        // 1. 이 블록에 필요한 최대 정렬 값을 계산
        size_t block_alignment = std::max({desc.data_alignment, alignof(BufferHeader), alignof(BufferState)});
        
        // 2. 블록의 시작 위치를 정렬
        current_offset = align_offset(current_offset, block_alignment);
        size_t block_start_offset = current_offset;
        
        // 3. BufferHeader 위치 설정 및 초기화
        desc.buffer_header_offset = block_start_offset;
        BufferHeader* block_header = (BufferHeader*)(base_char_ptr + desc.buffer_header_offset);
        new (block_header) BufferHeader();

        // 4. BufferState 배열 위치 설정 및 초기화
        size_t states_offset = block_start_offset + sizeof(BufferHeader);
        states_offset = align_offset(states_offset, alignof(BufferState));
        desc.buffer_states_offset = states_offset;
        BufferState* states_array = (BufferState*)(base_char_ptr + desc.buffer_states_offset);
        for(size_t j = 0; j < desc.block_count; ++j) {
            new (&states_array[j]) BufferState();
        }

        // 5. 데이터 영역 위치 설정
        size_t data_offset = desc.buffer_states_offset + sizeof(BufferState) * desc.block_count;
        data_offset = align_offset(data_offset, desc.data_alignment);
        desc.data_region_offset = data_offset;
        
        // 6. 이 블록이 차지하는 전체 크기를 계산하여 current_offset을 갱신
        size_t block_total_size = (desc.data_region_offset - block_start_offset) + (desc.data_stride * desc.block_count);
        current_offset = block_start_offset + block_total_size;
    }

    // 3. 캐시 빌드
    buildCache();
}

void SharedMemoryContext::buildCache() {
    if (!isAttached()) return;
    _descriptor_cache.clear();
    char* base = static_cast<char*>(_shm_base_ptr);
    DataBlockDescriptor* descriptors = (DataBlockDescriptor*)(base + _header->descriptor_array_offset);
    for (size_t i = 0; i < _header->descriptor_count; ++i) {
        _descriptor_cache[descriptors[i].key] = &descriptors[i];
    }
}

DataBlockDescriptor* SharedMemoryContext::getDescriptor(std::string_view key) {
    if (!isAttached()) return nullptr;
    auto it = _descriptor_cache.find(key);
    return (it != _descriptor_cache.end()) ? it->second : nullptr;
}
const DataBlockDescriptor* SharedMemoryContext::getDescriptor(std::string_view key) const {
    // const 버전
    if (!isAttached()) return nullptr;
    auto it = _descriptor_cache.find(key);
    return (it != _descriptor_cache.end()) ? it->second : nullptr;
}

TaskStats* SharedMemoryContext::getTaskStatsArray() {
    if (!isAttached()) return nullptr;
    return (TaskStats*)((char*)_shm_base_ptr + _header->task_stats_array_offset);
}
const TaskStats* SharedMemoryContext::getTaskStatsArray() const {
    if (!isAttached()) return nullptr;
    return (const TaskStats*)((const char*)_shm_base_ptr + _header->task_stats_array_offset);
}
size_t SharedMemoryContext::getTaskStatsCount() const {
    return isAttached() ? _header->task_stats_count : 0;
}

DataFlowInfo* SharedMemoryContext::getDataFlowArray() {
    if (!isAttached()) return nullptr;
    return (DataFlowInfo*)((char*)_shm_base_ptr + _header->data_flows_array_offset);
}
const DataFlowInfo* SharedMemoryContext::getDataFlowArray() const {
    if (!isAttached()) return nullptr;
    return (const DataFlowInfo*)((const char*)_shm_base_ptr + _header->data_flows_array_offset);
}
size_t SharedMemoryContext::getDataFlowCount() const {
    return isAttached() ? _header->data_flow_count : 0;
}

TimelineStats* SharedMemoryContext::getTimelineStatsArray() {
    if (!isAttached()) return nullptr;
    return (TimelineStats*)((char*)_shm_base_ptr + _header->timeline_stats_array_offset);
}
const TimelineStats* SharedMemoryContext::getTimelineStatsArray() const {
    if (!isAttached()) return nullptr;
    return (const TimelineStats*)((const char*)_shm_base_ptr + _header->timeline_stats_array_offset);
}
size_t SharedMemoryContext::getTimelineStatsCount() const {
    return isAttached() ? _header->timeline_stats_count : 0;
}

PoolStats* SharedMemoryContext::getPoolStatsArray() {
    if (!isAttached()) return nullptr;
    return (PoolStats*)((char*)_shm_base_ptr + _header->pool_stats_array_offset);
}
const PoolStats* SharedMemoryContext::getPoolStatsArray() const {
    if (!isAttached()) return nullptr;
    return (const PoolStats*)((const char*)_shm_base_ptr + _header->pool_stats_array_offset);
}
size_t SharedMemoryContext::getPoolStatsCount() const {
    return isAttached() ? _header->pool_stats_count : 0;
}
