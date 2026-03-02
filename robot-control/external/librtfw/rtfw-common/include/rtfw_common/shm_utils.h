// rtfw-common/shm_utils.h

#pragma once
#include "shm_layout.h"
#include <cstdint>
#include <algorithm> // for std::max


namespace rtfw::common {
    // size_t(오프셋)를 정렬하는 함수
    static inline size_t align_offset(size_t offset, size_t alignment) {
        if (alignment == 0) return offset;
        return (offset + alignment - 1) & ~(alignment - 1);
    }

    // void*(포인터)를 정렬하는 함수
    static inline void* align_ptr(void* ptr, size_t alignment) {
        if (alignment == 0) return ptr;
        uintptr_t int_ptr = reinterpret_cast<uintptr_t>(ptr);
        uintptr_t aligned_ptr = (int_ptr + alignment - 1) & ~(alignment - 1);
        return reinterpret_cast<void*>(aligned_ptr);
    }

    // DataBlockHeader의 시작 주소를 반환
    static inline BufferHeader* get_buffer_header(void* shm_base_ptr, const DataBlockDescriptor* desc) {
        if (!shm_base_ptr || !desc) return nullptr;
        // 이제 desc->offset은 데이터 블록 영역 시작점부터의 상대 오프셋이 아니라,
        // shm_base_ptr로부터의 절대 오프셋을 의미하게 됨 (Context가 설정해줌)
        return reinterpret_cast<BufferHeader*>(static_cast<char*>(shm_base_ptr) + desc->buffer_header_offset);
    }

    static inline BufferState* get_buffer_state(void* shm_base_ptr, const DataBlockDescriptor* desc, int buffer_index) {
        if (!shm_base_ptr || !desc || buffer_index < 0 || buffer_index >= desc->block_count) return nullptr;
        // 이제 desc->offset은 데이터 블록 영역 시작점부터의 상대 오프셋이 아니라,
        // shm_base_ptr로부터의 절대 오프셋을 의미하게 됨 (Context가 설정해줌)
        return reinterpret_cast<BufferState*>(static_cast<char*>(shm_base_ptr) + desc->buffer_states_offset + sizeof(BufferState)*buffer_index);
    }

    // 실제 데이터 버퍼의 주소를 반환
    static inline void* get_data_buffer(void* shm_base_ptr, const DataBlockDescriptor* desc, int buffer_index) {
        if (!shm_base_ptr || !desc || buffer_index < 0 || buffer_index >= desc->block_count) return nullptr;
        
        return (static_cast<char*>(shm_base_ptr) + desc->data_region_offset + desc->data_stride*buffer_index);
    }


    // ParameterBlockDescriptor의 시작 주소를 반환
    static inline ParameterBlockDescriptor* get_param_block_descriptor(void* shm_base_ptr) {
        if (!shm_base_ptr) return nullptr;
        auto* header = static_cast<SharedMemoryHeader*>(shm_base_ptr);
        return reinterpret_cast<ParameterBlockDescriptor*>((char*)shm_base_ptr + header->param_block_descriptor_offset);
    }

    // ParameterBlockHeader의 시작 주소를 반환
    static inline ParameterBlockHeader* get_param_block_header(void* shm_base_ptr) {
        auto* p_block_desc = get_param_block_descriptor(shm_base_ptr);
        if (!p_block_desc) return nullptr;
        return reinterpret_cast<ParameterBlockHeader*>((char*)shm_base_ptr + p_block_desc->block_header_offset);
    }

    // 개별 파라미터 정보(ParameterInfo) 배열의 시작 주소를 반환
    static inline ParameterInfo* get_param_info_array(void* shm_base_ptr) {
        if (!shm_base_ptr) return nullptr;
        auto* header = static_cast<SharedMemoryHeader*>(shm_base_ptr);
        return reinterpret_cast<ParameterInfo*>((char*)shm_base_ptr + header->param_info_array_offset);
    }

    // 특정 파라미터 버퍼(0 또는 1)의 데이터 영역 시작 주소를 반환
    static inline char* get_param_data_buffer(void* shm_base_ptr, int buffer_index) {
        auto* p_block_desc = get_param_block_descriptor(shm_base_ptr);
        if (!p_block_desc || (buffer_index != 0 && buffer_index != 1)) return nullptr;
        
        uint64_t offset = (buffer_index == 0) ? 
            p_block_desc->buffer_0_data_offset : p_block_desc->buffer_1_data_offset;
            
        return static_cast<char*>(shm_base_ptr) + offset;
    }
};