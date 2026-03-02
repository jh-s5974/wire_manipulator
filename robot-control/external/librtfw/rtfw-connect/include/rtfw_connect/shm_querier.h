// rtfw-client/shm_querier.h
#pragma once

#include <string>
#include <cstring>
#include <string_view>
#include <vector>
#include <map>
#include <memory>
#include <utility>
#include <optional>

#include "rtfw_common/shm_layout.h"
#include "rtfw_common/shm_utils.h"
#include "external_data_reader.h"


namespace rtfw::connect {
    // 외부 툴이 공유 메모리의 데이터를 안전하게 조회하기 위한 클래스
    class SharedMemoryQuerier {
    public:
        explicit SharedMemoryQuerier(void* shm_base_ptr);
        ~SharedMemoryQuerier() = default;

        // 복사 금지
        SharedMemoryQuerier(const SharedMemoryQuerier&) = delete;
        SharedMemoryQuerier& operator=(const SharedMemoryQuerier&) = delete;

        bool isConnected() const;
        void* getBasePtr() const {return _shm_base_ptr;}

        // --- 읽기 전용(const) 데이터 접근 API ---

        const common::SharedMemoryHeader* getHeader() const;
        const common::DataBlockDescriptor* getDescriptor(std::string_view key) const;
        std::vector<common::DataBlockDescriptor> getAllDescriptors() const;
        
        template<typename T>
        ExternalDataReader<T> getDataReader(std::string_view key) const;
        const void* accessRawData(std::string_view key) const;

        const common::TaskStats* getTaskStatsArray() const;
        size_t getTaskStatsCount() const;
        const common::TimelineStats* getTimelineStatsArray() const;
        size_t getTimelineStatsCount() const;
        const common::PoolStats* getPoolStatsArray() const;
        size_t getPoolStatsCount() const;

        std::vector<common::TaskGraphNodeInfo> getGraphNodes() const;
        std::vector<common::GraphEdge> getGraphEdges() const;
        std::vector<common::DataFlowInfo> getDataFlows() const;
        
        std::vector<common::LogEntry> getAllLogEntries() const;

        std::vector<common::ParameterInfo> getAllParameterInfos() const;
        template<typename T>
        std::optional<T> getParameterValue(std::string_view key) const;

    private:
        void* _shm_base_ptr;
        const common::SharedMemoryHeader* _header;
        // 빠른 조회를 위한 내부 캐시
        std::map<std::string_view, const common::DataBlockDescriptor*> _descriptor_cache;
        std::map<std::string_view, const common::ParameterInfo*> _param_info_cache;

        // 연결 후, 헤더를 읽어 캐시를 생성
        void buildCache();
    };

    // 템플릿 구현
    template<typename T>
    ExternalDataReader<T> SharedMemoryQuerier::getDataReader(std::string_view key) const {
        if (!isConnected()) {
            throw std::runtime_error("Querier is not connected.");
        }
        const common::DataBlockDescriptor* desc = getDescriptor(key);
        if (!desc) {
            throw std::runtime_error("Querier: DataReader key not found: " + std::string(key));
        }
        if (desc->data_size != sizeof(T) && desc->data_size != 0) { // size 0은 Signal 타입일 수 있음
            throw std::runtime_error("Querier: DataReader type mismatch for key: " + std::string(key));
        }
        return ExternalDataReader<T>(_shm_base_ptr, desc);
    }

    template<typename T>
    std::optional<T> SharedMemoryQuerier::getParameterValue(std::string_view key) const {
        if (!isConnected()) return std::nullopt;

        // 1. 키로 ParameterInfo 찾기
        auto it = _param_info_cache.find(key);
        if (it == _param_info_cache.end()) {
            return std::nullopt; // 키 없음
        }
        const common::ParameterInfo* info = it->second;

        // 2. 타입 크기 검증
        if (sizeof(T) != info->data_size) {
            // 타입 불일치. 빈 optional 반환.
            return std::nullopt;
        }

        // 3. Stable 버퍼에서 값 읽기
        auto* p_block_header = common::get_param_block_header(_shm_base_ptr);
        if (!p_block_header) return std::nullopt;

        int stable_idx = p_block_header->stable_index.load(std::memory_order_acquire);
        const char* stable_buffer = common::get_param_data_buffer(const_cast<void*>(_shm_base_ptr), stable_idx);
        if (!stable_buffer) return std::nullopt;
        
        T value;
        memcpy(&value, stable_buffer + info->offset_in_buffer, sizeof(T));
        
        return value;
    };
};