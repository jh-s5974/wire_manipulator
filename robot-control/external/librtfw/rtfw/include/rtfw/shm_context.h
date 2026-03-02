#pragma once

#include <vector>
#include <string>
#include <string_view>
#include <map>
#include <stdexcept>
#include <cstdint>

#include "rtfw_common/shm_layout.h"
#include "rtfw_common/shm_utils.h"


namespace rtfw::internal {
    // 전방 선언
    class SharedMemoryAllocator;

    // 프레임워크 내부에서 공유 메모리를 빌드하고, 읽고 쓰는 모든 권한을 가진 클래스
    class SharedMemoryContext {
    public:
        SharedMemoryContext();
        ~SharedMemoryContext() = default;

        // 복사 금지
        SharedMemoryContext(const SharedMemoryContext&) = delete;
        SharedMemoryContext& operator=(const SharedMemoryContext&) = delete;

        // --- 초기화 API (RealTimeFramework만 호출) ---

        /**
         * @brief 주어진 메타데이터에 필요한 총 메모리 크기를 계산합니다.
         */
        size_t calculateLayoutSize(
            const std::vector<common::DataBlockDescriptor>& descriptors,
            size_t task_count,
            size_t timeline_stats_count,
            size_t pool_stats_count,
            const std::vector<common::TaskGraphNodeInfo>& graph_nodes,
            const std::vector<common::GraphEdge>& graph_edges,
            const std::vector<common::DataFlowInfo>& data_flows,
            const std::vector<common::ParameterInfo>& param_infos
        ) const;

        /**
         * @brief 할당된 메모리에 붙어서, 레이아웃을 구성하고 메타데이터를 기록합니다.
         * @param base_ptr Allocator가 할당한 공유 메모리의 베이스 포인터.
         * @param ... (기록할 메타데이터 목록)
         */
        void buildAndPopulate(
            void* base_ptr,
            const std::vector<common::DataBlockDescriptor>& descriptors,
            size_t task_count,
            size_t timeline_stats_count,
            size_t pool_stats_count,
            const std::vector<common::TaskGraphNodeInfo>& graph_nodes,
            const std::vector<common::GraphEdge>& graph_edges,
            const std::vector<common::DataFlowInfo>& data_flows,
            std::vector<common::ParameterInfo>& param_infos
        );
        
        // --- 내부 접근 API ---

        bool isAttached() const { return _shm_base_ptr != nullptr; }
        void* getBasePtr() const { return _shm_base_ptr; }
        common::SharedMemoryHeader* getHeader() { return _header; }
        const common::SharedMemoryHeader* getHeader() const { return _header; }

        common::DataBlockDescriptor* getDescriptor(std::string_view key);
        const common::DataBlockDescriptor* getDescriptor(std::string_view key) const;

        common::TaskStats* getTaskStatsArray();
        const common::TaskStats* getTaskStatsArray() const;
        size_t getTaskStatsCount() const;

        common::DataFlowInfo* getDataFlowArray();
        const common::DataFlowInfo* getDataFlowArray() const;
        size_t getDataFlowCount() const;

        common::TimelineStats* getTimelineStatsArray();
        const common::TimelineStats* getTimelineStatsArray() const;
        size_t getTimelineStatsCount() const;

        common::PoolStats* getPoolStatsArray();
        const common::PoolStats* getPoolStatsArray() const;
        size_t getPoolStatsCount() const;
        
    private:
        void* _shm_base_ptr;
        common::SharedMemoryHeader* _header;
        
        // 빠른 조회를 위한 내부 캐시
        std::map<std::string_view, common::DataBlockDescriptor*> _descriptor_cache;

        void buildCache();
    };
};