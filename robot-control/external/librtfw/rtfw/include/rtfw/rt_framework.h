#pragma once

#include <vector>
#include <memory>
#include <iostream>
#include <map>
#include <set>
#include <thread>
#include <functional>
#include <atomic>
#include <string>
#include <typeindex>
#include <any>

#include "rtfw_common/blackbox.h"
#include "task.h"
#include "shm_allocator.h"
#include "shm_context.h"
#include "blocking_queue.h"
#include "parameter_manager.h"
#include "rtfw/shm_ringbuffer_sink.h"

namespace rtfw {
    enum class Mode { LIVE, RECORDING, SIMULATION, TRACE };
    enum class RealtimeLevel {
        SOFT, // 일반 스케줄러 사용 (best-effort)
        HARD  // 실시간 스케줄러(SCHED_FIFO) 사용 (requires privilege)
    };
    
    // rtfw::common::LogLevel의 별칭
    using LogLevel = common::LogLevel;

    struct ThreadPoolConfig {
        int num_common_threads = 0; // 0이면 하드웨어 스레드 수로 자동 설정
        int num_non_rt_threads = 0;
        // 특정 CPU 코어에 할당할 스레드 수
        std::map<int, int> dedicated_core_threads;
    };

    struct BlackboxConfig {
        std::unique_ptr<blackbox::IBlackbox> record_backend = nullptr;
        std::unique_ptr<blackbox::IBlackbox> replay_backend = nullptr;
    };

    // 프레임워크 전체 설정을 위한 객체
    struct FrameworkConfig {
        Mode mode = Mode::LIVE;
        RealtimeLevel realtime_level = RealtimeLevel::HARD;
        ThreadPoolConfig threads;
        BlackboxConfig blackbox;
        LogLevel log_level = LogLevel::INFO;  // 초기 로그 레벨

        // [신규] 파라미터 파일 경로
        std::string parameter_file_path = "";//"parameters.yaml";
    };

}; // namespace rtfw


namespace rtfw::common {
    struct SharedMemoryHeader;
}

namespace rtfw::internal {
    class Timeline;
};

namespace rtfw {
    class RealTimeFramework {
    public:
        RealTimeFramework();
        ~RealTimeFramework();

        // Immediate control helpers for in-app tests / tooling
        bool startRecord(const std::string& filename, uint64_t start_tick = 0, common::SharedMemoryHeader* header = nullptr);
        bool startReplay(const std::string& filename, common::SharedMemoryHeader* header = nullptr);
        bool startTrace(const std::string& record_filename, const std::string& replay_filename);
        void stopRecord(common::SharedMemoryHeader* header = nullptr);
        void stopReplay(common::SharedMemoryHeader* header = nullptr);

        // 복사 및 이동 금지
        RealTimeFramework(const RealTimeFramework&) = delete;
        RealTimeFramework& operator=(const RealTimeFramework&) = delete;

        // --- Public API ---
        void registerTask(std::unique_ptr<rt::ITask> task, int frequency, int cpu_affinity = -1);
        void registerNonRtTask(std::unique_ptr<rt::ITask> task, int frequency);

        void initialize(FrameworkConfig&& config);
        void start();
        void stop();
        void printStats() const;

    private:
        friend class internal::Timeline;

        // --- 초기화 헬퍼 함수 ---
        void registerTaskInternal(std::unique_ptr<rt::ITask> task, int frequency, int cpu_affinity, bool is_non_rt);
        void collectAndAnalyzeTasks(
            std::vector<common::DataBlockDescriptor>& descriptors,
            std::vector<common::TaskGraphNodeInfo>& graph_nodes,
            std::vector<common::GraphEdge>& graph_edges,
            std::vector<common::DataFlowInfo>& out_data_flows
        );
        void wireTaskProxies();
        void updateBlackboxSlots(bool clear_slots = false);
        
        // 스레드 정리 (internal use only)
        void join();
        
        // --- 실행 루프 관련 헬퍼 함수 ---
        void runSchedulerLoop();
        void commonWorkerLoop();
        void dedicatedWorkerLoop(int core_id);
        void nonRtWorkerLoop();
        void executeAndPropagateTask(rt::ITask* task);

        // --- 사이클 탐지 유틸리티 ---
        bool detectCycleUtil(
            uint32_t task_id,
            const std::map<uint32_t, std::vector<uint32_t>>& adj,
            std::set<uint32_t>& visited,
            std::vector<uint32_t>& path_stack);

        // --- Timeline 제어 ---
        size_t getTotalTaskCount() const { return _all_tasks_by_id.size(); };
        int getInitialInternalDependencyCount(uint32_t task_id) const { return _initial_dependency_counts[task_id]; };
        const std::vector<uint64_t>& getExternalDependencyKeys(uint32_t task_id) const { return _external_dependency_keys[task_id]; };
        const std::vector<rt::ITask*>& getInternalDependents(uint32_t task_id) const { return _task_dependents[task_id]; };
        void enqueueTask(rt::ITask* task);

        // -- 통계 관련 --
        common::TaskStats* getTaskStatsArray() {
            return _shmContext.getTaskStatsArray();
        }
        common::TaskStats* getTaskStats(uint32_t task_id) {
            // 경계 검사는 디버그 빌드에서만 추가하는 것을 고려해볼 수 있음
            // assert(task_id < _shmContext.getHeader()->task_stats_count);
            return &(_shmContext.getTaskStatsArray()[task_id]);
        }
        common::TimelineStats* getTimelineStats(int frequency) {
            auto it = _frequency_to_timeline_stats_idx.find(frequency);
            if (it != _frequency_to_timeline_stats_idx.end()) {
                int index = it->second;
                return &(_shmContext.getTimelineStatsArray()[index]);
            }
            return nullptr;
        }
        common::PoolStats* getPoolStats(int pool_index) {
            return &(_shmContext.getPoolStatsArray()[pool_index]);
        }
        common::PoolStats* getPoolStatsForCommonRT();
        common::PoolStats* getPoolStatsForNonRT();
        common::PoolStats* getPoolStatsForDedicatedCore(int core_id);

        // --- 멤버 변수 (재구성됨) ---

        // 1. 공유 메모리 관리
        internal::SharedMemoryAllocator _shmAllocator;
        internal::SharedMemoryContext _shmContext;

        // 2. Task 등록 및 관리
        struct TaskInfo {
            std::unique_ptr<rt::ITask> task;
            int frequency;
            int affinity;
            bool is_non_rt;
        };
        std::vector<TaskInfo> _registered_tasks_info;
        std::vector<rt::ITask*> _all_tasks_by_id;
        std::map<int, std::vector<rt::ITask*>> _task_groups_by_freq;
        std::map<std::string, uint32_t> _write_key_to_task_id;

        // 3. 스케줄링 및 실행 관련
        uint64_t _base_frequency;
        std::atomic<bool> _stop_token;
        std::atomic<bool> _paused{false};
        std::atomic<int> _single_step_remaining{0};
        std::atomic<bool> _joined{false};  // join 수행 여부를 추적
        FrameworkConfig _config;
        Mode _mode = Mode::LIVE;
        RealtimeLevel _realtime_level = RealtimeLevel::HARD;
        std::unique_ptr<blackbox::IBlackbox> _record_backend = nullptr;
        std::unique_ptr<blackbox::IBlackbox> _replay_backend = nullptr;


        // 4. 스레드 풀

         // 1. 실제 통계 데이터 (프레임워크가 소유)
        //    이 데이터는 공유 메모리에 복사될 원본이거나, SHM 포인터를 통해 직접 접근됨
        common::PoolStats _common_rt_pool_stats;
        common::PoolStats _non_rt_pool_stats;
        std::map<int, common::PoolStats> _dedicated_pool_stats; // Key: core_id

        // 2. 통계 기능이 내장된 작업 큐
        std::unique_ptr<internal::BlockingQueue<rt::ITask*>> _common_task_queue;
        std::unique_ptr<internal::BlockingQueue<rt::ITask*>> _non_rt_task_queue; // Non-RT용 큐 추가
        std::map<int, std::unique_ptr<internal::BlockingQueue<rt::ITask*>>> _dedicated_task_queues;
        
        int _num_common_worker_threads;
        std::vector<std::thread> _common_worker_threads;
        
        std::map<int, int> _dedicated_core_thread_counts;
        std::map<int, std::vector<std::thread>> _dedicated_worker_threads;

        int _num_non_rt_worker_threads;
        std::vector<std::thread> _non_rt_worker_threads;

        std::atomic<int> _rt_workers_ready_count;
        std::atomic<int> _rt_workers_pending_count;

        // 파라미터 관리
        std::unique_ptr<internal::ParameterManager> _paramManager;

        // Timeline 관련
        std::vector<std::vector<rt::ITask*>> _task_dependents;  // task_id가 끝났을 때 깨워야 할 태스크 목록
        std::vector<int> _initial_dependency_counts;    // task_id를 실행하기 위해 먼저 끝나야 하는 태스크 수
        std::vector<std::vector<uint64_t>> _external_dependency_keys;   // task_id가 스냅샷을 만들어야 할 데이터 키 목록
        std::map<int, std::unique_ptr<internal::Timeline>> _timelines;

        // State 관련
        std::vector<uint8_t> _state_buffer; // 모든 태스크의 상태를 담는 연속 메모리
        std::vector<size_t> _task_state_offsets;
        size_t _total_state_size = 0;

        // 통계 슬롯의 인덱스를 관리하기 위한 맵
        std::map<int, int> _frequency_to_timeline_stats_idx; // Key: frequency, Value: stats_array_index

        // 풀의 종류를 나타내는 enum (선택적이지만 권장)
        enum class PoolType { CommonRT, NonRT, Dedicated };
        std::map<PoolType, int> _pool_type_to_stats_idx;
        std::map<int, int> _dedicated_core_to_stats_idx; // ey: core_id, Value: stats_array_index

        mutable std::chrono::time_point<std::chrono::high_resolution_clock> _last_stats_print_time;
        mutable std::map<int, long long> _last_pool_busy_ns; // Key: pool_stats_index

        void handleAction(common::SharedMemoryHeader* header, uint64_t current_tick);
        std::vector<uint8_t> _checkpoint_buffer; // 스냅샷 임시 보관용
        bool _record_start_pending = false;
        uint64_t _record_start_request_tick = 0;
        std::string _record_start_pending_filename;

        bool isWeakDependency(const rt::ITask* reader_task, const rt::ITask::ReadRequest& read_req) const;
        std::vector<uint8_t> buildCheckpointV2(uint64_t checkpoint_tick) const;
        void restoreCheckpointV2(const std::vector<uint8_t>& checkpoint, common::SharedMemoryHeader* header);
    };
};