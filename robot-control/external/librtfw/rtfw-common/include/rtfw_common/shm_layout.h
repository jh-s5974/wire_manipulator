// rtfw-common/shm_layout.h
#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <utility> // for std::pair
#include "log_format.h"


namespace rtfw::common {
    // 공유 메모리 이름
    constexpr const char* SHM_NAME = "/robot_control_shm";

    enum class FrameworkAction : uint32_t {
        NONE = 0,
        // Preferred names: explicit about recording vs tracing
        START_RECORD = 1,
        STOP_RECORD = 2,
        RELOAD_PARAMETERS = 3,
        PAUSE_TICK = 4,
        PLAY_TICK = 5,
        PLAY_ONE_TICK = 10,
        START_REPLAY = 6,
        STOP_REPLAY = 7,
        START_TRACE = 8,
        STOP_TRACE = 9,
        SET_TASK_ENABLED = 11,  // Task on/off 제어
        // (No legacy START_LOGGING/STOP_LOGGING aliases - use START_RECORD/STOP_RECORD)
    };

    // --- 데이터 블록 관련 구조체 ---

    struct alignas(8) BufferHeader {
        std::atomic<int> ready_index{0};
        std::atomic<int> write_index{0};
    };

    struct alignas(8) BufferState {
        std::atomic<uint64_t> write_tick{0};
        std::atomic<int> ref_count{0};
    };

    struct DataBlockDescriptor {
        char key[64];
        uint32_t task_id;
        uint64_t key_hash;
        uint64_t type_hash;
        size_t block_count;
        size_t buffer_header_offset;
        size_t buffer_states_offset;
        size_t data_region_offset;
        size_t data_size;
        size_t data_alignment;
        size_t data_stride;
    };


    // --- 통계 정보 구조체 ---

    struct TaskStats {
        char task_name[64]{};
        std::atomic<uint64_t> last_pushed_to_queue_offset_ns{0};
        std::atomic<uint64_t> last_start_offset_ns{0};
        std::atomic<uint64_t> last_completion_offset_ns{0};
        std::atomic<long long> total_exec_time_ns{0};
        std::atomic<long long> max_exec_time_ns{0};
        std::atomic<long long> total_squared_exec_time_ns{0};
        std::atomic<long long> total_latency_ns{0};
        std::atomic<long long> max_latency_ns{0};
        std::atomic<long long> total_squared_latency_ns{0};
        std::atomic<long long> exec_count{0};
        std::atomic<bool> is_busy{false};
        std::atomic<bool> has_overrun{false};
        std::atomic<long long> stale_write_count{0};  // [NEW] Stale write 감지 횟수
        std::atomic<long long> overrun_recovery_count{0};  // [NEW] Overrun 복구 시도 횟수 (성공)
    };


    // 타임라인별 통계 구조체
    struct alignas(8) TimelineStats {
        int frequency;
        std::atomic<long long> total_busy_ns{0};
        std::atomic<long long> total_squared_busy_ns{0};
        std::atomic<long long> max_busy_ns{0};
        std::atomic<long long> tick_count{0};
        std::atomic<long long> deadline_miss_count{0};
    };

    // 스레드풀별 통계 구조체
    struct alignas(8) PoolStats {
        char name[64];
        int num_workers;
        std::atomic<long long> total_busy_ns{0};
        std::atomic<long long> total_squared_busy_ns{0};
        std::atomic<long long> sample_count{0};
        std::atomic<int> active_worker_count{0};
        std::atomic<long long> busy_start_time_ns{0};
        std::atomic<int> usage_percentage_x100{0}; 
    };
    
    // --- 의존성 그래프 정보 구조체 ---

    struct TaskGraphNodeInfo {
        char task_name[64];
        uint32_t task_id;
        int frequency;
        int affinity;
        bool is_non_rt;
    };

    struct GraphEdge {
        uint32_t writer_task_id;
        uint32_t reader_task_id;
        bool is_weak;
    };

    struct DataFlowInfo {
        uint32_t writer_task_id;
        uint32_t reader_task_id;
        char key[64]; // 이 연결을 만든 데이터 키
        size_t key_hash;
    };

    // --- log 정보 구조체 ---
    struct alignas(8) LogEntry {
        LogLevel level;
        char task_name[64]; // 태스크 또는 컴포넌트 이름을 저장할 충분한 공간
        // 남은 공간을 메시지를 위해 할당
        char message[191];  // 256  - 1 (level) - 64 (name) = 191
    };

    struct SharedLogBuffer {
        alignas(64) std::atomic<uint64_t> head{0};
        alignas(64) std::atomic<uint64_t> tail{0};
        LogEntry entries[1024]; // 예시: 1024개의 로그 항목을 저장
    };

    enum class ShmState : uint32_t {
        UNINITIALIZED = 0x00000000, // 초기 상태 (메모리가 0으로 채워진 상태)
        INITIALIZING  = 0x11111111, // RTFW가 초기화 중인 상태
        RUNNING       = 0x52544657, // 'R' 'T' 'F' 'W', 정상 실행 중 (이것이 매직 넘버 역할)
        SHUTTING_DOWN = 0xDEADBEEF, // RTFW가 정상 종료 절차를 밟는 중
    };

    // 파라미터 시스템 전체 블록의 헤더
    // 이중 버퍼의 상태(버전, 현재 stable)를 관리.
    struct alignas(8) ParameterBlockHeader {
        std::atomic<int> stable_index{0};
        std::atomic<uint64_t> version[2]{{0}, {0}}; // version[0] for buffer 0, version[1] for buffer 1
    };

    // 파라미터 시스템 전체 블록을 가리키는 최상위 디스크립터
    struct ParameterBlockDescriptor {
        uint64_t block_header_offset;     // ParameterBlockHeader의 위치
        uint64_t buffer_0_data_offset;    // 첫 번째 데이터 버퍼의 시작 위치
        uint64_t buffer_1_data_offset;    // 두 번째 데이터 버퍼의 시작 위치
        size_t buffer_size;               // 각 데이터 버퍼의 전체 크기
    };

    // 개별 파라미터의 위치를 찾기 위한 메타데이터.
    struct ParameterInfo {
        char key[64];
        uint64_t key_hash;
        uint64_t type_hash;
        size_t data_size;
        size_t data_alignment;
        size_t offset_in_buffer; // 각 파라미터 버퍼 내에서의 데이터 오프셋
    };

    // --- 최상위 공유 메모리 헤더 ---

    struct SharedMemoryHeader {
        std::atomic<ShmState> shm_state{ShmState::INITIALIZING};
        uint32_t base_frequency;
        uint32_t num_cpu_cores;

        // --- 시스템 전역 상태 및 통계 ---
        std::atomic<uint64_t> framework_tick_count{0};
        std::atomic<uint64_t> current_tick_start_time_ns{0};

        std::atomic<LogLevel> shared_log_level{LogLevel::INFO}; // 기본값은 INFO
        SharedLogBuffer log_buffer; // 헤더에 직접 포함시키거나 오프셋으로 관리

        // --- 동적 제어 인터페이스 ---
        std::atomic<FrameworkAction> requested_action{FrameworkAction::NONE};
        std::atomic<bool> recording_active{false}; // 실제 로깅 수행 여부 플래그
        std::atomic<bool> replaying_active{false}; // 실제 재생 수행 여부 플래그
        // Backwards-compatible single filename field (used historically for record).
        // Keep this for existing tools that write to SHM. New code should prefer
        // `replay_target_filename` when requesting a replay so both operations
        // can be specified concurrently.
        char target_filename[256];                 // 기록될 파일 경로 (record - legacy)
        // New field for specifying replay file path when running trace mode
        // (simultaneous record+replay). This allows controllers to request
        // independent paths for record and replay.
        char replay_target_filename[256];
        
        // --- Task On/Off 제어 인터페이스 ---
        std::atomic<uint32_t> target_task_id{0xFFFFFFFF};  // 제어 대상 task ID (기본값: 무효)
        std::atomic<bool> target_task_enabled{true};       // 활성화 여부
        

        // --- 메타데이터 위치 정보 ---
        // 각 배열의 시작 오프셋과 실제 요소 개수
        
        // 1. 데이터 블록 디스크립터 배열
        size_t descriptor_array_offset;
        size_t descriptor_count;

        // 2. Task 통계 배열
        size_t task_stats_array_offset;
        size_t task_stats_count;

        // 4. 의존성 그래프 노드 배열
        size_t graph_nodes_array_offset;
        size_t graph_node_count;

        // 5. 의존성 그래프 간선 배열
        size_t graph_edges_array_offset;
        size_t graph_edge_count;

        // 6. 데이터 연결 배열
        size_t data_flows_array_offset;
        size_t data_flow_count;

        // 7. 타임라인 통계 배열
        size_t timeline_stats_array_offset;
        size_t timeline_stats_count; // RT 타임라인의 개수

        // 8. 스레드풀 통계 배열
        size_t pool_stats_array_offset;
        size_t pool_stats_count; // Common RT, Non-RT, Dedicated 풀들의 총 개수

        // 파라미터 관련 오프셋들을 단 하나의 디스크립터 오프셋으로 통합
        size_t param_block_descriptor_offset;
        // 개별 파라미터 정보 배열
        size_t param_info_array_offset;
        size_t param_info_count;


        // --- 실제 데이터 영역 시작 위치 ---
        size_t data_blocks_area_offset;
    };
};