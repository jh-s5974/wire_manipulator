#include "rtfw/rt_framework.h"
#include "rtfw/timeline.h"
#include <algorithm>
#include <numeric>
#include <queue>
#include <chrono>
#include <typeindex>
#include <pthread.h>
#include <sys/mman.h>
#include <iomanip>
#include <memory> // for std::shared_ptr
#include <cstring>
#include <math.h>
#include <cstdint>

#include "spdlog/spdlog.h"
#include "spdlog/async.h"
#include "rtfw/shm_ringbuffer_sink.h"
#include <spdlog/sinks/stdout_color_sinks.h>
#include <mutex>

using namespace rtfw;
using namespace rtfw::internal;
using namespace rtfw::common;
using namespace std::chrono_literals;


namespace rtfw::internal {
    std::shared_ptr<spdlog::logger> async_logger;
};

namespace {
    constexpr uint32_t CHECKPOINT_V2_MAGIC = 0x32504B43U; // "CKP2"
    constexpr uint16_t CHECKPOINT_V2_VERSION = 1;
    constexpr uint32_t CHECKPOINT_ARCHIVE_SNAPSHOT_TASK_ID = 0xFFFFFFFFU;
    constexpr uint32_t CHECKPOINT_RT_BOUNDARY_SNAPSHOT_TASK_ID = 0xFFFFFFFEU;

    struct CheckpointV2Header {
        uint32_t magic;
        uint16_t version;
        uint16_t reserved;
        uint64_t state_size;
        uint64_t global_tick;
        uint32_t timeline_count;
        uint32_t weak_capture_count;
    };

    struct CheckpointTimelineTick {
        int32_t frequency;
        uint32_t reserved;
        uint64_t local_tick;
    };

    struct CheckpointWeakCaptureHeader {
        uint32_t task_id;
        uint32_t reserved;
        uint64_t key_hash;
        uint64_t write_tick;
        uint32_t data_size;
        uint32_t reserved2;
    };
}

RealTimeFramework::RealTimeFramework() : _base_frequency(0), _stop_token(true) {
    
}

RealTimeFramework::~RealTimeFramework() {
    if(!_stop_token.load(std::memory_order_relaxed)) {
        stop();
        join();
    }
    
    // 로거를 먼저 flush하고 안전하게 제거
    if (async_logger) {
        async_logger->info("RT Framework cleanup started.");
        async_logger->flush();
    }
    
    // Task 정리
    while(!_registered_tasks_info.empty()) {
        try {
            _registered_tasks_info.pop_back();
        }
        catch (const std::exception& e) {
            if (async_logger)
                async_logger->error("{}", e.what());
        }
    }
    
    // 로거 정리 (async_logger는 전역이므로 신중하게 처리)
    if (async_logger) {
        try {
            async_logger->flush();
            spdlog::drop("RTFW");
            async_logger.reset();
            spdlog::shutdown();
        }
        catch (...) {
            // 최후의 수단: 조용히 실패
        }
    }
    
    // 이 시점에서 async_logger 참조 금지
    std::cout << "RT Framework Released." << std::endl;
}
void RealTimeFramework::registerTask(std::unique_ptr<rt::ITask> task, int frequency, int cpu_affinity) {
    if (frequency <= 0) throw std::invalid_argument("RT-Task frequency must be positive.");
    registerTaskInternal(std::move(task), frequency, cpu_affinity, /* is_non_rt = */ false);
}

void RealTimeFramework::registerNonRtTask(std::unique_ptr<rt::ITask> task, int frequency) {
    if (frequency <= 0) throw std::invalid_argument("Non-RT-Task frequency must be positive.");
    registerTaskInternal(std::move(task), frequency, -1, /* is_non_rt = */ true);
}

void RealTimeFramework::registerTaskInternal(std::unique_ptr<rt::ITask> task, int frequency, int cpu_affinity, bool is_non_rt) {
    task->_setNonRt(is_non_rt);
    if (is_non_rt) {
        for (auto& writer: task->_data_writers) {
            writer->_stale_write_protect = false;
        }
    }
    task->_setFrequency(frequency);
    _registered_tasks_info.push_back({std::move(task), frequency, cpu_affinity, is_non_rt});
}

// [API 구현] 코어별 스레드 설정을 받는 initialize 함수
void RealTimeFramework::initialize(FrameworkConfig&& config) {
    spdlog::info("Framework v1.2 initialization started...");
    _paramManager = std::make_unique<internal::ParameterManager>();

    _config = std::move(config);
    _mode = _config.mode;
    _realtime_level = _config.realtime_level;
    _record_backend = std::move(_config.blackbox.record_backend);
    _replay_backend = std::move(_config.blackbox.replay_backend);

    // [준비 단계] Task 객체 목록 구성
    spdlog::info("Step 1: Preparing task instances...");

    uint32_t current_id = 0;
    for (auto& info : _registered_tasks_info) {
        rt::ITask* task_ptr = info.task.get();
        task_ptr->_setID(current_id++);
        task_ptr->_setAffinity(info.affinity);
        _all_tasks_by_id.push_back(task_ptr);
    }

    spdlog::info("  - Total {} RT tasks registered.", _all_tasks_by_id.size());
    for (auto& task: _all_tasks_by_id) {
        spdlog::info("  - {}:{}", task->getID(), task->getName());
    }

    int num_cpu_cores = std::thread::hardware_concurrency();
    if (num_cpu_cores <= 0) num_cpu_cores = 1;

    // [분석 단계] 의존성 정보 수집 및 그래프 분석
    spdlog::info("Step 2: Analyzing dependencies and building graph...");
    std::vector<DataBlockDescriptor> descriptors_temp;
    std::vector<TaskGraphNodeInfo> graph_nodes_temp;
    std::vector<GraphEdge> graph_edges_temp;
    std::vector<DataFlowInfo> data_flows_temp;
    collectAndAnalyzeTasks(descriptors_temp, graph_nodes_temp, graph_edges_temp, data_flows_temp);
    for (auto& desc: descriptors_temp) {
        desc.block_count = 2+_task_groups_by_freq.size();
    }
    _paramManager->collectRequests(_all_tasks_by_id);       // 2. 파라미터 요청 수집
    if (_paramManager->prepareLayout(_config.parameter_file_path)) {         // 3. YAML 로드 및 레이아웃 정보 계산
        spdlog::info("Parameter file loaded ({})", _config.parameter_file_path);
    } else {
        spdlog::info("Parameter file skipped");
    }
    
    // [분석 단계 2: 타임라인 및 스레드풀 개수 확정]
    spdlog::info("Step 3: Configuring timelines and thread pools...");

    // 1. 타임라인 통계 개수 및 인덱스 맵 생성
    int timeline_stats_idx_counter = 0;
    for (const auto& [freq, tasks] : _task_groups_by_freq) {
        if (freq > 0) { // RT 타임라인만 통계 대상
            _frequency_to_timeline_stats_idx[freq] = timeline_stats_idx_counter++;
        }
    }
    const size_t timeline_stats_count = _frequency_to_timeline_stats_idx.size();
    spdlog::info("  - {}  RT timelines configured.", timeline_stats_count);

    // 2. 스레드풀 통계 개수 및 인덱스 맵 생성
    // (이 로직은 스레드풀 설정 부분과 통합될 수 있음)
    int pool_stats_idx_counter = 0;
    _pool_type_to_stats_idx[PoolType::CommonRT] = pool_stats_idx_counter++;
    _pool_type_to_stats_idx[PoolType::NonRT] = pool_stats_idx_counter++;
    
    // 전용 코어 풀 개수 파악 (collectAndAnalyzeTasks 이후에 affinity 정보가 확정됨)
    std::set<int> dedicated_cores;
    for (const auto& task : _all_tasks_by_id) {
        if (task->getAffinity() >= 0) {
            dedicated_cores.insert(task->getAffinity());
        }
    }
    for (int core_id : dedicated_cores) {
        _dedicated_core_to_stats_idx[core_id] = pool_stats_idx_counter++;
    }
    const size_t pool_stats_count = pool_stats_idx_counter;
    spdlog::info("  - {} thread pools configured.", pool_stats_count);

    // Task의 상태 할당
    _total_state_size = 0;
    _task_state_offsets.assign(_all_tasks_by_id.size(), 0);

    for (size_t i = 0; i < _all_tasks_by_id.size(); ++i) {
        size_t s = _all_tasks_by_id[i]->getStateSize();
        if (s > 0) {
            _total_state_size = (_total_state_size + 7) & ~7; // 8-byte alignment
            _task_state_offsets[i] = _total_state_size;
            _total_state_size += s;
        }
    }

    if (_total_state_size > 0) {
        _state_buffer.resize(_total_state_size, 0); // 프레임워크 내부 힙에 할당
        _checkpoint_buffer.resize(_total_state_size, 0);
        spdlog::info("Internal State Buffer allocated: {} bytes", _total_state_size);
    }

    // [구축 단계] 공유 메모리 생성 및 기록
    spdlog::info("Step 4: Building and populating shared memory...");
    _shmAllocator.cleanup(); 
    size_t total_size = _shmContext.calculateLayoutSize(
        descriptors_temp, 
        _all_tasks_by_id.size(), 
        timeline_stats_count, 
        pool_stats_count, 
        graph_nodes_temp, 
        graph_edges_temp, 
        data_flows_temp,
        _paramManager->getParamInfos());
    void* base_ptr = _shmAllocator.allocate(SHM_NAME, total_size);
    _shmContext.buildAndPopulate(
        base_ptr, 
        descriptors_temp, 
        _all_tasks_by_id.size(), 
        timeline_stats_count, 
        pool_stats_count, 
        graph_nodes_temp, 
        graph_edges_temp, 
        data_flows_temp,
        _paramManager->getParamInfos());
    if (_shmContext.isAttached()) {
        _shmContext.getHeader()->base_frequency = _base_frequency;
        
        // Initialize task names in TaskStats
        TaskStats* stats_array = _shmContext.getTaskStatsArray();
        for (size_t i = 0; i < _all_tasks_by_id.size(); ++i) {
            strncpy(stats_array[i].task_name, _all_tasks_by_id[i]->getName(), 
                    sizeof(stats_array[i].task_name) - 1);
        }
    }
    _paramManager->initializeFromShm(_shmContext);      // 6. SHM 포인터 연결
    spdlog::info("  - Shared memory populated. Total size: {} bytes.", total_size);

    SharedLogBuffer* log_buffer_ptr = &(_shmContext.getHeader()->log_buffer);
    // // 2. SHM 버퍼 포인터로 싱크를 안전하게 생성

    auto shm_sink = std::make_shared<internal::ShmRingbufferSink>(log_buffer_ptr);
    auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    std::vector<spdlog::sink_ptr> sinks { shm_sink, console_sink };
    // // 3. 비동기 로거 생성 및 기본 로거로 설정
    spdlog::init_thread_pool(8192, 1);
    
    async_logger = std::make_shared<spdlog::async_logger>("RTFW", 
        sinks.begin(), sinks.end(),
        // shm_sink, // 이제 move를 해도 안전함 (이후에 sink를 사용하지 않음)
        // console_sink,
        spdlog::thread_pool(), 
        spdlog::async_overflow_policy::overrun_oldest);
    spdlog::register_logger(async_logger);
    
    // 로그 레벨 설정 (config 기준으로 덮어쓰기)
    auto spdlog_level = static_cast<spdlog::level::level_enum>(_config.log_level);
    async_logger->set_level(spdlog_level);
    _shmContext.getHeader()->shared_log_level.store(_config.log_level, std::memory_order_release);
    
    // // 이 시점 이후의 모든 로그는 SHM에 정상적으로 기록됨
    async_logger->info("Logging system activated and attached to SHM.");
        



    // 타임라인 객체 생성 (공유 메모리 생성 후)
    for (const auto& [freq, tasks] : _task_groups_by_freq) {
        if (freq > 0) {
            auto timeline = std::make_unique<internal::Timeline>(freq, _base_frequency, tasks, this);
            timeline->initialize();
            _timelines[freq] = std::move(timeline);
        }
    }
    async_logger->info(" - Timeline created.");


    // [연결 단계]
    wireTaskProxies();
    _paramManager->writeInitialValues();
    _paramManager->swapBuffers();
    async_logger->info(" - Task Proxies wired.");

    // --- check unresolved data flow ---
    DataFlowInfo* data_flows = _shmContext.getDataFlowArray();
    for (auto i=0; i<_shmContext.getDataFlowCount(); i++) {
        auto& data_flow = data_flows[i];
        if (data_flow.reader_task_id == -1) {
            auto& task = _all_tasks_by_id[data_flow.writer_task_id];
            async_logger->info("  * {} >> '{}' not wired", task->getName(), data_flow.key);
        }

        if (data_flow.writer_task_id == -1) {
            auto& task = _all_tasks_by_id[data_flow.reader_task_id];
            async_logger->info("  * {} << '{}' not wired", task->getName(), data_flow.key);
        }
    }
    
    // --- 스레드 풀 설정 ---
    async_logger->info("Step 5: Configuring thread pools and queues...");
    
    // 1. Common RT 큐 생성 및 통계 연결
    _common_task_queue = std::make_unique<internal::BlockingQueue<rt::ITask*>>();

    // 2. Non-RT 큐 생성 및 통계 연결
    _non_rt_task_queue = std::make_unique<internal::BlockingQueue<rt::ITask*>>();

    // 3. Dedicated 큐 생성 및 통계 연결
    _dedicated_task_queues.clear();
    for (const auto& [core_id, stats_idx] : _dedicated_core_to_stats_idx) {
        _dedicated_task_queues[core_id] = 
            std::make_unique<internal::BlockingQueue<rt::ITask*>>();
    }
    
    async_logger->info("  - Queues configured with statistics collectors.");
    
    _num_common_worker_threads = (_config.threads.num_common_threads > 0) ? _config.threads.num_common_threads : std::thread::hardware_concurrency();
    if (_num_common_worker_threads == 0) _num_common_worker_threads = 2;

    _num_non_rt_worker_threads = (_config.threads.num_non_rt_threads > 0) ? _config.threads.num_non_rt_threads : 2;
    _dedicated_core_thread_counts.clear();
    for (const auto& task : _all_tasks_by_id) {
        int affinity = task->getAffinity();
        if (affinity >= 0 && _dedicated_core_thread_counts.find(affinity) == _dedicated_core_thread_counts.end()) {
            // _dedicated_task_queues[affinity] = std::make_unique<BlockingQueue<rt::ITask*>>();
            int num_threads = _config.threads.dedicated_core_threads.count(affinity) ? _config.threads.dedicated_core_threads.at(affinity) : 1;
            _dedicated_core_thread_counts[affinity] = num_threads;
        }
    }


    getPoolStatsForCommonRT()->num_workers = _num_common_worker_threads;

    getPoolStatsForNonRT()->num_workers = _config.threads.num_non_rt_threads;

    // Common RT Pool
    {
        auto stats = getPoolStatsForCommonRT();
        stats->num_workers = _num_common_worker_threads;
        std::string name = "Common RT Pool";
        strncpy(stats->name, name.c_str(), sizeof(stats->name) - 1);
        stats->name[sizeof(stats->name) - 1] = '\0';
    }
    // Non-RT Pool
    {
        auto stats = getPoolStatsForNonRT();
        stats->num_workers = _num_non_rt_worker_threads;
        std::string name = "Non-RT Pool";
        strncpy(stats->name, name.c_str(), sizeof(stats->name) - 1);
        stats->name[sizeof(stats->name) - 1] = '\0';

    }
    // Dedicated Pools
    for (const auto& [core_id, index] : _dedicated_core_to_stats_idx) {
        auto stats = getPoolStatsForDedicatedCore(core_id);
        stats->num_workers = _dedicated_core_thread_counts[core_id];
        std::string name = "Dedicated Core " + std::to_string(core_id);
        strncpy(stats->name, name.c_str(), sizeof(stats->name) - 1);
        stats->name[sizeof(stats->name) - 1] = '\0';
    }

    async_logger->info("  - Thread pools configured.");

    for (auto& task: _all_tasks_by_id) {
        for (auto& param: task->_parameter_readers)
            param->capture();
        void* task_state = (task->getStateSize() > 0) ? 
                        &_state_buffer[_task_state_offsets[task->getID()]] : nullptr;
        task->initialize(task_state);
    }   

    async_logger->info("Framework v1.2 initialized successfully.");
}

void RealTimeFramework::collectAndAnalyzeTasks(
    std::vector<DataBlockDescriptor>& out_descriptors,
    std::vector<TaskGraphNodeInfo>& out_graph_nodes,
    std::vector<GraphEdge>& out_graph_edges,
    std::vector<DataFlowInfo>& out_data_flows) 
{
    spdlog::info("  - [1/3] Collecting metadata and verifying type safety...");
    std::map<std::string, DataBlockDescriptor> desc_map_temp;
    std::map<std::string, uint64_t> data_key_to_type;
    std::vector<int> _task_groups_by_freq_reverse_lookup;

    int max_freq = 0;
    _task_groups_by_freq_reverse_lookup.assign(_registered_tasks_info.size(), -1);
    for (const auto& info : _registered_tasks_info) {
        rt::ITask* task = info.task.get();

        TaskGraphNodeInfo node_info;
        strncpy(node_info.task_name, task->getName(), sizeof(node_info.task_name) - 1);
        node_info.task_name[sizeof(node_info.task_name) - 1] = '\0';
        node_info.task_id = task->getID();
        node_info.frequency = info.frequency;
        node_info.affinity = info.affinity;
        node_info.is_non_rt = info.is_non_rt;
        out_graph_nodes.push_back(node_info);

        auto verify_and_collect_requests = [&](const auto& reqs, rt::ITask& task) {
            for (const auto& req : reqs) {
                auto it = data_key_to_type.find(req.key);
                if (it == data_key_to_type.end()) {
                    data_key_to_type[req.key] = req.type_hash;
                } else if (it->second != req.type_hash) {
                    throw std::logic_error("Type mismatch for data key '" + req.key + "' in task '" + task.getName() + "'.");
                }
                if (desc_map_temp.find(req.key) == desc_map_temp.end()) {
                    DataBlockDescriptor desc = {};

                    strncpy(desc.key, req.key.c_str(), sizeof(desc.key) - 1);
                    desc.key[sizeof(desc.key) - 1] = '\0';

                    if (req.key.length() >= sizeof(desc.key)) {
                        spdlog::warn("Data key '{}' is too long (>= {} bytes). It will be truncated to '{}'." , req.key, sizeof(desc.key), desc.key);
                    }

                    desc.key_hash = req.key_hash;
                    desc.type_hash = req.type_hash;
                    desc.task_id = task.getID();
                    desc.data_size = req.size;
                    desc.data_alignment = req.alignment;
                    desc.data_stride = (req.alignment > 0) ? ((req.size + req.alignment - 1) & ~(req.alignment - 1)) : 0;
                    desc_map_temp[req.key] = desc;
                }
            }
        };
        verify_and_collect_requests(task->_read_requests, *task);
        verify_and_collect_requests(task->_write_requests, *task);

        for (const auto& req : task->_write_requests) {
            if (_write_key_to_task_id.count(req.key)) {
                auto writer_id = _write_key_to_task_id[req.key];
                throw std::runtime_error("Write Conflict on key '" + req.key + "' by " + task->getName() + " <--> " + _all_tasks_by_id[writer_id]->getName());
            }
            _write_key_to_task_id[req.key] = task->getID();
        }

        _task_groups_by_freq[info.frequency].push_back(task);
        max_freq = std::max(max_freq, info.frequency);
        _task_groups_by_freq_reverse_lookup[info.task->getID()] = info.frequency;
    }

    std::set<size_t> searched_key;
    for (const auto& info : _registered_tasks_info) {
        for (const auto& req : info.task->_read_requests) {
            DataFlowInfo data_flow = {};
            strncpy(data_flow.key, req.key.c_str(), sizeof(data_flow.key) - 1);
            data_flow.key[sizeof(data_flow.key) - 1] = 0;
            data_flow.key_hash = req.key_hash;
            data_flow.reader_task_id = info.task->getID();
            auto writer_id = _write_key_to_task_id.count(req.key) > 0? _write_key_to_task_id[req.key]: -1;
            data_flow.writer_task_id = writer_id;
            out_data_flows.push_back(data_flow);
            searched_key.insert(req.key_hash);
        }
    }
    for (const auto& info : _registered_tasks_info) {
        for (const auto& req : info.task->_write_requests) {
            if (searched_key.find(req.key_hash) != searched_key.end())
                continue;
            DataFlowInfo data_flow = {};
            strncpy(data_flow.key, req.key.c_str(), sizeof(data_flow.key) - 1);
            data_flow.key[sizeof(data_flow.key) - 1] = 0;
            data_flow.key_hash = req.key_hash;
            data_flow.writer_task_id = info.task->getID();
            data_flow.reader_task_id = -1;
            out_data_flows.push_back(data_flow);
        }
    }

    _base_frequency = max_freq > 0 ? max_freq : 1000; // base_freq가 0이 되지 않도록 방어
    for(const auto& info : _registered_tasks_info) {
        if(info.frequency == 0) {
             _task_groups_by_freq[_base_frequency].push_back(info.task.get());
        }
    }
    spdlog::info("  - Type safety check passed.");
    
    for(const auto& pair : desc_map_temp) {
        out_descriptors.push_back(pair.second);
    }

    spdlog::info("  - [2/3] Building dependency graph...");
    // 1. 중앙 의존성 저장소 벡터들의 크기를 초기화
    const size_t total_tasks = _all_tasks_by_id.size();
    _initial_dependency_counts.assign(total_tasks, 0);
    _external_dependency_keys.resize(total_tasks);
    _task_dependents.resize(total_tasks);

    // 임시 인접 리스트 (사이클 탐지용)
    std::map<uint32_t, std::vector<uint32_t>> adj;
    for (const auto& task : _all_tasks_by_id) {
        adj[task->getID()] = {};
    }

    // 2. 단일 루프를 통해 모든 의존성 관계를 분석
    for (rt::ITask* reader_task : _all_tasks_by_id) {
        const uint32_t reader_id = reader_task->getID();
        const int reader_freq = _task_groups_by_freq_reverse_lookup[reader_id];

        for (const auto& read_req : reader_task->_read_requests) {
            auto writer_it = _write_key_to_task_id.find(read_req.key);
            if (writer_it == _write_key_to_task_id.end()) {
                continue; // 이 데이터를 쓰는 Writer가 없음
            }
            
            rt::ITask* writer_task = _all_tasks_by_id[writer_it->second];
            const uint32_t writer_id = writer_task->getID();
            const int writer_freq = _task_groups_by_freq_reverse_lookup[writer_id];

            // 자기 자신에 대한 의존성 체크
            if (writer_id == reader_id) {
                throw std::runtime_error(std::string(reader_task->getName()) + ": self-loop detected on key '" + read_req.key + "'");
            }
            
            // --- 의존성 타입 결정 로직 ---
            rt::DependencyType final_dep_type = read_req.deptype; // 사용자가 요청한 타입

            // 규칙: Writer가 Non-RT이고 Reader가 RT이면, 무조건 Weak으로 강제
            if (writer_task->isNonRt() && !reader_task->isNonRt()) {
                if (final_dep_type == rt::DependencyType::Strong) {

                    spdlog::info(" Dependency from Non-RT '{}'"
                                " to RT '{}'"
                                " on key '{}'"
                                " was automatically forced to 'Weak' to protect RT deadlines.", 
                                writer_task->getName(), reader_task->getName(), read_req.key);
                }
                final_dep_type = rt::DependencyType::Weak;
            }
            // 1. 데이터 흐름 기록 (Strong, Weak 공통)
            //    - GraphEdge는 이제 '데이터 흐름' 자체를 나타냅니다.
            bool is_weak_flag = (final_dep_type == rt::DependencyType::Weak);
            out_graph_edges.push_back({writer_id, reader_id, is_weak_flag});

            // 2. 실행 순서 결정 (Strong만 해당)
            if (!is_weak_flag) { // final_dep_type == DependencyType::Strong
                // a. 인접 리스트에 추가 (사이클 탐지용)
                auto& deps = adj.at(writer_id);
                if (std::find(deps.begin(), deps.end(), reader_id) == deps.end()) {
                    deps.push_back(reader_id);
                }

                // b. 타임라인별 의존성 분석
                if (reader_freq == writer_freq) {
                    // 내부 의존성: 의존성 카운터 증가
                    _initial_dependency_counts[reader_id]++;
                    _task_dependents[writer_id].push_back(reader_task);
                } else {
                    // 외부 의존성: 스냅샷 대상에 추가
                    _external_dependency_keys[reader_id].push_back(read_req.key_hash);
                }
            } else { // final_dep_type == DependencyType::Weak
                // Weak 의존성은 실행 순서를 강제하지 않지만,
                // 다른 타임라인의 데이터라면 여전히 스냅샷이 필요함
                if (reader_freq != writer_freq) {
                    _external_dependency_keys[reader_id].push_back(read_req.key_hash);
                }
            }
        }
    }
    spdlog::info("  - Dependency analysis complete.");

    spdlog::info("  - [3/3] Validating graph for cycles...");
    std::set<uint32_t> visited;
    std::vector<uint32_t> path_stack;
    for (const auto& task : _all_tasks_by_id) {
        if (visited.find(task->getID()) == visited.end()) {
            if (detectCycleUtil(task->getID(), adj, visited, path_stack)) {
                return; 
            }
        }
    }


    spdlog::info("  - Dependency analysis for timelines complete.");
}

void RealTimeFramework::wireTaskProxies() {
    void* base_ptr = _shmContext.getBasePtr();
    SharedMemoryHeader* header = _shmContext.getHeader();
    if (!base_ptr || !header) throw std::runtime_error("Cannot wire proxies: SHM not attached.");

    _paramManager->wireParameterReaders(_all_tasks_by_id);

    for (rt::ITask* task : _all_tasks_by_id) {
        for (const auto& req : task->_read_requests) {
            DataBlockDescriptor* desc = _shmContext.getDescriptor(req.key);
            if (!desc) throw std::runtime_error("Wiring failed: descriptor not found for read key " + req.key);
            req.wire_job(base_ptr, desc);
        }
        for (const auto& req : task->_write_requests) {
            DataBlockDescriptor* desc = _shmContext.getDescriptor(req.key);
            if (!desc) throw std::runtime_error("Wiring failed: descriptor not found for write key " + req.key);
            
            blackbox::CacheSlot* record_slot_ptr = nullptr;
            blackbox::CacheSlot* replay_slot_ptr = nullptr;
            if (_record_backend)
                record_slot_ptr = _record_backend->getCacheSlot(req.key_hash);
            if (_replay_backend)
                replay_slot_ptr = _replay_backend->getCacheSlot(req.key_hash);
            req.wire_job(base_ptr, desc, 
                        &(task->_owner_timeline->_start_tick), 
                        &(task->_current_tick_in_cycle), 
                        record_slot_ptr, replay_slot_ptr);
        }
    }
    async_logger->info("  - All task proxies have been wired.");
}

// After a blackbox backend has initialized its metadata, cache slots
// (for record/replay) become available. This helper re-runs the
// write-request wiring to attach the cache slot pointers into the
// existing DataWriter proxies so runtime recording/replay works.
void RealTimeFramework::updateBlackboxSlots(bool clear_slots) {
    void* base_ptr = _shmContext.getBasePtr();
    if (!base_ptr) return;

    for (rt::ITask* task : _all_tasks_by_id) {
        if (!task->_owner_timeline) continue;
        for (const auto& req : task->_write_requests) {
            try {
                common::DataBlockDescriptor* desc = _shmContext.getDescriptor(req.key);
                if (!desc) continue;

                blackbox::CacheSlot* record_slot_ptr = nullptr;
                blackbox::CacheSlot* replay_slot_ptr = nullptr;

                if (!clear_slots) {
                    if (_record_backend)
                        record_slot_ptr = _record_backend->getCacheSlot(req.key_hash);
                    if (_replay_backend)
                        replay_slot_ptr = _replay_backend->getCacheSlot(req.key_hash);

                    // Sanity: ensure cache slot data size matches descriptor data_size
                    if (record_slot_ptr) {
                        if (record_slot_ptr->data.size() == 0) {
                            // lazily pre-size the cache slot to match descriptor
                            record_slot_ptr->data.resize(desc->data_size);
                            async_logger->debug("Blackbox record slot for '{}' resized to {} bytes", req.key, desc->data_size);
                        } else if (record_slot_ptr->data.size() != desc->data_size) {
                            async_logger->warn("Blackbox record slot size mismatch for key '{}': slot={} desc={}",
                                req.key, record_slot_ptr->data.size(), desc->data_size);
                            record_slot_ptr = nullptr; // avoid wiring bad slot
                        }
                    }
                    if (replay_slot_ptr) {
                        if (replay_slot_ptr->data.size() == 0) {
                            replay_slot_ptr->data.resize(desc->data_size);
                            async_logger->debug("Blackbox replay slot for '{}' resized to {} bytes", req.key, desc->data_size);
                        } else if (replay_slot_ptr->data.size() != desc->data_size) {
                            async_logger->warn("Blackbox replay slot size mismatch for key '{}': slot={} desc={}",
                                req.key, replay_slot_ptr->data.size(), desc->data_size);
                            replay_slot_ptr = nullptr;
                        }
                    }
                }

                // If clearing and both are already null, skip to avoid unnecessary writes
                if (clear_slots && !record_slot_ptr && !replay_slot_ptr) continue;

                // Prefer slot-only update to avoid rewiring other fields at runtime.
                if (req.slot_update_job) {
                    req.slot_update_job(record_slot_ptr, replay_slot_ptr);
                } else {
                    // Fallback: call the original wire_job to update the proxy's slot pointers.
                    req.wire_job(base_ptr, desc,
                                &(task->_owner_timeline->_start_tick),
                                &(task->_current_tick_in_cycle),
                                record_slot_ptr, replay_slot_ptr);
                }
            } catch (const std::exception& e) {
                async_logger->error("updateBlackboxSlots: failed wiring key '{}' : {}", req.key, e.what());
            } catch (...) {
                async_logger->error("updateBlackboxSlots: unknown error wiring key '{}'", req.key);
            }
        }
    }

    if (clear_slots)
        async_logger->info("  - Blackbox cache slots cleared from proxies.");
    else
        async_logger->info("  - Blackbox cache slots wired to proxies.");
}

PoolStats* RealTimeFramework::getPoolStatsForCommonRT() {
    int index = _pool_type_to_stats_idx[PoolType::CommonRT];
    return &(_shmContext.getPoolStatsArray()[index]);
}

PoolStats* RealTimeFramework::getPoolStatsForNonRT() {
    int index = _pool_type_to_stats_idx[PoolType::NonRT];
    return &(_shmContext.getPoolStatsArray()[index]);
}

PoolStats* RealTimeFramework::getPoolStatsForDedicatedCore(int core_id) {
    int index = _dedicated_core_to_stats_idx[core_id];
    return &(_shmContext.getPoolStatsArray()[index]);
}

void RealTimeFramework::start() {
    if (_base_frequency == 0) return;
    async_logger->info("Framework starting...");
    _stop_token.store(false);

    int total_rt_workers = _num_common_worker_threads;
    for (const auto& [core_id, num_threads] : _dedicated_core_thread_counts) {
        total_rt_workers += num_threads;
    }

    _rt_workers_ready_count.store(0);


    async_logger->info("ThreadPool starting..");
    pthread_attr_t rt_attr;
    sched_param rt_param;
    pthread_attr_init(&rt_attr);
    pthread_attr_setinheritsched(&rt_attr, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setschedpolicy(&rt_attr, SCHED_FIFO);

    // [로직 변경] 전용 워커 스레드를 코어별 스레드풀 설정에 맞게 생성
    for (const auto& [core_id, num_threads] : _dedicated_core_thread_counts) {
        // _dedicated_worker_threads[core_id].reserve(num_threads); 
        for (int i = 0; i < num_threads; ++i) {
            _dedicated_worker_threads[core_id].emplace_back([this, core_id]() {
                cpu_set_t cpuset;
                CPU_ZERO(&cpuset);
                CPU_SET(core_id, &cpuset);
                if (pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset) != 0) {
                    async_logger->warn("Could not set affinity for core {}", core_id);
                }
                sched_param param;
                param.sched_priority = 80;
                if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &param) != 0) {
                    async_logger->warn("Could not set priority for dedicated worker on core {}", core_id);
                }
                dedicatedWorkerLoop(core_id);
            });
        }
    }

    rt_param.sched_priority = 75;
    pthread_attr_setschedparam(&rt_attr, &rt_param);
    for (int i = 0; i < _num_common_worker_threads; ++i) {
        _common_worker_threads.emplace_back([this, rt_param]() {
            pthread_setschedparam(pthread_self(), SCHED_FIFO, &rt_param);
            commonWorkerLoop();
        });
    }

    rt_param.sched_priority = 70;
    pthread_attr_setschedparam(&rt_attr, &rt_param);
    for (int i = 0; i < _num_non_rt_worker_threads; ++i) {
        _non_rt_worker_threads.emplace_back([this, rt_param]() {
            pthread_setschedparam(pthread_self(), SCHED_FIFO, &rt_param);
            nonRtWorkerLoop();
        });
    }

    pthread_attr_destroy(&rt_attr);

    sched_param main_sched_param;
    main_sched_param.sched_priority = 90;
    if (sched_setscheduler(0, SCHED_FIFO, &main_sched_param) == -1) {
        async_logger->warn("Failed to set scheduler priority for main loop. Run with sudo.");
        if (_realtime_level == RealtimeLevel::HARD) {
            async_logger->warn("Hard realtime not available; falling back to soft realtime.");
            _realtime_level = RealtimeLevel::SOFT;
        }
    }

    async_logger->info("Scheduler is waiting for all {} RT worker threads to be ready...", total_rt_workers);
    while (_rt_workers_ready_count.load() < total_rt_workers) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1)); // Busy-wait 방지
    }

    _rt_workers_pending_count.store(_all_tasks_by_id.size(), std::memory_order_release);
    for (rt::ITask* task : _all_tasks_by_id) {
        task->_set_pushed_time();
        int affinity = task->getAffinity();
        if (task->isNonRt()) {
            _non_rt_task_queue->push(task);
        } else {
            if (affinity >= 0 && _dedicated_task_queues.count(affinity)) {
                _dedicated_task_queues.at(affinity)->push(task);
            } else {
                _common_task_queue->push(task);
            }
        }
    }
    while (_rt_workers_pending_count.load() > 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1)); // Busy-wait 방지
    }

    async_logger->info("ThreadPool ready. Starting scheduler loop.");

    _shmContext.getHeader()->shm_state.store(ShmState::RUNNING, std::memory_order_release);
    _shmContext.getHeader()->framework_tick_count.store(0, std::memory_order_relaxed);
    // _remained_task.store(0, std::memory_order_relaxed);
    // _tasks_for_this_tick.clear();

    runSchedulerLoop();

    // stop process
    // 2. Worker 스레드들의 대기를 중단시키기 위해 각 큐를 stop
    if (_common_task_queue)
        _common_task_queue->stop();
    if (_non_rt_task_queue)
        _non_rt_task_queue->stop();
    for (auto const& [core_id, queue_ptr] : _dedicated_task_queues) {
        queue_ptr->stop();
    }

    if (_shmContext.isAttached()) {
        _shmContext.getHeader()->shm_state.store(ShmState::SHUTTING_DOWN, std::memory_order_release);
    }

    if (_record_backend) {
        _record_backend->shutdown();
        _record_backend = nullptr;
        async_logger->info("Blackbox for record detached.");
    }

    if (_replay_backend) {
        _replay_backend->shutdown();
        _replay_backend = nullptr;
        async_logger->info("Blackbox for replay detached.");
    }

    join();
    
    // // 3. 로거 백엔드 종료 (만약 있다면)
    // if (_archive_backend) {
    //     _archive_backend->shutdown();
    // }
    async_logger->info("RT Framework stopped.");
}

void RealTimeFramework::stop() {
    if (async_logger)
        async_logger->info("Stopping framework...");
    else
        std::cout << "Stopping framework..." << std::endl;
    if (_stop_token.exchange(true)) {
        // 이미 중단 요청이 처리 중이면 아무것도 하지 않음
        return;
    }
    
}

// [로직 변경] 코어별로 여러 스레드를 join 하도록 수정
void RealTimeFramework::join() {
    // 이미 join이 수행되었으면 다시 하지 않음
    if (_joined.exchange(true)) {
        return;
    }
    
    if (async_logger)
        async_logger->info("Check ThreadPool");
    else
        std::cout << "Check ThreadPool" << std::endl;

    if (async_logger)
        async_logger->info("Common Worker Threads stopping..");
    else
        std::cout << "Common Worker Threads stopping.." << std::endl;

    for(auto& th : _common_worker_threads) {
        if(th.joinable()) th.join();
    }
    _common_worker_threads.clear();

    if (async_logger)
        async_logger->info("Dedicated Worker Threads stopping..");
    else
        std::cout << "Dedicated Worker Threads stopping..";

    for(auto& [core_id, thread_vec] : _dedicated_worker_threads) {
        for(auto& th : thread_vec) {
            if(th.joinable()) th.join();
        }
        thread_vec.clear();
    }
    _dedicated_worker_threads.clear();

    if (async_logger)
        async_logger->info("Non-RT Threads stopping..");
    else
        std::cout << "Non-RT Threads stopping.." << std::endl;

    for(auto& th : _non_rt_worker_threads) {
        if(th.joinable()) th.join();
    }
    _non_rt_worker_threads.clear();

    if (async_logger)
        async_logger->info("ThreadPool stopped.");
    else
        std::cout << "ThreadPool stopped." << std::endl;
}

void RealTimeFramework::enqueueTask(rt::ITask* task) {
    if (!task) return;

    TaskStats* stats = getTaskStats(task->getID());

    // CAS: 이미 busy(큐 대기 or 실행 중)이면 재투입 차단
    // plain store 대신 CAS를 사용하여 double enqueue → _remained_task_count 언더플로우 방지
    bool expected = false;
    if (!stats->is_busy.compare_exchange_strong(expected, true,
            std::memory_order_acq_rel,
            std::memory_order_acquire)) {
        async_logger->warn("Task '{}' already busy, skipping re-enqueue (double enqueue prevented)", task->getName());
        return;
    }

    internal::Timeline* timeline = task->getOwnerTimeline();

    // execution_local_tick을 enqueue 시점에 기록
    // tick_start에서 일괄 설정하면 stale task의 generation이 다음 tick_start에서 덮어써지는 문제 방지
    if (timeline) {
        task->_set_execution_local_tick(
            timeline->_local_tick_count.load(std::memory_order_acquire));
    }

    if (stats && timeline) {
        long long pushed_time_ns = std::chrono::high_resolution_clock::now().time_since_epoch().count();
        long long timeline_start_ns = timeline->_current_tick_start_time_ns;
        stats->last_pushed_to_queue_offset_ns.store(
            pushed_time_ns - timeline_start_ns, 
            std::memory_order_relaxed
        );
    }

    async_logger->trace(">>> QUEUING '{}' to {} queue.", task->getName(), (task->getAffinity() >= 0) ? 
                                                                            "Dedicated Core " + std::to_string(task->getAffinity()) : 
                                                                                (task->isNonRt()? 
                                                                                "NonRT": 
                                                                                "Common"));
    
    task->_set_pushed_time();
    int affinity = task->getAffinity();
    
    if (task->isNonRt()) {
        _non_rt_task_queue->push(task);
    } else {
        int affinity = task->getAffinity();
        if (affinity >= 0 && _dedicated_task_queues.count(affinity)) {
            _dedicated_task_queues.at(affinity)->push(task);
        } else {
            _common_task_queue->push(task);
        }
    }
}

// 공용 워커 스레드 루프
void RealTimeFramework::commonWorkerLoop() {
    PoolStats* pool_stats = getPoolStatsForCommonRT();
    _rt_workers_ready_count.fetch_add(1);
    rt::ITask* task;
    while (_common_task_queue->pop(task)) {
        internal::ShmRingbufferSink::set_context(task->getOwnerTimeline()->getName());
        if (_shmContext.getHeader()->shm_state.load(std::memory_order_relaxed) == ShmState::RUNNING) {
            if (pool_stats->active_worker_count.fetch_add(1, std::memory_order_relaxed) == 0) {
                // 풀이 Idle -> Busy 상태로 전환. 시작 시간을 기록.
                pool_stats->busy_start_time_ns.store(
                    std::chrono::high_resolution_clock::now().time_since_epoch().count(),
                    std::memory_order_relaxed
                );
            }
            executeAndPropagateTask(task);
            if (pool_stats->active_worker_count.fetch_sub(1, std::memory_order_relaxed) == 1) {
                // 풀이 Busy -> Idle 상태로 전환. Busy 구간을 계산하여 누적.
                long long start_time = pool_stats->busy_start_time_ns.load();
                
                // start_time이 0이 아닌 경우 (정상적인 시작-종료 쌍)
                if (start_time != 0) {
                    long long end_time = std::chrono::high_resolution_clock::now().time_since_epoch().count();
                    long long duration = end_time - start_time;

                    if (duration > 0) {
                        pool_stats->total_busy_ns.fetch_add(duration, std::memory_order_relaxed);
                        pool_stats->total_squared_busy_ns.fetch_add(duration * duration, std::memory_order_relaxed);
                        pool_stats->sample_count.fetch_add(1, std::memory_order_relaxed);
                    }
                }
            }
        } else {
            task->warmup();
            _rt_workers_pending_count.fetch_sub(1, std::memory_order_release);
        }
    }
}

// 전용 워커 스레드 루프 (코드는 변경 없음)
void RealTimeFramework::dedicatedWorkerLoop(int core_id) {
    PoolStats* pool_stats = getPoolStatsForDedicatedCore(core_id);
    _rt_workers_ready_count.fetch_add(1);
    auto& queue = _dedicated_task_queues.at(core_id);
    rt::ITask* task;
    while (queue->pop(task)) {
        internal::ShmRingbufferSink::set_context(task->getOwnerTimeline()->getName());
        if (_shmContext.getHeader()->shm_state.load(std::memory_order_relaxed) == ShmState::RUNNING) {
            if (pool_stats->active_worker_count.fetch_add(1, std::memory_order_relaxed) == 0) {
                // 풀이 Idle -> Busy 상태로 전환. 시작 시간을 기록.
                pool_stats->busy_start_time_ns.store(
                    std::chrono::high_resolution_clock::now().time_since_epoch().count(),
                    std::memory_order_relaxed
                );
            }
            executeAndPropagateTask(task);
            if (pool_stats->active_worker_count.fetch_sub(1, std::memory_order_relaxed) == 1) {
                // 풀이 Busy -> Idle 상태로 전환. Busy 구간을 계산하여 누적.
                long long start_time = pool_stats->busy_start_time_ns.load();
                
                // start_time이 0이 아닌 경우 (정상적인 시작-종료 쌍)
                if (start_time != 0) {
                    long long end_time = std::chrono::high_resolution_clock::now().time_since_epoch().count();
                    long long duration = end_time - start_time;

                    if (duration > 0) {
                        pool_stats->total_busy_ns.fetch_add(duration, std::memory_order_relaxed);
                        pool_stats->total_squared_busy_ns.fetch_add(duration * duration, std::memory_order_relaxed);
                        pool_stats->sample_count.fetch_add(1, std::memory_order_relaxed);
                    }
                }
            }
        } else {
            task->warmup();
            _rt_workers_pending_count.fetch_sub(1, std::memory_order_release);
        }
    }
}

// Non-RT 워커 스레드 루프
void RealTimeFramework::nonRtWorkerLoop() {
    PoolStats* pool_stats = getPoolStatsForNonRT();
    // _rt_workers_ready_count.fetch_add(1);
    rt::ITask* task;
    while (_non_rt_task_queue->pop(task)) {
        internal::ShmRingbufferSink::set_context("NonRT");
        if (_shmContext.getHeader()->shm_state.load(std::memory_order_relaxed) == ShmState::RUNNING) {
            if (pool_stats->active_worker_count.fetch_add(1, std::memory_order_relaxed) == 0) {
                // 풀이 Idle -> Busy 상태로 전환. 시작 시간을 기록.
                pool_stats->busy_start_time_ns.store(
                    std::chrono::high_resolution_clock::now().time_since_epoch().count(),
                    std::memory_order_relaxed
                );
            }
            executeAndPropagateTask(task);
            if (pool_stats->active_worker_count.fetch_sub(1, std::memory_order_relaxed) == 1) {
                // 풀이 Busy -> Idle 상태로 전환. Busy 구간을 계산하여 누적.
                long long start_time = pool_stats->busy_start_time_ns.load();
                
                // start_time이 0이 아닌 경우 (정상적인 시작-종료 쌍)
                if (start_time != 0) {
                    long long end_time = std::chrono::high_resolution_clock::now().time_since_epoch().count();
                    long long duration = end_time - start_time;

                    if (duration > 0) {
                        pool_stats->total_busy_ns.fetch_add(duration, std::memory_order_relaxed);
                        pool_stats->total_squared_busy_ns.fetch_add(duration * duration, std::memory_order_relaxed);
                        pool_stats->sample_count.fetch_add(1, std::memory_order_relaxed);
                    }
                }
            }
        } else {
            task->warmup();
            _rt_workers_pending_count.fetch_sub(1, std::memory_order_release);
        }
    }
}

void RealTimeFramework::runSchedulerLoop() {
    if (_base_frequency == 0) {
        async_logger->error("ERROR: Base frequency is 0. Cannot start scheduler loop.");
        return;
    }
    
    internal::ShmRingbufferSink::set_context("Framework");
    async_logger->info("Scheduler loop starting with base frequency: {} Hz", _base_frequency);

    // --- 초기 설정 ---
    // Context와 Header 포인터를 한 번만 가져와서 루프 내에서 계속 사용
    SharedMemoryHeader* header = _shmContext.getHeader();
    if (!header) {
        throw std::runtime_error("SharedMemoryHeader is null. Initialization failed.");
    }
    
    LogLevel current_framework_level = static_cast<LogLevel>(async_logger->level());

    // Tick 주기를 나노초 단위로 계산
    const long cycle_ns = 1'000'000'000L / _base_frequency;
    auto last_calc_time = std::chrono::high_resolution_clock::now();
    std::map<int, long long> last_pool_busy_ns; // Key: pool_stats_index
    
    // 다음 사이클 시작 시간을 위한 timespec 구조체
    struct timespec next_cycle_time_ts;
    clock_gettime(CLOCK_MONOTONIC, &next_cycle_time_ts);
    auto tick_start_time = std::chrono::high_resolution_clock::now();
    // --- 메인 루프 ---
    while (!_stop_token.load()) {        
        // 2. 현재 Tick 시간 정보를 공유 메모리에 기록
        header->current_tick_start_time_ns.store(tick_start_time.time_since_epoch().count(), std::memory_order_release);
        uint64_t current_tick = header->framework_tick_count.load(std::memory_order_relaxed);

        // 매 틱 시작 시 동적 액션 처리
        handleAction(header, current_tick);

        // handleAction() may change framework_tick_count (e.g. replay checkpoint restore).
        // Reload tick so the current loop uses the restored/updated tick immediately.
        current_tick = header->framework_tick_count.load(std::memory_order_relaxed);

        // If scheduler is paused, skip tick processing but continue polling for actions
        // Always advance the timing reference (next_cycle_time_ts) so that when
        // the scheduler resumes there are no large timing jumps. Use absolute
        // nanosleep to maintain the tick cadence; this applies both to replay
        // and live modes.
        if (_paused.load(std::memory_order_acquire)) {
            next_cycle_time_ts.tv_nsec += cycle_ns;
            while (next_cycle_time_ts.tv_nsec >= 1'000'000'000L) {
                next_cycle_time_ts.tv_nsec -= 1'000'000'000L;
                next_cycle_time_ts.tv_sec++;
            }
            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_cycle_time_ts, NULL);
            continue;
        }

        // --- 동적 로그 레벨 제어 ---
        LogLevel desired_level = header->shared_log_level.load(std::memory_order_relaxed);
        if (desired_level != current_framework_level) {
            async_logger->set_level(static_cast<spdlog::level::level_enum>(desired_level));
            current_framework_level = desired_level;
            // 레벨이 변경되었다는 사실 자체를 로그로 남기는 것이 좋음
            async_logger->warn("Log level changed to: {}", spdlog::level::to_string_view(static_cast<spdlog::level::level_enum>(desired_level)));
        }

        async_logger->trace("Tick {} Started.", current_tick);

        // 블랙박스 틱 신호 (상태 플래그 확인)
        if (header->recording_active.load(std::memory_order_acquire) && _record_backend)
            _record_backend->onTick(current_tick);
        if (header->replaying_active.load(std::memory_order_acquire) && _replay_backend)
            _replay_backend->onTick(current_tick);

        _paramManager->swapBuffers();

        // 3. 새로운 Tick의 시작 작업 (모든 타임라인에 tick_start 신호)
        for (auto const& [freq, timeline] : _timelines) {
            timeline->tick_start(current_tick);
        }
        
        if (false) {
            // soft realtime
            // bool idle = false;
            // while(!idle) {
            //     next_cycle_time_ts.tv_nsec += cycle_ns;
            //     while (next_cycle_time_ts.tv_nsec >= 1'000'000'000L) {
            //         next_cycle_time_ts.tv_nsec -= 1'000'000'000L;
            //         next_cycle_time_ts.tv_sec++;
            //     }
            //     clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_cycle_time_ts, NULL);

            //     idle = true;
            //     for (auto const& [freq, timeline] : _timelines) {
            //         idle &= timeline->check_idle();
            //     }
            // }
        } else {\

            if (_mode != Mode::TRACE) {
                next_cycle_time_ts.tv_nsec += cycle_ns;
                while (next_cycle_time_ts.tv_nsec >= 1'000'000'000L) {
                    next_cycle_time_ts.tv_nsec -= 1'000'000'000L;
                    next_cycle_time_ts.tv_sec++;
                }
                clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_cycle_time_ts, NULL);
            }

            if (_realtime_level == RealtimeLevel::SOFT) {
                bool pending = true;
                while(pending) {
                    std::this_thread::sleep_for(1000000us / (_base_frequency*100));
                    pending = false;
                    for (auto& timeline: _timelines)
                        pending |= timeline.second->isPending(current_tick);
                }
            }

        }
        tick_start_time = std::chrono::high_resolution_clock::now();

        internal::ShmRingbufferSink::set_context("Framework");
        async_logger->trace("Tick {} Ended.", current_tick);
        
        // 1. 이전 Tick의 마무리 작업 (모든 타임라인에 tick_end 신호)
        for (auto const& [freq, timeline] : _timelines) {
            timeline->tick_end(current_tick);
        }

        // Deferred record start: execute after current tick has fully ended,
        // so checkpoint captures RT boundary state at this tick.
        if (_record_start_pending) {
            if (startRecord(_record_start_pending_filename, current_tick, header)) {
                const uint64_t marker_tick = header->framework_tick_count.load(std::memory_order_acquire);
                async_logger->info("[Action] recording started: {} (marker_tick={})", _record_start_pending_filename, marker_tick);
            } else {
                async_logger->error("[Action] deferred START_RECORD failed: {}", _record_start_pending_filename);
            }
            _record_start_pending = false;
            _record_start_pending_filename.clear();
            _record_start_request_tick = 0;
        }

        // If a single-step was requested, decrement and pause after this tick
        if (_single_step_remaining.load(std::memory_order_acquire) > 0) {
            int prev = _single_step_remaining.fetch_sub(1, std::memory_order_acq_rel);
            if (prev <= 1) {
                // we've finished the requested single step(s)
                _paused.store(true, std::memory_order_release);
                async_logger->info("Single-step completed; scheduler paused.");
            }
        }

        // If replay is active and has finished, pause automatically
        if (_replay_backend && _replay_backend->getMode() == rtfw::blackbox::Mode::REPLAY) {
            if (_replay_backend->is_replay_finished(current_tick)) {
                _paused.store(true, std::memory_order_release);
                stopReplay(header);
                async_logger->info("Replay finished; scheduler paused.");
            }
        }

        // --- << 주기적인 부하율 계산 로직 >> ---
        // // 예: 매 1초마다 (또는 _base_frequency tick 마다) 실행
        // if (current_tick > 0 && current_tick % _base_frequency == 0) {
        //     auto now = std::chrono::high_resolution_clock::now();
        //     long long elapsed_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now - last_calc_time).count();
        //     if (elapsed_ns == 0) elapsed_ns = 1;

        //     const PoolStats* pool_stats_array = _shmContext.getPoolStatsArray();
            
        //     // 모든 풀에 대해 반복
        //     for (int i = 0; i < _shmContext.getHeader()->pool_stats_count; ++i) {
        //         long long current_busy = pool_stats_array[i].total_busy_ns.load();
        //         long long delta_busy = current_busy - last_pool_busy_ns[i];
        //         float usage = (static_cast<double>(delta_busy) / elapsed_ns) * 100.0f;
        //         if (usage > 100.0f) usage = 100.0f;

        //         // 계산된 최종 부하율을 공유 메모리에 직접 기록
        //         int usage_as_int = static_cast<int>(usage * 100.0f);
        //         const_cast<PoolStats&>(pool_stats_array[i]).usage_percentage_x100.store(usage_as_int, std::memory_order_relaxed);

        //         // 다음 계산을 위해 현재 값을 저장
        //         last_pool_busy_ns[i] = current_busy;
        //     }
        //     last_calc_time = now;
            
        //     if (_log_level >= LogLevel::DetailedDeps) {
        //         printStats();
        //     }
        // }
        
        // 5. 다음 Tick을 위해 프레임워크 Tick 카운터 증가
        header->framework_tick_count.fetch_add(1, std::memory_order_relaxed);
    }
    async_logger->debug("Scheduler loop finished.");
}


void RealTimeFramework::printStats() const {
    if (!_shmContext.isAttached()) {
        std::cout << "\n--- Purism Framework Stats ---" << std::endl;
        std::cout << "Framework is not initialized or SHM is not attached." << std::endl;
        return;
    }
    
    const SharedMemoryHeader* header = _shmContext.getHeader();
    if (!header) return;

    std::cout << "\n--- Purism Framework Stats (Global Tick: " 
              << header->framework_tick_count.load() << ") ---" << std::endl;

    // --- Level 1: Timeline Stats ---
    std::cout << "\n[Timeline Stats]" << std::endl;
    std::cout << "    Freq (Hz) | Period(us) | Avg Busy (us)      | Peak Busy(us) | Load(%) | Misses" << std::endl;
    std::cout << "    ----------|------------|--------------------|---------------|---------|-------" << std::endl;

    const TimelineStats* timeline_stats = _shmContext.getTimelineStatsArray();
    for (const auto& [freq, index] : _frequency_to_timeline_stats_idx) {

        const auto& stats = timeline_stats[index];
        long long N = stats.tick_count.load();
        if (N == 0)
            continue;
        double period_us = 1000000.0 / freq;
        double avg_us = 0.0, std_dev_us = 0.0, peak_us = 0.0, load = 0.0;

        if (N > 0) {
            avg_us = (stats.total_busy_ns.load() / (double)N) / 1000.0;
            peak_us = stats.max_busy_ns.load() / 1000.0;
            load = (avg_us / period_us) * 100.0;
            if (N > 1) {
                double mean_sq = stats.total_squared_busy_ns.load() / (double)N;
                double avg_sq = std::pow(stats.total_busy_ns.load() / (double)N, 2);
                double var_ns_sq = mean_sq - avg_sq;
                if (var_ns_sq < 0) var_ns_sq = 0;
                std_dev_us = std::sqrt(var_ns_sq) / 1000.0;
            }
        }
        std::cout << "    " << std::right << std::setw(9) << freq << " | "
                  << std::fixed << std::setprecision(2) << std::setw(10) << period_us << " | "
                  << std::setw(8) << avg_us << " (±" << std::setw(6) << std_dev_us << ") | "
                  << std::setw(13) << peak_us << " | "
                  << std::setw(7) << load << " | "
                  << std::setw(6) << stats.deadline_miss_count.load() << std::endl;
    }

    // --- Level 2: Thread Pool Occupancy ---
    std::cout << "\n[Thread Pool Occupancy]" << std::endl;
    std::cout << "    Pool Name            | Workers | Occupancy (%)" << std::endl;
    std::cout << "    ---------------------|---------|----------" << std::endl;

    const PoolStats* pool_stats = _shmContext.getPoolStatsArray();
    
    
    auto print_pool_stats = [&](const std::string& name, int workers, int index) {
        // 공유 메모리에서 미리 계산된 값을 읽기만 한다.
        int usage_as_int = pool_stats[index].usage_percentage_x100.load(std::memory_order_relaxed);
        float usage = static_cast<float>(usage_as_int) / 100.0f;
        
        std::cout << "    " << std::left << std::setw(20) << name << " | "
                << std::right << std::setw(7) << workers << " | "
                << std::fixed << std::setprecision(2) << std::setw(8) << usage << std::endl;
    };

    // Common RT Pool
    print_pool_stats("Common RT Pool", _num_common_worker_threads, _pool_type_to_stats_idx.at(PoolType::CommonRT));
    // Non-RT Pool
    print_pool_stats("Non-RT Pool", _num_non_rt_worker_threads, _pool_type_to_stats_idx.at(PoolType::NonRT));
    // Dedicated Pools
    for (const auto& [core_id, index] : _dedicated_core_to_stats_idx) {
        print_pool_stats("Dedicated Core " + std::to_string(core_id), _dedicated_core_thread_counts.at(core_id), index);
    }
    
    // 다음 계산을 위해 현재 값 저장
    for (const auto& [type, index] : _pool_type_to_stats_idx) { _last_pool_busy_ns[index] = pool_stats[index].total_busy_ns.load(); }
    for (const auto& [core_id, index] : _dedicated_core_to_stats_idx) { _last_pool_busy_ns[index] = pool_stats[index].total_busy_ns.load(); }

    // --- Level 3: Task Execution Stats ---
    std::cout << "\n[Task Execution Stats]" << std::endl;
    std::cout << "    Task Name (ID,Core)           | Avg/Peak Exec (us) | Avg/Peak Latency(us) |   Count" << std::endl;
    std::cout << "    ------------------------------|--------------------|----------------------|----------" << std::endl;
    
    const TaskStats* task_stats_array = _shmContext.getTaskStatsArray();
    for (const auto& task : _all_tasks_by_id) {
        task->printStats(task_stats_array[task->getID()]);
    }
}

// path_stack: 현재 DFS 경로에 있는 Task ID들을 추적하는 스택
// visited: 전체 방문 기록 (중복 탐색 방지)
bool RealTimeFramework::detectCycleUtil(
    uint32_t task_id,
    const std::map<uint32_t, std::vector<uint32_t>>& adj,
    std::set<uint32_t>& visited,
    std::vector<uint32_t>& path_stack)
{
    visited.insert(task_id);
    path_stack.push_back(task_id);

    for (uint32_t neighbor_id : adj.at(task_id)) {
        // 경로 상에 이미 있는 노드를 다시 만났다면 사이클!
        bool is_in_path = false;
        for(uint32_t node_in_path : path_stack) {
            if (node_in_path == neighbor_id) {
                is_in_path = true;
                break;
            }
        }
        
        if (is_in_path) {
            // --- 사이클 발견! 에러 메시지 생성 및 출력 ---
            std::string error_msg = "\n[FATAL] A cycle was detected in the dependency graph.\n";
            error_msg += "Cycle Path: \n";
            
            // 1. 사이클이 시작되는 지점의 반복자를 찾음
            auto cycle_start_it = std::find(path_stack.begin(), path_stack.end(), neighbor_id);
            
            // 2. 사이클 경로에 있는 Task들을 별도의 벡터로 복사
            std::vector<uint32_t> cycle_nodes;
            for (auto it = cycle_start_it; it != path_stack.end(); ++it) {
                cycle_nodes.push_back(*it);
            }
            // 사이클을 닫는 시작 노드를 마지막에 추가
            cycle_nodes.push_back(neighbor_id);

            // 3. 복사된 경로를 순회하며 의존성 출력 (이제 루프가 N번 돌면 N개의 간선이 출력됨)
            for (size_t i = 0; i < cycle_nodes.size() - 1; ++i) {
                uint32_t u_id = cycle_nodes[i];
                uint32_t v_id = cycle_nodes[i+1];

                rt::ITask* u_task = _all_tasks_by_id.at(u_id);
                rt::ITask* v_task = _all_tasks_by_id.at(v_id);

                // u_task가 v_task에게 의존성을 제공하는 데이터 키를 찾는다.
                std::string dependency_key = "???";
                bool key_found = false;
                for (const auto& write_req : u_task->_write_requests) {
                    for (const auto& read_req : v_task->_read_requests) {
                        if (write_req.key == read_req.key) {
                            dependency_key = write_req.key;
                            key_found = true;
                            break;
                        }
                    }
                    if(key_found) break;
                }

                error_msg += "  - Task '" + std::string(u_task->getName()) 
                        + "' writes to <--'" + dependency_key + "'--> which is read by Task '" 
                        + std::string(v_task->getName()) + "'";
                
                // 마지막 간선에만 "(completing the cycle)" 추가
                if (i == cycle_nodes.size() - 2) {
                    error_msg += " (completing the cycle)";
                }
                error_msg += "\n";
            }

            throw std::runtime_error(error_msg);
        }

        if (visited.find(neighbor_id) == visited.end()) {
            if (detectCycleUtil(neighbor_id, adj, visited, path_stack)) {
                return true;
            }
        }
    }

    path_stack.pop_back(); // 현재 노드에서 나갈 때 경로에서 제거
    return false;
}

void RealTimeFramework::executeAndPropagateTask(rt::ITask* task) {
    if (task == nullptr) {
        return;
    }

    uint32_t task_id = task->getID();
    const char* task_name = task->getName();
    
    // 이 Task에 해당하는 통계 구조체에 대한 포인터
    TaskStats* stats = &(_shmContext.getTaskStatsArray()[task_id]);
    internal::Timeline* owner_timeline = task->_owner_timeline;

    // --- Task 실행 및 시간 측정 ---
    
    // 현재 프레임워크 Tick을 task 객체에 기록 (DataWriter의 stale write 방지용)
    task->_set_current_tick(_shmContext.getHeader()->framework_tick_count.load(std::memory_order_acquire));
    
    try {
        auto start_exec_time = std::chrono::high_resolution_clock::now();
        stats->last_start_offset_ns = start_exec_time.time_since_epoch().count() - owner_timeline->_current_tick_start_time_ns;

        // 1. 스케줄링 지연 시간 계산 및 통계 업데이트
        auto latency_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(start_exec_time - task->_pushed_to_queue_time).count();
        stats->total_latency_ns.fetch_add(latency_ns, std::memory_order_relaxed);
        stats->total_squared_latency_ns.fetch_add(latency_ns * latency_ns, std::memory_order_relaxed);
        
        long long prev_max_lat = stats->max_latency_ns.load(std::memory_order_relaxed);
        while (latency_ns > prev_max_lat) {
            if (stats->max_latency_ns.compare_exchange_weak(prev_max_lat, latency_ns)) break;
        }

        async_logger->trace("'{}' (ID: {}) started.", task->getName(), task->getID());

        // 2. Replay injection must happen BEFORE task execution so readers
        // in this execution observe injected data context.
        task->_injection_proxy();

        // 3. 실제 Task 실행

        void* task_state = (task->getStateSize() > 0) ? 
                        &_state_buffer[_task_state_offsets[task_id]] : nullptr;
        task->execute(task_state);
        
        // 실행 완료 표시 (release: tick_end의 acquire load와 happens-before 관계 보장)
        task->_executed.store(true, std::memory_order_release);

        async_logger->trace("'{}' (ID: {}) finished.", task->getName(), task->getID());

        
        // 4. 실행 시간 계산 및 통계 업데이트
        auto end_exec_time = std::chrono::high_resolution_clock::now();
        auto exec_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(end_exec_time - start_exec_time).count();
        stats->last_completion_offset_ns = stats->last_start_offset_ns + exec_ns;
        
        stats->total_exec_time_ns.fetch_add(exec_ns, std::memory_order_relaxed);
        stats->total_squared_exec_time_ns.fetch_add(exec_ns * exec_ns, std::memory_order_relaxed);
        stats->exec_count.fetch_add(1, std::memory_order_relaxed);

        long long prev_max_exec = stats->max_exec_time_ns.load(std::memory_order_relaxed);
        while (exec_ns > prev_max_exec) {
            if (stats->max_exec_time_ns.compare_exchange_weak(prev_max_exec, exec_ns)) break;
        }

        // [NEW] Stale write 감지 및 통계 기록
        if (task->hadAnyStaleWrite()) {
            async_logger->error("Task '{}' (ID: {}) encountered stale write(s) during execution", 
                              task->getName(), task_id);
            stats->stale_write_count.fetch_add(1, std::memory_order_relaxed);
            task->clearStaleWriteFlags();
        }

    } catch (const std::exception& e) {
        async_logger->critical("!!! CRITICAL ERROR in Task '{}' (ID: {}): {}", task->getName(), task_id, e.what());
        // 필요 시, 여기에 추가적인 오류 상태를 TaskStats에 기록할 수 있음
    } catch (...) {
        async_logger->critical("!!! UNKNOWN CRITICAL ERROR in Task '{}' (ID: {})", task->getName(), task_id);
    }

    // --- 실행 완료 처리 ---

    if (task->_owner_timeline) {
        task->_owner_timeline->onTaskFinished(task);
    } else {
        async_logger->error("!!! CRITICAL ERROR in Task '{}' (ID: {}): missing timeline.", task->getName(), task_id);
    }

    // busy 플래그 해제
    stats->is_busy.store(false, std::memory_order_release);

    
    // // --- 리플레이 데이터 강제 주입 ---
    // // 프레임워크가 리플레이 모드일 경우에만 이 로직을 수행
    // if (_replay_backend) {
    //     // Task가 가진 모든 리플레이 훅(DataWriter)에 대해
    //     for (const auto& hook : task->_replay_hooks) {
    //         // 1. 현재 tick과 writer의 키로 백엔드에 데이터 요청
    //         uint64_t key_hash = hook.key_hash;
    //         uint64_t type_hash = hook.type_hash;
    //         auto result = _archive_backend->query(task->getCurrentTick(), key_hash);

    //         if (result && !result->data.empty()) {

    //             // --- 공유 메모리에서 필요한 정보 가져오기 ---
    //             SharedMemoryHeader* header = _shmContext.getHeader();
    //             if (!header) {
    //                 std::cerr << "CRITICAL: SharedMemoryHeader is null in worker thread." << std::endl;
    //                 return;
    //             }
                
    //             // 2. 리플레이할 데이터가 있으면, 캡처된 함수를 호출하여 주입
    //             hook.replay_function(result->data.data(), result->data.size(), task->getCurrentTick());
    //             if (_archive_backend->get_mode() == blackbox::Mode::TRACING)
    //                 _archive_backend->archive(header->framework_tick_count.load(std::memory_order_relaxed), key_hash, type_hash, result->data.data(), result->data.size());
    //         }
            
    //     }
    // }
}

bool RealTimeFramework::isWeakDependency(const rt::ITask* reader_task, const rt::ITask::ReadRequest& read_req) const {
    auto writer_it = _write_key_to_task_id.find(read_req.key);
    rt::DependencyType final_dep_type = read_req.deptype;

    if (writer_it != _write_key_to_task_id.end()) {
        const rt::ITask* writer_task = _all_tasks_by_id[writer_it->second];
        if (writer_task->isNonRt() && !reader_task->isNonRt()) {
            final_dep_type = rt::DependencyType::Weak;
        }
    }

    return final_dep_type == rt::DependencyType::Weak;
}

std::vector<uint8_t> RealTimeFramework::buildCheckpointV2(uint64_t checkpoint_tick) const {
    struct WeakCaptureEntry {
        uint32_t task_id;
        uint64_t key_hash;
        uint64_t write_tick;
        std::vector<uint8_t> data;
    };

    std::vector<CheckpointTimelineTick> timeline_ticks;
    timeline_ticks.reserve(_timelines.size());
    for (const auto& [freq, timeline] : _timelines) {
        CheckpointTimelineTick entry{};
        entry.frequency = freq;
        entry.local_tick = timeline->_local_tick_count.load(std::memory_order_acquire);
        timeline_ticks.push_back(entry);
    }

    std::vector<WeakCaptureEntry> weak_captures;
    for (const auto* task : _all_tasks_by_id) {
        for (const auto& read_req : task->_read_requests) {
            if (!isWeakDependency(task, read_req)) {
                continue;
            }

            auto reader_it = task->_data_readers.find(read_req.key_hash);
            if (reader_it == task->_data_readers.end() || !reader_it->second) {
                continue;
            }

            uint64_t write_tick = 0;
            std::vector<uint8_t> captured;
            if (!reader_it->second->export_capture_value(captured, write_tick)) {
                continue;
            }

            WeakCaptureEntry weak_entry{};
            weak_entry.task_id = task->getID();
            weak_entry.key_hash = read_req.key_hash;
            weak_entry.write_tick = write_tick;
            weak_entry.data = std::move(captured);
            weak_captures.push_back(std::move(weak_entry));
        }
    }

    // Also snapshot current ready-buffer values of archived writer keys.
    // This fixes first-tick replay mismatches where no injected sample exists yet
    // at the exact marker tick, but readers still need the same initial data view.
    std::set<uint64_t> archived_key_seen;
    void* shm_base = _shmContext.getBasePtr();
    for (const auto* task : _all_tasks_by_id) {
        for (const auto& write_req : task->_write_requests) {
            if (!write_req.archive_enabled) continue;
            if (!archived_key_seen.insert(write_req.key_hash).second) continue;

            const auto* desc = _shmContext.getDescriptor(write_req.key);
            if (!desc || !shm_base) continue;

            auto* header = common::get_buffer_header(shm_base, const_cast<common::DataBlockDescriptor*>(desc));
            if (!header) continue;
            int ready_idx = header->ready_index.load(std::memory_order_acquire);
            if (ready_idx < 0) continue;

            auto* state = common::get_buffer_state(shm_base, const_cast<common::DataBlockDescriptor*>(desc), ready_idx);
            auto* data_ptr = common::get_data_buffer(shm_base, const_cast<common::DataBlockDescriptor*>(desc), ready_idx);
            if (!state || !data_ptr) continue;

            WeakCaptureEntry arch_entry{};
            arch_entry.task_id = CHECKPOINT_ARCHIVE_SNAPSHOT_TASK_ID;
            arch_entry.key_hash = write_req.key_hash;
            arch_entry.write_tick = state->write_tick.load(std::memory_order_relaxed);
            arch_entry.data.resize(desc->data_size);
            memcpy(arch_entry.data.data(), data_ptr, desc->data_size);
            weak_captures.push_back(std::move(arch_entry));
        }
    }

    // Snapshot current ready-buffer values of non-archived RT writer keys.
    // These are used as RT boundary samples to make the very first replay RT
    // observation deterministic at the marker tick.
    std::set<uint64_t> rt_boundary_key_seen;
    for (const auto* task : _all_tasks_by_id) {
        if (task->isNonRt()) continue;

        for (const auto& write_req : task->_write_requests) {
            if (write_req.archive_enabled) continue;
            if (!rt_boundary_key_seen.insert(write_req.key_hash).second) continue;

            const auto* desc = _shmContext.getDescriptor(write_req.key);
            if (!desc || !shm_base) continue;

            auto* header = common::get_buffer_header(shm_base, const_cast<common::DataBlockDescriptor*>(desc));
            if (!header) continue;
            int ready_idx = header->ready_index.load(std::memory_order_acquire);
            if (ready_idx < 0) continue;

            auto* state = common::get_buffer_state(shm_base, const_cast<common::DataBlockDescriptor*>(desc), ready_idx);
            auto* data_ptr = common::get_data_buffer(shm_base, const_cast<common::DataBlockDescriptor*>(desc), ready_idx);
            if (!state || !data_ptr) continue;

            WeakCaptureEntry rt_entry{};
            rt_entry.task_id = CHECKPOINT_RT_BOUNDARY_SNAPSHOT_TASK_ID;
            rt_entry.key_hash = write_req.key_hash;
            rt_entry.write_tick = state->write_tick.load(std::memory_order_relaxed);
            rt_entry.data.resize(desc->data_size);
            memcpy(rt_entry.data.data(), data_ptr, desc->data_size);
            weak_captures.push_back(std::move(rt_entry));
        }
    }

    size_t total_size = sizeof(CheckpointV2Header);
    total_size += _state_buffer.size();
    total_size += timeline_ticks.size() * sizeof(CheckpointTimelineTick);
    for (const auto& weak_entry : weak_captures) {
        total_size += sizeof(CheckpointWeakCaptureHeader) + weak_entry.data.size();
    }

    std::vector<uint8_t> serialized;
    serialized.reserve(total_size);
    auto append_bytes = [&serialized](const void* data, size_t size) {
        const auto* p = static_cast<const uint8_t*>(data);
        serialized.insert(serialized.end(), p, p + size);
    };

    CheckpointV2Header header{};
    header.magic = CHECKPOINT_V2_MAGIC;
    header.version = CHECKPOINT_V2_VERSION;
    header.state_size = _state_buffer.size();
    header.global_tick = checkpoint_tick;
    header.timeline_count = static_cast<uint32_t>(timeline_ticks.size());
    header.weak_capture_count = static_cast<uint32_t>(weak_captures.size());
    append_bytes(&header, sizeof(header));

    if (!_state_buffer.empty()) {
        append_bytes(_state_buffer.data(), _state_buffer.size());
    }

    for (const auto& timeline_entry : timeline_ticks) {
        append_bytes(&timeline_entry, sizeof(timeline_entry));
    }

    for (const auto& weak_entry : weak_captures) {
        CheckpointWeakCaptureHeader weak_header{};
        weak_header.task_id = weak_entry.task_id;
        weak_header.key_hash = weak_entry.key_hash;
        weak_header.write_tick = weak_entry.write_tick;
        weak_header.data_size = static_cast<uint32_t>(weak_entry.data.size());
        append_bytes(&weak_header, sizeof(weak_header));
        if (!weak_entry.data.empty()) {
            append_bytes(weak_entry.data.data(), weak_entry.data.size());
        }
    }

    return serialized;
}

void RealTimeFramework::restoreCheckpointV2(const std::vector<uint8_t>& checkpoint, common::SharedMemoryHeader* header) {
    if (checkpoint.empty()) {
        return;
    }

    if (checkpoint.size() == _total_state_size) {
        if (_total_state_size > 0) {
            memcpy(_state_buffer.data(), checkpoint.data(), _total_state_size);
        }
        async_logger->info("[Replay] restored legacy state-only checkpoint ({} bytes)", checkpoint.size());
        return;
    }

    if (checkpoint.size() < sizeof(CheckpointV2Header)) {
        async_logger->warn("[Replay] checkpoint is too small for v2 format: {} bytes", checkpoint.size());
        return;
    }

    size_t offset = 0;
    auto read_bytes = [&](void* out, size_t size) -> bool {
        if (offset + size > checkpoint.size()) return false;
        memcpy(out, checkpoint.data() + offset, size);
        offset += size;
        return true;
    };

    CheckpointV2Header v2_header{};
    if (!read_bytes(&v2_header, sizeof(v2_header))) {
        async_logger->warn("[Replay] failed to read checkpoint v2 header");
        return;
    }

    if (v2_header.magic != CHECKPOINT_V2_MAGIC || v2_header.version != CHECKPOINT_V2_VERSION) {
        async_logger->warn("[Replay] unknown checkpoint format (magic=0x{:X}, version={})",
                           v2_header.magic, v2_header.version);
        return;
    }

    if (v2_header.state_size != _total_state_size) {
        async_logger->warn("[Replay] checkpoint state size mismatch (got {}, expected {})",
                           v2_header.state_size, _total_state_size);
        return;
    }

    if (_total_state_size > 0) {
        if (offset + _total_state_size > checkpoint.size()) {
            async_logger->warn("[Replay] checkpoint truncated while reading task state");
            return;
        }
        memcpy(_state_buffer.data(), checkpoint.data() + offset, _total_state_size);
        offset += _total_state_size;
    }

    for (uint32_t i = 0; i < v2_header.timeline_count; ++i) {
        CheckpointTimelineTick timeline_entry{};
        if (!read_bytes(&timeline_entry, sizeof(timeline_entry))) {
            async_logger->warn("[Replay] checkpoint truncated while reading timeline ticks");
            return;
        }

        auto it = _timelines.find(timeline_entry.frequency);
        if (it != _timelines.end() && it->second) {
            it->second->_local_tick_count.store(timeline_entry.local_tick, std::memory_order_release);
            it->second->_start_tick.store(v2_header.global_tick, std::memory_order_release);
        }
    }

    for (uint32_t i = 0; i < v2_header.weak_capture_count; ++i) {
        CheckpointWeakCaptureHeader weak_header{};
        if (!read_bytes(&weak_header, sizeof(weak_header))) {
            async_logger->warn("[Replay] checkpoint truncated while reading weak capture header");
            return;
        }

        if (offset + weak_header.data_size > checkpoint.size()) {
            async_logger->warn("[Replay] checkpoint truncated while reading weak capture payload");
            return;
        }

        if (weak_header.task_id == CHECKPOINT_ARCHIVE_SNAPSHOT_TASK_ID ||
            weak_header.task_id == CHECKPOINT_RT_BOUNDARY_SNAPSHOT_TASK_ID) {
            const uint8_t* payload = checkpoint.data() + offset;
            void* shm_base = _shmContext.getBasePtr();

            if (shm_base) {
                common::DataBlockDescriptor* matched_desc = nullptr;
                for (auto* task : _all_tasks_by_id) {
                    for (const auto& write_req : task->_write_requests) {
                        bool match = false;
                        if (weak_header.task_id == CHECKPOINT_ARCHIVE_SNAPSHOT_TASK_ID) {
                            match = write_req.archive_enabled && (write_req.key_hash == weak_header.key_hash);
                        } else { // CHECKPOINT_RT_BOUNDARY_SNAPSHOT_TASK_ID
                            match = (!task->isNonRt()) && (!write_req.archive_enabled) && (write_req.key_hash == weak_header.key_hash);
                        }

                        if (match) {
                            matched_desc = _shmContext.getDescriptor(write_req.key);
                            break;
                        }
                    }
                    if (matched_desc) break;
                }

                if (matched_desc && weak_header.data_size == matched_desc->data_size && matched_desc->block_count > 0) {
                    auto* bh = common::get_buffer_header(shm_base, matched_desc);
                    auto* bs = common::get_buffer_state(shm_base, matched_desc, 0);
                    auto* db = common::get_data_buffer(shm_base, matched_desc, 0);
                    if (bh && bs && db) {
                        memcpy(db, payload, weak_header.data_size);
                        bs->write_tick.store(weak_header.write_tick, std::memory_order_relaxed);
                        bs->ref_count.store(0, std::memory_order_relaxed);
                        bh->ready_index.store(0, std::memory_order_release);
                        bh->write_index.store(0, std::memory_order_relaxed);
                    }
                }
            }
        } else if (weak_header.task_id < _all_tasks_by_id.size()) {
            auto* task = _all_tasks_by_id[weak_header.task_id];
            auto reader_it = task->_data_readers.find(weak_header.key_hash);
            if (reader_it != task->_data_readers.end() && reader_it->second) {
                const uint8_t* payload = checkpoint.data() + offset;
                if (!reader_it->second->restore_capture_value(payload, weak_header.data_size, weak_header.write_tick)) {
                    async_logger->warn("[Replay] failed to restore weak capture for task={} key_hash={} size={}",
                                       weak_header.task_id, weak_header.key_hash, weak_header.data_size);
                }
            }
        }

        offset += weak_header.data_size;
    }

    common::SharedMemoryHeader* shm_header = header ? header : _shmContext.getHeader();
    if (shm_header) {
        shm_header->framework_tick_count.store(v2_header.global_tick, std::memory_order_release);
    }

    if (_replay_backend) {
        _replay_backend->set_tick_offset(static_cast<int64_t>(v2_header.global_tick));
    }

    async_logger->info("[Replay] restored v2 checkpoint: state={}B, timelines={}, weak_captures={}, global_tick={}",
                       v2_header.state_size,
                       v2_header.timeline_count,
                       v2_header.weak_capture_count,
                       v2_header.global_tick);
}

void RealTimeFramework::handleAction(common::SharedMemoryHeader* header, uint64_t current_tick) {
    auto action = header->requested_action.load(std::memory_order_acquire);
    if (action == FrameworkAction::NONE) return;

    switch (action) {
        case FrameworkAction::START_RECORD: {
            if (header->recording_active.load(std::memory_order_acquire)) {
                async_logger->warn("[Action] Record not available or already active.");
            } else if (header->target_filename[0] == '\0') {
                async_logger->error("[Action] Record filename is empty.");
            } else {
                _record_start_pending = true;
                _record_start_request_tick = current_tick;
                _record_start_pending_filename = header->target_filename;
                async_logger->info("[Action] START_RECORD queued for post-tick boundary: {} (request_tick={})",
                                   _record_start_pending_filename,
                                   _record_start_request_tick);
            }
            break;
        }

        case FrameworkAction::START_REPLAY: {
            if (startReplay(header->replay_target_filename, header)) {
                const uint64_t total_replay_ticks = _replay_backend->getLastTick() + 1;
                const uint64_t marker_tick = header->framework_tick_count.load(std::memory_order_acquire);
                async_logger->info("[Action] replay started: {} ~ {} ticks (marker_tick={})", header->replay_target_filename, total_replay_ticks, marker_tick);
            }
            break;
        }

        case FrameworkAction::STOP_REPLAY: {
            stopReplay(header);
            break;
        }

        case FrameworkAction::STOP_RECORD: {
            stopRecord(header);
            break;
        }

        case FrameworkAction::SET_TASK_ENABLED: {
            uint32_t task_id = header->target_task_id.load(std::memory_order_acquire);
            bool enabled = header->target_task_enabled.load(std::memory_order_acquire);
            
            if (task_id < _all_tasks_by_id.size()) {
                rt::ITask* task = _all_tasks_by_id[task_id];
                bool prev_enabled = task->isEnabled();
                task->setEnabled(enabled);
                
                if (enabled != prev_enabled) {
                    async_logger->info("[Action] Task '{}' (ID: {}) set to {}", 
                                       task->getName(), task_id, enabled ? "ENABLED" : "DISABLED");
                } else {
                    async_logger->debug("[Action] Task '{}' (ID: {}) already {}", 
                                        task->getName(), task_id, enabled ? "enabled" : "disabled");
                }
            } else {
                async_logger->warn("[Action] Invalid task ID: {}", task_id);
            }
            break;
        }

        case FrameworkAction::START_TRACE: {
            // START_TRACE: convenience action to start replay then record
            // using the SHM filename fields. Prefer controller to set
            // `replay_target_filename` and `target_filename` before issuing.
            if (_record_backend && _replay_backend && !header->replaying_active.load() && !header->recording_active.load()) {
                if (header->replay_target_filename[0] == '\0' || header->target_filename[0] == '\0') {
                    async_logger->error("[Action] START_TRACE: replay and record filenames must be set.");
                    break;
                }

                if (!startReplay(header->replay_target_filename, header)) {
                    async_logger->error("[Action] START_TRACE: startReplayNow failed.");
                    break;
                }

                if (!startRecord(header->target_filename, current_tick, header)) {
                    async_logger->error("[Action] START_TRACE: startRecordNow failed.");
                    break;
                }

                header->recording_active.store(true, std::memory_order_release);
                header->replaying_active.store(true, std::memory_order_release);
                _mode = Mode::TRACE;
                async_logger->info("[Action] START_TRACE: Trace started (record={}, replay={}) (mode=TRACE)", header->target_filename, header->replay_target_filename);
            }
            break;
        }

        case FrameworkAction::STOP_TRACE: {
            // stop both if present
            if (_record_backend) {
                _record_backend->shutdown();
                updateBlackboxSlots(true);
                async_logger->info("[Action] STOP_TRACE: Record backend stopped.");
            }
            if (_replay_backend) {
                _replay_backend->shutdown();
                updateBlackboxSlots(true);
                async_logger->info("[Action] STOP_TRACE: Replay backend stopped.");
            }
            header->recording_active.store(false, std::memory_order_release);
            // exiting trace -> live
            _mode = Mode::LIVE;
            break;
        }

        case FrameworkAction::PAUSE_TICK: {
            _paused.store(true, std::memory_order_release);
            async_logger->info("[Action] Scheduler paused.");
            break;
        }

        case FrameworkAction::PLAY_TICK: {
            _paused.store(false, std::memory_order_release);
            async_logger->info("[Action] Scheduler resumed.");
            break;
        }

        case FrameworkAction::PLAY_ONE_TICK: {
            // Request a single tick to run: increment remaining count and
            // unpause the scheduler so it will execute one tick then re-pause.
            _single_step_remaining.fetch_add(1, std::memory_order_release);
            _paused.store(false, std::memory_order_release);
            async_logger->info("[Action] PLAY_ONE_TICK: Scheduler will advance one tick.");
            break;
        }

        // 향후 RELOAD_PARAMETERS 등의 액션 추가 가능
        default: break;
    }

    // 요청 처리 완료 승인
    header->requested_action.store(FrameworkAction::NONE, std::memory_order_release);
}

bool RealTimeFramework::startRecord(const std::string& filename, uint64_t start_tick, common::SharedMemoryHeader* header) {
    if (!_record_backend) {
        async_logger->error("[Action] no record backend attached.");
        return false;
    }

    if (header && header->recording_active.load()) {
        async_logger->warn("[Action] Record not available or already active.");
        return false;
    }

    if (filename.empty()) {
        async_logger->error("[Action] Record filename is empty.");
        return false;
    }

    // 1. snapshot (task states + weak DataReader captures + global/local ticks)
    _checkpoint_buffer = buildCheckpointV2(start_tick);

    // 2. build keymap
    std::map<uint64_t, std::string> keymap;
    for (const auto& task : _all_tasks_by_id) {
        for (const auto& req : task->_write_requests) {
            if (req.archive_enabled) keymap[req.key_hash] = req.key;
        }
    }

    if (!_record_backend->start(filename, rtfw::blackbox::Mode::RECORD)) {
        async_logger->error("startRecordNow: failed to open record file: {}", filename);
        return false;
    }
    std::map<uint64_t, size_t> key_sizes_now;
    for (const auto& task : _all_tasks_by_id) {
        for (const auto& req : task->_write_requests) {
            if (req.archive_enabled) key_sizes_now[req.key_hash] = req.size;
        }
    }
                if (!_record_backend->initialize_metadata(keymap, _base_frequency, key_sizes_now, _checkpoint_buffer, start_tick)) {
        async_logger->error("startRecordNow: initialize_metadata failed.");
        return false;
    }

    // wire cache slots
    updateBlackboxSlots(false);

    if (header) {
        header->recording_active.store(true, std::memory_order_release);
    }
    _mode = Mode::RECORDING;
    return true;
}

bool RealTimeFramework::startReplay(const std::string& filename, common::SharedMemoryHeader* header) {
    if (!_replay_backend) {
        async_logger->error("[Action] no replay backend attached.");
        return false;
    }
    if (header && header->replaying_active.load()) {
        async_logger->warn("[Action] Replay not available or already active.");
        return false;
    }
    if (filename.empty()) {
        async_logger->error("[Action] Replay filename is empty.");
        return false;
    }
    std::map<uint64_t, std::string> keymap;
    for (const auto& task : _all_tasks_by_id) {
        for (const auto& req : task->_write_requests) {
            if (req.archive_enabled) keymap[req.key_hash] = req.key;
        }
    }

    if (!_replay_backend->start(filename, rtfw::blackbox::Mode::REPLAY)) {
        async_logger->error("[Action] failed to open replay file: {}", filename);
        return false;
    }
    std::map<uint64_t, size_t> key_sizes_now;
    for (const auto& task : _all_tasks_by_id) {
        for (const auto& req : task->_write_requests) {
            if (req.archive_enabled) key_sizes_now[req.key_hash] = req.size;
        }
    }
    if (!_replay_backend->initialize_metadata(keymap, _base_frequency, key_sizes_now)) {
        async_logger->error("[Action] initialize_metadata failed.");
        return false;
    }

    updateBlackboxSlots(false);

    std::vector<uint8_t> initial_checkpoint;
    if (_replay_backend->get_initial_checkpoint(initial_checkpoint)) {
        restoreCheckpointV2(initial_checkpoint, header);
    }

    // async_logger->info("[Action] replay backend initialized from {}", filename);
    if (header) {
        header->replaying_active.store(true, std::memory_order_release);
    }
    _mode = Mode::SIMULATION;
    return true;
}

bool RealTimeFramework::startTrace(const std::string& record_filename, const std::string& replay_filename) {
    // Start replay first to restore checkpoint, then start recording from the
    // restored state so both streams are aligned.
    if (!_record_backend || !_replay_backend) {
        async_logger->error("startTraceNow: both record and replay backends must be attached.");
        return false;
    }

    if (!startReplay(replay_filename, nullptr)) {
        async_logger->error("startTraceNow: startReplayNow failed.");
        return false;
    }

    if (!startRecord(record_filename, 0, nullptr)) {
        async_logger->error("startTraceNow: startRecordNow failed.");
        return false;
    }

    _mode = Mode::TRACE;
    async_logger->info("startTraceNow: trace started (record={}, replay={})", record_filename, replay_filename);
    return true;
}

void RealTimeFramework::stopRecord(common::SharedMemoryHeader* header) {
    header->recording_active.store(false, std::memory_order_release);
    if (_record_backend) {
        // Clear record slots from proxies (detach) before backend clears its cache
        updateBlackboxSlots(true);
        _record_backend->shutdown();
    }
    // Stopped recording -> return to LIVE unless replay is active or TRACE
    if (!_record_backend) _mode = Mode::LIVE;
    async_logger->info("[Action] Record stopped. ({} ticks)", _record_backend->getLastTick()+1);
}

void RealTimeFramework::stopReplay(common::SharedMemoryHeader* header) {
    header->replaying_active.store(false, std::memory_order_release);
    if (_replay_backend) {
        // Clear replay slots from proxies to avoid stale pointers before backend clears cache
        updateBlackboxSlots(true);
        _replay_backend->shutdown();
        async_logger->info("[Action] Replay stopped.");
    }
    // If replay stopped, return to LIVE mode unless recording is active
    if (!_shmContext.getHeader()->recording_active.load()) _mode = Mode::LIVE;
}