#include "rtfw/timeline.h"
#include "rtfw/rt_framework.h" // RealTimeFramework의 전체 정의를 위해 필요

#include <iostream> // 로깅용
// #include "rtfw_common/logging_context.h"
#include "rtfw/shm_ringbuffer_sink.h"
// #include "spdlog/spdlog.h"
// #include "spdlog/async.h"

// TODO: 설계 개선
//
//  // [신규] 두 개의 의존성 카운터 세트 (핑퐁 버퍼처럼 사용)
//     std::vector<std::atomic<int>> _dependency_counters_A;
//     std::vector<std::atomic<int>> _dependency_counters_B;
//     // 현재 실행 중인 세대가 어떤 카운터 세트를 사용하는지 가리키는 포인터
//     std::atomic<std::vector<std::atomic<int>>*> _current_deps_ptr;
//     // 다음 Tick을 준비하기 위한 카운터 세트를 가리키는 포인터
//     std::vector<std::atomic<int>>* _next_deps_ptr;
//
// Timeline::tick_start(N) (새로운 주기 준비):
//     가장 먼저, 이전 주기(N-1)가 성공했는지 확인합니다.
//         _current_deps_ptr가 가리키는 카운터 배열(N-1 세대용)을 순회하며, 하나라도 카운터가 0보다 큰 태스크가 있는지 확인합니다.
//         만약 있다면? -> 데드라인 미스 발생!
//             이번 주기(N)의 실행을 통째로 취소하고 즉시 return합니다. (오버런 처리)
//             _current_deps_ptr가 가리키는 카운터들을 리셋하지 않습니다. (실패 상태 유지)
//     이전 주기가 성공했다면?
//         _next_deps_ptr가 가리키는 "준비용" 카운터 배열(N 세대용)을 초기화합니다. (모든 RT 태스크의 초기 의존성 카운트로 설정)
//         "세대 교체": _current_deps_ptr와 _next_deps_ptr의 포인터를 서로 맞바꿉니다(swap). 이제 _current_deps_ptr는 tick N을 위한 새로운 카운터 세트를 가리킵니다.
//         _current_deps_ptr를 기반으로 루트 태스크를 찾아 큐에 넣습니다.
//         _capture_proxy를 일괄 호출합니다.
// Timeline::onTaskFinished (현재 세대 업데이트):
//     후속 태스크의 의존성 카운터를 감소시킬 때, 오직 _current_deps_ptr가 가리키는 현재 세대의 카운터만 건드립니다. _next_deps_ptr는 절대 건드리지 않습니다.
// Timeline::tick_end(N) (정리):
//     데드라인 미스가 없었을 때만 _release_proxy를 일괄 호출합니다. (데드라인 미스 여부는 tick_start(N+1)에서 판정)
//     배치 커밋을 수행합니다


using namespace rtfw::common;

namespace rtfw::internal {
    extern std::shared_ptr<spdlog::logger> async_logger;

Timeline::Timeline(int frequency,
                   uint64_t base_frequency,
                   std::vector<rt::ITask*> tasks,
                   rtfw::RealTimeFramework* owner_framework)
    : _frequency(frequency),
      _base_frequency(base_frequency),
      _owner_framework(owner_framework),
      _tasks_in_timeline(std::move(tasks)), // 벡터 소유권 이전
      _remained_task_count(0)
{
    _name = "T" + std::to_string(_frequency) + "Hz";
    const size_t total_task_count = _owner_framework->getTotalTaskCount();
    _dependency_counters = std::make_unique<std::atomic<int>[]>(total_task_count);

    // 각 태스크에 자신이 어떤 타임라인에 속하는지 알려줍니다.
    for (rt::ITask* task : _tasks_in_timeline) {
        task->_setOwnerTimeline(this);
    }
    _rt_task_count_in_timeline = std::count_if(
        _tasks_in_timeline.begin(), _tasks_in_timeline.end(),
        [](const rt::ITask* t){ return !t->isNonRt(); }
    );
}

void Timeline::initialize() {
    // 이 타임라인 내부의 루트 태스크(내부 의존성이 없는 태스크)를 찾아 캐싱합니다.
    _tasks_to_run_this_tick.reserve(_tasks_in_timeline.size());
    _initial_executable_tasks.reserve(_tasks_in_timeline.size());
    _local_tick_count.store(0, std::memory_order_relaxed);
    TimelineStats* stats = _owner_framework->getTimelineStats(this->_frequency);
    stats->frequency = _frequency;
}

void Timeline::applyEnableMaskAndCascade() {
    // enabled=false인 태스크들의 downstream strong chain을 순회하며 should_run=false로 전파
    // BFS를 사용하여 전파
    std::vector<rt::ITask*> queue;
    
    // enabled=false인 태스크들을 초기 큐에 추가
    for (rt::ITask* task : _tasks_in_timeline) {
        if (!task->isEnabled()) {
            task->_should_run = false;
            queue.push_back(task);
        }
    }
    
    // BFS로 strong dependents에게 전파
    size_t idx = 0;
    while (idx < queue.size()) {
        rt::ITask* current = queue[idx++];
        
        // 중앙 저장소의 _task_dependents를 직접 사용
        const auto& dependents = _owner_framework->getInternalDependents(current->getID());
        for (rt::ITask* dependent : dependents) {
            // 이미 should_run=false라면 이미 방문한 것이므로 스킵
            if (dependent->shouldRun()) {
                dependent->_should_run = false;
                queue.push_back(dependent);
            }
        }
    }
}

void Timeline::markOverrunCascadeSkipped(rt::ITask* overrun_task) {
    /**
     * [NOTE] overrun 경로에서는 더 이상 직접 호출되지 않음.
     * overrun task는 tick_end에서 setEnabled(false)로 처리되며,
     * 다음 tick의 applyEnableMaskAndCascade()가 cascade를 자동으로 담당.
     * 
     * 이 함수는 다른 곳에서 강제 cascade skip이 필요한 경우를 위해 남겨둠.
     */
    
    // BFS: Overrun task에 의존하는 모든 task 찾기
    std::vector<rt::ITask*> cascade_queue;
    std::set<uint32_t> visited;
    
    cascade_queue.push_back(overrun_task);
    visited.insert(overrun_task->getID());
    
    size_t idx = 0;
    while (idx < cascade_queue.size()) {
        rt::ITask* current = cascade_queue[idx++];
        
        // 이 task의 dependents (current에 의존하는 tasks) 찾기
        const auto& dependents = _owner_framework->getInternalDependents(current->getID());
        
        for (rt::ITask* dependent : dependents) {
            if (visited.find(dependent->getID()) != visited.end()) {
                continue;  // 이미 처리함
            }
            visited.insert(dependent->getID());
            
            // 이 timeline에 속한 task만 처리 (cross-timeline 의존성은 별도)
            // dependent가 이 타임라인에 속하는지 확인
            bool in_this_timeline = false;
            for (rt::ITask* timeline_task : _tasks_in_timeline) {
                if (timeline_task->getID() == dependent->getID()) {
                    in_this_timeline = true;
                    break;
                }
            }
            
            if (in_this_timeline) {
                // 이 dependent의 의존성 카운트를 0으로 설정
                // (실제로는 upstream이 완료되지 않았지만, cascade skip 처리)
                _dependency_counters[dependent->getID()].store(0, std::memory_order_release);
                
                // 로깅
                async_logger->warn("Overrun cascade: Task '{}' also skipped (downstream of '{}')",
                                 dependent->getName(), overrun_task->getName());
                
                // 더 깊은 downstream도 처리하기 위해 큐에 추가
                cascade_queue.push_back(dependent);
            }
        }
    }
    
    async_logger->info("Overrun cascade complete: {} tasks affected", visited.size() - 1);
}

void Timeline::tick_start(uint64_t current_global_tick) {
    // 1. 자신의 주기에 맞춰 실행될 차례인지 확인합니다.
    if (current_global_tick % (_base_frequency / _frequency) != 0) {
        return; // 이번 tick은 건너뜀
    }


    SharedMemoryHeader* header = _owner_framework->_shmContext.getHeader();
    if (!header) {
        std::cerr << "CRITICAL: SharedMemoryHeader is null in Timeline." << std::endl;
        return;
    }

    _current_tick_start_time_ns = header->current_tick_start_time_ns.load(std::memory_order_acquire);
    _start_tick.store(header->framework_tick_count.load(std::memory_order_acquire), std::memory_order_release);

    async_logger->trace("Tick {} Started (Gen: {}).", _local_tick_count.load(std::memory_order_acquire), _local_tick_count.load(std::memory_order_acquire));

    _tasks_to_run_this_tick.clear();
    _initial_executable_tasks.clear();

    // 2-1. 모든 태스크의 tick 상태를 리셋
    for (rt::ITask* task : _tasks_in_timeline) {
        if (task->isNonRt() && task->_exec_state == rt::ExecState::DeadlineMissSelf) {
            // 실행중인 Non-RT 태스크는 상태 리셋 없이 그대로 둠
            continue;
        } else {
            task->resetTickState();
        }
    }
    
    // 2-2. enable 마스크를 적용하고 strong chain으로 전파
    applyEnableMaskAndCascade();

    // applyEnableMaskAndCascade 이후 shouldRun()==true인 task만 실행 대상
    // 동시에 모든 task의 의존성 카운터를 일괄 초기화
    // (disabled/cascade-skip된 task 포함 → stale upstream 완료로 카운터가 음수로 오염되는 것 방지)
    for (rt::ITask* task : _tasks_in_timeline) {
        const uint32_t task_id = task->getID();
        _dependency_counters[task_id].store(
            _owner_framework->getInitialInternalDependencyCount(task_id),
            std::memory_order_release);

        if (task->shouldRun()) {
            if (task->isNonRt() && task->_exec_state == rt::ExecState::DeadlineMissSelf) {
                // 실행중인 Non-RT 태스크는 should_run이 true라도 이번 tick에는 실행되지 않도록 처리
                continue;
            } else {
                _tasks_to_run_this_tick.push_back(task);
            }
        }
    }

    // 2. 이번 Tick 실행을 위해 상태를 리셋합니다.
    size_t rt_tasks_to_run_count = std::count_if(
        _tasks_to_run_this_tick.begin(), _tasks_to_run_this_tick.end(),
        [](const rt::ITask* t){ return !t->isNonRt(); }
    );
    _remained_task_count.store(rt_tasks_to_run_count, std::memory_order_release);

    for (rt::ITask* task : _tasks_to_run_this_tick) {

        // _set_execution_local_tick은 enqueueTask()에서 설정됨
        // (여기서 설정 시 stale task의 generation이 덮어써지는 race 존재)
        const uint32_t task_id = task->getID();
        
        // should_run==true이고 의존성이 없는 태스크만 ready 큐에 추가
        int initial_deps = _owner_framework->getInitialInternalDependencyCount(task_id);
        if (initial_deps == 0) {
            _initial_executable_tasks.push_back(task);
        }

        // 외부 의존성이 있는 경우, 프록시가 데이터 스냅샷을 캡처하도록 합니다.
        const auto& external_keys = _owner_framework->getExternalDependencyKeys(task_id);
        if (!external_keys.empty()) {

            if (static_cast<LogLevel>(async_logger->level()) == LogLevel::TRACE) {
                std::string msg;
                msg += "[CAPTURE] '";
                msg += task->getName();
                msg += " << ";
                for (auto& key: external_keys) {
                    msg += task->_data_readers[key]->key();
                    msg += " ";
                }                
                async_logger->trace("{}", msg);
            }
        }
        task->_capture_proxy(external_keys);
    }

    // 3. 캐싱해 둔 루트 태스크들을 워커 큐에 넣어 실행을 시작합니다.
    for (rt::ITask* root_task : _initial_executable_tasks) {
        root_task->_became_ready.store(true, std::memory_order_release);
        _owner_framework->enqueueTask(root_task);
    }
}

void Timeline::tick_end(uint64_t current_global_tick) {
    if ((current_global_tick+1) % (_base_frequency / _frequency) != 0) {
        return; // 이번 tick은 건너뜀
    }

    _local_tick_count.fetch_add(1, std::memory_order_acq_rel);

    // ExecState 분류
    for (rt::ITask* task : _tasks_in_timeline) {
        if (!task->shouldRun()) {
            // should_run=false → NotScheduled
            task->_exec_state = rt::ExecState::NotScheduled;
        } else if (task->wasExecuted()) {
            // executed=true → Executed
            task->_exec_state = rt::ExecState::Executed;
        } else if (task->becameReady()) {
            // became_ready=true이지만 executed=false → DeadlineMissSelf
            // (ready 상태가 되었지만 실행이 tick을 넘김)
            task->_exec_state = rt::ExecState::DeadlineMissSelf;
        } else {
            // should_run=true이지만 became_ready=false → DeadlineMissUpstream
            // (실행 대상이었지만 upstream dependency가 완료되지 않아 ready가 안 됨)
            task->_exec_state = rt::ExecState::DeadlineMissUpstream;
        }
    }

    for (rt::ITask* task : _tasks_to_run_this_tick) {
        if (task->isNonRt())
            continue;

        // _exec_state는 이미 위에서 분류됨 (같은 tick_end, 같은 스레드)
        if (task->_exec_state == rt::ExecState::Executed) {
            task->_commit_proxy();
        } else {
            if (task->_exec_state == rt::ExecState::DeadlineMissSelf) {
                // DeadlineMissSelf = becameReady=true (enqueued) but wasExecuted=false
                // is_busy는 enqueueTask()에서 worker 시작 전에 세트되므로 tick_end 시점에
                // "실행 중" vs "큐 대기 중"을 구분하지 못함 → ExecState 기준으로 통일 처리
                //
                // NOTE: onOverrun()은 execute()와 동시 호출될 수 있음
                //       구현 시 task 내부 상태를 변경하지 말 것 (view-only으로 설계)
                TaskStats* task_stats = _owner_framework->getTaskStats(task->getID());
                task_stats->has_overrun.store(true, std::memory_order_relaxed);
                bool can_recover = task->onOverrun();
                task->setEnabled(false);
                if (can_recover) {
                    task->_pending_overrun_recovery.store(true, std::memory_order_release);
                    task_stats->overrun_recovery_count.fetch_add(1, std::memory_order_relaxed);
                    async_logger->warn("Overrun: Task '{}' missed deadline, will recover after completion", task->getName());
                } else {
                    async_logger->error("Overrun: Task '{}' permanently disabled", task->getName());
                }
            } else {
                async_logger->warn("Deadline miss ({}): skipping commit for task '{}'",
                    task->_exec_state == rt::ExecState::DeadlineMissSelf ? "self" : "upstream",
                    task->getName());
            }
        }
        
        if (static_cast<LogLevel>(async_logger->level()) == LogLevel::TRACE) {
            std::string msg;
            msg += "[RELEASE] '";
            msg += task->getName();
            msg += " << ";
            for (auto& proxy: task->_held_proxies) {
                msg += proxy->key();
                msg += " ";
            }                
            async_logger->trace("{}", msg);
        }

        // 모든 task의 프록시 해제 (disabled task도 포함)
        // → buffer slot 즉시 회수, stale write 방지
        task->_release_proxy();
    }

    async_logger->trace("Tick {} End.", _local_tick_count.load(std::memory_order_acquire)-1);

    if (_rt_task_count_in_timeline == 0) return;
    
    // --- 통계 집계 시작 ---
    long long busy_ns = 0;
    
    // 1. 가장 늦게 끝난 RT 태스크의 완료 시간을 찾는다
    uint64_t max_completion_offset = 0;
    for (rt::ITask* task : _tasks_in_timeline) {
        if (task->isNonRt()) continue; // RT 태스크만 통계에 포함

        TaskStats* task_stats = _owner_framework->getTaskStats(task->getID());
        uint64_t offset = task_stats->last_completion_offset_ns.load(std::memory_order_relaxed);
        if (offset > max_completion_offset) {
            max_completion_offset = offset;
        }
    }
    busy_ns = max_completion_offset;
    
    auto remained_task_count = _remained_task_count.load(std::memory_order_acquire);
    // 2. 데드라인 미스 확인 (remained_task_count는 RT 태스크만 센다고 가정)
    if (remained_task_count > 0) {
        // 데드라인 미스가 발생했으므로, busy_ns를 주기의 100%로 설정
        busy_ns = 1'000'000'000L / _frequency;
    }

    // 3. 공유 메모리의 TimelineStats에 최종 결과 기록
    TimelineStats* stats = _owner_framework->getTimelineStats(this->_frequency);
    if (stats) {
        if (remained_task_count > 0) {
            async_logger->error("DEADLINE MISS - {} tasks.", remained_task_count);
            stats->deadline_miss_count.fetch_add(1, std::memory_order_relaxed);
        }
        
        stats->total_busy_ns.fetch_add(busy_ns, std::memory_order_relaxed);
        stats->total_squared_busy_ns.fetch_add(busy_ns * busy_ns, std::memory_order_relaxed);
        stats->tick_count.fetch_add(1, std::memory_order_relaxed);

        // max_busy_ns 업데이트
        long long prev_max = stats->max_busy_ns.load(std::memory_order_relaxed);
        while (busy_ns > prev_max) {
            if (stats->max_busy_ns.compare_exchange_weak(prev_max, busy_ns)) break;
        }
    }

}

void Timeline::onTaskFinished(rt::ITask* finished_task) {
    // [CRITICAL] Generation (local_tick) check: 다른 generation의 task 콜백은 무시
    // Overrun task의 경우: 이미 tick_end()에서 _release_proxy() 호출됨
    uint64_t current_local_tick = _local_tick_count.load(std::memory_order_acquire);

    if (finished_task->isNonRt()) {
        finished_task->_commit_proxy();
        // proxies도 여기서 release 해야되지 않나?
        finished_task->_release_proxy();
        return;
    }
    
    if (finished_task->getExecutionLocalTick() != current_local_tick) {
        {
            // Stale generation task (overrun 후 나중에 완료)
            async_logger->warn("Task '{}' finished from stale generation (local_tick: {} vs current: {})",
                            finished_task->getName(),
                            finished_task->getExecutionLocalTick(),
                            current_local_tick);

            // Recoverable overrun: 완료됨으므로 다음 tick부터 re-enable
            // exchange(false): 플래그를 원자적으로 클리어하면서 이전 값을 읽음 (중복 re-enable 방지)
            if (finished_task->_pending_overrun_recovery.exchange(false, std::memory_order_acq_rel)) {
                finished_task->setEnabled(true);
                async_logger->info("Task '{}' re-enabled after overrun recovery", finished_task->getName());
            }            
        }

        return;  // ← proxies는 이미 release됨 (tick_end에서)
    }

    // 1. 이 타임라인의 후속 태스크(내부 의존성)에 실행 완료를 전파합니다.
    const auto& dependents = _owner_framework->getInternalDependents(finished_task->getID());

    for (rt::ITask* dependent_task : dependents) {
        // fetch_sub는 원자적으로 값을 1 감소시키고 '감소 전의 값'을 반환합니다.
        // 따라서 반환값이 1이면, 이제 카운터는 0이 되었다는 의미입니다.

        int prev_count = _dependency_counters[dependent_task->getID()].fetch_sub(1, std::memory_order_acq_rel);
        // 로깅이 활성화된 경우에만 상세 정보 출력
        async_logger->trace("  -> For '{}' (ID: {}) dpes decremented to {}.", dependent_task->getName(), dependent_task->getID(), prev_count-1);

        if (prev_count == 1) {
            // remaining_deps==0이 되었고 should_run==true인 경우에만 큐에 추가
            if (dependent_task->shouldRun()) {
                dependent_task->_became_ready.store(true, std::memory_order_release);
                _owner_framework->enqueueTask(dependent_task);
            }
        }
    }
    
    auto prev_count = _remained_task_count.fetch_sub(1, std::memory_order_acq_rel);
    // 2. 이번 tick에 남은 태스크 수를 감소시키고, 0이 되면 타임라인 종료 처리를 합니다.
    if (prev_count == 1) {
        // 타임라인의 모든 태스크가 완료되었습니다.
        for (rt::ITask* task : _tasks_in_timeline) {

        }
    }

    if (prev_count == 0) {
        std::string msg;
        msg += "OVERFLOW! - ";
        msg += std::to_string(_remained_task_count.load(std::memory_order_acquire));
        async_logger->critical("{}", msg);
        throw std::runtime_error(msg);
    }
}

bool Timeline::isPending(uint64_t current_global_tick) {
    if ((current_global_tick+1) % (_base_frequency / _frequency) != 0) {
        return false; // 이번 tick은 건너뜀
    }

    return _remained_task_count.load(std::memory_order_acquire) > 0;
}
} // namespace rtfw