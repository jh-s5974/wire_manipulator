#pragma once

#include <vector>
#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>

#include "task.h" // rt::ITask 사용을 위해 필요

// RealTimeFramework의 전체 정의 대신 전방 선언을 사용하여 컴파일 의존성을 낮춤
namespace rtfw {
    class RealTimeFramework;
}

namespace rtfw::internal {

/**
 * @class Timeline
 * @brief 특정 주파수를 가진 태스크 그룹의 실행 계획, 상태, 의존성 전파를 관리하는 클래스.
 *
 * 각 Timeline은 독립적으로 동작하지만, 의존성 데이터는 RealTimeFramework의 중앙 저장소를
 * 참조하여 O(1) 성능을 보장합니다.
 */
class Timeline {
public:
    // --- 생성자 및 소멸자 ---
    Timeline(int frequency,
             uint64_t base_frequency,
             std::vector<rt::ITask*> tasks,
             rtfw::RealTimeFramework* owner_framework);

    ~Timeline() = default;

    // --- 복사/이동 금지 ---
    Timeline(const Timeline&) = delete;
    Timeline& operator=(const Timeline&) = delete;
    Timeline(Timeline&&) = delete;
    Timeline& operator=(Timeline&&) = delete;

    // --- Public API (프레임워크가 호출) ---
    const std::string& getName() { return _name; }
    /**
     * @brief Timeline을 초기화합니다.
     * 이 함수는 프레임워크가 모든 의존성 분석을 마친 후 호출되어야 합니다.
     * 타임라인 내부의 루트 태스크를 식별하는 등의 준비 작업을 수행합니다.
     */
    void initialize();

    /**
     * @brief 프레임워크의 메인 스케줄러가 매 글로벌 tick마다 호출.
     *        타임라인의 주기가 시작되었는지 확인하고 작업을 준비/시작합니다.
     * @param current_global_tick 현재 프레임워크의 글로벌 tick.
     */
    void tick_start(uint64_t current_global_tick);

    /**
     * @brief 프레임워크의 메인 스케줄러가 다음 tick이 시작되기 직전에 호출.
     *        이번 주기의 작업이 모두 완료되었는지 확인하고 통계를 집계합니다.
     */
    void tick_end(uint64_t current_global_tick);

    // bool check_idle() {
    //     return _remained_task_count.load(std::memory_order_relaxed) == 0;
    // }

    /**
     * @brief 워커 스레드가 태스크 실행을 완료했을 때 호출하는 콜백 함수.
     * @param finished_task 방금 실행이 완료된 태스크의 포인터.
     */
    void onTaskFinished(rt::ITask* finished_task);

    bool isPending(uint64_t current_global_tick);

private:
    friend class rtfw::RealTimeFramework;

    // enabled=false인 태스크들의 downstream strong chain을 순회하며 should_run=false로 전파
    void applyEnableMaskAndCascade();

    /**
     * @brief Overrun된 task의 cascade effect를 처리합니다.
     * 이 task에 의존하는 downstream tasks의 의존성 카운트를 사전에 0으로 설정하여
     * 데드락 및 잘못된 remained_task_count를 방지합니다.
     * @param overrun_task Overrun이 감지된 task
     */
    void markOverrunCascadeSkipped(rt::ITask* overrun_task);

    // --- 멤버 변수 ---

    // 1. 기본 정보 및 프레임워크 참조
    std::string _name;
    const int _frequency;
    const uint64_t _base_frequency;
    rtfw::RealTimeFramework* _owner_framework; // 워커 큐, 중앙 데이터 접근용

    // 2. 소유한 태스크 정보
    const std::vector<rt::ITask*> _tasks_in_timeline; // 이 타임라인에 속한 모든 태스크
    std::vector<rt::ITask*> _tasks_to_run_this_tick;
    std::vector<rt::ITask*> _initial_executable_tasks; 
    size_t _rt_task_count_in_timeline;

    // 3. 런타임 상태 (매 tick마다 리셋됨)
    // 이 벡터의 크기는 프레임워크의 '전체' 태스크 수와 같으며, task_id로 인덱싱됩니다.
    std::unique_ptr<std::atomic<int>[]> _dependency_counters;
    std::atomic<size_t> _remained_task_count;

    bool _is_my_turn; // 이번 글로벌 tick이 나의 실행 주기인지 여부
    std::atomic<uint64_t> _local_tick_count;
    std::atomic<uint64_t> _start_tick;
    uint64_t _current_tick_start_time_ns;
};

} // namespace rtfw::internal