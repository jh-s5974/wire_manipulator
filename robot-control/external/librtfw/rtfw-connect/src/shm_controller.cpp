// rtfw-client/shm_controller.cpp
#include "rtfw_connect/shm_controller.h"
#include <cstring>
#include <chrono>
#include <thread>

using namespace rtfw::common;
using namespace rtfw::connect;

SharedMemoryController::SharedMemoryController(void* shm_base_ptr)
    : _shm_base_ptr(shm_base_ptr), _header(nullptr), _stats_array(nullptr), _task_count(0) {
    if (_shm_base_ptr) {
        _header = static_cast<SharedMemoryHeader*>(_shm_base_ptr);
        // 여기서도 매직 넘버/상태 체크로 유효성 검사
        if (_header->shm_state.load() == ShmState::RUNNING) {
            _stats_array = reinterpret_cast<TaskStats*>(static_cast<char*>(_shm_base_ptr) + _header->task_stats_array_offset);
            _task_count = _header->task_stats_count;
        }
        buildParamInfoCache();
    } else {
        _header = nullptr;
    }
}

bool SharedMemoryController::clear_task_overrun_flag(uint32_t task_id) {
    if (!_stats_array || task_id >= _task_count) {
        return false;
    }
    // 명시적으로 쓰기 작업을 수행
    _stats_array[task_id].has_overrun.store(false, std::memory_order_relaxed);
    return true;
}

void SharedMemoryController::setLogLevel(LogLevel level) {
    _header->shared_log_level.store(level);
}

void SharedMemoryController::buildParamInfoCache() {
    if (!_header) return;
    
    // 공유 메모리에서 ParameterInfo 배열의 위치와 개수를 가져옴
    ParameterInfo* info_array = reinterpret_cast<ParameterInfo*>((char*)_shm_base_ptr + _header->param_info_array_offset);
    size_t info_count = _header->param_info_count;

    // 모든 정보를 맵에 저장하여 O(log N) 시간 복잡도로 검색 가능하게 함
    for (size_t i = 0; i < info_count; ++i) {
        const auto& info = info_array[i];
        _param_info_cache[info.key] = &info;
    }
}

const ParameterInfo* SharedMemoryController::findParamInfo(std::string_view key) const {
    auto it = _param_info_cache.find(std::string(key));
    if (it != _param_info_cache.end()) {
        return it->second;
    }
    return nullptr;
}

// ============================================================================
// Scheduler Control
// ============================================================================
void SharedMemoryController::pauseTick() {
    if (!_header) return;
    _header->requested_action.store(FrameworkAction::PAUSE_TICK, std::memory_order_release);
}

void SharedMemoryController::playTick() {
    if (!_header) return;
    _header->requested_action.store(FrameworkAction::PLAY_TICK, std::memory_order_release);
}

void SharedMemoryController::playOneTick() {
    if (!_header) return;
    _header->requested_action.store(FrameworkAction::PLAY_ONE_TICK, std::memory_order_release);
}

// ============================================================================
// Recording Control
// ============================================================================
void SharedMemoryController::startRecord(const std::string& path) {
    if (!_header) return;
    
    strncpy(_header->target_filename, path.c_str(), sizeof(_header->target_filename) - 1);
    _header->target_filename[sizeof(_header->target_filename) - 1] = '\0';
    _header->requested_action.store(FrameworkAction::START_RECORD, std::memory_order_release);
}

void SharedMemoryController::stopRecord() {
    if (!_header) return;
    _header->requested_action.store(FrameworkAction::STOP_RECORD, std::memory_order_release);
}

// ============================================================================
// Replay Control
// ============================================================================
void SharedMemoryController::startReplay(const std::string& path) {
    if (!_header) return;
    
    strncpy(_header->replay_target_filename, path.c_str(), sizeof(_header->replay_target_filename) - 1);
    _header->replay_target_filename[sizeof(_header->replay_target_filename) - 1] = '\0';
    _header->requested_action.store(FrameworkAction::START_REPLAY, std::memory_order_release);
}

void SharedMemoryController::stopReplay() {
    if (!_header) return;
    _header->requested_action.store(FrameworkAction::STOP_REPLAY, std::memory_order_release);
}

// ============================================================================
// Trace Control
// ============================================================================
void SharedMemoryController::startTrace(const std::string& replay_path, const std::string& record_path) {
    if (!_header) return;
    
    strncpy(_header->replay_target_filename, replay_path.c_str(), sizeof(_header->replay_target_filename) - 1);
    _header->replay_target_filename[sizeof(_header->replay_target_filename) - 1] = '\0';
    
    strncpy(_header->target_filename, record_path.c_str(), sizeof(_header->target_filename) - 1);
    _header->target_filename[sizeof(_header->target_filename) - 1] = '\0';
    
    _header->requested_action.store(FrameworkAction::START_TRACE, std::memory_order_release);
}

void SharedMemoryController::stopTrace() {
    if (!_header) return;
    _header->requested_action.store(FrameworkAction::STOP_TRACE, std::memory_order_release);
}

// ============================================================================
// Task Control
// ============================================================================
bool SharedMemoryController::setTaskEnabled(uint32_t task_id, bool enabled) {
    if (!_header) return false;
    
    if (task_id >= _task_count) {
        return false;
    }
    
    _header->target_task_id.store(task_id, std::memory_order_release);
    _header->target_task_enabled.store(enabled, std::memory_order_release);
    _header->requested_action.store(FrameworkAction::SET_TASK_ENABLED, std::memory_order_release);
    
    return true;
}

// ============================================================================
// Action Completion Monitoring
// ============================================================================
bool SharedMemoryController::isActionBusy() const {
    if (!_header) return false;
    return _header->requested_action.load(std::memory_order_acquire) != FrameworkAction::NONE;
}

bool SharedMemoryController::waitForActionComplete(int timeout_ms) {
    if (!_header) return false;
    
    int timeout_count = 0;
    const int max_timeout = timeout_ms / 10;  // 10ms sleep interval
    
    while (isActionBusy()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        timeout_count++;
        if (timeout_count > max_timeout) {
            return false;  // Timeout occurred
        }
    }
    
    return true;  // Action completed successfully
}