#pragma once

#include <vector>
#include <string>
#include <string_view>
#include <functional>
#include <atomic>
#include <optional>
#include <typeindex>
#include <chrono>

#include "rtfw/task_proxies.h"
// #include "rtfw/parameter.h"
#include "rtfw_common/shm_layout.h"
#include "rtfw_common/type_utils.h"
#include "spdlog/spdlog.h"


// 전방 선언
namespace rtfw {
    class RealTimeFramework;
    namespace internal {
        class Timeline;
        class ParameterManager;
        extern std::shared_ptr<spdlog::logger> async_logger;
    }
};

namespace rtfw::rt {
    enum class ExecState : uint8_t {
        Executed = 0,              // 정상 실행됨
        DeadlineMissSelf,          // 자기 실행이 tick을 넘김
        DeadlineMissUpstream,      // 실행 대상이었지만 ready가 되지 못함
        NotScheduled               // 마스크/strong chain으로 실행 대상이 아님
    };

    class ITask {
    protected:
        ITask();
        virtual ~ITask();

        static std::vector<ITask*>& taskStack();
        static thread_local std::vector<ITask*> _task_stack;
    public:
        ITask(const ITask&) = delete;
        ITask& operator=(const ITask&) = delete;

        virtual const char* getName() const = 0;
        virtual size_t getStateSize() const { return 0; } // 기본은 상태 없음
        virtual void initialize(void* state_ptr) {}
        virtual void execute(void* state_ptr) = 0;
        virtual void warmup() const {}
        
        /**
         * Overrun 콜백: Task가 이전 주기에서 완료되지 못하고 다음 주기가 도래했을 때 호출
         * 
         * 반환값:
         * - true: 복구 가능 (graceful degradation 후 계속 실행)
         * - false: 복구 불가능 (task 영구 disabled)
         * 
         * 기본 동작: 복구 불가능 (false) → task가 disabled됨
         */
        virtual bool onOverrun() {
            return false;  // 기본: 복구 불가능
        }
        
        // 통계 출력 함수 (시그니처는 변경 없음)
        void printStats(const common::TaskStats& stats) const;

        uint32_t getID() const { return _id; }
        uint32_t getFrequency() const { return _frequency; }
        int getAffinity() const { return _affinity; }
        bool isNonRt() const { return _is_non_rt; }
        uint64_t getCurrentTick() const { return _current_tick_in_cycle; }
        uint64_t getExecutionLocalTick() const {return _execution_local_tick; }
        std::shared_ptr<spdlog::logger> getLogger() {
            return internal::async_logger;
        }

        // 태스크 활성화 상태 관리
        // NOTE: _enabled는 RT 스레드(tick_start)와 worker 스레드(onTaskFinished)에서
        //       동시에 접근되므로 atomic이어야 함
        void setEnabled(bool enabled) { _enabled.store(enabled, std::memory_order_release); }
        bool isEnabled() const { return _enabled.load(std::memory_order_acquire); }
        bool shouldRun() const { return _should_run; }
        bool becameReady() const { return _became_ready.load(std::memory_order_acquire); }
        bool wasExecuted() const { return _executed.load(std::memory_order_acquire); }
        ExecState getExecState() const { return _exec_state; }
        
        void resetTickState() {
            _should_run = true;
            _became_ready.store(false, std::memory_order_relaxed);
            _executed.store(false, std::memory_order_relaxed);
            _exec_state = ExecState::NotScheduled;
        }

    private:
        friend class rtfw::RealTimeFramework;
        friend class internal::Timeline;
        friend class TaskRegistry;
        friend class internal::ParameterManager;
        friend struct std::default_delete<ITask>;
        
        // Friend declarations for template classes that access private members
        template<typename> friend class DataReader;
        template<typename> friend class DataWriter;
        template<typename> friend class Parameter;

        static ITask* currentConstructingTask();
        
        struct BaseRequest {
            std::string key;
            uint64_t key_hash;
            uint64_t type_hash;
            size_t size;
            size_t alignment;
        };
        
        struct ReadRequest : BaseRequest {
            std::function<void(void*, common::DataBlockDescriptor*)> wire_job;
            DependencyType deptype;
        };
        
        struct WriteRequest : BaseRequest {
            std::function<void(void*, common::DataBlockDescriptor*, 
                                const std::atomic<uint64_t>*, 
                                const std::atomic<uint64_t>*, 
                                blackbox::CacheSlot*, blackbox::CacheSlot*)> wire_job;
            std::function<void(blackbox::CacheSlot*, blackbox::CacheSlot*)> slot_update_job;
            bool archive_enabled;
        };

        struct ParamReadRequest { // DataReader의 ReadRequest와 유사
            std::string key;
            uint64_t key_hash;
            uint64_t type_hash;
            size_t size;
            size_t alignment;
            internal::IParameter* parameter_proxy;
            std::function<void(void* shm_base_ptr, const common::ParameterInfo* info)> wire_job;
        };
        
        std::vector<ReadRequest> _read_requests;
        std::vector<WriteRequest> _write_requests;
        std::vector<ParamReadRequest> _param_read_requests;
        std::vector<ITask*> _dependents; // [UNUSED] 중앙 저장소 _task_dependents 사용
        std::map<uint64_t, internal::IDataReader*> _data_readers;
        std::vector<internal::IDataWriter*> _data_writers;
        std::vector<internal::IDataReader*> _held_proxies;
        std::vector<internal::IParameter*> _parameter_readers; 
        internal::Timeline* _owner_timeline;

        // 태스크 활성화 및 실행 상태
        // _enabled, _pending_overrun_recovery: RT 스레드(tick_start/tick_end)와
        //   worker 스레드(onTaskFinished)가 동시에 접근 → std::atomic<bool> 필수
        // _executed: execute() 완료 후 worker가 store, tick_end에서 RT가 load
        //   → plain bool이면 stats 계산 구간에서 tick_end가 false로 오탐해 DeadlineMissSelf 발생 가능
        std::atomic<bool> _enabled{true};                   // on/off 플래그 (RT+worker 양쪽에서 접근)
        bool _should_run = true;                            // 이번 tick에 실행 대상인지 (RT 스레드 전용)
        std::atomic<bool> _became_ready{false};             // 이번 tick에 ready 상태가 되었는지 (worker write / RT read)
        std::atomic<bool> _executed{false};                 // 이번 tick에 실행 완료되었는지 (worker write / RT read)
        std::atomic<bool> _pending_overrun_recovery{false}; // overrun 후 완료 시 re-enable 대기 (RT+worker 양쪽에서 접근)
        ExecState _exec_state = ExecState::NotScheduled;    // tick 끝 실행 상태 분류 (RT 스레드 전용)

        void _autoRegisterReadRequest(ReadRequest&& req, internal::IDataReader* reader);
        void _autoRegisterWriteRequest(WriteRequest&& req, internal::IDataWriter* writer);
        void _autoRegisterParamRequest(ParamReadRequest&& req, internal::IParameter* param);

        void _setID(uint32_t id) { _id = id; }
        void _setFrequency(uint32_t frequency) { _frequency = frequency; }
        void _set_dependents(std::vector<ITask*> deps) { _dependents = std::move(deps); }
        void _set_current_tick(uint64_t tick) { _current_tick_in_cycle = tick; }
        void _set_execution_local_tick(uint64_t local_tick) { _execution_local_tick = local_tick; }
        void _setAffinity(int affinity) { _affinity = affinity; }
        void _set_pushed_time() { _pushed_to_queue_time = std::chrono::high_resolution_clock::now(); }
        void _setNonRt(bool is_non_rt) { _is_non_rt=is_non_rt; }
        void _capture_proxy(const std::vector<uint64_t>& target_proxy) {
            _held_proxies.clear();
            for (const auto& target: target_proxy) {
                auto proxy = _data_readers[target];
                if (proxy) {
                    proxy->hold();
                    _held_proxies.push_back(proxy);
                }
            }

            for (auto* p_reader : _parameter_readers) {
                p_reader->capture();
            }
        }
        void _release_proxy() {
            for (const auto proxy: _held_proxies) {
                proxy->unhold();
            }
        }
        void _commit_proxy() {
            for (auto proxy: _data_writers)
                proxy->commit();
        }
        void _injection_proxy() {
            for (auto& proxy: _data_writers) {
                if (proxy->_replay_slot_ptr) {
                    auto& slot = *proxy->_replay_slot_ptr;
                    if (slot.update) {
                        slot.update = false;
                        auto data_ptr = proxy->access();
                        if (data_ptr) {
                            memcpy(data_ptr, slot.data.data(), slot.data.size());
                        }
                    } else {
                        auto header = common::get_buffer_header(proxy->_shm_base_ptr, proxy->_descriptor);
                        int ready_idx = header->ready_index.load(std::memory_order_relaxed);
                        header->write_index.store(ready_idx, std::memory_order_relaxed);
                    }
                }
            }
        }
        void _setOwnerTimeline(internal::Timeline* timeline) { _owner_timeline = timeline; };
        internal::Timeline* getOwnerTimeline() const { return _owner_timeline; };
        
        // [NEW] Stale write detection API
        /**
         * @brief 마지막 execute() 호출 중에 stale write 발생 여부 확인
         * @return true if any data writer failed due to stale write
         */
        bool hadAnyStaleWrite() const {
            for (const auto& writer : _data_writers) {
                if (writer->hadStaleWrite()) return true;
            }
            return false;
        }
        
        /**
         * @brief Stale write flag 초기화 (다음 execution 준비용)
         */
        void clearStaleWriteFlags() {
            for (auto& writer : _data_writers) {
                writer->clearStaleWriteFlag();
            }
        }
        
        uint32_t _id;
        uint32_t _frequency;
        std::atomic<uint64_t> _current_tick_in_cycle;
        std::atomic<uint64_t> _execution_local_tick;
        int _affinity;
        bool _is_non_rt;
        std::chrono::high_resolution_clock::time_point _pushed_to_queue_time;
    };

    template <typename StateT>
    class Task : public ITask {
    public:
        size_t getStateSize() const final { 
            // 상태가 POD(Plain Old Data)인지 검증하여 스냅샷 안전성 확보
            // 너무 엄격한거 같음. 생성자가 반드시 요구되지만 않으면 상관없을듯
            // static_assert(std::is_trivially_copyable_v<StateT>, "State must be POD for snapshotting.");
            return sizeof(StateT); 
        }


        void initialize(void* state_ptr) final {
            initialize(*static_cast<StateT*>(state_ptr));
        }

        void execute(void* state_ptr) final {
            // 멤버 변수가 아닌 인자로 들어온 state를 쓰도록 유도
            execute(*static_cast<StateT*>(state_ptr));
        }

        virtual void initialize(StateT& state) {};
        virtual void execute(StateT& state) = 0;
    };

    // --- Template Implementations ---

    // template<typename T>
    // inline void TaskRegistry::add_dependency(DataReader<T>& proxy) {
    //     ITask::ReadRequest req;
    //     req.key = proxy.key();
    //     req.key_hash = std::hash<std::string>{}(req.key);
    //     req.type_hash = std::hash<std::string_view>{}(common::get_stable_type_signature<T>());
    //     req.deptype = proxy.dependencyType();
    //     req.size = sizeof(T);
    //     req.alignment = alignof(T);
    //     req.wire_job = [&proxy](void* base, common::DataBlockDescriptor* desc){ proxy._wire(base, desc); };
    //     _task_ptr->_read_requests.push_back(std::move(req));
    //     _task_ptr->_data_readers.insert({req.key_hash, &proxy});

    //     _task_ptr->_held_proxies.push_back(&proxy);
    // }

    // template<typename T>
    // inline void TaskRegistry::add_dependency(DataWriter<T>& proxy) {

    //     ITask::WriteRequest req;
    //     req.key = proxy.key();
    //     req.key_hash = std::hash<std::string>{}(req.key);
    //     req.type_hash = std::hash<std::string_view>{}(common::get_stable_type_signature<T>());
    //     req.archive_enabled = (proxy.archiveOption() == ArchiveOption::Enable);
    //     req.size = sizeof(T);
    //     req.alignment = alignof(T);
    //     bool is_non_rt = _task_ptr->isNonRt();
    //     req.wire_job = [&proxy, is_non_rt](void* base, common::DataBlockDescriptor* desc, 
    //                             const std::atomic<uint64_t>* global_tick, 
    //                             const std::atomic<uint64_t>* execution_tick, 
    //                             blackbox::CacheSlot* record_slot_ptr, 
    //                             blackbox::CacheSlot* replay_slot_ptr){
    //         proxy._wire(base, desc, global_tick, execution_tick, record_slot_ptr, replay_slot_ptr, is_non_rt);
    //     };
    //     _task_ptr->_write_requests.push_back(std::move(req));
    //     _task_ptr->_data_writers.push_back(&proxy);
    // }

    // template<typename T>
    // void TaskRegistry::add_dependency(Parameter<T>& proxy) {
    //     ITask::ParamReadRequest req;
    //     req.key = proxy.key();
    //     req.key_hash = std::hash<std::string>{}(req.key);
    //     req.type_hash = std::hash<std::string_view>{}(common::get_stable_type_signature<T>());
    //     req.size = common::param_layout<T>::size;
    //     req.alignment = common::param_layout<T>::alignment;
    //     req.parameter_proxy = &proxy;
    //     req.wire_job = [&proxy](void* shm_base_ptr, const common::ParameterInfo* info){ proxy._wire(shm_base_ptr, info); };            
    //     _task_ptr->_param_read_requests.push_back(std::move(req));
    //     _task_ptr->_parameter_readers.push_back(&proxy);
    // }
    
    // ===================================================================
    // Template Constructor Implementations for DataReader, DataWriter, Parameter
    // (Defined here after ITask is complete)
    // ===================================================================
    
    template<typename T>
    DataReader<T>::DataReader(std::string_view key, DependencyType dep_type) 
        : IDataReader(key, dep_type) {
        if (auto* owner = ITask::currentConstructingTask()) {
            ITask::ReadRequest req;
            req.key = this->key();
            req.key_hash = std::hash<std::string>{}(req.key);
            req.type_hash = std::hash<std::string_view>{}(common::get_stable_type_signature<T>());
            req.deptype = this->dependencyType();
            req.size = sizeof(T);
            req.alignment = alignof(T);
            req.wire_job = [this](void* base, common::DataBlockDescriptor* desc){ 
                this->_wire(base, desc); 
            };

            owner->_autoRegisterReadRequest(std::move(req), this);
        } else {
            spdlog::error("DataReader '{}' must be a member of ITask.", key);
        }
    }

    template<typename T>
    DataWriter<T>::DataWriter(std::string_view key, ArchiveOption archive)
        : IDataWriter(key, archive) {
        if (auto* owner = ITask::currentConstructingTask()) {
            ITask::WriteRequest req;
            req.key = this->key();
            req.key_hash = std::hash<std::string>{}(req.key);
            req.type_hash = std::hash<std::string_view>{}(common::get_stable_type_signature<T>());
            req.archive_enabled = (this->archiveOption() == ArchiveOption::Enable);
            req.size = sizeof(T);
            req.alignment = alignof(T);
            // bool is_non_rt = owner->isNonRt();
            // req.wire_job = [this, is_non_rt](void* base, common::DataBlockDescriptor* desc, 
            req.wire_job = [this](void* base, common::DataBlockDescriptor* desc, 
                                    const std::atomic<uint64_t>* global_tick, 
                                    const std::atomic<uint64_t>* execution_tick, 
                                    blackbox::CacheSlot* record_slot_ptr, 
                                    blackbox::CacheSlot* replay_slot_ptr){
                // this->_wire(base, desc, global_tick, execution_tick, record_slot_ptr, replay_slot_ptr, is_non_rt);
                this->_wire(base, desc, global_tick, execution_tick, record_slot_ptr, replay_slot_ptr);
            };
            req.slot_update_job = [this](blackbox::CacheSlot* record_slot_ptr,
                                         blackbox::CacheSlot* replay_slot_ptr){
                this->_update_slots(record_slot_ptr, replay_slot_ptr);
            };

            owner->_autoRegisterWriteRequest(std::move(req), this);
        } else {
            spdlog::error("DataWriter '{}' must be a member of an ITask", key);
        }
    }

    template<typename T>
    Parameter<T>::Parameter(std::string_view key, T default_value)
        : IParameter(key) {
        _local_cache.assign(common::param_layout<T>::size, 0);
        common::param_codec<T>::encode(default_value, _local_cache.data(), _local_cache.size());
        _decoded_value = default_value;  // 초기값 설정

        if (auto* owner = ITask::currentConstructingTask()) {
            ITask::ParamReadRequest req;
            req.key = this->key();
            req.key_hash = std::hash<std::string>{}(req.key);
            req.type_hash = std::hash<std::string_view>{}(common::get_stable_type_signature<T>());
            req.size = common::param_layout<T>::size;
            req.alignment = common::param_layout<T>::alignment;
            req.parameter_proxy = this;
            req.wire_job = [this](void* shm_base_ptr, const common::ParameterInfo* info){ 
                this->_wire(shm_base_ptr, info); 
            };            

            owner->_autoRegisterParamRequest(std::move(req), this);
        } else {
            spdlog::error("Parameter '{}' must be a member of an ITask", key);
        }
    }
};