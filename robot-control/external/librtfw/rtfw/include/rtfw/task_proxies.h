#pragma once

#include <stdexcept>
#include <atomic>
#include <new>
#include <cstring>
#include <string>
#include <string_view>
#include <iostream>

#include "rtfw_common/shm_layout.h"
#include "rtfw_common/shm_utils.h"
#include "rtfw_common/blackbox.h"
#include "rtfw_common/parameter_utils.h"
#include "rtfw_common/type_utils.h"


#include "spdlog/spdlog.h"
#include <yaml-cpp/yaml.h>

namespace rtfw {
    class RealtimeFramework;
    namespace rt {
        enum class DependencyType {
            Strong,
            Weak
        };

        enum class ArchiveOption : bool {
            Disable = false,
            Enable = true,
        };

        struct Signal {};
        class ITask;
    };
}

namespace rtfw::internal {
    extern std::shared_ptr<spdlog::logger> async_logger;


    class IDataReader {
    public:
        IDataReader(std::string_view key, rt::DependencyType dep_type = rt::DependencyType::Strong)
            : _key(key), _dep_type(dep_type), _shm_base_ptr(nullptr), _descriptor(nullptr), 
            _last_read_tick(0), _ready_idx(-1), _is_hold(false), _has_restored_capture(false), _restored_write_tick(0) {}
        std::string_view key() const { return _key; }
        rt::DependencyType dependencyType() const { return _dep_type; }

        bool export_capture_value(std::vector<uint8_t>& out, uint64_t& write_tick) const {
            out.clear();
            write_tick = 0;
            if (!_shm_base_ptr || !_descriptor) return false;

            if (_has_restored_capture && _is_hold) {
                out = _restored_capture;
                write_tick = _restored_write_tick;
                return !out.empty();
            }

            auto header = common::get_buffer_header(_shm_base_ptr, _descriptor);
            if (!header) return false;

            int source_idx = _is_hold ? _ready_idx : header->ready_index.load(std::memory_order_acquire);
            if (source_idx < 0) return false;

            auto* state = common::get_buffer_state(_shm_base_ptr, _descriptor, source_idx);
            auto* data_ptr = common::get_data_buffer(_shm_base_ptr, _descriptor, source_idx);
            if (!state || !data_ptr) return false;

            out.resize(_descriptor->data_size);
            memcpy(out.data(), data_ptr, _descriptor->data_size);
            write_tick = state->write_tick.load(std::memory_order_relaxed);
            return true;
        }

        bool restore_capture_value(const uint8_t* data, size_t size, uint64_t write_tick) {
            if (!_shm_base_ptr || !_descriptor || !data) return false;
            if (size != _descriptor->data_size) return false;

            if (_is_hold && !_has_restored_capture && _ready_idx >= 0) {
                auto* state = common::get_buffer_state(_shm_base_ptr, _descriptor, _ready_idx);
                if (state) {
                    state->ref_count.fetch_sub(1, std::memory_order_relaxed);
                }
            }

            _restored_capture.assign(data, data + size);
            _restored_write_tick = write_tick;
            _has_restored_capture = true;
            _ready_idx = -1;
            _is_hold = false;
            return true;
        }
        
        void hold() {
            if (_is_hold) {
                std::string msg;
                msg += "DataReader already hold (";
                msg += key();
                msg += ")";
                throw std::runtime_error(msg);
            }

            if (_has_restored_capture) {
                _is_hold = true;
                return;
            }

            auto header = common::get_buffer_header(_shm_base_ptr, _descriptor);
            if (!header) return;
            
            // acquire 연산으로 ready_index를 읽고, 그 값을 계속 사용한다.
            const int captured_idx = header->ready_index.load(std::memory_order_acquire);
            if (captured_idx < 0) return;

            auto* state = common::get_buffer_state(_shm_base_ptr, _descriptor, captured_idx);
            if (!state) throw std::runtime_error("Cannot get buffer state.");

            state->ref_count.fetch_add(1, std::memory_order_relaxed);

            // 모든 작업이 성공한 후에 멤버 변수 업데이트
            _ready_idx = captured_idx;
            _is_hold = true;
        }
        void unhold() {
            if (!_is_hold)
                return;

            if (_has_restored_capture) {
                _is_hold = false;
                _has_restored_capture = false;
                _restored_capture.clear();
                _restored_write_tick = 0;
                return;
            }

            _is_hold = false;
            auto stats = common::get_buffer_state(_shm_base_ptr, _descriptor, _ready_idx);
            stats->ref_count.fetch_sub(1, std::memory_order_relaxed);
        }

        bool check_update(bool consume = true) {
            if (_has_restored_capture && _is_hold) {
                bool has_update = _restored_write_tick > _last_read_tick;
                if (has_update && consume) {
                    _last_read_tick = _restored_write_tick;
                }
                return has_update;
            }

            if (!_shm_base_ptr || !_descriptor) return false;
            auto header = common::get_buffer_header(_shm_base_ptr, _descriptor);
            if (!_is_hold) _ready_idx = header->write_index;
            if (_ready_idx < 0) return false;
            auto stats = common::get_buffer_state(_shm_base_ptr, _descriptor, _ready_idx);             
            uint64_t current_write_tick = stats->write_tick.load(std::memory_order_relaxed);
            bool has_update = current_write_tick > _last_read_tick;

            async_logger->trace("{} : check.({}). outdate: {}", _key, _ready_idx, has_update);

            if (has_update && consume) {
                _last_read_tick = current_write_tick;
            }
            return has_update;
        }

        explicit operator bool() const {
            if (_has_restored_capture && _is_hold) {
                return true;
            }

            if (!_shm_base_ptr || !_descriptor) {
                return false;
            }
            
            // hold 상태이거나, ready_index가 유효한 값(-1이 아님)을 가지고 있는지 확인
            int ready_idx = _is_hold 
                ? _ready_idx 
                : common::get_buffer_header(_shm_base_ptr, _descriptor)->ready_index.load(std::memory_order_acquire);
            
            return ready_idx >= 0;
        }

        void _wire(void* shm_base_ptr, common::DataBlockDescriptor* descriptor) {
            if (_key != descriptor->key) {
                throw std::logic_error("Key mismatch during wire-up for key '" + std::string(_key) + "'.");
            }
            _shm_base_ptr = shm_base_ptr;
            _descriptor = descriptor;
            _last_read_tick = 0;
        }

    protected:    
        std::string_view _key;
        rt::DependencyType _dep_type;
        void* _shm_base_ptr;
        common::DataBlockDescriptor* _descriptor;
        mutable uint64_t _last_read_tick;
        int _ready_idx;
        bool _is_hold;
        bool _has_restored_capture;
        uint64_t _restored_write_tick;
        std::vector<uint8_t> _restored_capture;
    };


    class IDataWriter {
    public:
        explicit IDataWriter(std::string_view key, rt::ArchiveOption archive = rt::ArchiveOption::Disable)
            : _key(key), _archive(archive), _shm_base_ptr(nullptr), _descriptor(nullptr), 
            is_write_pending(false), _stale_write_protect(true) {}
        virtual ~IDataWriter() = default;
        
        std::string_view key() const { return _key; }
        rt::ArchiveOption archiveOption() const { return _archive; }
    
        // [NEW] Stale write 여부 조회 (task가 write 실패를 감지하기 위해)
        bool hadStaleWrite() const { return _had_stale_write; }
        void clearStaleWriteFlag() { _had_stale_write = false; }
    
    protected:
        void* access() {
            if (!_shm_base_ptr || !_descriptor) {
                throw std::logic_error("DataWriter for key '" + std::string(_key) + "' is not wired.");
            }

            auto header = common::get_buffer_header(_shm_base_ptr, _descriptor);
            void* write_buffer_ptr = next_write_buffer();
            if (!write_buffer_ptr) throw std::runtime_error("write buffer full");
            
            auto write_idx = header->write_index.load(std::memory_order_relaxed);
            async_logger->trace("{}: accessed.({})", _key, write_idx);

            auto stats = common::get_buffer_state(_shm_base_ptr, _descriptor, write_idx);
            uint64_t global_tick = _start_tick_ptr->load(std::memory_order_relaxed);
            stats->write_tick.store(global_tick, std::memory_order_relaxed);


            is_write_pending = true;
            return write_buffer_ptr;
        }

        void _wire(void* shm_base_ptr, common::DataBlockDescriptor* descriptor, 
                const std::atomic<uint64_t>* global_tick_ptr, 
                const std::atomic<uint64_t>* execution_tick_ptr, 
                blackbox::CacheSlot* record_slot_ptr,
                // blackbox::CacheSlot* replay_slot_ptr,
                // bool is_non_rt) {                
                blackbox::CacheSlot* replay_slot_ptr) {
            if (_key != descriptor->key) {
                throw std::logic_error("Key mismatch during wire-up for key '" + std::string(_key) + "'.");
            }
            _shm_base_ptr = shm_base_ptr;
            _descriptor = descriptor;
            _start_tick_ptr = global_tick_ptr;
            _execution_tick_ptr = execution_tick_ptr;
            _record_slot_ptr.store(record_slot_ptr, std::memory_order_release);
            _replay_slot_ptr.store(replay_slot_ptr, std::memory_order_release);
            // if (is_non_rt) {
            //     _stale_write_protect = false;
            // } else {
            //     _stale_write_protect = true;
            // }
        }

        void _update_slots(blackbox::CacheSlot* record_slot_ptr,
                           blackbox::CacheSlot* replay_slot_ptr) {
            _record_slot_ptr.store(record_slot_ptr, std::memory_order_release);
            _replay_slot_ptr.store(replay_slot_ptr, std::memory_order_release);
        }


        void commit() {
            if (!_descriptor || !_execution_tick_ptr) return;

            // internal injection
            // if (_replay_slot_ptr) {
            //     if (_replay_slot_ptr->update) {
            //         _replay_slot_ptr->update = false;
            //         void* write_buffer = next_write_buffer();
            //         memcpy(write_buffer, _replay_slot_ptr->data.data(), _replay_slot_ptr->data.size());
            //         is_write_pending = true;
            //     } else {
            //         is_write_pending = false;
            //     }
            // }

            if (!is_write_pending) return;
            is_write_pending = false;


            uint64_t start_tick = _start_tick_ptr->load(std::memory_order_acquire);
            uint64_t commit_tick = _execution_tick_ptr->load(std::memory_order_acquire);
            if (_stale_write_protect && start_tick > commit_tick) {
                // Stale write 감지. 경고는 Non-RT에서 처리하도록 하고, 여기서는 쓰기 거부만.
                // if (rtfw::_log_level >= rtfw::LogLevel::DetailedDeps) {
                //     auto header = common::get_buffer_header(_shm_base_ptr, _descriptor);
                //     auto wrtie_idx = header->write_index.load(std::memory_order_relaxed);
                //     std::cout << "  " << _key << ": stale write detected.(" << wrtie_idx << ") at " << global_tick << std::endl;
                // }
                async_logger->warn("{}: stale write detected. {} tick -> {} tick (WRITE IGNORED)", _key, commit_tick, start_tick);
                _had_stale_write = true;  // [NEW] Stale write 플래그 설정
                return;
            }

            auto header = common::get_buffer_header(_shm_base_ptr, _descriptor);
            int ready_idx = header->ready_index.load(std::memory_order_relaxed);
            int write_idx = header->write_index.load(std::memory_order_relaxed);

            auto* record_slot_ptr = _record_slot_ptr.load(std::memory_order_acquire);
            if (record_slot_ptr) {
                const char* write_buffer = static_cast<const char*>(common::get_data_buffer(_shm_base_ptr, _descriptor, write_idx));
                // Seqlock-style write: mark odd -> write -> mark even
                record_slot_ptr->seq.fetch_add(1, std::memory_order_acq_rel); // odd
                record_slot_ptr->start_tick.store(start_tick, std::memory_order_relaxed);
                record_slot_ptr->type_hash = _descriptor->type_hash;
                if (record_slot_ptr->data.size() != _descriptor->data_size) {
                    async_logger->warn("{}: record slot size mismatch (slot={}, desc={})", _key,
                        record_slot_ptr->data.size(), _descriptor->data_size);
                    record_slot_ptr->seq.fetch_add(1, std::memory_order_acq_rel); // even
                    return;
                }
                memcpy(record_slot_ptr->data.data(), write_buffer, _descriptor->data_size);
                record_slot_ptr->seq.fetch_add(1, std::memory_order_acq_rel); // even
                record_slot_ptr->update.store(true, std::memory_order_release);
            }

            auto stats = common::get_buffer_state(_shm_base_ptr, _descriptor, write_idx);
            header->ready_index.store(write_idx, std::memory_order_release);

            // if (rtfw::_log_level >= rtfw::LogLevel::DetailedDeps) {
            //     auto header = common::get_buffer_header(_shm_base_ptr, _descriptor);
            //     auto wrtie_idx = header->write_index.load(std::memory_order_relaxed);
            //     std::cout << "  " << _key << ": commited.(" << wrtie_idx << ") at " << commit_tick << std::endl;
            // }
            async_logger->trace("{}: commited.({})", _key, write_idx);
        }

        bool isRecord() {
            return _record_slot_ptr.load(std::memory_order_relaxed) != nullptr;
        }

        bool isReplay() {
            return _replay_slot_ptr.load(std::memory_order_relaxed) != nullptr;
        }

    private:
        friend class rt::ITask;
        friend class rtfw::RealTimeFramework;

        void* next_write_buffer() {
            auto header = common::get_buffer_header(_shm_base_ptr, _descriptor);
            if (!header) return nullptr;

            // 현재 쓰기 인덱스에서 시작하여 한 바퀴 순회
            int start_idx = header->write_index.load(std::memory_order_relaxed);
            int next_idx = start_idx;

            for (size_t i = 0; i < _descriptor->block_count - 1; ++i) {
                // 다음 인덱스로 이동 (순환)
                next_idx = (start_idx + i + 1) % _descriptor->block_count;

                auto state = common::get_buffer_state(_shm_base_ptr, _descriptor, next_idx);
                if (!state) continue; // 안전장치

                if (state->ref_count.load(std::memory_order_relaxed) == 0) {
                    // 사용 가능한 버퍼를 찾음!
                    header->write_index.store(next_idx, std::memory_order_relaxed);
                    return common::get_data_buffer(_shm_base_ptr, _descriptor, next_idx);
                }
            }

            // 모든 버퍼가 사용 중임
            return nullptr;
        }


        std::string_view _key;
        rt::ArchiveOption _archive;
        void* _shm_base_ptr;
        common::DataBlockDescriptor* _descriptor;
        const std::atomic<uint64_t>* _start_tick_ptr;
        const std::atomic<uint64_t>* _execution_tick_ptr;
        std::atomic<blackbox::CacheSlot*> _record_slot_ptr{nullptr};
        std::atomic<blackbox::CacheSlot*> _replay_slot_ptr{nullptr};
        bool is_write_pending;
        bool _stale_write_protect;
        bool _had_stale_write{false};  // [NEW] Stale write 여부 추적
    };


    // 타입 소거를 위한 기본 인터페이스
    class IParameter {
    public:
        IParameter(std::string_view key) : _key(key), _shm_base_ptr(nullptr), _info(nullptr), _cached_version(0) {}
        std::string_view key() const { return _key; }

        // wire_job에서 ParameterManager가 호출하여 ParameterInfo를 연결해 줌
        void _wire(void* shm_base_ptr, const common::ParameterInfo* info) { 
            _shm_base_ptr = shm_base_ptr;
            _info = info;
            if (_local_cache.empty()) { _local_cache.resize(info->data_size); }

            auto* p_block_header = common::get_param_block_header(shm_base_ptr);
            if (!p_block_header) return;
            
            // [핵심 로직] 공유 메모리의 버퍼 0의 버전을 확인
            // (초기화 시점에는 buffer 0과 1이 동일하므로 0만 확인해도 무방)
            uint64_t current_shm_version = p_block_header->version[0].load(std::memory_order_acquire);

            if (current_shm_version == 0) {
                common::write_parameter(shm_base_ptr, _info, _local_cache.data(), _local_cache.size());
            } else {
                // [Warn] already initialized (different value)
            }
        }

        void capture() {
            if (!_shm_base_ptr || !_info) return;
            auto header = common::get_param_block_header(_shm_base_ptr);
            int stable_idx = header->stable_index.load(std::memory_order_acquire);
            uint64_t stable_ver = header->version[stable_idx].load(std::memory_order_acquire);

            // 내가 캐시한 버전과 Stable 버전이 다를 경우에만 값 업데이트
            if (_cached_version != stable_ver) {            
                const char* source_buffer = common::get_param_data_buffer(_shm_base_ptr, stable_idx);
                const void* source_data_ptr = source_buffer + _info->offset_in_buffer;
                
                // 전체 블록이 아닌, '내 데이터'만 복사
                memcpy(_local_cache.data(), source_data_ptr, _info->data_size);
                
                _cached_version = stable_ver;
                decode();  // binary -> decoded value (타입별 구현)
            }
        }

        // [신규] 타입별 decode 구현: capture() 후 호출됨
        virtual void decode() = 0;

    protected:
        friend class ParameterManager;

        virtual bool initializeFromYaml(void* shm_base_ptr, const YAML::Node& node) = 0;
        
        std::string_view _key;
        const common::ParameterInfo* _info; // key에 해당하는 메타데이터 포인터
        void* _shm_base_ptr;
        std::vector<uint8_t> _local_cache; // binary buffer (고정 크기)
    private:
        uint64_t _cached_version;
    };
};

namespace rtfw::rt {
    // --- DataReader ---
    template<typename T>
    class DataReader : public internal::IDataReader{
    public:
        explicit DataReader(std::string_view key, DependencyType dep_type = DependencyType::Strong);
        DataReader() = delete;  // Error: DataReader<T> must be initialized with a key name. Example: DataReader<T> dr{"key_name"};

        void read(T& val) {val = this->operator*();}
        const T& read() {return this->operator*();}

        template<typename Callback>
        void on_update(Callback&& callback) {
            constexpr bool should_consume_in_check = std::is_same_v<T, Signal>;
            if (check_update(should_consume_in_check)) {
                if constexpr (std::is_same_v<T, Signal>) {
                    callback();
                } else {
                    callback(this->read());
                }
            }
        }

        const T* operator->() {
            if (!(*this)) { // <<< operator bool()을 사용하여 유효성 검사
                // operator->는 예외 대신 nullptr을 반환하는 것이 일반적인 컨벤션
                return nullptr;
            }

            if (_has_restored_capture && _is_hold) {
                if (_restored_capture.size() != sizeof(T)) {
                    return nullptr;
                }
                _last_read_tick = _restored_write_tick;
                return reinterpret_cast<const T*>(_restored_capture.data());
            }

            int read_idx = _is_hold ? _ready_idx : common::get_buffer_header(_shm_base_ptr, _descriptor)->write_index.load(std::memory_order_relaxed);
            
            auto state = common::get_buffer_state(_shm_base_ptr, _descriptor, read_idx);
            if (!state) {
                return nullptr;
            }

            internal::async_logger->trace("{}: read.({})", _key, read_idx);
            
            _last_read_tick = state->write_tick.load(std::memory_order_relaxed);
            return static_cast<const T*>(common::get_data_buffer(_shm_base_ptr, _descriptor, read_idx));
        }
        
        const T& operator*() {
            if (!(*this)) {
                throw std::runtime_error("Attempted to access data for key '" + std::string(_key) + "', but it has not been written yet.");
            }
            return *(this->operator->());
        }
    };

    template<typename T>
    class DataWriter : public internal::IDataWriter {
        friend class TaskRegistry;
    public:
        explicit DataWriter(std::string_view key, ArchiveOption archive = ArchiveOption::Disable);
        DataWriter() = delete;  // Error: DataWriter<T> must be initialized with a key name. Example: DataWriter<T> dw{"key_name"};

        void write(const T& val) {
            if (isReplay())
                return;
            if constexpr (std::is_same_v<T, Signal>) {
                static_assert(!std::is_same_v<T, Signal>,
                    "Error: For DataWriter<Signal>, use the write() overload with no arguments.");
            } else {
                auto h = static_cast<T*>(access());
                *h = val;
            }
        }

        void write() {
            if (isReplay())
                return;
            // 컴파일 타임에 T가 Signal이 아닌지 확인
            if constexpr (!std::is_same_v<T, Signal>) {
                // T가 Signal이 아니면, 이 함수는 잘못 사용된 것이므로 컴파일 에러 >발생
                static_assert(std::is_same_v<T, Signal>,
                    "Error: The parameter-less write() is only for DataWriter<Signal>.");
            } else {
                // T가 Signal일 때만 이 코드가 컴파일됨
                this->access();
            }
        }
    };

    template<typename T>
    class Parameter : public internal::IParameter {
    public:
        explicit Parameter(std::string_view key, T default_value = T{});
        Parameter() = delete;  // Error: Parameter<T> must be initialized with a key name. Example: Parameter<T> p{"key_name", default_value};

        // [최고 성능] 캐시된 값 참조 반환 (매번 O(1))
        const T& read() const {
            return _decoded_value;
        }

        const T& operator*() const { return read(); }

        bool write(const T& v) {
            if (!_shm_base_ptr || !_info) return false;
            return common::param_codec<T>::write(_shm_base_ptr, _info, v);
        }

    private:
        // [내부] SHM binary -> _decoded_value (capture 호출 후 1회만)
        void decode() override {
            _decoded_value = common::param_codec<T>::decode(_local_cache.data(), _local_cache.size());
        }

        bool initializeFromYaml(void* shm_base_ptr, const YAML::Node& node) override {
            try {
                const T v = common::param_codec<T>::parse_yaml(node);
                return common::param_codec<T>::write(shm_base_ptr, _info, v);
            } catch (...) { return false; }
        }

        T _decoded_value{};  // 디코드된 값 캐시 (고정 크기)
    };
};