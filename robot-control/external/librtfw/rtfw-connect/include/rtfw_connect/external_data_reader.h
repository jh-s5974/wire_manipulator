// rtfw-client/external_data_reader.h
#pragma once

#include "rtfw_common/shm_layout.h"
#include "rtfw_common/shm_utils.h"
#include <stdexcept>
#include <string>


namespace rtfw::connect {    
    template<typename T>
    class ExternalDataReader {
    public:
        explicit ExternalDataReader(std::string_view key = "") 
            : _key(key), _shm_base_ptr(nullptr), _descriptor(nullptr) {}

        /**
         * @brief 최신 데이터를 안전하게 읽어올 수 있는 const 포인터를 반환합니다.
         *        이 함수는 내부적으로 더블 버퍼링의 읽기 프로토콜을 수행합니다.
         * @return 데이터에 대한 const 포인터. 데이터가 아직 없거나 리더가 유효하지 않으면 nullptr.
         */
        const T* access() const {
            if (!isValid()) {
                return nullptr;
            }

            auto header = get_buffer_header(_shm_base_ptr, _descriptor);

            // 2. 현재 읽기 가능한 버퍼의 인덱스를 원자적으로 가져옵니다.
            int idx = header->ready_index.load(std::memory_order_acquire);
            if (idx < 0) {
                // 아직 데이터가 한 번도 쓰여진 적이 없는 경우
                return nullptr;
            }

            // 3. 헬퍼 함수를 호출하여 실제 데이터 버퍼의 주소를 가져옵니다.
            return static_cast<const T*>(get_data_buffer(_shm_base_ptr, _descriptor, idx));
        }

        /**
         * @brief 최신 데이터를 복사하여 반환합니다. 데이터 접근에 실패하면 예외를 던집니다.
         *        포인터 관리가 번거로울 때 유용하며 Non-RT Task에서 사용을 권장합니다.
         * @return 최신 데이터의 복사본.
         */
        T read() const {
            const T* ptr = access();
            if (!ptr) {
                std::string err_msg = "Failed to read data for key '";
                if (_descriptor) err_msg += _descriptor->key;
                else err_msg += "unknown";
                err_msg += "': not available or reader is invalid.";
                throw std::runtime_error(err_msg);
            }
            return *ptr;
        }

        /**
         * @brief 리더가 유효한 공유 메모리 영역을 가리키고 있는지 확인합니다.
         * @return 유효하면 true, 아니면 false.
         */
        bool isValid() const {
            return _shm_base_ptr != nullptr && _descriptor != nullptr;
        }
        bool check_update(uint64_t* current_tick = nullptr) const {
            if (!isValid()) return false;
            
            auto header = get_buffer_header(_shm_base_ptr, _descriptor);
            int idx = header->ready_index.load(std::memory_order_acquire);
            if (idx < 0) return false;

            auto state = get_buffer_state(_shm_base_ptr, _descriptor, idx);
            if (!state) return false;
            
            uint64_t tick_in_shm = state->write_tick.load(std::memory_order_relaxed);
            if (current_tick) *current_tick = tick_in_shm;
            
            if (tick_in_shm > last_read_tick_) {
                last_read_tick_ = tick_in_shm;
                return true;
            }
            return false;
        }

        std::string_view key() const { return _key; }
    private:
        friend class SharedMemoryQuerier;

        // 포인터는 const이지만, 생성자에서 const_cast를 통해 초기화.
        // 이는 ExternalDataReader가 데이터를 수정하지 않음을 보장하면서도,
        // 내부적으로 non-const 포인터와도 호환되게 하기 위함.
        ExternalDataReader(void* shm_base_ptr, const common::DataBlockDescriptor* descriptor)
            : _shm_base_ptr(shm_base_ptr), 
            _descriptor(const_cast<common::DataBlockDescriptor*>(descriptor)) {}

        void _wire(void* shm_base_ptr, const common::DataBlockDescriptor* descriptor) {
            if (_key != descriptor->key) {
                throw std::logic_error("Key mismatch during wire-up for ExternalDataReader key '" + std::string(_key) + "'.");
            }
            _shm_base_ptr = shm_base_ptr;
            _descriptor = const_cast<common::DataBlockDescriptor*>(descriptor);
        }

        std::string_view _key;
        void* _shm_base_ptr;
        common::DataBlockDescriptor* _descriptor;
        mutable uint64_t last_read_tick_{0};
    };
};