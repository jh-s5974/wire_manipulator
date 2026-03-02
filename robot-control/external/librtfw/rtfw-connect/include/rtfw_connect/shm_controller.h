// rtfw-client/shm_controller.h
#pragma once
#include "rtfw_common/shm_layout.h"
#include "rtfw_common/shm_utils.h"
#include <cstdint>
#include <string_view>
#include <string>
#include <map>


namespace rtfw::connect {
    // SharedMemoryQuerier와는 별개의 클래스
    class SharedMemoryController {
    public:
        explicit SharedMemoryController(void* shm_base_ptr);

        // 제한적으로 허용된 쓰기 기능만 노출
        bool clear_task_overrun_flag(uint32_t task_id);
        void setLogLevel(common::LogLevel level);
        
        template<typename T>
        bool setParameter(std::string_view key, const T& value);

        // Scheduler control
        void pauseTick();
        void playTick();
        void playOneTick();

        // Recording control
        void startRecord(const std::string& path);
        void stopRecord();

        // Replay control
        void startReplay(const std::string& path);
        void stopReplay();

        // Trace control (replay + record simultaneously)
        void startTrace(const std::string& replay_path, const std::string& record_path);
        void stopTrace();

        // Task control
        bool setTaskEnabled(uint32_t task_id, bool enabled);

        // Action completion monitoring
        bool isActionBusy() const;
        bool waitForActionComplete(int timeout_ms = 1000);

        // (향후 확장 가능)
        // bool send_system_reset_signal();
        // bool update_global_parameter(const char* key, double value);

    private:
        void buildParamInfoCache();
        const common::ParameterInfo* findParamInfo(std::string_view key) const;

        void* _shm_base_ptr;
        common::SharedMemoryHeader* _header;
        common::TaskStats* _stats_array;
        size_t _task_count;
        std::map<std::string, const common::ParameterInfo*> _param_info_cache;
    };

    template<typename T>
    bool SharedMemoryController::setParameter(std::string_view key, const T& value) {
        if (!_header) return false;

        // 1. 키에 해당하는 ParameterInfo 찾기
        const common::ParameterInfo* info = findParamInfo(key);
        if (!info) {
            // (선택) 오류 메시지 출력
            // std::cerr << "Error: Parameter key '" << key << "' not found." << std::endl;
            return false;
        }

        // 2. rtfw-common에 정의된 쓰기 유틸리티 함수 호출
        //    이 함수가 모든 복잡한 작업을 처리해 줌 (Working 버퍼 찾기, 값 쓰기, 버전 올리기)
        return write_parameter(_shm_base_ptr, info, &value, sizeof(T));
    }
};