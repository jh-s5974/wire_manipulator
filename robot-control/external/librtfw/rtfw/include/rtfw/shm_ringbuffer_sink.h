#pragma once
#include "spdlog/sinks/base_sink.h"
#include "rtfw_common/shm_layout.h"


namespace rtfw::internal {

// [최종] 이 싱크는 생성 시점에 SHM 버퍼를 받아야만 함
// 따라서 초기 로그는 기록할 수 없음
class ShmRingbufferSink : public spdlog::sinks::base_sink<std::mutex> {
public:
    // 생성 시점에 유효한 SHM 버퍼 포인터를 받아야 함
    explicit ShmRingbufferSink(common::SharedLogBuffer* log_buffer) : _log_buffer(log_buffer) {
        if (!_log_buffer) {
            // 이 싱크는 유효한 버퍼 없이는 생성될 수 없음
            throw std::runtime_error("ShmRingbufferSink: log_buffer cannot be null.");
        }
    }

    static void set_context(const std::string& ctx) {
        tls_context() = ctx;
    }

protected:
    // 이 함수는 spdlog의 비동기 스레드에서 호출되며, base_sink의 뮤텍스로 보호됨
    void sink_it_(const spdlog::details::log_msg& msg) override {
        // 1. 다음 쓸 위치를 원자적으로 확보
        uint64_t next_head = _log_buffer->head.fetch_add(1, std::memory_order_relaxed);
        
        // --- << [핵심] 덮어쓰기 로직 >> ---
        uint64_t current_tail = _log_buffer->tail.load(std::memory_order_relaxed);
        // head가 tail을 한 바퀴 이상 추월했다면 (버퍼가 꽉 참)
        if (next_head - current_tail >= 1024) {
            // 가장 오래된 로그가 있던 자리를 "비워주기" 위해 tail을 head쪽으로 한 칸 민다.
            // compare_exchange를 사용하여, 클라이언트가 tail을 동시에 옮기는 경우에도 안전하게 처리.
            _log_buffer->tail.compare_exchange_strong(current_tail, current_tail + 1);
        }
        // ------------------------------------

        // 2. 실제 배열 인덱스에 데이터 쓰기
        size_t array_index = next_head % 1024;
        volatile common::LogEntry& entry = _log_buffer->entries[array_index];

        // const_cast를 사용하여 volatile 메모리에 쓰기
        const_cast<common::LogLevel&>(entry.level) = static_cast<common::LogLevel>(msg.level);
        
        strncpy(const_cast<char*>(entry.task_name), tls_context().data(), std::min(sizeof(entry.task_name) - 1, tls_context().size()));
        const_cast<char&>(entry.task_name[std::min(sizeof(entry.task_name) - 1, tls_context().size())]) = '\0';
        
        size_t msg_size = std::min(msg.payload.size(), sizeof(entry.message) - 1);
        memcpy(const_cast<char*>(entry.message), msg.payload.data(), msg_size);
        const_cast<char&>(entry.message[msg_size]) = '\0';

        std::string_view str_level;
        switch(entry.level) {
            case common::LogLevel::CRITICAL:    str_level = "CRITICAL"; break;
            case common::LogLevel::ERROR:       str_level = "ERROR";    break;
            case common::LogLevel::WARN:        str_level = "WARN";     break;
            case common::LogLevel::INFO:        str_level = "INFO";     break;
            case common::LogLevel::DEBUG:       str_level = "DEBUG";    break;
            case common::LogLevel::TRACE:       str_level = "TRACE";    break;
        }
        // std::cout << "[" << str_level << "] [" << std::string((const char*)entry.task_name).c_str() << "] " << (const char*)entry.message << std::endl;
    }

    void flush_() override {}

private:
    common::SharedLogBuffer* _log_buffer;

    static std::string& tls_context() {
        // static thread_local std::string context = "Framework";
        static std::string context = "Framework";
        return context;
    }
};

} // namespace rtfw