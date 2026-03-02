#include "rtfw_common/log_format.h"
#include <string_view>

namespace rtfw::internal {
    class ILogger {
    public:
        virtual ~ILogger() = default;
        
        template<typename... Args>
        void log(common::LogLevel level, std::string_view component, uint64_t tick, const char* fmt, Args&&... args) {
            // 포맷팅된 최종 메시지를 구현체에 전달
            log_impl(level, component, tick, fmt::format(fmt, std::forward<Args>(args)...));
        }
    protected:
        virtual void log_impl(common::LogLevel level, std::string_view component, uint64_t tick, const std::string& msg) = 0;
    };
}