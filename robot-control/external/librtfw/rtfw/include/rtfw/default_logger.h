#include "ILogger.h"
#include "spdlog/spdlog.h"

class SpdlogLogger : public rtfw::internal::ILogger {
public:
    SpdlogLogger() {
        
    }
    void log_impl(LogLevel level, std::string_view component, uint64_t tick, const std::string& msg) override;

    std::shared_ptr<spdlog::logger> _logger;
};

void SpdlogLogger::log_impl(LogLevel level, std::string_view component, uint64_t tick, const std::string& msg) {
    std::string final_log = fmt::format("[{}][{}] {}", tick, component, msg);
    _logger->log(static_cast<spdlog::level::level_enum>(level), final_log);
}