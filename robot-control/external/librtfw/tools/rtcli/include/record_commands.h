#pragma once

#include "command_handler.h"
#include <cstdint>
#include <rtfw_connect/shm_connector.h>

namespace rtcli {

/**
 * @brief Recording, replay, and trace commands
 */
class RecordCommandHandler : public CommandHandler {
public:
    int execute(const std::vector<std::string>& args, rtfw::connect::SharedMemoryConnector& connector) override;
    std::string name() const override { return "record"; }
    std::string help() const override;

private:
    int handle_record_start(const std::string& path, rtfw::connect::SharedMemoryConnector& connector);
    int handle_record_stop(rtfw::connect::SharedMemoryConnector& connector);
};

/**
 * @brief Replay command handler
 */
class ReplayCommandHandler : public CommandHandler {
public:
    int execute(const std::vector<std::string>& args, rtfw::connect::SharedMemoryConnector& connector) override;
    std::string name() const override { return "replay"; }
    std::string help() const override;

private:
    int handle_replay_start(const std::string& path, rtfw::connect::SharedMemoryConnector& connector);
    int handle_replay_stop(rtfw::connect::SharedMemoryConnector& connector);
};

/**
 * @brief Trace command handler (replay + record simultaneously)
 */
class TraceCommandHandler : public CommandHandler {
public:
    int execute(const std::vector<std::string>& args, rtfw::connect::SharedMemoryConnector& connector) override;
    std::string name() const override { return "trace"; }
    std::string help() const override;

private:
    int handle_trace_start(const std::string& replay_path, const std::string& record_path,
                          rtfw::connect::SharedMemoryConnector& connector);
    int handle_trace_stop(rtfw::connect::SharedMemoryConnector& connector);
};

/**
 * @brief Record file validation and analysis
 */
class CheckRecordCommandHandler : public CommandHandler {
public:
    int execute(const std::vector<std::string>& args, rtfw::connect::SharedMemoryConnector& connector) override;
    std::string name() const override { return "check_record"; }
    std::string help() const override;

private:
    int validate_record_file(const char* path, bool show_hash);
};

} // namespace rtcli
