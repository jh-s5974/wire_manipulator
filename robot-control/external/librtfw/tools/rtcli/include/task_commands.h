#pragma once

#include "command_handler.h"
#include <string>
#include <vector>
#include <rtfw_connect/shm_connector.h>

namespace rtcli {

class TaskCommandHandler : public CommandHandler {
public:
    int execute(const std::vector<std::string>& args, rtfw::connect::SharedMemoryConnector& connector) override;
    std::string name() const override { return "task"; }
    std::string help() const override;

private:
    int setTaskEnabled(rtfw::connect::SharedMemoryConnector& connector, uint32_t task_id, bool enabled);
    int showTaskList(rtfw::connect::SharedMemoryConnector& connector);
    int showTaskInfo(rtfw::connect::SharedMemoryConnector& connector, uint32_t task_id, bool verbose = false);
};

} // namespace rtcli

