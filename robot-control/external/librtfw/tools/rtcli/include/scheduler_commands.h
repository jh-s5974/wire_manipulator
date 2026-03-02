#pragma once

#include "command_handler.h"
#include <rtfw_connect/shm_connector.h>

namespace rtcli {

/**
 * @brief Scheduler control commands (pause, play, play_one)
 */
class SchedulerCommandHandler : public CommandHandler {
public:
    int execute(const std::vector<std::string>& args, rtfw::connect::SharedMemoryConnector& connector) override;
    std::string name() const override { return "sched"; }
    std::string help() const override;

private:
    int handle_pause(rtfw::connect::SharedMemoryConnector& connector);
    int handle_play(rtfw::connect::SharedMemoryConnector& connector);
    int handle_play_one(rtfw::connect::SharedMemoryConnector& connector);
};

} // namespace rtcli
