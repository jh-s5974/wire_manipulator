#pragma once

#include "command_handler.h"
#include <string>
#include <vector>
#include <rtfw_connect/shm_connector.h>

namespace rtcli {

/**
 * @brief Data key inspection commands (list, info)
 */
class DataCommandHandler : public CommandHandler {
public:
    int execute(const std::vector<std::string>& args, rtfw::connect::SharedMemoryConnector& connector) override;
    std::string name() const override { return "data"; }
    std::string help() const override;

private:
    int showDataList(rtfw::connect::SharedMemoryConnector& connector);
    int showDataInfo(rtfw::connect::SharedMemoryConnector& connector, const std::string& key);
};

} // namespace rtcli
