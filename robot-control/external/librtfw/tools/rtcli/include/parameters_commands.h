#pragma once

#include "command_handler.h"
#include <string>
#include <rtfw_connect/shm_connector.h>

namespace rtcli {

/**
 * @brief Parameters management commands
 *
 * Future implementation for:
 * - parameters load <path>  - Load parameters from YAML
 * - parameters reload       - Reload current parameters
 * - parameters list         - List all parameters
 * - parameters get <key>    - Get parameter value
 * - parameters set <key> <value> - Set parameter value
 */
class ParametersCommandHandler : public CommandHandler {
public:
    int execute(const std::vector<std::string>& args, rtfw::connect::SharedMemoryConnector& connector) override;
    std::string name() const override { return "parameters"; }
    std::string help() const override;

private:
    int handle_load(const std::string& path, rtfw::connect::SharedMemoryConnector& connector);
    int handle_reload(rtfw::connect::SharedMemoryConnector& connector);
    int handle_list(rtfw::connect::SharedMemoryConnector& connector);
    int handle_get(const std::string& key, rtfw::connect::SharedMemoryConnector& connector);
    int handle_set(const std::string& key, const std::string& value, rtfw::connect::SharedMemoryConnector& connector);
};

} // namespace rtcli
