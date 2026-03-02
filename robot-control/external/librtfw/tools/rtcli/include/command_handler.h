#pragma once

#include <memory>
#include <string>
#include <vector>
#include <functional>
#include <unordered_map>
#include <rtfw_connect/shm_connector.h>

namespace rtcli {

/**
 * @brief Base interface for command handlers
 *
 * All command groups (scheduler, recording, parameters, etc.) inherit from this.
 * This allows for easy extension of CLI functionality.
 */
class CommandHandler {
public:
    virtual ~CommandHandler() = default;

    /**
     * @brief Execute the command
     * @param args Arguments after the command group name
     * @param connector SharedMemoryConnector for framework interaction
     * @return Exit code (0 = success)
     */
    virtual int execute(const std::vector<std::string>& args, rtfw::connect::SharedMemoryConnector& connector) = 0;

    /**
     * @brief Get command group name
     */
    virtual std::string name() const = 0;

    /**
     * @brief Get help text for this command group
     */
    virtual std::string help() const = 0;
};

/**
 * @brief Command registry and dispatcher
 */
class CommandDispatcher {
public:
    CommandDispatcher();

    void register_handler(std::unique_ptr<CommandHandler> handler);

    int dispatch(const std::vector<std::string>& args, rtfw::connect::SharedMemoryConnector& connector);

    void print_help() const;

private:
    std::unordered_map<std::string, std::unique_ptr<CommandHandler>> handlers_;
};

} // namespace rtcli
