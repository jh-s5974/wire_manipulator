#include "command_handler.h"
#include <iostream>

namespace rtcli {

CommandDispatcher::CommandDispatcher() = default;

void CommandDispatcher::register_handler(std::unique_ptr<CommandHandler> handler) {
    if (handler) {
        handlers_[handler->name()] = std::move(handler);
    }
}

int CommandDispatcher::dispatch(const std::vector<std::string>& args, rtfw::connect::SharedMemoryConnector& connector) {
    if (args.empty()) {
        print_help();
        return 1;
    }

    const auto& command_name = args[0];
    auto it = handlers_.find(command_name);
    if (it == handlers_.end()) {
        std::cerr << "Unknown command: " << command_name << std::endl;
        print_help();
        return 1;
    }

    std::vector<std::string> handler_args(args.begin() + 1, args.end());
    return it->second->execute(handler_args, connector);
}

void CommandDispatcher::print_help() const {
    std::cout
        << "rt_cli - Real-Time Framework CLI\n"
        << "\n"
        << "Usage: rt_cli <command> [args]\n"
        << "\n"
        << "Commands:\n";

    for (const auto& [name, handler] : handlers_) {
        std::cout << "\n  " << handler->help();
    }
}

} // namespace rtcli
