// ============================================================================
// EXAMPLE: How to Add a New Command Handler
// ============================================================================
//
// This file demonstrates the pattern for adding new command handlers to rt_cli.
// 
// Example: Add a "status" command to check framework status
// ============================================================================

#pragma once

#include "command_handler.h"

namespace rtcli {

/**
 * @brief Status checking commands (example template)
 *
 * Usage examples:
 *   rt_cli status framework
 *   rt_cli status tasks
 *   rt_cli status memory
 */
class StatusCommandHandler : public CommandHandler {
public:
    int execute(const std::vector<std::string>& args, SharedMemoryContext& ctx) override;
    std::string name() const override { return "status"; }
    std::string help() const override;

private:
    int handle_framework(SharedMemoryContext& ctx);
    int handle_tasks(SharedMemoryContext& ctx);
    int handle_memory(SharedMemoryContext& ctx);
};

} // namespace rtcli

/*
// ============================================================================
// IMPLEMENTATION STEPS:
// ============================================================================
//
// 1. Create status_commands.h with handler class (as above)
//
// 2. Create status_commands.cpp with implementation:
//
//    #include "status_commands.h"
//    #include "shm_common.h"
//    #include <iostream>
//
//    namespace rtcli {
//
//    int StatusCommandHandler::execute(const std::vector<std::string>& args,
//                                       SharedMemoryContext& ctx) {
//        if (!ctx.is_valid()) {
//            std::cerr << "Error: Invalid shared memory context" << std::endl;
//            return 2;
//        }
//
//        if (args.empty()) {
//            std::cerr << "status: missing subcommand" << std::endl;
//            std::cout << help();
//            return 1;
//        }
//
//        const auto& cmd = args[0];
//        if (cmd == "framework") {
//            return handle_framework(ctx);
//        } else if (cmd == "tasks") {
//            return handle_tasks(ctx);
//        } else if (cmd == "memory") {
//            return handle_memory(ctx);
//        } else {
//            std::cerr << "status: unknown subcommand '" << cmd << "'" << std::endl;
//            std::cout << help();
//            return 1;
//        }
//    }
//
//    int StatusCommandHandler::handle_framework(SharedMemoryContext& ctx) {
//        // Access shared memory via ctx.header()
//        // Example: std::cout << "Framework state: " << ctx.header()->... << std::endl;
//        return 0;
//    }
//
//    int StatusCommandHandler::handle_tasks(SharedMemoryContext& ctx) {
//        // TODO: Implement
//        return 0;
//    }
//
//    int StatusCommandHandler::handle_memory(SharedMemoryContext& ctx) {
//        // TODO: Implement
//        return 0;
//    }
//
//    std::string StatusCommandHandler::help() const {
//        return
//            "status                  - Check framework status\n"
//            "  framework             - Framework state\n"
//            "  tasks                 - Task list and states\n"
//            "  memory                - Memory usage\n";
//    }
//
//    } // namespace rtcli
//
// 3. Update tools/CMakeLists.txt to include status_commands.cpp:
//
//    add_executable(rt_cli
//        rt_cli/main.cpp
//        rt_cli/shm_common.cpp
//        rt_cli/command_handler.cpp
//        rt_cli/scheduler_commands.cpp
//        rt_cli/record_commands.cpp
//        rt_cli/status_commands.cpp      // ADD THIS LINE
//    )
//
// 4. Update main.cpp to register the new handler:
//
//    #include "status_commands.h"
//
//    // In main():
//    dispatcher.register_handler(std::make_unique<rtcli::StatusCommandHandler>());
//
// 5. Build and test:
//
//    cd build && cmake --build . --target rt_cli
//    ./tools/rt_cli status framework
//
// ============================================================================
*/
