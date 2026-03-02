#include <iostream>
#include <vector>
#include <string>
#include <memory>

#include "command_handler.h"
#include "scheduler_commands.h"
#include "record_commands.h"
#include "task_commands.h"
#include "data_commands.h"
#include <rtfw_connect/shm_connector.h>

int main(int argc, char** argv) {
    try {
        // Create command dispatcher
        rtcli::CommandDispatcher dispatcher;

        // Register command handlers
        dispatcher.register_handler(std::make_unique<rtcli::SchedulerCommandHandler>());
        dispatcher.register_handler(std::make_unique<rtcli::RecordCommandHandler>());
        dispatcher.register_handler(std::make_unique<rtcli::ReplayCommandHandler>());
        dispatcher.register_handler(std::make_unique<rtcli::TraceCommandHandler>());
        dispatcher.register_handler(std::make_unique<rtcli::CheckRecordCommandHandler>());
        dispatcher.register_handler(std::make_unique<rtcli::TaskCommandHandler>());
        dispatcher.register_handler(std::make_unique<rtcli::DataCommandHandler>());

        if (argc < 2) {
            dispatcher.print_help();
            return 1;
        }

        // Convert command line arguments to vector
        std::vector<std::string> args;
        for (int i = 1; i < argc; ++i) {
            args.push_back(argv[i]);
        }

        // Handle special case: check_record doesn't need SHM connection
        if (!args.empty() && args[0] == "check_record") {
            // Create a dummy connector (not used for check_record)
            rtfw::connect::SharedMemoryConnector dummy_connector;
            return dispatcher.dispatch(args, dummy_connector);
        }

        // Connect to shared memory for other commands
        rtfw::connect::SharedMemoryConnector connector;
        if (connector.connect("rt_framework_shm") == nullptr) {
            std::cerr << "Error: Failed to connect to framework shared memory" << std::endl;
            return 2;
        }

        // Dispatch command
        int result = dispatcher.dispatch(args, connector);

        // Cleanup
        connector.cleanup();

        return result;

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 2;
    }
}
