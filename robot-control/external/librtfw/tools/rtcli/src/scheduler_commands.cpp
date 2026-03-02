#include "scheduler_commands.h"
#include <iostream>
#include <rtfw_connect/shm_querier.h>
#include <rtfw_connect/shm_controller.h>

namespace rtcli {

int SchedulerCommandHandler::execute(const std::vector<std::string>& args, rtfw::connect::SharedMemoryConnector& connector) {
    if (!connector.isAttached()) {
        std::cerr << "Error: Failed to connect to shared memory" << std::endl;
        return 2;
    }

    if (args.empty()) {
        std::cerr << "sched: missing subcommand" << std::endl;
        std::cout << help();
        return 1;
    }

    const auto& cmd = args[0];
    if (cmd == "pause") {
        return handle_pause(connector);
    } else if (cmd == "play") {
        return handle_play(connector);
    } else if (cmd == "play_one") {
        return handle_play_one(connector);
    } else {
        std::cerr << "sched: unknown subcommand '" << cmd << "'" << std::endl;
        std::cout << help();
        return 1;
    }
}

int SchedulerCommandHandler::handle_pause(rtfw::connect::SharedMemoryConnector& connector) {
    rtfw::connect::SharedMemoryController controller(connector.getBasePtr());
    controller.pauseTick();
    std::cout << "Wrote PAUSE_TICK" << std::endl;
    return 0;
}

int SchedulerCommandHandler::handle_play(rtfw::connect::SharedMemoryConnector& connector) {
    rtfw::connect::SharedMemoryController controller(connector.getBasePtr());
    controller.playTick();
    std::cout << "Wrote PLAY_TICK" << std::endl;
    return 0;
}

int SchedulerCommandHandler::handle_play_one(rtfw::connect::SharedMemoryConnector& connector) {
    rtfw::connect::SharedMemoryController controller(connector.getBasePtr());
    controller.playOneTick();
    std::cout << "Wrote PLAY_ONE_TICK" << std::endl;
    return 0;
}

std::string SchedulerCommandHandler::help() const {
    return
        "sched                   - Control tick scheduling\n"
        "  pause                 - Pause tick processing\n"
        "  play                  - Resume tick processing\n"
        "  play_one              - Advance one tick\n";
}

} // namespace rtcli
