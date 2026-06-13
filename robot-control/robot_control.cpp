// Wire Manipulator Control
// 태스크 구성:
//   RT:     Manager(30Hz), SafetyLayer(200Hz), CanBus0(200Hz, core6), CanBus1(200Hz, core7)
//   Non-RT: WsBridgeTask(30Hz)

#include "rtfw/framework.h"
#include "tasks/manager.hpp"
#include "tasks/safety_layer.hpp"
#include "tasks/driver/can_bus0.h"
#include "tasks/driver/can_bus1.h"
#include "tasks/ws_server/ws_bridge_task.hpp"
#include "tasks/data_logger.hpp"

#include <csignal>
#include <sys/mman.h>
#include <sys/resource.h>
#include <cerrno>
#include <cstring>

rtfw::RealTimeFramework* p_rtfw = nullptr;
void signalHandler(int) {
    if (p_rtfw) p_rtfw->stop();
}

bool setup_memory_locking() {
    struct rlimit rlim;
    if (getrlimit(RLIMIT_MEMLOCK, &rlim) == 0 && rlim.rlim_cur < rlim.rlim_max) {
        rlim.rlim_cur = rlim.rlim_max;
        setrlimit(RLIMIT_MEMLOCK, &rlim);
    }
    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
        std::cerr << "WARNING: mlockall failed: " << strerror(errno)
                  << " (continuing in soft real-time mode)\n";
        return false;
    }
    std::cout << "mlockall succeeded\n";
    return true;
}

int main(int /*argc*/, char** /*argv*/) {
    setup_memory_locking();

    try {
        rtfw::RealTimeFramework framework;
        p_rtfw = &framework;
        std::signal(SIGINT, signalHandler);

        // RT 태스크
        framework.registerTask(std::make_unique<task_pool::Manager>(),     30);
        framework.registerTask(std::make_unique<task_pool::SafetyLayer>(), 200);
        framework.registerTask(std::make_unique<task_pool::CanBus0>(),     200, 6);
        framework.registerTask(std::make_unique<task_pool::CanBus1>(),     200, 7);

        // Non-RT 태스크
        framework.registerNonRtTask(std::make_unique<task_pool::WsBridgeTask>(), 30);
        framework.registerNonRtTask(std::make_unique<task_pool::DataLogger>(), 200);

        rtfw::FrameworkConfig config;
        config.mode           = rtfw::Mode::LIVE;
        config.realtime_level = RealtimeLevel::SOFT;
        config.threads.num_common_threads      = 4;
        config.threads.dedicated_core_threads  = {{6, 1}, {7, 1}};
        config.threads.num_non_rt_threads      = 1;
        config.parameter_file_path = "config/robotnl.yaml";
        config.log_level = rtfw::LogLevel::INFO;

        framework.initialize(std::move(config));
        framework.start();
    } catch (const std::exception& e) {
        std::cerr << "FATAL: " << e.what() << "\n";
        return 1;
    }
    return 0;
}
