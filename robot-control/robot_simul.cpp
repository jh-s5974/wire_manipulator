// Wire Manipulator — MuJoCo 시뮬레이션
// 태스크 구성:
//   RT:     Manager(30Hz), SafetyLayer(200Hz)
//   Non-RT: MujocoEnv(200Hz), WsBridgeTask(30Hz)

#include "rtfw/framework.h"
#include "tasks/manager.hpp"
#include "tasks/safety_layer.hpp"
#include "tasks/mujoco_env.hpp"
#include "tasks/ws_server/ws_bridge_task.hpp"
#include "tasks/data_logger.hpp"

#include <csignal>
#include <sys/mman.h>
#include <sys/resource.h>
#include <sys/file.h>
#include <fcntl.h>
#include <cerrno>
#include <cstring>

rtfw::RealTimeFramework* p_rtfw = nullptr;
int g_lock_fd = -1;

bool acquire_lock(const char* path) {
    g_lock_fd = open(path, O_CREAT | O_RDWR, 0666);
    if (g_lock_fd < 0) {
        std::cerr << "ERROR: cannot open lock file " << path << ": " << strerror(errno) << "\n";
        return false;
    }
    if (flock(g_lock_fd, LOCK_EX | LOCK_NB) != 0) {
        std::cerr << "ERROR: another robot_simul instance is already running.\n";
        close(g_lock_fd);
        g_lock_fd = -1;
        return false;
    }
    return true;
}

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
                  << " (soft real-time mode)\n";
        return false;
    }
    std::cout << "mlockall succeeded\n";
    return true;
}

int main(int /*argc*/, char** /*argv*/) {
    if (!acquire_lock("/tmp/robot_simul.lock")) return 1;
    setup_memory_locking();

    try {
        rtfw::RealTimeFramework framework;
        p_rtfw = &framework;
        std::signal(SIGINT, signalHandler);

        // RT 태스크
        framework.registerTask(std::make_unique<task_pool::Manager>(),     30);
        framework.registerTask(std::make_unique<task_pool::SafetyLayer>(), 200);

        // Non-RT 태스크 (MujocoEnv는 GLFW 창 관리로 non-RT)
        framework.registerTask(std::make_unique<task_pool::MujocoEnv>(), 200);
        framework.registerNonRtTask(std::make_unique<task_pool::WsBridgeTask>(), 30);
        framework.registerNonRtTask(std::make_unique<task_pool::DataLogger>(), 200);

        rtfw::FrameworkConfig config;
        config.mode           = rtfw::Mode::LIVE;
        config.realtime_level = RealtimeLevel::SOFT;
        config.threads.num_common_threads     = 4;
        config.threads.dedicated_core_threads = {{6, 1}, {7, 1}};
        config.threads.num_non_rt_threads     = 2;
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
