#include "rtfw/framework.h"
#include "ros_test_tasks.h"
#include "ros_bridge_task.h"
#include <csignal>
#include <sys/mman.h>
#include <sys/resource.h>
#include <cerrno>
#include <cstring>
#include <rclcpp/rclcpp.hpp>

rtfw::RealTimeFramework *p_rtfw = nullptr;

void signalHandler(int signum) {
    std::cout << "SIGINT detected" << std::endl;
    if (p_rtfw)
        p_rtfw->stop();
}

bool setup_memory_locking() {
    struct rlimit rlim;
    if (getrlimit(RLIMIT_MEMLOCK, &rlim) == 0) {
        std::cout << "Current memlock limit: " << rlim.rlim_cur 
                  << " bytes (max: " << rlim.rlim_max << ")" << std::endl;
        
        if (rlim.rlim_cur < rlim.rlim_max) {
            rlim.rlim_cur = rlim.rlim_max;
            if (setrlimit(RLIMIT_MEMLOCK, &rlim) == 0) {
                std::cout << "Memlock limit increased to " << rlim.rlim_cur << " bytes" << std::endl;
            } else {
                std::cerr << "Warning: Could not increase memlock limit: " << strerror(errno) << std::endl;
            }
        }
    }

    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
        std::cerr << "WARNING: mlockall failed: " << strerror(errno) << std::endl;
        return false;
    }

    std::cout << "✓ mlockall succeeded: all memory locked in RAM" << std::endl;
    return true;
}

void print_usage(const char* prog_name) {
    std::cerr << "Usage: " << prog_name << " [--live | --record <out_path>]" << std::endl;
}

int main(int argc, char** argv) {
    std::cout << "=== RT Framework + ROS2 Test ===" << std::endl;
    bool memory_locked = setup_memory_locking();
    if (memory_locked) {
        std::cout << "Real-time memory guarantees enabled." << std::endl;
    } else {
        std::cout << "Running in degraded mode (soft real-time)." << std::endl;
    }
    std::cout << "==================================" << std::endl << std::endl;

    rtfw::FrameworkConfig config;
    config.mode = rtfw::Mode::LIVE;

    std::vector<std::string> args(argv + 1, argv + argc);
    for (size_t i = 0; i < args.size(); ++i) {
        const std::string& arg = args[i];
        if (arg == "--record" && i + 1 < args.size()) {
            config.mode = rtfw::Mode::RECORDING;
            ++i;
        }
    }

    try {
        // Initialize ROS2 before framework
        rclcpp::init(argc, argv);
        auto ros_node = std::make_shared<rclcpp::Node>("rtfw_ros_test");

        rtfw::RealTimeFramework framework;
        p_rtfw = &framework;
        std::signal(SIGINT, signalHandler);

        // Blackbox configuration
        auto recorder = std::make_unique<rtfw::blackbox::FileBlackbox>();
        auto replayer = std::make_unique<rtfw::blackbox::FileBlackbox>();
        config.blackbox.record_backend = std::move(recorder);
        config.blackbox.replay_backend = std::move(replayer);

        // --- RT 태스크 등록 (같은 구조로 poc_app과 동일) ---
        // 1000Hz Timeline (Core 7)
        framework.registerTask(std::make_unique<SensorProducer>(), 1000, 7);
        framework.registerTask(std::make_unique<TorqueController>(), 1000, 7);
        
        // 200Hz Timeline (Core 6)
        framework.registerTask(std::make_unique<StateEstimator>(), 200, 6);

        // 30Hz Timeline (Core 5)
        framework.registerTask(std::make_unique<VisionProcessor>(), 30);
        framework.registerTask(std::make_unique<GaitPlanner>(), 30);

        // --- Non-RT 태스크: ROS2 Bridge ---
        framework.registerNonRtTask(std::make_unique<RosBridgeTask>(ros_node), 100);

        // 스레드 풀 설정
        config.threads.num_common_threads = 4;
        config.threads.dedicated_core_threads = {{7, 1}, {6, 1}, {5, 1}};
        config.threads.num_non_rt_threads = 2;

        config.log_level = rtfw::LogLevel::DEBUG;

        std::cout << "\n>>> Starting RTFW with ROS2 bridge..." << std::endl;
        std::cout << ">>> ROS2 Topics exposed:" << std::endl;
        std::cout << "    - /sensors/high_freq (Float64MultiArray)" << std::endl;
        std::cout << "    - /robot/state (Float64MultiArray)" << std::endl;
        std::cout << "    - /motor/torques (Float64MultiArray)" << std::endl;
        std::cout << "    - /robot/gait (Float64MultiArray)" << std::endl;
        std::cout << "    - /vision/object (Float64MultiArray)" << std::endl;
        std::cout << ">>> Press Ctrl+C to stop\n" << std::endl;

        framework.initialize(std::move(config));
        framework.start();

        // Shutdown ROS2
        rclcpp::shutdown();

        framework.printStats();

    } catch (const std::exception& e) {
        std::cerr << "FATAL: " << e.what() << std::endl;
        rclcpp::shutdown();
        return 1;
    }
    return 0;
}
