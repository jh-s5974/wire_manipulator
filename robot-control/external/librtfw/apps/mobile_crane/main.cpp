#include "rtfw/framework.h"
#include <iostream>
#include <chrono>
#include <thread>
#include <cstdlib>
#include <csignal>
#include <sys/mman.h>
#include <unordered_map>

#include "task_pool/joystick.h"
#include "task_pool/imu.h"
#include "task_pool/sm_task.h"
#include "task_pool/crane_task.h"
#include "task_pool/can_if_task.h"
#include "task_pool/marker_tracker.h"
#include "task_pool/mecanum_task.h"
#include "task_pool/odometry.h"
#include "task_pool/post_process.h"
#include "task_pool/tracking_control.h"
#include "task_pool/abnormal_detector.h"
#include "task_pool/selector_task.h"
// #include "task_pool/ros_bridge.h"
#include "task_pool/vision_task.h"
// #include "task_pool/vision_dummy.h"
#include "task_pool/marker_lpf.h"
#include "task_pool/mecanum_iekf.h"
#include "task_pool/marker_iekf_unified.h"


RealTimeFramework *p_rtfw = nullptr;
void signalHandler(int signum) {
    std::cout << "SIGINT detected" << std::endl;
    if (p_rtfw)
        p_rtfw->stop();
}

void print_usage(const char* prog_name) {
    std::cerr << "Usage: " << prog_name << " [--live | --record <out_path> | --simul <src_path> | --trace <src_path> <out_path>]" << std::endl;
}

int main(int argc, char** argv) {
    std::signal(SIGINT, signalHandler);

    rtfw::FrameworkConfig config;
    config.mode = rtfw::Mode::LIVE;
    std::string record_file;
    std::string replay_file;

    // 2. C-style 인자 배열을 std::vector<std::string>으로 변환하여 사용하기 쉽게 만듭니다.
    std::vector<std::string> args(argv + 1, argv + argc);

    // 3. 인자를 순회하며 파싱합니다.
    for (size_t i = 0; i < args.size(); ++i) {
        const std::string& arg = args[i];

        if (arg == "--live") {
            config.mode = rtfw::Mode::LIVE;
        } else if (arg == "--record") {
            config.mode = rtfw::Mode::RECORDING;
            if (++i >= args.size()) { // 다음 인자가 없으면 에러
                std::cerr << "Error: --record option requires an output path." << std::endl;
                print_usage(argv[0]);
                return -1;
            }
            record_file = args[i];
        } else if (arg == "--simul") {
            config.mode = rtfw::Mode::SIMULATION;
            if (++i >= args.size()) { // 다음 인자가 없으면 에러
                std::cerr << "Error: --simulation option requires a source path." << std::endl;
                print_usage(argv[0]);
                return -1;
            }
            replay_file = args[i];
        } else if (arg == "--trace") {
            config.mode = rtfw::Mode::TRACE;
            if (i + 2 >= args.size()) { // 다음 인자가 2개 없으면 에러
                std::cerr << "Error: --trace option requires a source and an output path." << std::endl;
                print_usage(argv[0]);
                return -1;
            }
            replay_file = args[++i];
            record_file = args[++i];
        } else {
            std::cerr << "Error: Unknown option '" << arg << "'" << std::endl;
            print_usage(argv[0]);
            return -1;
        }
    }


    // if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
    //     std::cerr << "WARNING: mlockall failed: " << strerror(errno) << ". Run with sudo or check user limits." << std::endl;
    // }

    rtfw::RealTimeFramework framework;
    try {
        p_rtfw = &framework;

        // Blackbox 설정
        if (config.mode == rtfw::Mode::RECORDING) {
            auto recorder = std::make_unique<rtfw::blackbox::FileBlackbox>();
            if(!recorder->start(record_file, rtfw::blackbox::Mode::RECORD)) return -2;
            config.blackbox.record_backend = std::move(recorder);
            config.realtime_level = rtfw::RealtimeLevel::HARD;
        } 
        else if (config.mode == rtfw::Mode::SIMULATION) {
            auto replayer = std::make_unique<rtfw::blackbox::FileBlackbox>();
            if(!replayer->start(replay_file, rtfw::blackbox::Mode::REPLAY)) return -2;
            config.blackbox.replay_backend = std::move(replayer);
            config.realtime_level = rtfw::RealtimeLevel::SOFT;
        }
        else if (config.mode == rtfw::Mode::TRACE) {
            // Trace 모드는 두 개의 백엔드를 모두 사용
            auto recorder = std::make_unique<rtfw::blackbox::FileBlackbox>();
            if(!recorder->start(record_file, rtfw::blackbox::Mode::RECORD)) return -2;

            auto replayer = std::make_unique<rtfw::blackbox::FileBlackbox>();
            if(!replayer->start(replay_file, rtfw::blackbox::Mode::REPLAY)) return -2;

            config.blackbox.record_backend = std::move(recorder);
            config.blackbox.replay_backend = std::move(replayer);
            config.realtime_level = rtfw::RealtimeLevel::SOFT;
        }


        // RT Task 등록
        framework.registerTask(std::make_unique<task_pool::StateMachine>(), 100, 1);
        framework.registerTask(std::make_unique<task_pool::WheelIF>(), 100, 1);
        framework.registerTask(std::make_unique<task_pool::ImuReader>(), 100, 1);
        framework.registerTask(std::make_unique<task_pool::CraneTask>(), 100, 1);
        framework.registerTask(std::make_unique<task_pool::TrackingControl>(), 100, 1);
        framework.registerTask(std::make_unique<task_pool::MecanumFK>(), 100, 1);
        framework.registerTask(std::make_unique<task_pool::MecanumIK>(), 100, 1);
        framework.registerTask(std::make_unique<task_pool::Odometry>(), 100, 1);
        framework.registerTask(std::make_unique<task_pool::AbnormalDetector>(), 25, 1);
        framework.registerTask(std::make_unique<task_pool::PostProcessTask>(), 25, 1);
        // framework.registerTask(std::make_unique<task_pool::PostProcessLPF>(), 25, 1);
        // framework.registerTask(std::make_unique<task_pool::PostProcessESKF>(), 25, 1);
        // framework.registerTask(std::make_unique<task_pool::MarkerTracker>(), 25, 1);
        framework.registerTask(std::make_unique<task_pool::MarkerTracker>(), 100, 1);
        framework.registerTask(std::make_unique<task_pool::VelocitySelector>(), 100, 1);
        framework.registerTask(std::make_unique<task_pool::CraneSpdSelector>(), 100, 1);
        framework.registerTask(std::make_unique<task_pool::MarkerLPF>(), 25, 1);
        // framework.registerTask(std::make_unique<task_pool::MarkerESKF>(), 25, 1);
        framework.registerTask(std::make_unique<task_pool::IEKFOdometry>(), 100, 1);
        framework.registerTask(std::make_unique<task_pool::MarkerIEKFTask>(), 100, 1);
        // framework.registerTask(std::make_unique<task_pool::NotchFilterTask>(), 100, 1);
        
        // Non-RT Task 등록
        framework.registerNonRtTask(std::make_unique<task_pool::Joystick>(), 100);
        framework.registerNonRtTask(std::make_unique<task_pool::VisionTask>(), 25);
        // framework.registerNonRtTask(std::make_unique<task_pool::DataLogger>());
        // framework.registerNonRtTask(std::make_unique<task_pool::SimpleRosBridge>(argc, argv));
        // if (archive.get_mode() == Mode::REPLAY) {
        //     framework.registerNonRtTask(std::make_unique<task_pool::DataExtractor>());
        // }

        // 스레드 풀 설정        
        config.threads.num_common_threads = 4;
        config.threads.dedicated_core_threads = {{1, 4}};
        config.threads.num_non_rt_threads = 2;
        config.parameter_file_path = "crane_parameters.yaml";

        framework.initialize(std::move(config));
        framework.start();
        
    } catch (const std::exception& e) {
        std::cerr << "FATAL ERROR: " << e.what() << std::endl;
        return 1;
    }
    // framework.printStats();
    return 0;
}