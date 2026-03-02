#include "rtfw/framework.h"
#include "biped_task.h" // 위에 정의된 모든 태스크들의 헤더
#include <csignal>
#include <sys/mman.h>
#include <sys/resource.h>
#include <cerrno>
#include <cstring>


rtfw::RealTimeFramework *p_rtfw = nullptr;
void signalHandler(int signum) {
    std::cout << "SIGINT detected" << std::endl;
    if (p_rtfw)
        p_rtfw->stop();
}

// mlockall 설정 헬퍼 함수
bool setup_memory_locking() {
    // 1. 현재 memlock 제한 확인
    struct rlimit rlim;
    if (getrlimit(RLIMIT_MEMLOCK, &rlim) == 0) {
        std::cout << "Current memlock limit: " << rlim.rlim_cur 
                  << " bytes (max: " << rlim.rlim_max << ")" << std::endl;
        
        // 제한이 충분하지 않으면 최대로 증가 시도
        if (rlim.rlim_cur < rlim.rlim_max) {
            rlim.rlim_cur = rlim.rlim_max;
            if (setrlimit(RLIMIT_MEMLOCK, &rlim) == 0) {
                std::cout << "Memlock limit increased to " << rlim.rlim_cur << " bytes" << std::endl;
            } else {
                std::cerr << "Warning: Could not increase memlock limit: " << strerror(errno) << std::endl;
            }
        }
    }

    // 2. mlockall 호출 (MCL_CURRENT: 현재 매핑된 페이지, MCL_FUTURE: 미래 할당)
    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
        std::cerr << "WARNING: mlockall failed: " << strerror(errno) << std::endl;
        std::cerr << "Possible causes:" << std::endl;
        std::cerr << "  - Insufficient memlock limit (current: " << rlim.rlim_cur 
                  << " bytes, try 'ulimit -l unlimited')" << std::endl;
        std::cerr << "  - Need CAP_IPC_LOCK capability or root privileges" << std::endl;
        std::cerr << "  - Insufficient physical memory available" << std::endl;
        std::cerr << std::endl;
        std::cerr << "To fix, run one of:" << std::endl;
        std::cerr << "  sudo prlimit --memlock=unlimited --pid=$$" << std::endl;
        std::cerr << "  sudo setcap cap_ipc_lock=ep ./build/apps/poc_app/poc_app" << std::endl;
        std::cerr << std::endl;
        std::cerr << "Continuing without memory locking (may experience page faults)..." << std::endl;
        return false;
    }

    std::cout << "✓ mlockall succeeded: all memory locked in RAM" << std::endl;
    std::cout << "  - Page faults eliminated for deterministic RT performance" << std::endl;
    return true;
}

void print_usage(const char* prog_name) {
    std::cerr << "Usage: " << prog_name << " [--live | --record <out_path> | --simul <src_path> | --trace <src_path> <out_path>]" << std::endl;
}

int main(int argc, char** argv) {
    // ====================================================================
    // CRITICAL: mlockall must be called FIRST, before any major allocations
    // ====================================================================
    std::cout << "=== RT Framework Memory Locking Setup ===" << std::endl;
    bool memory_locked = setup_memory_locking();
    if (memory_locked) {
        std::cout << "Real-time memory guarantees enabled." << std::endl;
    } else {
        std::cout << "Running in degraded mode (soft real-time)." << std::endl;
    }
    std::cout << "===========================================" << std::endl << std::endl;

    rtfw::FrameworkConfig config;
    config.mode = rtfw::Mode::LIVE;
    std::string record_file;
    std::string replay_file;

    // 2. C-style 인자 배열을 std::vector<std::string>으로 변환하여 사용하기 쉽게 만듭니다.
    std::vector<std::string> args(argv + 1, argv + argc);

    // 3. 인자를 순회하며 파싱합니다.
    for (size_t i = 0; i < args.size(); ++i) {
        const std::string& arg = args[i];

        config.mode = rtfw::Mode::LIVE;
    }

    try {
        // 1. "비활성" 상태의 싱크로 로거를 미리 설정
        //    (큐 크기 8192, 작업 스레드 1개)

        rtfw::RealTimeFramework framework;
        p_rtfw = &framework;
        std::signal(SIGINT, signalHandler);

        // Blackbox 설정
        auto recorder = std::make_unique<rtfw::blackbox::FileBlackbox>();
        auto replayer = std::make_unique<rtfw::blackbox::FileBlackbox>();

        config.blackbox.record_backend = std::move(recorder);
        config.blackbox.replay_backend = std::move(replayer);


        // --- RT 태스크 등록 ---
        // 1000Hz Timeline (Core 7)
        framework.registerTask(std::make_unique<SensorProducer>(), 1000, 7);
        framework.registerTask(std::make_unique<TorqueController>(), 1000, 7);
        
        // 200Hz Timeline (Core 6)
        framework.registerTask(std::make_unique<StateEstimator>(), 200, 6);

        // 30Hz Timeline (Core 5)
        framework.registerTask(std::make_unique<VisionProcessor>(), 30);
        framework.registerTask(std::make_unique<GaitPlanner>(), 30);

        // 10Hz Timeline (Common RT) - Task example
        framework.registerTask(std::make_unique<StatefulCounterTask>(), 10);

        // --- Non-RT 태스크 등록 ---
        framework.registerNonRtTask(std::make_unique<ParameterTuner>(), 1); // 1Hz
        framework.registerNonRtTask(std::make_unique<DataLogger>(), 10);   // 10Hz

        // 스레드 풀 설정
        config.threads.num_common_threads = 4;
        config.threads.dedicated_core_threads = {{7, 1}, {6, 1}, {5, 1}};
        config.threads.num_non_rt_threads = 2;

        config.parameter_file_path = "param_test.yaml";
        config.log_level = rtfw::LogLevel::DEBUG;

        framework.initialize(std::move(config));
        framework.start();
        // start()가 반환되면 프레임워크가 완전히 정리됨

        // 종료 후 통계 출력
        framework.printStats();

    } catch (const std::exception& e) {
        std::cerr << "FATAL: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}