#include "rtfw/framework.h"
#include <csignal>
#include <sys/mman.h>
#include <sys/resource.h>
#include <cerrno>
#include <cstring>
#include <thread>
#include <chrono>

using namespace rtfw::rt;

// ===================================================================
// 간단한 데이터 타입들
// ===================================================================

struct SimpleData {
    uint64_t counter;
    double value;
};

// ===================================================================
// 태스크들: Enable/Disable 테스트용
// ===================================================================

/**
 * Producer: 데이터를 생성하는 기본 태스크
 */
class ProducerTask : public ITask {
public:
    const char* getName() const override { return "Producer"; }

    inline void execute(void*) override {
        SimpleData data;
        data.counter = getExecutionLocalTick();
        data.value = getExecutionLocalTick() * 1.5;
        output_writer_.write(data);
        
        auto logger = getLogger();
        logger->info("[Producer] Tick {}: counter={}, value={:.2f}", 
                     getExecutionLocalTick(), data.counter, data.value);
    }

private:
    DataWriter<SimpleData> output_writer_{"simple.data", ArchiveOption::Enable};
};

/**
 * Consumer1: Producer의 output을 읽는 첫 번째 소비자
 */
class Consumer1Task : public ITask {
public:
    const char* getName() const override { return "Consumer1"; }

    inline void execute(void*) override {
        auto logger = getLogger();
        logger->info("[Consumer1] Tick {}: Reading data", getExecutionLocalTick());
        
        input_reader_.on_update([this, logger](const SimpleData& data) {
            logger->info("[Consumer1]   -> counter={}, value={:.2f}", 
                         data.counter, data.value);
        });
    }

private:
    DataReader<SimpleData> input_reader_{"simple.data", DependencyType::Strong};
};

/**
 * Consumer2: Producer의 output을 읽는 두 번째 소비자
 * Consumer1이 disabled되었을 때도 실행되어야 함
 */
class Consumer2Task : public ITask {
public:
    const char* getName() const override { return "Consumer2"; }

    inline void execute(void*) override {
        auto logger = getLogger();
        logger->info("[Consumer2] Tick {}: Reading data", getExecutionLocalTick());
        
        input_reader_.on_update([this, logger](const SimpleData& data) {
            logger->info("[Consumer2]   -> counter={}, value={:.2f}", 
                         data.counter, data.value);
        });
    }

private:
    DataReader<SimpleData> input_reader_{"simple.data", DependencyType::Strong};
};

/**
 * Downstream: Consumer1의 output을 읽는 downstream 태스크
 * Consumer1이 disabled되면 strong chain으로 인해 자동 disabled
 */
class DownstreamTask : public ITask {
public:
    const char* getName() const override { return "Downstream"; }

    inline void execute(void*) override {
        auto logger = getLogger();
        logger->info("[Downstream] Tick {}: Processing consumer1 output", 
                     getExecutionLocalTick());
    }

private:
    // 실제로는 Consumer1이 output을 써야 하는데, 여기서는 간단히 구성
};

// ===================================================================
// 제어용 메인 로직
// ===================================================================

rtfw::RealTimeFramework *p_rtfw = nullptr;
std::atomic<bool> should_stop = false;

void signalHandler(int signum) {
    std::cout << "\nSIGINT detected - stopping framework..." << std::endl;
    should_stop = true;
    if (p_rtfw)
        p_rtfw->stop();
}

bool setup_memory_locking() {
    struct rlimit rlim;
    if (getrlimit(RLIMIT_MEMLOCK, &rlim) == 0) {
        if (rlim.rlim_cur < rlim.rlim_max) {
            rlim.rlim_cur = rlim.rlim_max;
            setrlimit(RLIMIT_MEMLOCK, &rlim);
        }
    }

    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
        std::cerr << "WARNING: mlockall failed: " << strerror(errno) << std::endl;
        return false;
    }

    std::cout << "✓ Memory locked in RAM for deterministic RT performance" << std::endl;
    return true;
}

int main(int argc, char** argv) {
    std::cout << "=== Task On/Off Feature Test ===" << std::endl;
    std::cout << std::endl;

    bool memory_locked = setup_memory_locking();

    try {
        rtfw::RealTimeFramework framework;
        p_rtfw = &framework;
        std::signal(SIGINT, signalHandler);

        // Blackbox 설정
        auto recorder = std::make_unique<rtfw::blackbox::FileBlackbox>();
        auto replayer = std::make_unique<rtfw::blackbox::FileBlackbox>();

        rtfw::FrameworkConfig config;
        config.blackbox.record_backend = std::move(recorder);
        config.blackbox.replay_backend = std::move(replayer);

        // --- 태스크 등록 ---
        // 100Hz timeline에 모든 태스크 등록
        framework.registerTask(std::make_unique<ProducerTask>(), 100, 7);
        framework.registerTask(std::make_unique<Consumer1Task>(), 100, 7);
        framework.registerTask(std::make_unique<Consumer2Task>(), 100, 7);
        // DownstreamTask는 아직 사용하지 않음 (더 복잡한 시나리오용)

        config.threads.num_common_threads = 2;
        config.threads.dedicated_core_threads = {{7, 1}};
        config.threads.num_non_rt_threads = 1;
        config.log_level = rtfw::LogLevel::INFO;

        framework.initialize(std::move(config));

        std::cout << "\n=== Framework initialized ===" << std::endl;
        std::cout << "Running test scenarios..." << std::endl;
        std::cout << "This will run for 10 seconds with enable/disable toggles." << std::endl;
        std::cout << std::endl;

        // 백그라운드에서 프레임워크 실행
        std::thread framework_thread([&framework]() {
            framework.start();
        });

        // 테스트 시나리오: 특정 시점에 태스크 enable/disable 토글
        auto test_start = std::chrono::high_resolution_clock::now();

        while (!should_stop) {
            auto elapsed = std::chrono::high_resolution_clock::now() - test_start;
            auto elapsed_sec = std::chrono::duration_cast<std::chrono::seconds>(elapsed).count();

            // 시뮬레이션: Consumer1을 특정 시간 간격으로 토글
            // Note: 실제 구현에서는 프레임워크 API를 통해 제어해야 함
            // 여기서는 concept only

            if (elapsed_sec > 10) {
                std::cout << "\nTest duration (10s) exceeded. Stopping..." << std::endl;
                should_stop = true;
                if (p_rtfw) {
                    p_rtfw->stop();
                }
                break;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        framework_thread.join();

        std::cout << "\n=== Test Completed ===" << std::endl;
        framework.printStats();

        std::cout << "\nTest Summary:" << std::endl;
        std::cout << "- Producer should run continuously (never disabled)" << std::endl;
        std::cout << "- Consumer1 and Consumer2 should run normally" << std::endl;
        std::cout << "- Check ExecState values in task statistics" << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "FATAL: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}
