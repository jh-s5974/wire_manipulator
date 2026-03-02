// test_app/main.cpp
//
// Stress-test application for the RT framework.
//
// Tasks layout (49 total):
//   1000 Hz  Core 5  (2 dedicated threads) – Gen1k A/B/C/D, Filter1k A/B/C, Ctrl1k
//    500 Hz  Core 4  (1 dedicated thread)  – Fuse500 A/B/C, Est500 A/B/C
//    200 Hz  Core 3  (1 dedicated thread)  – Plan200 A/B/C, Diag200 A/B
//    100 Hz  Common RT pool               – Monitor100 A/B/C/D/E
//     50 Hz  Common RT pool               – Health50 A/B/C/D, Stress50 A/B/C/D
//     10 Hz  Common RT pool               – Stateful10 A/B/C/D + Summary
//             +  DeriveRT_Pure, DeriveRT_TickTainted  (replay verification)
//      1 Hz  Non-RT                       – ParamGen A/B/C
//             +  DeriveNonRT_Pure, DeriveNonRT_TickTainted  (replay verification)
//      5 Hz  Non-RT                       – StatsLog A, DiagLog B, HealthLog C, PerfLog D
//             +  DerivedCheck_Logger  (logs all derived verification channels)
//
// Usage:
//   test_app                  → LIVE mode
//   test_app --record <path>  → LIVE + write record to <path>
//   test_app --replay <path>  → LIVE + replay from <path>

#include "rtfw/framework.h"
#include "test_tasks.h"
#include <csignal>
#include <sys/mman.h>
#include <sys/resource.h>
#include <cerrno>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

rtfw::RealTimeFramework* p_rtfw = nullptr;

void signalHandler(int) {
    std::cout << "\n[test_app] SIGINT – stopping framework..." << std::endl;
    if (p_rtfw) p_rtfw->stop();
}

static bool setup_memory_locking() {
    struct rlimit rlim{};
    if (getrlimit(RLIMIT_MEMLOCK, &rlim) == 0 && rlim.rlim_cur < rlim.rlim_max) {
        rlim.rlim_cur = rlim.rlim_max;
        setrlimit(RLIMIT_MEMLOCK, &rlim);
    }
    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
        std::cerr << "[test_app] WARNING: mlockall failed (" << strerror(errno)
                  << "). Running without memory locking." << std::endl;
        return false;
    }
    std::cout << "[test_app] mlockall OK – memory locked in RAM." << std::endl;
    return true;
}

int main(int argc, char** argv) {
    setup_memory_locking();

    // ----------------------------------------------------------------
    // Parse arguments
    // ----------------------------------------------------------------
    rtfw::FrameworkConfig config;
    config.mode = rtfw::Mode::LIVE;

    std::vector<std::string> args(argv + 1, argv + argc);
    for (size_t i = 0; i < args.size(); ++i) {
        if (args[i] == "--live") {
            config.mode = rtfw::Mode::LIVE;
        } else if (args[i] == "--record" && i + 1 < args.size()) {
            config.mode = rtfw::Mode::LIVE; // recording is triggered via rtcli
            ++i; // path ignored here; rtcli sends the start_dynamic command
        } else if (args[i] == "--replay" && i + 1 < args.size()) {
            config.mode = rtfw::Mode::LIVE; // replay is triggered via rtcli
            ++i;
        }
    }

    try {
        rtfw::RealTimeFramework framework;
        p_rtfw = &framework;
        std::signal(SIGINT, signalHandler);

        // ----------------------------------------------------------------
        // Blackbox backend setup (record + replay via rtcli)
        // ----------------------------------------------------------------
        config.blackbox.record_backend = std::make_unique<rtfw::blackbox::FileBlackbox>();
        config.blackbox.replay_backend = std::make_unique<rtfw::blackbox::FileBlackbox>();

        // ----------------------------------------------------------------
        // Register RT tasks
        // ----------------------------------------------------------------

        // ── 1000 Hz  Core 5 (dedicated, 2 threads) ──────────────────────
        framework.registerTask(std::make_unique<Gen1k_A>(),     1000, 5);
        framework.registerTask(std::make_unique<Gen1k_B>(),     1000, 5);
        framework.registerTask(std::make_unique<Gen1k_C>(),     1000, 5);
        framework.registerTask(std::make_unique<Gen1k_D>(),     1000, 5);
        framework.registerTask(std::make_unique<Filter1k_A>(),  1000, 5);
        framework.registerTask(std::make_unique<Filter1k_B>(),  1000, 5);
        framework.registerTask(std::make_unique<Filter1k_C>(),  1000, 5);
        framework.registerTask(std::make_unique<Ctrl1k>(),      1000, 5);

        // ── 500 Hz  Core 4 (dedicated, 1 thread) ────────────────────────
        framework.registerTask(std::make_unique<Fuse500_A>(),   500, 4);
        framework.registerTask(std::make_unique<Fuse500_B>(),   500, 4);
        framework.registerTask(std::make_unique<Fuse500_C>(),   500, 4);
        framework.registerTask(std::make_unique<Est500_A>(),    500, 4);
        framework.registerTask(std::make_unique<Est500_B>(),    500, 4);
        framework.registerTask(std::make_unique<Est500_C>(),    500, 4);

        // ── 200 Hz  Core 3 (dedicated, 1 thread) ────────────────────────
        framework.registerTask(std::make_unique<Plan200_A>(),   200, 3);
        framework.registerTask(std::make_unique<Plan200_B>(),   200, 3);
        framework.registerTask(std::make_unique<Plan200_C>(),   200, 3);
        framework.registerTask(std::make_unique<Diag200_A>(),   200, 3);
        framework.registerTask(std::make_unique<Diag200_B>(),   200, 3);

        // ── 100 Hz  Common RT pool ───────────────────────────────────────
        framework.registerTask(std::make_unique<Monitor100_A>(), 100);
        framework.registerTask(std::make_unique<Monitor100_B>(), 100);
        framework.registerTask(std::make_unique<Monitor100_C>(), 100);
        framework.registerTask(std::make_unique<Monitor100_D>(), 100);
        framework.registerTask(std::make_unique<Monitor100_E>(), 100);

        // ── 50 Hz  Common RT pool ────────────────────────────────────────
        framework.registerTask(std::make_unique<Health50_A>(),  50);
        framework.registerTask(std::make_unique<Health50_B>(),  50);
        framework.registerTask(std::make_unique<Health50_C>(),  50);
        framework.registerTask(std::make_unique<Health50_D>(),  50);
        framework.registerTask(std::make_unique<Stress50_A>(),  50);
        framework.registerTask(std::make_unique<Stress50_B>(),  50);
        framework.registerTask(std::make_unique<Stress50_C>(),  50);
        framework.registerTask(std::make_unique<Stress50_D>(),  50);

        // ── 10 Hz  Common RT pool (stateful / checkpoint-able) ───────────
        framework.registerTask(std::make_unique<Stateful10_A>(),       10);
        framework.registerTask(std::make_unique<Stateful10_B>(),       10);
        framework.registerTask(std::make_unique<Stateful10_C>(),       10);
        framework.registerTask(std::make_unique<Stateful10_D>(),       10);
        framework.registerTask(std::make_unique<Stateful10_Summary>(),    10);
        // Replay verification: RT-origin derived keys (non-archived)
        framework.registerTask(std::make_unique<DeriveRT_Pure>(),         10);
        framework.registerTask(std::make_unique<DeriveRT_TickTainted>(),  10);

        // ----------------------------------------------------------------
        // Register Non-RT tasks
        // ----------------------------------------------------------------
        framework.registerNonRtTask(std::make_unique<ParamGen_A>(),  1);
        framework.registerNonRtTask(std::make_unique<ParamGen_B>(),  1);
        framework.registerNonRtTask(std::make_unique<ParamGen_C>(),  1);
        // Replay verification: NonRT-origin derived keys (non-archived)
        framework.registerNonRtTask(std::make_unique<DeriveNonRT_Pure>(),         1);
        framework.registerNonRtTask(std::make_unique<DeriveNonRT_TickTainted>(),  1);

        framework.registerNonRtTask(std::make_unique<StatsLog_A>(),         5);
        framework.registerNonRtTask(std::make_unique<DiagLog_B>(),          5);
        framework.registerNonRtTask(std::make_unique<HealthLog_C>(),        5);
        framework.registerNonRtTask(std::make_unique<PerfLog_D>(),          5);
        framework.registerNonRtTask(std::make_unique<DerivedCheck_Logger>(), 5);

        // ----------------------------------------------------------------
        // Thread pool configuration
        //   - dedicated_core_threads: core_id → num_threads
        //   - Core 5: 2 threads (8 tasks at 1 kHz share two workers)
        //   - Core 4: 1 thread  (6 tasks at 500 Hz)
        //   - Core 3: 1 thread  (5 tasks at 200 Hz)
        //   - Common RT pool: 6 threads (handles 100/50/10 Hz – 18 tasks)
        //   - Non-RT pool: 2 threads
        // ----------------------------------------------------------------
        config.threads.dedicated_core_threads = {{5, 2}, {4, 1}, {3, 1}};
        config.threads.num_common_threads     = 6;
        config.threads.num_non_rt_threads     = 2;

        config.log_level = rtfw::LogLevel::INFO;

        // ----------------------------------------------------------------
        // Start
        // ----------------------------------------------------------------
        std::cout << "[test_app] Initializing – 49 tasks, base 1 kHz" << std::endl;
        framework.initialize(std::move(config));

        std::cout << "[test_app] Running. Use rtcli to control record/replay." << std::endl;
        std::cout << "[test_app]   record start: ./build/tools/rtcli record start /tmp/test_app.rtrec" << std::endl;
        std::cout << "[test_app]   record stop : ./build/tools/rtcli record stop" << std::endl;
        std::cout << "[test_app]   replay start: ./build/tools/rtcli replay start /tmp/test_app.rtrec" << std::endl;
        std::cout << "[test_app]   replay stop : ./build/tools/rtcli replay stop" << std::endl;
        std::cout << "[test_app]   task list   : ./build/tools/rtcli task list" << std::endl;

        framework.start();  // blocks until stop() or SIGINT

        framework.printStats();
        std::cout << "[test_app] Done." << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "[test_app] FATAL: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}
