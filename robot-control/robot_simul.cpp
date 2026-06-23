// ═══════════════════════════════════════════════════════════════════════════
// Wire Manipulator MuJoCo 시뮬레이션 — 진입점(main)
// ═══════════════════════════════════════════════════════════════════════════
//
// 역할: robot_control 과 동일한 상위 스택(Manager/SafetyLayer/GUI)을 띄우되, 실제 CAN
//       드라이버(CanBus0/1) 대신 MujocoEnv 를 붙여 물리 시뮬레이션으로 모터를 대체한다.
//       덕분에 하드웨어 없이 GUI·상태머신·세이프티 로직을 그대로 검증할 수 있다.
//
// 태스크 구성:
//   RT:     Manager(30Hz)      — 실하드웨어와 동일 (GUI 명령·상태머신)
//           SafetyLayer(200Hz) — 실하드웨어와 동일 (한계 검사·클램프)
//   Non-RT: MujocoEnv(200Hz)   — 물리 적분 + GLFW 렌더 (창 관리 때문에 non-RT)
//           WsBridgeTask(30Hz) — 웹 GUI WebSocket 브리지
//           DataLogger(200Hz)  — 상태/명령 로깅

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

// SIGINT 핸들러에서 프레임워크를 정지시키기 위한 전역 포인터
rtfw::RealTimeFramework* p_rtfw = nullptr;
// 단일 인스턴스 보장용 lock 파일 디스크립터 (flock)
int g_lock_fd = -1;

// 중복 실행 방지: lock 파일에 배타적 flock 을 건다. 이미 다른 robot_simul 이 실행
// 중이면(같은 GLFW 창·포트 충돌 방지) 실패로 종료한다.
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

// 실시간 결정성 확보: 프로세스 메모리를 RAM에 고정(mlockall)해 페이지 폴트 지터를 막는다.
// 실패해도 soft RT로 계속 진행한다 (시뮬레이션에서는 영향이 크지 않음).
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

        // RT 태스크 등록 — 실하드웨어와 동일한 Manager/SafetyLayer
        framework.registerTask(std::make_unique<task_pool::Manager>(),     30);
        framework.registerTask(std::make_unique<task_pool::SafetyLayer>(), 200);

        // 시뮬레이션 환경 + Non-RT 태스크 (MujocoEnv는 GLFW 창 관리 때문에 non-RT로 등록)
        framework.registerTask(std::make_unique<task_pool::MujocoEnv>(), 200);
        framework.registerNonRtTask(std::make_unique<task_pool::WsBridgeTask>(), 30);
        framework.registerNonRtTask(std::make_unique<task_pool::DataLogger>(), 200);

        // 프레임워크 설정 — non-RT 스레드 2개(MujocoEnv + 브리지/로거)
        rtfw::FrameworkConfig config;
        config.mode           = rtfw::Mode::LIVE;
        config.realtime_level = RealtimeLevel::SOFT;
        config.threads.num_common_threads     = 4;
        config.threads.dedicated_core_threads = {{6, 1}, {7, 1}};
        config.threads.num_non_rt_threads     = 2;
        config.parameter_file_path = "config/robotnl.yaml";
        config.log_level = rtfw::LogLevel::INFO;

        framework.initialize(std::move(config));
        framework.start();  // 블로킹 — stop() 호출 시까지 실행

    } catch (const std::exception& e) {
        std::cerr << "FATAL: " << e.what() << "\n";
        return 1;
    }
    return 0;
}
