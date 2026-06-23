// ═══════════════════════════════════════════════════════════════════════════
// Wire Manipulator 실하드웨어 제어 — 진입점(main)
// ═══════════════════════════════════════════════════════════════════════════
//
// 역할: RT 프레임워크를 띄우고 태스크를 등록·실행한다. 모든 제어 로직은 각 태스크
//       (Manager/SafetyLayer/CanBus0/CanBus1)에 있고, 이 파일은 배선과 부팅만 담당.
//
// 태스크 구성 (데이터 흐름: GUI → Manager → SafetyLayer → CanBusN → 모터):
//   RT:     Manager(30Hz)      — GUI 명령 수신·상태머신, 조인트 명령 발행
//           SafetyLayer(200Hz) — 한계 검사·클램프, MANUAL/LOCK/RESTORE 관리
//           CanBus0(60Hz, core6) — joint0~2 (RobStride 0x01/0x02, MyActuator 0x03)
//           CanBus1(60Hz, core7) — joint3/4  (MyActuator 0x04~0x07, 와이어 구동)
//   Non-RT: WsBridgeTask(30Hz) — 웹 GUI WebSocket 브리지
//           DataLogger(60Hz)   — 상태/명령 로깅

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

// SIGINT(Ctrl-C) 핸들러에서 프레임워크를 정지시키기 위한 전역 포인터
rtfw::RealTimeFramework* p_rtfw = nullptr;
void signalHandler(int) {
    if (p_rtfw) p_rtfw->stop();
}

// 실시간 결정성 확보: 프로세스 메모리를 RAM에 고정(mlockall)해 페이지 폴트로 인한
// 지터를 막는다. MEMLOCK 한계를 최대로 올리고, 실패해도 soft RT로 계속 진행한다.
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

        // RT 태스크 등록 — (태스크, 주파수[Hz], [전용코어]). 두 CAN 버스는 코어 6/7에 핀.
        framework.registerTask(std::make_unique<task_pool::Manager>(),     30);
        framework.registerTask(std::make_unique<task_pool::SafetyLayer>(), 200);
        framework.registerTask(std::make_unique<task_pool::CanBus0>(),     60, 6);
        framework.registerTask(std::make_unique<task_pool::CanBus1>(),     60, 7);

        // Non-RT 태스크 등록 — GUI 브리지와 로거(타이밍 민감하지 않음)
        framework.registerNonRtTask(std::make_unique<task_pool::WsBridgeTask>(), 30);
        framework.registerNonRtTask(std::make_unique<task_pool::DataLogger>(), 60);

        // 프레임워크 설정 — 스레드 풀, 전용 코어(6,7 각 1개), 파라미터 파일 경로
        rtfw::FrameworkConfig config;
        config.mode           = rtfw::Mode::LIVE;
        config.realtime_level = RealtimeLevel::SOFT;
        config.threads.num_common_threads      = 4;
        config.threads.dedicated_core_threads  = {{6, 1}, {7, 1}};
        config.threads.num_non_rt_threads      = 1;
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
