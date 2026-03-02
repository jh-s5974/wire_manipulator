# RTFW 개발자 매뉴얼

**버전**: 1.2 | **최종 수정**: 2026-02-21

---

## 목차

1. [아키텍처 개요](#1-아키텍처-개요)
2. [핵심 개념](#2-핵심-개념)
3. [새 태스크 작성하기](#3-새-태스크-작성하기)
4. [프레임워크 API 레퍼런스](#4-프레임워크-api-레퍼런스)
5. [데이터 IO API](#5-데이터-io-api)
6. [파라미터 시스템](#6-파라미터-시스템)
7. [블랙박스: 녹화 / 재생 / 트레이스](#7-블랙박스-녹화--재생--트레이스)
8. [rtcli 완전 참조](#8-rtcli-완전-참조)
9. [공유 메모리 레이아웃](#9-공유-메모리-레이아웃)
10. [외부 클라이언트 API (rtfw-connect)](#10-외부-클라이언트-api-rtfw-connect)
11. [오버런 처리](#11-오버런-처리)
12. [트러블슈팅](#12-트러블슈팅)

---

## 1. 아키텍처 개요

```
┌──────────────────────────────────────────────────────────────────────┐
│  애플리케이션 (예: poc_app)                                           │
│  registerTask(...)  registerNonRtTask(...)  initialize()  start()    │
└─────────────────────────────┬────────────────────────────────────────┘
                              │
                              ▼
┌──────────────────────────────────────────────────────────────────────┐
│  RealTimeFramework (rtfw/src/rt_framework.cpp)                       │
│                                                                      │
│  ① 의존성 분석 (DAG)   ② SHM 레이아웃 생성   ③ 타임라인 생성        │
│  ④ 스레드 풀 시작       ⑤ 스케줄러 루프      ⑥ 액션 처리            │
│                                                                      │
│  Timeline(1000Hz) → Timeline(200Hz) → Timeline(30Hz) → ...          │
│  CommonRT Pool(4) · DedicatedCore(7,6,5) · NonRT Pool(2)            │
└──────────┬───────────────────────────────────────────────────────────┘
           │  POSIX SHM  /rt_framework_shm
           ▼
┌──────────────────────────────────────────────────────────────────────┐
│  공유 메모리 (rtfw-common/include/rtfw_common/shm_layout.h)          │
│  SharedMemoryHeader · DataBlockDescriptor[] · TaskStats[]           │
│  TimelineStats[] · PoolStats[] · TaskGraphNodeInfo[] · 파라미터     │
└──────────┬───────────────────────────────────────────────────────────┘
           │
    ┌──────┴──────────────────────┐
    ▼                             ▼
┌──────────────┐        ┌──────────────────────────────┐
│  rtcli      │        │  rtmonitor / rtviewer         │
│  (tools/)    │        │  (rtfw-connect API 사용)      │
│  scheduler   │        │  SharedMemoryQuerier          │
│  record/     │        │  SharedMemoryController       │
│  replay 등   │        └──────────────────────────────┘
└──────────────┘
```

### 초기화 순서

`RealTimeFramework::initialize()` 내부 실행 순서:

1. `collectAndAnalyzeTasks()` — 의존성 그래프 분석, 태스크 ID 할당
2. `wireTaskProxies()` — DataWriter/DataReader를 SHM 포인터에 연결
3. `prepareLayout()` → `populateSHM()` — SHM 할당 및 메타데이터 기록
4. 타임라인/스레드 풀 생성
5. 로거(`ShmRingbufferSink`) 초기화 (**SHM 할당 이후**에 실행됨)

> ⚠️ **SHM 초기화 순서를 변경하지 마세요.** 로거는 반드시 SHM 생성 이후 초기화되어야 합니다.

---

## 2. 핵심 개념

### 2.1 Single-Writer 원칙

각 데이터 키(예: `"sensors.high_freq"`)는 정확히 **하나의 `DataWriter`**만 가질 수 있습니다.

```cpp
// ✅ 올바름 — SensorProducer만 이 키를 씁니다
DataWriter<HighFreqSensors> sensors_writer_{"sensors.high_freq"};

// ❌ 금지 — 동일 키에 두 번째 라이터 추가 불가
DataWriter<HighFreqSensors> sensors_writer2_{"sensors.high_freq"}; // 오류!
```

### 2.2 강한 의존성 vs 약한 의존성

| 타입 | 동작 | 사용 시점 |
|------|------|-----------|
| `DependencyType::Strong` | 라이터 태스크 완료까지 대기 (동기) | 가장 최신 데이터가 반드시 필요할 때 |
| `DependencyType::Weak` | 블록 없이 가용한 최신 데이터 사용 | 이전 틱의 데이터도 허용되는 경우 |

```cpp
// 강한 의존성 (기본값)
DataReader<HighFreqSensors> sensors_reader_{"sensors.high_freq"};

// 약한 의존성
DataReader<RobotState> state_reader_{"robot.state.estimated", DependencyType::Weak};
```

### 2.3 N중 버퍼링

버퍼 수 = **#타임라인 수 + #NonRT그룹 수 + slack(1)**

`poc_app` 예시:
- 타임라인: 1000Hz, 200Hz, 30Hz, 10Hz → 4개
- NonRT 그룹: 1개
- Slack: 1개
- **총 6개 버퍼**

각 타임라인은 틱 시작 시 `hold()` (ref_count++), 틱 종료 시 `unhold()` (ref_count--). ref_count > 0인 버퍼는 덮어쓰기 금지.

### 2.4 Task 구현 방식 선택

태스크는 상태 관리 필요 여부에 따라 두 가지 방식으로 구현합니다.

#### ITask - 상태가 필요 없는 경우

단순 데이터 변환, stateless 연산 등에 사용:

```cpp
class SimpleTask : public rt::ITask {
    const char* getName() const override { return "SimpleTask"; }
    
    void execute(void* state_ptr) override {
        auto input = reader_.read();
        writer_.write({input.x * 2.0});
    }
    
    DataReader<InputData> reader_{"input.key"};
    DataWriter<OutputData> writer_{"output.key"};
};
```

#### Task<StateT> - 상태 관리가 필요한 경우

틱 간 유지되어야 하는 상태(counter, 필터 상태, 이전 값 등)가 있을 때:

```cpp
struct MyState {
    uint64_t counter = 0;
    double last_value = 0.0;
    // ⚠️ 반드시 trivially copyable (POD) 이어야 합니다
};

class MyTask : public rt::Task<MyState> {
    const char* getName() const override { return "MyTask"; }
    
    void execute(MyState& state) override {
        state.counter++;
        state.last_value = reader_.read().value;
        writer_.write({state.counter, state.last_value});
    }
    
    DataReader<InputData> reader_{"input.key"};
    DataWriter<OutputData> writer_{"output.key"};
};
```

**Task<StateT>의 장점:**
- 프레임워크가 `_state_buffer`에 상태를 저장합니다
- 체크포인트 저장 시 `_state_buffer` 전체가 특수 로그 엔트리(`CHECKPOINT_KEY_HASH`)로 기록됩니다
- 재생 시 체크포인트에서 상태가 복원됩니다

**주의:** `Task<void>`는 컴파일 에러가 발생합니다. 상태가 필요 없다면 `ITask`를 사용하세요.

---

## 3. 새 태스크 작성하기

### 단계 1: 데이터 타입 정의

`biped_data_types.h` 또는 별도 헤더에 추가:

```cpp
struct MyData {
    double x, y, z;
    bool valid;
};
```

### 단계 2: 태스크 클래스 구현

**옵션 A: 상태가 필요 없는 경우 (ITask)**

```cpp
#include <rtfw/task.h>

class MyTask : public rt::ITask {
public:
    const char* getName() const override { return "MyTask"; }

    void execute(void* /*state_ptr*/) override {
        // 리더에 새 데이터가 있으면 처리
        sensor_reader_.on_update([this](const HighFreqSensors& s) {
            MyData out;
            out.x = s.imu_accel[0];
            out.valid = true;
            my_writer_.write(out);
        });
    }

private:
    DataWriter<MyData>         my_writer_{"my.output.key"};
    DataReader<HighFreqSensors> sensor_reader_{"sensors.high_freq"};
    DataReader<RobotState>      state_reader_{"robot.state.estimated",
                                               DependencyType::Weak};
};
```

**옵션 B: 상태 관리가 필요한 경우 (Task<StateT>)**

```cpp
struct MyState {
    uint64_t counter = 0;
    double filtered_value = 0.0;
    // ⚠️ POD 타입만 가능
};

class MyStatefulTask : public rt::Task<MyState> {
public:
    const char* getName() const override { return "MyStatefulTask"; }

    void execute(MyState& state) override {
        auto input = sensor_reader_.read();
        state.counter++;
        state.filtered_value = 0.9 * state.filtered_value + 0.1 * input.imu_accel[0];
        
        MyData out;
        out.x = state.filtered_value;
        out.valid = true;
        my_writer_.write(out);
    }

private:
    DataWriter<MyData>         my_writer_{"my.output.key"};
    DataReader<HighFreqSensors> sensor_reader_{"sensors.high_freq"};
};
```

### 단계 3: 태스크 등록

`main.cpp`에서:

```cpp
// RT 태스크 (주파수[Hz], CPU 어피니티 코어 번호)
framework.registerTask(std::make_unique<MyTask>(), 100, 5);

// NonRT 태스크 (CPU 어피니티 없음)
framework.registerNonRtTask(std::make_unique<MyTask>(), 10);
```

### 새 태스크 체크리스트

- [ ] 데이터 타입이 `biped_data_types.h` (또는 해당 앱의 타입 헤더)에 정의되었는가
- [ ] `getName()`이 고유한 이름을 반환하는가
- [ ] 상태 관리 필요 여부를 결정했는가 (필요시 `Task<StateT>`, 불필요시 `ITask`)
- [ ] `Task<StateT>` 사용 시 `StateT`가 trivially copyable (POD)인가
- [ ] 모든 `DataWriter`/`DataReader`가 멤버 변수로 선언되었는가
- [ ] 동일 키에 두 번째 `DataWriter`가 없는가
- [ ] `main.cpp`에서 `registerTask()` 또는 `registerNonRtTask()`로 등록했는가

---

## 4. 프레임워크 API 레퍼런스

### `FrameworkConfig`

```cpp
rtfw::FrameworkConfig config;

// 실행 모드
config.mode = rtfw::Mode::LIVE;        // 기본: 실시간 실행
config.mode = rtfw::Mode::RECORDING;   // 녹화하며 실행
config.mode = rtfw::Mode::SIMULATION;  // 재생 전용 (외부 입력 없음)

// 실시간 레벨
config.realtime_level = rtfw::RealtimeLevel::HARD;  // SCHED_FIFO
config.realtime_level = rtfw::RealtimeLevel::SOFT;  // 일반 스레드

// 스레드 풀
config.threads.num_common_threads = 4;                  // 공용 RT 스레드
config.threads.num_non_rt_threads = 2;                  // NonRT 스레드
config.threads.dedicated_core_threads = {{7,1},{6,1},{5,1}}; // 코어 → 스레드 수

// 블랙박스
config.blackbox.record_backend = std::make_unique<rtfw::blackbox::FileBlackbox>();
config.blackbox.replay_backend = std::make_unique<rtfw::blackbox::FileBlackbox>();

// 기타
config.parameter_file_path = "param_test.yaml";
config.log_level = rtfw::LogLevel::DEBUG;
```

### `RealTimeFramework` 주요 메서드

```cpp
rtfw::RealTimeFramework framework;

// 태스크 등록 (initialize() 이전에 호출)
framework.registerTask(std::make_unique<MyTask>(), 주파수_Hz, CPU코어=-1);
framework.registerNonRtTask(std::make_unique<MyNonRtTask>(), 주파수_Hz);

// 초기화 및 실행
framework.initialize(std::move(config));
framework.start();      // 블록
framework.stop();       // 명시적 중지
framework.printStats(); // 종료 후 통계 출력

// 런타임 제어 (in-process에서)
framework.startRecord("/tmp/out.rtrec");
framework.stopRecord();
framework.startReplay("/tmp/out.rtrec");
framework.stopReplay();
```

### 실행 모드 요약

| 모드 | 동작 |
|------|------|
| `LIVE` | 일반 실시간 실행 |
| `RECORDING` | 실행하면서 데이터 파일에 기록 |
| `SIMULATION` | 파일에서 데이터를 읽어 재생 (외부 센서 입력 없음) |
| `TRACE` | replay 결과 전체 데이터 기록 |

---

## 5. 데이터 IO API

### DataWriter

```cpp
template<typename T>
class DataWriter : public IDataWriter {
    // 생성자 (키 이름, 아카이빙 여부)
    DataWriter(std::string_view key,
               ArchiveOption = ArchiveOption::Disable);

    void write(const T& data);      // 데이터 쓰기 (RT 태스크: stale write 보호)
    bool hadStaleWrite() const;     // 이번 틱에 stale write 발생 여부
    void clearStaleWriteFlag();
};
```

> **Stale Write**: 프레임워크가 이미 틱을 진행했는데 이전 틱의 태스크가 늦게 쓰려 할 때 발생. 해당 write는 무시됩니다.

### DataReader

```cpp
template<typename T>
class DataReader : public IDataReader {
    DataReader(std::string_view key,
               DependencyType dep = DependencyType::Strong);

    const T& read() const;           // 현재 보유 버퍼에서 읽기
    const T* operator->() const;     // 포인터 접근

    bool check_update(bool consume = true);  // 새 데이터 있는지 확인

    // 콜백 스타일 (권장)
    template<typename Fn>
    void on_update(Fn&& fn);         // 새 데이터가 있을 때만 fn(data) 호출

    explicit operator bool() const;  // 데이터 사용 가능 여부
};
```

### 사용 패턴

```cpp
void execute(void*) override {
    // 패턴 1: on_update (권장)
    sensor_reader_.on_update([this](const HighFreqSensors& s) {
        // 새 데이터가 있을 때만 실행
        process(s);
    });

    // 패턴 2: check_update + read
    if (state_reader_.check_update()) {
        auto& state = state_reader_.read();
        use(state);
    }

    // 패턴 3: 항상 최신 데이터 읽기 (Weak 의존성 리더)
    auto& params = param_reader_.read();
}
```

---

## 6. 파라미터 시스템

### YAML 파일 형식

```yaml
# param_test.yaml
gait:
  step_length: 0.6
  step_height: 0.05
robot:
  max_speed: 1.0
```

### 파라미터 읽기 (태스크 내)

```cpp
void execute(void*) override {
    // 파라미터 리더 (DataReader와 유사)
    param_reader_.on_update([this](const GaitParameters& p) {
        current_step_length_ = p.new_step_length;
    });
}
```

### 런타임 파라미터 변경

```bash
# CLI로 변경 (스켈레톤 — 향후 구현 예정)
rtcli parameters set gait.step_length 0.8
rtcli parameters get gait.step_length
rtcli parameters reload  # YAML 파일 다시 로드
```

프레임워크는 `FrameworkAction::RELOAD_PARAMETERS` 액션을 받아 파라미터를 재로드합니다.

---

## 7. 블랙박스: 녹화 / 재생 / 트레이스

### 파일 포맷 (`.rtrec`)

```
FileHeader (36 bytes)
  magic[4]     = 'R','T','F','A'
  format_version = 0x0100
  last_tick, base_frequency
  start_timestamp_ns, metadata_offset

LogEntryHeader (36 bytes) × N
  start_tick, end_tick
  key_hash, type_hash
  data_size
  [data_size bytes of payload]

MetadataHeader ('RTMD')
  KeyMappingEntry[] (key_hash → key string)
```

특수 엔트리:
- `key_hash == CHECKPOINT_KEY_HASH (0xFFFFFFFFFFFFFFFF)` → 상태 스냅샷 (Task 상태 전체)

### 녹화 설정

```cpp
// 방법 1: 앱 시작 시 RECORDING 모드
config.mode = rtfw::Mode::RECORDING;
config.blackbox.record_backend = std::make_unique<rtfw::blackbox::FileBlackbox>();

// 방법 2: 런타임 동적 녹화 (LIVE 모드에서)
framework.startRecord("/tmp/output.rtrec");
// ... 실행 중 ...
framework.stopRecord();
```

### 체크포인트 (동적 녹화 시)

`startRecord()` 호출 시점에 `_state_buffer`(모든 Task 상태)가 파일 앞에 체크포인트로 기록됩니다. 이를 통해 재생 시 정확한 상태에서 복원 가능합니다.

```
녹화 파일 구조 (동적 시작 시):
  [CHECKPOINT 엔트리] ← 녹화 시작 시점의 Task 상태
  [일반 LogEntry들]  ← 녹화 기간 동안의 데이터
  [MetadataHeader]
```

### 재생

```cpp
// SIMULATION 모드 설정
config.mode = rtfw::Mode::SIMULATION;
config.blackbox.replay_backend = std::make_unique<rtfw::blackbox::FileBlackbox>();

// 또는 런타임 동적 재생
framework.startReplay("/tmp/output.rtrec");
```

재생 시 동작:
1. 체크포인트가 있으면 Task 상태 복원
2. 각 틱마다 `CacheSlot`을 통해 녹화된 데이터 주입
3. `DataWriter.commit()` 시 injection_proxy 경로 사용

### 트레이스 모드

재생 파일을 실행하면서 모든 데이터를 기록합니다.

```cpp
framework.startTrace("/tmp/replay.rtrec", "/tmp/trace_output.rtrec");
```

---

## 8. rtcli 완전 참조

### 아키텍처

```
사용자 명령 → CommandDispatcher → CommandHandler → SharedMemoryController
                                                  → SharedMemoryQuerier
                                                  → POSIX SHM (/rt_framework_shm)
                                                  → RealTimeFramework
```

### 스케줄러 명령

```bash
rtcli scheduler pause      # FrameworkAction::PAUSE_TICK 전송
rtcli scheduler play       # FrameworkAction::PLAY_TICK 전송
rtcli scheduler play_one   # FrameworkAction::PLAY_ONE_TICK 전송
```

**응답 시간**: CLI < 1ms, Framework 반응 ~5ms

### 녹화 명령

```bash
rtcli record start /path/to/output.rtrec
rtcli record stop

# 파일 검증 (SHM 불필요)
rtcli check_record /path/to/output.rtrec
```

`check_record` 출력 예시:
```
File: /tmp/test_record.rtrec
Total entries: 209704
Keys found: 5
Corrupted entries: 0
```

### 재생 명령

```bash
rtcli replay start /path/to/input.rtrec
rtcli replay stop
```

### 태스크 제어 명령

```bash
rtcli task list              # 모든 태스크 ID, 이름, 주파수 출력
rtcli task info <이름>        # 태스크 상세 정보
rtcli task enable <task_id>   # FrameworkAction::SET_TASK_ENABLED 전송
rtcli task disable <task_id>
```

### 새 rtcli 명령 추가하기

1. `CommandHandler`를 상속하는 클래스 구현:

```cpp
// tools/rtcli/include/my_commands.h
class MyCommandHandler : public CommandHandler {
    int execute(const std::vector<std::string>& args,
                rtfw::connect::SharedMemoryConnector& connector) override;
    std::string name() const override { return "mycommand"; }
    std::string help() const override { return "설명"; }
};
```

2. `tools/rtcli/src/main.cpp`에 등록:

```cpp
dispatcher.register_handler(std::make_unique<MyCommandHandler>());
```

3. CMakeLists.txt에 소스 파일 추가.

---

## 9. 공유 메모리 레이아웃

SHM 이름: `/rt_framework_shm`

### SharedMemoryHeader 주요 필드

```cpp
struct SharedMemoryHeader {
    atomic<ShmState> shm_state;          // UNINITIALIZED/RUNNING/SHUTTING_DOWN
    uint32_t base_frequency;              // 최고 주파수 (예: 1000)
    atomic<uint64_t> framework_tick_count;
    atomic<uint64_t> current_tick_start_time_ns;
    atomic<LogLevel> shared_log_level;
    SharedLogBuffer log_buffer;           // SHM 링버퍼 로그

    atomic<FrameworkAction> requested_action;  // ← rtcli가 여기에 씀
    atomic<bool> recording_active;
    atomic<bool> replaying_active;
    char target_filename[256];            // 녹화 파일 경로
    char replay_target_filename[256];     // 재생 파일 경로
    atomic<uint32_t> target_task_id;      // SET_TASK_ENABLED용

    // + DataBlockDescriptor[], TaskStats[], TimelineStats[], PoolStats[]
    // + TaskGraphNodeInfo[], GraphEdge[], DataFlowInfo[]
    // + ParameterBlockDescriptor
};
```

### TaskStats 구조

```cpp
struct TaskStats {
    char task_name[64];
    atomic<uint64_t> last_pushed_to_queue_offset_ns;
    atomic<uint64_t> last_start_offset_ns;
    atomic<uint64_t> last_completion_offset_ns;
    atomic<long long> total_exec_time_ns, max_exec_time_ns;
    atomic<long long> exec_count;
    atomic<bool> is_busy, has_overrun;
    atomic<long long> stale_write_count;
    atomic<long long> overrun_recovery_count;
};
```

### FrameworkAction 열거형

| 값 | 의미 |
|----|------|
| `NONE (0)` | 없음 |
| `START_RECORD (1)` | 녹화 시작 |
| `STOP_RECORD (2)` | 녹화 중지 |
| `RELOAD_PARAMETERS (3)` | 파라미터 재로드 |
| `PAUSE_TICK (4)` | 스케줄러 일시정지 |
| `PLAY_TICK (5)` | 스케줄러 재개 |
| `START_REPLAY (6)` | 재생 시작 |
| `STOP_REPLAY (7)` | 재생 중지 |
| `START_TRACE (8)` | 트레이스 시작 |
| `STOP_TRACE (9)` | 트레이스 중지 |
| `PLAY_ONE_TICK (10)` | 단일 틱 실행 |
| `SET_TASK_ENABLED (11)` | 태스크 활성/비활성화 |

---

## 10. 외부 클라이언트 API (rtfw-connect)

외부 도구(rtmonitor, rtcli 등)에서 SHM에 접근할 때 사용합니다.

### 연결

```cpp
#include <rtfw_connect/shm_connector.h>
#include <rtfw_connect/shm_querier.h>
#include <rtfw_connect/shm_controller.h>

rtfw::connect::SharedMemoryConnector connector;
void* base = connector.connect("/rt_framework_shm");

rtfw::connect::SharedMemoryQuerier querier(base);
rtfw::connect::SharedMemoryController controller(base);
```

### SharedMemoryQuerier (읽기 전용)

```cpp
// 프레임워크 상태
auto* header = querier.getHeader();
uint64_t tick = header->framework_tick_count.load();

// 태스크 통계
const TaskStats* stats = querier.getTaskStatsArray();

// 데이터 블록 읽기
auto reader = querier.getDataReader<RobotState>("robot.state.estimated");
const RobotState& state = reader.read();

// 태스크 그래프
auto nodes = querier.getGraphNodes();
auto edges = querier.getGraphEdges();

// 파라미터 읽기
auto val = querier.getParameterValue<double>("gait.step_length");
```

### SharedMemoryController (제어)

```cpp
controller.pauseTick();
controller.playTick();
controller.playOneTick();
controller.startRecord("/tmp/out.rtrec");
controller.stopRecord();
controller.setTaskEnabled(task_id, false);
controller.setLogLevel(rtfw::LogLevel::DEBUG);

// 완료 대기 (기본 1000ms 타임아웃)
controller.waitForActionComplete(500);
```

---

## 11. 오버런 처리

### 오버런 감지 메커니즘

```
tick_start():
  if (is_busy) {
    has_overrun = true   // Self-overrun: 이전 틱이 아직 실행 중
    skip this tick
  }

tick_end():
  was_ready && !was_executed → DeadlineMissSelf
  !was_ready                 → DeadlineMissUpstream

DataWriter::write() (RT 태스크만):
  if (stale write 감지) { log WARN; 무시 }
```

### ExecState 열거형

| 상태 | 의미 |
|------|------|
| `Executed` | 정상 실행 완료 |
| `DeadlineMissSelf` | 준비됐지만 실행 못함 (self-overrun) |
| `DeadlineMissUpstream` | 업스트림 의존성 미충족 |
| `NotScheduled` | 이번 틱 스케줄 없음 |

### 오버런 복구 (ITask 오버라이드)

```cpp
bool onOverrun() override {
    // true 반환: 복구 가능, 스케줄에 유지
    // false 반환: 영구 비활성화 (setEnabled(false))
    return true; // 기본값: false
}
```

### 알려진 미해결 이슈

| 이슈 | 상태 |
|------|------|
| Upstream Overrun 감지 (라이터 > 버퍼 슬롯) | ❌ 미구현 |
| `DataWriter::write()` stale write 시 호출자 알림 | ❌ void 반환 |
| `is_busy` 플래그 load-enqueue 사이 레이스 컨디션 | ⚠️ 잠재적 |

---

## 12. 트러블슈팅

### SHM 연결 실패

**증상**: `rtcli` 실행 시 `Failed to connect to SHM` 오류

**원인 및 해결**:
```bash
# 프레임워크가 실행 중인지 확인
ps aux | grep poc_app

# SHM 존재 확인
ls -la /dev/shm/rt_framework_shm

# 이전 SHM 잔재 제거 (프레임워크 재시작 후에도 남아있는 경우)
rm /dev/shm/rt_framework_shm
```

### 빈 녹화 파일

**증상**: `check_record` 시 엔트리 0개

**원인**: 녹화가 시작되기 전에 `stop`을 호출했거나, `ArchiveOption::Enable`로 설정된 DataWriter가 없음

**해결**:
```cpp
// DataWriter 아카이빙 활성화
DataWriter<MyData> writer_{"my.key", ArchiveOption::Enable};
```

### 재생 크래시 (`malloc(): unaligned tcache chunk detected`)

**위치**: `FileBlackbox::onTick()` 내부

**임시 해결**: 재생 전 새 파일로 녹화 후 시도. 조사 중인 이슈.

### 태스크가 실행되지 않음

**확인 사항**:
1. `setup()`에서 모든 의존성이 `r.add_dependency()`에 선언되었는가
2. 업스트림 라이터 태스크가 등록되어 있는가
3. `setEnabled(false)`가 호출되지 않았는가
4. `has_overrun` 플래그 확인: `rtcli task info <이름>`

### 로그가 보이지 않음

```cpp
// config에서 로그 레벨 확인
config.log_level = rtfw::LogLevel::DEBUG;  // TRACE/DEBUG/INFO/WARN/ERROR

// 런타임 변경
controller.setLogLevel(rtfw::LogLevel::DEBUG);
```

### 성능 최적화 팁

```bash
# CPU 아이들 비활성화 (레이턴시 감소)
sudo ./disable_cpuidle.sh

# 메모리 페이지 잠금 (poc_app main.cpp 참고)
mlockall(MCL_CURRENT | MCL_FUTURE);  # main() 최상단에서 호출

# 코어 격리 확인
sudo ./set_active_cores.sh

# 실시간 스케줄링 권한
sudo setcap 'cap_sys_nice+ep' ./build/apps/poc_app/poc_app
```

---

## 부록: 성능 지표 (poc_app 기준)

| 항목 | 수치 |
|------|------|
| `rtcli` 시작 시간 | < 0.5ms |
| `rtcli` 명령 레이턴시 | < 1ms |
| 프레임워크 액션 반응 | ~5ms |
| 녹화 처리량 | ~202 KB/s |
| `check_record` 검증 속도 | 85ms / 41MB |
| 스케줄러 pause/play 신뢰성 | 11/11 테스트 통과 |

## 부록: 주요 파일 위치 빠른 참조

| 파일 | 역할 |
|------|------|
| `rtfw/include/rtfw/rt_framework.h` | 메인 API — `RealTimeFramework`, `FrameworkConfig` |
| `rtfw/include/rtfw/task.h` | `ITask`, `Task`, `DataReader`, `DataWriter` |
| `rtfw-common/include/rtfw_common/shm_layout.h` | SHM 레이아웃, `FrameworkAction`, `TaskStats` |
| `rtfw-common/include/rtfw_common/log_format.h` | 파일 포맷, `CHECKPOINT_KEY_HASH` |
| `rtfw-common/include/rtfw_common/blackbox.h` | `IBlackbox`, `CacheSlot` |
| `rtfw-connect/include/rtfw_connect/shm_querier.h` | 외부 읽기 API |
| `rtfw-connect/include/rtfw_connect/shm_controller.h` | 외부 제어 API |
| `apps/poc_app/biped_task.h` | 8개 태스크 구현 예제 |
| `apps/poc_app/main.cpp` | 프레임워크 초기화 패턴 |
| `tools/rtcli/src/main.cpp` | rtcli 진입점 |
