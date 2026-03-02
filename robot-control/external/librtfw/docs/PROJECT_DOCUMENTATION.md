# RTFW (Real-Time Framework) 프로젝트 상세 문서

**작성 일자**: 2026년 2월  
**목적**: 팀 협업을 위한 프로젝트 구조 및 기능 설명서

---

## 목차

1. [프로젝트 개요](#프로젝트-개요)
2. [시스템 아키텍처](#시스템-아키텍처)
3. [핵심 개념](#핵심-개념)
4. [디렉토리 구조](#디렉토리-구조)
5. [주요 컴포넌트](#주요-컴포넌트)
6. [개발 가이드](#개발-가이드)
7. [빌드 및 실행](#빌드-및-실행)
8. [협업 체크리스트](#협업-체크리스트)

---

## 프로젝트 개요

### RTFW란?

**RTFW(Real-Time Framework)**는 복잡한 데이터 흐름을 가진 고성능 실시간 제어 시스템(예: 휴머노이드 로봇)을 위해 설계된 C++ 프레임워크입니다.

### 핵심 가치

| 특징 | 설명 |
|------|------|
| **자동화된 실행 순서** | 데이터 의존성 분석으로 최적의 태스크 실행 순서 자동 결정 |
| **단방향 데이터 흐름** | 단일 작성자 원칙으로 DAG(비순환 그래프) 보장 |
| **결정론적 실행** | 런타임 분석 없이 미리 정적으로 의존성 결정 → 예측 가능 |
| **다중 주파수 안전성** | N-way 버퍼링으로 다양한 빈도의 태스크 안전한 동기화 |
| **완벽한 재현성** | 외부 입력만 기록하면 전체 시스템 완벽히 재현 가능 |
| **높은 관찰 가능성** | 공유 메모리를 통해 실시간 시각화 및 제어 가능 |

---

## 시스템 아키텍처

### 전체 구조도

```
┌─────────────────────────────────────────────────────┐
│                   Application Code                  │
│              (Tasks, Data Types, Logic)             │
└────────────────────┬────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────┐
│              RTFW Core Engine (rtfw)                │
│  ┌──────────────────────────────────────────────┐   │
│  │ Scheduler & Timeline Management             │   │
│  │ • Task Registration & Analysis              │   │
│  │ • Dependency Resolution                     │   │
│  │ • Multi-frequency Timeline Isolation        │   │
│  └──────────────────────────────────────────────┘   │
│  ┌──────────────────────────────────────────────┐   │
│  │ Shared Memory System                        │   │
│  │ • N-way Buffering for Data Safety           │   │
│  │ • Reference Counting                        │   │
│  │ • Lock-free Data Exchange                   │   │
│  └──────────────────────────────────────────────┘   │
│  ┌──────────────────────────────────────────────┐   │
│  │ Thread Management                           │   │
│  │ • CPU Affinity & Pinning                    │   │
│  │ • SCHED_FIFO for Real-Time Tasks            │   │
│  │ • Non-RT Task Isolation                     │   │
│  └──────────────────────────────────────────────┘   │
└────────────────────┬────────────────────────────────┘
                     │
         ┌───────────┼───────────┐
         ▼           ▼           ▼
    ┌────────┐ ┌────────┐ ┌──────────┐
    │  SHM   │ │Logging │ │Recording │
    │ Layout │ │(spdlog)│ │ (Blackbox)
    └────────┘ └────────┘ └──────────┘
         │           │           │
         └───────────┼───────────┘
                     │
                     ▼
    ┌────────────────────────────────┐
    │   External Tools (rtfs-connect) │
    │ • SharedMemoryQuerier           │
    │ • Monitor GUI (rtmonitor)       │
    │ • Offline Viewer (rtviewer)     │
    └────────────────────────────────┘
```

### 실행 흐름

```
┌─────────────────────────────────────────────────────┐
│  Task Setup & Registration Phase                    │
│  • Each task defines its data dependencies          │
│  • Framework analyzes dependency graph              │
│  • Validates: Single-writer principle, No cycles    │
└─────────────────────────────────────────────────────┘
                     ▼
┌─────────────────────────────────────────────────────┐
│  Shared Memory Initialization                       │
│  • Allocate buffers for all data keys               │
│  • Set up N-way buffering based on timelines        │
│  • Initialize ringbuffer for logging                │
└─────────────────────────────────────────────────────┘
                     ▼
┌─────────────────────────────────────────────────────┐
│  Timeline Creation & Thread Launch                  │
│  • One timeline per frequency                       │
│  • Each timeline owns execution thread(s)           │
│  • Non-RT tasks run in separate thread pool         │
└─────────────────────────────────────────────────────┘
                     ▼
┌─────────────────────────────────────────────────────┐
│  Main Execution Loop (Per Timeline)                 │
│                                                     │
│  For each tick:                                     │
│  1. Capture: Increment ref-count for read buffers   │
│  2. Execute: Run dependent tasks in order           │
│  3. Release: Decrement ref-count, commit new data   │
│  4. Sleep until next tick                           │
└─────────────────────────────────────────────────────┘
```

---

## 핵심 개념

### 1. 데이터 흐름이 실행을 지배한다

#### Strong 의존성 (Synchronous)
- **정의**: 같은 주기 내에서 순차 실행을 강제
- **동작**: DataReader는 DataWriter의 완료를 기다림
- **용도**: 동일 주파수/타임라인의 밀접한 데이터 연결
- **예시**: 센서 → 상태 추정 → 제어 → 모터 출력

```cpp
// setup()에서 Strong 의존성 선언 (기본값)
void setup(TaskRegistry& r) override {
    r.add_dependency(sensor_data_);  // Strong (DependencyType::Strong이 기본)
}
```

#### Weak 의존성 (Asynchronous)
- **정의**: 실행 순서를 강제하지 않고 현재 사용 가능한 최신 데이터 사용
- **선언**: TaskRegistry에서 명시적으로 DependencyType::Weak 지정 필요
- **용도**: 서로 다른 주파수, RT-NonRT 간 데이터 교환
- **이점**: 
  - 고주파 타임라인이 저주파 완료 대기 없음
  - NonRT 예측불가 실행이 RT 영향 없음
  - 순환 참조 해결

```cpp
// Weak 의존성: 명시적으로 선언
void setup(TaskRegistry& r) override {
    r.add_dependency(state_reader_);  // Weak 의존성으로 선언
}

void execute(void* state_ptr) override {
    if (state_reader_) {  // 데이터 가용성 확인
        const auto& state = *state_reader_;  // 최신 데이터 사용
    }
}
```

### 2. 단일 작성자 원칙 (Single-Writer Principle)

```
┌─────────────────────────────────────────┐
│  Key: "sensors.high_freq"               │
├─────────────────────────────────────────┤
│  WRITER (O): SensorProducer (1000Hz)    │
│  READERS  : StateEstimator              │
│            TorqueController              │
│            DataLogger (NonRT)            │
└─────────────────────────────────────────┘

❌ 절대 금지: 같은 키에 2개 이상의 Writer
```

**이 규칙의 이점**:
1. 데이터 흐름이 항상 DAG(단방향)
2. 순환 의존성 원천 차단
3. 스케줄 예측 가능 → 결정론적 실행

### 3. N-Way 버퍼링으로 타임라인 격리

#### 버퍼 할당 전략

```cpp
N-way 버퍼 수 = (타임라인 수) + (NonRT 태스크 그룹 수) + 여유분
             = (1000Hz + 200Hz + 30Hz + 10Hz) + 1 + 1
             = 4 + 1 + 1 = 6개 버퍼
```

#### 실행 시나리오: "sensors.high_freq" 데이터 (1000Hz와 200Hz 타임라인)

**각 타임라인은 자신의 주기 시작/종료 시점에만 capture/release를 수행**

```
Tick 0 (1000Hz 주기 #0 / 200Hz 주기 #0 시작):
├─ 1000Hz Timeline:
│  ├─ tick_start: capture
│  │  • sensors.high_freq 버퍼의 ref_count 증가
│  │
│  ├─ execute:
│  │  • SensorProducer: 버퍼 #1에 센서 데이터 쓰기
│  │  • TorqueController: 읽기 버퍼에서 센서 읽기 (강 의존성)
│  │
│  └─ tick_end: commit & release
│     • 새 데이터(버퍼 #1)를 ready_index로 설정
│     • capture했던 버퍼의 ref_count 감소 (소유권 반환)
│
└─ 200Hz Timeline:
   ├─ tick_start: capture (주기 #0 시작)
   │  • sensors.high_freq, robot.gait.planned 버퍼 ref_count 증가
   │
   └─ execute
      [이 주기는 Tick 0~4 동안 지속]

Tick 1 (1000Hz 주기 #1):
├─ 1000Hz Timeline:
│  ├─ tick_start: capture
│  ├─ execute
│  └─ tick_end: commit & release
│
└─ 200Hz Timeline:
   └─ execute

Tick 2, 3: (1000Hz와 동일한 패턴, 200Hz는 주기 진행 중)



Tick 4 (1000Hz 주기 #5 / 200Hz 주기 #0 종료, 주기 #1 시작):
├─ 1000Hz Timeline:
│  ├─ tick_start: capture
│  ├─ execute
│  └─ tick_end: commit & release
│
└─ 200Hz Timeline:
   ├─ execute
   │
   └─ tick_end (주기 #0 종료): commit & release
      • 계산 결과를 ready_index로 설정
      • capture했던 버퍼들의 ref_count 감소 (소유권 반환)

Tick 5 (1000Hz 주기 #5 / 200Hz 주기 #0 종료, 주기 #1 시작):
├─ 1000Hz Timeline:
│  ├─ tick_start: capture
│  ├─ execute
│  └─ tick_end: commit & release
│
└─ 200Hz Timeline:
   ├─ tick_start: capture (주기 #1 시작)
   │  • Tick 0~4 동안 1000Hz가 갱신한 최신 센서 데이터 capture
   │  • 새로운 버퍼들의 ref_count 증가
   │
   └─ execute

Tick 6~9: (1000Hz와 동일한 패턴, 200Hz는 주기 #1 진행 중)

Tick 10 (1000Hz 주기 #10 / 200Hz 주기 #1 종료, 주기 #2 시작):
├─ 1000Hz Timeline: (위와 동일)
│
└─ 200Hz Timeline:
   ├─ tick_start (주기 #2 시작)
   └─ (이 주기는 Tick 10~14 동안 지속)
```

**핵심 개념**:
1. **주기별 독립 실행**: 1000Hz는 매 Tick마다 한 주기, 200Hz는 5개 Tick마다 한 주기
   - 1000Hz: Tick 0~1~2~3~4~5~... (매 tick마다 주기 #0, #1, #2, ...)
   - 200Hz: Tick 0~4 (주기 #0), Tick 5~9 (주기 #1), Tick 10~14 (주기 #2), ...

2. **Capture (주기 시작)**: 버퍼의 ref_count 증가로 "이 버퍼는 지금 내가 사용 중"이라 표시
   - 소유한 타임라인이 이 버퍼를 변경할 수 없도록 보호 (ref_count가 0이 될 때까지 기다림)

3. **Release (주기 종료)**: ref_count 감소로 "이제 내가 사용 완료, 소유한 타임라인이 쓸 수 있도록" 참조 해제
   - 반드시 capture한 타임라인만 release 호출 가능

4. **데이터 격리**: 200Hz가 Tick 0~4 동안 capture한 버퍼는 이 기간 동안 
   - 1000Hz가 계속 새로운 데이터를 쓰더라도, 200Hz는 capture 시점의 스냅샷만 사용

5. **Weak 의존성**: 다른 타임라인을 기다리지 않고 **최신 데이터(ready_index가 가리키는 버퍼)** 사용
   - TorqueController가 StateEstimator 결과를 Weak로 읽으면:
     - StateEstimator가 아직 이번 주기에 결과를 안 냈으면 이전 주기의 최신 결과 사용
     - StateEstimator를 기다리지 않으므로 1000Hz 데드라인에 영향 없음

### 4. 함수형 패러다임과 재현성

#### Pure Function 원칙

```cpp
class SensorProducer : public rt::ITask {
    void execute(void* state_ptr) override {
        // 입력: 현재 센서 데이터(외부)
        // 출력: 처리된 센서 데이터(deterministic)
        // 부작용: 없음 (logger 제외)
        
        double raw_value = hardware_sensor_.read();
        double filtered = lpf_.filter(raw_value);  // 순수 계산
        sensor_output_.write(filtered);
    }
};
```

#### 재현 메커니즘

```
Recording Mode:
  tick 0: [센서 입력 1, 콘솔 파라미터 입력] → 저장
  tick 1: [센서 입력 2, 파라미터 변화] → 저장
  ...
  File: record.rtrec (모든 외부 입력)

Simulation Mode (--simul record.rtrec):
  tick 0: [센서 입력 1, 파라미터 입력] → 재생
  모든 RT 태스크 실행 → 동일한 결과
  tick 1: [센서 입력 2, 파라미터 변화] → 재생
  ...
  
✓ 완벽 재현 (Float 정밀도 범위 내)
✓ 모든 중간 데이터 저장 불필요
```

---

## 디렉토리 구조

```
rtfw_project_purism/
│
├── 📁 rtfw/                          # 핵심 프레임워크 엔진
│   ├── include/rtfw/
│   │   ├── rt_framework.h           # 메인 API (registerTask, start, stop)
│   │   ├── task.h                   # ITask 인터페이스 & TaskRegistry
│   │   ├── timeline.h               # 내부: 타임라인 구현
│   │   ├── task_proxies.h           # DataReader/DataWriter 래퍼
│   │   ├── framework.h              # 통합 헤더
│   │   ├── parameter_manager.h      # 파라미터 로딩 (YAML)
│   │   ├── blocking_queue.h         # 스레드 안전 큐
│   │   ├── shm_context.h            # 공유 메모리 관리
│   │   ├── shm_allocator.h          # SHM 할당자
│   │   └── shm_ringbuffer_sink.h    # 로깅용 SHM 링버퍼
│   │
│   └── src/
│       ├── rt_framework.cpp         # 핵심 로직 (1000+ 줄)
│       ├── task.cpp                 # ITask 구현
│       ├── timeline.cpp             # 타임라인 실행 루프
│       ├── task_proxies.cpp
│       ├── parameter_manager.cpp
│       └── shm_context.cpp
│
├── 📁 rtfw-common/                  # 공유 데이터 구조 & 포맷
│   ├── include/rtfw_common/
│   │   ├── shm_layout.h             # SHM 바이너리 레이아웃
│   │   ├── blackbox.h               # 녹화/재생 인터페이스
│   │   ├── log_format.h             # 로그 엔트리 포맷
│   │   └── type_utils.h             # 타입 해싱, 직렬화
│   │
│   └── src/
│       └── blackbox.cpp             # 파일 기반 녹화/재생 구현
│
├── 📁 rtfw-connect/                 # 외부 애플리케이션용 클라이언트 API
│   ├── include/rtfw_connect/
│   │   ├── shm_querier.h            # 공유 메모리 읽기 API
│   │   └── shm_controller.h         # 프레임워크 제어 API
│   │
│   └── src/
│       ├── shm_querier.cpp
│       └── shm_controller.cpp
│
├── 📁 apps/                         # 예제 애플리케이션 및 도구
│   │
│   ├── 📁 poc_app/                  # 기본 예제 (휴머노이드 로봇 제어)
│   │   ├── biped_data_types.h      # 데이터 구조 정의
│   │   ├── biped_task.h            # 모든 태스크 클래스
│   │   └── main.cpp                # 진입점 (프레임워크 초기화)
│   │
│   ├── 📁 crane_4wis/              # 4륜 휠 다리 로봇 제어 앱
│   │   ├── main.cpp
│   │   ├── json_exporter.cpp       # JSON 변환 유틸리티
│   │   └── task_pool/              # 태스크 구현
│   │
│   ├── 📁 mobile_crane/            # 이동식 크레인 제어 앱
│   │   ├── main.cpp
│   │   ├── json_exporter.cpp
│   │   └── task_pool/
│   │
│   ├── 📁 rtmonitor/               # 실시간 모니터링 GUI
│   │   ├── main.cpp
│   │   ├── ui_state.h/cpp          # UI 상태 관리
│   │   ├── stats_renderer.h/cpp    # 통계 렌더링
│   │   ├── graph_renderer.h/cpp    # 의존성 그래프 시각화
│   │   └── bridge_sample.cpp       # 테스트/예제
│   │
│   ├── 📁 rtviewer/                # 오프라인 로그 분석 GUI
│   │   ├── main.cpp
│   │   ├── editor_ui.h/cpp         # 시간축 에디터
│   │   └── imgui_memory_editor.h   # 메모리 뷰어
│   │
│   └── 📁 test_4wis/               # ROS2 테스트 패키지
│       ├── launch/
│       ├── script/
│       └── src/
│
├── 📁 build/                        # 빌드 산출물 (git ignore)
│   ├── apps/
│   ├── rtfw/
│   ├── rtfw-common/
│   └── rtfw-connect/
│
├── CMakeLists.txt                   # 최상위 CMake 설정
├── README.txt                       # 기본 프로젝트 설명
├── PROJECT_DOCUMENTATION.md        # 본 문서
│
├── 🔧 설정 파일
│   ├── param_test.yaml             # 테스트용 파라미터
│   ├── robot_param.yaml            # 로봇 파라미터
│   └── crane_parameters.yaml       # 크레인 파라미터
│
└── 📊 녹화 파일 (테스트용)
    ├── record_20250825_1.rtrec
    ├── record_20250905_*.rtrec
    └── ...
```

### 핵심 파일별 역할

| 파일 | 목적 | 주요 클래스/함수 |
|------|------|-----------------|
| `rt_framework.h/cpp` | 프레임워크 메인 API | `RealTimeFramework`, `registerTask()` |
| `task.h/cpp` | 태스크 인터페이스 | `ITask`, `TaskRegistry` |
| `timeline.h/cpp` | 주파수별 실행 루프 | `Timeline`, `runSchedulerLoop()` |
| `shm_context.h/cpp` | 공유 메모리 관리 | `ShmContext`, 버퍼 할당/접근 |
| `biped_task.h` | 예제 태스크 | `SensorProducer`, `TorqueController` |
| `main.cpp` | 프레임워크 초기화 | 태스크 등록, 설정, 실행 |

---

## 주요 컴포넌트

### 1. RealTimeFramework (핵심 엔진)

**역할**: 전체 시스템 관리 및 오케스트레이션

```cpp
class RealTimeFramework {
    // --- 공개 API ---
    void registerTask(std::unique_ptr<rt::ITask> task, 
                      int frequency, 
                      int cpu_affinity = -1);
    void registerNonRtTask(std::unique_ptr<rt::ITask> task, 
                           int frequency);
    
    void initialize(FrameworkConfig&& config);
    void start();    // 모든 타임라인 스레드 시작
    void stop();     // 실행 중지 신호
    void join();     // 모든 스레드 종료 대기
    
    // --- 녹화/재생 제어 ---
    bool startRecord(const std::string& filename);
    void stopRecord();
    bool startReplay(const std::string& filename);
};
```

**핵심 프로세스**:

1. **Task Registration**: 모든 RT/NonRT 태스크 등록
2. **Dependency Analysis**: 데이터 의존성 분석 및 사이클 검사
3. **SHM Initialization**: 공유 메모리 레이아웃 구축
4. **Timeline Creation**: 주파수별 타임라인 생성
5. **Thread Launch**: 각 타임라인용 실행 스레드 시작
6. **Execution Loop**: 각 타임라인이 주기적으로 태스크 실행

### 2. ITask 인터페이스

**모든 태스크가 상속해야 할 기본 클래스**

```cpp
class ITask {
public:
    // 필수 구현
    virtual const char* getName() const = 0;
    virtual void setup(TaskRegistry& registry) = 0;
    virtual void execute(void* state_ptr) = 0;
    
    // 선택 사항
    virtual void initialize(void* state_ptr) {}
    virtual void warmup() const {}
    virtual size_t getStateSize() const { return 0; }
    
    // 접근자
    uint32_t getID() const;
    uint32_t getFrequency() const;
    int getAffinity() const;
    std::shared_ptr<spdlog::logger> getLogger();
};
```

**구현 예시**:

```cpp
class SensorProducer : public rt::ITask {
private:
    DataWriter<HighFreqSensors> sensor_output_;
    HardwareInterface* hw_;

public:
    const char* getName() const override { return "SensorProducer"; }
    
    void setup(TaskRegistry& r) override {
        // setup()에는 DataWriter 등록 (읽기 의존성 없음)
    }
    
    void execute(void* state_ptr) override {
        HighFreqSensors data;
        hw_->read(data);  // 센서에서 읽기
        sensor_output_.write(data);  // 프레임워크에 쓰기
    }
};
```

### 3. TaskRegistry & 의존성 선언

**역할**: 태스크가 자신의 입출력을 선언하는 인터페이스

```cpp
void setup(TaskRegistry& r) override {
    // Strong 의존성: 같은 주기/타임라인 내 순차 실행 보장
    r.add_dependency(sensor_data_);
    
    // Strong 의존성
    r.add_dependency(motor_command_);
    
    // Weak 의존성: 다른 주파수/타임라인의 데이터, 명시적 선언 필요
    r.add_dependency(state_reader_);  // DependencyType::Weak 지정
}
```

**프레임워크가 하는 일**:
- `add_dependency()` 호출을 추적
- 반대 방향 의존성 맵 구축
- 실행 순서 결정

### 4. DataReader & DataWriter (Task Proxies)

**역할**: 타입 안전한 데이터 읽기/쓰기

```cpp
// 태스크 멤버에서 선언
DataWriter<HighFreqSensors> sensor_output_{"sensors.high_freq"};
DataReader<HighFreqSensors> sensor_input_{"sensors.high_freq", DependencyType::Strong};
DataReader<RobotState> state_input_{"robot.state", DependencyType::Weak};

// execute() 내에서 사용
void execute(void* state_ptr) override {
    // 쓰기
    HighFreqSensors data = {...};
    sensor_output_.write(data);
    
    // Strong 의존성으로 읽기: 현재 버퍼에 데이터 있으면 읽음
    if (sensor_input_) {
        const auto& sensor_data = sensor_input_.read();
        // ...
    }
    
    // Weak 의존성으로 읽기: 최신 데이터 확인
    if (state_input_) {
        const auto& state = *state_input_;  // operator* 사용
        // ...
    }
}
```

**내부 동작**:
- 타입 해싱으로 고유 키 생성
- 프레임워크의 ShmContext에서 버퍼 할당
- Reference counting으로 N-way 버퍼 관리

### 5. Timeline (주파수별 실행 엔진)

**역할**: 단일 주파수의 태스크들을 주기적으로 실행

```cpp
class Timeline {
private:
    int _frequency;                           // 실행 빈도 (Hz)
    std::vector<rt::ITask*> _tasks_in_timeline;  // 이 타임라인의 모든 태스크
    std::atomic<int>[] _dependency_counters;  // 각 태스크의 남은 의존성 수
    std::atomic<size_t> _remained_task_count; // 남은 태스크 수
};

// 프레임워크 메인 루프에 의해 호출되는 함수들
// 메인 스케줄러가 매 글로벌 tick마다:
void tick_start(uint64_t current_global_tick) {
    // 이번 tick에서 이 타임라인이 실행할 차례인지 확인
    // 맞으면, 외부 버퍼 capture (ref_count 증가) 및
    // 초기 실행 가능 태스크를 워커 큐에 enqueue
}

void tick_end(uint64_t current_global_tick) {
    // 이번 주기의 모든 태스크가 완료되었는지 확인
    // 맞으면, 쓰기 버퍼 commit (ready_index 갱신)
    // 읽기 버퍼 release (ref_count 감소)
}

// 워커 스레드가 태스크 완료 후 호출:
void onTaskFinished(rt::ITask* finished_task) {
    // 이 태스크에 의존하는 다른 태스크의 dependency_counter 감소
    // 카운터가 0이 되면 해당 태스크를 워커 큐에 enqueue
}
```

### 6. Shared Memory System

**구조**:

```cpp
┌─────────────────────────────────┐
│  SharedMemoryHeader             │
│  • 버전, 크기, 타임스탬프      │
│  • 타임라인 정보                │
│  • 로그 링버퍼 오프셋           │
└─────────────────────────────────┘
         ▼
┌─────────────────────────────────┐
│  DataBlocks (사용자 데이터)      │
│  ┌─────────────┐                │
│  │ sensors     │  (N개 버퍼)    │
│  ├─────────────┤                │
│  │ motor_cmd   │  (N개 버퍼)    │
│  ├─────────────┤                │
│  │ state_est   │  (N개 버퍼)    │
│  └─────────────┘                │
└─────────────────────────────────┘
         ▼
┌─────────────────────────────────┐
│  Ringbuffer (로그 엔트리)        │
│  • TaskExecutionRecord          │
│  • DataWriteRecord              │
│  • CheckpointRecord             │
└─────────────────────────────────┘
```

**할당 알고리즘**:

```cpp
// 각 데이터 키마다
N-way 버퍼 수 = max(
    타임라인 수,
    NonRT 태스크 그룹 수
) + 여유분

// 예: 4개 주파수 + 1개 NonRT 그룹
= 4 + 1 = 5개 버퍼
```

---

## 개발 가이드

### 새로운 태스크 추가 체크리스트

#### Step 1: 데이터 구조 정의

파일: `apps/poc_app/biped_data_types.h`

```cpp
#pragma once

struct HighFreqSensors {
    double joint_positions[12];
    double joint_velocities[12];
    double imu_accel[3];
    double imu_gyro[3];
};

struct TorqueCommand {
    double desired_torque[12];
};

struct EstimatedState {
    double com_position[3];
    double com_velocity[3];
    double base_orientation[4];  // quaternion
};
```

**필수 조건**:
- Plain-Old-Data (POD) 타입 또는 trivially copyable
- 동적 메모리 없음 (shared memory 때문)
- 정렬/패딩 명시적으로 관리

#### Step 2: 태스크 클래스 구현

파일: `apps/poc_app/biped_task.h`

```cpp
#include "rtfw/task.h"
#include "biped_data_types.h"

class StateEstimator : public rtfw::rt::ITask {
private:
    // 입력 프록시
    DataReader<HighFreqSensors> sensor_input_;
    
    // 출력 프록시
    DataWriter<EstimatedState> state_output_;
    
    // 로컬 상태 (필요시)
    Eigen::Quaterniond orientation_;
    Eigen::Vector3d com_velocity_;

public:
    const char* getName() const override { 
        return "StateEstimator"; 
    }
    
    void setup(TaskRegistry& r) override {
        // ✓ 반드시 sensor_input_ 등록
        r.add_dependency(sensor_input_);
        // state_output_은 등록 불필요
    }
    
    void execute(void* state_ptr) override {
        // 입력 읽기
        const auto& sensors = sensor_input_.read();
        
        // 계산
        EstimatedState new_state;
        estimate(sensors, new_state);  // 순수 함수
        
        // 출력 쓰기
        state_output_.write(new_state);
    }

private:
    void estimate(const HighFreqSensors& sensors, 
                  EstimatedState& output) {
        // 상태 추정 알고리즘
        // Kalman Filter, Complementary Filter 등
    }
};
```

#### Step 3: 메인에서 등록

파일: `apps/poc_app/main.cpp`

```cpp
int main(int argc, char** argv) {
    rtfw::FrameworkConfig config;
    rtfw::RealTimeFramework framework;
    
    // --- 모든 RT 태스크 등록 ---
    
    // 1000Hz 타임라인 (센서 획득, 제어)
    framework.registerTask(
        std::make_unique<SensorProducer>(), 
        1000,  // frequency (Hz)
        7      // CPU core affinity
    );
    
    framework.registerTask(
        std::make_unique<TorqueController>(), 
        1000,
        7
    );
    
    // 200Hz 타임라인 (상태 추정)
    framework.registerTask(
        std::make_unique<StateEstimator>(), 
        200,
        6
    );
    
    // 10Hz 타임라인 (계획)
    framework.registerTask(
        std::make_unique<MotionPlanner>(), 
        10,
        -1  // 공용 스레드풀 사용
    );
    
    // --- NonRT 태스크 등록 ---
    framework.registerNonRtTask(
        std::make_unique<DataLogger>(), 
        1  // 1Hz (NonRT, 경성 실시간 무시)
    );
    
    // --- 설정 ---
    config.threads.num_common_threads = 4;
    config.threads.dedicated_core_threads = {
        {7, 1},  // Core 7: 1개 스레드
        {6, 1},  // Core 6: 1개 스레드
    };
    config.threads.num_non_rt_threads = 2;
    
    config.parameter_file_path = "param_test.yaml";
    
    // --- 실행 ---
    framework.initialize(std::move(config));
    framework.start();
    framework.join();
    
    return 0;
}
```

### 의존성 관계 설계 가이드

#### ✓ 올바른 패턴

```
Sensor (1000Hz)
    ↓ (strong: fresh data needed)
State Estimator (200Hz)
    ↓ (weak: latest is enough)
Motion Planner (10Hz)
    ↓ (strong: same timeline)
Trajectory Executor (10Hz)
    ↓ (weak: RT can't wait NonRT)
Data Logger (1Hz, NonRT)
```

#### ✗ 피해야 할 패턴

```
❌ 순환 참조
    Task A → Task B → Task C → Task A

❌ 같은 키에 2개 Writer
    SensorProducer.1 → "sensors"
    SensorProducer.2 → "sensors"  (금지!)

❌ NonRT → RT 강의존성
    DataLogger (NonRT) → [strong] → TorqueController (RT)
    // RT 데드라인 위협!
```

### 상태 유지가 필요한 경우

일부 태스크는 내부 상태 유지 필요 (필터, 인테그레이터 등).

```cpp
// 방법 1: 멤버 변수 (간단함)
class StateEstimator : public rtfw::rt::ITask {
private:
    Eigen::Quaterniond orientation_;  // 유지되는 상태
    
    void execute(void* state_ptr) override {
        const auto& sensors = sensor_input_.read();
        orientation_ = update_quaternion(orientation_, sensors);
        // ...
    }
};

// 방법 2: Task (복사 지원)
class StatefulCounter : public rtfw::Task<CounterState> {
    void execute(CounterState& state) override {
        state.count++;
        state.total += state.count;
    }
};

// Task는 다음과 같음:
// struct CounterState {
//     int count = 0;
//     int total = 0;
// };
```

### 파라미터 로딩

**Parameter**: DataReader/DataWriter와 독립적으로 관리되는 설정값. YAML 파일에서 로드됩니다.

```yaml
# param_test.yaml
gain:
  kp: 10.0
  ki: 0.1
  kd: 0.5

friction:
  damping: 0.05
  stiction: 0.01

robot:
  mass: 30.0
  height: 1.8
```

```cpp
class TorqueController : public rtfw::rt::ITask {
private:
    Parameter<double> kp_;    // YAML에서 로드: gain.kp
    Parameter<double> ki_;    // YAML에서 로드: gain.ki
    Parameter<double> kd_;    // YAML에서 로드: gain.kd

public:
    void setup(TaskRegistry& r) override {
        // 파라미터는 DataReader/Writer와 달리
        // setup()에서 명시적 등록 불필요
        // 프레임워크가 YAML 경로로 자동 로드
    }
    
    void execute(void* state_ptr) override {
        double error = target_ - current_;
        // Parameter::get()으로 값 접근
        double u = kp_.get() * error 
               + ki_.get() * integral_ 
               + kd_.get() * derivative_;
        // ...
    }
};
```

**Parameter vs DataReader/Writer**:
- **Parameter**: YAML 파일에서 정적으로 로드되는 설정값
- **DataReader/Writer**: 런타임에 태스크 간 전달되는 동적 데이터

### 로깅

모든 태스크에서 `getLogger()`로 접근 가능:

```cpp
void execute(void* state_ptr) override {
    auto logger = getLogger();
    
    logger->info("Sensor reading: {}", sensor_value);
    logger->warn("Temperature high: {} °C", temp);
    logger->error("Hardware error code: {}", error_code);
    
    // 로그는 SHM 링버퍼에 저장됨 (비동기)
}
```

---

## 빌드 및 실행

### 빌드 환경 요구사항

- **컴파일러**: GCC/Clang (C++17 이상)
- **빌드 시스템**: CMake 3.15+
- **의존성**:
  - Boost (headers only)
  - spdlog 1.8+
  - POSIX threads (`pthread`)
  - Real-time library (`librt`)
- **OS**: Linux (POSIX RT scheduler 필요)

### 빌드 단계

```bash
# 1. 빌드 디렉토리 생성
mkdir -p build && cd build

# 2. CMake 설정
cmake ..

# 3. 컴파일
cmake --build . -j$(nproc)

# 4. 산출물 위치
# - apps/poc_app/poc_app (실행 가능)
# - apps/rtmonitor/rtmonitor (GUI)
# - apps/rtviewer/rtviewer (로그 뷰어)
```

### 기본 실행

```bash
cd build/apps/poc_app

# LIVE 모드 (실시간 제어)
./poc_app

# 참고: poc_app은 현재 런타임 제어 지원으로 개선 중입니다.
# 실행 후 외부 도구(rtmonitor 등)에서 상태 제어 가능합니다.
```

**참고**: 이전의 `--live`, `--record`, `--simul`, `--trace` 옵션은 개발 중 단계적으로 변경될 수 있습니다. 최신 정보는 main.cpp 파일을 참조하세요.

### 실시간 모니터링

```bash
# 터미널 1: 애플리케이션 실행
./poc_app --live

# 터미널 2: 모니터 도구 실행
cd ../rtmonitor
./rtmonitor

# GUI 활성화
# - 좌측: 태스크 의존성 그래프
# - 중앙: 실시간 데이터 값
# - 우측: 성능 통계 (실행 시간, 주기, 메모리)
```

### 오프라인 분석

```bash
# 로그 뷰어
cd ../rtviewer
./rtviewer

# 메뉴: File → Open Record
# → 저장된 .rtrec 파일 선택
# → 타임라인 스크럽, 메모리 검사

# JSON 변환
cd ../crane_4wis
./json_exporter record.rtrec output.json

# Python에서 분석
python3 -c "
import json
with open('output.json') as f:
    data = json.load(f)
    print('Tasks:', list(data.keys()))
    print('Sensor data length:', len(data['sensors']))
"
```

### 문제 해결

```bash
# 1. 권한 오류 (SCHED_FIFO 실패)
# 해결: sudo 실행 또는 CAP_SYS_NICE 권한 부여
sudo ./poc_app --live

# 또는
setcap cap_sys_nice=ep ./poc_app
./poc_app --live

# 2. CPU affinity 오류
# 해결: 사용 가능한 코어 확인
nproc
# CPU 개수보다 높은 affinity 설정 제거

# 3. 공유 메모리 오류
# 해결: 기존 SHM 정리
ipcs -m
ipcrm -m <shmid>

# 4. 빌드 실패
# 해결: 의존성 확인
pkg-config --cflags --libs spdlog
sudo apt install libspdlog-dev  # Debian/Ubuntu
```

---

## 협업 체크리스트

### 코드 리뷰 시 확인 사항

#### 1. 단일 작성자 원칙

- [ ] 새 데이터 키에 Writer가 정확히 1개인가?
- [ ] 같은 키의 여러 Writer 존재하지 않음?
- [ ] Reader는 여러 개 가능함을 이해했는가?

#### 2. 의존성 선언

- [ ] `setup()`에서 모든 읽기 의존성 선언?
- [ ] 읽기 없는 Producer 태스크는 의존성 선언 안 함?
- [ ] Writer 선언은 필요 없음?

#### 3. 데이터 타입

- [ ] POD 타입 또는 trivially copyable?
- [ ] 동적 메모리 할당 없음?
- [ ] 정렬/패딩 명시적으로 처리?

#### 4. 함수형 패러다임

- [ ] execute()가 순수 함수 패턴?
- [ ] 멤버 상태 변화 최소화?
- [ ] 외부 상태에 의존하지 않음?

#### 5. 실시간 안정성

- [ ] execute() 내 sleep()/blocking call 없음?
- [ ] 동적 메모리 할당 없음?
- [ ] 예측 불가 라이브러리 함수 피함?
- [ ] NonRT 태스크와 RT 태스크 분리 정확?

#### 6. CPU Affinity

- [ ] 고주파 작업(1000Hz+)에 전용 코어 할당?
- [ ] 여러 타임라인이 같은 코어 공유하지 않음?
- [ ] 시스템의 코어 수 고려?

#### 7. 테스트

- [ ] LIVE 모드 작동 확인?
- [ ] RECORDING 모드 데이터 저장 확인?
- [ ] SIMULATION 모드 재현 성공?
- [ ] rtmonitor에서 통계 정상?

### 병합 전 커밋 메시지 템플릿

```
[태스크 타입] 짧은 설명 (50자 이내)

상세 설명:
- 어떤 문제를 해결했는가?
- 어떤 설계 결정을 했는가?
- 테스트 방법은?

의존성 정보:
- 새 태스크: [TaskName] (주파수, Core affinity)
- 새 데이터 키: [Key] (Writer: [TaskName], Reader: [...])

체크리스트:
- [ ] 단일 작성자 원칙 준수
- [ ] 의존성 선언 완료
- [ ] LIVE/RECORDING/SIMULATION 모드 테스트
- [ ] 실시간 안정성 검증
```

### 성능 프로파일링

#### 실행 시간 측정

```bash
# rtmonitor에서 우측 통계 확인
# 각 태스크의 min/max/avg 실행 시간 표시

# 콘솔 출력
./poc_app --live
# [Ctrl+C로 중지]
# Statistics 출력됨:
# Task: SensorProducer
#   Freq: 1000Hz, CPU: 7
#   Exec time: min=0.5ms, max=2.1ms, avg=0.8ms
#   Deadline misses: 0
```

#### 메모리 사용량

```bash
# 프로세스 메모리
ps aux | grep poc_app
# 또는 rtviewer에서 메모리 에디터 열기
```

#### CPU 사용률

```bash
# top 모니터링
top -p <pid> -H  # 스레드별 보기

# 또는 perf
sudo perf top -p <pid>
```

### 문서화 요구사항

새 태스크/기능 추가 시:

1. **inline 주석** (복잡한 알고리즘)
2. **클래스 주석** (역할, 입출력)
3. **파라미터 주석** (의미, 범위)
4. **실행 시간 주석** (예상 실행 시간)
5. **CPU affinity 정당화** (왜 이 코어인가?)

예:

```cpp
/**
 * StateEstimator: 센서 데이터에서 로봇 상태 추정
 * 
 * 입력: HighFreqSensors (1000Hz)
 * 출력: EstimatedState (200Hz로 다운샘플링)
 * 
 * 알고리즘: EKF (Extended Kalman Filter)
 * - 계산 복잡도: O(n²) where n=12 (자유도)
 * - 예상 실행 시간: 1-2ms
 * - CPU: Core 6 (센서와 분리, 고주파와 동기화)
 * 
 * 주의: 쿼터니언 정규화 매 주기 필수
 */
class StateEstimator : public rtfw::rt::ITask { ... };
```

---

## 추가 자료

### 참고 링크

- **CMake 공식 문서**: https://cmake.org/cmake/help/latest/
- **spdlog 문서**: https://github.com/gabime/spdlog
- **Real-time Linux**: https://wiki.linuxfoundation.org/realtime/

### 핵심 파일 링크

| 파일 | 용도 |
|------|------|
| [rtfw/src/rt_framework.cpp](rtfw/src/rt_framework.cpp) | 핵심 엔진 로직 |
| [apps/poc_app/main.cpp](apps/poc_app/main.cpp) | 사용 예제 |
| [rtfw/include/rtfw/task.h](rtfw/include/rtfw/task.h) | ITask API |
| [param_test.yaml](param_test.yaml) | 파라미터 파일 예제 |

### FAQ

**Q: 1000Hz 타임라인이 200Hz 데이터가 필요하면?**

A: `DataReader.readLatest()`로 최신 값만 사용. 강의존성으로 선언하지 않아야 1000Hz가 200Hz 완료 대기 안 함.

**Q: 같은 주파수 태스크들의 순서는?**

A: 프레임워크가 자동 결정. 의존성이 없으면 병렬 실행 가능.

**Q: NonRT 태스크 중에 오래 걸리는 작업은?**

A: RT에 영향 없음. 별도 스레드풀에서 실행. 하지만 다음 비RT 태스크는 대기 가능.

**Q: CPU affinity -1은?**

A: 공용 스레드풀 사용. OS 스케줄러가 자동 배치. 저주파/경성 아닌 작업에 적합.

---

## 요약

RTFW는 **데이터 흐름**으로 **실행을 결정**하는 프레임워크입니다.

- **개발자**: 순수 함수 같은 태스크만 구현
- **프레임워크**: 의존성 분석, 스케줄링, 동기화, 격리 담당
- **결과**: 예측 가능한 실시간 시스템

협업 시 **단일 작성자 원칙**과 **의존성 선언**을 철저히 지키면, 프레임워크가 나머지를 자동으로 처리합니다.

---

**최종 수정**: 2026년 2월 21일  
**작성자**: AI Assistant (GitHub Copilot)  
**피드백**: 이 문서에 대한 질문이나 제안은 팀 위키나 이슈 트래커에 등록해 주세요.
