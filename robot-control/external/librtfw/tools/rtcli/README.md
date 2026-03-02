# rtcli - Real-Time Framework CLI

## 개요

`rtcli`(Real-Time Framework CLI)는 RT Framework와 상호작용하기 위한 command-line interface입니다. 원래 `shm_control`이었던 도구를 리팩토링하여 모듈식 구조로 개선하였으며, 향후 기능 확장을 용이하게 설계했습니다.

## 특징

- **모듈식 아키텍처**: 커맨드 핸들러 기반의 확장 가능한 구조
- **그룹별 기능 분리**: `sched`, `record`, `replay`, `trace`, `check_record` 등 기능별 모듈화
- **향후 확장성**: 파라미터 로드, DataReader/Writer 속성 변경 등 새로운 기능 추가 용이

## 프로젝트 구조

```
tools/rtcli/
├── main.cpp                   # CLI 엔트리 포인트
├── shm_common.h/cpp          # 공유 메모리 컨텍스트 (RAII 패턴)
├── command_handler.h/cpp     # 커맨드 기본 인터페이스 및 디스패처
├── scheduler_commands.h/cpp  # 스케줄러 제어 (pause, play, play_one)
└── record_commands.h/cpp     # 레코딩/리플레이 (record, replay, trace, check_record)
```

## 사용 방법

### 기본 문법
```bash
rtcli <command> <subcommand> [args]
```

### 스케줄러 제어

```bash
# 틱 처리 일시 중지
./rtcli sched pause

# 틱 처리 재개
./rtcli sched play

# 한 틱 전진
./rtcli sched play_one
```

### 레코딩

```bash
# 레코딩 시작
./rtcli record start /path/to/output.rtrec

# 레코딩 중지
./rtcli record stop
```

### 리플레이

```bash
# 리플레이 시작
./rtcli replay start /path/to/input.rtrec

# 리플레이 중지
./rtcli replay stop
```

### 트레이스 (리플레이 + 레코딩)

```bash
# 트레이스 시작 (입력과 출력 동시 처리)
./rtcli trace start /path/to/input.rtrec /path/to/output.rtrec

# 트레이스 중지
./rtcli trace stop
```

### 레코드 파일 검증

```bash
# 기본 검증
./rtcli check_record /path/to/file.rtrec

# 키 해시값 함께 표시
./rtcli check_record /path/to/file.rtrec --show-hash
```

## 아키텍처 설명

### 계층 구조

```
┌─────────────────────────────────────┐
│         main.cpp                     │ 엔트리 포인트, 커맨드 등록 및 라우팅
├─────────────────────────────────────┤
│      CommandDispatcher               │ 커맨드 라우팅 및 등록 관리
├─────────────────────────────────────┤
│    CommandHandler (base)             │ 모든 핸들러의 기본 인터페이스
├─────────────────────────────────────┤
│  Concrete Handlers:                 │
│  - SchedulerCommandHandler           │ 스케줄러 제어
│  - RecordCommandHandler              │ 레코딩
│  - ReplayCommandHandler              │ 리플레이
│  - TraceCommandHandler               │ 트레이스
│  - CheckRecordCommandHandler         │ 파일 검증
├─────────────────────────────────────┤
│    SharedMemoryContext (RAII)        │ SHM 연결/정리
└─────────────────────────────────────┘
```

### 핵심 설계 패턴

#### 1. CommandHandler 인터페이스

모든 커맨드 핸들러는 `CommandHandler`를 상속받습니다:

```cpp
class CommandHandler {
public:
    virtual int execute(const std::vector<std::string>& args, 
                       SharedMemoryContext& ctx) = 0;
    virtual std::string name() const = 0;
    virtual std::string help() const = 0;
};
```

#### 2. SharedMemoryContext (RAII)

메모리 누수 방지를 위해 RAII 패턴 사용:

```cpp
class SharedMemoryContext {
    // 생성자: SHM 열기
    // 소멸자: 자동으로 SHM 정리
};
```

#### 3. CommandDispatcher

커맨드 등록 및 라우팅:

```cpp
dispatcher.register_handler(std::make_unique<MyCommandHandler>());
dispatcher.dispatch(args, shm_ctx);
```

## 향후 확장 계획

### 1. 파라미터 관리 (parameters_commands.h/cpp)

```bash
./rtcli param load /path/to/params.yaml
./rtcli param reload
./rtcli param list
./rtcli param get <key>
./rtcli param set <key> <value>
```

### 2. DataReader/Writer 제어 (data_commands.h/cpp)

```bash
./rtcli data list-keys
./rtcli data get <key>
./rtcli data properties <key>
./rtcli data writer-affinity <key> <core>
```

### 3. 상태 점검 (status_commands.h/cpp)

```bash
./rtcli status framework
./rtcli status tasks
./rtcli status memory
```

### 4. 추적/디버깅 (tracing_commands.h/cpp)

```bash
./rtcli tracing enable <key>
./rtcli tracing disable <key>
./rtcli tracing dump [--format json|csv]
```

## 새로운 커맨드 추가 방법

### 1단계: 핸들러 클래스 구현

`my_commands.h`:
```cpp
class MyCommandHandler : public rtcli::CommandHandler {
public:
    int execute(const std::vector<std::string>& args, 
                rtcli::SharedMemoryContext& ctx) override;
    std::string name() const override { return "mycommand"; }
    std::string help() const override;
};
```

`my_commands.cpp`:
```cpp
int MyCommandHandler::execute(const std::vector<std::string>& args, 
                               rtcli::SharedMemoryContext& ctx) {
    // 구현
    return 0;
}

std::string MyCommandHandler::help() const {
    return "mycommand - Description\n  subcommand - Help text\n";
}
```

### 2단계: main.cpp에 등록

```cpp
dispatcher.register_handler(std::make_unique<rtcli::MyCommandHandler>());
```

### 3단계: CMakeLists.txt 업데이트

```cmake
add_executable(rtcli
    ...
    rtcli/my_commands.cpp
)
```

## 컴파일 및 실행

### 빌드
```bash
cd /media/pms/DATA/project/rtfw_project_purism/build
cmake ..
cmake --build . --target rtcli
```

### 바이너리 위치
```
/media/pms/DATA/project/rtfw_project_purism/build/tools/rtcli
```

## 공유 메모리 인터페이스

`rtcli`는 Framework와 다음을 통해 통신합니다:

- **SHM 이름**: `SHM_NAME` (rtfw_common/shm_layout.h에서 정의)
- **제어 방식**: `SharedMemoryHeader.requested_action` (atomic)
- **데이터 전달**: 파일명 및 설정은 SHM 헤더의 문자열 필드를 통해 전달

## 에러 처리

각 커맨드는 다음 에러 코드를 반환합니다:

- `0`: 성공
- `1`: 잘못된 인자
- `2`: SHM 연결 실패 또는 파일 오류
- `3~16`: 레코드 파일 검증 중 특정 오류

## 노트

- **check_record**는 SHM 연결이 필요 없으므로, Framework가 실행 중이지 않아도 동작합니다.
- 다른 모든 커맨드는 Framework가 실행 중이어야 합니다.
- Framework의 action 처리는 비동기이므로 충분한 대기 시간을 두고 상태를 확인해야 합니다.
