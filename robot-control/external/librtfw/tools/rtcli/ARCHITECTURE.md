# rt_cli Architecture and Design Guide

## 개요

`rt_cli`는 Real-Time Framework와 상호작용하는 CLI 도구로, 확장 가능한 플러그인 아키텍처를 사용합니다.

## 핵심 컴포넌트

### 1. SharedMemoryContext (shm_common.h/cpp)

**목적**: Framework와의 IPC 채널 제공

**특징**:
- RAII 패턴 사용으로 자동 리소스 정리
- 단일 책임: SHM 연결/해제만 담당
- 예외 안전성

**사용 예시**:
```cpp
rtcli::SharedMemoryContext ctx;
if (ctx.is_valid()) {
    auto header = ctx.header();
    // header를 통해 Framework와 통신
}
// 소멸자에서 자동으로 정리
```

### 2. CommandHandler (command_handler.h/cpp)

**목적**: 모든 커맨드 핸들러의 기본 인터페이스

**인터페이스**:
```cpp
class CommandHandler {
    virtual int execute(const std::vector<std::string>& args, 
                       SharedMemoryContext& ctx) = 0;
    virtual std::string name() const = 0;      // 커맨드 이름
    virtual std::string help() const = 0;      // 도움말 텍스트
};
```

**설계 철학**:
- 단일 책임 원칙: 각 핸들러는 하나의 커맨드 그룹만 담당
- 느슨한 결합: CommandDispatcher를 통해서만 통신
- 확장성: 새로운 커맨드 추가는 새로운 핸들러 클래스만 필요

### 3. CommandDispatcher (command_handler.h/cpp)

**목적**: 커맨드 라우팅 및 핸들러 관리

**기능**:
- 핸들러 등록: `register_handler()`
- 커맨드 디스패치: `dispatch(args, ctx)`
- 도움말 출력: `print_help()`

**구조**:
```cpp
std::unordered_map<std::string, std::unique_ptr<CommandHandler>> handlers_;
```

## 모듈 구조

### Scheduler Commands (scheduler_commands.h/cpp)

Framework의 tick 처리를 제어합니다.

**커맨드**:
- `pause`: tick 처리 일시 중지
- `play`: tick 처리 재개
- `play_one`: 한 틱 전진

**구현 방식**:
1. `SchedulerCommandHandler::execute()` 분석
2. 서브커맨드별 private 메서드 호출
3. 각 메서드는 `header()->requested_action` 설정

### Record Commands (record_commands.h/cpp)

레코딩, 리플레이, 트레이스 및 파일 검증을 담당합니다.

**핸들러**:
1. **RecordCommandHandler**: 레코딩 제어
   - `start <path>`: 레코딩 시작
   - `stop`: 레코딩 중지

2. **ReplayCommandHandler**: 리플레이 제어
   - `start <path>`: 리플레이 시작
   - `stop`: 리플레이 중지

3. **TraceCommandHandler**: 트레이스 (동시 리플레이/레코딩)
   - `start <replay_path> <record_path>`: 트레이스 시작
   - `stop`: 트레이스 중지

4. **CheckRecordCommandHandler**: 레코드 파일 검증
   - `check_record <path>`: 파일 검증
   - `check_record <path> --show-hash`: 해시값 포함 표시

**특수 처리**:
- `CheckRecordCommandHandler`는 SHM 연결이 필요 없음
- `main.cpp`에서 특수 처리: check_record 첫 번째 커맨드 시 SHM 스킵

### Parameters Commands (parameters_commands.h/cpp)

파라미터 관리 기능 (향후 구현)

**예약된 커맨드**:
- `load <path>`: YAML 파일에서 파라미터 로드
- `reload`: 현재 파라미터 재로드
- `list`: 모든 파라미터 목록
- `get <key>`: 파라미터 값 조회
- `set <key> <value>`: 파라미터 값 설정

**구현 상태**: TODO (Framework API 추가 대기)

## 데이터 흐름

### 일반 커맨드 (scheduler, record, replay, trace)

```
Command Line Arguments
        ↓
    main.cpp
        ↓
  SHM 연결 확인
        ↓
CommandDispatcher::dispatch()
        ↓
    알맞은 핸들러 선택
        ↓
CommandHandler::execute()
        ↓
  SharedMemoryHeader 조작
        ↓
  Framework에 액션 전달
```

### check_record 커맨드

```
Command Line Arguments
        ↓
    main.cpp
        ↓
   SHM 스킵 (필요 없음)
        ↓
CommandDispatcher::dispatch()
        ↓
CheckRecordCommandHandler::execute()
        ↓
  파일 시스템 직접 접근
        ↓
  파일 검증 및 분석
```

## 확장 패턴

### 새로운 커맨드 추가 체크리스트

1. **핸들러 클래스 생성** (`my_command.h`)
   ```cpp
   class MyCommandHandler : public rtcli::CommandHandler {
       int execute(...) override;
       std::string name() const override { return "mycommand"; }
       std::string help() const override;
   };
   ```

2. **구현 파일** (`my_command.cpp`)
   - `execute()`: 주요 로직
   - 서브커맨드별 private 메서드
   - `help()`: 사용자 친화적 도움말

3. **CMakeLists.txt 업데이트**
   - `add_executable(rt_cli ... rt_cli/my_command.cpp)`

4. **main.cpp 업데이트**
   - 헤더 포함: `#include "my_command.h"`
   - 핸들러 등록: `dispatcher.register_handler(...)`

5. **테스트**
   - 빌드: `cmake --build . --target rt_cli`
   - 실행: `./rt_cli mycommand --help`

### 에러 처리 패턴

모든 핸들러는 다음 규칙을 따릅니다:

```cpp
int MyHandler::execute(...) {
    // 1. 입력 검증
    if (!ctx.is_valid()) {
        std::cerr << "Error: Invalid context" << std::endl;
        return 2;
    }

    // 2. 인자 분석
    if (args.empty()) {
        std::cerr << "mycommand: missing subcommand" << std::endl;
        std::cout << help();
        return 1;
    }

    // 3. 서브커맨드 디스패치
    const auto& cmd = args[0];
    if (cmd == "subcmd") {
        return handle_subcmd(/* args */);
    }

    // 4. 알 수 없는 커맨드
    std::cerr << "mycommand: unknown subcommand '" << cmd << "'" << std::endl;
    std::cout << help();
    return 1;
}
```

**반환값 규칙**:
- `0`: 성공
- `1`: 사용자 오류 (잘못된 인자, 알 수 없는 서브커맨드)
- `2`: 시스템 오류 (SHM 연결 실패, 파일 오류)
- `3+`: 특수 오류 (파일 검증 오류 등)

## SHM 인터페이스

### 주요 필드 (SharedMemoryHeader)

```cpp
// 액션 요청
std::atomic<FrameworkAction> requested_action;

// 파일 경로
char target_filename[256];           // 레코딩/트레이스 출력
char replay_target_filename[256];    // 리플레이/트레이스 입력
```

### FrameworkAction 열거형

```cpp
enum class FrameworkAction {
    NONE = 0,
    PAUSE_TICK,
    PLAY_TICK,
    PLAY_ONE_TICK,
    START_RECORD,
    STOP_RECORD,
    START_REPLAY,
    STOP_REPLAY,
    START_TRACE,
    STOP_TRACE,
    // 향후 추가 가능
};
```

## 메모리 안전성

### RAII 패턴 사용

```cpp
{
    SharedMemoryContext ctx;  // 생성자: 리소스 획득
    // 사용
    // 스코프 벗어남: 소멸자가 자동으로 정리
}
```

### 벡터 사용

```cpp
std::vector<std::string> args;  // 자동 메모리 관리
```

### unique_ptr 사용

```cpp
std::unique_ptr<CommandHandler> handler;  // 자동 정리
```

## 테스트 방법

### 단위 테스트 (개별 핸들러)

Framework가 실행 중이 아닐 때는 check_record만 테스트 가능:

```bash
./rt_cli check_record /path/to/record.rtrec
./rt_cli check_record /path/to/record.rtrec --show-hash
```

### 통합 테스트 (Framework 실행 필요)

```bash
# Framework 시작
./apps/poc_app/poc_app &

# CLI 커맨드 실행
./tools/rt_cli scheduler pause
./tools/rt_cli record start /tmp/test.rtrec
./tools/rt_cli scheduler play
sleep 5
./tools/rt_cli scheduler pause
./tools/rt_cli record stop

# 레코드 검증
./tools/rt_cli check_record /tmp/test.rtrec
```

## 성능 고려사항

1. **SHM 연결**: 한 번만 수행 (main에서)
2. **메모리 할당**: 필요할 때만 (check_record의 버퍼 resize)
3. **파일 I/O**: 순차 읽기 (check_record)
4. **동작 신호**: atomic 연산 (고속, lock-free)

## 향후 개선 계획

### 단기 (1-2주)

1. Parameters 명령어 구현
2. Data Reader/Writer 제어 명령어
3. Status 명령어 (Framework 상태 조회)

### 중기 (1-2개월)

1. 대화형 CLI 모드 (readline 통합)
2. 명령어 자동 완성
3. 로그 스트리밍 기능

### 장기

1. gRPC 인터페이스 추가
2. REST API 제공
3. GUI 대시보드
