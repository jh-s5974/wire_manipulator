# rt_cli 프로젝트 구조 요약

## 디렉토리 및 파일 구조

```
tools/
├── CMakeLists.txt                 # 빌드 설정 (rt_cli 타겟 정의)
└── rt_cli/
    ├── main.cpp                   # 엔트리 포인트 (57줄)
    │   └── 역할: CLI 초기화, 커맨드 등록, 디스패치
    │
    ├── shm_common.h/cpp           # 공유 메모리 컨텍스트 (RAII)
    │   └── 역할: Framework SHM 연결 관리
    │
    ├── command_handler.h/cpp      # 기본 인터페이스 및 디스패처
    │   ├── CommandHandler (추상 클래스)
    │   └── CommandDispatcher (라우터)
    │
    ├── scheduler_commands.h/cpp  # 스케줄러 제어
    │   └── SchedulerCommandHandler
    │       ├── pause
    │       ├── play
    │       └── play_one
    │
    ├── record_commands.h/cpp     # 레코딩/리플레이 (4개 핸들러)
    │   ├── RecordCommandHandler
    │   │   ├── start <path>
    │   │   └── stop
    │   ├── ReplayCommandHandler
    │   │   ├── start <path>
    │   │   └── stop
    │   ├── TraceCommandHandler
    │   │   ├── start <replay> <record>
    │   │   └── stop
    │   └── CheckRecordCommandHandler
    │       ├── validate_record_file()
    │       └── [--show-hash] option
    │
    ├── parameters_commands.h/cpp # 파라미터 관리 (예약됨, 미구현)
    │   └── ParametersCommandHandler (TODO)
    │
    ├── README.md                  # 사용자 가이드
    ├── ARCHITECTURE.md            # 아키텍처 및 개발 가이드
    ├── EXTENSION_TEMPLATE.hpp     # 확장 예제
    └── PROJECT_STRUCTURE.md       # 이 파일
```

## 빌드 시스템

```
CMakeLists.txt (tools/)
├── add_executable(rt_cli ...)
├── target_include_directories()
│   ├── rtfw-common/include
│   ├── rtfw/include
│   └── tools/rt_cli
├── target_link_libraries(rt pthread)
└── set_target_properties(RUNTIME_OUTPUT_DIRECTORY)
```

**바이너리 위치**: `build/tools/rt_cli`

## 런타임 커맨드 구조

```
rt_cli
├── scheduler <subcommand>
│   ├── pause
│   ├── play
│   └── play_one
├── record <subcommand>
│   ├── start <path>
│   └── stop
├── replay <subcommand>
│   ├── start <path>
│   └── stop
├── trace <subcommand>
│   ├── start <replay_path> <record_path>
│   └── stop
├── check_record <path> [--show-hash]
└── parameters <subcommand> (TODO)
    ├── load
    ├── reload
    ├── list
    ├── get
    └── set
```

## 클래스 계층 구조

```
CommandHandler (abstract)
├── SchedulerCommandHandler
├── RecordCommandHandler
├── ReplayCommandHandler
├── TraceCommandHandler
├── CheckRecordCommandHandler
└── ParametersCommandHandler (TODO)

SharedMemoryContext
├── SHM 열기/닫기 (RAII)
└── Framework 헤더 접근

CommandDispatcher
├── 핸들러 등록 (unordered_map)
├── 커맨드 라우팅
└── 도움말 출력
```

## 데이터 흐름 (시퀀스 다이어그램)

### 일반 커맨드 (scheduler 예시)

```
사용자 입력
    │
    v
┌─────────────────────────┐
│  main.cpp               │
│  - argc/argv 파싱       │
│  - SHM 연결            │
└──────────┬──────────────┘
           │
           v
┌─────────────────────────┐
│  CommandDispatcher      │
│  .dispatch(args, ctx)   │
└──────────┬──────────────┘
           │
           v
┌─────────────────────────────────────┐
│  SchedulerCommandHandler            │
│  .execute(["pause"], ctx)           │
└──────────┬──────────────────────────┘
           │
           v
┌─────────────────────────────────────────┐
│  handle_pause(ctx)                      │
│  ctx.header()->requested_action =       │
│      FrameworkAction::PAUSE_TICK        │
└──────────┬──────────────────────────────┘
           │
           v
    Framework가 
    SHM을 폴링하여
    액션 처리
```

### check_record 커맨드 (특수 경로)

```
사용자 입력
    │
    v
┌──────────────────────────┐
│  main.cpp                │
│  check_record 감지       │
│  SHM 스킵               │
└──────────┬───────────────┘
           │
           v
┌──────────────────────────┐
│  CommandDispatcher       │
│  .dispatch(args, ctx)    │
└──────────┬───────────────┘
           │
           v
┌──────────────────────────────────┐
│  CheckRecordCommandHandler       │
│  .validate_record_file(path)     │
└──────────┬───────────────────────┘
           │
           v
   파일 시스템 직접 접근
    파일 검증 및 분석
    결과 출력
```

## 의존성 관계

```
main.cpp
├── command_handler.h
├── scheduler_commands.h
├── record_commands.h
├── parameters_commands.h (향후)
└── shm_common.h

scheduler_commands.cpp
├── scheduler_commands.h
├── shm_common.h
└── rtfw_common/shm_layout.h

record_commands.cpp
├── record_commands.h
├── shm_common.h
├── rtfw_common/shm_layout.h
└── rtfw_common/log_format.h

command_handler.cpp
├── command_handler.h
└── (no external deps)

shm_common.cpp
├── shm_common.h
└── rtfw_common/shm_layout.h
```

## 코드 통계

| 파일 | 줄수 | 역할 |
|------|------|------|
| main.cpp | 57 | 엔트리 포인트 |
| command_handler.h/cpp | 55/38 | 인터페이스 및 라우터 |
| shm_common.h/cpp | 26/44 | SHM 관리 |
| scheduler_commands.h/cpp | 24/60 | 스케줄러 제어 |
| record_commands.h/cpp | 61/479 | 레코딩/검증 |
| parameters_commands.h/cpp | 38/65 | 파라미터 관리 (TODO) |
| **총합** | **~900** | - |

## 향후 확장 계획

### Phase 1: 파라미터 관리
- `parameters_commands.cpp` 구현
- YAML 파서 통합 (의존성 추가)
- SharedMemoryHeader 확장

### Phase 2: 데이터 제어
- `data_commands.h/cpp` 추가
- DataReader/Writer 속성 조회
- Key 모니터링

### Phase 3: 모니터링
- `status_commands.h/cpp` 추가
- `tracing_commands.h/cpp` 추가
- Real-time 통계 수집

### Phase 4: GUI/API
- gRPC 서버 통합
- REST API 추가
- 웹 대시보드

## 개발 체크리스트 (새 커맨드 추가)

- [ ] `xxx_commands.h` 작성 (핸들러 클래스)
- [ ] `xxx_commands.cpp` 작성 (구현)
- [ ] `CMakeLists.txt` 업데이트 (소스 파일 추가)
- [ ] `main.cpp` 업데이트 (헤더 포함, 핸들러 등록)
- [ ] 테스트 실행
- [ ] 도움말 업데이트 (README.md)

## 주요 설계 원칙

1. **단일 책임**: 각 핸들러는 하나의 커맨드 그룹만 담당
2. **개방-폐쇄 원칙**: 새 기능은 추가로만, 기존 코드는 수정 최소
3. **의존성 역전**: 구체적인 핸들러 ← 추상 인터페이스
4. **RAII**: 모든 리소스는 자동 정리
5. **에러 처리**: 일관된 반환 코드 시스템

## 빌드 및 배포

```bash
# 빌드
cd build
cmake ..
cmake --build . --target rt_cli

# 단독 실행 (check_record만 가능)
./tools/rt_cli check_record /path/to/file.rtrec

# Framework와 함께 실행
./apps/poc_app/poc_app &
./tools/rt_cli scheduler pause
./tools/rt_cli record start /tmp/test.rtrec
```

---

**작성일**: 2025-02-21
**버전**: 1.0 (구조화된 아키텍처)
