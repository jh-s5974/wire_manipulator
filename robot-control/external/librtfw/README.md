# RTFW — Real-Time Framework

고성능 C++ 실시간 제어 프레임워크. 데이터 주도(DAG) 스케줄링, N중 버퍼링, 공유 메모리 관측, 블랙박스 녹화/재생을 지원합니다.

---

## 목차

- [특징](#특징)
- [프로젝트 구조](#프로젝트-구조)
- [빌드 방법](#빌드-방법)
- [빠른 시작](#빠른-시작)
- [rtcli 사용법](#rtcli-사용법)
- [모니터링 도구](#모니터링-도구)
- [예제 앱](#예제-앱)
- [문서](#문서)

---

## 특징

| 특징 | 설명 |
|------|------|
| **데이터 주도 스케줄링** | 태스크 간 데이터 의존성을 DAG로 분석하여 실행 순서를 자동 결정 |
| **Single-Writer 원칙** | 각 데이터 키에 정확히 한 개의 `DataWriter`만 허용 (데이터 일관성 보장) |
| **N중 버퍼링** | 타임라인 격리를 위한 참조 카운트 기반 버퍼 (`#타임라인 + #NonRT그룹 + slack`) |
| **강한/약한 의존성** | Strong=동기 대기, Weak=최신 데이터 비블록 사용 |
| **RT/NonRT 혼합** | CPU 어피니티 지정 RT 태스크 + NonRT 태스크 동시 운용 |
| **블랙박스 녹화/재생** | `--simul` 모드로 완벽 재현; 체크포인트로 상태 복원 |
| **공유 메모리 관측** | POSIX SHM `/rt_framework_shm`으로 외부 도구 실시간 접근 |
| **ITask / Task<S>** | 상태 없으면 `ITask`, 상태 필요시 `Task<StateT>` (스냅샷/체크포인트 지원) |

---

## 프로젝트 구조

```
rtfw_project_purism/
├── rtfw/               # 프레임워크 코어 엔진
│   ├── include/rtfw/   # 공개 헤더 (rt_framework.h, task.h, ...)
│   └── src/            # 구현부 (rt_framework.cpp 등)
├── rtfw-common/        # 공유 타입, SHM 레이아웃, 파일 포맷 정의
│   └── include/rtfw_common/
├── rtfw-connect/       # 외부 클라이언트 API (SHM 연결, 쿼리, 제어)
│   └── include/rtfw_connect/
├── tools/
│   └── rtcli/          # rtcli: 커맨드라인 제어 도구
├── apps/
│   ├── poc_app/        # 참조 예제 앱 (이족보행 로봇 시뮬)
│   ├── rtmonitor/      # ImGui 실시간 모니터 GUI
│   ├── rtviewer/       # SHM 메모리 뷰어 (hex editor)
│   └── task_onoff_test/# 태스크 ON/OFF 테스트
└── build/              # CMake 빌드 출력
```

---

## 빌드 방법

### 의존성

- **C++17** 이상 컴파일러 (GCC 9+ 권장)
- **CMake** 3.16+
- **Boost** (헤더 전용)
- **spdlog** (비동기 로거)
- POSIX 라이브러리: `pthread`, `rt`

### 빌드

```bash
# 처음 빌드
mkdir -p build && cd build
cmake ..
cmake --build .

# 특정 타겟만
cmake --build . --target poc_app
cmake --build . --target rtcli
cmake --build . --target rtmonitor
```

### 빌드 타겟 목록

| 타겟 | 위치 | 설명 |
|------|------|------|
| `poc_app` | `build/apps/poc_app/` | 참조 예제 앱 |
| `rtcli` | `build/tools/rtcli/` | CLI 제어 도구 |
| `rtmonitor` | `build/apps/rtmonitor/` | 실시간 모니터 |
| `rtviewer` | `build/apps/rtviewer/` | SHM 뷰어 |

---

## 빠른 시작

### 1. poc_app 실행

```bash
# 실시간 커널 우선순위 설정 (선택)
sudo ./disable_cpuidle.sh
sudo ./set_active_cores.sh

# 앱 실행
./build/apps/poc_app/poc_app
```

### 2. 백그라운드 실행 + rtcli 제어

```bash
# 백그라운드 실행
./build/apps/poc_app/poc_app &

# 스케줄러 제어
./build/tools/rtcli sched pause     # 일시정지
./build/tools/rtcli sched play      # 재개
./build/tools/rtcli sched play_one  # 1틱 실행

# 녹화
./build/tools/rtcli record start /tmp/my_record.rtrec
./build/tools/rtcli record stop

# 녹화 파일 검증
./build/tools/rtcli check_record /tmp/my_record.rtrec

# 재생 (프레임워크 SIMULATION 모드에서)
./build/tools/rtcli replay start /tmp/my_record.rtrec
```

### 3. 파라미터 파일

```bash
# robot_param.yaml 또는 param_test.yaml 사용
./build/apps/poc_app/poc_app  # config에 parameter_file_path 지정 필요
```

---

## rtcli 사용법

```
사용법: rtcli <커맨드> [옵션]
```

### 스케줄러

| 명령 | 설명 |
|------|------|
| `rtcli sched pause` | 프레임워크 틱 일시정지 |
| `rtcli sched play` | 프레임워크 틱 재개 |
| `rtcli sched play_one` | 단일 틱 실행 후 자동 일시정지 |

### 녹화

| 명령 | 설명 |
|------|------|
| `rtcli record start <경로>` | 지정 파일에 녹화 시작 |
| `rtcli record stop` | 녹화 중지 |
| `rtcli check_record <파일>` | 녹화 파일 무결성 검증 |

### 재생

| 명령 | 설명 |
|------|------|
| `rtcli replay start <파일>` | 파일 재생 시작 |
| `rtcli replay stop` | 재생 중지 |

### 태스크 제어

| 명령 | 설명 |
|------|------|
| `rtcli task list` | 등록된 모든 태스크 목록 |
| `rtcli task info <id>` | 태스크 상세 정보 |
| `rtcli task enable <id>` | 태스크 활성화 |
| `rtcli task disable <id>` | 태스크 비활성화 |

---

## 모니터링 도구

### rtmonitor (ImGui GUI)

```bash
./build/apps/rtmonitor/rtmonitor
```

- 실시간 태스크 실행 통계 (실행시간, 지연, 오버런)
- 데이터 플로우 그래프 시각화
- 타임라인/스레드 풀 사용률

### rtviewer (SHM 메모리 뷰어)

```bash
./build/apps/rtviewer/rtviewer
```

- SHM 내 데이터 버퍼 실시간 hex 뷰

### JSON 데이터 분석

```bash
# 녹화 파일 → JSON 변환 (crane_4wis, mobile_crane 앱 내장)
# Python으로 분석
python3 json_plot.py        # 기본 플롯
python3 json_plot_paper.py  # 논문용 플롯
```

---

## 예제 앱

### poc_app (이족보행 로봇 POC)

8개 태스크로 구성된 참조 구현:

| 태스크 | 주파수 | 타입 | 역할 |
|--------|--------|------|------|
| `SensorProducer` | 1000Hz | RT (Core 7) | IMU, 관절 센서 데이터 생성 |
| `TorqueController` | 1000Hz | RT (Core 7) | 토크 명령 계산 |
| `StateEstimator` | 200Hz | RT (Core 6) | CoM, ZMP 추정 |
| `VisionProcessor` | 30Hz | RT (Core 5) | 비전 객체 탐지 |
| `GaitPlanner` | 30Hz | RT | 보행 궤적 계획 |
| `StatefulCounterTask` | 10Hz | RT | 상태 보존 카운터 (체크포인트 테스트) |
| `ParameterTuner` | 1Hz | NonRT | 런타임 파라미터 조정 |
| `DataLogger` | 10Hz | NonRT | 데이터 로깅 |

---

## 문서

### 메인 문서

| 문서 | 내용 |
|------|------|
| [MANUAL.md](docs/MANUAL.md) | 개발자 매뉴얼 (태스크 작성, API 레퍼런스, 트러블슈팅) |
| [README.txt](docs/README.txt) | 원본 설계 노트 (한국어, 개념 설명) |
| [PROJECT_DOCUMENTATION.md](docs/PROJECT_DOCUMENTATION.md) | RTFW 프로젝트 전체 아키텍처 (한국어 상세 가이드) |
| [QUICK_REFERENCE.md](docs/QUICK_REFERENCE.md) | rtcli 빠른 참조 가이드 |

### rtcli 도구 문서

| 문서 | 내용 |
|------|------|
| [README.md](tools/rtcli/README.md) | rtcli 사용자 가이드 |
| [ARCHITECTURE.md](tools/rtcli/ARCHITECTURE.md) | rtcli 아키텍처 및 설계 문서 |
| [PROJECT_STRUCTURE.md](tools/rtcli/PROJECT_STRUCTURE.md) | rtcli 프로젝트 구조 |
| [EXTENSION_TEMPLATE.hpp](tools/rtcli/EXTENSION_TEMPLATE.hpp) | 새 커맨드 추가 템플릿 |

---

## 알려진 이슈

- **Trace 모드**: CLI 구현됨, 프레임워크 연동 미완
- **파라미터 CLI**: `rtcli param` 커맨드는 스켈레톤 구현
- **Upstream Overrun 감지**: 라이터가 버퍼 슬롯보다 빠른 경우 감지 로직 미구현

---

## 라이선스

내부 프로젝트 — 별도 라이선스 없음.
