# Task On/Off Feature Test

이 디렉토리는 새로 추가된 **Task Enable/Disable 기능**을 테스트하는 샘플 프로그램입니다.

## 개요

Task on/off 기능은 실행 중 특정 태스크를 동적으로 활성화/비활성화하고, 강한 의존성이 있는 downstream 태스크들에 자동으로 전파합니다.

## 핵심 기능

### 1. 태스크 활성화 상태 관리
```cpp
task->setEnabled(bool enabled);        // 태스크 on/off 설정
bool is_enabled = task->isEnabled();   // 현재 상태 조회
```

### 2. Tick 실행 상태 추적
```cpp
task->resetTickState();              // Tick 시작 시 상태 초기화
bool should_run = task->shouldRun(); // 이번 tick에 실행될 예정인지
bool ready = task->becameReady();    // 의존성 만족되어 ready 상태가 된 시점
bool executed = task->wasExecuted(); // 실제 실행되었는지
```

### 3. 실행 상태 분류 (ExecState)
```cpp
enum class ExecState : uint8_t {
    Executed = 0,           // 정상 실행됨
    DeadlineMissSelf,       // 자기 실행이 tick을 넘김
    DeadlineMissUpstream,   // 의존성 때문에 ready가 안 됨
    NotScheduled            // enabled=false로 실행 대상 아님
};
```

## 프로그램 구조

### 태스크들
- **ProducerTask**: 데이터를 생성하는 기본 태스크
- **Consumer1Task**: Producer의 output을 읽음 (Strong dependency)
- **Consumer2Task**: Producer의 output을 읽음 (Strong dependency, 별도)
- **DownstreamTask**: 향후 extended test용 (미구현)

### 실행 흐름
1. 모든 태스크 정상 실행
2. (향후) 특정 시점에 Consumer1 disabled
3. Consumer1의 downstream 자동 disabled
4. Consumer2는 별도 의존성이므로 계속 실행

## 빌드 및 실행

```bash
# 빌드
cmake --build build --target task_onoff_test -j$(nproc)

# 실행 (10초)
./build/apps/task_onoff_test/task_onoff_test

# 로그 저장 후 분석
./build/apps/task_onoff_test/task_onoff_test > test_output.log 2>&1
python3 apps/task_onoff_test/analyze_test.py test_output.log
```

## 예상 출력

```
=== Task On/Off Feature Test ===

✓ Memory locked in RAM for deterministic RT performance

=== Framework initialized ===
Running test scenarios...
This will run for 10 seconds with enable/disable toggles.

[Producer] Tick 0: counter=0, value=0.00
[Consumer1] Tick 0: Reading data
[Consumer2] Tick 0: Reading data
...
```

## 통계 확인

프로그램 종료 후 `printStats()`가 각 태스크의 실행 통계를 출력합니다:
- 실행 횟수
- 평균/최대 실행 시간
- 평균/최대 스케줄링 지연시간

## 향후 확장

1. **Dynamic Control API**: 실행 중 enable/disable 토글 가능하게 개선
2. **ExecState 통계**: tick_end()에서 각 ExecState별 카운트 수집
3. **Visualization**: 시간에 따른 태스크 상태 변화 시각화
4. **Stress Test**: 복잡한 의존성 그래프에서 enable/disable 동작 검증

## 주요 구현 포인트

### Timeline::applyEnableMaskAndCascade()
```cpp
// enabled=false인 태스크부터 시작하여 BFS로 downstream 순회
// should_run=false 자동 전파
// Strong dependency를 통해서만 전파 (Weak dependency는 영향 없음)
```

### Timeline::tick_start()
```cpp
// 1. resetTickState() - 모든 태스크 상태 초기화
// 2. applyEnableMaskAndCascade() - enable 마스크 적용
// 3. 의존성 카운터 리셋 후 ready 큐에 태스크 추가
//    (should_run==true && remaining_deps==0만 추가)
```

### Timeline::tick_end()
```cpp
// ExecState 분류:
// - NotScheduled: should_run=false
// - Executed: executed=true
// - DeadlineMissSelf: became_ready=true이지만 executed=false
// - DeadlineMissUpstream: should_run=true이지만 became_ready=false
```

## 트러블슈팅

### Memory lock 실패
```bash
# mlockall 권한 부여
sudo setcap cap_ipc_lock=ep ./build/apps/task_onoff_test/task_onoff_test

# 또는 memlock limit 증가
ulimit -l unlimited
```

### 실시간 스케줄러 실패
프로그램은 soft realtime으로 자동 fallback됩니다. hard realtime 필요 시:
```bash
sudo ./build/apps/task_onoff_test/task_onoff_test
```

## 참고자료

- [Task 클래스](../../rtfw/include/rtfw/task.h)
- [Timeline 클래스](../../rtfw/include/rtfw/timeline.h)
- [Framework 클래스](../../rtfw/include/rtfw/rt_framework.h)
