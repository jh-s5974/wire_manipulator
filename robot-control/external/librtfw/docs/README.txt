RTFW: 실시간 로봇 제어를 위한 C++ 프레임워크 (문서 초안)
1. 프로젝트 개요 (README.md)

**RTFW(Real-Time Framework)**는 복잡한 데이터 흐름을 가진 고성능 실시간 제어 시스템(예: 휴머노이드 로봇)을 위해 설계된 C++ 프레임워크입니다. 이 프레임워크는 개발자가 개별 제어 로직(Task) 구현에만 집중할 수 있도록, 멀티스레딩, 데이터 동기화, 실시간 스케줄링의 복잡성을 자동화하고 추상화합니다.
1.1. 핵심 특징 및 철학

    자동화된 실행 순서 결정: "실행은 데이터를 따른다"는 원칙에 따라, 태스크 간의 데이터 의존성을 분석하여 실행 순서를 자동으로 결정합니다.

    사이클 없는 단방향 데이터 흐름: "단일 작성자 원칙"을 통해 데이터 흐름을 항상 비순환 방향 그래프(DAG)로 보장하여 시스템의 예측 가능성을 극대화합니다.

    결정론적 실시간 실행: 모든 태스크와 의존성은 시작 시점에 정적으로 분석되어, 런타임 오버헤드를 최소화하고 매 주기(tick)가 예측 가능하게 동작하도록 보장합니다.

    안전한 다중 주파수 동기화: 참조 카운팅 기반의 "N-Way 버퍼링"을 통해 각 타임라인이 독립적이고 일관된 데이터 '스냅샷'을 유지하며, 데드라인에 영향을 주지 않고 안전하게 데이터를 교환합니다.

    완벽한 재현성 및 분석: 시스템의 모든 외부 입력만 기록하면 전체 동작을 완벽하게 재현(--simul 모드)할 수 있으며, json_exporter를 통해 로그를 Python/MATLAB에서 분석할 수 있습니다.

    높은 관찰 가능성: 공유 메모리를 통해 시스템의 모든 상태와 통계 정보를 실시간으로 시각화(monitor_tool)하고 제어할 수 있습니다.

1.2. 프로젝트 구조

    rtfw-common: 공유 메모리 레이아웃, 파일 포맷, 공용 데이터 구조 등 프레임워크의 모든 구성 요소가 공유하는 핵심 정의를 포함합니다.

    rtfw: 스케줄러, 타임라인, 태스크 관리, 스레드 풀, 공유 메모리 구축 등 프레임워크의 핵심 로직을 담당하는 엔진입니다.

    rtfw-client: 외부 애플리케이션이 실행 중인 프레임워크의 공유 메모리에 안전하게 접근하여 데이터를 조회(SharedMemoryQuerier)하거나 시스템을 제어(SharedMemoryController)하기 위한 API 라이브러리입니다.

    apps: 프레임워크를 사용하는 예제 애플리케이션(poc_app), 실시간 모니터링 GUI(monitor_tool), 오프라인 로그 뷰어(log_viewer), 데이터 변환 유틸리티(json_exporter)를 포함합니다.

2. 핵심 개념 상세 (CONCEPTS.md)
2.1. 데이터 흐름이 실행을 지배한다

RTFW의 가장 중요한 설계 원칙은 **"실행 의존성은 데이터 의존성과 동일하다"**는 것입니다.

    Strong 의존성: 같은 주기 내에서의 순차 실행을 보장합니다. DataReader는 DataWriter가 이번 주기에 실행을 마칠 때까지 대기합니다. 이는 데이터의 신선도가 매우 중요한 동일 타임라인 내 RT 태스크 간 통신에 사용됩니다.

    Weak 의존성: 실행 순서를 강제하지 않고 현재 접근 가능한 최신 데이터를 사용합니다. 이는 RT 안정성을 해치지 않으면서 시스템의 유연성을 극대화하는 핵심 메커니즘으로, 다음 세 가지 주요 문제를 해결합니다.

        다른 주파수 간 동기화: 고주파 타임라인이 저주파 타임라인의 실행을 기다리지 않도록 합니다.

        RT-NonRT 간 안전한 데이터 교환: Non-RT 태스크의 예측 불가능한 실행 시간이 RT 태스크의 데드라인에 영향을 주지 않도록 격리합니다.

        외부 시스템과의 순환 참조 해결: 물리적 피드백 루프를 논리적으로 단절시켜 의존성 그래프의 사이클을 방지합니다.

    단일 작성자 원칙 (Single-Writer Principle): 하나의 데이터 키(e.g., "sensors.high_freq")는 시스템 전체에서 **단 하나의 DataWriter**만 가질 수 있습니다. 이 규칙은 데이터 흐름을 항상 단방향으로 만들어 의존성 그래프를 비순환 방향 그래프(DAG)로 보장하며, 사이클 발생을 원천적으로 차단합니다.

2.2. 타임라인 격리를 위한 N-Way 버퍼링

프레임워크는 단순한 더블 버퍼링을 넘어, 참조 카운팅 기반의 다중 버퍼링(N-Way Buffering) 메커니즘을 사용합니다. 버퍼의 개수는 (타임라인 수 + Non-RT 태스크 그룹 + 여유분) 만큼 동적으로 할당됩니다.

    캡처(Capture): 타임라인이 시작될 때, 읽어야 할 외부 데이터가 저장된 버퍼의 참조 카운트를 증가시켜 "읽기 잠금"을 겁니다.

    격리된 실행: 타임라인은 실행되는 동안 이 고정된 '스냅샷' 데이터만 읽습니다. 이 시간 동안 다른 타임라인이 해당 데이터를 갱신하더라도, 현재 타임라인은 전혀 영향을 받지 않아 데이터 일관성이 보장됩니다.

    해제 및 커밋(Release & Commit): 타임라인의 모든 작업이 끝나면, 캡처했던 버퍼의 참조 카운트를 감소시키고, 자신이 생성한 새로운 데이터를 다른 타임라인이 사용할 수 있도록 커밋합니다.

2.3. 함수형 패러다임과 재현성

RT 태스크는 외부 상태 변화 없이, 주어진 입력에 대해서만 동일한 출력을 내는 **순수 함수(pure function)**처럼 동작하는 것을 권장합니다.

    최소 로깅: 이 원칙 덕분에, 시스템 전체를 재현하기 위해 모든 중간 데이터를 기록할 필요가 없습니다. 오직 모든 외부 입력(모든 시간의 센서 데이터, Non-RT 태스크의 출력 등)만 기록하면 충분합니다.

    완벽한 재현 (--simul 모드): 기록된 외부 입력 데이터만으로 프레임워크는 모든 RT 태스크의 실행 결과를 매 주기마다 완벽하게 재구성할 수 있습니다.

    스냅샷 기능의 부재: 현재는 특정 시점부터 재현하는 스냅샷 기능은 지원하지 않습니다. 이는 프레임워크가 강제하지 않는 태스크 내부의 '히든 상태'까지 완벽하게 저장하고 복원하는 것의 복잡성 때문이며, 이는 향후 과제로 남아있습니다.

3. 시작하기 (TUTORIAL.md)

poc_app 예제를 바탕으로 새로운 태스크를 프레임워크에 추가하는 과정은 다음과 같습니다.
Step 1: 데이터 타입 정의

biped_data_types.h 와 같이 제어에 사용할 데이터 구조체를 정의합니다. 이 구조체들은 간단한 Aggregate 타입으로 정의하는 것을 권장합니다.
code C++
IGNORE_WHEN_COPYING_START
IGNORE_WHEN_COPYING_END

      
// biped_data_types.h
struct HighFreqSensors {
    double joint_positions[12];
    double joint_velocities[12];
};

    

Step 2: 태스크 클래스 구현

rtfw::rt::ITask를 상속받아 새로운 태스크 클래스를 만듭니다. setup()과 execute() 함수를 반드시 구현해야 합니다.
code C++
IGNORE_WHEN_COPYING_START
IGNORE_WHEN_COPYING_END

      
#include "rtfw/task.h"
#include "biped_data_types.h"

class SensorProducer : public rtfw::rt::ITask {
public:
    const char* getName() const override { return "SensorProducer"; }

    void setup(rtfw::rt::TaskRegistry& r) override;
    void execute() override;

private:
    rtfw::rt::DataWriter<HighFreqSensors> sensors_writer_{"sensors.high_freq"};
};

    

Step 3: 의존성 선언

setup() 함수 내에서 TaskRegistry를 통해 이 태스크가 읽거나 쓸 데이터를 선언합니다.
code C++
IGNORE_WHEN_COPYING_START
IGNORE_WHEN_COPYING_END

      
void SensorProducer::setup(rtfw::rt::TaskRegistry& r) {
    // 이 태스크는 "sensors.high_freq" 키에 HighFreqSensors 타입의 데이터를 쓸 것입니다.
    r.add_dependency(sensors_writer_);
}

    

Step 4: 로직 구현

execute() 함수 내에 실제 제어 로직을 구현합니다.
code C++
IGNORE_WHEN_COPYING_START
IGNORE_WHEN_COPYING_END

      
void SensorProducer::execute() {
    HighFreqSensors data;
    // ... 센서 데이터를 읽어와 data를 채우는 로직 ...
    sensors_writer_.write(data);
}

    

Step 5: 프레임워크에 등록

main.cpp에서 RealTimeFramework 인스턴스를 생성하고, 작성한 태스크를 원하는 주파수와 CPU 코어에 등록합니다.
code C++
IGNORE_WHEN_COPYING_START
IGNORE_WHEN_COPYING_END

      
// main.cpp
#include "rtfw/framework.h"
#include "my_sensor_producer.h"

int main() {
    rtfw::RealTimeFramework framework;
    // SensorProducer 태스크를 1000Hz 주파수로, CPU 7번 코어에 고정하여 등록
    framework.registerTask(std::make_unique<SensorProducer>(), 1000, 7);

    // ... 다른 태스크들도 등록 ...

    framework.initialize(rtfw::RealTimeFramework::Mode::LIVE);
    framework.start();
    framework.join();
    return 0;
}

    

4. 도구 생태계 (TOOLS.md)

    monitor_tool: ImGui 기반의 실시간 모니터링 GUI입니다. 타임라인 및 스레드 풀의 부하, 태스크별 실행 통계, 실시간 로그, 그리고 태스크 간의 의존성 그래프를 시각적으로 보여줍니다.

    log_viewer: --record 모드로 생성된 .rttrace 파일을 열어, 각 tick 별로 모든 데이터의 내용을 헥스 뷰어로 상세히 분석할 수 있는 오프라인 툴입니다.

    json_exporter: .rttrace 파일을 JSON 형식으로 변환하는 유틸리티입니다. 이를 통해 Python(Jupyter, pandas)이나 MATLAB 등 외부 분석 도구에서 데이터를 손쉽게 불러와 그래프를 그리거나 복잡한 후처리를 수행할 수 있습니다.

5. 향후 계획 (ROADMAP.md)

    파라미터 관리 시스템: DataReader/Writer와는 별도로, JSON/YAML 파일에서 설정을 불러오고 rtfw-client를 통해 실시간으로 튜닝할 수 있는 중앙 집중형 파라미터 관리 시스템을 도입할 계획입니다.

    프로파일링 기능 강화: PROFILE_SCOPE("name") 매크로를 도입하여, 태스크 내부의 특정 코드 블록 실행 시간을 정밀하게 측정하고 시각화하는 기능을 추가할 예정입니다.
