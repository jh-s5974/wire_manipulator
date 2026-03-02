// my_robot_exporter/main.cpp

// 1. 프레임워크가 제공하는 보일러플레이트를 가져옵니다.
// #include "frame.hpp"
#include <manif/manif.h>
#include "interfaces/CvResultData.h"
#include "interfaces/ImuData.h"
#include "rtfw_common/json_exporter.h"


// 2. 이 프로젝트에서만 사용하는 도메인 타입 정보를 가져옵니다.
// void to_json(json& j, const SE3& p) {
//     Eigen::Quaterniond quat(p.R);
//     std::array<double, 4> q{quat.w(), quat.x(), quat.y(), quat.z()};
//     j = json{{"position", p.T}, {"orient", q}};
// }
// void to_json(json& j, const se3& p) {
//     j = json{{"linear", p.linear}, {"angular", p.angular}};
// }

namespace Eigen {
    void to_json(json& j, const Eigen::Vector3d& p) {
        j = json{p.x(), p.y(), p.z()};
    }
    void to_json(json& j, const Eigen::Quaterniond& p) {
        j = json{p.w(), p.x(), p.y(), p.z()};
    }
};

namespace manif {
    void to_json(json& j, const manif::SE3d& p) {
        // const auto& quat = p.quat();
        // std::array<double, 4> q{quat.w(), quat.x(), quat.y(), quat.z()};
        j = json{{"position", p.translation()}, {"orient", p.quat()}};
    }
    void to_json(json& j, const manif::SE3Tangentd& p) {
        Eigen::Vector3d linear = p.lin();
        Eigen::Vector3d angular = p.ang();
        j = json{{"linear", linear}, {"angular", angular}};
    }

    void to_json(json& j, const manif::SE2d& p) {
        j = json{
            p.x(), p.y(), p.angle()
        };
    }
    void to_json(json& j, const manif::SE2Tangentd& p) {
        j = json{
            p.x(), p.y(), p.angle()
        };        
    }
};

namespace custom_types {
    void to_json(json& j, const custom_types::CvResultData& p) {
        j = json{
                    {"frame_id", p.frame_id},
                    {"fps", p.fps},
                    {"stamp", p.stamp.time_since_epoch().count()},
                    {"detect", p.detect},
                    {"marker", p.marker},
                };
    }

    void to_json(json& j, const custom_types::ImuData& p) {
        j = json{
                    {"orient", p.orient},
                    {"gyro", p.gyro},
                    {"accel", p.accel},
                };
    }
}

// 3. ExporterApp을 상속받아, '타입 등록'이라는 단 하나의 책임만 구현합니다.
class CustomExporter : public rtfw::common::ExporterApp {
protected:
    // [개발자의 유일한 작업]
    // 로봇 시스템이 사용하는 모든 타입을 팩토리에 등록합니다.
    void register_domain_types(rtfw::common::JsonSerializerFactory& factory) override {
        // 프레임워크 기본 타입들
        factory.register_type<double>();
        factory.register_type<int>();
        factory.register_type<bool>();
        
        // 전용 타입들
        // factory.register_type<SE3>();
        // factory.register_type<se3>();
        factory.register_type<Eigen::Vector3d>();
        factory.register_type<std::array<double, 4>>();
        factory.register_type<manif::SE3d>();
        factory.register_type<manif::SE3Tangentd>();
        factory.register_type<manif::SE2d>();
        factory.register_type<manif::SE2Tangentd>();
        factory.register_type<custom_types::CvResultData>();
        factory.register_type<custom_types::ImuData>();
        // ... 새로운 타입을 추가할 때 여기 한 줄만 더하면 됩니다.
    }
};


// 4. 애플리케이션을 생성하고 실행합니다.
int main(int argc, char* argv[]) {
    CustomExporter app;
    return app.run(argc, argv);
}