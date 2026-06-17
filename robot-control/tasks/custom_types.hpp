#pragma once

#include <array>
#include <cstdint>

namespace custom_types {
    struct Imu {
        struct {
            double x;
            double y;
            double z;
        } acc;
        struct {
            double x;
            double y;
            double z;
        } gyro;
        struct {        
            double w;
            double x;
            double y;
            double z;
        } orientation;
    };

    struct MotorState {
        char name[32];
        double pos;
        double vel;
        double torque;
        double temp;
        int status;
        bool enabled = false;
    };

    // CAN tx/rx 통계 (GUI 모터 뷰 진단용)
    struct MotorIoStats {
        uint32_t tx_count = 0;
        uint32_t rx_count = 0;
        double   tx_hz = 0.0;
        double   rx_hz = 0.0;
    };

    struct MotorCmd {
        char name[32];
        double pos;
        double vel;
        double torque;
        double kp;
        double kd;
        double duration_ms = 0.0;  // interpolation duration (ms), 0 = immediate
    };

    static constexpr int kMaxJointCount = 20;

    // Fixed-size joint state array for real-time use.
    // Index-to-name mapping is configured separately via parameter.
    struct JointStates {
        int count = 0;
        std::array<double, kMaxJointCount> positions{};
    };

    // 데이터 로거 상태 (DataLogger → WsBridgeTask)
    struct LoggerInfo {
        bool recording    = false;
        int  sample_count = 0;
        char filename[256] = {};
    };

}