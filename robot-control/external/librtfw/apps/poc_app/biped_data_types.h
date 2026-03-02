#pragma once
#include <cstdint>

// 고주파 센서 데이터 (IMU, Joint Encoders)
struct HighFreqSensors {
    double imu_accel[3];
    double imu_gyro[3];
    double joint_positions[12];
    double joint_velocities[12];
};

// 모터 토크 명령
struct MotorTorques {
    double joint_torques[12];
};

// 추정된 로봇의 상태 (CoM, ZMP 등)
struct RobotState {
    double center_of_mass[3];
    double zero_moment_point[2];
    bool is_stable;
};

// 보행 패턴 생성기(Planner)의 목표
struct WalkingGait {
    double step_length;
    double step_height;
    double step_frequency;
};

// 저주파 센서 데이터 (카메라)
struct VisionObject {
    int id;
    double position_in_world[3];
};

// Non-RT <-> RT 통신용 데이터 (Gait 파라미터 튜닝)
struct GaitParameters {
    double new_step_length;
    double new_step_height;
};

// Task example state (must be trivially copyable)
struct StatefulCounterState {
    uint64_t counter;
    double last_value;
};

// Task output
struct StatefulCounterOutput {
    uint64_t counter;
    double value;
};