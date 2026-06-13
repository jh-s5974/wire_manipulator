#pragma once
#include <cmath>
#include <utility>

// Wire-based manipulator joint kinematics
//
// Joint 2: 하단 링크 길이 (prismatic)  — 볼스크류, motor 0x03
// Joint 3: 팔꿈치 피치    (revolute)   — 와이어 구동, motors 0x04 & 0x05
// Joint 4: 상단 링크 길이 (prismatic)  — 와이어 구동, motors 0x06 & 0x07
//
// TODO: 각 함수의 placeholder 구현을 실제 기구학으로 교체할 것.
//       파라미터(볼스크류 리드, 풀리 반경 등)도 실측값으로 수정 필요.

namespace kin_manip {

// ────────────────────────────────────────────────────────────
// Joint 2: 하단 링크 길이 (prismatic, 볼스크류, motor 0x03)
//
// 파라미터:
//   JOINT2_BALL_SCREW_LEAD  : 볼스크류 리드 [m/rev]
//                             (1 회전당 직선 이동량, 실측 후 수정할 것)
// ────────────────────────────────────────────────────────────

static constexpr double JOINT2_BALL_SCREW_LEAD = 0.005; // [m/rev] — placeholder

// motor position [rad] → prismatic joint position [m]
inline double joint2_motor_to_joint(double motor_rad) {
    // TODO: 실제 볼스크류 기구학으로 교체
    return motor_rad / (2.0 * M_PI) * JOINT2_BALL_SCREW_LEAD;
}

// prismatic joint position [m] → motor position [rad]
inline double joint2_joint_to_motor(double joint_m) {
    // TODO: 실제 볼스크류 기구학으로 교체
    return joint_m / JOINT2_BALL_SCREW_LEAD * (2.0 * M_PI);
}

// motor velocity [rad/s] → joint velocity [m/s]
inline double joint2_vel_motor_to_joint(double motor_vel) {
    return motor_vel / (2.0 * M_PI) * JOINT2_BALL_SCREW_LEAD;
}

// joint velocity [m/s] → motor velocity [rad/s]
inline double joint2_vel_joint_to_motor(double joint_vel) {
    return joint_vel / JOINT2_BALL_SCREW_LEAD * (2.0 * M_PI);
}

// motor torque [Nm] → joint force [N]  (볼스크류 토크-추력 변환)
inline double joint2_torque_motor_to_joint(double motor_Nm) {
    // TODO: 효율, 마찰각 등 보정 필요
    return motor_Nm / (JOINT2_BALL_SCREW_LEAD / (2.0 * M_PI));
}

// joint force [N] → motor torque [Nm]
inline double joint2_force_joint_to_motor(double joint_N) {
    return joint_N * (JOINT2_BALL_SCREW_LEAD / (2.0 * M_PI));
}


// ────────────────────────────────────────────────────────────
// Joint 3: 팔꿈치 피치 (revolute, 와이어 구동, motors 0x04 & 0x05)
//
// 와이어 배치: motor_a(0x04)와 motor_b(0x05)가 각각 반대 방향으로
//             와이어를 감아 관절을 회전시키는 차동 구조 가정.
// TODO: 실제 풀리 반경, 와이어 배치에 맞게 수정할 것
// ────────────────────────────────────────────────────────────

// motor positions [rad] → revolute joint angle [rad]
inline double joint3_motors_to_joint(double motor_a_rad, double motor_b_rad) {
    // TODO: 실제 와이어 기구학으로 교체
    return (motor_a_rad - motor_b_rad) * 0.5;
}

// revolute joint angle [rad] → (motor_a, motor_b) positions [rad]
inline std::pair<double, double> joint3_joint_to_motors(double joint_rad) {
    // TODO: 실제 와이어 기구학으로 교체
    return { joint_rad, -joint_rad };
}

// motor velocities [rad/s] → joint velocity [rad/s]
inline double joint3_vel_motors_to_joint(double va, double vb) {
    // TODO: Jacobian 기반으로 교체
    return (va - vb) * 0.5;
}

// joint velocity [rad/s] → motor velocities [rad/s]
inline std::pair<double, double> joint3_vel_joint_to_motors(double joint_vel) {
    return { joint_vel, -joint_vel };
}

// motor torques [Nm] → joint torque [Nm]  (J^{-T} * tau)
inline double joint3_torque_motors_to_joint(double tau_a, double tau_b) {
    // TODO: 실제 Jacobian 기반으로 교체
    return (tau_a - tau_b);
}

// joint torque [Nm] → motor torques [Nm]  (J^T * tau)
inline std::pair<double, double> joint3_torque_joint_to_motors(double joint_tau) {
    return { joint_tau * 0.5, -joint_tau * 0.5 };
}


// ────────────────────────────────────────────────────────────
// Joint 4: 상단 링크 길이 (prismatic, 와이어 구동, motors 0x06 & 0x07)
//
// 와이어 배치: motor_a(0x06)와 motor_b(0x07)가 동일 방향으로
//             와이어를 감아 슬라이더를 이동시키는 구조 가정.
// TODO: 실제 풀리 반경, 와이어 배치에 맞게 수정할 것
// ────────────────────────────────────────────────────────────

// motor positions [rad] → prismatic joint position [m]
inline double joint4_motors_to_joint(double motor_a_rad, double motor_b_rad) {
    // TODO: 실제 와이어-프리즈매틱 기구학으로 교체
    return (motor_a_rad + motor_b_rad) * 0.5;
}

// prismatic joint position [m] → (motor_a, motor_b) positions [rad]
inline std::pair<double, double> joint4_joint_to_motors(double joint_m) {
    // TODO: 실제 와이어-프리즈매틱 기구학으로 교체
    return { joint_m, joint_m };
}

// motor velocities [rad/s] → joint velocity [m/s]
inline double joint4_vel_motors_to_joint(double va, double vb) {
    return (va + vb) * 0.5;
}

// joint velocity [m/s] → motor velocities [rad/s]
inline std::pair<double, double> joint4_vel_joint_to_motors(double joint_vel) {
    return { joint_vel, joint_vel };
}

// motor torques [Nm] → joint force [N]
inline double joint4_torque_motors_to_joint(double tau_a, double tau_b) {
    // TODO: 실제 Jacobian 기반으로 교체
    return (tau_a + tau_b);
}

// joint force [N] → motor torques [Nm]
inline std::pair<double, double> joint4_force_joint_to_motors(double joint_force) {
    return { joint_force * 0.5, joint_force * 0.5 };
}

} // namespace kin_manip
