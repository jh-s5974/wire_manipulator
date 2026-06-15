#pragma once
#include <cmath>

// Wire-based manipulator joint kinematics (CAN Bus 1 joints)
//
// Joint 2 (ji=0): lower_link  — 볼스크류 프리즈매틱,    motor 0x03
// Joint 3 (ji=1): elbow_pitch — 와이어 구동,            motors 0x04(A,장력) + 0x05(B,위치)
// Joint 4 (ji=2): upper_link  — 와이어 구동(방향별 전환), motors 0x06(A) + 0x07(B)
//
// 변환 계수 출처: Arduino motor_total.ino
//   - 팔꿈치 비율  : 3.92 × 0.928 [deg_motor / deg_joint]
//   - 상단링크 비율: 74.48         [deg_motor / deg_joint]
//   - lower_link 커플링 (팔꿈치): -4.0 / 90  [motor_B_rad / motor_03_rad]
//   - lower_link 커플링 (상단링크): -5.8 / 90 [motor_rad   / motor_03_rad]

namespace kin_manip {

// ═══════════════════════════════════════════════════════════════════
// Joint 2: lower_link — 볼스크류 프리즈매틱, motor 0x03
// ═══════════════════════════════════════════════════════════════════

// 볼스크류 리드 [m/rev] — 실측 후 수정 필요
static constexpr double JOINT2_BALL_SCREW_LEAD = 0.005;

// motor [rad] → joint [m]
inline double joint2_motor_to_joint(double motor_rad) {
    return motor_rad / (2.0 * M_PI) * JOINT2_BALL_SCREW_LEAD;
}
// joint [m] → motor [rad]
inline double joint2_joint_to_motor(double joint_m) {
    return joint_m / JOINT2_BALL_SCREW_LEAD * (2.0 * M_PI);
}
// motor vel [rad/s] → joint vel [m/s]
inline double joint2_vel_motor_to_joint(double motor_vel) {
    return motor_vel / (2.0 * M_PI) * JOINT2_BALL_SCREW_LEAD;
}
// joint vel [m/s] → motor vel [rad/s]
inline double joint2_vel_joint_to_motor(double joint_vel) {
    return joint_vel / JOINT2_BALL_SCREW_LEAD * (2.0 * M_PI);
}
// motor torque [Nm] → joint force [N]
inline double joint2_torque_motor_to_joint(double motor_Nm) {
    return motor_Nm / (JOINT2_BALL_SCREW_LEAD / (2.0 * M_PI));
}
// joint force [N] → motor torque [Nm]
inline double joint2_force_joint_to_motor(double joint_N) {
    return joint_N * (JOINT2_BALL_SCREW_LEAD / (2.0 * M_PI));
}

// ═══════════════════════════════════════════════════════════════════
// Joint 3: elbow_pitch — 와이어 구동, motors 0x04(A) + 0x05(B)
//
// 구동 구조:
//   motor A (0x04, motors[1]): 항상 장력(전류) 제어
//   motor B (0x05, motors[2]): 항상 위치 제어 (active)
//
// lower_link 커플링:
//   motor 0x03 이 θ rad 회전하면 motor B 목표에 ELBOW_LOWER_COUPLING × θ 를 더함.
//   (Arduino: -4.0 deg_m3 / deg_dlink = -4.0/90 rad_mB / rad_m03)
//
// 장력 토크 부호: motor A 와이어 권선 방향에 따라 조정 필요
// ═══════════════════════════════════════════════════════════════════

static constexpr double ELBOW_RATIO          = 3.92 * 0.928;  // [motor_B_rad / joint3_rad]
static constexpr double ELBOW_LOWER_COUPLING = -4.0 / 90.0;   // [motor_B_rad / motor_03_rad]
static constexpr double ELBOW_TENSION_TORQUE = 0.5;           // [Nm] — 실측 후 튜닝 필요

// FK: motor B 위치 + lower_link 커플링 보정 → 팔꿈치 각도 [rad]
// motor_b_rad : motors[2] (0x05) 현재 위치
// motor2_rad  : motors[0] (0x03) 현재 위치
inline double joint3_motor_to_joint(double motor_b_rad, double motor2_rad) {
    return (motor_b_rad - ELBOW_LOWER_COUPLING * motor2_rad) / ELBOW_RATIO;
}

// IK: 팔꿈치 각도 + 커플링 보정 → motor B 목표 위치 [rad]
// motor2_rad : motors[0] (0x03) 현재 위치 (커플링 보정용)
inline double joint3_joint_to_motor_pos(double q3_rad, double motor2_rad) {
    return q3_rad * ELBOW_RATIO + ELBOW_LOWER_COUPLING * motor2_rad;
}

// 속도 변환
inline double joint3_vel_joint_to_motor(double joint_vel) { return joint_vel * ELBOW_RATIO; }
inline double joint3_vel_motor_to_joint(double motor_vel) { return motor_vel / ELBOW_RATIO; }

// ═══════════════════════════════════════════════════════════════════
// Joint 4: upper_link — 와이어 구동, motors 0x06(A) + 0x07(B)
//
// 구동 구조 (방향별 전환):
//   q4 증가 방향 (extending): motor B (0x07, motors[4]) 위치 제어, motor A 장력
//   q4 감소 방향 (retracting): motor A (0x06, motors[3]) 위치 제어, motor B 장력
//   (Arduino: past_ulink >= ulink → m5 위치/m4 장력, past_ulink < ulink → m4 위치/m5 장력)
//
// lower_link 커플링:
//   (Arduino: -5.8 deg_motor / deg_dlink = -5.8/90 rad_motor / rad_m03)
//
// 장력 토크 부호: 실제 권선 방향에 따라 조정 필요
// ═══════════════════════════════════════════════════════════════════

static constexpr double UPPER_RATIO          = 74.48;          // [motor_rad / joint4_rad]
static constexpr double UPPER_LOWER_COUPLING = -5.8 / 90.0;   // [motor_rad / motor_03_rad]
static constexpr double UPPER_TENSION_TORQUE = 0.5;           // [Nm] — 실측 후 튜닝 필요

// FK: active 모터 위치 + lower_link 커플링 보정 → upper_link 각도 [rad]
// motor_active_rad : 현재 active 모터(A 또는 B) 위치
// motor2_rad       : motors[0] (0x03) 현재 위치
inline double joint4_motor_to_joint(double motor_active_rad, double motor2_rad) {
    return (motor_active_rad - UPPER_LOWER_COUPLING * motor2_rad) / UPPER_RATIO;
}

// IK: upper_link 각도 + 커플링 보정 → active 모터 목표 위치 [rad]
// motor2_rad : motors[0] (0x03) 현재 위치 (커플링 보정용)
inline double joint4_joint_to_motor_pos(double q4_rad, double motor2_rad) {
    return q4_rad * UPPER_RATIO + UPPER_LOWER_COUPLING * motor2_rad;
} 

// 속도 변환
inline double joint4_vel_joint_to_motor(double joint_vel) { return joint_vel * UPPER_RATIO; }
inline double joint4_vel_motor_to_joint(double motor_vel) { return motor_vel / UPPER_RATIO; }

} // namespace kin_manip
