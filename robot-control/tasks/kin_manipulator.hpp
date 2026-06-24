#pragma once
#include <cmath>

// ═══════════════════════════════════════════════════════════════════════════
// 와이어 매니퓰레이터 — 조인트 ↔ 모터 단위 변환 (kinematics)
// ═══════════════════════════════════════════════════════════════════════════
//
// 이 파일은 "가상 조인트 공간"과 "물리 모터 공간" 사이의 순수 변환 함수만 모은다.
// (위치/속도/토크 FK·IK). 제어 로직·CAN 통신은 can_bus0.h / can_bus1.h 가 담당하고,
// 여기서는 상태나 부작용 없이 값 변환만 한다.
//
// 담당 조인트 (전역 조인트 인덱스 기준):
//   joint2: lower_link  — 볼스크류 프리즈매틱, motor 0x03            (CanBus0 소유)
//   joint3: elbow_pitch — 와이어 구동, 0x06(A,장력) + 0x07(B,위치)   (CanBus1 소유)
//   joint4: upper_link  — 와이어 구동, 0x04(A,장력) + 0x05(B,위치)   (CanBus1 소유)
//
// 와이어 구동 조인트(joint3/4)의 모터 역할은 "고정"이다:
//   - B 모터(0x07 / 0x05): 항상 위치 제어 — 조인트 각도/길이를 직접 추종
//   - A 모터(0x04 / 0x06): 항상 장력(토크) 제어 — 반대편에서 와이어를 당겨 장력 유지
//   (이동 방향에 따라 위치/장력 모터가 뒤바뀌지 않는다.)
//
// 변환 계수 출처: Arduino motor_total.ino
//   - 팔꿈치 비율  : 3.92 × 0.928 [deg_motor / deg_joint]
//   - 상단링크 비율: 동력전달 체인 기하로 계산 (아래 UPPER_RATIO 주석 참조)
//   - lower_link 커플링(팔꿈치)  : -4.0 / 90  [motor_B_rad / motor_03_rad]
//   - lower_link 커플링(상단링크): -5.8 / 90  [motor_rad   / motor_03_rad]
//
// 모터 회전 방향(실제 배선/장착에 따른 부호)은 여기서 다루지 않는다.
// config/robotnl.yaml 의 motor_direction_sign 으로 물리 모터별로 보정하며,
// can_bus0.h / can_bus1.h 가 모터 상태를 읽고 명령을 보내는 시점에 적용한다.
// 따라서 이 파일의 함수는 모두 "보정된(= Arduino 기준과 동일한 방향) 모터 값"을 다룬다.

namespace kin_manip {

// ═══════════════════════════════════════════════════════════════════
// Joint 2: lower_link — 볼스크류 프리즈매틱 (motor 0x03)
//   모터 회전[rad] ↔ 직선 이동[m] 의 1:1 선형 변환. 와이어 커플링 없음.
// ═══════════════════════════════════════════════════════════════════

// 볼스크류 리드 [m/rev] — 실측: 모터 90°(π/2 rad)당 1mm → 1회전당 4mm = 0.004 m/rev
static constexpr double JOINT2_BALL_SCREW_LEAD = 0.004;

// 위치: motor [rad] → joint [m]
inline double joint2_motor_to_joint(double motor_rad) {
    return motor_rad / (2.0 * M_PI) * JOINT2_BALL_SCREW_LEAD;
}
// 위치: joint [m] → motor [rad]
inline double joint2_joint_to_motor(double joint_m) {
    return joint_m / JOINT2_BALL_SCREW_LEAD * (2.0 * M_PI);
}
// 속도: motor [rad/s] → joint [m/s]
inline double joint2_vel_motor_to_joint(double motor_vel) {
    return motor_vel / (2.0 * M_PI) * JOINT2_BALL_SCREW_LEAD;
}
// 속도: joint [m/s] → motor [rad/s]
inline double joint2_vel_joint_to_motor(double joint_vel) {
    return joint_vel / JOINT2_BALL_SCREW_LEAD * (2.0 * M_PI);
}
// 힘/토크: motor torque [Nm] → joint force [N]
inline double joint2_torque_motor_to_joint(double motor_Nm) {
    return motor_Nm / (JOINT2_BALL_SCREW_LEAD / (2.0 * M_PI));
}
// 힘/토크: joint force [N] → motor torque [Nm]
inline double joint2_force_joint_to_motor(double joint_N) {
    return joint_N * (JOINT2_BALL_SCREW_LEAD / (2.0 * M_PI));
}

// ═══════════════════════════════════════════════════════════════════
// Joint 3: elbow_pitch — 와이어 구동 (motors 0x06 A + 0x07 B)
//
// 모터 역할 (고정):
//   motor A (0x06): 항상 장력(전류) 제어 — kp=kd=0, torque=ELBOW_TENSION_TORQUE
//   motor B (0x07): 항상 위치 제어       — 조인트 각도를 추종 (active)
//
// lower_link 커플링:
//   motor 0x03 이 θ[rad] 회전하면 motor B 목표에 ELBOW_LOWER_COUPLING × θ 를 더한다.
//   (볼스크류가 움직이면 팔꿈치 와이어 길이가 변하는 것을 보상)
//
// 아래 FK/IK 함수는 위치 제어 모터(B, 0x07) 기준이다. 장력 모터(A)는 변환 대상이 아니다.
// ═══════════════════════════════════════════════════════════════════

static constexpr double ELBOW_RATIO          = 1.6;  // [motor_B_rad / joint3_rad]
static constexpr double ELBOW_LOWER_COUPLING = 0.0;  // [motor_B_rad / motor_03_rad]
static constexpr double ELBOW_TENSION_TORQUE = 0.2;  // [Nm] 장력 모터(A) 토크 — 실측 후 튜닝 필요

// FK: 위치 모터 B(0x07) 각도 + lower_link 커플링 보정 → 팔꿈치 각도 [rad]
//   motor_b_rad : motors[1](0x07) 현재 위치,  motor2_rad : lower_link motor(0x03) 현재 위치
inline double joint3_motor_to_joint(double motor_b_rad, double motor2_rad) {
    return (motor_b_rad - ELBOW_LOWER_COUPLING * motor2_rad) / ELBOW_RATIO;
}

// IK: 팔꿈치 각도 + lower_link 커플링 보정 → 위치 모터 B(0x07) 목표 위치 [rad]
//   motor2_rad : lower_link motor(0x03) 현재 위치 (커플링 보정용)
inline double joint3_joint_to_motor_pos(double q3_rad, double motor2_rad) {
    return q3_rad * ELBOW_RATIO + ELBOW_LOWER_COUPLING * motor2_rad;
}

// 속도 변환 (위치 모터 B 기준)
inline double joint3_vel_joint_to_motor(double joint_vel) { return joint_vel * ELBOW_RATIO; }
inline double joint3_vel_motor_to_joint(double motor_vel) { return motor_vel / ELBOW_RATIO; }

// 토크 변환 (위치 모터 B 기준)
inline double joint3_torque_joint_to_motor(double joint_Nm) { return joint_Nm / ELBOW_RATIO; }
inline double joint3_torque_motor_to_joint(double motor_Nm) { return motor_Nm * ELBOW_RATIO; }

// ═══════════════════════════════════════════════════════════════════
// Joint 4: upper_link — 와이어 구동 (motors 0x04 A + 0x05 B)
//
// 모터 역할 (고정 — 이동 방향과 무관):
//   motor A (0x04): 항상 장력(전류) 제어 — kp=kd=0, torque=UPPER_TENSION_TORQUE
//   motor B (0x05): 항상 위치 제어       — 상단링크 길이를 추종 (active)
//
// lower_link 커플링:
//   motor 0x03 회전이 상단링크 와이어 길이에 주는 영향 보상 (원래값 -5.8/90, 현재 미측정 0).
//
// 아래 FK/IK 함수는 위치 제어 모터(B, 0x05) 기준이다. 장력 모터(A)는 변환 대상이 아니다.
// ═══════════════════════════════════════════════════════════════════

// 동력전달 체인 (모터 → 와이어풀리 → 타이밍벨트풀리 → 기어 → 볼스크류):
//   모터축 40mm 풀리 --(와이어)--> 20mm 풀리 (동축 28mm 타이밍풀리)
//     --(타이밍벨트)--> 48mm 타이밍풀리 (동축 큰 기어, 2.8:1)
//     --(기어 맞물림)--> 작은 기어 (동축 볼스크류, 90°당 1mm → 4mm/rev)
// [motor_rad / joint4_m] = 2π ÷ { 볼스크류리드(0.004) × [ (40/20) × (28/48) × 2.8 ] }
//   (40/20): 모터풀리/아이들러풀리, (28/48): 작은타이밍풀리/큰타이밍풀리, 2.8: 큰기어:작은기어
static constexpr double UPPER_RATIO          =
    (2.0 * M_PI) / (0.004 * (40.0 / 20.0) * (28.0 / 48.0) * 2.8);
static constexpr double UPPER_LOWER_COUPLING = 0.0;  // [motor_rad / motor_03_rad]
static constexpr double UPPER_TENSION_TORQUE = 0.2;  // [Nm] 장력 모터(A) 토크 — 실측 후 튜닝 필요

// FK: 위치 모터 B(0x05) 각도 + lower_link 커플링 보정 → 상단링크 길이 [m]
//   motor_b_rad : motors[3](0x05) 현재 위치,  motor2_rad : lower_link motor(0x03) 현재 위치
inline double joint4_motor_to_joint(double motor_b_rad, double motor2_rad) {
    return (motor_b_rad - UPPER_LOWER_COUPLING * motor2_rad) / UPPER_RATIO;
}

// IK: 상단링크 길이 + lower_link 커플링 보정 → 위치 모터 B(0x05) 목표 위치 [rad]
//   motor2_rad : lower_link motor(0x03) 현재 위치 (커플링 보정용)
inline double joint4_joint_to_motor_pos(double q4_rad, double motor2_rad) {
    return q4_rad * UPPER_RATIO + UPPER_LOWER_COUPLING * motor2_rad;
}

// 속도 변환 (위치 모터 B 기준)
inline double joint4_vel_joint_to_motor(double joint_vel) { return joint_vel * UPPER_RATIO; }
inline double joint4_vel_motor_to_joint(double motor_vel) { return motor_vel / UPPER_RATIO; }

// 토크 변환 (위치 모터 B 기준)
inline double joint4_torque_joint_to_motor(double joint_Nm) { return joint_Nm / UPPER_RATIO; }
inline double joint4_torque_motor_to_joint(double motor_Nm) { return motor_Nm * UPPER_RATIO; }

} // namespace kin_manip
