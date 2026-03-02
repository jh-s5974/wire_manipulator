// eskf_ca.h
#pragma once

#ifndef ESKF_CA_H
#define ESKF_CA_H

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

namespace {
    // 상태 및 오차 상태의 차원을 상수로 정의
    constexpr int STATE_DIM = 18;
    constexpr int ERROR_STATE_DIM = 18;
}

// Eigen 타입 별칭으로 가독성 향상
using Vector3d = Eigen::Vector3d;
using Quaterniond = Eigen::Quaterniond;
using Matrix3d = Eigen::Matrix3d;
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Matrix18d = Eigen::Matrix<double, ERROR_STATE_DIM, ERROR_STATE_DIM>;
using Matrix6x18d = Eigen::Matrix<double, 6, ERROR_STATE_DIM>;
using Matrix18x6d = Eigen::Matrix<double, ERROR_STATE_DIM, 6>;
using Vector18d = Eigen::Matrix<double, ERROR_STATE_DIM, 1>;


class ESKF_CA {
public:
    // --- 상태 변수 (Public으로 두어 외부에서 접근 용이) ---
    Vector3d p;     // Position
    Vector3d v;     // Velocity
    Quaterniond q;  // Orientation
    Vector3d a;     // Acceleration
    Vector3d omega; // Angular velocity
    Vector3d alpha; // Angular acceleration

    // --- 공분산 행렬 ---
    Matrix18d P; // 오차 상태 공분산
    Matrix18d Q; // 프로세스 노이즈 공분산

    /**     * 
     * @brief 생성자
     */
    ESKF_CA();
    
    /**
     * @brief 생성자
     * @param P0 초기 오차 상태 공분산
     * @param Q_ 프로세스 노이즈 공분산
     */
    ESKF_CA(const Matrix18d& P0, const Matrix18d& Q_);

    /**
     * @brief 예측 단계: 등가속도, 등각가속도 모델을 사용하여 상태와 공분산을 예측합니다.
     * @param dt 시간 간격 (초)
     */
    void predict(double dt);

    /**
     * @brief 업데이트 단계: ArUco 마커로부터 얻은 위치 및 자세 측정을 사용하여 상태를 보정합니다.
     * @param z_p 측정된 위치 (measurement position)
     * @param z_q 측정된 자세 (measurement orientation)
     * @param R_pos 위치 측정 노이즈 공분산
     * @param R_rot 자세 측정 노이즈 공분산
     */
    void update(const Vector3d& z_p, const Quaterniond& z_q, const Matrix3d& R_pos, const Matrix3d& R_rot);
    void reset();

private:
    /**
     * @brief 행렬의 수치적 안정성을 위해 대칭으로 만듭니다.
     * @param mat 대칭으로 만들 행렬
     * @return 대칭이 보장된 행렬
     */
    template<typename T>
    T force_symmetry(const T& mat) {
        return 0.5 * (mat + mat.transpose());
    }
};

ESKF_CA::ESKF_CA() {
    reset();
}

ESKF_CA::ESKF_CA(const Matrix18d& P0, const Matrix18d& Q_): ESKF_CA() {
    // 오차 상태 공분산 및 노이즈 공분산 초기화
    P << P0;
    Q << Q_;
}

void ESKF_CA::reset() {
    // 공칭 상태 초기화
    p.setZero();
    v.setZero();
    // Scipy의 [x,y,z,w]와 달리 Eigen은 Quaterniond(w,x,y,z) 순서
    q = Quaterniond::Identity(); // w=1, x=y=z=0
    a.setZero();
    omega.setZero();
    alpha.setZero();
}

void ESKF_CA::predict(double dt) {
    // 1. 공칭 상태 예측 (Nominal State Prediction)
    p += v * dt + 0.5 * a * dt * dt;
    v += a * dt;
    
    Vector3d rot_vec_pred = omega * dt + 0.5 * alpha * dt * dt;
    
    // 회전 벡터를 쿼터니언으로 변환
    // 크기가 작은 회전에 대해 안정적
    double angle = rot_vec_pred.norm();
    Vector3d axis = (angle > 1e-9) ? rot_vec_pred.normalized() : Vector3d(1, 0, 0);
    Quaterniond delta_q(Eigen::AngleAxisd(angle, axis));
    
    // 왼쪽 곱셈(Left Multiplication)으로 자세 업데이트
    q = delta_q * q;
    q.normalize(); // 정규화 추가하여 수치 안정성 확보

    omega += alpha * dt;

    // 2. 오차 공분산 예측 (Error Covariance Prediction)
    Matrix18d F = Matrix18d::Identity();
    Matrix3d I3 = Matrix3d::Identity();

    // Python: F[0:3, 3:6] = np.eye(3) * dt
    F.block<3, 3>(0, 3) = I3 * dt;
    F.block<3, 3>(0, 9) = I3 * 0.5 * dt * dt;
    F.block<3, 3>(3, 9) = I3 * dt;
    // Python: F[6:9, 6:9] = delta_q.inv().as_matrix()
    // delta_q의 역행렬은 delta_q.inverse()의 회전행렬과 같음
    F.block<3, 3>(6, 6) = delta_q.toRotationMatrix().transpose();
    F.block<3, 3>(6, 12) = I3 * dt;
    F.block<3, 3>(6, 15) = I3 * 0.5 * dt * dt;
    F.block<3, 3>(12, 15) = I3 * dt;

    P = F * P * F.transpose() + Q;
    P = force_symmetry(P); // 안정성을 위한 대칭성 강제
}

void ESKF_CA::update(const Vector3d& z_p, const Quaterniond& z_q, const Matrix3d& R_pos, const Matrix3d& R_rot) {
    // 1. 혁신(Innovation) 계산
    // 위치 오차
    Vector3d y_p = z_p - p;

    // 자세 오차 (회전 벡터 형태)
    Quaterniond delta_q_measurement = z_q * q.inverse();
    Eigen::AngleAxisd angle_axis(delta_q_measurement);
    Vector3d y_theta = angle_axis.angle() * angle_axis.axis();

    // 6차원 혁신 벡터
    Vector6d y;
    y.head<3>() = y_p;
    y.tail<3>() = y_theta;

    // 2. 측정 모델 및 칼만 이득 계산
    Matrix6x18d H = Matrix6x18d::Zero();
    H.block<3, 3>(0, 0) = Matrix3d::Identity(); // 위치 오차(dp)에 대한 H
    H.block<3, 3>(3, 6) = Matrix3d::Identity(); // 각도 오차(d_theta)에 대한 H

    Matrix6d R = Matrix6d::Zero();
    R.block<3, 3>(0, 0) = R_pos;
    R.block<3, 3>(3, 3) = R_rot;

    Matrix6d S = H * P * H.transpose() + R;
    Matrix18x6d K = P * H.transpose() * S.inverse();
    
    // 3. 오차 상태 보정량 계산
    Vector18d delta_x = K * y;

    // 4. 공분산 업데이트 (Joseph form)
    Matrix18d I = Matrix18d::Identity();
    P = (I - K * H) * P;
    P = force_symmetry(P);

    // 5. 오차 상태 주입 (Injection)
    p += delta_x.segment<3>(0);
    v += delta_x.segment<3>(3);
    
    // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    // 왼쪽 곱셈으로 오차 주입 (Python 코드와 동일)
    Vector3d d_theta = delta_x.segment<3>(6);
    double angle = d_theta.norm();
    Vector3d axis = (angle > 1e-9) ? d_theta.normalized() : Vector3d(1, 0, 0);
    Quaterniond delta_q_error(Eigen::AngleAxisd(angle, axis));
    
    q = delta_q_error * q;
    q.normalize(); // 정규화
    // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    
    a += delta_x.segment<3>(9);
    omega += delta_x.segment<3>(12);
    alpha += delta_x.segment<3>(15);
}

#endif // ESKF_CA_H