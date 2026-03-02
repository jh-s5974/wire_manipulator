#pragma once // 헤더 가드

#include "eigen3/Eigen/Dense"

template<int ny, int nx, int nu>
struct KF {
    // 시스템 모델 행렬 (사용자가 직접 설정)
    Eigen::Matrix<double, nx, nx> A; // 상태 전이 행렬
    Eigen::Matrix<double, nx, nu> B; // 제어 입력 행렬
    Eigen::Matrix<double, ny, nx> H; // 관측 행렬

    // 노이즈 공분산 행렬 (대각 행렬로 가정)
    Eigen::DiagonalMatrix<double, nx> Q_; // 프로세스 노이즈 공분산
    Eigen::DiagonalMatrix<double, ny> R_; // 측정 노이즈 공분산

    // 필터 내부 상태
    Eigen::Vector<double, nx> x_pri;    // 추정 상태 (x_k|k)
    Eigen::Vector<double, nx> x_post;    // 추정 상태 (x_k|k)
    Eigen::Matrix<double, nx, nx> P_pri;    // 추정 오차 공분산 (P_k|k)
    Eigen::Matrix<double, nx, nx> P_post;    // 추정 오차 공분산 (P_k|k)
    
    // 생성자
    KF() {
        P_post = Eigen::Matrix<double, nx, nx>::Identity() * 1e-3; // 초기 공분산 (작은 값)
        
        // Q와 R의 대각 성분을 기본값으로 초기화 (set_weights로 반드시 재설정 필요)
        if (nx > 0) Q_.diagonal().setOnes(); // 0차원 방지
        if (ny > 0) R_.diagonal().setOnes(); // 0차원 방지
        reset();
    }

    void build_model(const Eigen::Matrix<double, nx, nx>& A, 
        const Eigen::Matrix<double, nx, nu>& B, 
        const Eigen::Matrix<double, ny, nx>& H) {
        this->A << A;
        this->B << B;
        this->H << H;
    }

    // 상태 및 공분산 초기화
    void reset() {
        x_post.setZero();
        P_post = 1e-3 * Eigen::MatrixXd::Identity(nx, nx);
    }

    // Q와 R의 대각 성분 설정
    void set_weights(const Eigen::Vector<double, nx>& q_diag, const Eigen::Vector<double, ny>& r_diag) {
        Q_.diagonal() = q_diag;
        R_.diagonal() = r_diag;
    }

    // 칼만 필터 업데이트 단계
    void update(const Eigen::Vector<double, ny>& y, const Eigen::Vector<double, nu>& u) {
        // 1. 예측 (Prediction)
        // x_k|k-1 = A * x_k-1|k-1 + B * u_k
        x_pri = A * x_post + B * u;
        // P_k|k-1 = A * P_k-1|k-1 * A' + Q
        P_pri = A * P_post * A.transpose() + Q_.toDenseMatrix();

        // 2. 업데이트 (Correction / Measurement Update)
        // S_k = H * P_k|k-1 * H' + R (Innovation covariance)
        Eigen::Matrix<double, ny, ny> S = H * P_pri * H.transpose() + R_.toDenseMatrix();

        // K_k = P_k|k-1 * H' * S_k^-1 (Kalman gain)
        // 수치 안정성을 위해 S의 역행렬을 직접 구하는 대신 선형 시스템 S * K_transpose_temp = (H * P_pred) 를 푼다.
        // (H * P_pred)는 ny x nx 행렬. S.llt().solve(...)의 결과는 K_transpose_temp (ny x nx).
        // K = K_transpose_temp.transpose() (nx x ny)
        Eigen::Matrix<double, nx, ny> K = (S.llt().solve(H * P_pri)).transpose();
        // Cholesky 분해가 실패했는지 확인 (선택 사항)
        // if(S.llt().info() != Eigen::Success) { /* 에러 처리 */ }

        // x_k|k = x_k|k-1 + K_k * (y_k - H * x_k|k-1) (Updated state estimate)
        x_post = x_pri + K * (y - H * x_pri);

        // P_k|k = (I - K_k * H) * P_k|k-1 (Updated covariance estimate)
        // P_ = (Eigen::Matrix<double, nx, nx>::Identity() - K * H) * P_pred;
        // 더 수치적으로 안정적인 Joseph form (선택 사항):
        // Eigen::Matrix<double,nx,nx> I_KH = Eigen::Matrix<double, nx, nx>::Identity() - K * H;
        // P_ = I_KH * P_pred * I_KH.transpose() + K * R_ * K.transpose();
        // 원래 코드의 단순 형태:
        P_post = (Eigen::Matrix<double, nx, nx>::Identity() - K * H) * P_pri;

        // P_가 항상 대칭인지 확인 (디버깅용)
        // P_ = 0.5 * (P_ + P_.transpose());
    }
};
