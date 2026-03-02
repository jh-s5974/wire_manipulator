#pragma once

#include "rtfw/task.h"
// #include "parameters.h"
#include "eigen3/Eigen/Dense"
#include "interfaces/ImuData.h"
#include <manif/manif.h>

namespace {
    static double acc_noise;
    static double gyro_noise;
    static double acc_bias_noise;
    static double gyro_bias_noise;
    static double odom_vel_noise;
};

class MecanumIEKF {
public:
    // 에러 상태의 순서를 정의 (매우 중요)
    enum ErrorStateIdx {
        P = 0,     // 위치 오차 (Position)
        V = 3,     // 속도 오차 (Velocity)
        Ang = 6,   // 각도 오차 (Angle)
        AccBias = 9,  // 가속도 바이어스 오차 (Accelerometer Bias)
        GyroBias = 12 // 자이로 바이어스 오차 (Gyroscope Bias)
    };

    MecanumIEKF() : g_(0, 0, -9.80665) {
        p_IMU_in_body_ << -0.25, 0.0, 0.0;
        reset();
    }
    
    void reset() {
        // --- 상태 초기화 ---
        R_.setIdentity();
        v_.setZero();
        p_.setZero();
        b_a_.setZero();
        b_g_.setZero();
        last_unbiased_gyro_.setZero(); // 이전 자이로 값도 초기화

        // --- 공분산 초기화 ---
        P_.setIdentity();
        P_ *= 1e-4; // 일반적으로 작은 값으로 초기화 (dt 곱하기보다 안정적)

        Eigen::Matrix<double, 12, 1> Q_diag;
        Q_diag << Eigen::Vector3d::Constant(acc_noise).array().square(),
                Eigen::Vector3d::Constant(gyro_noise).array().square(),
                Eigen::Vector3d::Constant(acc_bias_noise).array().square(),
                Eigen::Vector3d::Constant(gyro_bias_noise).array().square();
        Q_noise_ = Q_diag.asDiagonal();

        R_noise_base_ = Eigen::Vector2d::Constant(odom_vel_noise).array().square().matrix().asDiagonal();
    }

    // 메인 함수들
    void predict(const custom_types::ImuData& imu, double dt) {
        // 1. 바이어스 제거 및 현재 자이로 계산
        Eigen::Vector3d accel_measured = imu.accel - b_a_;
        Eigen::Vector3d current_unbiased_gyro = imu.gyro - b_g_;

        // --- [수정] IMU 레버 암(Lever Arm) 보상 강화 ---
        // 각가속도(alpha) 추정 (이전 스텝 값과의 차이 이용)
        Eigen::Vector3d angular_acceleration = (current_unbiased_gyro - last_unbiased_gyro_) / dt;

        // 회전으로 인해 IMU에서 발생하는 가속도 계산
        Eigen::Vector3d centripetal_accel = current_unbiased_gyro.cross(current_unbiased_gyro.cross(p_IMU_in_body_));
        Eigen::Vector3d tangential_accel = angular_acceleration.cross(p_IMU_in_body_);
        
        // IMU 측정 가속도에서 회전 효과(구심+접선)를 모두 제거하여 로봇 중심의 가속도 추정
        Eigen::Vector3d accel_at_robot_center = accel_measured - centripetal_accel - tangential_accel;

        // 다음 계산을 위해 현재 자이로 값을 멤버 변수에 저장
        last_unbiased_gyro_ = current_unbiased_gyro;

        // 2. 공칭 상태 전파 (Nominal State Propagation)
        v_ += (R_.rotation() * accel_at_robot_center + g_) * dt;
        p_ += v_ * dt;
        
        manif::SO3d::Tangent delta_theta(last_unbiased_gyro_ * dt);
        R_ = R_ * delta_theta.exp();

        // 3. 에러 상태 공분산 전파 (Error-State Covariance Propagation)
        // A. 에러 상태 동역학 행렬 Fd 계산
        Eigen::Matrix<double, 15, 15> F = Eigen::Matrix<double, 15, 15>::Zero();
        F.block<3, 3>(P, V) = Eigen::Matrix3d::Identity();        
        F.block<3, 3>(V, Ang) = -R_.rotation() * manif::SO3d::Tangent(accel_at_robot_center).hat();
        F.block<3, 3>(V, AccBias) = -R_.rotation();
        F.block<3, 3>(Ang, Ang) = -manif::SO3d::Tangent(last_unbiased_gyro_).hat();
        F.block<3, 3>(Ang, GyroBias) = -Eigen::Matrix3d::Identity();
        
        Eigen::Matrix<double, 15, 15> Fd = Eigen::Matrix<double, 15, 15>::Identity() + F * dt;
        
        // B. 프로세스 노이즈 행렬 Gd 계산
        Eigen::Matrix<double, 15, 12> G = Eigen::Matrix<double, 15, 12>::Zero();
        G.block<3, 3>(V, 0) = -R_.rotation();
        G.block<3, 3>(Ang, 3) = -Eigen::Matrix3d::Identity();
        G.block<3, 3>(AccBias, 6) = Eigen::Matrix3d::Identity();
        G.block<3, 3>(GyroBias, 9) = Eigen::Matrix3d::Identity();

        Eigen::Matrix<double, 15, 15> Qd = G * Q_noise_ * G.transpose() * dt;

        // C. 공분산 전파
        P_ = Fd * P_ * Fd.transpose() + Qd;
    }

    void update(const manif::SE3Tangentd body_vel_odom, const custom_types::ImuData& last_imu) {
        Eigen::Vector2d measurement = body_vel_odom.lin().head<2>();

        // --- [수정] 적응형 측정 노이즈(Adaptive Measurement Noise) 활성화 ---
        double odom_yaw_rate = body_vel_odom.ang().z();
        double imu_yaw_rate = last_unbiased_gyro_.z();
        double yaw_rate_diff = std::abs(odom_yaw_rate - imu_yaw_rate);
        
        Eigen::Matrix<double, 2, 2> R_adaptive = R_noise_base_;
        double slip_factor = 50.0; // 실험을 통해 튜닝이 필요한 값
        R_adaptive(0, 0) += slip_factor * yaw_rate_diff;
        R_adaptive(1, 1) += slip_factor * yaw_rate_diff;

        // --- [수정] 측정 모델 자코비안 H 행렬 완성 ---
        Eigen::Matrix<double, 2, 15> H = Eigen::Matrix<double, 2, 15>::Zero();
        Eigen::Matrix3d R_T = R_.rotation().transpose();
        H.block<2, 3>(0, V) = R_T.block<2, 3>(0, 0); // 속도 오차(dv)에 대한 자코비안
        
        Eigen::Vector3d v_body = R_T * v_;
        // 자세 오차(d_theta)가 측정값(body_vel)에 미치는 영향을 나타내는 핵심 항
        H.block<2, 3>(0, Ang) = manif::SO3d::Tangent(v_body).hat().block<2, 3>(0, 0);

        // 4. 칼만 게인 및 업데이트
        Eigen::Matrix<double, 2, 2> S = H * P_ * H.transpose() + R_adaptive;
        Eigen::Matrix<double, 15, 2> K = P_ * H.transpose() * S.inverse();
        
        // 이노베이션 계산 (측정값 - 예측값)
        Eigen::Vector2d predicted_measurement = v_body.head<2>();
        Eigen::Vector2d innovation = measurement - predicted_measurement;
        
        // 에러 상태 보정
        Eigen::Matrix<double, 15, 1> delta_x = K * innovation;

        // --- [수정] 수치적으로 안정적인 Joseph Form으로 공분산 업데이트 ---
        Eigen::Matrix<double, 15, 15> I_KH = Eigen::Matrix<double, 15, 15>::Identity() - K * H;
        P_ = I_KH * P_ * I_KH.transpose() + K * R_adaptive * K.transpose();

        // 5. 상태 주입 (State Injection)
        p_ += delta_x.segment<3>(P);
        v_ += delta_x.segment<3>(V);
        R_ = R_ * manif::SO3d::Tangent(delta_x.segment<3>(Ang)).exp();
        b_a_ += delta_x.segment<3>(AccBias);
        b_g_ += delta_x.segment<3>(GyroBias);
    }
    /**
     * @brief 로봇이 정지한 상태에서 IMU 데이터를 받아 필터의 초기 상태를 설정합니다.
     * @param imu_samples 로봇이 정지해 있는 동안 수집된 IMU 데이터 샘플들
     */
    void initialize_from_stationary(const std::vector<custom_types::ImuData>& imu_samples) {
        if (imu_samples.empty()) {
            // getLogger()->warn("Cannot initialize IEKF with empty IMU samples.");
            return;
        }

        // 1. 데이터 평균 계산
        Eigen::Vector3d avg_accel = Eigen::Vector3d::Zero();
        Eigen::Vector3d avg_gyro = Eigen::Vector3d::Zero();
        for (const auto& data : imu_samples) {
            avg_accel += data.accel;
            avg_gyro += data.gyro;
        }
        avg_accel /= imu_samples.size();
        avg_gyro /= imu_samples.size();

        // 2. 자이로 바이어스 초기화
        b_g_ = avg_gyro;
        
        // 3. 초기 자세(Tilt) 계산
        Eigen::Quaterniond q_init = Eigen::Quaterniond::FromTwoVectors(-avg_accel, g_);
        R_ = manif::SO3d(q_init);

        // 4. 가속도계 바이어스 계산
        b_a_ = avg_accel + R_.rotation().inverse() * g_;
        
        // 초기화 후 last_unbiased_gyro_도 업데이트
        last_unbiased_gyro_ = Eigen::Vector3d::Zero(); // 정지 상태이므로 0

        // getLogger()->info("IEKF initialized from {} IMU samples.", imu_samples.size());
        // getLogger()->info("Initial Gyro Bias: {:.4f}, {:.4f}, {:.4f}", b_g_.x(), b_g_.y(), b_g_.z());
        // getLogger()->info("Initial Accel Bias: {:.4f}, {:.4f}, {:.4f}", b_a_.x(), b_a_.y(), b_a_.z());
    }

    // 상태 접근자
    const manif::SO3d& getOrientation() const { return R_; }
    const Eigen::Vector3d& getVelocity() const { return v_; }
    const Eigen::Vector3d& getPosition() const { return p_; }
    const Eigen::Vector3d& getUnbiasedGyro() const { return last_unbiased_gyro_; }

private:
    // --- 공칭 상태 (Nominal State) ---
    manif::SO3d R_;         // Orientation (Body to World)
    Eigen::Vector3d v_;     // Velocity (in World Frame)
    Eigen::Vector3d p_;     // Position (in World Frame)
    Eigen::Vector3d b_a_;   // Accelerometer bias
    Eigen::Vector3d b_g_;   // Gyroscope bias
    Eigen::Vector3d last_unbiased_gyro_; // 이전 스텝의 자이로 값 저장용

    // --- 에러 상태 공분산 (Error-State Covariance) ---
    Eigen::Matrix<double, 15, 15> P_;

    // --- 노이즈 파라미터 ---
    Eigen::Matrix<double, 12, 12> Q_noise_;    // 프로세스 노이즈
    Eigen::Matrix<double, 2, 2> R_noise_base_; // 측정 노이즈의 기본값

    // --- 상수 및 헬퍼 ---
    const Eigen::Vector3d g_;
    Eigen::Vector3d p_IMU_in_body_;
};


namespace task_pool {

    class IEKFOdometry : public ITask {
    private:
        // [신규] 오도메트리 태스크의 상태 정의
        enum class OdometryState {
            INITIALIZING,
            RUNNING
        };

    public:
        const char* getName() const override { return "IEKFOdometry"; }

        void setup(TaskRegistry& r) override {
            r.add_dependency(dw_pose);
            r.add_dependency(dw_twist);
            r.add_dependency(dr_imu);
            r.add_dependency(dr_velocity);            
            r.add_dependency(p_dt);
            r.add_dependency(p_odom_vel_noise);
            r.add_dependency(p_acc_noise);
            r.add_dependency(p_gyro_noise);
            r.add_dependency(p_acc_bias_noise);
            r.add_dependency(p_gyro_bias_noise);
        }

        void execute() override {
            // [변경] 상태에 따라 다른 로직을 수행하도록 switch 문 사용
            switch (state_) {
                case OdometryState::INITIALIZING:
                    execute_initializing();
                    break;
                case OdometryState::RUNNING:
                    execute_running();
                    break;
            }
        }

    private:
        void execute_initializing() {
            // 목표: 로봇이 정지한 상태에서 IMU 데이터를 수집하여 IEKF를 초기화한다.
            
            dr_velocity.on_update([&](const manif::SE3Tangentd& vel) {
                // 로봇이 움직이는지 확인. 속도가 0에 가까워야 함.
                const double linear_vel_norm = vel.lin().norm();
                const double angular_vel_norm = vel.ang().norm();
                
                if (linear_vel_norm > 0.01 || angular_vel_norm > 0.01) {
                    // 로봇이 움직이면 초기화 샘플을 리셋하고 다시 수집 시작
                    if (!imu_init_samples_.empty()) {
                        getLogger()->warn("[{}] Robot is moving. Resetting initialization.", getName());
                        imu_init_samples_.clear();
                    }
                }
            });

            dr_imu.on_update([&](const custom_types::ImuData& data) {
                imu_init_samples_.push_back(data);

                PERIODIC_CALL(
                    getLogger()->info("[{}] Initializing... Collected IMU samples [{}/{}]", 
                                      getName(), imu_init_samples_.size(), NUM_INIT_SAMPLES);
                , 500ms);

                // 정해진 샘플 개수가 모이면 초기화 수행 및 상태 전환
                if (imu_init_samples_.size() >= NUM_INIT_SAMPLES) {
                    getLogger()->info("[{}] Collected enough samples. Initializing IEKF...", getName());
                    _iekf.initialize_from_stationary(imu_init_samples_);
                    
                    imu_init_samples_.clear(); // 메모리 정리
                    state_ = OdometryState::RUNNING; // 상태를 RUNNING으로 전환!
                    
                    getLogger()->info("[{}] Initialization complete. Starting odometry estimation.", getName());
                }
            });
        }

        void execute_running() {
            acc_noise = p_acc_noise.read();
            gyro_noise = p_gyro_noise.read();
            acc_bias_noise = p_acc_bias_noise.read();
            gyro_bias_noise = p_gyro_bias_noise.read();
            odom_vel_noise = p_odom_vel_noise.read();
            
            // 목표: IMU와 휠 오도메트리를 이용해 정상적인 상태 추정을 수행한다.
            // 이 로직은 기존 execute() 함수의 내용과 동일합니다.
            bool imu_update = false;
            bool wheel_update = false;

            dr_imu.on_update([&](const custom_types::ImuData& data) {
                imu_update = true;
                _last_imu = data;
            });

            dr_velocity.on_update([&](const manif::SE3Tangentd& data) {
                wheel_update = true;
                _last_vel = data;
            });
            
            if (imu_update) {
                _iekf.predict(_last_imu, p_dt.read());
                if (wheel_update) {
                    _iekf.update(_last_vel, _last_imu);
                    
                    // 결과 발행
                    manif::SE3d pose(_iekf.getPosition(), _iekf.getOrientation());
                    manif::SE3Tangentd spatial_twist;
                    spatial_twist.lin() = _iekf.getVelocity();
                    spatial_twist.ang() = _iekf.getUnbiasedGyro();
                    dw_pose.write(pose);
                    dw_twist.write(spatial_twist);

                    PERIODIC_CALL(
                        auto aa = Eigen::AngleAxisd(pose.quat());
                        auto rodrig = aa.axis() * aa.angle();
                        getLogger()->info("[{}] position=[{:.03f}, {:.03f}, {:.03f}]", getName(), pose.translation().x(), pose.translation().y(), pose.translation().z());
                        getLogger()->info("[{}] orient=[{:.03f}, {:.03f}, {:.03f}]", getName(), rodrig.x(), rodrig.y(), rodrig.z());
                        getLogger()->info("[{}] lin_vel=[{:.03f}, {:.03f}, {:.03f}]", getName(), spatial_twist.lin().x(), spatial_twist.lin().y(), spatial_twist.lin().z());
                        getLogger()->info("[{}] ang_vel=[{:.03f}, {:.03f}, {:.03f}]", getName(), spatial_twist.ang().x(), spatial_twist.ang().y(), spatial_twist.ang().z());
                    , 1s);
                }
            } else {
                PERIODIC_CALL(
                    getLogger()->warn("[{}] imu data not available!", getName());
                , 1s);
            }
        }

    private:
        DataWriter<manif::SE3d> dw_pose{"odom/pose"};
        DataWriter<manif::SE3Tangentd> dw_twist{"odom/spatial_twist"};
        DataReader<custom_types::ImuData> dr_imu{"imu"};
        DataReader<manif::SE3Tangentd> dr_velocity{"velocity_pv"};
    
        Parameter<double> p_dt{"param.mecanum_iekf.dt", 0.01};
        Parameter<double> p_odom_vel_noise{"param.mecanum_iekf.odom_vel_noise", 1e-6};//0.05;
        Parameter<double> p_acc_noise{"param.mecanum_iekf.acc_noise", 1.0};    // m/s^2/sqrt(Hz)
        Parameter<double> p_gyro_noise{"param.mecanum_iekf.gyro_noise", 0.1};   // rad/s/sqrt(Hz)
        Parameter<double> p_acc_bias_noise{"param.mecanum_iekf.acc_bias_noise", 0.01};
        Parameter<double> p_gyro_bias_noise{"param.mecanum_iekf.gyro_bias_noise", 0.001};

    private:
        MecanumIEKF _iekf;
        custom_types::ImuData _last_imu;
        manif::SE3Tangentd _last_vel;

        // [신규] 상태 머신 관련 멤버 변수
        OdometryState state_ = OdometryState::INITIALIZING;
        std::vector<custom_types::ImuData> imu_init_samples_;
        const size_t NUM_INIT_SAMPLES = 200; // 100Hz에서 2초 분량
    };
};