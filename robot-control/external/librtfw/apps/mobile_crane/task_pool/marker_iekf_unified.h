#pragma once

#include "rtfw/task.h"
#include "eigen3/Eigen/Dense"
#include <manif/manif.h>
#include <cmath>
#include <vector>
#include <numeric>

// =================================================================================
// 통합 마커 필터 네임스페이스
// =================================================================================

namespace UnifiedMarkerFilter {
namespace SE3 {

// ======================= 타입/차원 =======================
static constexpr int POSE_DIM = 6;
static constexpr int VEL_DIM  = 6;
static constexpr int STATE_DIM = POSE_DIM + VEL_DIM; // CV: {pose, vel}
static constexpr int MEAS_DIM  = 6;

using StateVector = Eigen::Matrix<double, STATE_DIM, 1>;         // 12x1
using StateMatrix = Eigen::Matrix<double, STATE_DIM, STATE_DIM>; // 12x12
using MeasVec     = Eigen::Matrix<double, MEAS_DIM , 1>;         // 6x1
using MeasMat     = Eigen::Matrix<double, MEAS_DIM , MEAS_DIM>;  // 6x6
using GainMat     = Eigen::Matrix<double, STATE_DIM, MEAS_DIM>;  // 12x6
using Hmat        = Eigen::Matrix<double, MEAS_DIM , STATE_DIM>; // 6x12

// ======================= 추정기 (CV + FOH + 분산혁신) =======================
class Estimator {
public:
  struct State {
    manif::SE3d        pose;        // W_X
    manif::SE3Tangentd world_vel;   // se(3), W_V
  };

  Estimator() {
    P_.setIdentity();
    P_.block<6,6>(0,0) *= 1.0;   // pose
    P_.block<6,6>(6,6) *= 10.0;  // vel
    last_dt_ = 0.01; // 기본 100 Hz 가정 (필요 시 덮어씀)
  }

  // ---------- CV 예측: v 랜덤워크 (Qc_vel = 6x6, 연속 스펙트럼 밀도) ----------
  void predict(double dt, const Eigen::Matrix<double,6,6>& Qc_vel) {
    last_dt_ = dt;

    // (1) 상태 전파: pose_{k+1} = exp(v*dt) * pose_k, vel_{k+1} = vel_k
    const auto v = state_.world_vel;
    state_.pose  = (v * dt).exp() * state_.pose;

    // (2) 선형화 F
    StateMatrix F = StateMatrix::Identity();
    F.block<6,6>(0,6) = Eigen::Matrix<double,6,6>::Identity() * dt;

    // (3) 이산화 잡음 (CV 표준)
    const double dt2 = dt*dt, dt3 = dt2*dt;
    StateMatrix Qd = StateMatrix::Zero();
    Qd.block<6,6>(0,0)   = Qc_vel * (dt3/3.0);
    Qd.block<6,6>(0,6)   = Qc_vel * (dt2/2.0);
    Qd.block<6,6>(6,0)   = Qc_vel * (dt2/2.0);
    Qd.block<6,6>(6,6)   = Qc_vel *  dt;

    P_ = F * P_ * F.transpose() + Qd;
    symmetrize_(P_);
  }

  // ---------- 포즈 업데이트 (비전/의사측정 공용) ----------
  void updatePoseVision(const manif::SE3d& Z, const MeasMat& R) {
    const State      prev_state = state_;
    const StateMatrix prevP      = P_;

    // 혁신 y = log(Z * X^{-1})
    MeasVec y = (Z * state_.pose.inverse()).log().coeffs();

    // H = [I6, 0]
    Hmat H = Hmat::Zero();
    H.block<6,6>(0,0).setIdentity();

    // 칼만 이득
    MeasMat S = H * P_ * H.transpose() + R;
    S += MeasMat::Identity() * 1e-9;
    GainMat K = P_ * H.transpose() * S.inverse();

    // 보정
    StateVector dx = K * y;
    state_.pose      = manif::SE3Tangentd(dx.head<6>()).exp() * state_.pose;
    state_.world_vel += manif::SE3Tangentd(dx.tail<6>());

    // Joseph form
    const StateMatrix I = StateMatrix::Identity();
    StateMatrix I_KH = I - K * H;
    P_ = I_KH * P_ * I_KH.transpose() + K * R * K.transpose();

    // 가드
    if (!P_.allFinite() || !state_.pose.coeffs().allFinite()) {
      state_ = prev_state; P_ = prevP; 
    }
    state_.pose.normalize();
    symmetrize_(P_);

    // using Vec6 = Eigen::Matrix<double,6,1>;
    // const auto state_bak = state_;
    // const auto P_bak     = P_;

    // auto joseph = [&](const GainMat& K, const Hmat& H, const MeasMat& Rm){
    //     const StateMatrix I = StateMatrix::Identity();
    //     const auto IKH = I - K*H;
    //     P_ = IKH*P_*IKH.transpose() + K*Rm*K.transpose();
    //     P_ = 0.5*(P_ + P_.transpose());
    // };

    // // 공통 H
    // Hmat H; H.setZero();
    // H.block<6,6>(0,0).setIdentity();

    // // -------- 1) ROT only --------
    // {
    //     Vec6 y = (Z * state_.pose.inverse()).log().coeffs();
    //     // rot 성분만 사용
    //     Vec6 y_rot = Vec6::Zero(); y_rot.tail<3>() = y.tail<3>();

    //     // R 마스킹: pos에는 큰 수(=신뢰 안 함)
    //     MeasMat Rrot = MeasMat::Zero();
    //     Rrot.diagonal() << 1e8,1e8,1e8,  R(3,3),R(4,4),R(5,5);

    //     MeasMat S = H*P_*H.transpose() + Rrot; S += MeasMat::Identity()*1e-9;
    //     GainMat K = P_*H.transpose()*S.inverse();

    //     // 보정
    //     StateVector dx = K * y_rot;
    //     state_.pose      = manif::SE3Tangentd( dx.head<6>() ).exp() * state_.pose;
    //     state_.world_vel += manif::SE3Tangentd( dx.tail<6>() );

    //     joseph(K,H,Rrot);
    //     state_.pose.normalize();
    // }

    // // -------- 2) re-linearize & POS only --------
    // {
    //     Vec6 y = (Z * state_.pose.inverse()).log().coeffs();
    //     Vec6 y_pos = Vec6::Zero(); y_pos.head<3>() = y.head<3>();

    //     MeasMat Rpos = MeasMat::Zero();
    //     Rpos.diagonal() << R(0,0),R(1,1),R(2,2), 1e8,1e8,1e8;

    //     MeasMat S = H*P_*H.transpose() + Rpos; S += MeasMat::Identity()*1e-9;
    //     GainMat K = P_*H.transpose()*S.inverse();

    //     StateVector dx = K * y_pos;
    //     state_.pose      = manif::SE3Tangentd( dx.head<6>() ).exp() * state_.pose;
    //     state_.world_vel += manif::SE3Tangentd( dx.tail<6>() );

    //     joseph(K,H,Rpos);
    //     state_.pose.normalize();
    // }

    // if (!P_.allFinite() || !state_.pose.coeffs().allFinite()){
    //     state_ = state_bak; P_ = P_bak;
    // }
  }

  // ========================= FOH + 분산혁신 지원 =========================
  struct VisionFrame {
    double t;       // [s]
    manif::SE3d Z;  // 측정 포즈 (W_X)
    MeasMat    R;   // 6x6 분산
  };

  // 25 Hz 프레임 들어올 때 호출 (실제 측정 등록)
  void pushVisionFrame(double t, const manif::SE3d& Z, const MeasMat& R) {
    z_prev_ = z_curr_;
    z_curr_ = VisionFrame{t, Z, R};

    // 분산혁신 준비: 다음 카메라 주기 동안 M등분
    if (use_spread_ && z_prev_) {
      const double T = std::max(1e-6, z_curr_->t - z_prev_->t);
      const int M = std::max(1, int(std::round(T / std::max(1e-6,last_dt_))));
      spread_meas_ = *z_curr_;
      spread_steps_left_ = M;
      // 정보량 보존: 한 번에 R 대신, M번에 나눠 R_step = M*scale*R
      spread_R_step_ = spread_meas_.R * (spread_scale_ * double(M));
    }

    if (!use_spread_ && !use_foh_)
        updatePoseVision(Z, R);
  }

  // 매 100 Hz에서 호출: 예측 후
  // - FOH 보간 pseudo-measurement 업데이트
  // - (옵션) 분산혁신 1스텝 적용
  void updateFOHAndSpread(double t_now) {
    t_now_ = t_now;

    // (1) FOH 보간 의사측정
    if (use_foh_ && z_prev_ && z_curr_) {
      const double t1 = z_prev_->t, t2 = z_curr_->t;
      if (t_now_ >= t1 && t_now_ <= t2) {
        const double T   = std::max(1e-9, t2 - t1);
        const double tau = t_now_ - t1;
        const auto Xi    = (z_prev_->Z.inverse() * z_curr_->Z).log() * (1.0 / T);
        const auto Zfoh  = (Xi * tau).exp() * z_prev_->Z;

        MeasMat Rfoh = z_curr_->R * foh_R_scale_; // pseudo는 약하게
        updatePoseVision(Zfoh, Rfoh);
      }
    }

    // (2) 분산혁신 1 스텝
    if (use_spread_ && spread_steps_left_ > 0) {
      updatePoseVision(spread_meas_.Z, spread_R_step_);
      --spread_steps_left_;
    }
  }

  // ---------- 설정/토글 ----------
  void setFOHStrength(bool on, double r_scale) {
    use_foh_ = on; 
    foh_R_scale_ = std::max(1.0, r_scale); 
  }
  void enableInnovationSpreading(bool on, double scale = 1.0) {
    use_spread_   = on;
    spread_scale_ = std::max(1.0, scale);
  }

  // ---------- 상태 접근/설정 ----------
  const State& getState() const { return state_; }

  bool setState(const manif::SE3d& init_pose) {
    state_.pose = init_pose; state_.pose.normalize();
    if (!state_.pose.coeffs().allFinite()) return false;
    if (state_.pose.quat().norm() < 0.9)    return false;
    state_.world_vel.setZero();
    return true;
  }

private:
  // 대칭화 유틸
  static void symmetrize_(StateMatrix& P) { P = (P + P.transpose()) * 0.5; }

private:
  // 상태/공분산
  State       state_;
  StateMatrix P_;

  // 시간/옵션
  double t_now_   = 0.0;
  double last_dt_ = 0.01;

  // FOH 자료구조
  bool   use_foh_ = false;
  std::optional<VisionFrame> z_prev_;
  std::optional<VisionFrame> z_curr_;
  double foh_R_scale_ = 4.0;  // FOH pseudo 측정은 약하게 (R' = scale*R)

  // 혁신 분산
  bool   use_spread_ = false;
  int    spread_steps_left_ = 0;
  MeasMat    spread_R_step_ = MeasMat::Identity();
  VisionFrame spread_meas_{};
  double spread_scale_ = 1.5; // 1.0=정보량 보존, >1.0=더 약하게
};

} // namespace SE3
} // namespace UnifiedMarkerFilter


namespace task_pool {
    class MarkerIEKFTask : public ITask {
    private:
        enum class FilterState {
            WAITING_FOR_FIRST_SIGHTING,
            RUNNING
        };

    public:
        const char* getName() const override { return "MarkerIEKFTask"; }

        void setup(TaskRegistry& r) override {            
            for (int i=0; i<4; i++) {
                r.add_dependency(dr_pose[i]);
                r.add_dependency(dw_fpose[i]);
                r.add_dependency(dw_fvel[i]);
                r.add_dependency(dw_facc[i]);
            }
            // 파라미터 의존성 등록
            r.add_dependency(param.body.r_pos_noise);
            r.add_dependency(param.body.r_rot_noise);
            r.add_dependency(param.body.r_vel_noise);
            r.add_dependency(param.body.q_linear_accel_noise);
            r.add_dependency(param.body.q_angular_accel_noise);
            r.add_dependency(param.body.use_foh);
            r.add_dependency(param.body.foh_strength);
            r.add_dependency(param.body.spread_gain);
            r.add_dependency(param.left_foot.r_pos_noise);
            r.add_dependency(param.left_foot.r_rot_noise);
            r.add_dependency(param.left_foot.r_vel_noise);
            r.add_dependency(param.left_foot.q_linear_accel_noise);
            r.add_dependency(param.left_foot.q_angular_accel_noise);
            r.add_dependency(param.left_foot.use_foh);
            r.add_dependency(param.left_foot.foh_strength);
            r.add_dependency(param.left_foot.spread_gain);
            r.add_dependency(param.right_foot.r_pos_noise);
            r.add_dependency(param.right_foot.r_rot_noise);
            r.add_dependency(param.right_foot.r_vel_noise);
            r.add_dependency(param.right_foot.q_linear_accel_noise);
            r.add_dependency(param.right_foot.q_angular_accel_noise);
            r.add_dependency(param.right_foot.use_foh);
            r.add_dependency(param.right_foot.foh_strength);
            r.add_dependency(param.right_foot.spread_gain);
        }

        void initialize() override {
            for (int i=0; i<4; i++) {
                states_[i] = FilterState::WAITING_FOR_FIRST_SIGHTING;
                
                bool use_foh;
                double foh_r_gain, spread_gain;
                switch (i) {
                    case 0:
                    case 3:
                        use_foh = param.body.use_foh.read();
                        foh_r_gain = param.body.foh_strength.read();
                        spread_gain = param.body.spread_gain.read();
                        break;
                    case 1:
                        use_foh = param.left_foot.use_foh.read();
                        foh_r_gain = param.left_foot.foh_strength.read();
                        spread_gain = param.left_foot.spread_gain.read();
                        break;
                    case 2:
                        use_foh = param.right_foot.use_foh.read();
                        foh_r_gain = param.right_foot.foh_strength.read();
                        spread_gain = param.right_foot.spread_gain.read();
                        break;
                    default:
                        use_foh = false;
                        spread_gain = 0;
                }
                if (use_foh)
                    estimators_[i].setFOHStrength(use_foh, foh_r_gain);
                if (spread_gain > 0)
                    estimators_[i].enableInnovationSpreading(true, spread_gain);
                marker_lost[i] = 0;
            }
        }

        void execute() override {
            // --- Vision 측정 ---
            manif::SE3d marker_meas[4];
            bool marker_updated[4] = {false, false, false, false};

            for (auto i=0; i<4; i++) {
                dr_pose[i].on_update([i, &marker_meas, &marker_updated](const manif::SE3d& data) {
                    marker_meas[i] = data;
                    marker_updated[i] = true;
                });

                if (!marker_updated[i]) 
                    marker_lost[i]++; 
                else 
                    marker_lost[i] = 0;
            }

            for (int i=0; i<4; ++i) {
                if (states_[i] == FilterState::WAITING_FOR_FIRST_SIGHTING) {
                    if (estimators_[i].setState(marker_meas[i]))
                        states_[i] = FilterState::RUNNING;
                    continue;
                }

                if (marker_lost[i] > 100) {
                    states_[i] = FilterState::WAITING_FOR_FIRST_SIGHTING;
                    continue;
                }

                // ---------------- RUNNING ----------------
                // 파라미터 읽기
                double r_pos, r_rot, r_vel, q_lin, q_ang;
                bool use_foh;
                switch(i) {
                    case 0: case 3:
                        r_pos = param.body.r_pos_noise.read();
                        r_rot = param.body.r_rot_noise.read();
                        r_vel = param.body.r_vel_noise.read();
                        q_lin = param.body.q_linear_accel_noise.read();
                        q_ang = param.body.q_angular_accel_noise.read();
                        use_foh = param.body.use_foh.read();
                        break;
                    case 1:
                        r_pos = param.left_foot.r_pos_noise.read();
                        r_rot = param.left_foot.r_rot_noise.read();
                        r_vel = param.left_foot.r_vel_noise.read();
                        q_lin = param.left_foot.q_linear_accel_noise.read();
                        q_ang = param.left_foot.q_angular_accel_noise.read();
                        use_foh = param.left_foot.use_foh.read();
                        break;
                    case 2:
                        r_pos = param.right_foot.r_pos_noise.read();
                        r_rot = param.right_foot.r_rot_noise.read();
                        r_vel = param.right_foot.r_vel_noise.read();
                        q_lin = param.right_foot.q_linear_accel_noise.read();
                        q_ang = param.right_foot.q_angular_accel_noise.read();
                        use_foh = param.right_foot.use_foh.read();
                        break;
                }

                // Qc_accel (jerk 잡음)
                Eigen::Matrix<double,6,6> Qc_accel = Eigen::Matrix<double,6,6>::Zero();
                Qc_accel(0,0)=q_lin; Qc_accel(1,1)=q_lin; Qc_accel(2,2)=q_lin;
                Qc_accel(3,3)=q_ang; Qc_accel(4,4)=q_ang; Qc_accel(5,5)=q_ang;

                const auto t_wall = ((double)getCurrentTick()) / getFrequency();
                const auto dt = 1.0 / getFrequency();

                // Predict (100 Hz)
                estimators_[i].predict(dt, Qc_accel);

                // Vision 업데이트 (25 Hz)
                if (marker_updated[i]) {
                    manif::SE3d& W_X_meas = marker_meas[i];
                    Eigen::Matrix<double,6,6> R_pose = Eigen::Matrix<double,6,6>::Identity();
                    R_pose.diagonal() << r_pos, r_pos, r_pos, r_rot, r_rot, r_rot;

                    estimators_[i].pushVisionFrame(t_wall, W_X_meas, R_pose);
                }

                estimators_[i].updateFOHAndSpread(t_wall);

                // 상태 publish
                auto st = estimators_[i].getState();
                dw_fpose[i].write(st.pose);
                dw_fvel[i].write(st.world_vel);
            }
        }

    private:
        DataReader<manif::SE3d> dr_pose[4] = {
            DataReader<manif::SE3d>{"lpf/g_pose_body"},
            DataReader<manif::SE3d>{"lpf/g_pose_left_foot"},
            DataReader<manif::SE3d>{"lpf/g_pose_right_foot"},
            DataReader<manif::SE3d>{"lpf/g_pose_target"}
        };
        DataWriter<manif::SE3d> dw_fpose[4] = {            
            DataWriter<manif::SE3d>{"marker_iekf/pose_body"},
            DataWriter<manif::SE3d>{"marker_iekf/pose_left_foot"},            
            DataWriter<manif::SE3d>{"marker_iekf/pose_right_foot"},
            DataWriter<manif::SE3d>{"marker_iekf/pose_target"}
        };
        DataWriter<manif::SE3Tangentd> dw_fvel[4] = {            
            DataWriter<manif::SE3Tangentd>{"marker_iekf/vel_body"},
            DataWriter<manif::SE3Tangentd>{"marker_iekf/vel_left_foot"},            
            DataWriter<manif::SE3Tangentd>{"marker_iekf/vel_right_foot"},
            DataWriter<manif::SE3Tangentd>{"marker_iekf/vel_target"}
        };
        DataWriter<manif::SE3Tangentd> dw_facc[4] = {        
            DataWriter<manif::SE3Tangentd>{"marker_iekf/acc_body"},
            DataWriter<manif::SE3Tangentd>{"marker_iekf/acc_left_foot"},            
            DataWriter<manif::SE3Tangentd>{"marker_iekf/acc_right_foot"},
            DataWriter<manif::SE3Tangentd>{"marker_iekf/acc_target"}
        };
        struct {
            struct {
                Parameter<double> r_pos_noise {"param.marker_iekf.body.r_pos_noise", 1e-3};
                Parameter<double> r_rot_noise {"param.marker_iekf.body.r_rot_noise", 1e-3};
                Parameter<double> r_vel_noise {"param.marker_iekf.body.r_vel_noise", 1e-3};
                Parameter<double> q_linear_accel_noise {"param.marker_iekf.body.q_linear_accel_noise", 1e0};
                Parameter<double> q_angular_accel_noise {"param.marker_iekf.body.q_angular_accel_noise", 1e0};
                Parameter<bool> use_foh {"param.marker_iekf.body.use_foh", false};
                Parameter<double> foh_strength {"param.marker_iekf.body.foh_strength", 4.0};
                Parameter<double> spread_gain {"param.marker_iekf.body.spread_gain", 1.0};
            } body;
            struct {
                Parameter<double> r_pos_noise {"param.marker_iekf.left_foot.r_pos_noise", 1e-3};
                Parameter<double> r_rot_noise {"param.marker_iekf.left_foot.r_rot_noise", 1e-3};
                Parameter<double> r_vel_noise {"param.marker_iekf.left_foot.r_vel_noise", 1e-3};
                Parameter<double> q_linear_accel_noise {"param.marker_iekf.left_foot.q_linear_accel_noise", 1e0};
                Parameter<double> q_angular_accel_noise {"param.marker_iekf.left_foot.q_angular_accel_noise", 1e0};
                Parameter<bool> use_foh {"param.marker_iekf.left_foot.use_foh", false};
                Parameter<double> foh_strength {"param.marker_iekf.left_foot.foh_strength", 4.0};
                Parameter<double> spread_gain {"param.marker_iekf.left_foot.spread_gain", 1.0};
            } left_foot;
            struct {
                Parameter<double> r_pos_noise {"param.marker_iekf.right_foot.r_pos_noise", 1e-3};
                Parameter<double> r_rot_noise {"param.marker_iekf.right_foot.r_rot_noise", 1e-3};
                Parameter<double> r_vel_noise {"param.marker_iekf.right_foot.r_vel_noise", 1e-3};
                Parameter<double> q_linear_accel_noise {"param.marker_iekf.right_foot.q_linear_accel_noise", 1e0};
                Parameter<double> q_angular_accel_noise {"param.marker_iekf.right_foot.q_angular_accel_noise", 1e0};
                Parameter<bool> use_foh {"param.marker_iekf.right_foot.use_foh", false};
                Parameter<double> foh_strength {"param.marker_iekf.right_foot.foh_strength", 4.0};
                Parameter<double> spread_gain {"param.marker_iekf.right_foot.spread_gain", 1.0};
            } right_foot;
        } param;

    private:
        UnifiedMarkerFilter::SE3::Estimator estimators_[4];
        FilterState states_[4];

        uint64_t marker_lost[4];
    };
};