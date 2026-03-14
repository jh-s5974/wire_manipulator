#include "policy_runner.hpp"

#include <torch/script.h>
#include <ATen/Parallel.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <stdexcept>
#include "util.hpp"

namespace task_pool {

struct PolicyRunner::Impl {
    torch::jit::script::Module policy_pt;
    torch::jit::script::Module stand_pt;
    torch::Tensor input_tensor;  // pre-allocated, reused every inference

    static std::string resolve_model_path(const std::string& raw_path) {
        namespace fs = std::filesystem;

        const std::vector<fs::path> candidates = {
            fs::path(raw_path),
            fs::path("src/mujoco_rl") / raw_path,
            fs::path("./") / raw_path,
        };

        for (const auto& candidate : candidates) {
            if (fs::exists(candidate)) {
                return fs::absolute(candidate).string();
            }
        }

        throw std::runtime_error("Model file not found: " + raw_path);
    }
};

void PolicyRunner::initialize(PolicyRunnerState& s) {
    s.base_quat = Eigen::Quaterniond::Identity();
    s.base_ang_vel.setZero();
    s.cmd_vel.setZero();
    s.imu_online = false;
    s.activated = false;
    s.motor_online.fill(false);
    s.cnt = 0;

    joint_names_ = p_joint_names.read();
    if (joint_names_.empty()) {
        joint_names_ = {
            "hip_yaw_left", "hip_yaw_right",
            "hip_roll_left", "hip_roll_right",
            "hip_pitch_left", "hip_pitch_right",
            "knee_left", "knee_right",
            "ankle_pitch_left", "ankle_pitch_right",
            "ankle_roll_left", "ankle_roll_right",
        };
    }

    default_joint_positions_ = p_default_joint_positions.read();
    joint_kp_ = p_joint_kp.read();
    joint_kd_ = p_joint_kd.read();

    num_observations_ = p_num_observations.read();
    num_actions_ = p_num_actions.read();
    history_length_ = p_history_length.read();
    action_limit_lower_ = static_cast<float>(p_action_limit_lower.read());
    action_limit_upper_ = static_cast<float>(p_action_limit_upper.read());
    action_scale_ = static_cast<float>(p_action_scale.read());
    clock_hz_ = p_clock_hz.read();

    if (joint_names_.size() != 12 ||
        default_joint_positions_.size() != static_cast<size_t>(num_actions_) ||
        joint_kp_.size() != static_cast<size_t>(num_actions_) ||
        joint_kd_.size() != static_cast<size_t>(num_actions_)) {
        throw std::runtime_error("[" + std::string(getName()) + "] PolicyRunner parameter size mismatch");
    }

    const size_t history_size = static_cast<size_t>(num_observations_) * static_cast<size_t>(history_length_ + 1);
    policy_observations_.assign(history_size, 0.0f);
    pre_actions_.assign(num_actions_, 0.0f);
    actions_scaled_.assign(num_actions_, 0.0f);
    actions_raw_.assign(num_actions_, 0.0f);
    actions_clipped_.assign(num_actions_, 0.0f);
    single_observation_.assign(num_observations_, 0.0f);

    const std::string policy_path = Impl::resolve_model_path(p_policy_checkpoint_path.read());
    const std::string stand_path = Impl::resolve_model_path(p_policy_stand_path.read());

    // LibTorch: for batch=1 small MLP, single thread is much faster than multi-thread
    // (multi-thread overhead >> compute time for tiny networks)
    at::set_num_threads(1);
    at::set_num_interop_threads(1);

    impl_ = std::make_shared<Impl>();
    try {
        impl_->policy_pt = torch::jit::load(policy_path);
        impl_->stand_pt = torch::jit::load(stand_path);
        impl_->policy_pt.eval();
        impl_->stand_pt.eval();
    } catch (const c10::Error& e) {
        throw std::runtime_error(std::string("Failed to load torch models: ") + e.what_without_backtrace());
    }

    // Pre-allocate input tensor (reused every inference to avoid allocation overhead)
    const long obs_size = static_cast<long>(policy_observations_.size());
    impl_->input_tensor = torch::zeros({1, obs_size}, torch::TensorOptions().dtype(torch::kFloat32));

    // Warmup: run dummy inference so TorchScript JIT compiles on first call
    {
        at::InferenceMode guard;
        impl_->policy_pt.forward({impl_->input_tensor});
        impl_->stand_pt.forward({impl_->input_tensor});
    }

    getLogger()->info("PolicyRunner initialized. policy={} stand={}", policy_path, stand_path);
}

void PolicyRunner::execute(PolicyRunnerState& s) {
    dr_rl_signal.on_update([&](const bool& on) {
        if (on != s.activated) {
            s.activated = on;
            getLogger()->info("[{}] RL signal {}", getName(), on ? "ON" : "OFF");
        }
    });


    dr_cmd_vel.on_update([&](const manif::SE2Tangentd& cmd) {
        s.cmd_vel << cmd.x(), cmd.y(), cmd.angle();
    });

    dr_imu.on_update([&](const custom_types::Imu& imu) {
        s.base_ang_vel << imu.gyro.x, imu.gyro.y, imu.gyro.z;
        s.base_quat = Eigen::Quaterniond(
            imu.orientation.w,
            imu.orientation.x,
            imu.orientation.y,
            imu.orientation.z
        );
        s.imu_online = true;
    });

    for (int i = 0; i < 12; ++i) {
        dr_mtr_stat[i].on_update([&, i](const custom_types::MotorState& data) {
            s.motor[i] = data;
            s.motor_online[i] = true;
        });
    }

    if (!s.activated) {
        return;
    }

    if (!input_ready(s)) {
        PERIODIC_CALL(
            getLogger()->warn("[{}] PolicyRunner waiting for IMU/Motor inputs...", getName());
        , 1s);
        return;
    }

    if (s.cnt++ % 4 > 0) {
        return;
    }
    build_observation(s);
    if (single_observation_.size() != static_cast<size_t>(num_observations_)) {
        PERIODIC_CALL(
            getLogger()->error("[{}] Observation size mismatch: got {}, expected {}", getName(), single_observation_.size(), num_observations_);
        , 1s);
        return;
    }

    policy_observations_.erase(policy_observations_.begin(), policy_observations_.begin() + num_observations_);
    policy_observations_.insert(policy_observations_.end(), single_observation_.begin(), single_observation_.end());

    // auto tic = std::chrono::steady_clock::now();
    run_inference(s.cmd_vel);
    // auto toc = std::chrono::steady_clock::now();
    // getLogger()->info("[{}] Inference time: {} ms", getName(), std::chrono::duration_cast<std::chrono::milliseconds>(toc - tic).count());

    for (int i = 0; i < num_actions_; ++i) {
        actions_clipped_[i] = std::clamp(actions_raw_[i], action_limit_lower_, action_limit_upper_);
        pre_actions_[i] = actions_clipped_[i];
        actions_scaled_[i] = actions_clipped_[i] * action_scale_ + static_cast<float>(default_joint_positions_[i]);
    }

    for (int i = 0; i < num_actions_; ++i) {
        custom_types::MotorCmd cmd{};
        std::strncpy(cmd.name, joint_names_[i].c_str(), sizeof(cmd.name) - 1);
        cmd.name[sizeof(cmd.name) - 1] = '\0';
        cmd.pos = static_cast<double>(actions_scaled_[i]);
        cmd.vel = 0.0;
        cmd.torque = 0.0;
        cmd.kp = joint_kp_[i];
        cmd.kd = joint_kd_[i];
        cmd.duration_ms = 20;
        dw_action[i].write(cmd);
    }
}

bool PolicyRunner::input_ready(const PolicyRunnerState& s) const {
    if (!s.imu_online) {
        return false;
    }
    return std::all_of(s.motor_online.begin(), s.motor_online.end(), [](bool online) { return online; });
}

void PolicyRunner::build_observation(const PolicyRunnerState& s) {
    single_observation_.clear();
    single_observation_.reserve(num_observations_);

    const double t = static_cast<double>(getExecutionLocalTick()) / static_cast<double>(getFrequency());
    const double phase = 2.0 * M_PI * clock_hz_ * t;

    single_observation_.push_back(static_cast<float>(std::sin(phase)));
    single_observation_.push_back(static_cast<float>(std::cos(phase)));

    single_observation_.push_back(static_cast<float>(s.base_ang_vel.x()));
    single_observation_.push_back(static_cast<float>(s.base_ang_vel.y()));
    single_observation_.push_back(static_cast<float>(s.base_ang_vel.z()));

    const Eigen::Vector3d gravity = s.base_quat.conjugate() * Eigen::Vector3d(0.0, 0.0, -1.0);
    single_observation_.push_back(static_cast<float>(gravity.x()));
    single_observation_.push_back(static_cast<float>(gravity.y()));
    single_observation_.push_back(static_cast<float>(gravity.z()));

    single_observation_.push_back(static_cast<float>(s.cmd_vel.x()));
    single_observation_.push_back(static_cast<float>(s.cmd_vel.y()));
    single_observation_.push_back(static_cast<float>(s.cmd_vel.z()));

    for (int i = 0; i < num_actions_; ++i) {
        single_observation_.push_back(static_cast<float>(s.motor[i].pos - default_joint_positions_[i]));
    }
    for (int i = 0; i < num_actions_; ++i) {
        single_observation_.push_back(static_cast<float>(s.motor[i].vel));
    }
    single_observation_.insert(single_observation_.end(), pre_actions_.begin(), pre_actions_.end());
}

void PolicyRunner::run_inference(const Eigen::Vector3d& cmd_vel) {
    if (!impl_) {
        throw std::runtime_error("[" + std::string(getName()) + "] PolicyRunner impl is not initialized");
    }

    // InferenceMode: more efficient than NoGradGuard (disables full autograd infrastructure)
    at::InferenceMode guard;

    // Copy observation data into pre-allocated tensor (avoid heap allocation every call)
    std::memcpy(impl_->input_tensor.data_ptr<float>(),
                policy_observations_.data(),
                sizeof(float) * policy_observations_.size());

    const float cmd_norm = static_cast<float>(cmd_vel.norm());
    at::Tensor output_tensor;
    if (cmd_norm < 0.1f) {
        output_tensor = impl_->stand_pt.forward({impl_->input_tensor}).toTensor();
    } else {
        output_tensor = impl_->policy_pt.forward({impl_->input_tensor}).toTensor();
    }

    // No .to(kCPU) needed - already CPU-only libtorch
    if (output_tensor.numel() < num_actions_) {
        throw std::runtime_error("[" + std::string(getName()) + "] Policy output size is smaller than num_actions");
    }

    const float* out_ptr = output_tensor.contiguous().data_ptr<float>();
    std::memcpy(actions_raw_.data(), out_ptr, sizeof(float) * static_cast<size_t>(num_actions_));
}

} // namespace task_pool
