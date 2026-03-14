#pragma once

#include <rtfw/task.h>
#include "custom_types.hpp"
#include "util.hpp"
#include "kin_2rsu.hpp"

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>
#include <mujoco/mjvisualize.h>
#include <Eigen/Dense>

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

using namespace std::chrono_literals;
using namespace rtfw;
using namespace rtfw::rt;

namespace task_pool {

struct MujocoEnvState {
    std::array<custom_types::MotorCmd, 12> cmd{};
    std::array<bool, 12> motor_on{};
    std::array<bool, 12> cmd_online{};
};

class EmaFilter {
public:
    EmaFilter() = default;

    void initialize(double cutoff_freq, double dt, int size) {
        const double tau = 1.0 / (2.0 * M_PI * std::max(cutoff_freq, 1e-6));
        alpha_ = dt / (dt + tau);
        value_ = Eigen::VectorXd::Zero(size);
        initialized_ = true;
    }

    Eigen::VectorXd filter(const Eigen::VectorXd& x) {
        if (!initialized_ || value_.size() != x.size()) {
            value_ = x;
            initialized_ = true;
            return value_;
        }
        value_ = alpha_ * x + (1.0 - alpha_) * value_;
        return value_;
    }

private:
    double alpha_ = 1.0;
    bool initialized_ = false;
    Eigen::VectorXd value_;
};

class MujocoEnv : public Task<MujocoEnvState> {
public:
    const char* getName() const override { return "MujocoEnv"; }

    void initialize(MujocoEnvState& s) override {
        policy_dt_ = p_policy_dt.read();
        control_dt_ = p_control_dt.read();
        physics_dt_ = p_physics_dt.read();
        cutoff_freq_ = p_cutoff_freq.read();

        joint_names_ = p_joint_names.read();
        joint_kp_ = p_joint_kp.read();
        joint_kd_ = p_joint_kd.read();
        default_joint_positions_ = p_default_joint_positions.read();
        joint_reset_positions_ = p_joint_reset_positions.read();
        effort_limits_ = p_effort_limits.read();

        const auto base_pos = p_default_base_position.read();
        if (base_pos.size() >= 3) {
            default_base_position_ << base_pos[0], base_pos[1], base_pos[2];
        } else {
            default_base_position_ << 0.0, 0.0, 0.77;
        }

        if (joint_names_.size() != 12 ||
            joint_kp_.size() != 12 ||
            joint_kd_.size() != 12 ||
            default_joint_positions_.size() != 12 ||
            joint_reset_positions_.size() != 12 ||
            effort_limits_.size() != 12) {
            throw std::runtime_error("MujocoEnv parameter size mismatch. Expected 12 joints.");
        }

        std::fill(s.cmd_online.begin(), s.cmd_online.end(), false);
        std::fill(s.motor_on.begin(), s.motor_on.end(), true);

        for (int i = 0; i < 12; ++i) {
            std::strncpy(s.cmd[i].name, joint_names_[i].c_str(), sizeof(s.cmd[i].name) - 1);
            s.cmd[i].name[sizeof(s.cmd[i].name) - 1] = '\0';
            s.cmd[i].pos = default_joint_positions_[i];
            s.cmd[i].vel = 0.0;
            s.cmd[i].torque = 0.0;
            s.cmd[i].kp = joint_kp_[i];
            s.cmd[i].kd = joint_kd_[i];
        }

        load_model(resolve_model_path(p_model_path.read()));

        model_->opt.timestep = physics_dt_;
        des_pos_ = Eigen::VectorXd::Zero(12);
        des_vel_ = Eigen::VectorXd::Zero(12);
        ff_tau_ = Eigen::VectorXd::Zero(12);
        kp_vec_ = Eigen::VectorXd::Zero(12);
        kd_vec_ = Eigen::VectorXd::Zero(12);
        effort_limits_vec_ = Eigen::Map<Eigen::VectorXd>(effort_limits_.data(), 12);

        gravity_vector_ << 0.0, 0.0, -1.0;
        ema_filter_.initialize(cutoff_freq_, physics_dt_, 12);
        build_joint_mapping();

        reset_sim();
        start_viewer_thread();
        getLogger()->info("MujocoEnv initialized.");
    }

    void execute(MujocoEnvState& s) override {
        if (!model_ || !data_) {
            PERIODIC_CALL(
                getLogger()->warn("MujocoEnv model/data are not ready.");
            , 1s);
            return;
        }

        for (int i = 0; i < 12; ++i) {
            dr_motor_on_[i].on_update([&, i](const bool& on) {
                s.motor_on[i] = on;
            });

            dr_cmd_[i].on_update([&, i](const custom_types::MotorCmd& cmd) {
                s.cmd[i] = cmd;
                s.cmd_online[i] = true;

                auto itr = actuator_index_by_joint_name_.find(joint_names_[i]);
                if (itr == actuator_index_by_joint_name_.end()) return;
                const int act_idx = itr->second;

                if (cmd.duration_ms > 0.0) {
                    // 새 타겟이거나 처음 진입할 때만 interpolation 시작
                    if (!cmd_interp_[i].active || std::abs(cmd.pos - cmd_interp_[i].target_pos) > 1e-4) {
                        cmd_interp_[i].start_ref = data_->qpos[joint_qpos_adr_[i]]; // 실제 시뮬레이션 조인트 위치에서 시작
                        cmd_interp_[i].target_pos = cmd.pos;
                        cmd_interp_[i].duration_ms = cmd.duration_ms;
                        cmd_interp_[i].elapsed_ms = 0.0;
                        cmd_interp_[i].active = true;
                    }
                    // 같은 타겟이면 계속 진행
                } else {
                    // 즉시 명령: interpolation 취소, des_pos 즉시 갱신
                    cmd_interp_[i].active = false;
                    des_pos_[act_idx] = cmd.pos;
                }
                des_vel_[act_idx] = cmd.vel;
                ff_tau_[act_idx] = cmd.torque;
                kp_vec_[act_idx] = cmd.kp;
                kd_vec_[act_idx] = cmd.kd;
            });
        }

        // interpolation 진행: des_pos_를 현재 ref로 갱신
        const double dt_ms = control_dt_ * 1000.0;
        for (int i = 0; i < 12; ++i) {
            if (!cmd_interp_[i].active) continue;
            auto itr = actuator_index_by_joint_name_.find(joint_names_[i]);
            if (itr == actuator_index_by_joint_name_.end()) continue;
            const int act_idx = itr->second;
            cmd_interp_[i].elapsed_ms += dt_ms;
            const double ratio = std::min(cmd_interp_[i].elapsed_ms / cmd_interp_[i].duration_ms, 1.0);
            des_pos_[act_idx] = cmd_interp_[i].start_ref + ratio * (cmd_interp_[i].target_pos - cmd_interp_[i].start_ref);
            if (ratio >= 1.0) {
                cmd_interp_[i].active = false;
            }
        }

        // int substeps = static_cast<int>(std::round(policy_dt_ / physics_dt_));
        int substeps = static_cast<int>(std::round(control_dt_ / physics_dt_));
        // substeps = std::max(substeps, 1);

        {
            std::lock_guard<std::mutex> lock(sim_mutex_);
            motor_on_ = s.motor_on;
            const bool base_fixed = fixed_base_.load();
            for (int i = 0; i < substeps; ++i) {
                apply_controller();
                mj_step(model_, data_);
                if (base_fixed) {
                    data_->qpos[0] = default_base_position_.x();
                    data_->qpos[1] = default_base_position_.y();
                    data_->qpos[2] = default_base_position_.z();
                    data_->qpos[3] = 1.0;
                    data_->qpos[4] = 0.0;
                    data_->qpos[5] = 0.0;
                    data_->qpos[6] = 0.0;
                    for (int j = 0; j < 6; ++j) {
                        data_->qvel[j] = 0.0;
                    }
                }
            }

            publish_state();
        }
    }

    ~MujocoEnv() override {
        viewer_stop_requested_ = true;
        if (viewer_thread_.joinable()) {
            viewer_thread_.join();
        }

        if (data_) {
            mj_deleteData(data_);
            data_ = nullptr;
        }
        if (model_) {
            mj_deleteModel(model_);
            model_ = nullptr;
        }
    }

private:
    void start_viewer_thread() {
        viewer_stop_requested_ = false;
        viewer_thread_ = std::thread([this]() { viewer_loop(); });
    }

    void viewer_loop() {
        if (!glfwInit()) {
            getLogger()->error("Failed to initialize GLFW");
            return;
        }

        window_ = glfwCreateWindow(1200, 900, "MuJoCo RTFW Viewer", nullptr, nullptr);
        if (!window_) {
            getLogger()->error("Failed to create GLFW window");
            glfwTerminate();
            return;
        }

        glfwMakeContextCurrent(window_);
        glfwSwapInterval(1);
        glfwSetWindowUserPointer(window_, this);

        glfwSetKeyCallback(window_, &MujocoEnv::key_callback);
        glfwSetCursorPosCallback(window_, &MujocoEnv::mouse_move_callback);
        glfwSetMouseButtonCallback(window_, &MujocoEnv::mouse_button_callback);
        glfwSetScrollCallback(window_, &MujocoEnv::scroll_callback);

        mjv_defaultCamera(&cam_);
        mjv_defaultOption(&opt_);
        mjv_defaultScene(&scn_);
        mjr_defaultContext(&con_);

        {
            std::lock_guard<std::mutex> lock(sim_mutex_);
            mjv_makeScene(model_, &scn_, 2000);
            mjr_makeContext(model_, &con_, mjFONTSCALE_150);

            base_link_id_ = mj_name2id(model_, mjOBJ_BODY, "base_link");
        }

        if (base_link_id_ < 0) {
            camera_tracking_ = false;
            PERIODIC_CALL(
                getLogger()->warn("Body 'base_link' not found. Camera tracking disabled.");
            , 1s);
        }

        while (!viewer_stop_requested_ && !glfwWindowShouldClose(window_)) {
            {
                std::lock_guard<std::mutex> lock(sim_mutex_);
                if (camera_tracking_ && base_link_id_ >= 0) {
                    cam_.lookat[0] = data_->xpos[base_link_id_ * 3 + 0];
                    cam_.lookat[1] = data_->xpos[base_link_id_ * 3 + 1];
                    cam_.lookat[2] = data_->xpos[base_link_id_ * 3 + 2];
                }

                mjrRect viewport{0, 0, 0, 0};
                glfwGetFramebufferSize(window_, &viewport.width, &viewport.height);
                mjv_updateScene(model_, data_, &opt_, nullptr, &cam_, mjCAT_ALL, &scn_);
                mjr_render(viewport, &scn_, &con_);

                // HUD overlay
                const bool is_fixed = fixed_base_.load();
                const std::string title = "[C] Fixed Base: " + std::string(is_fixed ? "ON  (locked)" : "OFF (floating)");
                mjr_overlay(mjFONT_NORMAL, mjGRID_BOTTOMLEFT, viewport, title.c_str(), nullptr, &con_);
            }

            glfwSwapBuffers(window_);
            glfwPollEvents();
            std::this_thread::sleep_for(1ms);
        }

        viewer_stop_requested_ = true;
        mjr_freeContext(&con_);
        mjv_freeScene(&scn_);
        if (window_) {
            glfwDestroyWindow(window_);
            window_ = nullptr;
        }
        glfwTerminate();
    }

    static void key_callback(GLFWwindow* window, int key, int, int action, int) {
        auto* env = static_cast<MujocoEnv*>(glfwGetWindowUserPointer(window));
        if (!env) {
            return;
        }

        if (action == GLFW_PRESS) {
            if (key == GLFW_KEY_ESCAPE) {
                glfwSetWindowShouldClose(window, GLFW_TRUE);
                env->viewer_stop_requested_ = true;
            } else if (key == GLFW_KEY_BACKSPACE) {
                env->reset_sim();
                env->getLogger()->info("MuJoCo simulation reset.");
            } else if (key == GLFW_KEY_F) {
                env->camera_tracking_ = !env->camera_tracking_;
                env->getLogger()->info("Camera tracking {}", env->camera_tracking_ ? "ENABLED" : "DISABLED");
            } else if (key == GLFW_KEY_C) {
                const bool now_fixed = !env->fixed_base_.load();
                env->fixed_base_.store(now_fixed);
                env->getLogger()->info("Fixed base {}", now_fixed ? "ENABLED (base locked at default position)" : "DISABLED (floating base)");
            }
        }
    }

    static void mouse_button_callback(GLFWwindow* window, int, int, int) {
        auto* env = static_cast<MujocoEnv*>(glfwGetWindowUserPointer(window));
        if (!env) {
            return;
        }

        env->button_left_ = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
        env->button_middle_ = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
        env->button_right_ = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);
        glfwGetCursorPos(window, &env->lastx_, &env->lasty_);
    }

    static void mouse_move_callback(GLFWwindow* window, double xpos, double ypos) {
        auto* env = static_cast<MujocoEnv*>(glfwGetWindowUserPointer(window));
        if (!env) {
            return;
        }

        if (!env->button_left_ && !env->button_middle_ && !env->button_right_) {
            return;
        }

        const double dx = xpos - env->lastx_;
        const double dy = ypos - env->lasty_;
        env->lastx_ = xpos;
        env->lasty_ = ypos;

        int width = 0;
        int height = 0;
        glfwGetWindowSize(window, &width, &height);
        if (height == 0) {
            return;
        }

        const bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                                glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

        mjtMouse action_mouse;
        if (env->button_right_) {
            action_mouse = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
        } else if (env->button_left_) {
            action_mouse = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
        } else {
            action_mouse = mjMOUSE_ZOOM;
        }

        mjv_moveCamera(env->model_, action_mouse, dx / static_cast<double>(height), dy / static_cast<double>(height), &env->scn_, &env->cam_);
    }

    static void scroll_callback(GLFWwindow* window, double, double yoffset) {
        auto* env = static_cast<MujocoEnv*>(glfwGetWindowUserPointer(window));
        if (!env) {
            return;
        }
        mjv_moveCamera(env->model_, mjMOUSE_ZOOM, 0.0, -0.05 * yoffset, &env->scn_, &env->cam_);
    }

    void load_model(const std::string& xml_path) {
        char error[1024] = {0};
        model_ = mj_loadXML(xml_path.c_str(), nullptr, error, sizeof(error));
        if (!model_) {
            throw std::runtime_error(std::string("Failed to load MuJoCo XML: ") + error);
        }

        data_ = mj_makeData(model_);
        if (!data_) {
            throw std::runtime_error("Failed to allocate MuJoCo data");
        }

        getLogger()->info("MuJoCo model loaded: {}", xml_path);
    }

    void build_joint_mapping() {
        actuator_index_by_joint_name_.clear();
        for (int i = 0; i < model_->nu; ++i) {
            if (model_->actuator_trntype[i] != mjTRN_JOINT) {
                continue;
            }
            const int joint_id = model_->actuator_trnid[2 * i];
            const char* joint_name = mj_id2name(model_, mjOBJ_JOINT, joint_id);
            if (joint_name != nullptr) {
                actuator_index_by_joint_name_[joint_name] = i;
            }
        }

        actuator_indices_.assign(12, -1);
        joint_qpos_adr_.assign(12, -1);
        joint_qvel_adr_.assign(12, -1);

        for (int i = 0; i < 12; ++i) {
            const auto it = actuator_index_by_joint_name_.find(joint_names_[i]);
            if (it == actuator_index_by_joint_name_.end()) {
                throw std::runtime_error("Joint is not mapped to actuator: " + joint_names_[i]);
            }

            const int actuator_idx = it->second;
            const int joint_id = model_->actuator_trnid[2 * actuator_idx];

            actuator_indices_[i] = actuator_idx;
            joint_qpos_adr_[i] = model_->jnt_qposadr[joint_id];
            joint_qvel_adr_[i] = model_->jnt_dofadr[joint_id];
        }
    }

    void reset_sim() {
        std::lock_guard<std::mutex> lock(sim_mutex_);
        mj_resetData(model_, data_);

        data_->qpos[0] = default_base_position_.x();
        data_->qpos[1] = default_base_position_.y();
        data_->qpos[2] = default_base_position_.z();
        data_->qpos[3] = 1.0;
        data_->qpos[4] = 0.0;
        data_->qpos[5] = 0.0;
        data_->qpos[6] = 0.0;

        for (int i = 0; i < 12; ++i) {
            data_->qpos[7+i] = joint_reset_positions_[i];
        }

        for (int i = 0; i < model_->nv; ++i) {
            data_->qvel[i] = 0.0;
        }

        mj_forward(model_, data_);
        prev_base_vel_local_.setZero();
        prev_time_ = data_->time;
    }

    Eigen::VectorXd get_joint_pos() const {
        Eigen::VectorXd q(12);
        for (int i = 0; i < 12; ++i) {
            q[i] = data_->qpos[joint_qpos_adr_[i]];
        }
        return q;
    }

    Eigen::VectorXd get_joint_vel() const {
        Eigen::VectorXd dq(12);
        for (int i = 0; i < 12; ++i) {
            dq[i] = data_->qvel[joint_qvel_adr_[i]];
        }
        return dq;
    }

    Eigen::Quaterniond get_base_quat() const {
        return Eigen::Quaterniond(data_->qpos[3], data_->qpos[4], data_->qpos[5], data_->qpos[6]);
    }

    Eigen::Vector3d get_base_ang_vel_body() const {
        Eigen::Map<const Eigen::Vector3d> ang_vel_world(data_->qvel + 3);
        const Eigen::Quaterniond q = get_base_quat();
        return q.conjugate() * ang_vel_world;
    }

    Eigen::Vector3d get_base_vel_body() const {
        Eigen::Map<const Eigen::Vector3d> vel_world(data_->qvel + 0);
        const Eigen::Quaterniond q = get_base_quat();
        return q.conjugate() * vel_world;
    }

    void apply_controller() {
        const Eigen::VectorXd q = get_joint_pos();
        const Eigen::VectorXd dq = get_joint_vel();

        Eigen::VectorXd torques =
            kp_vec_.array() * (des_pos_ - q).array() +
            kd_vec_.array() * (des_vel_ - dq).array() +
            ff_tau_.array();

        torques = ema_filter_.filter(torques);
        torques = torques.array().min(effort_limits_vec_.array()).max(-effort_limits_vec_.array());

        for (int i = 0; i < 12; ++i) {
            if (!motor_on_[i]) {
                data_->ctrl[actuator_indices_[i]] = 0.0;
                continue;
            }
            data_->ctrl[actuator_indices_[i]] = torques[i];
        }
    }

    void publish_state() {
        const Eigen::VectorXd q = get_joint_pos();
        const Eigen::VectorXd dq = get_joint_vel();

        for (int i = 0; i < 12; ++i) {
            custom_types::MotorCmd applied_cmd{};
            std::strncpy(applied_cmd.name, joint_names_[i].c_str(), sizeof(applied_cmd.name) - 1);
            applied_cmd.name[sizeof(applied_cmd.name) - 1] = '\0';
            applied_cmd.pos = motor_on_[i] ? des_pos_[i] : 0.0;
            applied_cmd.vel = motor_on_[i] ? des_vel_[i] : 0.0;
            applied_cmd.torque = motor_on_[i] ? ff_tau_[i] : 0.0;
            applied_cmd.kp = motor_on_[i] ? kp_vec_[i] : 0.0;
            applied_cmd.kd = motor_on_[i] ? kd_vec_[i] : 0.0;
            dw_mtr_cmd_[i].write(applied_cmd);

            custom_types::MotorState state{};
            std::strncpy(state.name, joint_names_[i].c_str(), sizeof(state.name) - 1);
            state.name[sizeof(state.name) - 1] = '\0';
            state.pos = q[i];
            state.vel = dq[i];
            state.torque = data_->ctrl[actuator_indices_[i]];
            state.temp = 0.0;
            state.status = motor_on_[i] ? 0 : 1;
            dw_mtr_stat_[i].write(state);
        }

        // 가상 ankle upper/lower cmd_applied/state 발행 (실제 제어는 pitch/roll)
        auto publish_virtual_ankle_side =
            [&](int pitch_idx, int roll_idx, int upper_writer_idx, int lower_writer_idx, char side) {
                const auto cmd_map = map_rp_to_theta(
                    static_cast<float>(des_pos_[pitch_idx]),
                    static_cast<float>(des_pos_[roll_idx]),
                    static_cast<float>(des_vel_[pitch_idx]),
                    static_cast<float>(des_vel_[roll_idx]),
                    static_cast<float>(ff_tau_[pitch_idx]),
                    static_cast<float>(ff_tau_[roll_idx]),
                    side);

                custom_types::MotorCmd upper_cmd{};
                upper_cmd.pos = motor_on_[pitch_idx] ? cmd_map.theta(0) : 0.0;
                upper_cmd.vel = motor_on_[pitch_idx] ? cmd_map.theta_vel(0) : 0.0;
                upper_cmd.torque = motor_on_[pitch_idx] ? cmd_map.theta_tau(0) : 0.0;
                upper_cmd.kp = motor_on_[pitch_idx] ? kp_vec_[pitch_idx] : 0.0;
                upper_cmd.kd = motor_on_[pitch_idx] ? kd_vec_[pitch_idx] : 0.0;
                dw_mtr_cmd_[upper_writer_idx].write(upper_cmd);

                custom_types::MotorCmd lower_cmd{};
                lower_cmd.pos = motor_on_[roll_idx] ? cmd_map.theta(1) : 0.0;
                lower_cmd.vel = motor_on_[roll_idx] ? cmd_map.theta_vel(1) : 0.0;
                lower_cmd.torque = motor_on_[roll_idx] ? cmd_map.theta_tau(1) : 0.0;
                lower_cmd.kp = motor_on_[roll_idx] ? kp_vec_[roll_idx] : 0.0;
                lower_cmd.kd = motor_on_[roll_idx] ? kd_vec_[roll_idx] : 0.0;
                dw_mtr_cmd_[lower_writer_idx].write(lower_cmd);

                const auto state_map = map_rp_to_theta(
                    static_cast<float>(q[pitch_idx]),
                    static_cast<float>(q[roll_idx]),
                    static_cast<float>(dq[pitch_idx]),
                    static_cast<float>(dq[roll_idx]),
                    static_cast<float>(data_->ctrl[actuator_indices_[pitch_idx]]),
                    static_cast<float>(data_->ctrl[actuator_indices_[roll_idx]]),
                    side);

                custom_types::MotorState upper_state{};
                upper_state.pos = state_map.theta(0);
                upper_state.vel = state_map.theta_vel(0);
                upper_state.torque = state_map.theta_tau(0);
                upper_state.temp = 0.0;
                upper_state.status = motor_on_[pitch_idx] ? 0 : 1;
                upper_state.enabled = motor_on_[pitch_idx];
                dw_mtr_stat_[upper_writer_idx].write(upper_state);

                custom_types::MotorState lower_state{};
                lower_state.pos = state_map.theta(1);
                lower_state.vel = state_map.theta_vel(1);
                lower_state.torque = state_map.theta_tau(1);
                lower_state.temp = 0.0;
                lower_state.status = motor_on_[roll_idx] ? 0 : 1;
                lower_state.enabled = motor_on_[roll_idx];
                dw_mtr_stat_[lower_writer_idx].write(lower_state);
            };

        publish_virtual_ankle_side(8, 10, 12, 14, 'l');
        publish_virtual_ankle_side(9, 11, 13, 15, 'r');

        const Eigen::Quaterniond quat = get_base_quat();
        const Eigen::Vector3d base_ang_vel = get_base_ang_vel_body();
        const Eigen::Vector3d base_vel = get_base_vel_body();

        const double dt = std::max(data_->time - prev_time_, 1e-6);
        const Eigen::Vector3d base_acc = (base_vel - prev_base_vel_local_) / dt;
        prev_base_vel_local_ = base_vel;
        prev_time_ = data_->time;

        custom_types::Imu imu{};
        imu.acc.x = base_acc.x();
        imu.acc.y = base_acc.y();
        imu.acc.z = base_acc.z();
        imu.gyro.x = base_ang_vel.x();
        imu.gyro.y = base_ang_vel.y();
        imu.gyro.z = base_ang_vel.z();
        imu.orientation.w = quat.w();
        imu.orientation.x = quat.x();
        imu.orientation.y = quat.y();
        imu.orientation.z = quat.z();
        dw_imu_.write(imu);

        
    }

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

        throw std::runtime_error("MuJoCo model file not found: " + raw_path);
    }

    struct AnkleThetaMapping {
        Eigen::Vector2f theta = Eigen::Vector2f::Zero();
        Eigen::Matrix2f jac = Eigen::Matrix2f::Zero();
        Eigen::Vector2f theta_vel = Eigen::Vector2f::Zero();
        Eigen::Vector2f theta_tau = Eigen::Vector2f::Zero();
    };

    static inline Eigen::Matrix2f safe_inverse_2x2(const Eigen::Matrix2f& m) {
        if (std::abs(m.determinant()) > 1e-6f) {
            return m.inverse();
        }
        return Eigen::Matrix2f::Zero();
    }

    static inline AnkleThetaMapping map_rp_to_theta(
        float rp_r, float rp_p,
        float rp_vel_r, float rp_vel_p,
        float rp_tau_r, float rp_tau_p,
        char side) {
        AnkleThetaMapping out;
        out.theta = kin_2rsu::ankle_ik(rp_r, rp_p, side);
        out.jac = kin_2rsu::ankle_J(rp_r, rp_p, out.theta(0), out.theta(1), side);
        const Eigen::Vector2f rp_vel(rp_vel_r, rp_vel_p);
        const Eigen::Vector2f rp_tau(rp_tau_r, rp_tau_p);
        out.theta_vel = safe_inverse_2x2(out.jac) * rp_vel;
        out.theta_tau = out.jac.transpose() * rp_tau;
        return out;
    }

private:
    DataReader<bool> dr_motor_on_[12] = {
        DataReader<bool>{"hip_yaw_left/on"},
        DataReader<bool>{"hip_yaw_right/on"},
        DataReader<bool>{"hip_roll_left/on"},
        DataReader<bool>{"hip_roll_right/on"},
        DataReader<bool>{"hip_pitch_left/on"},
        DataReader<bool>{"hip_pitch_right/on"},
        DataReader<bool>{"knee_left/on"},
        DataReader<bool>{"knee_right/on"},
        DataReader<bool>{"ankle_pitch_left/on"},
        DataReader<bool>{"ankle_pitch_right/on"},
        DataReader<bool>{"ankle_roll_left/on"},
        DataReader<bool>{"ankle_roll_right/on"},
    };

    DataReader<custom_types::MotorCmd> dr_cmd_[12] = {
        DataReader<custom_types::MotorCmd>{"hip_yaw_left/cmd"},
        DataReader<custom_types::MotorCmd>{"hip_yaw_right/cmd"},
        DataReader<custom_types::MotorCmd>{"hip_roll_left/cmd"},
        DataReader<custom_types::MotorCmd>{"hip_roll_right/cmd"},
        DataReader<custom_types::MotorCmd>{"hip_pitch_left/cmd"},
        DataReader<custom_types::MotorCmd>{"hip_pitch_right/cmd"},
        DataReader<custom_types::MotorCmd>{"knee_left/cmd"},
        DataReader<custom_types::MotorCmd>{"knee_right/cmd"},
        DataReader<custom_types::MotorCmd>{"ankle_pitch_left/cmd"},
        DataReader<custom_types::MotorCmd>{"ankle_pitch_right/cmd"},
        DataReader<custom_types::MotorCmd>{"ankle_roll_left/cmd"},
        DataReader<custom_types::MotorCmd>{"ankle_roll_right/cmd"},
    };

    DataWriter<custom_types::MotorCmd> dw_mtr_cmd_[12+4] = {
        DataWriter<custom_types::MotorCmd>{"hip_yaw_left/cmd_applied"},
        DataWriter<custom_types::MotorCmd>{"hip_yaw_right/cmd_applied"},
        DataWriter<custom_types::MotorCmd>{"hip_roll_left/cmd_applied"},
        DataWriter<custom_types::MotorCmd>{"hip_roll_right/cmd_applied"},
        DataWriter<custom_types::MotorCmd>{"hip_pitch_left/cmd_applied"},
        DataWriter<custom_types::MotorCmd>{"hip_pitch_right/cmd_applied"},
        DataWriter<custom_types::MotorCmd>{"knee_left/cmd_applied"},
        DataWriter<custom_types::MotorCmd>{"knee_right/cmd_applied"},
        DataWriter<custom_types::MotorCmd>{"ankle_pitch_left/cmd_applied"},
        DataWriter<custom_types::MotorCmd>{"ankle_pitch_right/cmd_applied"},
        DataWriter<custom_types::MotorCmd>{"ankle_roll_left/cmd_applied"},
        DataWriter<custom_types::MotorCmd>{"ankle_roll_right/cmd_applied"},

        DataWriter<custom_types::MotorCmd>{"ankle_upper_left/cmd_applied"},
        DataWriter<custom_types::MotorCmd>{"ankle_upper_right/cmd_applied"},
        DataWriter<custom_types::MotorCmd>{"ankle_lower_left/cmd_applied"},
        DataWriter<custom_types::MotorCmd>{"ankle_lower_right/cmd_applied"},
    };    

    DataWriter<custom_types::MotorState> dw_mtr_stat_[12+4] = {
        DataWriter<custom_types::MotorState>{"hip_yaw_left/state"},
        DataWriter<custom_types::MotorState>{"hip_yaw_right/state"},
        DataWriter<custom_types::MotorState>{"hip_roll_left/state"},
        DataWriter<custom_types::MotorState>{"hip_roll_right/state"},
        DataWriter<custom_types::MotorState>{"hip_pitch_left/state"},
        DataWriter<custom_types::MotorState>{"hip_pitch_right/state"},
        DataWriter<custom_types::MotorState>{"knee_left/state"},
        DataWriter<custom_types::MotorState>{"knee_right/state"},
        DataWriter<custom_types::MotorState>{"ankle_pitch_left/state"},
        DataWriter<custom_types::MotorState>{"ankle_pitch_right/state"},
        DataWriter<custom_types::MotorState>{"ankle_roll_left/state"},
        DataWriter<custom_types::MotorState>{"ankle_roll_right/state"},
        
        DataWriter<custom_types::MotorState>{"ankle_upper_left/state"},
        DataWriter<custom_types::MotorState>{"ankle_upper_right/state"},
        DataWriter<custom_types::MotorState>{"ankle_lower_left/state"},
        DataWriter<custom_types::MotorState>{"ankle_lower_right/state"},
    };

    DataWriter<custom_types::Imu> dw_imu_{"imu_data"};

    Parameter<std::string> p_model_path{"mujoco.model_path", "mjcf/robot_nl.xml"};
    Parameter<double> p_policy_dt{"policy_dt", 0.02};
    Parameter<double> p_control_dt{"control_dt", 0.004};
    Parameter<double> p_physics_dt{"physics_dt", 0.0005};
    Parameter<double> p_cutoff_freq{"cutoff_freq", 200.0};

    Parameter<std::vector<std::string>> p_joint_names{"joints", std::vector<std::string>()};
    Parameter<std::vector<double>> p_joint_kp{"joint_kp"};
    Parameter<std::vector<double>> p_joint_kd{"joint_kd"};
    Parameter<std::vector<double>> p_default_joint_positions{"default_joint_positions"};
    Parameter<std::vector<double>> p_joint_reset_positions{"joint_reset_positions"};
    Parameter<std::vector<double>> p_effort_limits{"effort_limits"};
    Parameter<std::vector<double>> p_default_base_position{"default_base_position", std::vector<double>{0.0, 0.0, 0.77}};

    mjModel* model_ = nullptr;
    mjData* data_ = nullptr;

    double policy_dt_ = 0.02;
    double control_dt_ = 0.005;
    double physics_dt_ = 0.0005;
    double cutoff_freq_ = 200.0;

    std::vector<std::string> joint_names_;
    std::vector<double> joint_kp_;
    std::vector<double> joint_kd_;
    std::vector<double> default_joint_positions_;
    std::vector<double> joint_reset_positions_;
    std::vector<double> effort_limits_;

    std::unordered_map<std::string, int> actuator_index_by_joint_name_;
    std::vector<int> actuator_indices_;
    std::vector<int> joint_qpos_adr_;
    std::vector<int> joint_qvel_adr_;

    Eigen::VectorXd des_pos_;
    Eigen::VectorXd des_vel_;
    Eigen::VectorXd ff_tau_;
    Eigen::VectorXd kp_vec_;
    Eigen::VectorXd kd_vec_;
    Eigen::VectorXd effort_limits_vec_;
    std::array<bool, 12> motor_on_ = {true, true, true, true, true, true, true, true, true, true, true, true};
    Eigen::Vector3d gravity_vector_{0.0, 0.0, -1.0};
    Eigen::Vector3d default_base_position_{0.0, 0.0, 0.77};

    Eigen::Vector3d prev_base_vel_local_{0.0, 0.0, 0.0};
    double prev_time_ = 0.0;

    // 모터단 interpolation 상태 (상위에서 target+duration을 받으면 현재 des_pos_ 기준으로 보간)
    struct MotorCmdInterp {
        bool active = false;
        double start_ref = 0.0;    // interpolation 시작 시점의 des_pos
        double target_pos = 0.0;   // 목표 position (변경 감지용)
        double duration_ms = 0.0;
        double elapsed_ms = 0.0;
    };
    std::array<MotorCmdInterp, 12> cmd_interp_{};

    EmaFilter ema_filter_;

    GLFWwindow* window_ = nullptr;
    mjvCamera cam_;
    mjvOption opt_;
    mjvScene scn_;
    mjrContext con_;

    bool camera_tracking_ = true;
    int base_link_id_ = -1;
    std::atomic<bool> fixed_base_{true};

    std::thread viewer_thread_;
    std::atomic<bool> viewer_stop_requested_{false};
    std::mutex sim_mutex_;

    bool button_left_ = false;
    bool button_middle_ = false;
    bool button_right_ = false;
    double lastx_ = 0.0;
    double lasty_ = 0.0;
};

} // namespace task_pool
