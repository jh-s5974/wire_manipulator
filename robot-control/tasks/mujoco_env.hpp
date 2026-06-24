#pragma once

// MuJoCo 시뮬레이션 환경 — 와이어 매니퓰레이터 (5 조인트)
//
// MJCF: mjcf/manipulator.xml
// 채널 이름: joint0~joint4
//
// config/robotnl.yaml 의 `joints` 리스트 순서로 MJCF 조인트 이름을 지정:
//   - yaw1   → joint0
//   - pitch1 → joint1
//   - pris1  → joint2
//   - pitch2 → joint3
//   - pris2  → joint4

#include <rtfw/task.h>
#include "custom_types.hpp"
#include "util.hpp"

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

static constexpr int MJ_N = 5; // 매니퓰레이터 조인트 수

struct MujocoEnvState {
    std::array<custom_types::MotorCmd, MJ_N> cmd{};
    std::array<bool, MJ_N> motor_on{};
    std::array<bool, MJ_N> cmd_online{};
};

// EMA 필터 (토크 스파이크 억제)
class EmaFilter {
public:
    void initialize(double cutoff_freq, double dt, int size) {
        const double tau = 1.0 / (2.0 * M_PI * std::max(cutoff_freq, 1e-6));
        alpha_ = dt / (dt + tau);
        value_ = Eigen::VectorXd::Zero(size);
    }

    Eigen::VectorXd filter(const Eigen::VectorXd& x) {
        if (value_.size() != x.size()) { value_ = x; return value_; }
        value_ = alpha_ * x + (1.0 - alpha_) * value_;
        return value_;
    }

private:
    double alpha_ = 1.0;
    Eigen::VectorXd value_;
};

class MujocoEnv : public Task<MujocoEnvState> {
public:
    const char* getName() const override { return "MujocoEnv"; }

    void initialize(MujocoEnvState& s) override {
        control_dt_  = p_control_dt.read();
        physics_dt_  = p_physics_dt.read();
        cutoff_freq_ = p_cutoff_freq.read();

        joint_names_            = p_joint_names.read();
        joint_kp_               = p_joint_kp.read();
        joint_kd_               = p_joint_kd.read();
        default_joint_positions_ = p_default_joint_positions.read();
        effort_limits_          = p_effort_limits.read();
        gravity_comp_           = p_gravity_comp.read();

        if ((int)joint_names_.size() < MJ_N || (int)joint_kp_.size() < MJ_N ||
            (int)joint_kd_.size() < MJ_N     || (int)default_joint_positions_.size() < MJ_N ||
            (int)effort_limits_.size() < MJ_N) {
            throw std::runtime_error(
                "MujocoEnv: parameter size mismatch. Need at least " +
                std::to_string(MJ_N) + " joints.");
        }

        s.cmd_online.fill(false);
        s.motor_on.fill(true);
        for (int i = 0; i < MJ_N; ++i) {
            std::strncpy(s.cmd[i].name, joint_names_[i].c_str(), sizeof(s.cmd[i].name) - 1);
            s.cmd[i].pos    = default_joint_positions_[i];
            s.cmd[i].kp     = joint_kp_[i];
            s.cmd[i].kd     = joint_kd_[i];
        }

        load_model(resolve_model_path(p_model_path.read()));
        model_->opt.timestep = physics_dt_;

        des_pos_ = Eigen::VectorXd::Zero(MJ_N);
        des_vel_ = Eigen::VectorXd::Zero(MJ_N);
        ff_tau_  = Eigen::VectorXd::Zero(MJ_N);
        kp_vec_  = Eigen::VectorXd::Zero(MJ_N);
        kd_vec_  = Eigen::VectorXd::Zero(MJ_N);
        effort_limits_vec_ = Eigen::Map<Eigen::VectorXd>(effort_limits_.data(), MJ_N);

        ema_filter_.initialize(cutoff_freq_, physics_dt_, MJ_N);
        build_joint_mapping();

        // 첫 GUI 명령이 오기 전까지도 default_joint_positions/joint_kp/joint_kd로
        // 자세를 잡고 있도록 제어 변수를 미리 채움 (안 그러면 kp=kd=0인 채로
        // 시작해 중력/결합 영향으로 처짐)
        for (int i = 0; i < MJ_N; ++i) {
            const int act_idx = actuator_indices_[i];
            des_pos_[act_idx] = default_joint_positions_[i];
            kp_vec_[act_idx]  = joint_kp_[i];
            kd_vec_[act_idx]  = joint_kd_[i];
        }

        reset_sim();
        start_viewer_thread();

        getLogger()->info("[{}] initialized (model: {})", getName(), p_model_path.read());
    }

    void execute(MujocoEnvState& s) override {
        if (!model_ || !data_) {
            PERIODIC_CALL(getLogger()->warn("[{}] model/data not ready", getName()); , 1s);
            return;
        }

        // 명령 수신 및 interpolation 설정
        for (int i = 0; i < MJ_N; ++i) {
            dr_motor_on_[i].on_update([&, i](const bool& on) { s.motor_on[i] = on; });

            dr_cmd_[i].on_update([&, i](const custom_types::MotorCmd& cmd) {
                s.cmd[i]        = cmd;
                s.cmd_online[i] = true;

                const int act_idx = actuator_indices_[i];
                if (cmd.duration_ms > 0.0) {
                    // 목표값이 "바뀔 때만" 한 번 보간을 시작한다(현재→새 목표).
                    // 램프가 끝났다는 이유(active=false)만으로는 재시작하지 않으므로,
                    // ROS 가 같은 명령을 10Hz로 계속 보내도 끄떡거리지 않고 목표를 유지한다.
                    if (!cmd_interp_[i].has_target ||
                        std::abs(cmd.pos - cmd_interp_[i].target_pos) > 1e-4) {
                        cmd_interp_[i].has_target  = true;
                        cmd_interp_[i].start_ref   = data_->qpos[joint_qpos_adr_[i]];
                        cmd_interp_[i].target_pos  = cmd.pos;
                        cmd_interp_[i].duration_ms = cmd.duration_ms;
                        cmd_interp_[i].elapsed_ms  = 0.0;
                        cmd_interp_[i].active       = true;
                    }
                } else {
                    // 즉시 이동: 보간 끄고 목표를 바로 적용. target_pos 도 갱신해 두어야
                    // 이후 duration>0 동일 목표 명령이 불필요하게 재보간하지 않는다.
                    cmd_interp_[i].active      = false;
                    cmd_interp_[i].has_target  = true;
                    cmd_interp_[i].target_pos  = cmd.pos;
                    des_pos_[act_idx]          = cmd.pos;
                }
                des_vel_[act_idx] = cmd.vel;
                ff_tau_[act_idx]  = cmd.torque;
                kp_vec_[act_idx]  = cmd.kp;
                kd_vec_[act_idx]  = cmd.kd;
            });
        }

        // 위치 interpolation 진행
        const double dt_ms = control_dt_ * 1000.0;
        for (int i = 0; i < MJ_N; ++i) {
            if (!cmd_interp_[i].active) continue;
            const int act_idx = actuator_indices_[i];
            cmd_interp_[i].elapsed_ms += dt_ms;
            const double ratio = std::min(cmd_interp_[i].elapsed_ms / cmd_interp_[i].duration_ms, 1.0);
            des_pos_[act_idx] = cmd_interp_[i].start_ref +
                                ratio * (cmd_interp_[i].target_pos - cmd_interp_[i].start_ref);
            if (ratio >= 1.0) cmd_interp_[i].active = false;
        }

        // 물리 시뮬레이션 스텝
        const int substeps = std::max(1, (int)std::round(control_dt_ / physics_dt_));
        {
            std::lock_guard<std::mutex> lock(sim_mutex_);
            motor_on_ = s.motor_on;
            for (int step = 0; step < substeps; ++step) {
                apply_controller();
                mj_step(model_, data_);
            }
            publish_state();
        }
    }

    ~MujocoEnv() override {
        viewer_stop_requested_ = true;
        if (viewer_thread_.joinable()) viewer_thread_.join();
        if (data_)  { mj_deleteData(data_);   data_  = nullptr; }
        if (model_) { mj_deleteModel(model_); model_ = nullptr; }
    }

private:
    // ── MuJoCo 초기화 ──
    void load_model(const std::string& xml_path) {
        char error[1024] = {0};
        model_ = mj_loadXML(xml_path.c_str(), nullptr, error, sizeof(error));
        if (!model_)
            throw std::runtime_error(std::string("MuJoCo load failed: ") + error);
        data_ = mj_makeData(model_);
        if (!data_)
            throw std::runtime_error("MuJoCo mj_makeData failed");
        getLogger()->info("[{}] model loaded: {}", getName(), xml_path);
    }

    void build_joint_mapping() {
        actuator_index_by_joint_name_.clear();
        for (int i = 0; i < model_->nu; ++i) {
            if (model_->actuator_trntype[i] != mjTRN_JOINT) continue;
            const int joint_id = model_->actuator_trnid[2 * i];
            const char* name   = mj_id2name(model_, mjOBJ_JOINT, joint_id);
            if (name) actuator_index_by_joint_name_[name] = i;
        }

        actuator_indices_.assign(MJ_N, -1);
        joint_qpos_adr_.assign(MJ_N, -1);
        joint_qvel_adr_.assign(MJ_N, -1);

        for (int i = 0; i < MJ_N; ++i) {
            const auto it = actuator_index_by_joint_name_.find(joint_names_[i]);
            if (it == actuator_index_by_joint_name_.end())
                throw std::runtime_error("Joint not found in actuators: " + joint_names_[i]);

            const int act_idx  = it->second;
            const int joint_id = model_->actuator_trnid[2 * act_idx];
            actuator_indices_[i] = act_idx;
            joint_qpos_adr_[i]   = model_->jnt_qposadr[joint_id];
            joint_qvel_adr_[i]   = model_->jnt_dofadr[joint_id];
        }
        getLogger()->info("[{}] joint mapping built for {} joints", getName(), MJ_N);
        for (int i = 0; i < MJ_N; ++i)
            getLogger()->info("[DEBUG] joint {} ({}) -> actuator_idx={}", i, joint_names_[i], actuator_indices_[i]);
    }

    void reset_sim() {
        std::lock_guard<std::mutex> lock(sim_mutex_);
        mj_resetData(model_, data_);

        for (int i = 0; i < MJ_N; ++i)
            data_->qpos[joint_qpos_adr_[i]] = default_joint_positions_[i];
        for (int i = 0; i < model_->nv; ++i)
            data_->qvel[i] = 0.0;

        mj_forward(model_, data_);
    }

    // ── 제어기 ──
    void apply_controller() {
        Eigen::VectorXd q(MJ_N), dq(MJ_N);
        for (int i = 0; i < MJ_N; ++i) {
            q[i]  = data_->qpos[joint_qpos_adr_[i]];
            dq[i] = data_->qvel[joint_qvel_adr_[i]];
        }

        // 중력 보상: qfrc_bias = 중력 + 코리올리 (저속에서 ≈ 중력만)
        Eigen::VectorXd grav_ff = Eigen::VectorXd::Zero(MJ_N);
        if (gravity_comp_) {
            for (int i = 0; i < MJ_N; ++i)
                grav_ff[i] = data_->qfrc_bias[joint_qvel_adr_[i]];
        }

        Eigen::VectorXd torques =
            kp_vec_.array() * (des_pos_ - q).array() +
            kd_vec_.array() * (des_vel_ - dq).array() +
            ff_tau_.array() +
            grav_ff.array();

        torques = ema_filter_.filter(torques);
        torques = torques.array()
                      .min(effort_limits_vec_.array())
                      .max(-effort_limits_vec_.array());

        for (int i = 0; i < MJ_N; ++i) {
            data_->ctrl[actuator_indices_[i]] =
                motor_on_[i] ? torques[i] : 0.0;
        }
    }

    void publish_state() {
        for (int i = 0; i < MJ_N; ++i) {
            const double q  = data_->qpos[joint_qpos_adr_[i]];
            const double dq = data_->qvel[joint_qvel_adr_[i]];

            custom_types::MotorCmd applied{};
            std::strncpy(applied.name, joint_names_[i].c_str(), sizeof(applied.name) - 1);
            applied.pos    = motor_on_[i] ? des_pos_[i] : 0.0;
            applied.vel    = motor_on_[i] ? des_vel_[i] : 0.0;
            applied.torque = motor_on_[i] ? ff_tau_[i]  : 0.0;
            applied.kp     = motor_on_[i] ? kp_vec_[i]  : 0.0;
            applied.kd     = motor_on_[i] ? kd_vec_[i]  : 0.0;
            dw_cmd_applied_[i].write(applied);

            custom_types::MotorState state{};
            std::strncpy(state.name, joint_names_[i].c_str(), sizeof(state.name) - 1);
            state.pos    = q;
            state.vel    = dq;
            state.torque = data_->ctrl[actuator_indices_[i]];
            state.temp   = 0.0;
            state.status = motor_on_[i] ? 0 : 1;
            state.enabled = motor_on_[i];
            dw_mtr_stat_[i].write(state);
        }
    }

    static std::string resolve_model_path(const std::string& raw_path) {
        namespace fs = std::filesystem;
        for (const fs::path& cand : { fs::path(raw_path),
                                       fs::path("./") / raw_path,
                                       fs::path("..") / raw_path }) {
            if (fs::exists(cand)) return fs::absolute(cand).string();
        }
        throw std::runtime_error("MuJoCo model not found: " + raw_path);
    }

    // ── 뷰어 스레드 ──
    void start_viewer_thread() {
        viewer_stop_requested_ = false;
        viewer_thread_ = std::thread([this]() { viewer_loop(); });
    }

    void viewer_loop() {
        if (!glfwInit()) {
            getLogger()->error("[{}] GLFW init failed", getName());
            return;
        }
        window_ = glfwCreateWindow(1200, 900, "Wire Manipulator — MuJoCo", nullptr, nullptr);
        if (!window_) {
            getLogger()->error("[{}] GLFW window failed", getName());
            glfwTerminate();
            return;
        }
        glfwMakeContextCurrent(window_);
        glfwSwapInterval(1);
        glfwSetWindowUserPointer(window_, this);
        glfwSetKeyCallback(window_,         &MujocoEnv::key_cb);
        glfwSetCursorPosCallback(window_,   &MujocoEnv::mouse_move_cb);
        glfwSetMouseButtonCallback(window_, &MujocoEnv::mouse_btn_cb);
        glfwSetScrollCallback(window_,      &MujocoEnv::scroll_cb);

        mjv_defaultCamera(&cam_);
        mjv_defaultOption(&opt_);
        mjv_defaultScene(&scn_);
        mjr_defaultContext(&con_);

        {
            std::lock_guard<std::mutex> lk(sim_mutex_);
            mjv_makeScene(model_, &scn_, 2000);
            mjr_makeContext(model_, &con_, mjFONTSCALE_150);
            base_link_id_ = mj_name2id(model_, mjOBJ_BODY, "base_link");
        }

        while (!viewer_stop_requested_ && !glfwWindowShouldClose(window_)) {
            {
                std::lock_guard<std::mutex> lk(sim_mutex_);
                mjrRect vp{0, 0, 0, 0};
                glfwGetFramebufferSize(window_, &vp.width, &vp.height);
                mjv_updateScene(model_, data_, &opt_, nullptr, &cam_, mjCAT_ALL, &scn_);
                mjr_render(vp, &scn_, &con_);

                std::string info = "[Backspace] Reset  [ESC] Quit";
                mjr_overlay(mjFONT_NORMAL, mjGRID_BOTTOMLEFT, vp, info.c_str(), nullptr, &con_);
            }
            glfwSwapBuffers(window_);
            glfwPollEvents();
            std::this_thread::sleep_for(1ms);
        }

        viewer_stop_requested_ = true;
        mjr_freeContext(&con_);
        mjv_freeScene(&scn_);
        if (window_) { glfwDestroyWindow(window_); window_ = nullptr; }
        glfwTerminate();
    }

    static void key_cb(GLFWwindow* w, int key, int, int action, int) {
        auto* env = static_cast<MujocoEnv*>(glfwGetWindowUserPointer(w));
        if (!env || action != GLFW_PRESS) return;
        if (key == GLFW_KEY_ESCAPE)
            { glfwSetWindowShouldClose(w, GLFW_TRUE); env->viewer_stop_requested_ = true; }
        else if (key == GLFW_KEY_BACKSPACE)
            { env->reset_sim(); env->getLogger()->info("[MujocoEnv] reset"); }
    }
    static void mouse_btn_cb(GLFWwindow* w, int, int, int) {
        auto* env = static_cast<MujocoEnv*>(glfwGetWindowUserPointer(w));
        if (!env) return;
        env->btn_l_ = (glfwGetMouseButton(w, GLFW_MOUSE_BUTTON_LEFT)   == GLFW_PRESS);
        env->btn_m_ = (glfwGetMouseButton(w, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
        env->btn_r_ = (glfwGetMouseButton(w, GLFW_MOUSE_BUTTON_RIGHT)  == GLFW_PRESS);
        glfwGetCursorPos(w, &env->lastx_, &env->lasty_);
    }
    static void mouse_move_cb(GLFWwindow* w, double xpos, double ypos) {
        auto* env = static_cast<MujocoEnv*>(glfwGetWindowUserPointer(w));
        if (!env || (!env->btn_l_ && !env->btn_m_ && !env->btn_r_)) return;
        const double dx = xpos - env->lastx_, dy = ypos - env->lasty_;
        env->lastx_ = xpos; env->lasty_ = ypos;
        int ww, wh; glfwGetWindowSize(w, &ww, &wh);
        if (wh == 0) return;
        const bool shift = (glfwGetKey(w, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                            glfwGetKey(w, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);
        mjtMouse act = env->btn_r_ ? (shift ? mjMOUSE_MOVE_H    : mjMOUSE_MOVE_V)
                     : env->btn_l_ ? (shift ? mjMOUSE_ROTATE_H  : mjMOUSE_ROTATE_V)
                                   : mjMOUSE_ZOOM;
        mjv_moveCamera(env->model_, act, dx / wh, dy / wh, &env->scn_, &env->cam_);
    }
    static void scroll_cb(GLFWwindow* w, double, double dy) {
        auto* env = static_cast<MujocoEnv*>(glfwGetWindowUserPointer(w));
        if (!env) return;
        mjv_moveCamera(env->model_, mjMOUSE_ZOOM, 0.0, -0.05 * dy, &env->scn_, &env->cam_);
    }

    // ── 데이터 채널 ──
    DataReader<bool> dr_motor_on_[MJ_N] = {
        DataReader<bool>{"joint0/on", DependencyType::Weak},
        DataReader<bool>{"joint1/on", DependencyType::Weak},
        DataReader<bool>{"joint2/on", DependencyType::Weak},
        DataReader<bool>{"joint3/on", DependencyType::Weak},
        DataReader<bool>{"joint4/on", DependencyType::Weak},
    };
    DataReader<custom_types::MotorCmd> dr_cmd_[MJ_N] = {
        DataReader<custom_types::MotorCmd>{"joint0/cmd", DependencyType::Weak},
        DataReader<custom_types::MotorCmd>{"joint1/cmd", DependencyType::Weak},
        DataReader<custom_types::MotorCmd>{"joint2/cmd", DependencyType::Weak},
        DataReader<custom_types::MotorCmd>{"joint3/cmd", DependencyType::Weak},
        DataReader<custom_types::MotorCmd>{"joint4/cmd", DependencyType::Weak},
    };
    DataWriter<custom_types::MotorCmd> dw_cmd_applied_[MJ_N] = {
        DataWriter<custom_types::MotorCmd>{"joint0/cmd_applied"},
        DataWriter<custom_types::MotorCmd>{"joint1/cmd_applied"},
        DataWriter<custom_types::MotorCmd>{"joint2/cmd_applied"},
        DataWriter<custom_types::MotorCmd>{"joint3/cmd_applied"},
        DataWriter<custom_types::MotorCmd>{"joint4/cmd_applied"},
    };
    DataWriter<custom_types::MotorState> dw_mtr_stat_[MJ_N] = {
        DataWriter<custom_types::MotorState>{"joint0/state"},
        DataWriter<custom_types::MotorState>{"joint1/state"},
        DataWriter<custom_types::MotorState>{"joint2/state"},
        DataWriter<custom_types::MotorState>{"joint3/state"},
        DataWriter<custom_types::MotorState>{"joint4/state"},
    };

    // ── 파라미터 ──
    Parameter<std::string>           p_model_path{"mujoco.model_path", "mjcf/manipulator.xml"};
    Parameter<double>                p_control_dt{"control_dt", 0.005};
    Parameter<double>                p_physics_dt{"physics_dt", 0.001};
    Parameter<double>                p_cutoff_freq{"cutoff_freq", 200.0};
    Parameter<std::vector<std::string>> p_joint_names{"joints", {}};
    Parameter<std::vector<double>>   p_joint_kp{"joint_kp"};
    Parameter<std::vector<double>>   p_joint_kd{"joint_kd"};
    Parameter<std::vector<double>>   p_default_joint_positions{"default_joint_positions"};
    Parameter<std::vector<double>>   p_effort_limits{"effort_limits"};
    Parameter<bool>                  p_gravity_comp{"gravity_compensation", false};

    // ── MuJoCo 객체 ──
    mjModel* model_ = nullptr;
    mjData*  data_  = nullptr;

    double control_dt_  = 0.005;
    double physics_dt_  = 0.001;
    double cutoff_freq_ = 200.0;

    std::vector<std::string> joint_names_;
    std::vector<double> joint_kp_, joint_kd_;
    std::vector<double> default_joint_positions_;
    std::vector<double> effort_limits_;

    std::unordered_map<std::string, int> actuator_index_by_joint_name_;
    std::vector<int> actuator_indices_;
    std::vector<int> joint_qpos_adr_;
    std::vector<int> joint_qvel_adr_;

    Eigen::VectorXd des_pos_, des_vel_, ff_tau_, kp_vec_, kd_vec_, effort_limits_vec_;
    std::array<bool, MJ_N> motor_on_{};

    struct MotorCmdInterp {
        bool   active      = false;
        bool   has_target  = false;   // 첫 명령 수신 여부(첫 명령은 무조건 보간 시작)
        double start_ref   = 0.0;
        double target_pos  = 0.0;     // 마지막으로 명령된 목표(변화 감지 기준)
        double duration_ms = 0.0;
        double elapsed_ms  = 0.0;
    };
    std::array<MotorCmdInterp, MJ_N> cmd_interp_{};

    EmaFilter ema_filter_;
    bool      gravity_comp_ = false;

    // ── 뷰어 ──
    GLFWwindow* window_ = nullptr;
    mjvCamera   cam_;
    mjvOption   opt_;
    mjvScene    scn_;
    mjrContext  con_;

    int  base_link_id_ = -1;
    std::thread          viewer_thread_;
    std::atomic<bool>    viewer_stop_requested_{false};
    std::mutex           sim_mutex_;

    bool   btn_l_ = false, btn_m_ = false, btn_r_ = false;
    double lastx_ = 0.0, lasty_ = 0.0;
};

} // namespace task_pool
