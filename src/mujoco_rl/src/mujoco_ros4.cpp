#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <cmath>
#include <iostream>

#include <GLFW/glfw3.h>
#include "mujoco/mjvisualize.h"

#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "mujoco/mujoco.h"
#include "yaml-cpp/yaml.h"
#include <torch/torch.h>

#include "msg_interfaces/msg/joymsg.hpp"
#include "msg_interfaces/msg/observation.hpp"
#include "msg_interfaces/msg/action.hpp"

using namespace std::chrono_literals;

struct Config {
    double policy_dt;
    double physics_dt;
    std::vector<float> joint_kp;
    std::vector<float> joint_kd;
    std::vector<float> default_joint_positions;
    std::vector<float> effort_limits;
    int num_joints;
    int num_actions;
    std::vector<int> action_indices;
    int num_observations;
    float cutoff_freq;
};

torch::Tensor quat_rotate_inverse(torch::Tensor q, torch::Tensor v) {
    torch::Tensor q_w = q[0];
    torch::Tensor q_vec = q.slice(0, 1, 4);
    auto a = v * (2.0 * torch::pow(q_w, 2) - 1.0);
    auto b = torch::cross(q_vec, v, -1) * q_w * 2.0;
    auto c = q_vec * (torch::dot(q_vec, v)) * 2.0;
    return a - b + c;
}

class Ema {
public:
    Ema(float cutoff_freq, float dt, const torch::Tensor& init_value) {
        alpha = dt / (dt + 1.0 / (2.0 * M_PI * cutoff_freq));
        value = init_value.clone();
    }

    torch::Tensor filter(const torch::Tensor& x) {
        value = alpha * x + (1.0 - alpha) * value;
        return value;
    }

private:
    float alpha;
    torch::Tensor value;
};

class MujocoEnv : public rclcpp::Node {
public:
    MujocoEnv() : Node("mujoco_node") {
        std::string package_share_dir = ament_index_cpp::get_package_share_directory("mujoco_rl");
        // std::string config_path = package_share_dir + "/config/robotnl_pre.yaml";
        std::string config_path = package_share_dir + "/config/robotnl.yaml";
        load_config(config_path);

        // std::string xml_path = package_share_dir + "/mjcf/robot_nl_pre.xml";
        std::string xml_path = package_share_dir + "/mjcf/robot_nl.xml";
        RCLCPP_INFO(this->get_logger(), "Loading model from: %s", xml_path.c_str());
        char error[1024] = {0};
        model_ = mj_loadXML(xml_path.c_str(), nullptr, error, 1024);
        if (!model_) {
            RCLCPP_FATAL(this->get_logger(), "Failed to load XML model: %s", error);
            rclcpp::shutdown();
            return;
        }
        data_ = mj_makeData(model_);
        model_->opt.timestep = cfg_.physics_dt;

        obs_pub_ = this->create_publisher<msg_interfaces::msg::Observation>("mujoco_obs", 10);
        joy_sub_ = this->create_subscription<msg_interfaces::msg::Joymsg>(
            "joystick_state", 10, std::bind(&MujocoEnv::joystick_callback, this, std::placeholders::_1));
        action_sub_ = this->create_subscription<msg_interfaces::msg::Action>(
            "action", 10, std::bind(&MujocoEnv::action_callback, this, std::placeholders::_1));

        commands_ = torch::zeros({3}, torch::kFloat32);
        joint_kp_ = torch::from_blob(cfg_.joint_kp.data(), {cfg_.num_joints}, torch::kFloat32);
        joint_kd_ = torch::from_blob(cfg_.joint_kd.data(), {cfg_.num_joints}, torch::kFloat32);
        effort_limits_ = torch::from_blob(cfg_.effort_limits.data(), {cfg_.num_joints}, torch::kFloat32);
        gravity_vector_ = torch::tensor({0.0, 0.0, -1.0}, torch::kFloat32);
        ema_filter_ = std::make_unique<Ema>(cfg_.cutoff_freq,cfg_.physics_dt,torch::zeros({cfg_.num_joints}));
        
        auto default_positions_tensor = torch::from_blob(cfg_.default_joint_positions.data(), {cfg_.num_joints}, torch::kFloat32);
        std::vector<long> indices;
        for(int i : cfg_.action_indices){
            indices.push_back(i);
        }
        torch::Tensor index_tensor = torch::tensor(indices, torch::kLong);
        default_actions_ = default_positions_tensor.index_select(0, index_tensor);
        latest_actions_ = default_actions_.clone();

        RCLCPP_INFO(this->get_logger(), "Mujoco C++ Node has been initialized.");
    }
    ~MujocoEnv() {
        RCLCPP_INFO(this->get_logger(), "MuJoCo shutdown");
        mjr_freeContext(&con_);
        mjv_freeScene(&scn_);
        if (data_) mj_deleteData(data_);
        if (model_) mj_deleteModel(model_);
    }
    bool initialize_viewer(GLFWwindow* window) {
        window_ = window;
        mjv_defaultCamera(&cam_);
        mjv_defaultOption(&opt_);
        mjv_defaultScene(&scn_);
        mjr_defaultContext(&con_);

        mjv_makeScene(model_, &scn_, 2000);
        mjr_makeContext(model_, &con_, mjFONTSCALE_150);
        return true;
    }

    void run_step() {
        torch::Tensor current_actions;
        {
            std::lock_guard<std::mutex> lock(action_mutex_);
            current_actions = latest_actions_.clone();
        }
        
        step(current_actions);
    }

    void shutdown_msg(){
        publish_observations();
    }
    
    void render() {
        if (!window_) return;
        
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window_, &viewport.width, &viewport.height);

        {
            std::lock_guard<std::mutex> lock(physics_mutex_);
        mjv_updateScene(model_, data_, &opt_, nullptr, &cam_, mjCAT_ALL, &scn_);
        }
        mjr_render(viewport, &scn_, &con_);

        glfwSwapBuffers(window_);
        glfwPollEvents();
    }

    void reset() {
        std::lock_guard<std::mutex> lock(physics_mutex_);
        mj_resetData(model_, data_);
        mjtNum* qpos = data_->qpos;
        qpos[0] = 0.0; qpos[1] = 0.0; qpos[2] = 0.77;
        qpos[3] = 1.0; qpos[4] = 0.0; qpos[5] = 0.0; qpos[6] = 0.0;
        for (int i = 0; i < cfg_.num_joints; ++i) {
            data_->qpos[7 + i] = cfg_.default_joint_positions[i];
        }
        mj_forward(model_, data_);
        RCLCPP_INFO(this->get_logger(), "Simulation Reset.");
    }

    void set_keyboard_command(int index, float value) {
        if (index >= 0 && index < 3) {
            commands_[index] = value;
        }
    }

public:
    mjModel* model_ = nullptr;
    mjData* data_ = nullptr;
    Config cfg_;

    GLFWwindow* window_ = nullptr;
    mjvCamera cam_;
    mjvOption opt_;
    mjvScene scn_;
    mjrContext con_;

    bool button_left_ = false;
    bool button_middle_ = false;
    bool button_right_ = false;
    double lastx_ = 0;
    double lasty_ = 0;

    int mode_ = 0;
    bool close_ = false;

private:
    void load_config(const std::string& path) {
        try {
            RCLCPP_INFO(this->get_logger(), "Loading config from: %s", path.c_str());
            YAML::Node yaml_config = YAML::LoadFile(path);
            cfg_.policy_dt = yaml_config["policy_dt"].as<float>();
            cfg_.physics_dt = yaml_config["physics_dt"].as<float>();
            cfg_.num_joints = yaml_config["num_joints"].as<int>();
            cfg_.num_actions = yaml_config["num_actions"].as<int>();
            cfg_.joint_kp = yaml_config["joint_kp"].as<std::vector<float>>();
            cfg_.joint_kd = yaml_config["joint_kd"].as<std::vector<float>>();
            cfg_.default_joint_positions = yaml_config["default_joint_positions"].as<std::vector<float>>();
            cfg_.effort_limits = yaml_config["effort_limits"].as<std::vector<float>>();
            cfg_.action_indices = yaml_config["action_indices"].as<std::vector<int>>();
            cfg_.num_observations = yaml_config["num_observations"].as<int>();
            cfg_.cutoff_freq = yaml_config["cutoff_freq"].as<float>();
        } catch (const YAML::Exception& e) {
            RCLCPP_FATAL(this->get_logger(), "Failed to load or parse config file: %s", e.what());
            rclcpp::shutdown();
        }
    }
    
    void joystick_callback(const msg_interfaces::msg::Joymsg::SharedPtr msg) {
        if (msg->buttons.size() > 7) {
            if (msg->buttons[0] == 1){
                mode_ = 1;
            }
            else if (msg->buttons[1] == 1){
                mode_ = 0;
            }

            reset_state_ = msg->buttons[7];
            if (reset_state_ == 1 && prev_reset_state_ == 0) {
                reset();
            }
            prev_reset_state_ = reset_state_;
        }

        if (mode_ == 1){
            if (msg->axes.size() > 1) {
                commands_[0] = (std::abs(msg->axes[1]) < 0.1) ? 0.0 : -msg->axes[1] * 1.0;
            }
            if (msg->axes.size() > 0) {
                commands_[1] = (std::abs(msg->axes[0]) < 0.1) ? 0.0 : -msg->axes[0] * 0.5;
            }
            if (msg->axes.size() > 2) {
                commands_[2] = (std::abs(msg->axes[2]) < 0.1) ? 0.0 : -msg->axes[2] * 0.5;
            }
        }
        else{
            commands_ = torch::zeros({3}, torch::kFloat32);
        }
        
        if (msg->cross.size() > 1) {
            if (msg->cross[0] == -1){
                close_ = true;
            }
        }
    }

    void action_callback(const msg_interfaces::msg::Action::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(action_mutex_);
        latest_actions_ = torch::from_blob(msg->action.data(), {cfg_.num_actions}, torch::kFloat32).clone();
    }

    void step(const torch::Tensor& actions) {
        std::lock_guard<std::mutex> lock(physics_mutex_);
        int substeps = static_cast<int>(round(cfg_.policy_dt / cfg_.physics_dt));
        for (int i = 0; i < substeps; ++i) {
            _apply_actions(actions);
            mj_step(model_, data_);
        }
        publish_observations();
    }
    
    void publish_observations() {
        auto obs_msg = msg_interfaces::msg::Observation();
        auto obs_clock = _get_clock();
        auto obs_base_vel = _get_base_vel();
        auto obs_base_ang_vel = _get_base_ang_vel();
        auto obs_gravity = _get_projected_gravity();
        auto obs_commands = _get_commands();
        // auto obs_joint_pos = _get_joint_pos() - torch::tensor(cfg_.default_joint_positions, torch::dtype(torch::kFloat32));
        auto obs_joint_pos = _get_joint_pos();
        auto obs_joint_vel = _get_joint_vel();
        
        obs_msg.clock.assign(obs_clock.data_ptr<float>(), obs_clock.data_ptr<float>() + obs_clock.numel());
        obs_msg.base_vel.assign(obs_base_vel.data_ptr<float>(), obs_base_vel.data_ptr<float>() + obs_base_vel.numel());
        obs_msg.base_ang_vel.assign(obs_base_ang_vel.data_ptr<float>(), obs_base_ang_vel.data_ptr<float>() + obs_base_ang_vel.numel());
        obs_msg.gravity.assign(obs_gravity.data_ptr<float>(), obs_gravity.data_ptr<float>() + obs_gravity.numel());
        obs_msg.commands.assign(obs_commands.data_ptr<float>(), obs_commands.data_ptr<float>() + obs_commands.numel());
        obs_msg.joint_pos.assign(obs_joint_pos.data_ptr<float>(), obs_joint_pos.data_ptr<float>() + obs_joint_pos.numel());
        obs_msg.joint_vel.assign(obs_joint_vel.data_ptr<float>(), obs_joint_vel.data_ptr<float>() + obs_joint_vel.numel());
        obs_msg.flag = close_;
        obs_pub_->publish(obs_msg);
    }

    void _apply_actions(const torch::Tensor& actions) {
        auto target_positions = torch::zeros({cfg_.num_joints}, torch::kFloat32);
        std::vector<long> indices;
        for(int i : cfg_.action_indices) indices.push_back(i);
        torch::Tensor index_tensor = torch::tensor(indices, torch::kLong);
        if (mode_ == 1){
            target_positions.index_put_({index_tensor}, actions);
        }
        else{
            target_positions = torch::from_blob(cfg_.default_joint_positions.data(), {cfg_.num_joints}, torch::kFloat32);
        }
        auto output_torques = joint_kp_ * (target_positions - _get_joint_pos()) + joint_kd_ * (-_get_joint_vel());
        auto output_torques_filtered = ema_filter_->filter(output_torques);
        auto output_torques_clipped = torch::clamp(output_torques_filtered, -effort_limits_, effort_limits_);
        
        auto torques_accessor = output_torques_clipped.accessor<float, 1>();
        for (int i = 0; i < model_->nu; ++i) {
            data_->ctrl[i] = torques_accessor[i];
        }
    }

    torch::Tensor _get_joint_pos() {
        return torch::from_blob(data_->sensordata, {cfg_.num_joints}, torch::kDouble).to(torch::kFloat32);
    }

    torch::Tensor _get_joint_vel() {
        return torch::from_blob(data_->sensordata + cfg_.num_joints, {cfg_.num_joints}, torch::kDouble).to(torch::kFloat32);
    }

    torch::Tensor _get_base_quat() {
        return torch::from_blob(data_->sensordata + 3 * model_->nu, {4}, torch::kDouble).to(torch::kFloat32);
    }

    torch::Tensor _get_base_ang_vel() {
        return torch::from_blob(data_->sensordata + 3 * model_->nu + 4, {3}, torch::kDouble).to(torch::kFloat32);
    }

    torch::Tensor _get_base_vel() {
        torch::Tensor base_quat = _get_base_quat();
        return quat_rotate_inverse(base_quat, torch::from_blob(data_->qvel, {3}, torch::kDouble).to(torch::kFloat32));
    }

    torch::Tensor _get_projected_gravity() {
        torch::Tensor base_quat = _get_base_quat();
        return quat_rotate_inverse(base_quat, gravity_vector_);
    }
    
    torch::Tensor _get_commands() {
        return commands_;
    }

    torch::Tensor _get_clock() {
        double total_time = data_->time;
        double phase = 2 * (0.15 + 0.3);
        double local_phi = fmod(total_time, phase);
        double phi = local_phi / phase;
        torch::Tensor clock = torch::tensor({(float)std::sin(2 * M_PI * phi), (float)std::cos(2 * M_PI * phi)}, torch::kFloat32);
        return clock;
    }

    rclcpp::Publisher<msg_interfaces::msg::Observation>::SharedPtr obs_pub_;
    rclcpp::Subscription<msg_interfaces::msg::Joymsg>::SharedPtr joy_sub_;
    rclcpp::Subscription<msg_interfaces::msg::Action>::SharedPtr action_sub_;

    std::mutex action_mutex_;
    std::mutex physics_mutex_;
    torch::Tensor latest_actions_;
    torch::Tensor default_actions_;
    torch::Tensor commands_;
    torch::Tensor joint_kp_, joint_kd_, effort_limits_;
    torch::Tensor gravity_vector_;
    std::unique_ptr<Ema> ema_filter_;

    int prev_reset_state_ = 0;
    int reset_state_ = 0;
};

void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    auto* env = static_cast<MujocoEnv*>(glfwGetWindowUserPointer(window));
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
        rclcpp::shutdown();
    }
    if (key == GLFW_KEY_BACKSPACE && action == GLFW_PRESS) {
        env->reset();
    }

    if (action == GLFW_PRESS || action == GLFW_REPEAT) {
        switch (key) {
            case GLFW_KEY_W: env->set_keyboard_command(0, 1.0); break;
            case GLFW_KEY_S: env->set_keyboard_command(0, -1.0); break;
            case GLFW_KEY_A: env->set_keyboard_command(1, 0.5); break;
            case GLFW_KEY_D: env->set_keyboard_command(1, -0.5); break;
            case GLFW_KEY_Q: env->set_keyboard_command(2, 0.5); break;
            case GLFW_KEY_E: env->set_keyboard_command(2, -0.5); break;
        }
    } 
    else if (action == GLFW_RELEASE) {
        switch (key) {
            case GLFW_KEY_W: case GLFW_KEY_S: env->set_keyboard_command(0, 0.0); break;
            case GLFW_KEY_A: case GLFW_KEY_D: env->set_keyboard_command(1, 0.0); break;
            case GLFW_KEY_Q: case GLFW_KEY_E: env->set_keyboard_command(2, 0.0); break;
        }
    }

    if (action == GLFW_PRESS || action == GLFW_REPEAT) {
        if (key == GLFW_KEY_P){
            if (env->close_ == false){
                env->close_ = true;
            }
            else if (env->close_ == true){
                env->close_ = false;
            }
        }
        
        if (key == GLFW_KEY_M){
            if (env->mode_ == 0){
                env->mode_ = 1;
            }
            else if (env->mode_ == 1){
                env->mode_ = 0;
            }
        }
    }
}

void mouse_button_callback(GLFWwindow* window, int button, int action, int mods) {
    auto* env = static_cast<MujocoEnv*>(glfwGetWindowUserPointer(window));
    env->button_left_ = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
    env->button_middle_ = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
    env->button_right_ = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);
    glfwGetCursorPos(window, &env->lastx_, &env->lasty_);
}

void mouse_move_callback(GLFWwindow* window, double xpos, double ypos) {
    auto* env = static_cast<MujocoEnv*>(glfwGetWindowUserPointer(window));
    if (!env->button_left_ && !env->button_middle_ && !env->button_right_) {
        return;
    }
    double dx = xpos - env->lastx_;
    double dy = ypos - env->lasty_;
    env->lastx_ = xpos;
    env->lasty_ = ypos;
    
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS || 
                      glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

    mjtMouse action;
    if (env->button_right_) {
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    } else if (env->button_left_) {
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    } else {
        action = mjMOUSE_ZOOM;
    }
    mjv_moveCamera(env->model_, action, dx / height, dy / height, &env->scn_, &env->cam_);
}

void scroll_callback(GLFWwindow* window, double xoffset, double yoffset) {
    auto* env = static_cast<MujocoEnv*>(glfwGetWindowUserPointer(window));
    mjv_moveCamera(env->model_, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &env->scn_, &env->cam_);
}


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto mujoco_node = std::make_shared<MujocoEnv>();

    if (!mujoco_node->model_) {
        return -1;
    }

    if (!glfwInit()) {
        std::cerr << "Could not initialize GLFW." << std::endl;
        return -1;
    }
    GLFWwindow* window = glfwCreateWindow(1200, 900, "MuJoCo ROS 2 Viewer", NULL, NULL);
    if (!window) {
        glfwTerminate();
        std::cerr << "Could not create GLFW window." << std::endl;
        return -1;
    }
    glfwMakeContextCurrent(window);
    mujoco_node->initialize_viewer(window);

    glfwSetWindowUserPointer(window, mujoco_node.get());
    glfwSetKeyCallback(window, key_callback);
    glfwSetCursorPosCallback(window, mouse_move_callback);
    glfwSetMouseButtonCallback(window, mouse_button_callback);
    glfwSetScrollCallback(window, scroll_callback);

    std::thread spin_thread([&]() {
        rclcpp::spin(mujoco_node);
    });

    mujoco_node->reset();
    auto policy_period = std::chrono::duration<double>(mujoco_node->cfg_.policy_dt);

    while (!glfwWindowShouldClose(window) && rclcpp::ok() && !mujoco_node->close_) {
        auto step_start = std::chrono::steady_clock::now();
        
        mujoco_node->run_step();
        mujoco_node->render();

        auto step_end = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = step_end - step_start;
            
        if (elapsed < policy_period) {
            auto sleep_duration = std::chrono::duration_cast<std::chrono::nanoseconds>(policy_period - elapsed);
            rclcpp::sleep_for(sleep_duration);
        }
    }

    mujoco_node->shutdown_msg();

    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }

    if (spin_thread.joinable()) {
        spin_thread.join();
    }
    glfwTerminate();
    return 0;
}