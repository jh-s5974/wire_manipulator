#include <iostream>
#include <string>
#include <vector>
#include <chrono>
#include <thread>
#include <cmath>
#include <algorithm>
#include <stdexcept>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "yaml-cpp/yaml.h"
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>

//--- 설정 값 (전역 변수) ---
int NUM_JOINTS;
std::vector<double> JOINT_KP;
std::vector<double> JOINT_KD;
std::vector<double> DEFAULT_JOINT_POS;
std::vector<double> EFFORT_LIMITS;
double PHYSICS_DT;
double POLICY_DT;

//--- 전역 변수 ---
mjModel* m = NULL;
mjData* d = NULL;
mjvCamera cam;
mjvOption opt;
mjvScene scn;
mjrContext con;

bool button_left = false;
bool button_middle = false;
bool button_right = false;
double lastx = 0;
double lasty = 0;

double commands[3] = {0.0, 0.0, 0.0};
std::vector<double> target_joint_pos;

//--- 함수 선언 ---
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods);
void mouse_button_callback(GLFWwindow* window, int button, int act, int mods);
void mouse_move_callback(GLFWwindow* window, double xpos, double ypos);
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);

//--- YAML 설정 로드 ---
bool load_config(const std::string& config_path) {
    try {
        YAML::Node config = YAML::LoadFile(config_path);

        NUM_JOINTS = config["num_joints"].as<int>();
        JOINT_KP = config["joint_kp"].as<std::vector<double>>();
        JOINT_KD = config["joint_kd"].as<std::vector<double>>();
        DEFAULT_JOINT_POS = config["default_joint_positions"].as<std::vector<double>>();
        EFFORT_LIMITS = config["effort_limits"].as<std::vector<double>>();
        PHYSICS_DT = config["physics_dt"].as<double>();
        POLICY_DT = config["policy_dt"].as<double>();

        std::cout << "Config loaded successfully!" << std::endl;
        std::cout << " - Num Joints: " << NUM_JOINTS << std::endl;
        std::cout << " - Joint Kp values: ";
        for (int i = 0; i < NUM_JOINTS; ++i) {
            std::cout << JOINT_KP[i] << (i == NUM_JOINTS - 1 ? "" : ", ");
        }
        std::cout << std::endl;

    } catch (const YAML::Exception& e) {
        std::cerr << "Error loading or parsing config file: " << e.what() << std::endl;
        return false;
    }
    return true;
}

//--- 제어 및 유틸리티 함수 ---
std::vector<double> get_joint_pos() {
    std::vector<double> pos(NUM_JOINTS);
    for (int i = 0; i < NUM_JOINTS; ++i) pos[i] = d->sensordata[i];
    return pos;
}

std::vector<double> get_joint_vel() {
    std::vector<double> vel(NUM_JOINTS);
    for (int i = 0; i < NUM_JOINTS; ++i) vel[i] = d->sensordata[NUM_JOINTS + i];
    return vel;
}

void apply_pd_actions() {
    std::vector<double> current_pos = get_joint_pos();
    std::vector<double> current_vel = get_joint_vel();
    std::vector<double> output_torques(NUM_JOINTS);

    for (int i = 0; i < NUM_JOINTS; ++i) {
        double pos_error = target_joint_pos[i] - current_pos[i];
        double vel_error = 0 - current_vel[i];
        output_torques[i] = JOINT_KP[i] * pos_error + JOINT_KD[i] * vel_error;
    }

    for (int i = 0; i < NUM_JOINTS; ++i) {
        output_torques[i] = std::max(-EFFORT_LIMITS[i], std::min(EFFORT_LIMITS[i], output_torques[i]));
    }

    for (int i = 0; i < NUM_JOINTS; ++i) {
        d->ctrl[i] = output_torques[i];
    }
}

//--- 콜백 함수 정의 ---
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods) {
    if (act == GLFW_PRESS || act == GLFW_REPEAT) {
        switch (key) {
            case GLFW_KEY_W: commands[0] = 1.0; break;
            case GLFW_KEY_S: commands[0] = -1.0; break;
            case GLFW_KEY_A: commands[1] = 0.5; break;
            case GLFW_KEY_D: commands[1] = -0.5; break;
            case GLFW_KEY_Q: commands[2] = 0.5; break;
            case GLFW_KEY_E: commands[2] = -0.5; break;
        }
    } else if (act == GLFW_RELEASE) {
        switch (key) {
            case GLFW_KEY_W: case GLFW_KEY_S: commands[0] = 0.0; break;
            case GLFW_KEY_A: case GLFW_KEY_D: commands[1] = 0.0; break;
            case GLFW_KEY_Q: case GLFW_KEY_E: commands[2] = 0.0; break;
        }
    }
}

void mouse_button_callback(GLFWwindow* window, int button, int act, int mods) {
    button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
    button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);
    glfwGetCursorPos(window, &lastx, &lasty);
}

void mouse_move_callback(GLFWwindow* window, double xpos, double ypos) {
    if (!button_left && !button_middle && !button_right) return;
    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;
    int width, height;
    glfwGetWindowSize(window, &width, &height);
    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);
    mjtMouse action;
    if (button_right) action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    else if (button_left) action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else action = mjMOUSE_ZOOM;
    mjv_moveCamera(m, action, dx / height, -dy / height, &scn, &cam);
}

void scroll_callback(GLFWwindow* window, double xoffset, double yoffset) {
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &scn, &cam);
}

//--- Main 함수 ---
int main(int argc, char ** argv) {
    std::string package_share_dir = ament_index_cpp::get_package_share_directory("mujoco_rl");
    std::string config_path = package_share_dir + "/config/robotnl.yaml";
    if (!load_config(config_path)) {
        return -1;
    }

    target_joint_pos.resize(NUM_JOINTS);

    std::string xml_path = package_share_dir + "/mjcf/robot_nl.xml";

    char error[1024] = {0};
    m = mj_loadXML(xml_path.c_str(), nullptr, error, 1024);
    if (!m) { mju_error_s("Load model error: %s", error); return -1; }
    d = mj_makeData(m);

    m->opt.timestep = PHYSICS_DT;

    for (int i = 0; i < NUM_JOINTS; ++i) {
        target_joint_pos[i] = DEFAULT_JOINT_POS[i];
    }
    
    if (!glfwInit()) return -1;
    GLFWwindow* window = glfwCreateWindow(1200, 900, "MuJoCo C++ PD Control", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_makeScene(m, &scn, 2000);
    mjr_defaultContext(&con);
    mjr_makeContext(m, &con, mjFONTSCALE_150);

    glfwSetKeyCallback(window, keyboard);
    glfwSetMouseButtonCallback(window, mouse_button_callback);
    glfwSetCursorPosCallback(window, mouse_move_callback);
    glfwSetScrollCallback(window, scroll_callback);

    while (!glfwWindowShouldClose(window)) {
        auto step_start = std::chrono::high_resolution_clock::now();

        apply_pd_actions();
        
        int substeps = static_cast<int>(POLICY_DT / PHYSICS_DT);
        for(int i = 0; i < substeps; ++i) {
            mj_step(m, d);
        }

        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
        mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);
        glfwSwapBuffers(window);
        glfwPollEvents();

        auto step_end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = step_end - step_start;
        double time_until_next_step = POLICY_DT - elapsed.count();
        if (time_until_next_step > 0) {
            std::this_thread::sleep_for(std::chrono::duration<double>(time_until_next_step));
        }
    }
    
    mj_deleteData(d);
    mj_deleteModel(m);
    mjr_freeContext(&con);
    mjv_freeScene(&scn);
    glfwTerminate();

    return 0;
}