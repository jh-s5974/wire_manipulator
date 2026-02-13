#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "msg_interfaces/msg/observation.hpp"
#include "msg_interfaces/msg/action.hpp"
#include "yaml-cpp/yaml.h"
#include <torch/script.h>
#include <vector>
#include <string>
#include <algorithm>
#include <numeric>
#include <cmath>
#include <atomic>

// YAML 설정 값을 담을 구조체
struct PolicyConfig {
    std::string policy_checkpoint_path;
    std::string policy_stand_path;
    int num_observations;
    int num_actions;
    int history_length;
    std::vector<float> default_joint_positions;
    float action_limit_lower;
    float action_limit_upper;
    float action_scale;
};

class PolicyCalNode : public rclcpp::Node {
public:
    PolicyCalNode() : Node("policy_runner_node") {
        this->initialize();
    }

private:
    void initialize() {
        // 1. 설정 파일 파싱
        try {
            std::string package_share_dir = ament_index_cpp::get_package_share_directory("mujoco_rl");
            std::string config_file_path = package_share_dir + "/config/robotnl.yaml";
            RCLCPP_INFO(this->get_logger(), "Loading config file from %s", config_file_path.c_str());
            YAML::Node config_yaml = YAML::LoadFile(config_file_path);

            cfg_.policy_checkpoint_path = package_share_dir + "/" + config_yaml["policy_checkpoint_path"].as<std::string>();
            cfg_.policy_stand_path = package_share_dir + "/" + config_yaml["policy_stand_path"].as<std::string>();
            cfg_.num_observations = config_yaml["num_observations"].as<int>();
            cfg_.num_actions = config_yaml["num_actions"].as<int>();
            cfg_.history_length = config_yaml["history_length"].as<int>();
            cfg_.default_joint_positions = config_yaml["default_joint_positions"].as<std::vector<float>>();
            cfg_.action_limit_lower = config_yaml["action_limit_lower"].as<float>();
            cfg_.action_limit_upper = config_yaml["action_limit_upper"].as<float>();
            cfg_.action_scale = config_yaml["action_scale"].as<float>();
        }
        catch (const YAML::Exception& e) {
            RCLCPP_FATAL(this->get_logger(), "Failed to load or parse config file: %s", e.what());
            rclcpp::shutdown();
            return;
        }

        // 2. 모델 로딩
        if (cfg_.policy_checkpoint_path.find(".pt") != std::string::npos) {
            runner_type_ = "torch";
            try {
                policy_pt_ = torch::jit::load(cfg_.policy_checkpoint_path);
                stand_pt_ = torch::jit::load(cfg_.policy_stand_path);
                RCLCPP_INFO(this->get_logger(), "Using Torch runner");
                RCLCPP_INFO(this->get_logger(), "policy_pt : %s", cfg_.policy_checkpoint_path.c_str());
                RCLCPP_INFO(this->get_logger(), "stand_pt : %s", cfg_.policy_stand_path.c_str());
            } catch (const c10::Error& e) {
                RCLCPP_FATAL(this->get_logger(), "Failed to load Torch model: %s", e.what());
                rclcpp::shutdown();
                return;
            }
        }
        else {
            RCLCPP_FATAL(this->get_logger(), "Unrecognized policy format");
            rclcpp::shutdown();
            return;
        }

        // 3. 변수 초기화
        size_t history_size = cfg_.num_observations * (cfg_.history_length + 1);
        policy_observations_.resize(history_size, 0.0f);
        pre_actions_.resize(cfg_.num_actions, 0.0f);
        actions_scaled.resize(cfg_.num_actions, 0.0f);
        // std::cout << "test: " << policy_observations_ << std::endl;

        // --- 조인트 이름 목록 초기화 ---
        // joint_names_ = {
        //     "hip_yaw_left",         "hip_yaw_right",
        //     "hip_roll_left",        "hip_roll_right",
        //     "hip_pitch_left",       "hip_pitch_right",
        //     "knee_left",            "knee_right",
        //     "ankle_pitch_left",     "ankle_pitch_right",
        //     "ankle_roll_left",       "ankle_roll_right"
        // };

        joint_names_ = {
            "hip_yaw_left",         
            "hip_yaw_right",
            "hip_roll_left",        
            "hip_roll_right",
            "hip_pitch_left",       
            "hip_pitch_right",
            "knee_left",            
            "knee_right",
            "ankle_pitch_left",     
            "ankle_pitch_right",
            "ankle_roll_left"     
            "ankle_roll_right",
        };
        // ------------------------------------

        // 4. ROS 퍼블리셔/서브스크라이버 생성
        action_publisher_ = this->create_publisher<msg_interfaces::msg::Action>("action", 10);
        observation_subscriber_ = this->create_subscription<msg_interfaces::msg::Observation>(
            "mujoco_obs",
            10,
            std::bind(&PolicyCalNode::observation_callback, this, std::placeholders::_1)
        );
        RCLCPP_INFO(this->get_logger(), "Policy Runner Node has been initialized.");
    }

    void observation_callback(const msg_interfaces::msg::Observation::SharedPtr msg) {
        if (!(msg->flag)) {
            RCLCPP_INFO(this->get_logger(), "Shutdown flag received. Shutting down policy node...");
            std::thread([](){ rclcpp::shutdown(); }).detach();
            return;
        }
        // obs_count++;

        // 1. Observation 데이터 업데이트
        policy_observations_.erase(
            policy_observations_.begin(),
            policy_observations_.begin() + cfg_.num_observations
        );

        policy_observations_.insert(policy_observations_.end(), msg->clock.begin(), msg->clock.end());
        policy_observations_.insert(policy_observations_.end(), msg->base_ang_vel.begin(), msg->base_ang_vel.end());
        policy_observations_.insert(policy_observations_.end(), msg->gravity.begin(), msg->gravity.end());
        policy_observations_.insert(policy_observations_.end(), msg->commands.begin(), msg->commands.end());
        policy_observations_.insert(policy_observations_.end(), msg->joint_pos.begin(), msg->joint_pos.end());
        policy_observations_.insert(policy_observations_.end(), msg->joint_vel.begin(), msg->joint_vel.end());
        policy_observations_.insert(policy_observations_.end(), pre_actions_.begin(), pre_actions_.end());

        // std::cout << "test: " << policy_observations_ << std::endl;
        
        // 2. 모델 추론
        std::vector<float> actions_raw(cfg_.num_actions);
        if (runner_type_ == "torch") {
            torch::Tensor input_tensor = torch::from_blob(policy_observations_.data(), {1, static_cast<long>(policy_observations_.size())}, torch::kFloat32);
            at::Tensor output_tensor;
            float commands_norm = std::sqrt(std::inner_product(msg->commands.begin(), msg->commands.end(), msg->commands.begin(), 0.0f));
            if((commands_norm < 0.1f)){
                output_tensor = stand_pt_.forward({input_tensor}).toTensor();
            }
            else{
                output_tensor = policy_pt_.forward({input_tensor}).toTensor();
            }
            std::memcpy(actions_raw.data(), output_tensor.data_ptr<float>(), actions_raw.size() * sizeof(float));
        }
        

        // 3. 행동 후처리 (Clip, Scale)
        std::vector<float> actions_clipped(cfg_.num_actions);
        for (size_t i = 0; i < cfg_.num_actions; ++i) {
            actions_clipped[i] = std::clamp(actions_raw[i], cfg_.action_limit_lower, cfg_.action_limit_upper);
        }
        if (msg->obs_flag){
            pre_actions_ = actions_clipped; // 다음 스텝을 위해 저장
            
            for (size_t i = 0; i < cfg_.num_actions; ++i) {
                actions_scaled[i] = actions_clipped[i] * cfg_.action_scale + cfg_.default_joint_positions[i];
            }
        }

        // --- CHANGED: 행동 퍼블리시 시 조인트 이름 추가 ---
        auto action_msg = msg_interfaces::msg::Action();
        action_msg.header.stamp = this->now();
        action_msg.action = actions_scaled;
        action_msg.name = joint_names_; // 조인트 이름 목록 할당
        action_publisher_->publish(action_msg);
        // if (obs_count == 4){
        //     action_publisher_->publish(action_msg);
        //     obs_count = 0;
        // }
        // ---------------------------------------------
    }

    rclcpp::Publisher<msg_interfaces::msg::Action>::SharedPtr action_publisher_;
    rclcpp::Subscription<msg_interfaces::msg::Observation>::SharedPtr observation_subscriber_;

    PolicyConfig cfg_;
    std::string runner_type_;

    // Model-related members
    torch::jit::script::Module policy_pt_;
    torch::jit::script::Module stand_pt_;

    // Data buffers
    std::vector<float> policy_observations_;
    std::vector<float> pre_actions_;
    std::vector<float> actions_scaled;
    std::vector<std::string> joint_names_; // ADDED: 조인트 이름 저장 변수
    int obs_count = 0;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PolicyCalNode>();

    rclcpp::spin(node);

    RCLCPP_INFO(node->get_logger(), "Spin has finished. Exiting main.");
    return 0;
}
