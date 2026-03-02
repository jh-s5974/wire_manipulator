#pragma once

#include <rtfw/task.h>
#include "custom_types.hpp"

#include <manif/manif.h>
#include <Eigen/Dense>

#include <array>
#include <memory>
#include <string>
#include <vector>

using namespace std::chrono_literals;
using namespace rtfw;
using namespace rtfw::rt;

namespace task_pool {

struct PolicyRunnerState {
	std::array<custom_types::MotorState, 12> motor;
	std::array<bool, 12> motor_online;
	Eigen::Quaterniond base_quat;
	Eigen::Vector3d base_ang_vel;
	Eigen::Vector3d cmd_vel;
	bool imu_online;
	bool activated;
    int cnt;
};

class PolicyRunner : public Task<PolicyRunnerState> {
public:
	const char* getName() const override { return "PolicyRunner"; }
	void initialize(PolicyRunnerState& s) override;
	void execute(PolicyRunnerState& s) override;

private:
	struct Impl;
	std::shared_ptr<Impl> impl_;

	DataReader<manif::SE2Tangentd> dr_cmd_vel{"manager/cmd_vel"};
	DataReader<bool> dr_rl_signal{"manager/rl_signal"};
	DataReader<custom_types::Imu> dr_imu{"imu_data", DependencyType::Weak};

	DataReader<custom_types::MotorState> dr_mtr_stat[12] = {
		DataReader<custom_types::MotorState>{"hip_yaw_left/state", DependencyType::Weak},
		DataReader<custom_types::MotorState>{"hip_yaw_right/state", DependencyType::Weak},
		DataReader<custom_types::MotorState>{"hip_roll_left/state", DependencyType::Weak},
		DataReader<custom_types::MotorState>{"hip_roll_right/state", DependencyType::Weak},
		DataReader<custom_types::MotorState>{"hip_pitch_left/state", DependencyType::Weak},
		DataReader<custom_types::MotorState>{"hip_pitch_right/state", DependencyType::Weak},
		DataReader<custom_types::MotorState>{"knee_left/state", DependencyType::Weak},
		DataReader<custom_types::MotorState>{"knee_right/state", DependencyType::Weak},
		DataReader<custom_types::MotorState>{"ankle_pitch_left/state", DependencyType::Weak},
		DataReader<custom_types::MotorState>{"ankle_pitch_right/state", DependencyType::Weak},
		DataReader<custom_types::MotorState>{"ankle_roll_left/state", DependencyType::Weak},
		DataReader<custom_types::MotorState>{"ankle_roll_right/state", DependencyType::Weak},
	};

	DataWriter<custom_types::MotorCmd> dw_action[12] = {
		DataWriter<custom_types::MotorCmd>{"hip_yaw_left/action"},
		DataWriter<custom_types::MotorCmd>{"hip_yaw_right/action"},
		DataWriter<custom_types::MotorCmd>{"hip_roll_left/action"},
		DataWriter<custom_types::MotorCmd>{"hip_roll_right/action"},
		DataWriter<custom_types::MotorCmd>{"hip_pitch_left/action"},
		DataWriter<custom_types::MotorCmd>{"hip_pitch_right/action"},
		DataWriter<custom_types::MotorCmd>{"knee_left/action"},
		DataWriter<custom_types::MotorCmd>{"knee_right/action"},
		DataWriter<custom_types::MotorCmd>{"ankle_pitch_left/action"},
		DataWriter<custom_types::MotorCmd>{"ankle_pitch_right/action"},
		DataWriter<custom_types::MotorCmd>{"ankle_roll_left/action"},
		DataWriter<custom_types::MotorCmd>{"ankle_roll_right/action"},
	};

	Parameter<std::string> p_policy_checkpoint_path{"policy_checkpoint_path", "policy/policy47_7.pt"};
	Parameter<std::string> p_policy_stand_path{"policy_stand_path", "policy/policy_stand3.pt"};
	Parameter<int> p_num_observations{"num_observations", 47};
	Parameter<int> p_num_actions{"num_actions", 12};
	Parameter<int> p_history_length{"history_length", 0};
	Parameter<double> p_action_limit_lower{"action_limit_lower", -10000.0};
	Parameter<double> p_action_limit_upper{"action_limit_upper", 10000.0};
	Parameter<double> p_action_scale{"action_scale", 0.25};
	Parameter<double> p_clock_hz{"policy.clock_hz", 1.0};

	Parameter<std::vector<std::string>> p_joint_names{"joints", std::vector<std::string>()};
	Parameter<std::vector<double>> p_default_joint_positions{"default_joint_positions"};
	Parameter<std::vector<double>> p_joint_kp{"joint_kp"};
	Parameter<std::vector<double>> p_joint_kd{"joint_kd"};

	int num_observations_ = 47;
	int num_actions_ = 12;
	int history_length_ = 0;
	float action_limit_lower_ = -10000.0f;
	float action_limit_upper_ = 10000.0f;
	float action_scale_ = 0.25f;
	double clock_hz_ = 1.0;

	std::vector<std::string> joint_names_;
	std::vector<double> default_joint_positions_;
	std::vector<double> joint_kp_;
	std::vector<double> joint_kd_;

	std::vector<float> policy_observations_;
	std::vector<float> pre_actions_;
	std::vector<float> actions_scaled_;
	std::vector<float> actions_raw_;
	std::vector<float> actions_clipped_;
	std::vector<float> single_observation_;

	bool input_ready(const PolicyRunnerState& s) const;
	void build_observation(const PolicyRunnerState& s);
	void run_inference(const Eigen::Vector3d& cmd_vel);
};

} // namespace task_pool
