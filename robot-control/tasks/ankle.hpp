#pragma once

#include <rtfw/task.h>
#include "custom_types.hpp"
#include "util.hpp"
#include "kin_2rsu.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <limits>
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


class AnkleFK : public ITask {
public:
    const char* getName() const override { return "AnkleFK"; }

    void initialize(void*) override {
        for (auto& s : motor_state_) s = {};
    }

    // motor state (ll/lr/rl/rr) → ankle_fk() → joint state (pitch/roll)
    // Joint vector layout: [R, P] = [roll, pitch] = [rp(0), rp(1)]
    // Jacobian: d_joint = J * d_motor
    //   vel_joint  = J * vel_motor
    //   tau_joint  = J^{-T} * tau_motor
    void execute(void* s) override {
        bool left_updated = false;
        bool right_updated = false;

        dr_mtr_stat_[0].on_update([&](const custom_types::MotorState& data) {
            motor_state_[0] = data; left_updated = true;
        });
        dr_mtr_stat_[1].on_update([&](const custom_types::MotorState& data) {
            motor_state_[1] = data; left_updated = true;
        });
        dr_mtr_stat_[2].on_update([&](const custom_types::MotorState& data) {
            motor_state_[2] = data; right_updated = true;
        });
        dr_mtr_stat_[3].on_update([&](const custom_types::MotorState& data) {
            motor_state_[3] = data; right_updated = true;
        });

        if (left_updated) {
            // t1=ll[0], t2=lr[1], side='l'
            auto [rp, jac] = kin_2rsu::ankle_fk(
                (float)motor_state_[0].pos, (float)motor_state_[1].pos, 'l');

            Eigen::Vector2f mvel((float)motor_state_[0].vel,    (float)motor_state_[1].vel);
            Eigen::Vector2f mtrq((float)motor_state_[0].torque, (float)motor_state_[1].torque);
            Eigen::Vector2f jvel = jac * mvel;
            Eigen::Vector2f jtrq = jac.inverse().transpose() * mtrq;

            custom_types::MotorState pitch_stat{};
            pitch_stat.pos    = rp(1);   pitch_stat.vel    = jvel(1);
            pitch_stat.torque = jtrq(1); pitch_stat.status = motor_state_[0].status;

            custom_types::MotorState roll_stat{};
            roll_stat.pos    = rp(0);   roll_stat.vel    = jvel(0);
            roll_stat.torque = jtrq(0); roll_stat.status = motor_state_[0].status;

            dw_joint_stat_[0].write(pitch_stat);   // ankle_pitch_left/state
            dw_joint_stat_[2].write(roll_stat);    // ankle_roll_left/state
        }

        if (right_updated) {
            // t1=rl[2], t2=rr[3], side='r'
            auto [rp, jac] = kin_2rsu::ankle_fk(
                (float)motor_state_[2].pos, (float)motor_state_[3].pos, 'r');

            Eigen::Vector2f mvel((float)motor_state_[2].vel,    (float)motor_state_[3].vel);
            Eigen::Vector2f mtrq((float)motor_state_[2].torque, (float)motor_state_[3].torque);
            Eigen::Vector2f jvel = jac * mvel;
            Eigen::Vector2f jtrq = jac.inverse().transpose() * mtrq;

            custom_types::MotorState pitch_stat{};
            pitch_stat.pos    = rp(1);   pitch_stat.vel    = jvel(1);
            pitch_stat.torque = jtrq(1); pitch_stat.status = motor_state_[2].status;

            custom_types::MotorState roll_stat{};
            roll_stat.pos    = rp(0);   roll_stat.vel    = jvel(0);
            roll_stat.torque = jtrq(0); roll_stat.status = motor_state_[2].status;

            dw_joint_stat_[1].write(pitch_stat);   // ankle_pitch_right/state
            dw_joint_stat_[3].write(roll_stat);    // ankle_roll_right/state
        }
    }


private:
    custom_types::MotorState motor_state_[4];  // ll[0], lr[1], rl[2], rr[3]

    DataReader<custom_types::MotorCmd> dr_motor_cmd[4] = {
        DataReader<custom_types::MotorCmd>{"ankle_upper_left/cmd"},
        DataReader<custom_types::MotorCmd>{"ankle_upper_right/cmd"},
        DataReader<custom_types::MotorCmd>{"ankle_lower_left/cmd"},
        DataReader<custom_types::MotorCmd>{"ankle_lower_right/cmd"},
    };

    DataWriter<custom_types::MotorCmd> dw_joint_cmd_[4] = {
        DataWriter<custom_types::MotorCmd>{"ankle_pitch_left/cmd"},
        DataWriter<custom_types::MotorCmd>{"ankle_pitch_right/cmd"},
        DataWriter<custom_types::MotorCmd>{"ankle_roll_left/cmd"},
        DataWriter<custom_types::MotorCmd>{"ankle_roll_right/cmd"},
    };

    DataReader<custom_types::MotorState> dr_mtr_stat_[4] = {
        DataReader<custom_types::MotorState>{"ankle_upper_left/state", DependencyType::Weak},
        DataReader<custom_types::MotorState>{"ankle_upper_right/state", DependencyType::Weak},
        DataReader<custom_types::MotorState>{"ankle_lower_left/state", DependencyType::Weak},
        DataReader<custom_types::MotorState>{"ankle_lower_right/state", DependencyType::Weak},
    };

    DataWriter<custom_types::MotorState> dw_joint_stat_[4] = {
        DataWriter<custom_types::MotorState>{"ankle_pitch_left/state"},
        DataWriter<custom_types::MotorState>{"ankle_pitch_right/state"},
        DataWriter<custom_types::MotorState>{"ankle_roll_left/state"},
        DataWriter<custom_types::MotorState>{"ankle_roll_right/state"},
    };

};


class AnkleIK : public ITask {
public:
    const char* getName() const override { return "AnkleIK"; }

    void initialize(void*) override {
        for (auto& c : joint_cmd_) c = {};
        for (auto& s : joint_stat_) s = {};
    }

    // joint cmd (pitch/roll) → ankle_ik() + ankle_J() → motor cmd (ll/lr/rl/rr)
    // Jacobian: d_joint = J * d_motor
    //   vel_motor  = J^{-1} * vel_joint
    //   tau_motor  = J^T   * tau_joint
    void execute(void* s) override {
        bool left_cmd_updated  = false;
        bool right_cmd_updated = false;

        // Cache joint commands
        dr_joint_cmd_[0].on_update([&](const custom_types::MotorCmd& data) {
            joint_cmd_[0] = data; left_cmd_updated = true;   // pitch_left
        });
        dr_joint_cmd_[1].on_update([&](const custom_types::MotorCmd& data) {
            joint_cmd_[1] = data; right_cmd_updated = true;  // pitch_right
        });
        dr_joint_cmd_[2].on_update([&](const custom_types::MotorCmd& data) {
            joint_cmd_[2] = data; left_cmd_updated = true;   // roll_left
        });
        dr_joint_cmd_[3].on_update([&](const custom_types::MotorCmd& data) {
            joint_cmd_[3] = data; right_cmd_updated = true;  // roll_right
        });

        // Cache joint states (for Jacobian at current configuration)
        dr_joint_stat_[0].on_update([&](const custom_types::MotorState& data) { joint_stat_[0] = data; });
        dr_joint_stat_[1].on_update([&](const custom_types::MotorState& data) { joint_stat_[1] = data; });
        dr_joint_stat_[2].on_update([&](const custom_types::MotorState& data) { joint_stat_[2] = data; });
        dr_joint_stat_[3].on_update([&](const custom_types::MotorState& data) { joint_stat_[3] = data; });

        if (left_cmd_updated) {
            // roll_left=joint_cmd_[2], pitch_left=joint_cmd_[0] → ll[0], lr[1]
            float R_cmd = (float)joint_cmd_[2].pos;
            float P_cmd = (float)joint_cmd_[0].pos;
            Eigen::Vector2f motor_pos = kin_2rsu::ankle_ik(R_cmd, P_cmd, 'l');

            // Jacobian at current joint state
            float R_cur = (float)joint_stat_[2].pos;
            float P_cur = (float)joint_stat_[0].pos;
            Eigen::Vector2f motor_cur = kin_2rsu::ankle_ik(R_cur, P_cur, 'l');
            Eigen::Matrix2f jac = kin_2rsu::ankle_J(R_cur, P_cur, motor_cur(0), motor_cur(1), 'l');
            Eigen::Matrix2f jac_inv = jac.inverse();

            // vel: [roll_vel, pitch_vel] in joint space → motor space
            Eigen::Vector2f jvel((float)joint_cmd_[2].vel, (float)joint_cmd_[0].vel);
            Eigen::Vector2f mvel = jac_inv * jvel;

            // torque: tau_motor = J^T * tau_joint
            Eigen::Vector2f jtrq((float)joint_cmd_[2].torque, (float)joint_cmd_[0].torque);
            Eigen::Vector2f mtrq = jac.transpose() * jtrq;

            custom_types::MotorCmd ll_cmd{};
            ll_cmd.pos = motor_pos(0); ll_cmd.vel = mvel(0); ll_cmd.torque = mtrq(0);
            ll_cmd.kp  = joint_cmd_[0].kp; ll_cmd.kd = joint_cmd_[0].kd;

            custom_types::MotorCmd lr_cmd{};
            lr_cmd.pos = motor_pos(1); lr_cmd.vel = mvel(1); lr_cmd.torque = mtrq(1);
            lr_cmd.kp  = joint_cmd_[0].kp; lr_cmd.kd = joint_cmd_[0].kd;

            dw_motor_cmd[0].write(ll_cmd);   // ankle_ll/cmd
            dw_motor_cmd[1].write(lr_cmd);   // ankle_lr/cmd
        }

        if (right_cmd_updated) {
            // roll_right=joint_cmd_[3], pitch_right=joint_cmd_[1] → rl[2], rr[3]
            float R_cmd = (float)joint_cmd_[3].pos;
            float P_cmd = (float)joint_cmd_[1].pos;
            Eigen::Vector2f motor_pos = kin_2rsu::ankle_ik(R_cmd, P_cmd, 'r');

            float R_cur = (float)joint_stat_[3].pos;
            float P_cur = (float)joint_stat_[1].pos;
            Eigen::Vector2f motor_cur = kin_2rsu::ankle_ik(R_cur, P_cur, 'r');
            Eigen::Matrix2f jac = kin_2rsu::ankle_J(R_cur, P_cur, motor_cur(0), motor_cur(1), 'r');
            Eigen::Matrix2f jac_inv = jac.inverse();

            Eigen::Vector2f jvel((float)joint_cmd_[3].vel, (float)joint_cmd_[1].vel);
            Eigen::Vector2f mvel = jac_inv * jvel;

            Eigen::Vector2f jtrq((float)joint_cmd_[3].torque, (float)joint_cmd_[1].torque);
            Eigen::Vector2f mtrq = jac.transpose() * jtrq;

            custom_types::MotorCmd rl_cmd{};
            rl_cmd.pos = motor_pos(0); rl_cmd.vel = mvel(0); rl_cmd.torque = mtrq(0);
            rl_cmd.kp  = joint_cmd_[1].kp; rl_cmd.kd = joint_cmd_[1].kd;

            custom_types::MotorCmd rr_cmd{};
            rr_cmd.pos = motor_pos(1); rr_cmd.vel = mvel(1); rr_cmd.torque = mtrq(1);
            rr_cmd.kp  = joint_cmd_[1].kp; rr_cmd.kd = joint_cmd_[1].kd;

            dw_motor_cmd[2].write(rl_cmd);   // ankle_rl/cmd
            dw_motor_cmd[3].write(rr_cmd);   // ankle_rr/cmd
        }
    }


private:
    custom_types::MotorCmd   joint_cmd_[4];   // pitch_left[0], pitch_right[1], roll_left[2], roll_right[3]
    custom_types::MotorState joint_stat_[4];  // same order

    DataReader<custom_types::MotorCmd> dr_joint_cmd_[4] = {
        DataReader<custom_types::MotorCmd>{"ankle_pitch_left/cmd"},
        DataReader<custom_types::MotorCmd>{"ankle_pitch_right/cmd"},
        DataReader<custom_types::MotorCmd>{"ankle_roll_left/cmd"},
        DataReader<custom_types::MotorCmd>{"ankle_roll_right/cmd"},
    };

    DataWriter<custom_types::MotorCmd> dw_motor_cmd[4] = {
        DataWriter<custom_types::MotorCmd>{"ankle_upper_left/cmd"},
        DataWriter<custom_types::MotorCmd>{"ankle_upper_right/cmd"},
        DataWriter<custom_types::MotorCmd>{"ankle_lower_left/cmd"},
        DataWriter<custom_types::MotorCmd>{"ankle_lower_right/cmd"},
    };

    DataReader<custom_types::MotorState> dr_joint_stat_[4] = {
        DataReader<custom_types::MotorState>{"ankle_pitch_left/state"},
        DataReader<custom_types::MotorState>{"ankle_pitch_right/state"},
        DataReader<custom_types::MotorState>{"ankle_roll_left/state"},
        DataReader<custom_types::MotorState>{"ankle_roll_right/state"},
    };

    DataWriter<custom_types::MotorState> dw_motor_stat_[4] = {
        DataWriter<custom_types::MotorState>{"ankle_upper_left/state", DependencyType::Weak},
        DataWriter<custom_types::MotorState>{"ankle_upper_right/state", DependencyType::Weak},
        DataWriter<custom_types::MotorState>{"ankle_lower_left/state", DependencyType::Weak},
        DataWriter<custom_types::MotorState>{"ankle_lower_right/state", DependencyType::Weak},
    };
};


} // namespace task_pool
