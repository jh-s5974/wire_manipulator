#!/usr/bin/env python3
import mujoco
import mujoco.viewer
import os
import numpy as np
import torch
import threading
import time
import socket
import struct
from typing import Union
import argparse

import rclpy
from rclpy.node import Node
from msg_interfaces.msg import Joymsg, Obs, Action

from ament_index_python.packages import get_package_share_directory
from omegaconf import DictConfig, ListConfig, OmegaConf  # YAML 설정 파일을 쉽게 다루기 위한 라이브러리

os.environ['ROS_DOMAIN_ID'] = '46'

def quat_rotate_inverse(q: torch.Tensor, v: torch.Tensor) -> torch.Tensor:
    q_w = q[0]
    q_vec = q[1:4]
    a = v * (2.0 * q_w ** 2 - 1.0)
    b = torch.cross(q_vec, v, dim=-1) * q_w * 2.0
    c = q_vec * (torch.dot(q_vec, v)) * 2.0
    return a - b + c

class Cfg:
    package_share_dir = get_package_share_directory('mujoco_rl')
    config_file_path = os.path.join(package_share_dir, 'config', 'robotnl.yaml')
    print("Loading config file from ", config_file_path)
    with open(config_file_path, "r") as f:
        cfg_data = OmegaConf.load(f)

    policy_dt = cfg_data.policy_dt
    physics_dt = cfg_data.physics_dt
    cutoff_freq = cfg_data.cutoff_freq

    num_joints = cfg_data.num_joints
    joint_kp = cfg_data.joint_kp
    joint_kd = cfg_data.joint_kd
    default_joint_positions = cfg_data.default_joint_positions
    num_actions = cfg_data.num_actions
    action_indices = cfg_data.action_indices
    effort_limits = cfg_data.effort_limits
    action_scale = cfg_data.action_scale
    action_limit_lower = cfg_data.action_limit_lower
    action_limit_upper = cfg_data.action_limit_upper
    num_observations = cfg_data.num_observations
    dsp = 0.15
    ssp = 0.3

class MujocoEnv(Node):
    def __init__(self, cfg: Cfg):
        super().__init__("mujoco_node")

        self.cfg = cfg
        self.physics_substeps = int(np.round(self.cfg.policy_dt / self.cfg.physics_dt))

        self.phase = 2*(self.cfg.dsp + self.cfg.ssp)
        self.total_time = 0
        self.mode = 0

        self.xml_path = os.path.join(self.cfg.package_share_dir, 'mjcf', 'robot_nl.xml')
        print(f"Loading model from: {self.xml_path}")
        self.model = mujoco.MjModel.from_xml_path(self.xml_path)
        self.data = mujoco.MjData(self.model)
        self.model.opt.timestep = self.cfg.physics_dt

        self.sensordata_dof_size = 3 * self.model.nu  # nu = num_actuator
        self.gravity_vector = torch.tensor([0.0, 0.0, -1.0])

        self.joint_kp = torch.zeros((self.cfg.num_joints,), dtype=torch.float32)
        self.joint_kd = torch.zeros((self.cfg.num_joints,), dtype=torch.float32)
        self.effort_limits = torch.zeros((self.cfg.num_joints,), dtype=torch.float32)
        self.joint_kp[:] = torch.tensor(self.cfg.joint_kp)
        self.joint_kd[:] = torch.tensor(self.cfg.joint_kd)
        self.effort_limits[:] = torch.tensor(self.cfg.effort_limits)

        self.commands = torch.zeros((3,), dtype=torch.float32)
        self.latest_actions = None
        self.action_lock = threading.Lock()

        self.joy_sub = self.create_subscription(
            Joymsg,
            'joystick_state',
            self.joystick_callback,
            10)
        
        self.obs_pub = self.create_publisher(
            Obs, 
            'obs', 
            10)

        self.joint_ref_sub = self.create_subscription(
            Action,
            'action',
            self.action_callback,
            10)

    def joystick_callback(self, msg: Joymsg):
        if len(msg.buttons) > 1:
            if(msg.buttons[0] == 1):
                self.mode = 1
            elif(msg.buttons[1] == 1):
                self.mode = 0

        if self.mode == 1:
            if len(msg.axes) > 1:
                if(abs(msg.axes[1]) < 0.1):
                    self.commands[0]= 0.0
                else:
                    self.commands[0]= round(-msg.axes[1] * 1.0, 2)
            if len(msg.axes) > 0:
                if(abs(msg.axes[0]) < 0.1):
                    self.commands[1]= 0.0
                else:
                    self.commands[1] = round(msg.axes[0] * 0.5, 2)
            if len(msg.axes) > 2:
                if(abs(msg.axes[2]) < 0.1):
                    self.commands[2]= 0.0
                else:
                    self.commands[2] = round(msg.axes[2] * 0.5, 2)
        else:
            self.commands = torch.zeros((3,), dtype=torch.float32)

    def action_callback(self, msg: Action):
         with self.action_lock:
            if self.mode == 1:
                self.latest_actions = torch.tensor(msg.action, dtype=torch.float32)
            else:
                self.latest_actions = None

    def reset(self):
        self.data.qpos[0:3] = [0.0, 0.0, 0.77]
        self.data.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
        self.data.qpos[7:] = self.cfg.default_joint_positions
        self.data.qvel[:] = 0
        mujoco.mj_forward(self.model, self.data)

        obs = self._get_observations()
        return obs

    def step(self, actions: torch.Tensor):
        for _ in range(self.physics_substeps):
            self._apply_actions(actions)
            mujoco.mj_step(self.model, self.data)

        obs = self._get_observations()
        return obs
    

    def _get_clock(self) -> torch.Tensor:
        self.total_time = self.data.time
        self.local_phi = self.total_time % self.phase
        self.phi = self.local_phi / self.phase
        self.clock_sin = np.sin((2 * np.pi * self.phi))
        self.clock_cos = np.cos((2 * np.pi * self.phi))
        return torch.tensor([self.clock_sin, self.clock_cos], dtype=torch.float32)

    def _get_base_pos(self) -> torch.Tensor:
            return torch.tensor(self.data.qpos[:3], dtype=torch.float32)
        
    def _get_base_vel(self) -> torch.Tensor:
        return torch.tensor(self.data.qvel[:3], dtype=torch.float32)

    def _get_base_quat(self) -> torch.Tensor:
        return torch.tensor(self.data.sensordata[self.sensordata_dof_size+0:self.sensordata_dof_size+4],
                            dtype=torch.float32)

    def _get_base_ang_vel(self) -> torch.Tensor:
        return torch.tensor(self.data.sensordata[self.sensordata_dof_size+4:self.sensordata_dof_size+7],
                            dtype=torch.float32)

    def _get_projected_gravity(self) -> torch.Tensor:
        base_quat = self._get_base_quat()
        projected_gravity = quat_rotate_inverse(base_quat, self.gravity_vector)
        return projected_gravity

    def _get_joint_pos(self) -> torch.Tensor:
        return torch.tensor(self.data.sensordata[0:self.cfg.num_joints], dtype=torch.float32)

    def _get_joint_vel(self) -> torch.Tensor:
        return torch.tensor(self.data.sensordata[self.cfg.num_joints:2*self.cfg.num_joints],
                            dtype=torch.float32)
    
    def _get_commands(self) -> torch.Tensor:
        return self.commands

    def _get_observations(self) -> torch.Tensor:
        return torch.cat([
            self._get_clock(),
            # self._get_base_vel(),
            self._get_base_ang_vel(),
            self._get_projected_gravity(),
            self._get_commands(),
            self._get_joint_pos()[self.cfg.action_indices],
            self._get_joint_vel()[self.cfg.action_indices],
        ], dim=-1)
    
    def _apply_actions(self, actions: torch.Tensor):
        target_positions = torch.zeros((self.cfg.num_joints,))
        target_positions[self.cfg.action_indices] = actions

        output_torques = self.joint_kp * (target_positions - self._get_joint_pos()) + \
                        self.joint_kd * (-self._get_joint_vel())
        
        output_torques_clipped = torch.clip(output_torques, -self.effort_limits, self.effort_limits)
        # print(output_torques_clipped)
        self.data.ctrl[:] = output_torques_clipped.numpy()



def main(args=None):
    rclpy.init(args=args)
    domain_id = os.getenv('ROS_DOMAIN_ID', '기본값 0')
    print(f"--- 현재 ROS_DOMAIN_ID: {domain_id} ---")
    env = MujocoEnv(Cfg())
    obs = env.reset()
    default_actions = torch.tensor(env.cfg.default_joint_positions, dtype=torch.float32)[env.cfg.action_indices]

    spin_thread = threading.Thread(target=rclpy.spin, args=(env,), daemon=True)
    spin_thread.start()

    try:
        with mujoco.viewer.launch_passive(env.model, env.data) as viewer:
            while viewer.is_running():
                step_start = time.time()
                with env.action_lock:
                    if env.latest_actions is not None:
                        actions = env.latest_actions
                    else:
                        actions = default_actions

                obs = env.step(actions)
                obs_msg = Obs()
                obs_msg.obs = obs.tolist()
                env.obs_pub.publish(obs_msg)

                viewer.sync()
                time_until_next_step = env.cfg.policy_dt - (time.time() - step_start)
                if time_until_next_step > 0:
                    time.sleep(time_until_next_step)
                
    except KeyboardInterrupt:
        env.get_logger().info("프로그램을 종료합니다.")
    finally:
        env.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
  main()