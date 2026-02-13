#!/usr/bin/env python3
import time
import json
from typing import Union
import os
import numpy as np 
from omegaconf import DictConfig, ListConfig, OmegaConf 
import argparse

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from msg_interfaces.msg import Obs, Action

os.environ['ROS_DOMAIN_ID'] = '46'


def parse_policy_config():
    package_share_dir = get_package_share_directory('mujoco_rl')
    # config_file_path = os.path.join(package_share_dir, 'config', 'robotnl_pre.yaml')
    config_file_path = os.path.join(package_share_dir, 'config', 'robotnl.yaml')
    print("Loading config file from ", config_file_path)
    with open(config_file_path, "r") as f:
        return OmegaConf.load(f)


class PolicyCal(Node):
    def __init__(self, cfg: Union[DictConfig, ListConfig]):
        super().__init__('policy_runner_node')
        self.cfg = cfg
        self.runner = None
        package_share_dir = get_package_share_directory('mujoco_rl')
        model_checkpoint_path = os.path.join(package_share_dir, self.cfg.policy_checkpoint_path)

        if ".pt" in model_checkpoint_path:
            import torch
            torch.set_printoptions(precision=2)
            self.runner = "torch"
            self.policy_pt: torch.nn.Module = torch.jit.load(model_checkpoint_path)
            self.get_logger().info("Using Torch runner")
        else:
            raise ValueError("Unrecognized policy format")
        
        self.default_joint_positions = np.array(self.cfg.default_joint_positions, dtype=np.float32)
        self.policy_observations = np.zeros((1, self.cfg.num_observations * (self.cfg.history_length + 1)), dtype=np.float32)
        self.actions_raw = np.zeros((1, self.cfg.num_actions), dtype=np.float32)
        self.pre_actions = np.zeros((self.cfg.num_actions,), dtype=np.float32)
        self.flag = 0

        self.action_publisher = self.create_publisher(Action, 'action', 10)
        self.observation_subscriber = self.create_subscription(
            Obs,
            'obs',
            self.observation_callback,
            10)
        self.get_logger().info('Policy Runner Node has been initialized.')

    def observation_callback(self, msg: Obs):
        observations = np.array(msg.obs, dtype=np.float32)
        self.flag = msg.flag

        self.policy_observations[:] = np.concatenate([
            self.policy_observations[0, self.cfg.num_observations:],
            observations,
            self.pre_actions
        ], axis=0)

        if self.runner == "torch":
            import torch
            self.policy_observations_tensor = torch.tensor(self.policy_observations)
            self.actions_raw[:] = self.policy_pt.forward(self.policy_observations_tensor).detach().numpy()

        actions_clipped = np.clip(self.actions_raw[0], self.cfg.action_limit_lower, self.cfg.action_limit_upper)
        self.pre_actions[:] = actions_clipped
        actions_scaled = actions_clipped * self.cfg.action_scale + self.default_joint_positions

        action_msg = Action()
        action_msg.action = actions_scaled.tolist()
        
        self.action_publisher.publish(action_msg)


def main(args=None):
    rclpy.init(args=args)
    cfg = parse_policy_config()
    policy_cal_node = PolicyCal(cfg)
    
    try:
        while rclpy.ok() and not policy_cal_node.flag:
            rclpy.spin_once(policy_cal_node, timeout_sec=0.1)
    except KeyboardInterrupt:
        print("Shutting down policy node by KeyboardInterrupt.")
    finally:
        policy_cal_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("Policy node has been shut down successfully.")


if __name__ == "__main__":
    main()