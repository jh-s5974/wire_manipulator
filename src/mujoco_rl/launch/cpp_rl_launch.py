from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    joystick_node = Node(
        package='mujoco_rl',
        executable='controller_ros_code.py',
        name='joystick_publisher_node',
        output='screen'
    )
    
    mujoco_env_node = Node(
        package='mujoco_rl',
        executable='mujoco_ros6',
        name='mujoco_ros6',
        output='screen'
    )
    
    policy_node = Node(
        package='mujoco_rl',
        executable='policy_runner2',
        name='policy_runner2',
        output='screen'
    )
    
    return LaunchDescription([
        joystick_node,
        mujoco_env_node,
        policy_node,
    ])