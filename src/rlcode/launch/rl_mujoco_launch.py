import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, Command

def generate_launch_description():
    pkg_share = FindPackageShare(package='rlcode').find('rlcode')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/robot_ik.rviz')
    default_urdf_model_path = os.path.join(pkg_share, 'urdf/robot_alined.urdf')
    robot_description = Command(['xacro ', default_urdf_model_path])

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_node',
        output='screen',
        arguments=['-d', default_rviz_config_path], # 미리 정의된 Rviz 설정 파일 사용
    )

    obstorviz_node = Node(
        package='rlcode',
        executable='obstorviz.py',
        namespace="rl",
        name='obstorviz_node',
        output='screen'
    )

    joystick_node = Node(
        package='rlcode',
        executable='controller.py',
        namespace="rl",
        name='joystick_node',
        output='screen'
    )
    
    # mujoco_env_node = Node(
    #     package='rlcode',
    #     executable='mujoco_sim',
    #     namespace="rl",
    #     name='mujoco_sim',
    #     output='screen'
    # )
    
    policy_node = Node(
        package='rlcode',
        executable='policy_code2',
        namespace="rl",
        name='policy_node',
        output='screen'
    )
    
    control_node = Node(
        package='rlcode',
        executable='motor_test5',
        namespace="rl",
        name='robot_control_node',
        output='screen'
    )

    imu_node = Node(
        package='rlcode',
        executable='imu_test4',
        namespace="rl",
        name='imu_node',
        output='screen'
    )

    return LaunchDescription([
        # robot_state_publisher_node,
        # rviz_node,
        # obstorviz_node,
        joystick_node,
        # mujoco_env_node,
        policy_node,
        control_node,
        imu_node,
    ])


# sudo setcap cap_net_raw,cap_sys_nice+eip $(find ~/mujoco_ws/install -type f -name motor_test4)
# sudo setcap cap_net_raw,cap_sys_nice+eip $(find ~/mujoco_ws/install -type f -name imu_test4)
# sudo setcap cap_net_raw,cap_sys_nice+eip $(find ~/mujoco_ws/install -type f -name policy_code2)
# sudo setcap cap_net_raw,cap_sys_nice+eip $(find ~/mujoco_ws/install -type f -name controller.py)
