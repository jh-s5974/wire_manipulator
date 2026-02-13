import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, Command

def generate_launch_description():
    pkg_share = FindPackageShare(package='rlcode').find('rlcode')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/robot_ik.rviz')
    default_urdf_model_path = os.path.join(pkg_share, 'urdf/robot_alined_world.urdf')
    robot_description = Command(['xacro ', default_urdf_model_path])

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        namespace="rl",
        name='rviz_node',
        output='screen',
        arguments=['-d', default_rviz_config_path],
    )

    obstorviz_node = Node(
        package='rlcode',
        executable='obstorviz.py',
        namespace="rl",
        name='obstorviz_node',
        output='screen'
    )


    return LaunchDescription([
        robot_state_publisher_node,
        rviz_node,
        obstorviz_node,
    ])