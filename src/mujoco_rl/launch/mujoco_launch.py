from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 조이스틱 노드 실행 설정
    joystick_node = Node(
        package='mujoco_rl',
        executable='controller_ros_code.py',
        name='joystick_publisher_node',
        output='screen'
    )
    
    # 2. MuJoCo 환경 노드 실행 설정
    mujoco_env_node = Node(
        package='mujoco_rl',
        executable='mujoco_ros_code.py',
        name='mujoco_environment_node',
        output='screen'
    )
    
    # 3. AI 정책(PolicyCal) 노드 실행 설정
    policy_cal_node = Node(
        package='mujoco_rl',
        executable='policy_ros_code.py',
        name='policy_calculation_node',
        output='screen'
    )
    
    # 생성된 노드들을 LaunchDescription에 담아 반환
    return LaunchDescription([
        joystick_node,
        mujoco_env_node,
        policy_cal_node,
    ])