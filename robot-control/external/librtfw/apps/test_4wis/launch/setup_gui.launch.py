from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    os.environ['OPENCV_LOG_LEVEL'] = 'OFF'
    return LaunchDescription([
        Node(
            package='mobile_crane',
            executable='joystick',
            remappings=[
                ("state", "js_state"),
                ("cmd_state", "cmd_js_state"),
                ("axis1", "cmd_vx"),
                ("axis0", "cmd_vy"),
                ("axis3", "cmd_wz"),
                ("axis5", "cmd_vc_up"),
                ("axis2", "cmd_vc_down"),
                ("sw0", "cmd_tracking_on"),
                ("sw1", "cmd_tracking_off"),
                ("sw2", "cmd_origin_reset"),
                ("sw3", "cmd_soft_emg"),
                ("sw4", "cmd_crane_teach"),
                ("sw5", "cmd_vxy_reverse"),
            ],
            # output={
            #     'stdout': 'screen',
            #     'stderr': 'screen',
            # }
        ),
        Node(
            package='mobile_crane',
            executable='wheel_if',
            remappings=[
                ("state", "wm_state"),
                ("cmd_state", "cmd_wm_state"),
                # ('/rpm_sv', '/wheel_sv'),
                # ('/rpm_pv', '/wheel_pv'),
            ],
            # output={
            #     'stdout': 'screen',
            #     'stderr': 'screen',
            # }
        ),
        Node(
            package='mobile_crane',
            executable='crane_if',
            remappings=[
                ("state", "cm_state"),
                ("cmd_state", "cmd_cm_state"),
            ],
        ),
        Node(
            package='mobile_crane',
            executable='mecanum',
            remappings=[
                ("state", "mk_state"),
                ("cmd_state", "cmd_mk_state"),
                ('/wheel_sv', '/rpm_sv'),
                ('/wheel_pv', '/rpm_pv'),
            ],
        ),
        # Node(
        #     package='mobile_crane',
        #     executable='imu',
        #     remappings=[
        #         ('/imu', '/crane_orient'),
        #     ]
        # ),
        Node(
            package='mobile_crane',
            executable='vision',
            parameters=[
                {'gui': True},
            ],
            remappings=[
                ("state", "cv_state"),
                ("cmd_state", "cmd_cv_state"),
            ],
            output={
                'stdout': 'screen',
                'stderr': 'screen',
            }
        ),
        Node(
            package='mobile_crane',
            executable='daq',
            remappings=[
                ('/position', '/height_pv'),
                ("state", "dq_state"),
                ("cmd_state", "cmd_dq_state"),
            ],
            # output={
            #     'stdout': 'screen',
            #     'stderr': 'screen',
            # }
        ),
        Node(
            package='mobile_crane',
            executable='crane_pos_control',
            remappings=[
                ("state", "cc_state"),
                ("cmd_state", "cmd_cc_state"),
                ("spd_sv", "crane_spd_sv"),
                ("pos_sv", "height_sv"),
                ("pos_pv", "height_pv"),
            ],
        ),
        Node(
            package='mobile_crane',
            executable='tracking_control',
            remappings=[
                ("state", "tc_state"),
                ("cmd_state", "cmd_tc_state"),
            ],
        ),
        Node(
            package='mobile_crane',
            executable='abnormal_detector',
            remappings=[
                ("state", "ad_state"),
                ("cmd_state", "cmd_ad_state"),
            ],
        ),
        Node(
            package='mobile_crane',
            executable='crane',
            remappings=[
                ('/pose', '/robot_pose'),
                # ('/position', '/position_pv'),
            ],
            output={
                'stdout': 'screen',
                'stderr': 'screen',
            },
        ),
    ])