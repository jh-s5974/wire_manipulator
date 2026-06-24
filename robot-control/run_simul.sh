#!/usr/bin/env bash
# robot_simul 실행 런처.
# 시뮬은 CAN 하드웨어를 안 쓰므로 sudo 없이 실행한다(RT는 SOFT, mlockall 실패해도 계속).
# ROS2 환경을 source 해야 rclcpp/rmw 라이브러리를 찾는다(librcl_interfaces ... 로드 에러 방지).
set -e
cd "$(dirname "$0")"            # config/robotnl.yaml 상대경로를 위해 robot-control 으로 이동
source /opt/ros/humble/setup.bash
exec ./build/robot_simul "$@"
