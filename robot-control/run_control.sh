#!/usr/bin/env bash
# robot_control 실행 런처 (실하드웨어: CAN + RT 스케줄링 → sudo 필요).
# sudo 는 LD_LIBRARY_PATH 를 지우므로 바깥에서 source 해도 소용없다.
# → root 셸 "안에서" ROS2 를 source 한 뒤 실행해야 rmw/typesupport 라이브러리를 찾는다.
set -e
cd "$(dirname "$0")"            # config/robotnl.yaml 상대경로를 위해 robot-control 으로 이동

# FastDDS UDP 전용 프로파일 — root(sudo) robot_control 이 일반 사용자 publisher 의
# /joint_command 를 받으려면 SHM 대신 UDP 전송이 필요하다(SHM 은 UID 격리됨).
# 절대경로로 만들어 root 셸 "안에서" export 한다(sudo 가 환경을 지우므로).
DDS_PROFILE="$PWD/config/fastdds_udp_only.xml"

# ROS 도메인/네트워크 환경을 root 로 그대로 전달한다.
# sudo 는 환경을 지우고 root 셸은 사용자 ~/.bashrc 를 안 읽으므로, 명시적으로
# 넘기지 않으면 robot_control 은 ROS_DOMAIN_ID=0 으로 떨어진다. 사용자 publisher 가
# ~/.bashrc 의 ROS_DOMAIN_ID(예: 10)를 쓰면 도메인이 어긋나 discovery 자체가 안 되어
# 메시지가 한 개도 안 들어온다(rx=0). 여기서 launch 터미널의 값을 그대로 물려준다.
FWD_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
FWD_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"

exec sudo bash -c '
  export FASTRTPS_DEFAULT_PROFILES_FILE="'"$DDS_PROFILE"'"
  export ROS_DOMAIN_ID="'"$FWD_DOMAIN_ID"'"
  export ROS_LOCALHOST_ONLY="'"$FWD_LOCALHOST_ONLY"'"
  source /opt/ros/humble/setup.bash
  exec ./build/robot_control "$@"
' _ "$@"
