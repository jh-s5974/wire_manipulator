#!/usr/bin/env python3
import mujoco
import mujoco.viewer
import os
# ament_index_python 라이브러리를 임포트
from ament_index_python.packages import get_package_share_directory

# 패키지의 share 디렉토리 경로를 찾음
package_share_dir = get_package_share_directory('mujoco_rl')

# .xml 파일의 전체 경로를 생성
xml_path = os.path.join(package_share_dir, 'mjcf', 'robot_nl.xml')
print(f"Loading model from: {xml_path}")

# XML 모델
model = mujoco.MjModel.from_xml_path(xml_path)
data = mujoco.MjData(model)

# 뷰어 실행
with mujoco.viewer.launch_passive(model, data) as viewer:
  while viewer.is_running():
    mujoco.mj_step(model, data)
    viewer.sync()