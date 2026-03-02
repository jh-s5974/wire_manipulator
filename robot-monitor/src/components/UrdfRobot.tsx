import { useEffect, useMemo, useState } from "react";
import * as THREE from "three";
import URDFLoader from "urdf-loader";
import type { URDFRobot, URDFJoint } from "urdf-loader";
import type { ImuState } from "../types";

export interface JointState {
  [jointName: string]: number; // rad
}

export type FrameMode = "global" | "local";

interface UrdfRobotProps {
  urdfPath: string;
  jointState?: JointState;
  imuState?: ImuState | null;
  frameMode?: FrameMode;
}

const ROBOT_MATERIAL = new THREE.MeshStandardMaterial({
  color: 0xaaaaaa,
  roughness: 0.5,
  metalness: 0.3,
});

export function UrdfRobot({ urdfPath, jointState = {}, imuState, frameMode = "global" }: UrdfRobotProps) {
  const [robot, setRobot] = useState<URDFRobot | null>(null);

  useEffect(() => {
    const loader = new URDFLoader();
    loader.workingPath = urdfPath.substring(0, urdfPath.lastIndexOf("/") + 1);
    loader.load(
      urdfPath,
      (loadedRobot) => {
        loadedRobot.traverse((child) => {
          if ((child as THREE.Mesh).isMesh) {
            (child as THREE.Mesh).material = ROBOT_MATERIAL;
          }
        });
        console.log("[URDF] 로딩 성공, 머티리얼 오버라이드 완료");
        setRobot(loadedRobot);
      },
      undefined,
      (err) => console.error("[URDF] 로딩 실패:", err),
    );
  }, [urdfPath]);

  useEffect(() => {
    if (!robot) return;
    Object.entries(jointState).forEach(([name, value]) => {
      const joint = robot.joints[name] as URDFJoint | undefined;
      if (joint) joint.setJointValue(value);
    });
  }, [jointState, robot]);

  // Z-up(ROS) → Y-up(THREE) 기본 변환 + Global 모드에서 IMU 회전 합성
  const rotation = useMemo<[number, number, number]>(() => {
    const q_convert = new THREE.Quaternion().setFromAxisAngle(
      new THREE.Vector3(0, 0, 1), -Math.PI / 2
    );
    if (frameMode === "global" && imuState) {
      const { roll, pitch, yaw } = imuState.orientation_rpy;
      // ROS IMU(Z-up)를 THREE(Y-up) 세계좌표로 변환:
      // 먼저 Z-up→Y-up 변환(q_convert)한 뒤, 세계좌표에서 IMU 회전 적용
      const q_imu = new THREE.Quaternion().setFromEuler(
        new THREE.Euler(roll, pitch, yaw, "XYZ")
      );
      const q_final = q_imu.clone().multiply(q_convert);
      const e = new THREE.Euler().setFromQuaternion(q_final);
      return [e.x, e.y, e.z];
    }
    return [-Math.PI / 2, 0, 0];
  }, [frameMode, imuState]);

  if (!robot) return null;

  return <primitive object={robot} rotation={rotation} />;
}