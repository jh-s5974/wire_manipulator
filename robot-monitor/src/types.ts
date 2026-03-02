// src/types.ts

export type MotorMode = "POSITION" | "VELOCITY" | "TORQUE" | "IDLE" | string;

export interface MotorState {
  id: number;
  name?: string;
  mode: MotorMode;
  position: number;
  velocity: number;
  torque: number;
  temperature: number;
  voltage?: number;
  error: boolean;
  warning: boolean;
  command_position?: number;
  command_velocity?: number;
  command_torque?: number;
  command_kp?: number;
  command_kd?: number;
  kp?: number;
  kd?: number;

  // 전원/enable 상태 (로봇에서 보내주면 사용, 안 보내면 UI에서 false로 취급)
  enabled?: boolean;
}

export interface ImuOrientationRpy {
  roll: number;
  pitch: number;
  yaw: number;
}

export interface Vector3 {
  x: number;
  y: number;
  z: number;
}

export interface ImuState {
  orientation_rpy: ImuOrientationRpy;
  angular_velocity: Vector3;
  linear_acceleration: Vector3;
}

export type RobotMode = "IDLE" | "MANU" | "READY" | "RL WALK";

export interface RobotModeState {
  current: RobotMode;
}

export interface SafetyState {
  level: "ESSENTIAL" | "STRICT" | string;
  locked: boolean;
}

export interface StatePayload {
  control?: {
    requested: boolean;
    granted: boolean;
  };
  robot_mode?: RobotModeState;
  safety?: SafetyState;
  motors: MotorState[];
  imu: ImuState;
}

export interface BaseMessage<T = unknown> {
  type: string;
  timestamp: number;
  payload: T;
}

export type StateMessage = BaseMessage<StatePayload>;