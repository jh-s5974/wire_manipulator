import { Canvas, useThree } from "@react-three/fiber";
import { OrbitControls } from "@react-three/drei";
import { Suspense, useEffect, useState } from "react";
import { UrdfRobot } from "./UrdfRobot";
import type { JointState, FrameMode } from "./UrdfRobot";
import type { ImuState } from "../types";

interface RobotSceneProps {
  jointState?: JointState;
  imuState?: ImuState | null;
}

function SceneDebug() {
  const { camera, scene } = useThree();
  useEffect(() => {
    console.log("[Scene] camera position:", camera.position);
    console.log("[Scene] scene children:", scene.children.map(c => `${c.type}(${c.name})`));
  // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);
  return null;
}

export function RobotScene({ jointState, imuState }: RobotSceneProps) {
  const [frameMode, setFrameMode] = useState<FrameMode>("global");

  return (
    <div style={{ width: "100%", height: "100%", position: "relative" }}>
      {/* Local / Global 토글 버튼 */}
      <div style={{
        position: "absolute", top: 6, right: 6, zIndex: 10,
        display: "flex", gap: 2,
        fontSize: 10, fontWeight: 600,
      }}>
        {(["global", "local"] as FrameMode[]).map((m) => (
          <button
            key={m}
            onClick={() => setFrameMode(m)}
            style={{
              padding: "2px 7px",
              borderRadius: 4,
              border: "none",
              cursor: "pointer",
              background: frameMode === m ? "#2563eb" : "#374151",
              color: frameMode === m ? "#fff" : "#9ca3af",
            }}
          >
            {m === "global" ? "Global" : "Local"}
          </button>
        ))}
      </div>

      <Canvas camera={{ position: [1.5, 1.5, 2.5], fov: 50 }}>
        <ambientLight intensity={0.5} />
        <directionalLight position={[3, 5, 3]} intensity={1.0} />

        {/* 기즈모: -PI/2 회전 → ROS 관례 색상 (X=빨강/앞, Y=초록/왼, Z=파랑/위) */}
        <group rotation={[-Math.PI / 2, 0, 0]}>
          <axesHelper args={[0.4]} />
        </group>

        {/* 원점 확인용 박스 */}
        <mesh>
          <boxGeometry args={[0.03, 0.03, 0.03]} />
          <meshStandardMaterial color="white" />
        </mesh>

        <Suspense fallback={null}>
          <UrdfRobot
            urdfPath="/robot/robot.urdf"
            jointState={jointState}
            imuState={imuState}
            frameMode={frameMode}
          />
        </Suspense>

        <SceneDebug />
        <OrbitControls />
      </Canvas>
    </div>
  );
}