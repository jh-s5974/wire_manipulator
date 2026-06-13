// src/App.tsx
import * as React from "react";
import "./App.css";
import { useRobotWs } from "./hooks/useRobotWs";
import type { MotorState, RobotMode } from "./types";

const WS_URL =
  import.meta.env.VITE_WS_URL ??
  `${window.location.protocol === "https:" ? "wss" : "ws"}://${window.location.hostname}:8080`;
const getInitialWsPassword = () => {
  return "";
};
type MotorCommandFields = {
  position: string;
  velocity: string;
  torque: string;
  kp: string;
  kd: string;
  durationMs: string;
};

const EMPTY_MOTOR_COMMAND: MotorCommandFields = {
  position: "",
  velocity: "",
  torque: "",
  kp: "",
  kd: "",
  durationMs: "1.000",
};

const MOTOR_COMMAND_DIGITS: Record<keyof MotorCommandFields, number> = {
  position: 2,
  velocity: 2,
  torque: 2,
  kp: 1,
  kd: 1,
  durationMs: 1,
};

function formatCommandValue(
  value: number | undefined,
  field: keyof MotorCommandFields,
  fallback = "",
) {
  if (typeof value !== "number" || !Number.isFinite(value)) {
    return fallback;
  }
  return value.toFixed(MOTOR_COMMAND_DIGITS[field]);
}

type JointLimits = {
  lower: number;       // position lower (rad or m)
  upper: number;       // position upper (rad or m)
  effortMax: number;   // peak torque/force (Nm or N)
  velMax: number;      // peak velocity (rad/s or m/s)
  kpMax: number;       // kp upper bound
  kdMax: number;       // kd upper bound
  durationMax: number; // duration bar upper bound (s)
  unit: string;        // "rad" | "m"
  type: string;        // "revolute" | "prismatic"
};

// config/robotnl.yaml 및 safety_layer 파라미터와 일치하게 유지
const JOINT_LIMITS: Record<string, JointLimits> = {
  //          pos lower     pos upper   effort  vel   kp   kd  dur  unit       type
  joint0: { lower: -2.094395, upper:  2.094395, effortMax: 100, velMax: 30, kpMax: 600, kdMax: 5, durationMax: 10, unit: "rad", type: "revolute"  },
  joint1: { lower: -0.523599, upper:  1.570796, effortMax: 100, velMax: 30, kpMax: 600, kdMax: 5, durationMax: 10, unit: "rad", type: "revolute"  },
  joint2: { lower: -0.09,     upper:  0.0,      effortMax: 500, velMax: 30, kpMax: 600, kdMax: 5, durationMax: 10, unit: "m",   type: "prismatic" },
  joint3: { lower: -1.570796, upper:  1.570796, effortMax: 100, velMax: 30, kpMax: 600, kdMax: 5, durationMax: 10, unit: "rad", type: "revolute"  },
  joint4: { lower: -0.095,    upper:  0.0,      effortMax: 500, velMax: 30, kpMax: 600, kdMax: 5, durationMax: 10, unit: "m",   type: "prismatic" },
};

const MOTOR_NAMES: string[] = ["joint0", "joint1", "joint2", "joint3", "joint4"];

const NON_CONTROLLABLE_MOTOR_NAMES = new Set<string>();

function isMotorControlEnabled(name?: string) {
  if (!name) return true;
  return !NON_CONTROLLABLE_MOTOR_NAMES.has(name);
}

const PLACEHOLDER_MOTORS: MotorState[] = MOTOR_NAMES.map((name, id) => ({
  id,
  name,
  mode: "N/A",
  position: 0,
  velocity: 0,
  torque: 0,
  temperature: 0,
  error: false,
  warning: false,
  enabled: false,
}));

const DEFAULT_SAMPLING_MS = 100;
const DEFAULT_BUFFER_LENGTH = 240;
const DEFAULT_PLOT_COLUMNS = 2;
const DEFAULT_PLOT_HEIGHT = 240;
const MIN_SAMPLING_MS = 20;
const MAX_SAMPLING_MS = 2000;
const MIN_BUFFER_LENGTH = 30;
const MAX_BUFFER_LENGTH = 2000;
const MIN_PLOT_COLUMNS = 1;
const MAX_PLOT_COLUMNS = 4;
const MIN_PLOT_HEIGHT = 160;
const MAX_PLOT_HEIGHT = 520;

type PlotSource = "motor_stat" | "motor_cmd" | "imu";
type MotorStatPlotKey = "position" | "velocity" | "torque" | "temperature";
type MotorCmdPlotKey = "position" | "velocity" | "torque" | "kp" | "kd";
type ImuPlotKey =
  | "roll"
  | "pitch"
  | "yaw"
  | "gyro_x"
  | "gyro_y"
  | "gyro_z"
  | "accel_x"
  | "accel_y"
  | "accel_z";

type SeriesDef<K extends string> = {
  key: K;
  label: string;
  color: string;
};

const MOTOR_STAT_SERIES_DEFS: SeriesDef<MotorStatPlotKey>[] = [
  { key: "position", label: "Pos", color: "#2563eb" },
  { key: "velocity", label: "Vel", color: "#16a34a" },
  { key: "torque", label: "Torque", color: "#dc2626" },
  { key: "temperature", label: "Temp", color: "#d97706" },
];

const MOTOR_CMD_SERIES_DEFS: SeriesDef<MotorCmdPlotKey>[] = [
  { key: "position", label: "Cmd Pos", color: "#2563eb" },
  { key: "velocity", label: "Cmd Vel", color: "#16a34a" },
  { key: "torque", label: "Cmd Torque", color: "#dc2626" },
  { key: "kp", label: "Cmd Kp", color: "#7c3aed" },
  { key: "kd", label: "Cmd Kd", color: "#d97706" },
];

const IMU_SERIES_DEFS: SeriesDef<ImuPlotKey>[] = [
  { key: "roll", label: "Roll", color: "#2563eb" },
  { key: "pitch", label: "Pitch", color: "#16a34a" },
  { key: "yaw", label: "Yaw", color: "#dc2626" },
  { key: "gyro_x", label: "Gyro X", color: "#0ea5e9" },
  { key: "gyro_y", label: "Gyro Y", color: "#7c3aed" },
  { key: "gyro_z", label: "Gyro Z", color: "#f59e0b" },
  { key: "accel_x", label: "Accel X", color: "#ef4444" },
  { key: "accel_y", label: "Accel Y", color: "#22c55e" },
  { key: "accel_z", label: "Accel Z", color: "#6366f1" },
];

type PlotSeries = {
  key: string;
  label: string;
  color: string;
  values: number[];
};

type PlotPanelConfig = {
  id: number;
  source: PlotSource;
  motorId: number;
  motorStatFields: MotorStatPlotKey[];
  motorCmdFields: MotorCmdPlotKey[];
  imuFields: ImuPlotKey[];
  scaleMode: "auto" | "fixed";
  fixedMin: string;
  fixedMax: string;
  settingsCollapsed: boolean;
};

function getPlotSourceLabel(source: PlotSource) {
  if (source === "motor_stat") return "모터 상태";
  if (source === "motor_cmd") return "모터 명령";
  return "IMU";
}

function getPlotPanelTitle(panel: PlotPanelConfig) {
  const sourceLabel = getPlotSourceLabel(panel.source);
  if (panel.source === "imu") {
    return sourceLabel;
  }
  return `${sourceLabel} · Motor ${panel.motorId}`;
}

function clampInt(value: number, min: number, max: number) {
  return Math.min(Math.max(Math.round(value), min), max);
}

const TimeSeriesPlot = React.memo(function TimeSeriesPlot({
  series,
  scaleMode,
  fixedMin,
  fixedMax,
  plotHeight,
}: {
  series: PlotSeries[];
  scaleMode: "auto" | "fixed";
  fixedMin?: number;
  fixedMax?: number;
  plotHeight: number;
}) {
  const width = 1020;
  const height = clampInt(plotHeight, MIN_PLOT_HEIGHT, MAX_PLOT_HEIGHT);
  const padLeft = 52;
  const padRight = 14;
  const padTop = 14;
  const padBottom = 24;

  const maxLen = Math.max(...series.map((s) => s.values.length), 0);
  const allValues = series.flatMap((s) => s.values).filter((v) => Number.isFinite(v));

  if (maxLen === 0 || allValues.length === 0) {
    return <div className="plot-empty">플롯 데이터가 아직 없습니다.</div>;
  }

  let min = Math.min(...allValues);
  let max = Math.max(...allValues);
  if (
    scaleMode === "fixed" &&
    fixedMin !== undefined &&
    fixedMax !== undefined &&
    Number.isFinite(fixedMin) &&
    Number.isFinite(fixedMax) &&
    fixedMax > fixedMin
  ) {
    min = fixedMin;
    max = fixedMax;
  }

  if (Math.abs(max - min) < 1e-9) {
    min -= 1;
    max += 1;
  }

  const innerW = width - padLeft - padRight;
  const innerH = height - padTop - padBottom;

  const mapX = (index: number, len: number) =>
    padLeft + (len <= 1 ? 0 : (index / (len - 1)) * innerW);
  const mapY = (value: number) => padTop + ((max - value) / (max - min)) * innerH;

  const guideLevels = 4;
  const guideValues = Array.from({ length: guideLevels + 1 }, (_, idx) => {
    const ratio = idx / guideLevels;
    return max - ratio * (max - min);
  });

  return (
    <div className="plot-chart-wrap">
      <svg
        viewBox={`0 0 ${width} ${height}`}
        className="plot-svg"
        role="img"
        aria-label="Timeseries plot"
        style={{ height: `${height}px` }}
      >
        {guideValues.map((value) => {
          const y = mapY(value);
          return (
            <g key={`grid-${value.toFixed(6)}`}>
              <line x1={padLeft} y1={y} x2={width - padRight} y2={y} className="plot-grid" />
              <text x={padLeft - 8} y={y + 4} textAnchor="end" className="plot-axis-label">
                {value.toFixed(3)}
              </text>
            </g>
          );
        })}

        {series.map((s) => {
          if (s.values.length === 0) return null;
          const points = s.values
            .map((value, index) => `${mapX(index, s.values.length)},${mapY(value)}`)
            .join(" ");
          return (
            <polyline
              key={s.key}
              fill="none"
              stroke={s.color}
              strokeWidth={2}
              strokeLinecap="round"
              strokeLinejoin="round"
              points={points}
            />
          );
        })}
      </svg>

      <div className="plot-legend">
        {series.map((s) => {
          const latest = s.values[s.values.length - 1];
          return (
            <div className="plot-legend-item" key={`legend-${s.key}`}>
              <span className="plot-legend-dot" style={{ backgroundColor: s.color }} />
              <span>{s.label}</span>
              <strong>{Number.isFinite(latest) ? latest.toFixed(3) : "N/A"}</strong>
            </div>
          );
        })}
      </div>
    </div>
  );
});

function makeInitialCommand(m: MotorState): MotorCommandFields {
  return {
    position: formatCommandValue(m.command_position ?? m.position, "position", "0.00"),
    velocity: formatCommandValue(m.command_velocity ?? m.velocity, "velocity", "0.00"),
    torque: formatCommandValue(m.command_torque ?? m.torque, "torque", "0.00"),
    kp:
      m.command_kp !== undefined
        ? formatCommandValue(m.command_kp, "kp", "0.0")
        : m.kp !== undefined
          ? formatCommandValue(m.kp, "kp", "0.0")
          : "0.0",
    kd:
      m.command_kd !== undefined
        ? formatCommandValue(m.command_kd, "kd", "0.0")
        : m.kd !== undefined
          ? formatCommandValue(m.kd, "kd", "0.0")
          : "0.0",
    durationMs: "1.0",
  };
}

function JointLimitsTable() {
  return (
    <div className="card card-limits">
      <div className="card-header">
        <h3>Joint Limits</h3>
        <span className="limits-note">config/robotnl.yaml 기준</span>
      </div>
      <div className="table-wrapper">
        <table className="data-table">
          <thead>
            <tr>
              <th>Joint</th>
              <th>Type</th>
              <th>Unit</th>
              <th>Pos Min</th>
              <th>Pos Max</th>
              <th>Vel Max</th>
              <th>Effort Max</th>
              <th>Kp Max</th>
              <th>Kd Max</th>
            </tr>
          </thead>
          <tbody>
            {Object.entries(JOINT_LIMITS).map(([name, lim]) => (
              <tr key={name}>
                <td><strong>{name}</strong></td>
                <td>{lim.type}</td>
                <td>{lim.unit}</td>
                <td className="num">{lim.lower.toFixed(4)}</td>
                <td className="num">{lim.upper.toFixed(4)}</td>
                <td className="num">{lim.velMax}</td>
                <td className="num">{lim.effortMax}</td>
                <td className="num">{lim.kpMax}</td>
                <td className="num">{lim.kdMax}</td>
              </tr>
            ))}
          </tbody>
        </table>
      </div>
    </div>
  );
}

const barPcts = (value: number, min: number, max: number) => {
  const clamp = (v: number) => Math.max(0, Math.min(100, v));
  const zeroPct  = clamp(((0 - min) / (max - min)) * 100);
  const valPct   = clamp(((value - min) / (max - min)) * 100);
  return { zeroPct, fillLeft: Math.min(zeroPct, valPct), fillWidth: Math.abs(valPct - zeroPct) };
};

const StatBar = ({ value, min, max, color }: { value: number; min: number; max: number; color: string }) => {
  const { zeroPct, fillLeft, fillWidth } = barPcts(value, min, max);
  return (
    <div className="stat-bar-bg">
      <div className="stat-bar-fill" style={{ left: `${fillLeft}%`, width: `${fillWidth}%`, background: color, pointerEvents: "none" }} />
      <div className="stat-bar-zero" style={{ left: `${zeroPct}%`, pointerEvents: "none" }} />
    </div>
  );
};

interface MotorStatusRowProps {
  motor: MotorState;
  hasData: boolean;
}

const MotorStatusRow = React.memo(function MotorStatusRow({ motor, hasData }: MotorStatusRowProps) {
  const limits = motor.name ? JOINT_LIMITS[motor.name] : undefined;
  const renderNum = (v: number | undefined, d = 3) =>
    hasData && typeof v === "number" && Number.isFinite(v) ? v.toFixed(d) : "N/A";
  const currentKp = motor.driver_command_kp;
  const currentKd = motor.driver_command_kd;
  const renderCommandPair = (
    currentValue: number | undefined,
    commandValue: number | undefined,
    barRange?: { min: number; max: number; color: string },
    digits = 2,
  ) => (
    <td className="num stat-cell">
      <div className="status-pair-inline">
        <span className="status-pair-cmd">{renderNum(commandValue, digits)}</span>
        <span className="status-pair-now">{renderNum(currentValue, digits)}</span>
      </div>
      {limits && hasData && barRange && typeof currentValue === "number" && (
        <StatBar value={currentValue} min={barRange.min} max={barRange.max} color={barRange.color} />
      )}
    </td>
  );
  return (
    <tr className={hasData && motor.error ? "row-error" : hasData && motor.warning ? "row-warning" : ""}>
      <td className="cell-id">{motor.id}</td>
      <td className="cell-name">{motor.name ?? "-"}</td>
      <td className="cell-mode">
        <span
          className={
            "motor-status-dot" +
            (!hasData || motor.enabled === false
              ? " dot-inactive"
              : motor.error
              ? " dot-error"
              : motor.warning
              ? " dot-warning"
              : " dot-ok")
          }
          title={hasData ? motor.mode : "N/A"}
        />
      </td>
      {renderCommandPair(motor.position, motor.driver_command_position, limits ? {
        min: limits.lower,
        max: limits.upper,
        color: "#3b82f6",
      } : undefined)}
      {renderCommandPair(motor.velocity, motor.driver_command_velocity, limits ? {
        min: -limits.velMax,
        max: limits.velMax,
        color: "#8b5cf6",
      } : undefined)}
      {renderCommandPair(motor.torque, motor.driver_command_torque, limits ? {
        min: -limits.effortMax,
        max: limits.effortMax,
        color: "#f59e0b",
      } : undefined, 2)}
      <td className="num">{renderNum(currentKp, 1)}</td>
      <td className="num">{renderNum(currentKd, 1)}</td>
      <td className="num">{hasData ? `${motor.temperature.toFixed(1)}°C` : "N/A"}</td>
    </tr>
  );
});

interface MotorCommandRowProps {
  motor: MotorState;
  hasData: boolean;
  canControl: boolean;
  cmd: MotorCommandFields;
  onChange: (motorId: number, next: MotorCommandFields) => void;
  onSync: (motorId: number) => void;
  onSend: (motorId: number) => void;
  powered: boolean;
  onTogglePower: (motorId: number) => void;
  selected: boolean;
  onToggleSelect: (motorId: number) => void;
}

const MotorCommandRow = React.memo(function MotorCommandRow({
  motor,
  hasData,
  canControl,
  cmd,
  onChange,
  onSync,
  onSend,
  powered,
  onTogglePower,
  selected,
  onToggleSelect,
}: MotorCommandRowProps) {
  const limits: JointLimits | undefined = motor.name ? JOINT_LIMITS[motor.name] : undefined;

  const handleFieldChange =
    (field: keyof MotorCommandFields) =>
    (e: React.ChangeEvent<HTMLInputElement>) => {
      onChange(motor.id, { ...cmd, [field]: e.target.value });
    };

  const renderCmdBar = (
    field: keyof MotorCommandFields,
    strVal: string,
    min: number, max: number,
    color: string,
    disabled: boolean,
    digits = 3,
  ) => {
    const numVal = parseFloat(strVal);
    const { zeroPct, fillLeft, fillWidth } = barPcts(isNaN(numVal) ? 0 : numVal, min, max);
    const handleClick = (e: React.MouseEvent<HTMLDivElement>) => {
      const rect = e.currentTarget.getBoundingClientRect();
      const val  = min + (e.clientX - rect.left) / rect.width * (max - min);
      onChange(motor.id, { ...cmd, [field]: val.toFixed(digits) });
    };
    return (
      <div
        className={`stat-bar-bg${disabled ? "" : " cmd-bar-active"}`}
        onClick={disabled ? undefined : handleClick}
      >
        <div className="stat-bar-fill" style={{ left: `${fillLeft}%`, width: `${fillWidth}%`, background: color, pointerEvents: "none" }} />
        <div className="stat-bar-zero" style={{ left: `${zeroPct}%`, pointerEvents: "none" }} />
      </div>
    );
  };

  return (
    <tr
      className={
        `${hasData && motor.error ? "row-error" : hasData && motor.warning ? "row-warning" : ""}${selected ? " row-selected" : ""}`
      }
    >
      <td className="cell-check">
        <input
          type="checkbox"
          checked={selected}
          onChange={() => onToggleSelect(motor.id)}
        />
      </td>
      <td className="cell-id">{motor.id}</td>

      {/* 명령 입력칸 */}
      <td className="stat-cell">
        <input
          className="cmd-input cmd-input-full"
          value={cmd.position}
          onChange={handleFieldChange("position")}
          disabled={!powered || !canControl}
        />
        {limits && renderCmdBar("position", cmd.position, limits.lower, limits.upper, "#3b82f6", !powered || !canControl, 2)}
      </td>
      <td className="stat-cell">
        <input
          className="cmd-input cmd-input-full"
          value={cmd.velocity}
          onChange={handleFieldChange("velocity")}
          disabled={!powered || !canControl}
        />
        {limits && renderCmdBar("velocity", cmd.velocity, -limits.velMax, limits.velMax, "#8b5cf6", !powered || !canControl, 2)}
      </td>
      <td className="stat-cell">
        <input
          className="cmd-input cmd-input-full"
          value={cmd.torque}
          onChange={handleFieldChange("torque")}
          disabled={!powered || !canControl}
        />
        {limits && renderCmdBar("torque", cmd.torque, -limits.effortMax, limits.effortMax, "#f59e0b", !powered || !canControl, 2)}
      </td>
      <td className="stat-cell">
        <input
          className="cmd-input cmd-input-full"
          value={cmd.kp}
          onChange={handleFieldChange("kp")}
          disabled={!powered || !canControl}
        />
        {limits && renderCmdBar("kp", cmd.kp, 0, limits.kpMax, "#10b981", !powered || !canControl, 1)}
      </td>
      <td className="stat-cell">
        <input
          className="cmd-input cmd-input-full"
          value={cmd.kd}
          onChange={handleFieldChange("kd")}
          disabled={!powered || !canControl}
        />
        {limits && renderCmdBar("kd", cmd.kd, 0, limits.kdMax, "#10b981", !powered || !canControl, 1)}
      </td>
      <td className="stat-cell">
        <input
          className="cmd-input cmd-input-full"
          value={cmd.durationMs}
          onChange={handleFieldChange("durationMs")}
          disabled={!canControl}
          title="이동 시간 (초), 0 = 즉시"
        />
        {limits && renderCmdBar("durationMs", cmd.durationMs, 0, limits.durationMax, "#6b7280", !canControl, 1)}
      </td>

      {/* 컨트롤: Sync / Send + 슬라이드 토글 */}
      <td>
        <div className="btn-row">
          <button className="btn btn-outline btn-xs" onClick={() => onSync(motor.id)} disabled={!hasData}>
            Sync
          </button>
          <button className="btn btn-primary btn-xs" onClick={() => onSend(motor.id)} disabled={!canControl}>
            Send
          </button>

          <label className="toggle-switch">
            <input
              type="checkbox"
              checked={powered}
              onChange={() => {}}
              onClick={(e) => {
                e.preventDefault();
                onTogglePower(motor.id);
              }}
              disabled={!canControl}
            />
            <span className="toggle-slider" />
          </label>
        </div>
      </td>
    </tr>
  );
});

function App() {
  const [wsPassword, setWsPassword] = React.useState(getInitialWsPassword);
  const [authPasswordInput, setAuthPasswordInput] = React.useState(getInitialWsPassword);
  const {
    connected,
    state,
    authStatus,
    authRetryAt,
    sendMotorPower,
    sendMotorCommand,
    sendMotorControlRequest,
    sendSafetyReset,
    sendDataLogger,
    retryAuth,
    manualConnect,
  } = useRobotWs({
    url: WS_URL,
    password: wsPassword,
  });

  const loggerState = state?.data_logger;
  const isRecording = loggerState?.recording ?? false;

  const [authRetryNow, setAuthRetryNow] = React.useState(Date.now());

  React.useEffect(() => {
    if (authStatus !== "failed") {
      return;
    }

    const timer = window.setInterval(() => {
      setAuthRetryNow(Date.now());
    }, 200);

    return () => {
      window.clearInterval(timer);
    };
  }, [authStatus]);

  // 모터별 명령 상태
  const [motorCommands, setMotorCommands] = React.useState<
    Record<number, MotorCommandFields>
  >({});

  // 일괄 입력 상태
  const [selectedMotorIds, setSelectedMotorIds] = React.useState<Set<number>>(new Set());
  const [bulkCmd, setBulkCmd] = React.useState<MotorCommandFields>(EMPTY_MOTOR_COMMAND);
  const [showPlot, setShowPlot] = React.useState(false);
  const [plotPaused, setPlotPaused] = React.useState(false);
  const [plotColumns, setPlotColumns] = React.useState(DEFAULT_PLOT_COLUMNS);
  const [plotHeight, setPlotHeight] = React.useState(DEFAULT_PLOT_HEIGHT);
  const [samplingMs, setSamplingMs] = React.useState(DEFAULT_SAMPLING_MS);
  const [bufferLength, setBufferLength] = React.useState(DEFAULT_BUFFER_LENGTH);
  const [draftPlotColumns, setDraftPlotColumns] = React.useState(String(DEFAULT_PLOT_COLUMNS));
  const [draftPlotHeight, setDraftPlotHeight] = React.useState(String(DEFAULT_PLOT_HEIGHT));
  const [draftSamplingMs, setDraftSamplingMs] = React.useState(String(DEFAULT_SAMPLING_MS));
  const [draftBufferLength, setDraftBufferLength] = React.useState(String(DEFAULT_BUFFER_LENGTH));
  const nextPlotIdRef = React.useRef(2);
  const lastPlotSampleAtRef = React.useRef(0);

  const [plotPanels, setPlotPanels] = React.useState<PlotPanelConfig[]>([
    {
      id: 1,
      source: "motor_stat",
      motorId: 0,
      motorStatFields: ["position", "velocity"],
      motorCmdFields: ["position", "velocity"],
      imuFields: ["roll", "pitch", "yaw"],
      scaleMode: "auto",
      fixedMin: "-1.0",
      fixedMax: "1.0",
      settingsCollapsed: false,
    },
  ]);

  const [motorStatPlotHistory, setMotorStatPlotHistory] = React.useState<
    Record<number, Array<Record<MotorStatPlotKey, number>>>
  >({});
  const [motorCmdPlotHistory, setMotorCmdPlotHistory] = React.useState<
    Record<number, Array<Record<MotorCmdPlotKey, number>>>
  >({});
  const [imuPlotHistory, setImuPlotHistory] = React.useState<Array<Record<ImuPlotKey, number>>>([]);

  // 상태 들어오면 아직 없는 모터만 초기화
  React.useEffect(() => {
    if (!state) return;
    setMotorCommands((prev) => {
      const next = { ...prev };
      let changed = false;
      for (const m of state.motors) {
        if (!next[m.id]) {
          next[m.id] = makeInitialCommand(m);
          changed = true;
        }
      }
      return changed ? next : prev;
    });
  }, [state]);

  React.useEffect(() => {
    if (!showPlot || !state?.motors.length) return;
    setPlotPanels((prev) => {
      let changed = false;
      const next = prev.map((panel) => {
        if (panel.source === "imu") {
          return panel;
        }
        const stillExists = state.motors.some((motor) => motor.id === panel.motorId);
        if (stillExists) {
          return panel;
        }
        changed = true;
        return { ...panel, motorId: state.motors[0].id };
      });
      return changed ? next : prev;
    });
  }, [showPlot, state]);

  const trackedMotorIds = React.useMemo(() => {
    if (!showPlot) {
      return [] as number[];
    }
    const ids = new Set<number>();
    for (const panel of plotPanels) {
      if (panel.source === "motor_stat" || panel.source === "motor_cmd") {
        ids.add(panel.motorId);
      }
    }
    return [...ids];
  }, [showPlot, plotPanels]);

  const hasImuPanel = React.useMemo(
    () => showPlot && plotPanels.some((panel) => panel.source === "imu"),
    [showPlot, plotPanels],
  );

  React.useEffect(() => {
    if (!showPlot || !state || plotPaused) return;

    const now = Date.now();
    if (now - lastPlotSampleAtRef.current < samplingMs) {
      return;
    }
    lastPlotSampleAtRef.current = now;

    if (trackedMotorIds.length > 0) {
      const motorMap = new Map(state.motors.map((motor) => [motor.id, motor] as const));

      setMotorStatPlotHistory((prev) => {
        const next = { ...prev };
        for (const motorId of trackedMotorIds) {
          const motor = motorMap.get(motorId);
          if (!motor) continue;

          const history = next[motor.id] ? [...next[motor.id]] : [];
          history.push({
            position: motor.position,
            velocity: motor.velocity,
            torque: motor.torque,
            temperature: motor.temperature,
          });
          if (history.length > bufferLength) {
            history.splice(0, history.length - bufferLength);
          }
          next[motor.id] = history;
        }
        return next;
      });

      setMotorCmdPlotHistory((prev) => {
        const next = { ...prev };
        for (const motorId of trackedMotorIds) {
          const motor = motorMap.get(motorId);
          if (!motor) continue;

          const history = next[motor.id] ? [...next[motor.id]] : [];
          history.push({
            position: motor.command_position ?? motor.position,
            velocity: motor.command_velocity ?? motor.velocity,
            torque: motor.command_torque ?? motor.torque,
            kp: motor.command_kp ?? motor.kp ?? 0,
            kd: motor.command_kd ?? motor.kd ?? 0,
          });
          if (history.length > bufferLength) {
            history.splice(0, history.length - bufferLength);
          }
          next[motor.id] = history;
        }
        return next;
      });
    }

    if (hasImuPanel) {
      setImuPlotHistory((prev) => {
        const next = [
          ...prev,
          {
            roll: state.imu.orientation_rpy.roll,
            pitch: state.imu.orientation_rpy.pitch,
            yaw: state.imu.orientation_rpy.yaw,
            gyro_x: state.imu.angular_velocity.x,
            gyro_y: state.imu.angular_velocity.y,
            gyro_z: state.imu.angular_velocity.z,
            accel_x: state.imu.linear_acceleration.x,
            accel_y: state.imu.linear_acceleration.y,
            accel_z: state.imu.linear_acceleration.z,
          },
        ];

        if (next.length > bufferLength) {
          next.splice(0, next.length - bufferLength);
        }

        return next;
      });
    }
  }, [showPlot, state, plotPaused, samplingMs, bufferLength, trackedMotorIds, hasImuPanel]);

  React.useEffect(() => {
    if (!plotPaused) {
      lastPlotSampleAtRef.current = 0;
    }
  }, [plotPaused, samplingMs]);

  React.useEffect(() => {
    const trimMotorHistory = <K extends string,>(
      prev: Record<number, Array<Record<K, number>>>,
    ) => {
      let changed = false;
      const next: Record<number, Array<Record<K, number>>> = {};
      for (const [motorId, history] of Object.entries(prev)) {
        if (history.length > bufferLength) {
          next[Number(motorId)] = history.slice(history.length - bufferLength);
          changed = true;
        } else {
          next[Number(motorId)] = history;
        }
      }
      return changed ? next : prev;
    };

    setMotorStatPlotHistory((prev) => trimMotorHistory(prev));
    setMotorCmdPlotHistory((prev) => trimMotorHistory(prev));

    setImuPlotHistory((prev) =>
      prev.length > bufferLength ? prev.slice(prev.length - bufferLength) : prev,
    );
  }, [bufferLength]);

  const parseNumberOrUndefined = (s: string): number | undefined => {
    const trimmed = s.trim();
    if (!trimmed) return undefined;
    const v = Number(trimmed);
    if (Number.isNaN(v)) return undefined;
    return v;
  };

  const handleMotorCommandChange = React.useCallback((motorId: number, next: MotorCommandFields) => {
    setMotorCommands((prev) => ({
      ...prev,
      [motorId]: next,
    }));
  }, []);

  const handleMotorSyncById = React.useCallback(
    (motorId: number) => {
      if (!state) return;
      const motor = state.motors.find((m) => m.id === motorId);
      if (!motor) return;
      setMotorCommands((prev) => ({
        ...prev,
        [motor.id]: {
          ...(prev[motor.id] ?? makeInitialCommand(motor)),
          position: formatCommandValue(motor.command_position ?? motor.position, "position", prev[motor.id]?.position ?? "0.00"),
          velocity: formatCommandValue(motor.command_velocity ?? motor.velocity, "velocity", prev[motor.id]?.velocity ?? "0.00"),
          torque: formatCommandValue(motor.command_torque ?? motor.torque, "torque", prev[motor.id]?.torque ?? "0.00"),
          kp:
            motor.command_kp !== undefined
              ? formatCommandValue(motor.command_kp, "kp", prev[motor.id]?.kp ?? "0.0")
              : motor.kp !== undefined
                ? formatCommandValue(motor.kp, "kp", prev[motor.id]?.kp ?? "0.0")
                : (prev[motor.id]?.kp ?? "0.0"),
          kd:
            motor.command_kd !== undefined
              ? formatCommandValue(motor.command_kd, "kd", prev[motor.id]?.kd ?? "0.0")
              : motor.kd !== undefined
                ? formatCommandValue(motor.kd, "kd", prev[motor.id]?.kd ?? "0.0")
                : (prev[motor.id]?.kd ?? "0.0"),
        },
      }));
    },
    [state],
  );

  const handleMotorSendById = React.useCallback(
    (motorId: number) => {
      if (!state) return;
      const motor = state.motors.find((m) => m.id === motorId);
      if (!motor || !isMotorControlEnabled(motor.name)) return;
      const cmd = motorCommands[motorId];
      if (!cmd) return;
      const durationSec = parseNumberOrUndefined(cmd.durationMs);
      sendMotorCommand(motorId, {
        position: parseNumberOrUndefined(cmd.position),
        velocity: parseNumberOrUndefined(cmd.velocity),
        torque: parseNumberOrUndefined(cmd.torque),
        kp: parseNumberOrUndefined(cmd.kp),
        kd: parseNumberOrUndefined(cmd.kd),
        duration_ms: durationSec !== undefined && durationSec > 0 ? durationSec * 1000 : 0,
      });
    },
    [state, motorCommands, sendMotorCommand],
  );

  const handleMotorTogglePowerById = React.useCallback(
    (motorId: number) => {
      if (!state) return;
      const motor = state.motors.find((m) => m.id === motorId);
      if (!motor || !isMotorControlEnabled(motor.name)) return;
      sendMotorPower(motorId, !(motor.enabled ?? false));
    },
    [state, sendMotorPower],
  );

  // === 선택 / 일괄 입력 ===

  const handleToggleSelectMotor = React.useCallback((motorId: number) => {
    const motor = state?.motors.find((m) => m.id === motorId);
    if (motor && !isMotorControlEnabled(motor.name)) return;
    setSelectedMotorIds((prev) => {
      const next = new Set(prev);
      if (next.has(motorId)) next.delete(motorId);
      else next.add(motorId);
      return next;
    });
  }, [state]);

  const handleToggleSelectAll = React.useCallback(() => {
    setSelectedMotorIds((prev) => {
      const allIds =
        state?.motors.filter((m) => isMotorControlEnabled(m.name)).map((m) => m.id) ?? [];
      if (allIds.length > 0 && allIds.every((id) => prev.has(id))) {
        return new Set();
      }
      return new Set(allIds);
    });
  }, [state]);

  const handleBulkApply = React.useCallback(() => {
    setMotorCommands((prev) => {
      const next = { ...prev };
      for (const motorId of selectedMotorIds) {
        const motor = state?.motors.find((m) => m.id === motorId);
        if (motor && !isMotorControlEnabled(motor.name)) {
          continue;
        }
        const cur = next[motorId] ?? EMPTY_MOTOR_COMMAND;
        next[motorId] = {
          position:   bulkCmd.position   !== "" ? bulkCmd.position   : cur.position,
          velocity:   bulkCmd.velocity   !== "" ? bulkCmd.velocity   : cur.velocity,
          torque:     bulkCmd.torque     !== "" ? bulkCmd.torque     : cur.torque,
          kp:         bulkCmd.kp         !== "" ? bulkCmd.kp         : cur.kp,
          kd:         bulkCmd.kd         !== "" ? bulkCmd.kd         : cur.kd,
          durationMs: bulkCmd.durationMs !== "" ? bulkCmd.durationMs : cur.durationMs,
        };
      }
      return next;
    });
  }, [selectedMotorIds, bulkCmd]);

  const handleSendSelected = React.useCallback(() => {
    if (!state) return;
    for (const motorId of selectedMotorIds) {
      const cmd = motorCommands[motorId];
      if (!cmd) continue;
      const motor = state.motors.find((m) => m.id === motorId);
      if (!motor || !isMotorControlEnabled(motor.name)) continue;
      const durationSec = parseNumberOrUndefined(cmd.durationMs);
      sendMotorCommand(motorId, {
        position: parseNumberOrUndefined(cmd.position),
        velocity: parseNumberOrUndefined(cmd.velocity),
        torque:   parseNumberOrUndefined(cmd.torque),
        kp:       parseNumberOrUndefined(cmd.kp),
        kd:       parseNumberOrUndefined(cmd.kd),
        duration_ms: durationSec !== undefined && durationSec > 0 ? durationSec * 1000 : 0,
      });
    }
  }, [state, selectedMotorIds, motorCommands, sendMotorCommand]);

  // === 일괄 동작 ===

  // 현재 상태 값으로 전 모터 명령 Sync
  const handleAllSync = () => {
    if (!state) return;
    setMotorCommands((prev) => {
      const next: Record<number, MotorCommandFields> = { ...prev };
      for (const m of state.motors) {
        if (!isMotorControlEnabled(m.name)) continue;
        next[m.id] = {
          ...(next[m.id] ?? makeInitialCommand(m)),
          position: formatCommandValue(m.command_position ?? m.position, "position", next[m.id]?.position ?? "0.00"),
          velocity: formatCommandValue(m.command_velocity ?? m.velocity, "velocity", next[m.id]?.velocity ?? "0.00"),
          torque: formatCommandValue(m.command_torque ?? m.torque, "torque", next[m.id]?.torque ?? "0.00"),
          kp:
            m.command_kp !== undefined
              ? formatCommandValue(m.command_kp, "kp", next[m.id]?.kp ?? "0.0")
              : m.kp !== undefined
                ? formatCommandValue(m.kp, "kp", next[m.id]?.kp ?? "0.0")
              : (next[m.id]?.kp ?? "0.0"),
          kd:
            m.command_kd !== undefined
              ? formatCommandValue(m.command_kd, "kd", next[m.id]?.kd ?? "0.0")
              : m.kd !== undefined
                ? formatCommandValue(m.kd, "kd", next[m.id]?.kd ?? "0.0")
              : (next[m.id]?.kd ?? "0.0"),
        };
      }
      return next;
    });
  };

  // 각 row에 입력된 값 기준으로 전 모터 Send (각각 메시지 전송)
  const handleAllSend = () => {
    if (!state) return;
    for (const m of state.motors) {
      if (!isMotorControlEnabled(m.name)) continue;
      const cmd = motorCommands[m.id];
      if (!cmd) continue;
      const durationSec = parseNumberOrUndefined(cmd.durationMs);
      sendMotorCommand(m.id, {
        position: parseNumberOrUndefined(cmd.position),
        velocity: parseNumberOrUndefined(cmd.velocity),
        torque: parseNumberOrUndefined(cmd.torque),
        kp: parseNumberOrUndefined(cmd.kp),
        kd: parseNumberOrUndefined(cmd.kd),
        duration_ms: durationSec !== undefined && durationSec > 0 ? durationSec * 1000 : 0,
      });
    }
  };

  const handleAllOn = () => {
    if (!state) return;
    for (const m of state.motors) {
      if (!isMotorControlEnabled(m.name)) continue;
      sendMotorPower(m.id, true);
    }
  };

  const handleAllOff = () => {
    if (!state) return;
    for (const m of state.motors) {
      if (!isMotorControlEnabled(m.name)) continue;
      sendMotorPower(m.id, false);
    }
  };

  const handleRequestControl = () => {
    sendMotorControlRequest(true);
  };

  const handleReleaseControl = () => {
    sendMotorControlRequest(false);
  };

  const handleSafetyReset = () => {
    sendSafetyReset();
  };

  const controlRequested = state?.control?.requested ?? false;
  const controlGranted = state?.control?.granted ?? false;
  const canControl = connected && !!state && controlGranted;
  const [controlDeniedUntil, setControlDeniedUntil] = React.useState(0);
  const [controlStatusNow, setControlStatusNow] = React.useState(Date.now());
  const prevControlRequestedRef = React.useRef(false);

  React.useEffect(() => {
    if (!connected || controlGranted) {
      prevControlRequestedRef.current = controlRequested;
      setControlDeniedUntil(0);
      return;
    }

    const prevRequested = prevControlRequestedRef.current;
    if (prevRequested && !controlRequested) {
      setControlDeniedUntil(Date.now() + 2000);
    }
    prevControlRequestedRef.current = controlRequested;
  }, [connected, controlRequested, controlGranted]);

  React.useEffect(() => {
    if (controlDeniedUntil <= Date.now()) {
      return;
    }

    const timer = window.setInterval(() => {
      setControlStatusNow(Date.now());
    }, 200);

    return () => {
      window.clearInterval(timer);
    };
  }, [controlDeniedUntil]);

  const showRecentlyDenied = !controlGranted && controlDeniedUntil > controlStatusNow;

  React.useEffect(() => {
    if (!state || controlGranted) {
      return;
    }

    setMotorCommands((prev) => {
      const next: Record<number, MotorCommandFields> = { ...prev };
      for (const motor of state.motors) {
        next[motor.id] = {
          ...(next[motor.id] ?? makeInitialCommand(motor)),
          position: formatCommandValue(motor.command_position ?? motor.position, "position", next[motor.id]?.position ?? "0.00"),
          velocity: formatCommandValue(motor.command_velocity ?? motor.velocity, "velocity", next[motor.id]?.velocity ?? "0.00"),
          torque: formatCommandValue(motor.command_torque ?? motor.torque, "torque", next[motor.id]?.torque ?? "0.00"),
          kp:
            motor.command_kp !== undefined
              ? formatCommandValue(motor.command_kp, "kp", next[motor.id]?.kp ?? "0.0")
              : motor.kp !== undefined
                ? formatCommandValue(motor.kp, "kp", next[motor.id]?.kp ?? "0.0")
                : (next[motor.id]?.kp ?? "0.0"),
          kd:
            motor.command_kd !== undefined
              ? formatCommandValue(motor.command_kd, "kd", next[motor.id]?.kd ?? "0.0")
              : motor.kd !== undefined
                ? formatCommandValue(motor.kd, "kd", next[motor.id]?.kd ?? "0.0")
                : (next[motor.id]?.kd ?? "0.0"),
        };
      }
      return next;
    });
  }, [state, controlGranted]);

  const controlStatusText = !connected
    ? "권한 상태: 미연결"
    : controlGranted
      ? "권한 상태: 승인됨"
      : controlRequested || showRecentlyDenied
        ? "권한 상태: 거부됨"
        : "권한 상태: 미요청";
  const controlStatusClass = !connected
    ? "text-bad"
    : controlGranted
      ? "text-ok"
      : "text-bad";
  const currentRobotMode: RobotMode = state?.robot_mode?.current ?? "IDLE";
  const currentSafetyLevel = state?.safety?.level ?? "ESSENTIAL";
  const safetyLocked = state?.safety?.locked ?? false;
  const safetyRestoring = state?.safety?.restoring ?? false;
  const addPlotPanel = () => {
    const nextId = nextPlotIdRef.current;
    nextPlotIdRef.current += 1;
    const firstMotorId = state?.motors?.[0]?.id ?? 0;
    setPlotPanels((prev) => [
      ...prev,
      {
        id: nextId,
        source: "motor_stat",
        motorId: firstMotorId,
        motorStatFields: ["position", "velocity"],
        motorCmdFields: ["position", "velocity"],
        imuFields: ["roll", "pitch", "yaw"],
        scaleMode: "auto",
        fixedMin: "-1.0",
        fixedMax: "1.0",
        settingsCollapsed: false,
      },
    ]);
  };

  const applyPlotSettings = () => {
    const parsedColumns = Number(draftPlotColumns);
    const parsedHeight = Number(draftPlotHeight);
    const parsedSampling = Number(draftSamplingMs);
    const parsedBuffer = Number(draftBufferLength);
    if (
      !Number.isFinite(parsedColumns) ||
      !Number.isFinite(parsedHeight) ||
      !Number.isFinite(parsedSampling) ||
      !Number.isFinite(parsedBuffer)
    ) {
      return;
    }

    const nextColumns = clampInt(parsedColumns, MIN_PLOT_COLUMNS, MAX_PLOT_COLUMNS);
    const nextHeight = clampInt(parsedHeight, MIN_PLOT_HEIGHT, MAX_PLOT_HEIGHT);
    const nextSampling = clampInt(parsedSampling, MIN_SAMPLING_MS, MAX_SAMPLING_MS);
    const nextBuffer = clampInt(parsedBuffer, MIN_BUFFER_LENGTH, MAX_BUFFER_LENGTH);

    setPlotColumns(nextColumns);
    setPlotHeight(nextHeight);
    setSamplingMs(nextSampling);
    setBufferLength(nextBuffer);
    setDraftPlotColumns(String(nextColumns));
    setDraftPlotHeight(String(nextHeight));
    setDraftSamplingMs(String(nextSampling));
    setDraftBufferLength(String(nextBuffer));
    lastPlotSampleAtRef.current = 0;
  };

  const removePlotPanel = (panelId: number) => {
    setPlotPanels((prev) => prev.filter((panel) => panel.id !== panelId));
  };

  const getSeriesForPanel = (panel: PlotPanelConfig): PlotSeries[] => {
    if (panel.source === "motor_stat") {
      const history = motorStatPlotHistory[panel.motorId] ?? [];
      return MOTOR_STAT_SERIES_DEFS.filter((seriesDef) => panel.motorStatFields.includes(seriesDef.key)).map(
        (seriesDef) => ({
          ...seriesDef,
          values: history.map((sample) => sample[seriesDef.key]),
        }),
      );
    }

    if (panel.source === "motor_cmd") {
      const history = motorCmdPlotHistory[panel.motorId] ?? [];
      return MOTOR_CMD_SERIES_DEFS.filter((seriesDef) => panel.motorCmdFields.includes(seriesDef.key)).map(
        (seriesDef) => ({
          ...seriesDef,
          values: history.map((sample) => sample[seriesDef.key]),
        }),
      );
    }

    return IMU_SERIES_DEFS.filter((seriesDef) => panel.imuFields.includes(seriesDef.key)).map(
      (seriesDef) => ({
        ...seriesDef,
        values: imuPlotHistory.map((sample) => sample[seriesDef.key]),
      }),
    );
  };

  const updatePlotPanel = (panelId: number, updater: (panel: PlotPanelConfig) => PlotPanelConfig) => {
    setPlotPanels((prev) => prev.map((panel) => (panel.id === panelId ? updater(panel) : panel)));
  };

  const handleMotorStatFieldToggle = (panelId: number, field: MotorStatPlotKey) => {
    updatePlotPanel(panelId, (panel) => {
      const fields = panel.motorStatFields;
      if (fields.includes(field)) {
        if (fields.length === 1) {
          return panel;
        }
        return { ...panel, motorStatFields: fields.filter((item) => item !== field) };
      }
      return { ...panel, motorStatFields: [...fields, field] };
    });
  };

  const handleMotorCmdFieldToggle = (panelId: number, field: MotorCmdPlotKey) => {
    updatePlotPanel(panelId, (panel) => {
      const fields = panel.motorCmdFields;
      if (fields.includes(field)) {
        if (fields.length === 1) {
          return panel;
        }
        return { ...panel, motorCmdFields: fields.filter((item) => item !== field) };
      }
      return { ...panel, motorCmdFields: [...fields, field] };
    });
  };

  const handleImuFieldToggle = (panelId: number, field: ImuPlotKey) => {
    updatePlotPanel(panelId, (panel) => {
      const fields = panel.imuFields;
      if (fields.includes(field)) {
        if (fields.length === 1) {
          return panel;
        }
        return { ...panel, imuFields: fields.filter((item) => item !== field) };
      }
      return { ...panel, imuFields: [...fields, field] };
    });
  };

  const hasStateData = !!state;
  const allPanelsCollapsed =
    plotPanels.length > 0 && plotPanels.every((panel) => panel.settingsCollapsed);
  const displayMotors = React.useMemo<MotorState[]>(
    () => state?.motors ?? PLACEHOLDER_MOTORS,
    [state],
  );
  const controlMotors = React.useMemo<MotorState[]>(
    () => displayMotors.filter((m) => isMotorControlEnabled(m.name)),
    [displayMotors],
  );
  const selectedControlMotorCount = React.useMemo(
    () => controlMotors.filter((m) => selectedMotorIds.has(m.id)).length,
    [controlMotors, selectedMotorIds],
  );

  const retryRemainingMs = authRetryAt ? Math.max(0, authRetryAt - authRetryNow) : 0;
  const retryRemainingSec = Math.ceil(retryRemainingMs / 1000);
  const canInputPassword = authStatus === "required" || authStatus === "failed";
  const canSubmitPassword = canInputPassword && retryRemainingMs <= 0 && authPasswordInput.trim().length > 0;

  const handleAuthRetry = (event?: React.FormEvent) => {
    event?.preventDefault();

    if (!canSubmitPassword) {
      return;
    }

    const input = authPasswordInput.trim();
    if (input.length === 0) {
      return;
    }

    window.sessionStorage.setItem("ws_password", input);
    setWsPassword(input);
    retryAuth(input);
  };

  if (authStatus !== "ok") {
    const title =
      authStatus === "failed"
        ? "인증 실패"
        : authStatus === "required"
          ? "인증 필요"
          : "인증 중";
    const description =
      authStatus === "failed"
        ? "비밀번호가 올바르지 않아 접속할 수 없습니다."
        : authStatus === "required"
          ? "비밀번호를 입력해야 접속할 수 있습니다."
          : "서버 인증을 진행 중입니다.";
    const buttonLabel =
      retryRemainingMs > 0
        ? `${retryRemainingSec}초 후 재시도`
        : canInputPassword
          ? "비밀번호 입력"
          : "연결 대기중";

    return (
      <div className="auth-screen">
        <div className="auth-card">
          <h2>{title}</h2>
          <p>{description}</p>

          <form className="auth-form" onSubmit={handleAuthRetry}>
            <input
              className="auth-input"
              type="password"
              placeholder="비밀번호"
              value={authPasswordInput}
              onChange={(event) => setAuthPasswordInput(event.target.value)}
              disabled={!canInputPassword || retryRemainingMs > 0}
              autoFocus
            />
            <button
              className="btn btn-primary"
              type="submit"
              disabled={!canSubmitPassword}
            >
              {buttonLabel}
            </button>
          </form>
        </div>
      </div>
    );
  }

  return (
    <div className="app-shell">
      <div className="top-mode-bar">
        <div className="top-mode-content">
          <span className="toolbar-label">모드:</span>
          <span className="mode-badge">{currentRobotMode}</span>
          <div className="top-connection">
            SAFETY:
            <span>{currentSafetyLevel}</span>
            <span className={safetyLocked ? "text-bad" : safetyRestoring ? "text-warn" : "text-ok"}>
              {safetyLocked ? "LOCK" : safetyRestoring ? "RESTORE" : "UNLOCK"}
            </span>
            <button
              className="btn btn-secondary btn-xs"
              onClick={handleSafetyReset}
              disabled={!connected || !safetyLocked}
            >
              LOCK Reset
            </button>
          </div>
          <div className="top-connection">
            연결 상태:
            {connected ? (
              <span className="text-ok">Connected</span>
            ) : (
              <>
                <span className="text-bad">Disconnected</span>
                <button
                  className="btn btn-secondary btn-xs"
                  onClick={manualConnect}
                >
                  연결
                </button>
              </>
            )}
          </div>
        </div>
      </div>

      <div className="app">
        {/* 메인 모니터 */}
        <div className="main-panel">
          <header className="header">
            <h2>Robot Monitor</h2>
          </header>

          {/* 모터 일괄 액션 툴바 */}
          <div className="toolbar">
            <span className="toolbar-label">모터 일괄 작업:</span>
            <span className={controlStatusClass}>{controlStatusText}</span>
            <button className="btn btn-secondary btn-xs" onClick={handleRequestControl} disabled={!connected}>
              제어권한 요청
            </button>
            <button className="btn btn-secondary btn-xs" onClick={handleReleaseControl} disabled={!connected}>
              제어권한 해제
            </button>
            <button className="btn btn-secondary btn-xs" onClick={handleAllSync} disabled={!connected || !state}>
              Sync All
            </button>
            <button className="btn btn-primary btn-xs" onClick={handleAllSend} disabled={!canControl}>
              Send All
            </button>
            <button
              className="btn btn-primary btn-xs"
              onClick={handleSendSelected}
              disabled={!canControl || selectedControlMotorCount === 0}
              title={selectedControlMotorCount === 0 ? "모터를 선택하세요" : `${selectedControlMotorCount}개 모터에 전송`}
            >
              Send Sel ({selectedControlMotorCount})
            </button>
            <button className="btn btn-secondary btn-xxs" onClick={handleAllOn} disabled={!canControl}>
              On All
            </button>
            <button className="btn btn-secondary btn-xxs" onClick={handleAllOff} disabled={!canControl}>
              Off All
            </button>
            <button className="btn btn-secondary btn-xs" onClick={() => setShowPlot((prev) => !prev)}>
              {showPlot ? "Plot Hide" : "Plot Show"}
            </button>
            <div className="record-group">
              <button
                className={isRecording ? "btn btn-record-stop btn-xs" : "btn btn-record btn-xs"}
                onClick={() => sendDataLogger(!isRecording)}
                disabled={!connected}
                title={isRecording ? "기록 중지" : "조인트 데이터 CSV 기록 시작"}
              >
                {isRecording ? "■ Stop" : "● Record"}
              </button>
              {loggerState && (
                <span className="record-status">
                  {isRecording
                    ? `${loggerState.sample_count.toLocaleString()} samples`
                    : loggerState.sample_count > 0
                      ? `저장됨: ${loggerState.filename.split("/").pop()}`
                      : ""}
                </span>
              )}
            </div>
          </div>

          <>
            <div className="motors-panel">
              {/* 왼쪽: 상태 테이블 */}
              <div className="card motors-status-card">
                <div className="card-header">
                  <div className="card-header-split">
                    <h3>Motor Status</h3>
                    <span className="status-legend">
                      <span className="status-legend-cmd">L: Cmd</span>
                      <span className="status-legend-now">R: Now</span>
                    </span>
                  </div>
                </div>
                <div className="table-wrapper">
                  <table className="data-table status-table">
                    <colgroup>
                      <col style={{ width: "28px" }} />
                      <col style={{ width: "88px" }} />
                      <col style={{ width: "32px" }} />
                      <col style={{ width: "72px" }} />
                      <col style={{ width: "72px" }} />
                      <col style={{ width: "72px" }} />
                      <col style={{ width: "36px" }} />
                      <col style={{ width: "36px" }} />
                      <col style={{ width: "46px" }} />
                    </colgroup>
                    <thead>
                      <tr>
                        <th className="head-id">ID</th>
                        <th className="head-name">Name</th>
                        <th className="head-mode">St</th>
                        <th className="head-pair">Pos</th>
                        <th className="head-pair">Vel</th>
                        <th className="head-pair">Torque</th>
                        <th className="num">Kp</th>
                        <th className="num">Kd</th>
                        <th className="num">Temp</th>
                      </tr>
                    </thead>
                    <tbody>
                      <tr className="bulk-spacer-row"><td colSpan={9} /></tr>
                      {displayMotors.map((m) => (
                        <MotorStatusRow key={m.id} motor={m} hasData={hasStateData} />
                      ))}
                    </tbody>
                  </table>
                </div>
              </div>

              {/* 오른쪽: 제어 테이블 */}
              <div className="card motors-control-card">
                <div className="card-header">
                  <h3>Motor Control</h3>
                </div>
                <div className="table-wrapper">
                  <table className="data-table control-table">
                    <colgroup>
                      <col style={{ width: "18px" }} />
                      <col style={{ width: "22px" }} />
                      <col style={{ width: "46px" }} />
                      <col style={{ width: "46px" }} />
                      <col style={{ width: "50px" }} />
                      <col style={{ width: "46px" }} />
                      <col style={{ width: "42px" }} />
                      <col style={{ width: "46px" }} />
                      <col style={{ width: "122px" }} />
                    </colgroup>
                    <thead>
                      <tr>
                        <th className="head-check">
                          <input
                            type="checkbox"
                            checked={
                              controlMotors.length > 0 &&
                              controlMotors.every((m) => selectedMotorIds.has(m.id))
                            }
                            ref={(el) => {
                              if (el) {
                                const some = controlMotors.some((m) => selectedMotorIds.has(m.id));
                                const all  = controlMotors.length > 0 && controlMotors.every((m) => selectedMotorIds.has(m.id));
                                el.indeterminate = some && !all;
                              }
                            }}
                            onChange={handleToggleSelectAll}
                          />
                        </th>
                        <th className="head-id">ID</th>
                        <th className="num">Pos</th>
                        <th className="num">Vel</th>
                        <th className="num">Torque</th>
                        <th className="num">Kp</th>
                        <th className="num">Kd</th>
                        <th className="num">시간(s)</th>
                        <th className="head-control">Control</th>
                      </tr>
                    </thead>
                    <tbody>
                      {/* 일괄 입력 행 */}
                      <tr className="bulk-row">
                        <td className="cell-check bulk-label" colSpan={2}>일괄</td>
                        {(["position", "velocity", "torque", "kp", "kd", "durationMs"] as (keyof MotorCommandFields)[]).map((field) => (
                          <td key={field} className="stat-cell">
                            <input
                              className="cmd-input cmd-input-full"
                              value={bulkCmd[field]}
                              placeholder="—"
                              onChange={(e) => setBulkCmd((prev) => ({ ...prev, [field]: e.target.value }))}
                              disabled={!canControl}
                            />
                          </td>
                        ))}
                        <td>
                          <div className="btn-row">
                            <button
                              className="btn btn-primary btn-xs"
                              onClick={handleBulkApply}
                              disabled={!canControl || selectedControlMotorCount === 0}
                              title={selectedControlMotorCount === 0 ? "모터를 선택하세요" : `${selectedControlMotorCount}개 모터에 적용`}
                            >
                              Apply ({selectedControlMotorCount})
                            </button>
                          </div>
                        </td>
                      </tr>
                      {controlMotors.map((m) => {
                        const powered = hasStateData ? (m.enabled ?? false) : false;
                        return (
                          <MotorCommandRow
                            key={m.id}
                            motor={m}
                            hasData={hasStateData}
                            canControl={canControl}
                            cmd={motorCommands[m.id] ?? EMPTY_MOTOR_COMMAND}
                            onChange={handleMotorCommandChange}
                            onSync={handleMotorSyncById}
                            onSend={handleMotorSendById}
                            powered={powered}
                            onTogglePower={handleMotorTogglePowerById}
                            selected={selectedMotorIds.has(m.id)}
                            onToggleSelect={handleToggleSelectMotor}
                          />
                        );
                      })}
                    </tbody>
                  </table>
                </div>
              </div>
            </div>

            {/* Joint Limits 카드 — config/robotnl.yaml 기준 */}
            <div className="bottom-row">
              <JointLimitsTable />
            </div>

            {showPlot && (
              <div className="card card-plot">
                <div className="card-header">
                  <div className="plot-header-row">
                    <h3>Plot Monitor</h3>
                    <div className="plot-header-actions">
                      <button
                        className="btn btn-secondary btn-xs"
                        onClick={() =>
                          setPlotPanels((prev) =>
                            prev.map((panel) => ({
                              ...panel,
                              settingsCollapsed: !allPanelsCollapsed,
                            })),
                          )
                        }
                        disabled={plotPanels.length === 0}
                      >
                        {allPanelsCollapsed ? "설정 일괄 펼치기" : "설정 일괄 접기"}
                      </button>
                      <button
                        className="btn btn-secondary btn-xs"
                        onClick={() => setPlotPaused((prev) => !prev)}
                      >
                        {plotPaused ? "Resume" : "Pause"}
                      </button>
                      <button className="btn btn-secondary btn-xs" onClick={addPlotPanel}>
                        + Plot 추가
                      </button>
                    </div>
                  </div>
                </div>

                <div className="plot-controls">
                  <div className="plot-control-group">
                    <label htmlFor="plot-sampling-ms">샘플링 주기 (ms)</label>
                    <input
                      id="plot-sampling-ms"
                      type="number"
                      min={MIN_SAMPLING_MS}
                      max={MAX_SAMPLING_MS}
                      value={draftSamplingMs}
                      onChange={(e) => setDraftSamplingMs(e.target.value)}
                    />
                  </div>

                  <div className="plot-control-group">
                    <label htmlFor="plot-buffer-length">버퍼 길이 (points)</label>
                    <input
                      id="plot-buffer-length"
                      type="number"
                      min={MIN_BUFFER_LENGTH}
                      max={MAX_BUFFER_LENGTH}
                      value={draftBufferLength}
                      onChange={(e) => setDraftBufferLength(e.target.value)}
                    />
                  </div>

                  <div className="plot-control-group">
                    <label htmlFor="plot-columns">컬럼 수</label>
                    <input
                      id="plot-columns"
                      type="number"
                      min={MIN_PLOT_COLUMNS}
                      max={MAX_PLOT_COLUMNS}
                      value={draftPlotColumns}
                      onChange={(e) => setDraftPlotColumns(e.target.value)}
                    />
                  </div>

                  <div className="plot-control-group">
                    <label htmlFor="plot-height-global">플롯 높이 (px)</label>
                    <input
                      id="plot-height-global"
                      type="number"
                      min={MIN_PLOT_HEIGHT}
                      max={MAX_PLOT_HEIGHT}
                      value={draftPlotHeight}
                      onChange={(e) => setDraftPlotHeight(e.target.value)}
                    />
                  </div>

                  <div className="plot-control-group plot-control-apply">
                    <button className="btn btn-primary btn-xs" onClick={applyPlotSettings}>
                      Apply
                    </button>
                    <span>
                      적용값: {plotColumns} col / {plotHeight}px / {samplingMs} ms / {bufferLength} pts
                    </span>
                  </div>
                </div>

                {plotPanels.length === 0 ? (
                  <div className="plot-empty">플롯이 없습니다. + Plot 추가를 눌러주세요.</div>
                ) : (
                  <div
                    className="plot-panels"
                    style={{ gridTemplateColumns: `repeat(${plotColumns}, minmax(0, 1fr))` }}
                  >
                    {plotPanels.map((panel) => {
                      const panelSeries = getSeriesForPanel(panel);

                      return (
                        <div className="plot-panel" key={panel.id}>
                          <div className="plot-panel-header">
                            <h4>{getPlotPanelTitle(panel)}</h4>
                            <div className="plot-panel-header-actions">
                              <button
                                className="btn btn-secondary btn-xs"
                                onClick={() =>
                                  updatePlotPanel(panel.id, (prev) => ({
                                    ...prev,
                                    settingsCollapsed: !prev.settingsCollapsed,
                                  }))
                                }
                              >
                                {panel.settingsCollapsed ? "설정 펼치기" : "설정 접기"}
                              </button>
                              <button
                                className="btn btn-secondary btn-xs"
                                onClick={() => removePlotPanel(panel.id)}
                              >
                                - 제거
                              </button>
                            </div>
                          </div>

                          {!panel.settingsCollapsed && (
                            <div className="plot-controls plot-controls-panel">
                              <div className="plot-control-group">
                                <label htmlFor={`plot-source-${panel.id}`}>데이터 소스</label>
                                <select
                                  id={`plot-source-${panel.id}`}
                                  value={panel.source}
                                  onChange={(e) =>
                                    updatePlotPanel(panel.id, (prev) => ({
                                      ...prev,
                                      source: e.target.value as PlotSource,
                                    }))
                                  }
                                >
                                  <option value="motor_stat">모터 상태</option>
                                  <option value="motor_cmd">모터 명령</option>
                                  <option value="imu">IMU</option>
                                </select>
                              </div>

                              {(panel.source === "motor_stat" || panel.source === "motor_cmd") && (
                                <div className="plot-control-group">
                                  <label htmlFor={`plot-motor-id-${panel.id}`}>모터 ID</label>
                                  <select
                                    id={`plot-motor-id-${panel.id}`}
                                    value={panel.motorId}
                                    onChange={(e) =>
                                      updatePlotPanel(panel.id, (prev) => ({
                                        ...prev,
                                        motorId: Number(e.target.value),
                                      }))
                                    }
                                    disabled={!state?.motors.length}
                                  >
                                    {(state?.motors ?? []).map((motor) => (
                                      <option key={`motor-opt-${panel.id}-${motor.id}`} value={motor.id}>
                                        {motor.id} ({motor.name ?? `motor_${motor.id}`})
                                      </option>
                                    ))}
                                  </select>
                                </div>
                              )}

                              <div className="plot-control-group">
                                <label htmlFor={`plot-scale-mode-${panel.id}`}>Y 스케일</label>
                                <select
                                  id={`plot-scale-mode-${panel.id}`}
                                  value={panel.scaleMode}
                                  onChange={(e) =>
                                    updatePlotPanel(panel.id, (prev) => ({
                                      ...prev,
                                      scaleMode: e.target.value as "auto" | "fixed",
                                    }))
                                  }
                                >
                                  <option value="auto">자동 스케일</option>
                                  <option value="fixed">고정 스케일</option>
                                </select>
                              </div>

                              {panel.scaleMode === "fixed" && (
                                <>
                                  <div className="plot-control-group">
                                    <label htmlFor={`plot-fixed-min-${panel.id}`}>Y Min</label>
                                    <input
                                      id={`plot-fixed-min-${panel.id}`}
                                      type="number"
                                      step="0.1"
                                      value={panel.fixedMin}
                                      onChange={(e) =>
                                        updatePlotPanel(panel.id, (prev) => ({
                                          ...prev,
                                          fixedMin: e.target.value,
                                        }))
                                      }
                                    />
                                  </div>

                                  <div className="plot-control-group">
                                    <label htmlFor={`plot-fixed-max-${panel.id}`}>Y Max</label>
                                    <input
                                      id={`plot-fixed-max-${panel.id}`}
                                      type="number"
                                      step="0.1"
                                      value={panel.fixedMax}
                                      onChange={(e) =>
                                        updatePlotPanel(panel.id, (prev) => ({
                                          ...prev,
                                          fixedMax: e.target.value,
                                        }))
                                      }
                                    />
                                  </div>
                                </>
                              )}

                              <div className="plot-control-group plot-control-fields">
                                <span>표시 항목</span>

                                <div className="plot-field-row">
                                  {panel.source === "motor_stat"
                                    ? MOTOR_STAT_SERIES_DEFS.map((seriesDef) => (
                                        <label key={`motor-field-${panel.id}-${seriesDef.key}`}>
                                          <input
                                            type="checkbox"
                                            checked={panel.motorStatFields.includes(seriesDef.key)}
                                            onChange={() =>
                                              handleMotorStatFieldToggle(panel.id, seriesDef.key)
                                            }
                                          />
                                          {seriesDef.label}
                                        </label>
                                      ))
                                    : panel.source === "motor_cmd"
                                      ? MOTOR_CMD_SERIES_DEFS.map((seriesDef) => (
                                          <label key={`motor-cmd-field-${panel.id}-${seriesDef.key}`}>
                                            <input
                                              type="checkbox"
                                              checked={panel.motorCmdFields.includes(seriesDef.key)}
                                              onChange={() =>
                                                handleMotorCmdFieldToggle(panel.id, seriesDef.key)
                                              }
                                            />
                                            {seriesDef.label}
                                          </label>
                                        ))
                                    : IMU_SERIES_DEFS.map((seriesDef) => (
                                        <label key={`imu-field-${panel.id}-${seriesDef.key}`}>
                                          <input
                                            type="checkbox"
                                            checked={panel.imuFields.includes(seriesDef.key)}
                                            onChange={() => handleImuFieldToggle(panel.id, seriesDef.key)}
                                          />
                                          {seriesDef.label}
                                        </label>
                                      ))}
                                </div>
                              </div>
                            </div>
                          )}

                          <div className="plot-body">
                            {panelSeries.length > 0 ? (
                              <TimeSeriesPlot
                                series={panelSeries}
                                scaleMode={panel.scaleMode}
                                fixedMin={Number(panel.fixedMin)}
                                fixedMax={Number(panel.fixedMax)}
                                plotHeight={plotHeight}
                              />
                            ) : (
                              <div className="plot-empty">표시할 데이터를 1개 이상 선택해주세요.</div>
                            )}
                          </div>
                        </div>
                      );
                    })}
                  </div>
                )}
              </div>
            )}
          </>
        </div>
      </div>
    </div>
  );
}

export default App;
