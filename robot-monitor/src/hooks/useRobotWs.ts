import { useEffect, useRef, useState } from "react";
import type { StateMessage, StatePayload, BaseMessage, RobotMode } from "../types";

export type WsAuthStatus = "not_required" | "pending" | "required" | "ok" | "failed";

interface UseRobotWsOptions {
  url: string;
  autoReconnect?: boolean;
  password?: string;
  reconnectIntervalMs?: number;
  connectTimeoutMs?: number;
}

export function useRobotWs({
  url,
  autoReconnect = true,
  password,
  reconnectIntervalMs = 3000,
  connectTimeoutMs = 3000,
}: UseRobotWsOptions) {
  const [connected, setConnected] = useState(false);
  const [state, setState] = useState<StatePayload | null>(null);
  const [authStatus, setAuthStatus] = useState<WsAuthStatus>("pending");
  const [authRetryAt, setAuthRetryAt] = useState<number | null>(null);
  const wsRef = useRef<WebSocket | null>(null);
  const reconnectTimerRef = useRef<number | null>(null);
  const connectTimeoutRef = useRef<number | null>(null);
  const stateWatchdogRef = useRef<number | null>(null);
  const connectionIdRef = useRef(0);
  const passwordRef = useRef((password ?? "").trim());
  const manualConnectRef = useRef<(() => void) | null>(null);

  useEffect(() => {
    passwordRef.current = (password ?? "").trim();
  }, [password]);

  const sendRaw = (msg: BaseMessage<any>) => {
    if (!wsRef.current || wsRef.current.readyState !== WebSocket.OPEN) {
      console.warn("WebSocket not connected, cannot send", msg);
      return;
    }
    wsRef.current.send(JSON.stringify(msg));
  };

  const sendMotorPower = (motorId: number | "all", on: boolean) => {
    const msg: BaseMessage<{ motorId: number | "all"; on: boolean }> = {
      type: "motor_power",
      timestamp: Date.now(),
      payload: { motorId, on },
    };
    sendRaw(msg);
  };

  const sendMotorCommand = (
    motorId: number | "all",
    cmd: {
      position?: number;
      velocity?: number;
      torque?: number;
      kp?: number;
      kd?: number;
      duration_ms?: number;
    },
  ) => {
    const msg: BaseMessage<{
      motorId: number | "all";
      command: typeof cmd;
    }> = {
      type: "motor_command",
      timestamp: Date.now(),
      payload: { motorId, command: cmd },
    };
    sendRaw(msg);
  };

  const sendMotorControlRequest = (request = true) => {
    const msg: BaseMessage<{ request: boolean }> = {
      type: "motor_control_request",
      timestamp: Date.now(),
      payload: { request },
    };
    sendRaw(msg);
  };

  const sendRobotModeRequest = (mode: RobotMode) => {
    const msg: BaseMessage<{ mode: RobotMode }> = {
      type: "robot_mode_request",
      timestamp: Date.now(),
      payload: { mode },
    };
    sendRaw(msg);
  };

  const sendSafetyReset = () => {
    const msg: BaseMessage<{ request: boolean }> = {
      type: "safety_reset",
      timestamp: Date.now(),
      payload: { request: true },
    };
    sendRaw(msg);
  };

  const sendDataLogger = (start: boolean) => {
    const msg: BaseMessage<{ start: boolean }> = {
      type: "data_logger",
      timestamp: Date.now(),
      payload: { start },
    };
    sendRaw(msg);
  };

  const sendAuth = (socket: WebSocket, passwordText?: string) => {
    const candidate = (passwordText ?? passwordRef.current ?? "").trim();
    if (candidate.length === 0) {
      setAuthStatus("required");
      setAuthRetryAt(null);
      return false;
    }

    const authMsg: BaseMessage<{ password: string }> = {
      type: "auth",
      timestamp: Date.now(),
      payload: { password: candidate },
    };
    socket.send(JSON.stringify(authMsg));
    setAuthStatus("pending");
    return true;
  };

  const retryAuth = (passwordText?: string) => {
    const socket = wsRef.current;
    if (!socket || socket.readyState !== WebSocket.OPEN) {
      return false;
    }

    if (authRetryAt !== null && Date.now() < authRetryAt) {
      return false;
    }

    if (typeof passwordText === "string") {
      passwordRef.current = passwordText.trim();
    }

    return sendAuth(socket, passwordRef.current);
  };

  useEffect(() => {
    let disposed = false;

    const clearReconnectTimer = () => {
      if (reconnectTimerRef.current !== null) {
        clearTimeout(reconnectTimerRef.current);
        reconnectTimerRef.current = null;
      }
    };

    const clearConnectTimeout = () => {
      if (connectTimeoutRef.current !== null) {
        clearTimeout(connectTimeoutRef.current);
        connectTimeoutRef.current = null;
      }
    };

    const clearStateWatchdog = () => {
      if (stateWatchdogRef.current !== null) {
        clearTimeout(stateWatchdogRef.current);
        stateWatchdogRef.current = null;
      }
    };

    const STATE_WATCHDOG_MS = 5000;
    const armStateWatchdog = (ws: WebSocket, connectionId: number) => {
      clearStateWatchdog();
      stateWatchdogRef.current = window.setTimeout(() => {
        if (disposed || wsRef.current !== ws || connectionIdRef.current !== connectionId) return;
        console.warn(`No state received for ${STATE_WATCHDOG_MS}ms, forcing reconnect`);
        ws.close();
      }, STATE_WATCHDOG_MS);
    };

    const scheduleReconnect = () => {
      if (!autoReconnect || disposed) {
        return;
      }

      clearReconnectTimer();
      reconnectTimerRef.current = window.setTimeout(connect, reconnectIntervalMs);
    };

    function connect() {
      if (disposed) return;

      clearReconnectTimer();
      clearConnectTimeout();
      const connectionId = ++connectionIdRef.current;
      const ws = new WebSocket(url);
      wsRef.current = ws;

      connectTimeoutRef.current = window.setTimeout(() => {
        if (disposed || wsRef.current !== ws || connectionIdRef.current !== connectionId) {
          return;
        }

        if (ws.readyState === WebSocket.CONNECTING) {
          console.warn(`WebSocket connect timeout (${connectTimeoutMs}ms), forcing reconnect`);
          ws.close();
        }
      }, connectTimeoutMs);

      const sendSubscribe = () => {
        const msg: BaseMessage<{ rate: number }> = {
          type: "subscribe_state",
          timestamp: Date.now(),
          payload: { rate: 10 },
        };
        ws.send(JSON.stringify(msg));
        armStateWatchdog(ws, connectionId);
      };

      ws.onopen = () => {
        if (disposed || wsRef.current !== ws || connectionIdRef.current !== connectionId) {
          return;
        }

        clearConnectTimeout();
        setConnected(true);
        if (passwordRef.current.length === 0) {
          setAuthStatus("not_required");
          setAuthRetryAt(null);
          sendSubscribe();
          return;
        }

        sendAuth(ws, passwordRef.current);
      };

      ws.onclose = () => {
        if (disposed || wsRef.current !== ws || connectionIdRef.current !== connectionId) {
          return;
        }

        clearConnectTimeout();
        clearStateWatchdog();
        wsRef.current = null;
        setConnected(false);
        scheduleReconnect();
      };

      ws.onerror = () => {
        if (disposed || wsRef.current !== ws || connectionIdRef.current !== connectionId) {
          return;
        }
        console.error("WebSocket error");
        if (ws.readyState === WebSocket.CONNECTING) {
          ws.close();
        }
      };

      ws.onmessage = (event) => {
        if (disposed || wsRef.current !== ws || connectionIdRef.current !== connectionId) {
          return;
        }
        try {
          const msg = JSON.parse(event.data) as BaseMessage;
          if (msg.type === "auth_ok") {
            setAuthStatus("ok");
            setAuthRetryAt(null);
            sendSubscribe();
          } else if (msg.type === "auth_required") {
            console.warn("WebSocket auth required");
            if (passwordRef.current.length === 0) {
              setAuthStatus("required");
            }
          } else if (msg.type === "auth_failed") {
            console.error("WebSocket auth failed");
            setAuthStatus("failed");
            setAuthRetryAt(Date.now() + 3000);
            passwordRef.current = "";
          } else if (msg.type === "state") {
            const stateMsg = msg as StateMessage;
            setState(stateMsg.payload);
            armStateWatchdog(ws, connectionId);
          } else {
            console.debug("Unhandled message", msg);
          }
        } catch (e) {
          console.error("Failed to parse message", e);
        }
      };
    }

    manualConnectRef.current = connect;
    connect();

    return () => {
      disposed = true;
      clearReconnectTimer();
      clearConnectTimeout();
      clearStateWatchdog();
      if (wsRef.current) {
        const active = wsRef.current;
        wsRef.current = null;
        setConnected(false);
        active.onopen = null;
        active.onclose = null;
        active.onerror = null;
        active.onmessage = null;
        active.close();
      }
    };
  }, [url, autoReconnect, password, reconnectIntervalMs, connectTimeoutMs]);

  const manualConnect = () => {
    if (wsRef.current && wsRef.current.readyState === WebSocket.OPEN) return;
    // cancel pending auto-reconnect, then connect immediately
    if (reconnectTimerRef.current !== null) {
      clearTimeout(reconnectTimerRef.current);
      reconnectTimerRef.current = null;
    }
    manualConnectRef.current?.();
  };

  return {
    connected,
    state,
    authStatus,
    authRetryAt,
    sendMotorPower,
    sendMotorCommand,
    sendMotorControlRequest,
    sendRobotModeRequest,
    sendSafetyReset,
    sendDataLogger,
    sendRaw,
    retryAuth,
    manualConnect,
  };
}