#pragma once
#include <vector>
#include <string>
#include <optional>
#include <deque>
#include <mutex>
#include <atomic>
#include <thread>
#include <set>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <memory>

#include <App.h>
#include <nlohmann/json.hpp>

namespace wsbridge {

using json = nlohmann::json;

// ======================
//  상태 스냅샷 타입들
// ======================

struct MotorSnapshot {
    int id;
    std::string name;
    std::string mode;

    double position;
    double velocity;
    double torque;
    double temperature;

    bool error;
    bool warning;

    double command_position;
    double command_velocity;
    double command_torque;
    double command_kp;
    double command_kd;

    double driver_command_position;
    double driver_command_velocity;
    double driver_command_torque;
    double driver_command_kp;
    double driver_command_kd;

    double kp;
    double kd;

    bool enabled;
};

struct Vec3 {
    double x, y, z;
};

struct ImuSnapshot {
    Vec3 rpy;   // roll, pitch, yaw
    Vec3 gyro;  // angular velocity
    Vec3 accel; // linear acceleration
};

struct BridgeState {
    struct ControlSnapshot {
        bool requested = false;
        bool granted = false;
    } control;

    struct RobotModeSnapshot {
        std::string current = "IDLE";
        bool walk_ready = false;
    } robot_mode;

    struct SafetySnapshot {
        std::string level = "ESSENTIAL";
        bool locked = false;
        bool restoring = false;
    } safety;

    struct DataLoggerSnapshot {
        bool recording    = false;
        int  sample_count = 0;
        std::string filename;
    } data_logger;

    std::vector<MotorSnapshot> motors;
    std::vector<MotorSnapshot> physical_motors; // 7개 물리 모터 raw 상태
    ImuSnapshot imu;
    std::unordered_map<std::string, double> joint_states;
};

// JSON 직렬화 (BridgeState -> JSON)
inline void to_json(json& j, const MotorSnapshot& m) {
    j = json{
        {"id", m.id},
        {"name", m.name},
        {"mode", m.mode},
        {"position", m.position},
        {"velocity", m.velocity},
        {"torque", m.torque},
        {"temperature", m.temperature},
        {"error", m.error},
        {"warning", m.warning},
        {"command_position", m.command_position},
        {"command_velocity", m.command_velocity},
        {"command_torque", m.command_torque},
        {"command_kp", m.command_kp},
        {"command_kd", m.command_kd},
        {"driver_command_position", m.driver_command_position},
        {"driver_command_velocity", m.driver_command_velocity},
        {"driver_command_torque", m.driver_command_torque},
        {"driver_command_kp", m.driver_command_kp},
        {"driver_command_kd", m.driver_command_kd},
        {"kp", m.kp},
        {"kd", m.kd},
        {"enabled", m.enabled}
    };
}

inline void to_json(json& j, const ImuSnapshot& imu) {
    j = json{
        {"orientation_rpy", {{"roll", imu.rpy.x}, {"pitch", imu.rpy.y}, {"yaw", imu.rpy.z}}},
        {"angular_velocity", {{"x", imu.gyro.x}, {"y", imu.gyro.y}, {"z", imu.gyro.z}}},
        {"linear_acceleration", {{"x", imu.accel.x}, {"y", imu.accel.y}, {"z", imu.accel.z}}}
    };
}

inline void to_json(json& j, const BridgeState& s) {
    j = json{
        {"control", {{"requested", s.control.requested}, {"granted", s.control.granted}}},
        {"robot_mode", {{"current", s.robot_mode.current}, {"walk_ready", s.robot_mode.walk_ready}}},
        {"safety", {{"level", s.safety.level}, {"locked", s.safety.locked}, {"restoring", s.safety.restoring}}},
        {"data_logger", {
            {"recording",    s.data_logger.recording},
            {"sample_count", s.data_logger.sample_count},
            {"filename",     s.data_logger.filename}
        }},
        {"motors", s.motors},
        {"physical_motors", s.physical_motors},
        {"imu", s.imu},
        {"joint_states", s.joint_states}
    };
}

// ======================
//  GUI -> 로봇 이벤트
// ======================

struct MotorCommand {
    std::optional<double> position;
    std::optional<double> velocity;
    std::optional<double> torque;
    std::optional<double> kp;
    std::optional<double> kd;
    double duration_ms = 0.0;  // interpolation duration (ms), 0 = immediate
};

struct Event {
    struct SubscribePayload {
        int rate_hz = 10;
    };

    struct PowerPayload {
        bool is_all = false;
        int motor_id = -1;
        bool on = true;
    };

    struct CommandPayload {
        bool is_all = false;
        int motor_id = -1;
        MotorCommand cmd;
    };

    struct ControlRequestPayload {
        bool request = true;
    };

    struct RobotModeRequestPayload {
        std::string mode;
    };

    struct SafetyResetPayload {
        bool request = true;
    };

    struct DataLoggerPayload {
        bool start = false;
    };

    enum class Kind {
        SubscribeState,
        MotorPower,
        MotorCommand,
        MotorControlRequest,
        RobotModeRequest,
        SafetyReset,
        DataLogger
    } kind;

    std::int64_t timestamp_ms = 0;

    SubscribePayload subscribe;
    PowerPayload power;
    CommandPayload command;
    ControlRequestPayload control_request;
    RobotModeRequestPayload robot_mode_request;
    SafetyResetPayload safety_reset;
    DataLoggerPayload data_logger;
};

} // namespace wsbridge


// ws_bridge_uws.hpp 계속

namespace wsbridge {

class WebsocketBridge {
public:
    explicit WebsocketBridge(uint16_t port = 8080)
        : port_(port) {}

    ~WebsocketBridge() {
        stop();
    }

    // uWS 서버 스레드 시작
    void start() {
        if (running_.exchange(true)) return;

        server_thread_ = std::thread([this]() {
            run_server();
        });
    }

    void stop() {
        if (!running_.exchange(false)) return;

        // uWS loop에 stop 요청
        if (loop_) {
            loop_->defer([this]() {
                if (app_) {
                    app_->close();
                }
            });
        }

        if (server_thread_.joinable())
            server_thread_.join();
    }

    // NonRT에서 현재 상태를 받아서 GUI로 브로드캐스트
    void publish_state(const BridgeState& state) {
        if (!running_) return;
        if (state_subscriber_count_.load() <= 0) return;
        if (!loop_) return;

        json payload = state;
        json msg = {
            {"type", "state"},
            {"timestamp",
             std::chrono::duration_cast<std::chrono::milliseconds>(
                 std::chrono::system_clock::now().time_since_epoch())
                 .count()},
            {"payload", payload}
        };

        auto msg_ptr = std::make_shared<std::string>(msg.dump());

        // uWS 이벤트 루프 스레드에서만 send 호출하도록 defer
        loop_->defer([this, msg_ptr]() {
            for (auto* ws : connections_) {
                ws->send(*msg_ptr, uWS::OpCode::TEXT);
            }
        });
    }

    // NonRT에서 이벤트 polling
    bool try_pop_event(Event& out) {
        if (event_count_.load() <= 0)
            return false;

        std::lock_guard<std::mutex> lock(event_mtx_);
        if (event_queue_.empty())
            return false;

        out = std::move(event_queue_.front());
        event_queue_.pop_front();
        event_count_.fetch_sub(1);
        return true;
    }

    bool has_state_subscriber() const {
        return state_subscriber_count_.load() > 0;
    }

    void set_password(const std::string& password) {
        if (running_.load()) {
            std::cout << "[WS] set_password ignored while server is running\n";
            return;
        }

        password_ = password;
        auth_required_ = !password_.empty();

        if (auth_required_) {
            std::cout << "[WS] password auth enabled\n";
        } else {
            std::cout << "[WS] password auth disabled (empty password)\n";
        }
    }

private:
    struct PerSocketData {
        uint32_t mask = 0; // bit0: state subscribed
        bool authenticated = false;
        int auth_fail_count = 0;
    };

    uint16_t port_;
    std::thread server_thread_;
    std::atomic<bool> running_{false};

    // uWS 관련
    std::unique_ptr<uWS::App> app_;
    uWS::Loop* loop_ = nullptr;

    // 연결 관리 (uWS 스레드에서만 접근)
    std::set<uWS::WebSocket<false, true, PerSocketData>*> connections_;

    // 이벤트 큐 (uWS 스레드 → NonRT 스레드)
    std::deque<Event> event_queue_;
    std::mutex event_mtx_;
    std::atomic<int> event_count_{0};

    // 구독자 수
    std::atomic<int> state_subscriber_count_{0};

    std::string password_;
    bool auth_required_ = false;

    static std::int64_t now_ms() {
        return std::chrono::duration_cast<std::chrono::milliseconds>(
                   std::chrono::system_clock::now().time_since_epoch())
            .count();
    }

    void run_server() {
        // 이 스레드에서 uWS::App 생성 & 실행
        app_ = std::make_unique<uWS::App>();

        app_->ws<PerSocketData>("/*", {
            .compression = uWS::DISABLED,
            .maxPayloadLength = 16 * 1024,
            .idleTimeout = 10,           // 10초 동안 PONG 없으면 연결 강제 종료 (uWS 제약: 0 또는 >=9)
            .maxBackpressure = 1 * 1024 * 1024,
            .sendPingsAutomatically = true, // 브라우저에 WebSocket PING 자동 전송
            .upgrade = nullptr,
            .open = [this](auto* ws) {
                // 첫 연결
                connections_.insert(ws);
                ws->getUserData()->mask = 0;
                ws->getUserData()->authenticated = !auth_required_;
                ws->getUserData()->auth_fail_count = 0;

                if (auth_required_) {
                    json auth_required_msg = {
                        {"type", "auth_required"},
                        {"timestamp", now_ms()},
                        {"payload", {{"required", true}}}
                    };
                    ws->send(auth_required_msg.dump(), uWS::OpCode::TEXT);
                }
                std::cout << "[WS] client connected\n";
            },
            .message = [this](auto* ws, std::string_view msg, uWS::OpCode op) {
                if (op != uWS::OpCode::TEXT) return;
                handle_message(ws, msg);
            },
            .close = [this](auto* ws, int /*code*/, std::string_view /*msg*/) {
                auto it = connections_.find(ws);
                bool became_empty = false;
                if (it != connections_.end()) {
                    // 구독 중이던 상태를 정리
                    auto* data = ws->getUserData();
                    if (data->mask & 0x1) {
                        state_subscriber_count_.fetch_sub(1);
                    }
                    connections_.erase(it);
                    became_empty = connections_.empty();
                }

                if (became_empty) {
                    Event release_ev;
                    release_ev.kind = Event::Kind::MotorControlRequest;
                    release_ev.timestamp_ms = now_ms();
                    release_ev.control_request.request = false;

                    std::lock_guard<std::mutex> lock(event_mtx_);
                    event_queue_.push_back(std::move(release_ev));
                    event_count_.fetch_add(1);
                }
                std::cout << "[WS] client disconnected\n";
            }
        });

        static constexpr const char* kBindHost = "0.0.0.0";
        app_->listen(std::string{kBindHost}, port_, [this](auto* token) {
            if (token) {
                std::cout << "[WS] listening on " << kBindHost << ":" << port_ << "\n";
            } else {
                std::cerr << "[WS] failed to listen on " << kBindHost << ":" << port_ << "\n";
            }
        });

        // 루프 포인터 저장
        loop_ = uWS::Loop::get();

        // run()은 이 스레드를 막음
        app_->run();

        std::cout << "[WS] server loop exited\n";

        // app_은 이 스레드(uWS 루프 스레드)에서 생성됐으므로
        // 반드시 같은 스레드에서 소멸시켜야 한다.
        // 메인 스레드에서 소멸하면 스레드-로컬 루프가 이미 사라진 후라
        // uWS 내부에서 dangling 포인터 접근 → SIGSEGV 발생.
        app_.reset();
        loop_ = nullptr;
    }

    void handle_message(uWS::WebSocket<false, true, PerSocketData>* ws,
                        std::string_view txt) {
        try {
            json j = json::parse(txt);
            std::string type = j.value("type", "");

            auto* data = ws->getUserData();

            if (type == "auth") {
                if (!j.contains("payload") || !j["payload"].is_object()) {
                    std::cout << "[WS] auth payload missing\n";
                    return;
                }

                const auto& payload = j["payload"];
                const std::string password = payload.value("password", std::string(""));
                const bool ok = !auth_required_ || (!password_.empty() && password == password_);

                if (ok) {
                    data->authenticated = true;
                    data->auth_fail_count = 0;
                    json auth_ok_msg = {
                        {"type", "auth_ok"},
                        {"timestamp", now_ms()},
                        {"payload", {{"authenticated", true}}}
                    };
                    ws->send(auth_ok_msg.dump(), uWS::OpCode::TEXT);
                } else {
                    data->authenticated = false;
                    data->auth_fail_count += 1;
                    json auth_fail_msg = {
                        {"type", "auth_failed"},
                        {"timestamp", now_ms()},
                        {"payload", {{"reason", "invalid_password"}}}
                    };
                    ws->send(auth_fail_msg.dump(), uWS::OpCode::TEXT);
                    if (data->auth_fail_count >= 3) {
                        ws->close();
                    }
                }
                return;
            }

            if (auth_required_ && !data->authenticated) {
                json auth_required_msg = {
                    {"type", "auth_required"},
                    {"timestamp", now_ms()},
                    {"payload", {{"required", true}}}
                };
                ws->send(auth_required_msg.dump(), uWS::OpCode::TEXT);
                return;
            }

            if (!j.contains("payload") || !j["payload"].is_object()) {
                std::cout << "[WS] invalid payload format\n";
                return;
            }
            const auto& payload = j["payload"];

            Event ev;
            ev.timestamp_ms = now_ms();

            if (type == "subscribe_state") {
                ev.kind = Event::Kind::SubscribeState;
                ev.subscribe.rate_hz = payload.value("rate", 10);

                // 구독 마스크 설정
                auto* data = ws->getUserData();
                if (!(data->mask & 0x1)) {
                    data->mask |= 0x1;
                    state_subscriber_count_.fetch_add(1);
                }
            }
            else if (type == "motor_power") {
                ev.kind = Event::Kind::MotorPower;
                ev.power.on = payload.value("on", true);

                if (!payload.contains("motorId")) {
                    std::cout << "[WS] motor_power missing motorId\n";
                    return;
                }

                if (payload["motorId"].is_string() &&
                    payload["motorId"].get<std::string>() == "all") {
                    ev.power.is_all = true;
                } else {
                    ev.power.is_all = false;
                    ev.power.motor_id = payload["motorId"].get<int>();
                }
            }
            else if (type == "motor_command") {
                ev.kind = Event::Kind::MotorCommand;

                if (!payload.contains("motorId") || !payload.contains("command") || !payload["command"].is_object()) {
                    std::cout << "[WS] motor_command missing fields\n";
                    return;
                }

                if (payload["motorId"].is_string() &&
                    payload["motorId"].get<std::string>() == "all") {
                    ev.command.is_all = true;
                } else {
                    ev.command.is_all = false;
                    ev.command.motor_id = payload["motorId"].get<int>();
                }

                const auto& c = payload["command"];
                auto get_opt = [&](const char* key) -> std::optional<double> {
                    if (!c.contains(key) || c[key].is_null())
                        return std::nullopt;
                    return c[key].get<double>();
                };

                ev.command.cmd.position    = get_opt("position");
                ev.command.cmd.velocity    = get_opt("velocity");
                ev.command.cmd.torque      = get_opt("torque");
                ev.command.cmd.kp          = get_opt("kp");
                ev.command.cmd.kd          = get_opt("kd");
                ev.command.cmd.duration_ms = c.value("duration_ms", 0.0);
            }
            else if (type == "motor_control_request") {
                ev.kind = Event::Kind::MotorControlRequest;
                ev.control_request.request = payload.value("request", true);
            }
            else if (type == "robot_mode_request") {
                ev.kind = Event::Kind::RobotModeRequest;
                ev.robot_mode_request.mode = payload.value("mode", std::string(""));
                if (ev.robot_mode_request.mode.empty()) {
                    std::cout << "[WS] robot_mode_request missing mode\n";
                    return;
                }
            }
            else if (type == "safety_reset") {
                ev.kind = Event::Kind::SafetyReset;
                ev.safety_reset.request = payload.value("request", true);
            }
            else if (type == "data_logger") {
                ev.kind = Event::Kind::DataLogger;
                ev.data_logger.start = payload.value("start", false);
            }
            else {
                std::cout << "[WS] unknown type: " << type << "\n";
                return;
            }

            // 이벤트 큐에 push (uWS 스레드 → NonRT 스레드)
            {
                std::lock_guard<std::mutex> lock(event_mtx_);
                event_queue_.push_back(std::move(ev));
                event_count_.fetch_add(1);
            }
        } catch (std::exception& e) {
            std::cerr << "[WS] parse error: " << e.what() << "\n";
        }
    }
};

} // namespace wsbridge