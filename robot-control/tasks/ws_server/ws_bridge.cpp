#include "ws_bridge.h"
#include <iostream>

namespace wsbridge {

WebsocketBridge::WebsocketBridge(uint16_t port) : port_(port) {
    server_.config.port = port_;
    setup_endpoints();
}

WebsocketBridge::~WebsocketBridge() {
    stop();
}

void WebsocketBridge::setup_endpoints() {
    auto& ep = server_.endpoint["^/.*$"];

    ep.on_open = [this](std::shared_ptr<WsServer::Connection> conn) {
        std::lock_guard<std::mutex> lock(conns_mtx_);
        connections_.insert(conn);
        ClientInfo info;
        info.conn = conn;
        info.mask = 0; // 아직 아무것도 구독 안 함
        client_info_[conn.get()] = info;
        std::cout << "[WS] client connected\n";
    };

    ep.on_close = [this](std::shared_ptr<WsServer::Connection> conn, int, const std::string&) {
        std::lock_guard<std::mutex> lock(conns_mtx_);
        connections_.erase(conn);
        client_info_.erase(conn.get());
        std::cout << "[WS] client disconnected\n";
    };

    ep.on_message = [this](std::shared_ptr<WsServer::Connection> conn,
                           std::shared_ptr<WsServer::InMessage> msg) {
        try {
            handle_message(conn, msg->string());
        } catch (std::exception& e) {
            std::cerr << "[WS] on_message error: " << e.what() << "\n";
        }
    };
}

void WebsocketBridge::handle_message(std::shared_ptr<WsServer::Connection> conn,
                                     const std::string& txt) {
    auto j = json::parse(txt);
    std::string type = j.value("type", "");
    auto payload = j["payload"];

    if (type == "subscribe_state") {
        int rate = payload.value("rate", 10);
        std::cout << "[WS] subscribe_state: rate=" << rate << "\n";
        // 여기서는 rate는 그냥 로그만 찍고, NonRT에서 상태 주기를 조절할 수도 있음

        std::lock_guard<std::mutex> lock(conns_mtx_);
        auto it = client_info_.find(conn.get());
        if (it != client_info_.end()) {
            it->second.mask |= 0x1; // bit0: state
        }
    }
    else if (type == "motor_power") {
        bool on = payload.value("on", true);
        bool is_all = false;
        int id = -1;

        if (payload["motorId"].is_string() &&
            payload["motorId"].get<std::string>() == "all") {
            is_all = true;
        } else {
            id = payload["motorId"].get<int>();
        }

        if (motor_power_handler_) {
            motor_power_handler_(id, on, is_all);
        }
    }
    else if (type == "motor_command") {
        bool is_all = false;
        int id = -1;
        if (payload["motorId"].is_string() &&
            payload["motorId"].get<std::string>() == "all") {
            is_all = true;
        } else {
            id = payload["motorId"].get<int>();
        }

        MotorCommand cmd;
        auto c = payload["command"];
        auto get_opt = [&](const char* key) -> std::optional<double> {
            if (!c.contains(key) || c[key].is_null())
                return std::nullopt;
            return c[key].get<double>();
        };

        cmd.position = get_opt("position");
        cmd.velocity = get_opt("velocity");
        cmd.torque   = get_opt("torque");
        cmd.kp       = get_opt("kp");
        cmd.kd       = get_opt("kd");

        if (motor_cmd_handler_) {
            motor_cmd_handler_(id, is_all, cmd);
        }
    }
    else {
        std::cout << "[WS] unknown message type: " << type << "\n";
    }
}

void WebsocketBridge::start() {
    if (running_.exchange(true)) return; // 이미 실행중

    server_thread_ = std::thread([this]() {
        try {
            std::cout << "[WS] server starting at port " << port_ << "\n";
            server_.start();
            std::cout << "[WS] server stopped\n";
        } catch (std::exception& e) {
            std::cerr << "[WS] server error: " << e.what() << "\n";
        }
    });
}

void WebsocketBridge::stop() {
    if (!running_.exchange(false)) return;

    server_.stop();
    if (server_thread_.joinable())
        server_thread_.join();
}

void WebsocketBridge::broadcast_state() {
    if (!state_provider_) return;

    BridgeState s = state_provider_(); // NonRT에서 제공하는 현재 상태
    json payload = s;
    json msg = {
        {"type", "state"},
        {"timestamp",
         std::chrono::duration_cast<std::chrono::milliseconds>(
             std::chrono::system_clock::now().time_since_epoch()).count()},
        {"payload", payload}
    };
    std::string txt = msg.dump();

    std::lock_guard<std::mutex> lock(conns_mtx_);
    for (auto& conn : connections_) {
        auto it = client_info_.find(conn.get());
        if (it == client_info_.end()) continue;

        if (it->second.mask & 0x1) { // state 구독한 클라이언트만
            conn->send(txt);
        }
    }
}

} // namespace wsbridge