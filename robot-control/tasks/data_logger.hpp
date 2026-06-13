#pragma once

// DataLogger — 매니퓰레이터 조인트 데이터 CSV 저장 (Non-RT 태스크)
//
// 기록 채널:
//   joint*/state        — 실제 위치/속도/토크 (MuJoCo or 하드웨어 피드백)
//   joint*/cmd          — 세이프티 레이어 통과 후 명령 (kp, kd 포함)
//   manager/joint*/cmd  — 사용자 명령 (세이프티 레이어 이전)
//
// 기록 제어:  data_logger/record_cmd (bool) — true=시작, false=종료
// 상태 출력:  data_logger/info (LoggerInfo) — GUI용 상태 스냅샷

#include <rtfw/task.h>
#include "custom_types.hpp"

#include <array>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <fstream>
#include <sys/stat.h>
#include <sys/types.h>

using namespace rtfw;
using namespace rtfw::rt;

namespace task_pool {

static constexpr int LOG_N = 5;

// State는 프레임워크가 관리하므로 POD-like 타입만 포함 (std::ofstream 제외)
struct DataLoggerState {
    std::array<custom_types::MotorState, LOG_N> state{};
    std::array<custom_types::MotorCmd,   LOG_N> cmd{};
    std::array<custom_types::MotorCmd,   LOG_N> mgr_cmd{};
    bool   recording    = false;
    int    sample_count = 0;
    double start_tick   = 0.0;
    char   filename[256] = {};
};

class DataLogger : public Task<DataLoggerState> {
public:
    const char* getName() const override { return "DataLogger"; }

    void initialize(DataLoggerState& s) override {
        s.recording    = false;
        s.sample_count = 0;
        publish_info(s);
    }

    void execute(DataLoggerState& s) override {
        // 최신 상태 캐시
        for (int i = 0; i < LOG_N; i++) {
            dr_state_[i].on_update([&, i](const custom_types::MotorState& d) { s.state[i] = d; });
            dr_cmd_[i].on_update([&, i](const custom_types::MotorCmd& d)     { s.cmd[i]   = d; });
            dr_mgr_cmd_[i].on_update([&, i](const custom_types::MotorCmd& d) { s.mgr_cmd[i] = d; });
        }

        // 기록 시작/종료 명령
        dr_record_cmd_.on_update([&](const bool& v) {
            if (v && !s.recording)      start_recording(s);
            else if (!v && s.recording) stop_recording(s);
        });

        if (!s.recording) return;

        // CSV 행 작성
        double t = static_cast<double>(getExecutionLocalTick()) / getFrequency() - s.start_tick;
        char buf[1024];
        int n = std::snprintf(buf, sizeof(buf), "%.6f", t);

        for (int i = 0; i < LOG_N; i++)
            n += std::snprintf(buf + n, sizeof(buf) - n,
                ",%.6f,%.6f,%.6f", s.state[i].pos, s.state[i].vel, s.state[i].torque);

        for (int i = 0; i < LOG_N; i++)
            n += std::snprintf(buf + n, sizeof(buf) - n,
                ",%.6f,%.6f,%.6f,%.4f,%.4f",
                s.cmd[i].pos, s.cmd[i].vel, s.cmd[i].torque, s.cmd[i].kp, s.cmd[i].kd);

        for (int i = 0; i < LOG_N; i++)
            n += std::snprintf(buf + n, sizeof(buf) - n,
                ",%.6f,%.6f,%.6f,%.4f,%.4f",
                s.mgr_cmd[i].pos, s.mgr_cmd[i].vel, s.mgr_cmd[i].torque,
                s.mgr_cmd[i].kp,  s.mgr_cmd[i].kd);

        buf[n++] = '\n';
        file_.write(buf, n);
        s.sample_count++;

        if (s.sample_count % 50 == 0)
            publish_info(s);
    }

private:
    // ── std::ofstream은 state가 아닌 태스크 멤버로 ──
    std::ofstream file_;

    // ── 입력 채널 ──
    DataReader<bool> dr_record_cmd_{"data_logger/record_cmd", DependencyType::Weak};

    DataReader<custom_types::MotorState> dr_state_[LOG_N] = {
        DataReader<custom_types::MotorState>{"joint0/state", DependencyType::Weak},
        DataReader<custom_types::MotorState>{"joint1/state", DependencyType::Weak},
        DataReader<custom_types::MotorState>{"joint2/state", DependencyType::Weak},
        DataReader<custom_types::MotorState>{"joint3/state", DependencyType::Weak},
        DataReader<custom_types::MotorState>{"joint4/state", DependencyType::Weak},
    };
    DataReader<custom_types::MotorCmd> dr_cmd_[LOG_N] = {
        DataReader<custom_types::MotorCmd>{"joint0/cmd", DependencyType::Weak},
        DataReader<custom_types::MotorCmd>{"joint1/cmd", DependencyType::Weak},
        DataReader<custom_types::MotorCmd>{"joint2/cmd", DependencyType::Weak},
        DataReader<custom_types::MotorCmd>{"joint3/cmd", DependencyType::Weak},
        DataReader<custom_types::MotorCmd>{"joint4/cmd", DependencyType::Weak},
    };
    DataReader<custom_types::MotorCmd> dr_mgr_cmd_[LOG_N] = {
        DataReader<custom_types::MotorCmd>{"manager/joint0/cmd", DependencyType::Weak},
        DataReader<custom_types::MotorCmd>{"manager/joint1/cmd", DependencyType::Weak},
        DataReader<custom_types::MotorCmd>{"manager/joint2/cmd", DependencyType::Weak},
        DataReader<custom_types::MotorCmd>{"manager/joint3/cmd", DependencyType::Weak},
        DataReader<custom_types::MotorCmd>{"manager/joint4/cmd", DependencyType::Weak},
    };

    // ── 출력 채널 ──
    DataWriter<custom_types::LoggerInfo> dw_info_{"data_logger/info"};

    // ── 내부 유틸리티 ──
    void publish_info(DataLoggerState& s) {
        custom_types::LoggerInfo info;
        info.recording    = s.recording;
        info.sample_count = s.sample_count;
        std::strncpy(info.filename, s.filename, sizeof(info.filename) - 1);
        info.filename[sizeof(info.filename) - 1] = '\0';
        dw_info_.write(info);
    }

    static std::string make_filename() {
        auto t = std::time(nullptr);
        struct tm tm_buf{};
        localtime_r(&t, &tm_buf);
        char buf[64];
        std::strftime(buf, sizeof(buf), "logs/joint_log_%Y%m%d_%H%M%S.csv", &tm_buf);
        return std::string(buf);
    }

    void start_recording(DataLoggerState& s) {
        ::mkdir("logs", 0755);
        std::string fname = make_filename();
        std::strncpy(s.filename, fname.c_str(), sizeof(s.filename) - 1);
        s.filename[sizeof(s.filename) - 1] = '\0';

        file_.open(fname, std::ios::out | std::ios::trunc);
        if (!file_.is_open()) {
            getLogger()->error("[{}] Cannot open file: {}", getName(), fname);
            return;
        }

        // CSV 헤더
        file_ << "time_s";
        for (int i = 0; i < LOG_N; i++)
            file_ << ",j" << i << "_pos,j" << i << "_vel,j" << i << "_torque";
        for (int i = 0; i < LOG_N; i++)
            file_ << ",j" << i << "_cmd_pos,j" << i << "_cmd_vel"
                  << ",j" << i << "_cmd_torque,j" << i << "_cmd_kp,j" << i << "_cmd_kd";
        for (int i = 0; i < LOG_N; i++)
            file_ << ",j" << i << "_mgr_pos,j" << i << "_mgr_vel"
                  << ",j" << i << "_mgr_torque,j" << i << "_mgr_kp,j" << i << "_mgr_kd";
        file_ << "\n";

        s.recording    = true;
        s.sample_count = 0;
        s.start_tick   = static_cast<double>(getExecutionLocalTick()) / getFrequency();

        publish_info(s);
        getLogger()->info("[{}] Recording started → {}", getName(), fname);
    }

    void stop_recording(DataLoggerState& s) {
        file_.flush();
        file_.close();
        s.recording = false;

        publish_info(s);
        getLogger()->info("[{}] Recording stopped: {} samples → {}",
                          getName(), s.sample_count, s.filename);
    }
};

} // namespace task_pool
