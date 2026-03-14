#pragma once

#include <rtfw/task.h>
#include "../util.hpp"
#include "../custom_types.hpp"
#include "motor.impl.hpp"
#include "../kin_2rsu.hpp"

#include <chrono>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/can/error.h>
#include <inttypes.h>
#include <cmath>
#include <chrono>
#include <thread>
#include <vector>
#include <memory>
#include <csignal>
#include <fcntl.h>
#include <arpa/inet.h> // ntohs 사용을 위해 추가
#include <errno.h>
#include <sys/wait.h>




using namespace std::chrono_literals;
using namespace rtfw;
using namespace rtfw::rt;


namespace task_pool {


    class CanBus1 : public ITask {
    public:
        const char* getName() const override { return "CanBus1"; }

        void initialize(void*) override {
            for (auto& flag: on_flag) flag = false;
            for (auto& s: motor_sync_) s = false;
            for (auto& p: joint_feedback_pos_) p = 0.0;

            for (auto& cmd: cmds) {
                cmd.kp = 0;
                cmd.kd = 0;
                cmd.pos = 0;
                cmd.vel = 0;
                cmd.torque = 0;
            }
            for (auto& interp: cmd_interp) interp = {};
        }

        void execute(void*) override {
            std::array<bool, 6> cmd_updated;
            cmd_updated.fill(false);
            for (int i=0; i<cmds.size(); i++) {
                dr_mtr_cmd[i].on_update([&, i](const custom_types::MotorCmd& data) {
                    cmd_wdt[i] = getExecutionLocalTick();
                    if (data.duration_ms > 0.0) {
                        // 새 타겟이거나 처음 진입할 때만 interpolation 시작
                        if (!cmd_interp[i].active || std::abs(data.pos - cmd_interp[i].target_pos) > 1e-4) {
                            cmd_interp[i].start_ref = joint_feedback_pos_[i]; // 실제 피드백(발목은 FK 변환값)에서 시작
                            cmd_interp[i].target_pos = data.pos;
                            cmd_interp[i].duration_ms = data.duration_ms;
                            cmd_interp[i].elapsed_ms = 0.0;
                            cmd_interp[i].active = true;
                        }
                        // 같은 타겟이면 계속 진행
                    } else {
                        // 즉시 명령: interpolation 취소, ref 즉시 갱신
                        cmd_interp[i].active = false;
                        cmds[i].pos = data.pos;
                    }
                    cmds[i].vel = data.vel;
                    cmds[i].torque = data.torque;
                    cmds[i].kp = data.kp;
                    cmds[i].kd = data.kd;
                    cmd_updated[i] = true;
                });

                dr_motor_on[i].on_update([&, i](const bool& on) {
                    if (!on_flag[i] && on) {
                        getLogger()->info("[{}] Motor {} ON", getName(), i);
                        on_flag[i] = on;
                        motor_sync_[i] = false; // Start 후 피드백 수신 시 동기화
                        cmd_interp[i] = {};
                        if (so > 0) motors[i]->Start(so);
                    }
                    if (on_flag[i] && !on) {
                        getLogger()->info("[{}] Motor {} OFF", getName(), i);
                        on_flag[i] = on;
                        motor_sync_[i] = false;
                        cmd_interp[i] = {};
                        if (so > 0) motors[i]->Stop(so);
                    }
                });
            }

            // if (cmd_updated[4] || cmd_updated[5]) {
            //     auto theta = kin_2rsu::ankle_ik(cmds[4].pos, cmds[5].pos, 'l');
            //     cmds[4].pos = theta(0);
            //     cmds[5].pos = theta(1);
            // }

            if (so > 0) {
                struct can_frame rx;
                while (read(so, &rx, sizeof(rx)) > 0) {
                    // 에러 프레임 감지 (bus-off, 버스 에러)
                    if (rx.can_id & CAN_ERR_FLAG) {
                        if (rx.can_id & CAN_ERR_BUSOFF) {
                            getLogger()->error("[{}] CAN bus-off detected! Trying controller restart...", getName());
                            restart_required = true;
                        }
                        else if (rx.can_id & CAN_ERR_BUSERROR) {
                            getLogger()->warn("[{}] CAN bus error detected", getName());
                        }
                        can_close();
                        break;
                    }
                    for (auto i=0; i<motors.size(); i++) {
                        if (motors[i]->isMyFrame(&rx)) {
                            if (motors[i]->parseFeedback(&rx)) {
                                com_wdt[i] = getExecutionLocalTick();

                                // 동기화 미완료 상태: 피드백 위치로 동기화 후 제어 허용
                                if (!motor_sync_[i]) {
                                    if (i < 4) {
                                        joint_feedback_pos_[i] = motors[i]->state.pos;
                                        cmds[i].pos = joint_feedback_pos_[i];
                                    }
                                    cmd_interp[i] = {};
                                    motor_sync_[i] = true;
                                    getLogger()->info("[{}] Motor {} synced at pos={:.3f}", getName(), i, motors[i]->state.pos);
                                }

                                custom_types::MotorState state{};
                                state.pos = motors[i]->state.pos;
                                state.vel = motors[i]->state.vel;
                                state.torque = motors[i]->state.torque;
                                state.status = motors[i]->state.status;
                                state.enabled = on_flag[i];
                                dw_mtr_stat[i].write(state);
                            }
                            break;
                        }
                    }
                }

                const auto ankle_fb = map_theta_to_rp(
                    static_cast<float>(motors[4]->state.pos),
                    static_cast<float>(motors[5]->state.pos),
                    static_cast<float>(motors[4]->state.vel),
                    static_cast<float>(motors[5]->state.vel),
                    static_cast<float>(motors[4]->state.torque),
                    static_cast<float>(motors[5]->state.torque),
                    'r');
                joint_feedback_pos_[4] = ankle_fb.rp(0);
                joint_feedback_pos_[5] = ankle_fb.rp(1);

                if (motor_sync_[4] && motor_sync_[5]) {
                    cmds[4].pos = joint_feedback_pos_[4];
                    cmds[5].pos = joint_feedback_pos_[5];
                }

                // 가상 pitch/roll 상태 발행 (실모터 upper/lower 피드백 -> FK)
                custom_types::MotorState ankle_pitch_state{};
                ankle_pitch_state.pos = ankle_fb.rp(0);
                ankle_pitch_state.vel = ankle_fb.rp_vel(0);
                ankle_pitch_state.torque = ankle_fb.rp_tau(0);
                ankle_pitch_state.status = (motors[4]->state.status || motors[5]->state.status);
                ankle_pitch_state.enabled = on_flag[4] && on_flag[5];
                dw_mtr_stat[6].write(ankle_pitch_state);

                custom_types::MotorState ankle_roll_state{};
                ankle_roll_state.pos = ankle_fb.rp(1);
                ankle_roll_state.vel = ankle_fb.rp_vel(1);
                ankle_roll_state.torque = ankle_fb.rp_tau(1);
                ankle_roll_state.status = (motors[4]->state.status || motors[5]->state.status);
                ankle_roll_state.enabled = on_flag[4] && on_flag[5];
                dw_mtr_stat[7].write(ankle_roll_state);

                // 2. 모터단 interpolation 진행 및 제어 명령 송신
                const double dt_ms = 1000.0 / getFrequency();
                std::array<MotorCommand, 6> motor_cmds = cmds;
                std::array<MotorCommand, 6> applied_motor_cmds{};
                std::array<bool, 6> motor_ready{};
                int offline = 0;
                auto tick = getExecutionLocalTick();
                for (auto i=0; i<motors.size(); i++) {
                    if (tick - com_wdt[i] > 1 * getFrequency()) {
                        motors[i]->state.online = false;
                        offline++;
                        // 피드백 타임아웃: 동기화 해제 및 interpolation 취소
                        if (motor_sync_[i]) {
                            motor_sync_[i] = false;
                            cmd_interp[i] = {};
                            getLogger()->warn("[{}] Motor {} feedback timeout, re-sync required", getName(), i);
                        }
                    }

                    // interpolation 진행: cmds[i].pos를 현재 ref로 갱신
                    if (cmd_interp[i].active) {
                        cmd_interp[i].elapsed_ms += dt_ms;
                        const double ratio = std::min(cmd_interp[i].elapsed_ms / cmd_interp[i].duration_ms, 1.0);
                        cmds[i].pos = cmd_interp[i].start_ref + ratio * (cmd_interp[i].target_pos - cmd_interp[i].start_ref);
                        if (ratio >= 1.0) {
                            cmd_interp[i].active = false;
                        }
                    }
                }

                // pitch/roll 명령 -> upper/lower 모터 명령 변환 (IK)
                const auto ankle_cmd = map_rp_to_theta(
                    static_cast<float>(cmds[4].pos),
                    static_cast<float>(cmds[5].pos),
                    static_cast<float>(cmds[4].vel),
                    static_cast<float>(cmds[5].vel),
                    static_cast<float>(cmds[4].torque),
                    static_cast<float>(cmds[5].torque),
                    'r');
                motor_cmds[4].pos = ankle_cmd.theta(0);
                motor_cmds[5].pos = ankle_cmd.theta(1);
                motor_cmds[4].vel = ankle_cmd.theta_vel(0);
                motor_cmds[5].vel = ankle_cmd.theta_vel(1);
                motor_cmds[4].torque = ankle_cmd.theta_tau(0);
                motor_cmds[5].torque = ankle_cmd.theta_tau(1);

                for (auto i=0; i<motors.size(); i++) {
                    const bool motor_on = on_flag[i];
                    const bool cmd_alive = (tick - cmd_wdt[i] <= 1 * getFrequency());
                    const bool ready = motor_on && motor_sync_[i] && cmd_alive;
                    motor_ready[i] = ready;

                    const MotorCommand applied_cmd = ready ? motor_cmds[i] : MotorCommand{};
                    applied_motor_cmds[i] = applied_cmd;
                    custom_types::MotorCmd applied{};
                    applied.pos = applied_cmd.pos;
                    applied.vel = applied_cmd.vel;
                    applied.torque = applied_cmd.torque;
                    applied.kp = applied_cmd.kp;
                    applied.kd = applied_cmd.kd;
                    dw_mtr_cmd_applied[i].write(applied);

                    if (ready) {
                        motors[i]->Control(so, motor_cmds[i]);
                    } else {
                        MotorCommand zero_cmd{};
                        motors[i]->Control(so, zero_cmd);
                    }
                }

                // 가상 pitch/roll cmd_applied 발행 (실적용 upper/lower -> FK)
                custom_types::MotorCmd applied_pitch{};
                custom_types::MotorCmd applied_roll{};
                if (motor_ready[4] && motor_ready[5]) {
                    const auto ankle_applied = map_theta_to_rp(
                        static_cast<float>(applied_motor_cmds[4].pos),
                        static_cast<float>(applied_motor_cmds[5].pos),
                        static_cast<float>(applied_motor_cmds[4].vel),
                        static_cast<float>(applied_motor_cmds[5].vel),
                        static_cast<float>(applied_motor_cmds[4].torque),
                        static_cast<float>(applied_motor_cmds[5].torque),
                        'r');
                    applied_pitch.pos = ankle_applied.rp(0);
                    applied_roll.pos = ankle_applied.rp(1);
                    applied_pitch.vel = ankle_applied.rp_vel(0);
                    applied_roll.vel = ankle_applied.rp_vel(1);
                    applied_pitch.torque = ankle_applied.rp_tau(0);
                    applied_roll.torque = ankle_applied.rp_tau(1);
                    applied_pitch.kp = cmds[4].kp;
                    applied_roll.kp = cmds[5].kp;
                    applied_pitch.kd = cmds[4].kd;
                    applied_roll.kd = cmds[5].kd;
                }
                dw_mtr_cmd_applied[6].write(applied_pitch);
                dw_mtr_cmd_applied[7].write(applied_roll);

                PERIODIC_CALL(
                    for (auto i=0; i<motors.size(); i++) {
                        if (!motors[i]->state.online)
                            getLogger()->warn("[{}] Motor {} seems offline. Sending start command", getName(), motors[i]->id);
                    }
                , 1s);


                if (offline < cmds.size()) {
                    can_wdt = getExecutionLocalTick();
                }
                if (getExecutionLocalTick() - can_wdt > 3 * getFrequency()) {
                    PERIODIC_CALL(
                        getLogger()->warn("[{}] All motors seem offline. Check connections!", getName());
                    , 1s);
                    can_close();
                }
            }
            else {
                if (getExecutionLocalTick() % getFrequency() == 0) {
                    getLogger()->info("[{}] try connection", getName());
                    so = can_open(const_cast<char*>(p_port.read().c_str()));
                    if (so > 0) {
                        can_wdt = getExecutionLocalTick();
                        for (auto& t: cmd_wdt)  t = getExecutionLocalTick();
                        for (auto& t: com_wdt)  t = getExecutionLocalTick();
                        for (auto& mtr: motors) mtr->Start(so, true);
                        for (auto& on: on_flag) on = false;
                    }
                }
            }

            dw_state.write(so > 0);
        }

        bool onOverrun() override {
            // if (getExecutionLocalTick() > 1 * getFrequency()) {
            //     return false;
            // }
            return true;  // 기본: 복구 불가능
        }

        ~CanBus1() {
            if (so > 0) {
                for (auto& motor : motors) {
                    motor->Stop(so);
                }
                can_close();
            }
        }
    private:
        struct AnkleRpMapping {
            Vector2f rp = Vector2f::Zero();
            Matrix2f jac = Matrix2f::Zero();
            Vector2f rp_vel = Vector2f::Zero();
            Vector2f rp_tau = Vector2f::Zero();
        };

        struct AnkleThetaMapping {
            Vector2f theta = Vector2f::Zero();
            Matrix2f jac = Matrix2f::Zero();
            Vector2f theta_vel = Vector2f::Zero();
            Vector2f theta_tau = Vector2f::Zero();
        };

        static inline Matrix2f safe_inverse(const Matrix2f& m) {
            if (std::abs(m.determinant()) > 1e-6f) {
                return m.inverse();
            }
            return Matrix2f::Zero();
        }

        static inline Matrix2f safe_inverse_transpose(const Matrix2f& m) {
            return safe_inverse(m.transpose());
        }

        static inline AnkleRpMapping map_theta_to_rp(
            float theta_upper, float theta_lower,
            float theta_vel_upper, float theta_vel_lower,
            float theta_tau_upper, float theta_tau_lower,
            char side) {
            AnkleRpMapping out;
            std::tie(out.rp, out.jac) = kin_2rsu::ankle_fk(theta_upper, theta_lower, side);
            const Vector2f theta_vel(theta_vel_upper, theta_vel_lower);
            const Vector2f theta_tau(theta_tau_upper, theta_tau_lower);
            out.rp_vel = out.jac * theta_vel;
            out.rp_tau = safe_inverse_transpose(out.jac) * theta_tau;
            return out;
        }

        static inline AnkleThetaMapping map_rp_to_theta(
            float rp_r, float rp_p,
            float rp_vel_r, float rp_vel_p,
            float rp_tau_r, float rp_tau_p,
            char side) {
            AnkleThetaMapping out;
            out.theta = kin_2rsu::ankle_ik(rp_r, rp_p, side);
            out.jac = kin_2rsu::ankle_J(rp_r, rp_p, out.theta(0), out.theta(1), side);
            const Vector2f rp_vel(rp_vel_r, rp_vel_p);
            const Vector2f rp_tau(rp_tau_r, rp_tau_p);
            out.theta_vel = safe_inverse(out.jac) * rp_vel;
            out.theta_tau = out.jac.transpose() * rp_tau;
            return out;
        }

        DataWriter<bool> dw_state{"can1_state", ArchiveOption::Enable};
            
        DataWriter<custom_types::MotorState> dw_mtr_stat[8] = {
            DataWriter<custom_types::MotorState>{"hip_yaw_right/state", ArchiveOption::Enable},
            DataWriter<custom_types::MotorState>{"hip_roll_right/state", ArchiveOption::Enable},
            DataWriter<custom_types::MotorState>{"hip_pitch_right/state", ArchiveOption::Enable},
            DataWriter<custom_types::MotorState>{"knee_right/state", ArchiveOption::Enable},
            DataWriter<custom_types::MotorState>{"ankle_upper_right/state", ArchiveOption::Enable},
            DataWriter<custom_types::MotorState>{"ankle_lower_right/state", ArchiveOption::Enable},
            DataWriter<custom_types::MotorState>{"ankle_pitch_right/state", ArchiveOption::Enable},
            DataWriter<custom_types::MotorState>{"ankle_roll_right/state", ArchiveOption::Enable},
        };

        DataReader<custom_types::MotorCmd> dr_mtr_cmd[6] = {
            DataReader<custom_types::MotorCmd>{"hip_yaw_right/cmd"},
            DataReader<custom_types::MotorCmd>{"hip_roll_right/cmd"},
            DataReader<custom_types::MotorCmd>{"hip_pitch_right/cmd"},
            DataReader<custom_types::MotorCmd>{"knee_right/cmd"},
            DataReader<custom_types::MotorCmd>{"ankle_pitch_right/cmd"},
            DataReader<custom_types::MotorCmd>{"ankle_roll_right/cmd"},
        };
        DataWriter<custom_types::MotorCmd> dw_mtr_cmd_applied[8] = {
            DataWriter<custom_types::MotorCmd>{"hip_yaw_right/cmd_applied", ArchiveOption::Enable},
            DataWriter<custom_types::MotorCmd>{"hip_roll_right/cmd_applied", ArchiveOption::Enable},
            DataWriter<custom_types::MotorCmd>{"hip_pitch_right/cmd_applied", ArchiveOption::Enable},
            DataWriter<custom_types::MotorCmd>{"knee_right/cmd_applied", ArchiveOption::Enable},
            DataWriter<custom_types::MotorCmd>{"ankle_upper_right/cmd_applied", ArchiveOption::Enable},
            DataWriter<custom_types::MotorCmd>{"ankle_lower_right/cmd_applied", ArchiveOption::Enable},
            DataWriter<custom_types::MotorCmd>{"ankle_pitch_right/cmd_applied", ArchiveOption::Enable},
            DataWriter<custom_types::MotorCmd>{"ankle_roll_right/cmd_applied", ArchiveOption::Enable},
        };
        DataReader<bool> dr_motor_on[6] = {
            DataReader<bool>{"hip_yaw_right/on"},
            DataReader<bool>{"hip_roll_right/on"},
            DataReader<bool>{"hip_pitch_right/on"},
            DataReader<bool>{"knee_right/on"},
            DataReader<bool>{"ankle_pitch_right/on"},
            DataReader<bool>{"ankle_roll_right/on"},
        };

        Parameter<std::string> p_port{"can1.port", "can1"};
        

    private:

        bool run_ip_command(const char* port, const char* action, bool log_failure = true) {
            char cmd[256];
            snprintf(cmd, sizeof(cmd), "ip link set dev %s %s", port, action);
            const int ret = system(cmd);
            if (ret == -1) {
                if (log_failure) {
                    getLogger()->error("[{}] Failed to execute '{}'", getName(), cmd);
                }
                return false;
            }
            if (!WIFEXITED(ret) || WEXITSTATUS(ret) != 0) {
                if (log_failure) {
                    getLogger()->error("[{}] Command failed (exit={}): {}", getName(), WIFEXITED(ret) ? WEXITSTATUS(ret) : -1, cmd);
                }
                return false;
            }
            return true;
        }

        bool is_link_up(const char* port) {
            int fd = socket(AF_INET, SOCK_DGRAM, 0);
            if (fd < 0) {
                getLogger()->warn("[{}] Failed to inspect CAN interface {}: {}", getName(), port, strerror(errno));
                return false;
            }

            struct ifreq ifr {};
            strncpy(ifr.ifr_name, port, IFNAMSIZ - 1);
            const bool ok = ioctl(fd, SIOCGIFFLAGS, &ifr) == 0;
            const bool is_up = ok && (ifr.ifr_flags & IFF_UP);
            if (!ok) {
                getLogger()->warn("[{}] Failed to read flags for {}: {}", getName(), port, strerror(errno));
            }

            close(fd);
            return is_up;
        }

        void reset_can_state() {
            can_wdt = 0;
            for (auto& t : cmd_wdt) t = 0;
            for (auto& t : com_wdt) t = 0;
            for (auto& mtr : motors) {
                mtr->state.online = false;
            }
            for (auto& s : motor_sync_) s = false;
            for (auto& interp : cmd_interp) interp = {};
        }

        bool can_ifup(const char* port) {
            reset_can_state();

            const char* config_attempts[] = {
                "up type can bitrate 1000000 restart-ms 100 berr-reporting on",
                "up type can bitrate 1000000 restart-ms 100",
                "up type can bitrate 1000000"
            };

            bool brought_up = false;
            for (const char* action : config_attempts) {
                if (run_ip_command(port, action, false)) {
                    brought_up = true;
                    getLogger()->info("[{}] {} reset and brought up with '{}'", getName(), port, action);
                    break;
                }

                getLogger()->warn("[{}] {} does not accept '{}', trying a simpler CAN setup", getName(), port, action);
            }

            if (!brought_up) {
                getLogger()->error("[{}] Failed to configure {}. Check interface support or permissions.", getName(), port);
                return false;
            }
            return true;
        }

        bool can_restart(const char* port) {
            if (run_ip_command(port, "restart", false)) {
                getLogger()->info("[{}] {} controller restart requested", getName(), port);
                return true;
            }

            getLogger()->warn("[{}] {} does not accept 'restart', falling back to socket reopen only", getName(), port);
            return false;
        }

        int can_open(char* port) {
            if (!link_initialized) {
                if (is_link_up(port)) {
                    link_initialized = true;
                    getLogger()->info("[{}] {} is already up. Skipping CAN reconfiguration.", getName(), port);
                } else {
                    if (!can_ifup(port)) return -1;
                    link_initialized = true;
                }
            } else if (restart_required) {
                can_restart(port);
            }

            int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);

            if (s <= 0) {
                getLogger()->error("[{}] Failed to open CAN interface on {}", getName(), port);
                return s;
            }

            // 에러 프레임 수신 활성화 (bus-off, 버스 에러 감지용)
            can_err_mask_t err_mask = CAN_ERR_TX_TIMEOUT | CAN_ERR_BUSOFF | CAN_ERR_BUSERROR | CAN_ERR_RESTARTED;
            setsockopt(s, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &err_mask, sizeof(err_mask));

            struct ifreq ifr; strcpy(ifr.ifr_name, port);
            if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) {
                getLogger()->error("[{}] Failed to resolve CAN interface {}: {}", getName(), port, strerror(errno));
                close(s);
                return -1;
            }
            struct sockaddr_can addr = {0};
            addr.can_family = AF_CAN; addr.can_ifindex = ifr.ifr_ifindex;
            if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
                getLogger()->error("[{}] Failed to bind CAN interface {}: {}", getName(), port, strerror(errno));
                close(s);
                return -1;
            }

            // 수신 Non-blocking 설정
            int flags = fcntl(s, F_GETFL, 0); fcntl(s, F_SETFL, flags | O_NONBLOCK);

            restart_required = false;
            getLogger()->info("[{}] CanBus connection established on {}", getName(), port);
            return s;
        }

        void can_close() {
            if (so > 0) {
                close(so);
                getLogger()->info("[{}] CanBus connection closed", getName());
            }
            so = -1;
            reset_can_state();
        }

        

    private:
        // 모터단 interpolation 상태 (상위에서 target+duration을 받으면 현재 ref 기준으로 보간)
        struct MotorCmdInterp {
            bool active = false;
            double start_ref = 0.0;    // interpolation 시작 시점의 ref position
            double target_pos = 0.0;   // 목표 position (변경 감지용)
            double duration_ms = 0.0;
            double elapsed_ms = 0.0;
        };
        std::array<MotorCmdInterp, 6> cmd_interp{};

        int so=-1;
        int cmd_wdt[6] = {0,};
        int com_wdt[6] = {0,};
        int can_wdt = 0;
        bool on_flag[6] = {false,};
        bool motor_sync_[6] = {false,}; // 피드백 기반 동기화 완료 여부
        bool link_initialized = false;
        bool restart_required = false;
        std::array<double, 6> joint_feedback_pos_{};

        std::array<MotorCommand, 6> cmds;
        std::vector<std::shared_ptr<Motor>> motors = {
            std::make_shared<RmdX6P36>(0x11), // hip_yaw_right
            std::make_shared<RmdX6P36>(0x12), // hip_roll_right
            std::make_shared<RmdX6P36>(0x13), // hip_pitch_right
            std::make_shared<RmdX6P36>(0x14), // knee_right
            std::make_shared<RobStride03>(0x15), // ankle_upper_right
            std::make_shared<RobStride03>(0x16), // ankle_lower_right
        };

    };
};
