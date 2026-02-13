#include <iostream>
#include <cstring>
#include <cmath>
#include <chrono>
#include <thread>
#include <csignal>
#include <iomanip>
#include <string>
#include <cstdint>
#include <mutex>
#include <atomic>
#include <vector>
#include <memory> // unique_ptr를 위해 추가

// 시스템 헤더 파일
#include <cstdlib>
#include <fcntl.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <unistd.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/select.h>

// 리얼타임 관련 헤더
#include <sched.h>
#include <sys/mman.h>
#include <pthread.h>

// --- 설정 변수 ---
const char* CAN0_INTERFACE = "can0";
const char* CAN2_INTERFACE = "can2";
double TARGET_POS_A_DEG = 180.0;
double TARGET_POS_B_DEG = 0.0;
double POSITION_TOLERANCE_DEG = 5.0;

// 주기 설정
constexpr int TX_FREQ_HZ = 500;
constexpr auto TX_PERIOD = std::chrono::microseconds(1000000 / TX_FREQ_HZ);
constexpr int RX_FREQ_HZ = 1000;
constexpr int RX_PERIOD = 1000000 / RX_FREQ_HZ;

std::atomic<bool> run_flag{true};

// --- Motor 구조체 정의 ---
struct Motor {
    canid_t can_id;
    uint16_t max_speed;
    int32_t zero_point;
    float max_torque;
    float tc;

    std::atomic<float> pos{0.0f};
    std::atomic<int8_t> temp{0};
    std::atomic<int16_t> cur{0};
    std::atomic<int16_t> vel{0};
    std::atomic<double> target_pos_deg{180.0};
    
    int target_reached_count{0};
    std::atomic<int> tx_scount{0};
    std::atomic<int> rx_scount{0};
    std::atomic<int> tx_pcount{0};
    std::atomic<int> rx_pcount{0};

    Motor(canid_t id, uint16_t speed, int32_t zp, float torque, float torque_const)
        : can_id(id), max_speed(speed), zero_point(zp), max_torque(torque), tc(torque_const) {}
};

std::vector<std::unique_ptr<Motor>> motors;

// --- 유틸리티 함수 ---
void signal_handler(int) { run_flag = false; }

void set_thread_affinity(int core, int priority) {
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(core, &cpuset);
    if (pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset) != 0) {
        perror("pthread_setaffinity_np failed");
    }
    struct sched_param param = { .sched_priority = priority };
    if (sched_setscheduler(0, SCHED_FIFO, &param) != 0) {
        perror("sched_setscheduler failed");
    }
}

int open_can_socket(const char* interface, const std::vector<struct can_filter>& filters) {
    int s;
    struct sockaddr_can addr;
    struct ifreq ifr;

    s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s < 0) { perror("socket"); return -1; }

    int flags = fcntl(s, F_GETFL, 0);
    if (flags < 0) { perror("fcntl get"); close(s); return -1; }
    if (fcntl(s, F_SETFL, flags | O_NONBLOCK) < 0) { perror("fcntl set"); close(s); return -1; }

    strcpy(ifr.ifr_name, interface);
    if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) { perror("ioctl"); close(s); return -1; }

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(s, (struct sockaddr*)&addr, sizeof(addr)) < 0) { perror("bind"); close(s); return -1; }
    
    if (!filters.empty()) {
        if (setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, filters.data(), filters.size() * sizeof(struct can_filter)) != 0) {
            perror("Setting CAN filters failed");
            close(s);
            return -1;
        }
    }

    return s;
}

// --- CAN 통신 스레드 ---

void can_tx_task(int core, const char* interface_name, canid_t id_min, canid_t id_max) {
    set_thread_affinity(core, 98);
    std::cout << interface_name << " TX 스레드 시작 (Core " << core << ")" << std::endl;
    int sock = open_can_socket(interface_name, {});
    if (sock < 0) { std::cerr << interface_name << " TX 소켓 생성 실패" << std::endl; return; }

    auto next_cycle = std::chrono::steady_clock::now();
    int loop_counter = 0;

    while (run_flag) {
        loop_counter++;
        for (const auto& motor_ptr : motors) {
            // 자신의 담당 ID 범위에 있는 모터만 처리
            if (motor_ptr->can_id >= id_min && motor_ptr->can_id <= id_max) {
                if (loop_counter % (TX_FREQ_HZ / 50) == 0) {
                    struct can_frame frame = { .can_id = motor_ptr->can_id, .can_dlc = 8, .data = {0x92, 0,0,0,0,0,0,0} };
                    if(write(sock, &frame, sizeof(frame)) > 0) motor_ptr->tx_scount++;
                }
                if (loop_counter % (TX_FREQ_HZ / 50) == 4) {
                    struct can_frame frame = { .can_id = motor_ptr->can_id, .can_dlc = 8, .data = {0x9C, 0,0,0,0,0,0,0} };
                    if(write(sock, &frame, sizeof(frame)) > 0) motor_ptr->tx_pcount++;
                }
            }
        }
        next_cycle += TX_PERIOD;
        std::this_thread::sleep_until(next_cycle);
    }
    close(sock);
    std::cout << interface_name << " TX 스레드 종료" << std::endl;
}

void can_rx_task(int core, const char* interface_name, canid_t id_min, canid_t id_max) {
    set_thread_affinity(core, 99);
    std::cout << interface_name << " RX 스레드 시작 (Core " << core << ")" << std::endl;
    
    std::vector<struct can_filter> filters;
    for (const auto& motor_ptr : motors) {
        if (motor_ptr->can_id >= id_min && motor_ptr->can_id <= id_max) {
            filters.push_back({ .can_id = motor_ptr->can_id + 0x100, .can_mask = CAN_SFF_MASK });
        }
    }
    int sock = open_can_socket(interface_name, filters);
    if (sock < 0) { std::cerr << interface_name << " RX 소켓 생성 실패" << std::endl; return; }

    while (run_flag) {
        fd_set read_fds;
        FD_ZERO(&read_fds);
        FD_SET(sock, &read_fds);
        struct timeval timeout = {0, RX_PERIOD}; // 1ms timeout

        if (select(sock + 1, &read_fds, NULL, NULL, &timeout) > 0) {
            struct can_frame frame;
            while (read(sock, &frame, sizeof(frame)) > 0) { 
                if (frame.can_dlc == 8) {
                    canid_t cmd_id = frame.can_id - 0x100; // 버그 수정: 응답 ID -> 명령 ID
                    for (const auto& motor_ptr : motors) {
                        if (motor_ptr->can_id == cmd_id) {
                            if (frame.data[0] == 0x9C || frame.data[0] == 0xA1 || frame.data[0] == 0xA4) {
                                motor_ptr->temp.store(static_cast<int8_t>(frame.data[1]));
                                motor_ptr->cur.store(static_cast<int16_t>((frame.data[3] << 8) | frame.data[2]));
                                motor_ptr->vel.store(static_cast<int16_t>((frame.data[5] << 8) | frame.data[4]));
                                motor_ptr->rx_pcount++;
                            } else if (frame.data[0] == 0x92) {
                                int32_t pos_val;
                                memcpy(&pos_val, &frame.data[4], 4);
                                motor_ptr->pos.store(pos_val * 0.01f);
                                motor_ptr->rx_scount++;
                            }
                            break; 
                        }
                    }
                }
            }
        }
    }
    close(sock);
    std::cout << interface_name << " RX 스레드 종료" << std::endl;
}


// --- 제어 및 메인 함수 ---
void main_control_thread() {
    set_thread_affinity(0, 95);
    std::cout << "메인 제어 스레드 시작 (Core 0)" << std::endl;

    const int CONTROL_FREQ_HZ = 500;
    const auto CONTROL_PERIOD = std::chrono::microseconds(1000000 / CONTROL_FREQ_HZ);
    
    auto next_cycle = std::chrono::steady_clock::now();
    int loop_counter = 0;

    while (run_flag) {
        loop_counter++;
        for (auto& motor_ptr : motors) {
            float current_pos_deg = motor_ptr->pos.load();
            double target_pos_deg = motor_ptr->target_pos_deg.load();

            if (std::abs(current_pos_deg - target_pos_deg) < POSITION_TOLERANCE_DEG) {
                if (++motor_ptr->target_reached_count >= static_cast<int>(CONTROL_FREQ_HZ * 0.5)) {
                    double new_target = (target_pos_deg == TARGET_POS_A_DEG) ? TARGET_POS_B_DEG : TARGET_POS_A_DEG;
                    motor_ptr->target_pos_deg.store(new_target);
                    motor_ptr->target_reached_count = 0;
                }
            } else {
                motor_ptr->target_reached_count = 0;
            }
        }

        if (loop_counter % (CONTROL_FREQ_HZ/50) == 0) { 
             std::cout << "\n--- 모터 상태 (" << std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count() << "s) ---" << std::endl;
            for (const auto& motor_ptr : motors) {
                std::cout << "  [ID 0x" << std::hex << motor_ptr->can_id << std::dec << "]"
                          << " 위치: " << std::fixed << std::setw(6) << std::setprecision(1) << motor_ptr->pos.load() << "°"
                          << " | 속도: " << std::setw(5) << motor_ptr->vel.load() << " rpm"
                          << " | 전류: " << std::setw(5) << motor_ptr->cur.load() << " mA"
                          << " | 온도: " << std::setw(3) << static_cast<int>(motor_ptr->temp.load()) << "°C"
                          << " | 목표: " << std::setw(5) << std::setprecision(1) << motor_ptr->target_pos_deg.load() << "°" 
                          << " | TX(S/P): " << std::setw(5) << motor_ptr->tx_scount.load() << "/" << std::setw(5) << motor_ptr->tx_pcount.load()
                          << " | RX(S/P): " << std::setw(5) << motor_ptr->rx_scount.load() << "/" << std::setw(5) << motor_ptr->rx_pcount.load()
                          << std::endl;
            }
        }
        next_cycle += CONTROL_PERIOD;
        std::this_thread::sleep_until(next_cycle);
    }
    std::cout << "\n메인 제어 스레드 종료" << std::endl;
}

void send_stop_command_to_all() {
    int sock_can0 = open_can_socket(CAN0_INTERFACE, {});
    int sock_can2 = open_can_socket(CAN2_INTERFACE, {});
    if(sock_can0 < 0 && sock_can2 < 0) return;

    std::cout << "모든 모터에 정지 명령 전송..." << std::endl;
    for (const auto& motor_ptr : motors) {
        int current_sock = (motor_ptr->can_id < 0x150) ? sock_can0 : sock_can2;
        if(current_sock < 0) continue;
        struct can_frame frame = { .can_id = motor_ptr->can_id, .can_dlc = 8, .data = {0x80, 0, 0, 0, 0, 0, 0, 0} };
        (void)write(current_sock, &frame, sizeof(frame));
        std::this_thread::sleep_for(std::chrono::milliseconds(5)); 
    }

    if(sock_can0 >= 0) close(sock_can0);
    if(sock_can2 >= 0) close(sock_can2);
}

int main() {
    signal(SIGINT, signal_handler);
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) { perror("mlockall"); }
    
    motors.push_back(std::make_unique<Motor>(0x141, 100, 0, 30, 30.0f/4.0f));
    motors.push_back(std::make_unique<Motor>(0x142, 100, 0, 30, 30.0f/4.0f));
    motors.push_back(std::make_unique<Motor>(0x143, 100, 0, 30, 50.0f/6.7f));
    motors.push_back(std::make_unique<Motor>(0x144, 100, 0, 30, 50.0f/6.7f));
    motors.push_back(std::make_unique<Motor>(0x145, 100, 0, 30, 4.0f/3.6f));
    motors.push_back(std::make_unique<Motor>(0x146, 100, 0, 30, 4.0f/3.6f));
    motors.push_back(std::make_unique<Motor>(0x151, 100, 0, 30, 30.0f/4.0f));
    motors.push_back(std::make_unique<Motor>(0x152, 100, 0, 30, 30.0f/4.0f));
    motors.push_back(std::make_unique<Motor>(0x153, 100, 0, 30, 50.0f/6.7f));
    motors.push_back(std::make_unique<Motor>(0x154, 100, 0, 30, 50.0f/6.7f));
    motors.push_back(std::make_unique<Motor>(0x155, 100, 0, 30, 4.0f/3.6f));
    motors.push_back(std::make_unique<Motor>(0x156, 100, 0, 30, 4.0f/3.6f));

    std::cout << "RMD-X 다중 모터 제어 시작 (5-스레드, 5-코어 모델)" << std::endl;
    
    std::thread tx0(can_tx_task, 1, CAN0_INTERFACE, 0x141, 0x146);
    std::thread tx1(can_tx_task, 2, CAN2_INTERFACE, 0x151, 0x156);
    std::thread rx0(can_rx_task, 3, CAN0_INTERFACE, 0x141, 0x146);
    std::thread rx1(can_rx_task, 4, CAN2_INTERFACE, 0x151, 0x156);
    std::thread control(main_control_thread);

    std::cout << "왕복 운동 시작: " << TARGET_POS_A_DEG << "도 ↔ " << TARGET_POS_B_DEG << "도 (Ctrl+C로 종료)" << std::endl;

    tx0.join();
    tx1.join();
    rx0.join();
    rx1.join();
    control.join();
    
    send_stop_command_to_all();
    munlockall();
    std::cout << "모든 스레드 정리 완료" << std::endl;

    return 0;
}
