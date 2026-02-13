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
#include <memory>
#include <sstream> // for std::stringstream

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

// ROS 2 헤더
#include "rclcpp/rclcpp.hpp"
#include "msg_interfaces/msg/joymsg.hpp"
#include "msg_interfaces/msg/observation.hpp"
#include "msg_interfaces/msg/action.hpp"

// --- 설정 변수 ---
const char* CAN0_INTERFACE = "can0";
const char* CAN2_INTERFACE = "can2";

// 주기 설정 (Hz)
constexpr int TX_FREQ_HZ = 500;
constexpr int RX_FREQ_HZ = 1000;
constexpr int CONTROL_PUB_FREQ_HZ = 500; // 제어 및 발행 주기

// 시간 변환
constexpr auto TX_PERIOD = std::chrono::microseconds(1000000 / TX_FREQ_HZ);
constexpr auto RX_PERIOD_US = 1000000 / RX_FREQ_HZ;
constexpr auto CONTROL_PUB_PERIOD = std::chrono::microseconds(1000000 / CONTROL_PUB_FREQ_HZ);


// --- Motor 구조체 정의 ---
struct Motor {
    canid_t can_id;
    uint16_t max_speed;
    int32_t zero_point;
    float max_torque;
    float tc;

    std::atomic<float> pos_deg{0.0f};
    std::atomic<int8_t> temp{0};
    std::atomic<int16_t> cur{0};
    std::atomic<int16_t> vel_rpm{0};
    std::atomic<double> target_pos_deg{0.0};
    std::atomic<int> tx_scount{0};
    std::atomic<int> rx_scount{0};
    std::atomic<int> tx_pcount{0};
    std::atomic<int> rx_pcount{0};
    std::atomic<int> tx_cmd_count{0};

    Motor(canid_t id, uint16_t speed, int32_t zp, float torque, float torque_const)
        : can_id(id), max_speed(speed), zero_point(zp), max_torque(torque), tc(torque_const) {}
};

class MotorControllerNode : public rclcpp::Node
{
public:
    MotorControllerNode() : Node("motor_controller_node")
    {
        obs_pub_ = this->create_publisher<msg_interfaces::msg::Observation>("motor_observation", 10);
        joy_sub_ = this->create_subscription<msg_interfaces::msg::Joymsg>(
            "joy_command", 10, std::bind(&MotorControllerNode::joy_callback, this, std::placeholders::_1));
        action_sub_ = this->create_subscription<msg_interfaces::msg::Action>(
            "motor_action", 10, std::bind(&MotorControllerNode::action_callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "ROS 2 Publisher & Subscribers initialized.");

        if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
            RCLCPP_WARN(this->get_logger(), "mlockall failed: %s. Run with sudo or set capabilities.", strerror(errno));
        }

        initialize_motors();
        RCLCPP_INFO(this->get_logger(), "%zu motors initialized.", motors_.size());

        run_flag_.store(true);
        threads_.emplace_back(&MotorControllerNode::can_tx_task, this, 1, CAN0_INTERFACE, 0x141, 0x146);
        threads_.emplace_back(&MotorControllerNode::can_tx_task, this, 2, CAN2_INTERFACE, 0x151, 0x156);
        threads_.emplace_back(&MotorControllerNode::can_rx_task, this, 3, CAN0_INTERFACE, 0x141, 0x146);
        threads_.emplace_back(&MotorControllerNode::can_rx_task, this, 4, CAN2_INTERFACE, 0x151, 0x156);
        threads_.emplace_back(&MotorControllerNode::control_and_publish_thread, this, 0);
        
        RCLCPP_INFO(this->get_logger(), "RMD-X Multi-motor controller node started.");
        RCLCPP_INFO(this->get_logger(), "Waiting for Action commands on topic 'motor_action'...");
    }

    ~MotorControllerNode()
    {
        run_flag_.store(false);
        RCLCPP_INFO(this->get_logger(), "Stopping threads...");
        for (auto& t : threads_) {
            if (t.joinable()) t.join();
        }
        send_stop_command_to_all();
        munlockall();
        RCLCPP_INFO(this->get_logger(), "All threads and motors stopped. Node shutting down.");
    }

private:
    rclcpp::Publisher<msg_interfaces::msg::Observation>::SharedPtr obs_pub_;
    rclcpp::Subscription<msg_interfaces::msg::Joymsg>::SharedPtr joy_sub_;
    rclcpp::Subscription<msg_interfaces::msg::Action>::SharedPtr action_sub_;
    std::vector<std::unique_ptr<Motor>> motors_;
    std::vector<std::thread> threads_;
    std::atomic<bool> run_flag_{false};
    std::atomic<bool> motors_enabled_{false};
    std::mutex motor_data_mutex_;

    void initialize_motors() {
        motors_.push_back(std::make_unique<Motor>(0x141, 600, 0, 30, 30.0f/4.0f));
        motors_.push_back(std::make_unique<Motor>(0x142, 600, 0, 30, 30.0f/4.0f));
        motors_.push_back(std::make_unique<Motor>(0x143, 600, 0, 30, 50.0f/6.7f));
        motors_.push_back(std::make_unique<Motor>(0x144, 600, 0, 30, 50.0f/6.7f));
        motors_.push_back(std::make_unique<Motor>(0x145, 600, 0, 30, 4.0f/3.6f));
        motors_.push_back(std::make_unique<Motor>(0x146, 600, 0, 30, 4.0f/3.6f));
        motors_.push_back(std::make_unique<Motor>(0x151, 600, 0, 30, 30.0f/4.0f));
        motors_.push_back(std::make_unique<Motor>(0x152, 600, 0, 30, 30.0f/4.0f));
        motors_.push_back(std::make_unique<Motor>(0x153, 600, 0, 30, 50.0f/6.7f));
        motors_.push_back(std::make_unique<Motor>(0x154, 600, 0, 30, 50.0f/6.7f));
        motors_.push_back(std::make_unique<Motor>(0x155, 600, 0, 30, 4.0f/3.6f));
        motors_.push_back(std::make_unique<Motor>(0x156, 600, 0, 30, 4.0f/3.6f));
        for(const auto& motor : motors_) {
            motor->target_pos_deg.store(motor->pos_deg.load());
        }
    }

    void joy_callback(const msg_interfaces::msg::Joymsg::SharedPtr msg) {
        if (msg->buttons.size() > 4 && msg->buttons[4]) {
            motors_enabled_ = !motors_enabled_.load();
            RCLCPP_INFO(this->get_logger(), "Motors %s by joystick.", motors_enabled_ ? "ENABLED" : "DISABLED");
        }
    }

    void action_callback(const msg_interfaces::msg::Action::SharedPtr msg) {
        if (msg->action.size() != motors_.size()) {
            RCLCPP_WARN(this->get_logger(), "Received action size (%zu) != motor count (%zu).", msg->action.size(), motors_.size());
            return;
        }
        std::lock_guard<std::mutex> lock(motor_data_mutex_);
        for (size_t i = 0; i < motors_.size(); ++i) {
            motors_[i]->target_pos_deg.store(static_cast<double>(msg->action[i]));
        }
    }
    
    void set_thread_affinity(int core, int priority, const std::string& thread_name) {
        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        CPU_SET(core, &cpuset);
        if (pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset) != 0) {
             RCLCPP_WARN(this->get_logger(), "pthread_setaffinity_np for %s failed: %s", thread_name.c_str(), strerror(errno));
        }
        struct sched_param param = { .sched_priority = priority };
        if (sched_setscheduler(0, SCHED_FIFO, &param) != 0) {
            RCLCPP_WARN(this->get_logger(), "sched_setscheduler for %s failed: %s. Run with sudo or set capabilities.", thread_name.c_str(), strerror(errno));
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

    void can_tx_task(int core, const char* interface_name, canid_t id_min, canid_t id_max) {
        set_thread_affinity(core, 98, std::string(interface_name) + "_tx");
        RCLCPP_INFO(this->get_logger(), "%s TX thread started (Core %d)", interface_name, core);
        int sock = open_can_socket(interface_name, {});
        if (sock < 0) { 
            RCLCPP_ERROR(this->get_logger(), "%s TX socket creation failed", interface_name); 
            return; 
        }
        auto next_cycle = std::chrono::steady_clock::now();
        int loop_counter = 0;
        while (run_flag_.load()) {
            loop_counter++;
            for (const auto& motor_ptr : motors_) {
                if (motor_ptr->can_id >= id_min && motor_ptr->can_id <= id_max) {
                    if (loop_counter % (TX_FREQ_HZ / 50) == 1) {
                        struct can_frame frame = { .can_id = motor_ptr->can_id, .can_dlc = 8, .data = {0x92, 0,0,0,0,0,0,0} };
                        if(write(sock, &frame, sizeof(frame)) > 0) motor_ptr->tx_scount++;
                    }
                    if (loop_counter % (TX_FREQ_HZ / 50) == 4) {
                        struct can_frame frame = { .can_id = motor_ptr->can_id, .can_dlc = 8, .data = {0x9C, 0,0,0,0,0,0,0} };
                        if(write(sock, &frame, sizeof(frame)) > 0) motor_ptr->tx_pcount++;
                    }
                    if (motors_enabled_.load()) {
                        int32_t target_pos_int = static_cast<int32_t>(motor_ptr->target_pos_deg.load() * 100.0);
                        uint16_t speed_limit = motor_ptr->max_speed;
                        struct can_frame frame = { .can_id = motor_ptr->can_id, .can_dlc = 8 };
                        frame.data[0] = 0xA4;
                        frame.data[1] = 0x00;
                        frame.data[2] = speed_limit & 0xFF;
                        frame.data[3] = (speed_limit >> 8) & 0xFF;
                        memcpy(&frame.data[4], &target_pos_int, 4);
                        if(write(sock, &frame, sizeof(frame)) > 0) motor_ptr->tx_cmd_count++;
                    }
                }
            }
            next_cycle += TX_PERIOD;
            std::this_thread::sleep_until(next_cycle);
        }
        close(sock);
        RCLCPP_INFO(this->get_logger(), "%s TX thread finished.", interface_name);
    }

    void can_rx_task(int core, const char* interface_name, canid_t id_min, canid_t id_max) {
        set_thread_affinity(core, 99, std::string(interface_name) + "_rx");
        RCLCPP_INFO(this->get_logger(), "%s RX thread started (Core %d)", interface_name, core);
        std::vector<struct can_filter> filters;
        for (const auto& motor_ptr : motors_) {
            if (motor_ptr->can_id >= id_min && motor_ptr->can_id <= id_max) {
                filters.push_back({ .can_id = motor_ptr->can_id, .can_mask = CAN_SFF_MASK });
            }
        }
        int sock = open_can_socket(interface_name, filters);
        if (sock < 0) { 
            RCLCPP_ERROR(this->get_logger(), "%s RX socket creation failed", interface_name);
            return;
        }
        while (run_flag_.load()) {
            fd_set read_fds;
            FD_ZERO(&read_fds);
            FD_SET(sock, &read_fds);
            struct timeval timeout = {0, RX_PERIOD_US};
            if (select(sock + 1, &read_fds, NULL, NULL, &timeout) > 0) {
                struct can_frame frame;
                while (read(sock, &frame, sizeof(frame)) > 0) { 
                    if (frame.can_dlc == 8) {
                        for (const auto& motor_ptr : motors_) {
                            if (motor_ptr->can_id == frame.can_id) {
                                if (frame.data[0] == 0x9C || frame.data[0] == 0xA1 || frame.data[0] == 0xA4) {
                                    motor_ptr->temp.store(static_cast<int8_t>(frame.data[1]));
                                    motor_ptr->cur.store(static_cast<int16_t>((frame.data[3] << 8) | frame.data[2]));
                                    motor_ptr->vel_rpm.store(static_cast<int16_t>((frame.data[5] << 8) | frame.data[4]));
                                    motor_ptr->rx_pcount++;
                                } else if (frame.data[0] == 0x92) {
                                    int32_t pos_val;
                                    memcpy(&pos_val, &frame.data[4], 4); 
                                    motor_ptr->pos_deg.store(pos_val * 0.01f);
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
        RCLCPP_INFO(this->get_logger(), "%s RX thread finished.", interface_name);
    }
    
    void control_and_publish_thread(int core) {
        set_thread_affinity(core, 95, "control_pub");
        RCLCPP_INFO(this->get_logger(), "Control & Publish thread started (Core %d)", core);
        auto next_cycle = std::chrono::steady_clock::now();
        int loop_counter = 0;
        while(run_flag_.load()) {
            loop_counter++;
            auto obs_msg = msg_interfaces::msg::Observation();
            obs_msg.header.stamp = this->get_clock()->now();
            obs_msg.joint_pos.resize(motors_.size());
            obs_msg.joint_vel.resize(motors_.size());
            {
                std::lock_guard<std::mutex> lock(motor_data_mutex_);
                for (size_t i = 0; i < motors_.size(); ++i) {
                    obs_msg.joint_pos[i] = motors_[i]->pos_deg.load();
                    obs_msg.joint_vel[i] = motors_[i]->vel_rpm.load();
                }
            }
            obs_pub_->publish(obs_msg);

            if (loop_counter % (CONTROL_PUB_FREQ_HZ / 2) == 0) { // 2Hz
                std::stringstream ss;
                ss << "\n--- Motor Status (" << std::fixed << std::setprecision(2) << this->get_clock()->now().seconds() << "s) | Motors Enabled: " << (motors_enabled_.load() ? "YES" : "NO") << " ---";
                RCLCPP_INFO_STREAM(this->get_logger(), ss.str());

                for (const auto& motor_ptr : motors_) {
                    std::stringstream motor_ss;
                    motor_ss << "  [ID 0x" << std::hex << motor_ptr->can_id << std::dec << "]"
                             << " 위치: " << std::fixed << std::setw(6) << std::setprecision(1) << motor_ptr->pos_deg.load() << "°"
                             << " | 속도: " << std::setw(5) << motor_ptr->vel_rpm.load() << " rpm"
                             << " | 전류: " << std::setw(5) << motor_ptr->cur.load() << " mA"
                             << " | 온도: " << std::setw(3) << static_cast<int>(motor_ptr->temp.load()) << "°C"
                             << " | 목표: " << std::setw(5) << std::setprecision(1) << motor_ptr->target_pos_deg.load() << "°" 
                             << " | TX(S/P/C): " << motor_ptr->tx_scount.load() << "/" << motor_ptr->tx_pcount.load() << "/" << motor_ptr->tx_cmd_count.load()
                             << " | RX(S/P): " << motor_ptr->rx_scount.load() << "/" << motor_ptr->rx_pcount.load();
                    RCLCPP_INFO_STREAM(this->get_logger(), motor_ss.str());
                }
            }
            next_cycle += CONTROL_PUB_PERIOD;
            std::this_thread::sleep_until(next_cycle);
        }
        RCLCPP_INFO(this->get_logger(), "Control & Publish thread finished.");
    }
    
    void send_stop_command_to_all() {
        int sock_can0 = open_can_socket(CAN0_INTERFACE, {});
        int sock_can2 = open_can_socket(CAN2_INTERFACE, {});
        if(sock_can0 < 0 && sock_can2 < 0) return;
        RCLCPP_INFO(this->get_logger(), "Sending STOP command to all motors...");
        for (const auto& motor_ptr : motors_) {
            int current_sock = (motor_ptr->can_id < 0x150) ? sock_can0 : sock_can2;
            if(current_sock < 0) continue;
            struct can_frame frame = { .can_id = motor_ptr->can_id, .can_dlc = 8, .data = {0x80, 0, 0, 0, 0, 0, 0, 0} };
            (void)write(current_sock, &frame, sizeof(frame));
            std::this_thread::sleep_for(std::chrono::milliseconds(5)); 
        }
        if(sock_can0 >= 0) close(sock_can0);
        if(sock_can2 >= 0) close(sock_can2);
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotorControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
