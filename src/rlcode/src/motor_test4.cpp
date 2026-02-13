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
#include <tuple> 

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
#include <Eigen/Eigen>
#include <Eigen/Dense>

// 리얼타임 관련 헤더
#include <sched.h>
#include <sys/mman.h>
#include <pthread.h>

// ROS 2 헤더
#include "rclcpp/rclcpp.hpp"
#include "msg_interfaces/msg/joymsg.hpp"
#include "msg_interfaces/msg/observation.hpp"
#include "msg_interfaces/msg/action.hpp"
#include "msg_interfaces/msg/imudata2.hpp"
#include "msg_interfaces/msg/test.hpp"

// --- 설정 변수 ---
const char* CAN0_INTERFACE = "can0";
const char* CAN2_INTERFACE = "can2";
#define PI 3.1415926
using namespace std;
using namespace Eigen;

// 주기 설정 (Hz)
constexpr int TX_FREQ_HZ = 500;
constexpr int RX_FREQ_HZ = 1000;
constexpr int CONTROL_PUB_FREQ_HZ = 200; // 제어 및 발행 주기

// 시간 변환
constexpr auto TX_PERIOD = std::chrono::microseconds(1000000 / TX_FREQ_HZ);
constexpr auto RX_PERIOD_US = 1000000 / RX_FREQ_HZ;
constexpr auto CONTROL_PUB_PERIOD = std::chrono::microseconds(1000000 / CONTROL_PUB_FREQ_HZ);


// --- Motor 구조체 정의 ---
struct Motor {
    canid_t can_id;
    uint16_t max_speed;
    float max_torque;
    float tc;
    float kp;
    float kd;
    float ready_pos;

    // 모터의 현재 상태 (CAN RX 스레드에서 업데이트)
    bool reset_flag{false};
    int32_t zero_point{0};
    float target_torque{0.0f};
    float filtered_torque{0.0f};
    float pos{0.0f};
    float pos_deg{0.0f};
    int8_t temp{0};
    int16_t cur{0};
    int16_t vel{0};
    float rad_vel{0.0f};
    float motor_torque{0.0f};

    std::mutex state_mutex;
    
    // 모터의 목표 위치 (Action Subscriber에서 업데이트)
    std::atomic<float> target_pos_deg{0.0f};
    std::atomic<float> target_pos{0.0f};
    
    // 통신 카운트
    std::atomic<int> tx_scount{0};
    std::atomic<int> rx_scount{0};
    std::atomic<int> tx_pcount{0};
    std::atomic<int> rx_pcount{0};
    std::atomic<int> tx_cmd_count{0};

    Motor(canid_t id, uint16_t speed, float torque, float torque_const, float kp, float kd, float ready_pos)
        : can_id(id), max_speed(speed), max_torque(torque), tc(torque_const), kp(kp), kd(kd), ready_pos(ready_pos) {}
};

class MotorControllerNode : public rclcpp::Node
{
public:
    MotorControllerNode() : Node("motor_controller_node")
    {
        // can 모듈 활성화 --------------------------------------------------------------------------------
        system("sudo ip link set can0 up type can bitrate 1000000");
        // system("sudo ip link set can1 up type can bitrate 1000000");
        system("sudo ip link set can2 up type can bitrate 1000000");
        // ---------------------------------------------------------------------------------------

        // 시작 시간 초기화 (clock 함수용)
        start_time_ = std::chrono::steady_clock::now();
        
        // --- ROS 2 인터페이스 초기화 ---
        obs_pub_ = this->create_publisher<msg_interfaces::msg::Observation>("mujoco_obs", 10);

        test_pub_ = this->create_publisher<msg_interfaces::msg::Test>("test", 10);
        
        joy_sub_ = this->create_subscription<msg_interfaces::msg::Joymsg>(
            "joystick_state", 10, std::bind(&MotorControllerNode::joy_callback, this, std::placeholders::_1));
            
        action_sub_ = this->create_subscription<msg_interfaces::msg::Action>(
            "action", 10, std::bind(&MotorControllerNode::action_callback, this, std::placeholders::_1));
        
        imu_sub_ = this->create_subscription<msg_interfaces::msg::Imudata2>(
            "imu_data", 10, std::bind(&MotorControllerNode::imu_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "ROS 2 Publisher & Subscribers initialized.");
        
        // 변수 설정
        joint_pos.assign(num_joints, 0.0f);
        joint_vel.assign(num_joints, 0.0f);
        ankle_Jac_r << 0,0,
                       0,0;
        ankle_Jac_l << 0,0,
                       0,0;
        
        // --- 실시간 메모리 락 ---
        if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
            RCLCPP_ERROR(this->get_logger(), "mlockall failed: %s", strerror(errno));
        }

        // --- 모터 객체 생성 ---
        initialize_motors();
        RCLCPP_INFO(this->get_logger(), "%zu motors initialized.", motors_.size());

        // --- 통신 및 제어 스레드 시작 ---
        run_flag_.store(true);
        threads_.emplace_back(&MotorControllerNode::can_tx_task, this, 1, CAN0_INTERFACE, 0x141, 0x146);
        threads_.emplace_back(&MotorControllerNode::can_tx_task, this, 2, CAN2_INTERFACE, 0x151, 0x156);
        threads_.emplace_back(&MotorControllerNode::can_rx_task, this, 3, CAN0_INTERFACE, 0x141, 0x146);
        threads_.emplace_back(&MotorControllerNode::can_rx_task, this, 4, CAN2_INTERFACE, 0x151, 0x156);
        threads_.emplace_back(&MotorControllerNode::control_and_publish_thread, this, 5);
        
        RCLCPP_INFO(this->get_logger(), "RMD-X Multi-motor controller node started.");
        RCLCPP_INFO(this->get_logger(), "Waiting for Action commands on topic 'motor_action'...");
    }

    ~MotorControllerNode()
    {
        run_flag_.store(false);
        RCLCPP_INFO(this->get_logger(), "Stopping threads...");

        for (auto& t : threads_) {
            if (t.joinable()) {
                t.join();
            }
        }
        
        send_stop_command_to_all();
        munlockall();
        RCLCPP_INFO(this->get_logger(), "All threads and motors stopped. Node shutting down.");
    }

Matrix2f ankle_J(float R, float P, float t1, float t2, char rl){
    float l1 = 46.25;
    float l2_l = 298.457;
    float l2_s = 210.997;
    float l3 = 300;
    float w1 = 64.75;
    float Cx = 37;
    float Cy = 55.5;
    float Cz = -22.5;
    float origin_t = (31.29*PI/180);

    Matrix3f ROF;
    Vector3f OB1;
    Vector3f OB2;
    Vector3f FC1;
    Vector3f FC2;
    Vector3f OF;
    Vector3f BC1;
    Vector3f BC2;
    Vector3f AB1;
    Vector3f AB2;
    Vector2f A;
    Vector3f B1;
    Vector3f B2;
    Matrix2f C;

    ROF << cos(P), sin(P)*sin(R), cos(R)*sin(P),
            0,        cos(R),       -sin(R),
            -sin(P), cos(P)*sin(R), cos(P)*cos(R);

    if (rl == 'r'){
        OB1 << l1*cos(t1), w1, -l1*sin(t1);
        OB2 << l1*cos(t2), -w1, -87.5-l1*sin(t2);
    }
    else if (rl == 'l'){
        OB1 << l1*cos(t1), w1, -87.5-l1*sin(t1);
        OB2 << l1*cos(t2), -w1, -l1*sin(t2);
    }

    FC1 << Cx, Cy, Cz;
    FC2 << Cx, -Cy, Cz;

    OF << 0,0,-l3;

    BC1 = OF + ROF*FC1 - OB1;
    BC2 = OF + ROF*FC2 - OB2;

    AB1 << l1*cos(t1+origin_t), 0, -l1*sin(t1+origin_t);
    AB2 << l1*cos(t2+origin_t), 0, -l1*sin(t2+origin_t);
    // AB1 << l1*cos(t1), 0, -l1*sin(t1);
    // AB2 << l1*cos(t2), 0, -l1*sin(t2);

    Vector3f X(1,0,0);
    Vector3f Y(0,1,0);

    A << Y.dot(AB1.cross(BC1)),
         Y.dot(AB2.cross(BC2));

    B1 << FC1.cross(BC1);
    B2 << FC2.cross(BC2);

    C << X.dot(B1), Y.dot(B1),
         X.dot(B2), Y.dot(B2);

    Matrix2f J_1;
    J_1.row(0) = C.row(0) / A(0);
    J_1.row(1) = C.row(1) / A(1);
    Matrix2f J = J_1.inverse();

    return J;
}

Vector2f ankle_ik(float R, float P, char rl){
    float l1 = 46.25;
    float l2_l = 298.457;
    float l2_s = 210.997;
    float l3 = 300;
    float w1 = 64.75;
    float Cx = 37;
    float Cy = 55.5;
    float Cz = -22.5;
    float origin_t = (31.29*PI/180);
    
    double A1;
    double B1;
    double C1;
    double A2;
    double B2;
    double C2;

    if(rl == 'r'){
        A1 = Cx*Cx + Cy*Cy + Cz*Cz + l1*l1 - l2_l*l2_l + l3*l3 + w1*w1 - Cz*l3*cos(P + R) - Cy*l3*sin(P + R) - 2*Cy*w1*cos(R) + 2*Cx*l3*sin(P) - Cz*l3*cos(P - R) + 2*Cz*w1*sin(R) + Cy*l3*sin(P - R);
        B1 = - 2*l1*(l3*sin((1043*PI)/6000) + Cx*cos((1043*PI)/6000)*cos(P) + Cx*sin((1043*PI)/6000)*sin(P) + Cz*cos((1043*PI)/6000)*cos(R)*sin(P) - Cz*sin((1043*PI)/6000)*cos(P)*cos(R) + Cy*cos((1043*PI)/6000)*sin(P)*sin(R) - Cy*sin((1043*PI)/6000)*cos(P)*sin(R));
        C1 = 2*l1*(Cx*sin((1043*PI)/6000)*cos(P) - Cx*cos((1043*PI)/6000)*sin(P) - l3*cos((1043*PI)/6000) + Cz*cos((1043*PI)/6000)*cos(P)*cos(R) + Cy*cos((1043*PI)/6000)*cos(P)*sin(R) + Cz*sin((1043*PI)/6000)*cos(R)*sin(P) + Cy*sin((1043*PI)/6000)*sin(P)*sin(R));

        A2 = Cx*Cx + Cy*Cy + Cz*Cz + l1*l1 - l2_s*l2_s + l3*l3 + w1*w1 + (175*Cz*cos(P + R))/2 - 175*l3 - (175*Cy*sin(P + R))/2 - 175*Cx*sin(P) + (175*Cz*cos(P - R))/2 + (175*Cy*sin(P - R))/2  - Cz*l3*cos(P + R) + Cy*l3*sin(P + R) - 2*Cy*w1*cos(R) + 2*Cx*l3*sin(P) - Cz*l3*cos(P - R) - 2*Cz*w1*sin(R) - Cy*l3*sin(P - R) + 30625/4;
        B2 = -l1*(2*l3*sin((1043*PI)/6000) - 175*sin((1043*PI)/6000) + 2*Cx*cos((1043*PI)/6000)*cos(P) + 2*Cx*sin((1043*PI)/6000)*sin(P) + 2*Cz*cos((1043*PI)/6000)*cos(R)*sin(P) - 2*Cz*sin((1043*PI)/6000)*cos(P)*cos(R) - 2*Cy*cos((1043*PI)/6000)*sin(P)*sin(R) + 2*Cy*sin((1043*PI)/6000)*cos(P)*sin(R));
        C2 = l1*(175*cos((1043*PI)/6000) - 2*l3*cos((1043*PI)/6000) - 2*Cx*cos((1043*PI)/6000)*sin(P) + 2*Cx*sin((1043*PI)/6000)*cos(P) + 2*Cz*cos((1043*PI)/6000)*cos(P)*cos(R) - 2*Cy*cos((1043*PI)/6000)*cos(P)*sin(R) + 2*Cz*sin((1043*PI)/6000)*cos(R)*sin(P) - 2*Cy*sin((1043*PI)/6000)*sin(P)*sin(R));
    }
    else if(rl == 'l'){
        A1 = Cx*Cx + Cy*Cy + Cz*Cz + l1*l1 - l2_s*l2_s + l3*l3 + w1*w1 + (175*Cz*cos(P + R))/2- 175*l3+ (175*Cy*sin(P + R))/2- 175*Cx*sin(P)+ (175*Cz*cos(P - R))/2- (175*Cy*sin(P - R))/2- Cz*l3*cos(P + R)- Cy*l3*sin(P + R)- 2*Cy*w1*cos(R)+ 2*Cx*l3*sin(P)- Cz*l3*cos(P - R)+ 2*Cz*w1*sin(R)+ Cy*l3*sin(P - R)+ 30625/4;
        B1 = -l1*(2*l3*sin((1043*PI)/6000) - 175*sin((1043*PI)/6000) + 2*Cx*cos((1043*PI)/6000)*cos(P) + 2*Cx*sin((1043*PI)/6000)*sin(P) + 2*Cz*cos((1043*PI)/6000)*cos(R)*sin(P) - 2*Cz*sin((1043*PI)/6000)*cos(P)*cos(R) + 2*Cy*cos((1043*PI)/6000)*sin(P)*sin(R) - 2*Cy*sin((1043*PI)/6000)*cos(P)*sin(R));
        C1 = l1*(175*cos((1043*PI)/6000) - 2*l3*cos((1043*PI)/6000) - 2*Cx*cos((1043*PI)/6000)*sin(P) + 2*Cx*sin((1043*PI)/6000)*cos(P) + 2*Cz*cos((1043*PI)/6000)*cos(P)*cos(R) + 2*Cy*cos((1043*PI)/6000)*cos(P)*sin(R) + 2*Cz*sin((1043*PI)/6000)*cos(R)*sin(P) + 2*Cy*sin((1043*PI)/6000)*sin(P)*sin(R));
        
        A2 = Cx*Cx + Cy*Cy + Cz*Cz + l1*l1 - l2_l*l2_l + l3*l3 + w1*w1 - Cz*l3*cos(P + R) + Cy*l3*sin(P + R) - 2*Cy*w1*cos(R) + 2*Cx*l3*sin(P) - Cz*l3*cos(P - R) - 2*Cz*w1*sin(R) - Cy*l3*sin(P - R);
        B2 = (2*Cz*l1*sin((1043*PI)/6000)*cos(P)*cos(R) - 2*Cx*l1*cos((1043*PI)/6000)*cos(P) - 2*Cx*l1*sin((1043*PI)/6000)*sin(P) - 2*Cz*l1*cos((1043*PI)/6000)*cos(R)*sin(P) - 2*l1*l3*sin((1043*PI)/6000) + 2*Cy*l1*cos((1043*PI)/6000)*sin(P)*sin(R) - 2*Cy*l1*sin((1043*PI)/6000)*cos(P)*sin(R));
        C2 = (2*Cx*l1*sin((1043*PI)/6000)*cos(P) - 2*Cx*l1*cos((1043*PI)/6000)*sin(P) - 2*l1*l3*cos((1043*PI)/6000) + 2*Cz*l1*cos((1043*PI)/6000)*cos(P)*cos(R) - 2*Cy*l1*cos((1043*PI)/6000)*cos(P)*sin(R) + 2*Cz*l1*sin((1043*PI)/6000)*cos(R)*sin(P) - 2*Cy*l1*sin((1043*PI)/6000)*sin(P)*sin(R));
    }

    Vector2f t1;
    t1 << (-2*C1 + sqrt(4*C1*C1 - 4*(A1-B1)*(A1+B1)))/(2*(A1-B1)), 
          (-2*C1 - sqrt(4*C1*C1 - 4*(A1-B1)*(A1+B1)))/(2*(A1-B1));
    
    Vector2f t2;
    t2 << (-2*C2 + sqrt(4*C2*C2 - 4*(A2-B2)*(A2+B2)))/(2*(A2-B2)), 
          (-2*C2 - sqrt(4*C2*C2 - 4*(A2-B2)*(A2+B2)))/(2*(A2-B2));

    Vector2f theta;
    if(abs(2*atan(t1(0))*180/PI) < 80){
        theta(0) = 2*atan(t1(0));
    }
    else if(abs(2*atan(t1(1))*180/PI) < 80){
        theta(0) = 2*atan(t1(1));
    }
    if(abs(2*atan(t2(0))*180/PI) < 80){
        theta(1) = 2*atan(t2(0));
    }
    else if(2*atan(t2(1)*180/PI) < 80){
        theta(1) = 2*atan(t2(1));
    }

    return theta;
}

tuple<Vector2f,Matrix2f> ankle_fk(float t1, float t2, char rl){
    Vector2f theta;
    float alpha = 1;
    Matrix2f jac;
    Vector2f error;
    Vector2f ankle_rp(0,0); 
    for(int i = 0; i < 12; i++){
        theta = ankle_ik(ankle_rp(0), ankle_rp(1), rl);
        error << t1-theta(0), t2-theta(1);
        jac = ankle_J(ankle_rp(0), ankle_rp(1), theta(0), theta(1), rl);
        ankle_rp += alpha*jac*error;
        if(sqrt(pow(error(0),2) + pow(error(1),2)) < 1e-4){
            break;
        }
    }
    return make_tuple(ankle_rp, jac);
}

private:
    // --- ROS 2 멤버 ---
    rclcpp::Publisher<msg_interfaces::msg::Observation>::SharedPtr obs_pub_;
    rclcpp::Publisher<msg_interfaces::msg::Test>::SharedPtr test_pub_;
    rclcpp::Subscription<msg_interfaces::msg::Joymsg>::SharedPtr joy_sub_;
    rclcpp::Subscription<msg_interfaces::msg::Action>::SharedPtr action_sub_;
    rclcpp::Subscription<msg_interfaces::msg::Imudata2>::SharedPtr imu_sub_;

    // --- 클래스 멤버 ---
    std::vector<std::unique_ptr<Motor>> motors_;
    std::vector<std::thread> threads_;
    std::atomic<bool> run_flag_{false};
    std::atomic<bool> state_flag_{true}; // 조이스틱으로 rl mode 활성화
    std::atomic<bool> torque_flag_{false}; // 조이스틱으로 torque mode 활성화
    std::atomic<bool> origin_flag_{false}; // 조이스틱으로 origin mode 활성화
    std::atomic<bool> ready_flag_{false}; // 조이스틱으로 stop_mode 활성화
    std::atomic<bool> stop_flag_{false}; // 조이스틱으로 stop_mode 활성화
    std::mutex motor_data_mutex_; // 모터 데이터 접근 동기화를 위한 뮤텍스

    std::chrono::steady_clock::time_point start_time_; // Clock 함수용 시작 시간
    
    const int num_joints = 12;
    std::vector<float> ang_vel_{0.0f, 0.0f, 0.0f};
    std::vector<float> projection_gravity_{0.0f, 0.0f, 0.0f};
    std::vector<float> commands_{0.0f, 0.0f, 0.0f};
    std::vector<float> joint_pos;
    std::vector<float> default_joint_positions{0.0f, 0.0f, 0.0f, 0.0f, -0.2f, -0.2f, 0.4f, 0.4f, -0.3f, -0.3f, 0.0f, 0.0};
    std::vector<float> joint_vel;
    
    Vector2f ankle_rp_r{0,0};
    Vector2f ankle_rp_vel_r{0,0};
    Vector2f ankle_rp_tor_r{0,0};
    Vector2f motor2_r{0,0};
    Vector2f motor2_vel_r{0,0};
    Vector2f motor2_tor_r{0,0};
    Matrix2f ankle_Jac_r;
    Vector2f lpf_ankle_rp_vel_r{0,0};

    Vector2f ankle_rp_l{0,0};
    Vector2f ankle_rp_vel_l{0,0};
    Vector2f ankle_rp_tor_l{0,0};
    Vector2f motor2_l{0,0};
    Vector2f motor2_vel_l{0,0};
    Vector2f motor2_tor_l{0,0};
    Matrix2f ankle_Jac_l;
    Vector2f lpf_ankle_rp_vel_l{0,0};
    float alpha = 0.9;
    int stop_count = 0;
    int reset_count = 0;
    int obs_count = 0;

    // --- 모터 초기화 ---
    void initialize_motors() {
        // // can0
        // motors_.push_back(std::make_unique<Motor>(0x141, 2,  0, 30, 30.0f/4.0f, false));
        // motors_.push_back(std::make_unique<Motor>(0x142, 2,  0, 30, 30.0f/4.0f, false));
        // motors_.push_back(std::make_unique<Motor>(0x143, 5,  0, 50, 50.0f/6.7f, false));
        // motors_.push_back(std::make_unique<Motor>(0x144, 11, 0, 50, 50.0f/6.7f, false));
        // motors_.push_back(std::make_unique<Motor>(0x145, 5,  0, 8 , 4.0f/3.6f , false));
        // motors_.push_back(std::make_unique<Motor>(0x146, 5,  0, 8 , 4.0f/3.6f , false));
        // // can2
        // motors_.push_back(std::make_unique<Motor>(0x151, 2,  0, 30, 30.0f/4.0f, false));
        // motors_.push_back(std::make_unique<Motor>(0x152, 2,  0, 30, 30.0f/4.0f, false));
        // motors_.push_back(std::make_unique<Motor>(0x153, 5,  0, 50, 50.0f/6.7f, false));
        // motors_.push_back(std::make_unique<Motor>(0x154, 11, 0, 50, 50.0f/6.7f, false));
        // motors_.push_back(std::make_unique<Motor>(0x156, 5,  0, 8 , 4.0f/3.6f , false));
        // motors_.push_back(std::make_unique<Motor>(0x155, 5,  0, 8 , 4.0f/3.6f , false));   

        motors_.push_back(std::make_unique<Motor>(0x151, 2,  30, 30.0f/4.0f, 240.0, 5.0,  0.0));
        motors_.push_back(std::make_unique<Motor>(0x141, 2,  30, 30.0f/4.0f, 240.0, 5.0,  0.0));
        motors_.push_back(std::make_unique<Motor>(0x152, 2,  30, 30.0f/4.0f, 240.0, 5.0,  0.0));
        motors_.push_back(std::make_unique<Motor>(0x142, 2,  30, 30.0f/4.0f, 240.0, 5.0,  0.0));
        motors_.push_back(std::make_unique<Motor>(0x153, 5,  50, 50.0f/6.7f,  50.0, 1.8, -0.2));
        motors_.push_back(std::make_unique<Motor>(0x143, 5,  50, 50.0f/6.7f,  50.0, 1.8,  0.2));
        motors_.push_back(std::make_unique<Motor>(0x154, 11, 50, 50.0f/6.7f,  20.0, 1.1, -0.4));
        motors_.push_back(std::make_unique<Motor>(0x144, 11, 50, 50.0f/6.7f,  20.0, 1.1,  0.4));
        motors_.push_back(std::make_unique<Motor>(0x156, 5,  8 , 4.0f/3.6f ,  10.0, 1.0, -0.27));
        motors_.push_back(std::make_unique<Motor>(0x145, 5,  8 , 4.0f/3.6f ,  10.0, 1.0, -0.27));
        motors_.push_back(std::make_unique<Motor>(0x155, 5,  8 , 4.0f/3.6f ,  10.0, 1.0,  0.27));
        motors_.push_back(std::make_unique<Motor>(0x146, 5,  8 , 4.0f/3.6f ,  10.0, 1.0,  0.27));

        for(const auto& motor : motors_) {
            motor->target_pos_deg.store(motor->zero_point * 180.0 / PI);
        }
    }

    double deg2rad(double degree){
        double radian = degree*PI/180;
        return radian;
    }
    double rad2deg(double rad){
        double deg = rad*180/PI;
        return deg;
    }
    float lpf(float pre_data, float data, float alpha){
        float lpf_data = pre_data * (1-alpha) + data * alpha;
        return lpf_data;
    }

    std::vector<float> _get_clock() {
        auto now = std::chrono::steady_clock::now();
        double total_time = std::chrono::duration_cast<std::chrono::duration<double>>(now - start_time_).count();
        
        double phase = 2 * (0.15 + 0.3);
        double local_phi = fmod(total_time, phase);
        double phi = local_phi / phase;
        
        std::vector<float> clock = {
            (float)std::sin(2 * M_PI * phi), 
            (float)std::cos(2 * M_PI * phi)
        };
        return clock;
    }

    void publish_observations() {
        obs_count++;
        auto obs_msg = msg_interfaces::msg::Observation();
        obs_msg.header.stamp = this->get_clock()->now();
        obs_msg.joint_pos.resize(motors_.size());
        obs_msg.joint_vel.resize(motors_.size());
        
        for (size_t i = 0; i < motors_.size(); ++i) {
            obs_msg.joint_pos[i] = joint_pos[i] - default_joint_positions[i];
            obs_msg.joint_vel[i] = joint_vel[i];
        }
        obs_msg.base_ang_vel = ang_vel_;
        obs_msg.gravity = projection_gravity_;
        obs_msg.commands = commands_;

        std::vector<float> clock = _get_clock();
        obs_msg.clock = clock;
        obs_msg.flag = run_flag_;
        obs_msg.obs_flag = torque_flag_;
        if (obs_count == 4){
            obs_pub_->publish(obs_msg);
            obs_count = 0;
        }
    }

    void publish_test() {
        auto test_msg = msg_interfaces::msg::Test();
        test_msg.header.stamp = this->get_clock()->now();
        test_msg.test.resize(motors_.size());
        
        for (size_t i = 0; i < motors_.size(); ++i) {
            test_msg.test[i] = joint_pos[i];
        }        
        test_pub_->publish(test_msg);
    }

    // --- ROS 2 콜백 함수 ---
    void joy_callback(const msg_interfaces::msg::Joymsg::SharedPtr msg) {
        if (msg->buttons.size() > 7){
            if (msg->buttons[4] == 1 && state_flag_.load() == false) {
                state_flag_.store(true);
                origin_flag_.store(false);
                stop_flag_.store(false);
                torque_flag_.store(false);
                ready_flag_.store(false);
                if(state_flag_.load()){
                    RCLCPP_INFO(this->get_logger(), "Set RL");
                }
                stop_count = 0;
            }
            if (msg->buttons[3] == 1 && origin_flag_.load() == false) {
                origin_flag_.store(true);
                state_flag_.store(false);
                stop_flag_.store(false);
                torque_flag_.store(false);
                ready_flag_.store(false);
                for(const auto& motor : motors_) {
                    motor->target_pos_deg.store(motor->zero_point * 180.0 / PI);
                }
                if(origin_flag_.load()){
                    RCLCPP_INFO(this->get_logger(), "Set Origin");
                }
                stop_count = 0;
            }
            if (msg->buttons[0] == 1 && stop_flag_.load() == false) {
                stop_flag_.store(true);
                origin_flag_.store(false);
                state_flag_.store(false);
                torque_flag_.store(false);
                ready_flag_.store(false);
                if(stop_flag_.load()){
                    RCLCPP_INFO(this->get_logger(), "Set stop");
                }
            }
            if (msg->buttons[1] == 1 && torque_flag_.load() == false) {
                torque_flag_.store(true);
                stop_flag_.store(false);
                origin_flag_.store(false);
                state_flag_.store(false);
                ready_flag_.store(false);
                if(torque_flag_.load()){
                    RCLCPP_INFO(this->get_logger(), "Set torque control");
                }
                stop_count = 0;
            }
            if (msg->buttons[7] == 1 && ready_flag_.load() == false) {
                ready_flag_.store(true);
                stop_flag_.store(false);
                origin_flag_.store(false);
                state_flag_.store(false);
                torque_flag_.store(false);
                for(const auto& motor : motors_) {
                    motor->target_pos_deg.store(motor->ready_pos * 180.0 / PI);
                }
                if(ready_flag_.load()){
                    RCLCPP_INFO(this->get_logger(), "Set ready");
                }

                stop_count = 0;
            }
        }
        
        if (msg->cross.size() > 1) {
            if (msg->cross[0] == -1){
                RCLCPP_INFO(this->get_logger(), "Shutdown flag received. Shutting down imudata node...");
                run_flag_.store(false);
                std::thread([](){ rclcpp::shutdown(); }).detach();
                return;
            }
        }

        if (torque_flag_.load() == true){
            if (msg->axes.size() > 1) {
                commands_[0] = (std::abs(msg->axes[1]) < 0.1) ? 0.0 : -msg->axes[1] * 1.0;
            }
            if (msg->axes.size() > 0) {
                commands_[1] = (std::abs(msg->axes[0]) < 0.1) ? 0.0 : -msg->axes[0] * 0.5;
            }
            if (msg->axes.size() > 2) {
                commands_[2] = (std::abs(msg->axes[2]) < 0.1) ? 0.0 : -msg->axes[2] * 0.5;
            }
        }
        else{
            commands_ = {0.0f, 0.0f, 0.0f};
        }
    }

    void action_callback(const msg_interfaces::msg::Action::SharedPtr msg) {
        if (msg->action.size() != motors_.size()) {
            RCLCPP_WARN(this->get_logger(), "Received action size (%zu) does not match motor count (%zu).", msg->action.size(), motors_.size());
            return;
        }

        std::lock_guard<std::mutex> lock(motor_data_mutex_);
        for (size_t i = 0; i < motors_.size(); ++i) {
            motors_[i]->target_pos.store(static_cast<double>(msg->action[i]));
            // motors_[i]->target_pos.store(-0.3);
        }
    }

    void imu_callback(const msg_interfaces::msg::Imudata2::SharedPtr msg) {
        if (msg->angvel.size() != ang_vel_.size()) {
            RCLCPP_WARN(this->get_logger(), "Received angvel size (%zu) does not match motor count (%zu).", msg->angvel.size(), ang_vel_.size());
            return;
        }
        if (msg->projection_gravity.size() != projection_gravity_.size()) {
            RCLCPP_WARN(this->get_logger(), "Received projection_gravity size (%zu) does not match motor count (%zu).", msg->projection_gravity.size(), projection_gravity_.size());
            return;
        }

        ang_vel_ = msg->angvel;
        projection_gravity_ = msg->projection_gravity;
    }
    
    // --- 유틸리티 함수 ---
    void set_thread_affinity(int core, int priority, const std::string& thread_name) {
        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        CPU_SET(core, &cpuset);
        if (pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset) != 0) {
             RCLCPP_ERROR(this->get_logger(), "pthread_setaffinity_np for %s failed: %s", thread_name.c_str(), strerror(errno));
        }
        struct sched_param param = { .sched_priority = priority };
        if (sched_setscheduler(0, SCHED_FIFO, &param) != 0) {
            RCLCPP_ERROR(this->get_logger(), "sched_setscheduler for %s failed: %s", thread_name.c_str(), strerror(errno));
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

    int TX_Reset(int sock, const auto& motor_ptr) {
        struct can_frame frame;
        frame.can_id = motor_ptr->can_id;
        frame.can_dlc = 8;
        frame.data[0] = 0x76;
        frame.data[1] = 0x00;
        frame.data[2] = 0x00;
        frame.data[3] = 0x00;
        frame.data[4] = 0x00;
        frame.data[5] = 0x00;
        frame.data[6] = 0x00;
        frame.data[7] = 0x00;
        
        if(write(sock, &frame, sizeof(frame)) > 0) {
            cout << "motor reset" << endl;
        }

        return 0;
    }

    int TX_Stop(int sock, const auto& motor_ptr) {
        struct can_frame frame;
        frame.can_id = motor_ptr->can_id;
        frame.can_dlc = 8;
        frame.data[0] = 0x80;
        frame.data[1] = 0x00;
        frame.data[2] = 0x00;
        frame.data[3] = 0x00;
        frame.data[4] = 0x00;
        frame.data[5] = 0x00;
        frame.data[6] = 0x00;
        frame.data[7] = 0x00;
        
        if(write(sock, &frame, sizeof(frame)) > 0) motor_ptr->tx_scount++;

        return 0;
    }

    int TX_Pos_state(int sock, const auto& motor_ptr) {
        struct can_frame frame;
        frame.can_id = motor_ptr->can_id;
        frame.can_dlc = 8;
        frame.data[0] = 0x92;
        frame.data[1] = 0x00;
        frame.data[2] = 0x00;
        frame.data[3] = 0x00;
        frame.data[4] = 0x00;
        frame.data[5] = 0x00;
        frame.data[6] = 0x00;
        frame.data[7] = 0x00;
        
        if(write(sock, &frame, sizeof(frame)) > 0) motor_ptr->tx_scount++;

        return 0;
    }
    
    int TX_Pos_ctrl(int sock, const auto& motor_ptr) {
        int32_t target_pos_deg_int = static_cast<int32_t>(motor_ptr->target_pos_deg.load() * 100.0);
        uint16_t speed_limit = motor_ptr->max_speed;

        struct can_frame frame;
        frame.can_id = motor_ptr->can_id;
        frame.can_dlc = 8;
        frame.data[0] = 0xA4;
        frame.data[1] = 0x00;
        frame.data[2] = static_cast<uint8_t>(speed_limit);
        frame.data[3] = static_cast<uint8_t>(speed_limit >> 8);
        frame.data[4] = static_cast<uint8_t>(target_pos_deg_int);
        frame.data[5] = static_cast<uint8_t>(target_pos_deg_int >> 8);
        frame.data[6] = static_cast<uint8_t>(target_pos_deg_int >> 16);
        frame.data[7] = static_cast<uint8_t>(target_pos_deg_int >> 24);
        
        if(write(sock, &frame, sizeof(frame)) > 0) motor_ptr->tx_cmd_count++;

        return 0;
    }

    int TX_Tq_ctrl(int sock, const auto& motor_ptr) {
        int32_t torque = static_cast<int32_t>(motor_ptr->motor_torque * 100.0);

        struct can_frame frame;
        frame.can_id = motor_ptr->can_id;
        frame.can_dlc = 8;
        frame.data[0] = 0xA1;
        frame.data[1] = 0x00;
        frame.data[2] = 0x00;
        frame.data[3] = 0x00;
        frame.data[4] = static_cast<uint8_t>(torque);
        frame.data[5] = static_cast<uint8_t>(torque >> 8);
        frame.data[6] = 0x00;
        frame.data[7] = 0x00;
        
        if(write(sock, &frame, sizeof(frame)) > 0) motor_ptr->tx_cmd_count++;

        return 0;
    }

    int TX_state(int sock, const auto& motor_ptr) {
        struct can_frame frame;
        frame.can_id = motor_ptr->can_id;
        frame.can_dlc = 8;
        frame.data[0] = 0x9C;
        frame.data[1] = 0x00;
        frame.data[2] = 0x00;
        frame.data[3] = 0x00;
        frame.data[4] = 0x00;
        frame.data[5] = 0x00;
        frame.data[6] = 0x00;
        frame.data[7] = 0x00;
        
        if(write(sock, &frame, sizeof(frame)) > 0) motor_ptr->tx_pcount++;

        return 0;
    }

    int RX_state(const auto& motor_ptr, struct can_frame frame) {
        std::lock_guard<std::mutex> lock(motor_ptr->state_mutex);

        motor_ptr->temp = static_cast<int8_t>(frame.data[1]);
        motor_ptr->cur = static_cast<int16_t>((frame.data[3] << 8) | frame.data[2]);
        motor_ptr->vel = static_cast<int16_t>((frame.data[5] << 8) | frame.data[4]);
        if(motor_ptr->can_id == 0x142 || motor_ptr->can_id == 0x143 || motor_ptr->can_id == 0x152 || motor_ptr->can_id == 0x154){
            motor_ptr->rad_vel = deg2rad(-motor_ptr->vel);
        }
        else{
            motor_ptr->rad_vel = deg2rad(motor_ptr->vel);
        }
        motor_ptr->rx_pcount++;

        return 0;
    }
    
    int RX_Pos(const auto& motor_ptr, struct can_frame frame) {
        std::lock_guard<std::mutex> lock(motor_ptr->state_mutex);
        
        int32_t pos_val;
        memcpy(&pos_val, &frame.data[4], 4);
        motor_ptr->pos_deg = pos_val * 0.01f;
        if(motor_ptr->can_id == 0x142 || motor_ptr->can_id == 0x143 || motor_ptr->can_id == 0x152 || motor_ptr->can_id == 0x154){
            motor_ptr->pos = deg2rad(-motor_ptr->pos_deg);
        }
        else{
            motor_ptr->pos = deg2rad(motor_ptr->pos_deg);
        }
        
        motor_ptr->rx_scount++;

        return 0;
    }

    // --- CAN 통신 스레드 ---
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
                    if (stop_flag_.load() == true){
                        if (loop_counter % (TX_FREQ_HZ / 50) == 1 && stop_count < 20) {
                            TX_Stop(sock, motor_ptr);
                            stop_count++;
                        }
                    }

                    else if(state_flag_.load() == true){
                        if (loop_counter % (TX_FREQ_HZ / 50) == 1) { // 50Hz
                            TX_Pos_state(sock, motor_ptr);
                        }

                        if (loop_counter % (TX_FREQ_HZ / 50) == 4) { // 50Hz
                            TX_state(sock, motor_ptr);
                        }
                    }

                    else if(torque_flag_.load() == true){
                        if (loop_counter % (TX_FREQ_HZ / 50) == 1) { // 50Hz
                            TX_Pos_state(sock, motor_ptr);
                        }

                        if (loop_counter % (TX_FREQ_HZ / 50) == 4) { // 50Hz
                            // TX_Tq_ctrl(sock, motor_ptr);
                            TX_state(sock, motor_ptr);
                        }
                    }

                    else if(origin_flag_.load() == true){
                        if (loop_counter % (TX_FREQ_HZ / 50) == 1) { // 50Hz
                            TX_Pos_state(sock, motor_ptr);
                        }

                        if (loop_counter % (TX_FREQ_HZ / 50) == 4) { // 50Hz
                            // if (motor_ptr->can_id == 0x141 || motor_ptr->can_id == 0x142 || motor_ptr->can_id == 0x143 || motor_ptr->can_id == 0x144 || 
                            //     motor_ptr->can_id == 0x151 || motor_ptr->can_id == 0x152 || motor_ptr->can_id == 0x153 || motor_ptr->can_id == 0x154) {
                            //     if (motor_ptr->reset_flag == true) {
                            //         reset_count++;
                            //         if(reset_count < 50){
                            //             if (reset_count % 5 == 1){
                            //                 TX_Reset(sock, motor_ptr);
                            //             }
                            //             else {
                            //                 TX_Pos_state(sock, motor_ptr);
                            //             }
                            //         }
                            //         else{
                            //             motor_ptr->reset_flag = false;
                            //             reset_count = 0;
                            //         }
                            //     }
                            //     else{
                            //         TX_Pos_ctrl(sock, motor_ptr);
                            //     }
                            // }
                            // else TX_state(sock, motor_ptr);

                            if (motor_ptr->reset_flag == true) {
                                reset_count++;
                                if(reset_count < 50){
                                    if (reset_count % 5 == 1){
                                        TX_Reset(sock, motor_ptr);
                                    }
                                    else {
                                        TX_Pos_state(sock, motor_ptr);
                                    }
                                }
                                else{
                                    motor_ptr->reset_flag = false;
                                    reset_count = 0;
                                }
                            }
                            else{
                                TX_Pos_ctrl(sock, motor_ptr);
                            }
                        }
                    }

                    else if(ready_flag_.load() == true){
                        if (loop_counter % (TX_FREQ_HZ / 50) == 1) { // 50Hz
                            TX_Pos_state(sock, motor_ptr);
                        }

                        if (loop_counter % (TX_FREQ_HZ / 50) == 4) { // 50Hz
                            TX_Pos_ctrl(sock, motor_ptr);
                        }
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
                filters.push_back({ .can_id = motor_ptr->can_id + 0x100, .can_mask = CAN_SFF_MASK });
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
                            if (motor_ptr->can_id == frame.can_id - 0x100) {
                                if (frame.data[0] == 0x9C || frame.data[0] == 0xA1 || frame.data[0] == 0xA4) {
                                    RX_state(motor_ptr, frame);
                                } else if (frame.data[0] == 0x92) {
                                    RX_Pos(motor_ptr, frame);
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

    // --- 제어 및 발행 스레드 ---
    void control_and_publish_thread(int core) {
        set_thread_affinity(core, 94, "control_pub");
        RCLCPP_INFO(this->get_logger(), "Control & Publish thread started (Core %d)", core);
        
        auto next_cycle = std::chrono::steady_clock::now();
        int loop_counter = 0;
        
        while(run_flag_.load()) {
            if (origin_flag_.load() == true){
                for (const auto& motor_ptr : motors_) {
                    if (motor_ptr->can_id == 0x154) {
                        if(motor_ptr->pos_deg >= 357 && motor_ptr->pos_deg < 360){
                            motor_ptr->reset_flag = true;
                            motor_ptr->target_pos_deg.store(0);
                        }
                        else if(motor_ptr->pos_deg > 300 && motor_ptr->pos_deg < 355){
                            motor_ptr->target_pos_deg.store(360);
                        } 
                    }
                }
            }

            if (torque_flag_.load() == true || state_flag_.load() == true){
                for (const auto& motor_ptr : motors_) {
                    if(motor_ptr->can_id == 0x142 || motor_ptr->can_id == 0x143 || motor_ptr->can_id == 0x152 || motor_ptr->can_id == 0x154){
                        motor_ptr->target_pos_deg.store(rad2deg(-motor_ptr->target_pos.load()));
                    }
                    else{
                        motor_ptr->target_pos_deg.store(rad2deg(motor_ptr->target_pos.load()));
                    }
                }
            }

            if (torque_flag_.load() == true || state_flag_.load() == true){
                for (size_t i = 0; i < motors_.size(); ++i) {
                    motors_[i]->target_torque = (motors_[i]->kp * (motors_[i]->target_pos.load() - joint_pos[i]) + motors_[i]->kd * (0 - joint_vel[i]));
                    motors_[i]->filtered_torque = std::clamp(motors_[i]->target_torque, -motors_[i]->max_torque, motors_[i]->max_torque);
                }
            }

            loop_counter++;

            float motor4_pos_rad =  motors_[9 ]->pos;
            float motor5_pos_rad =  motors_[11]->pos;
            float motor10_pos_rad = motors_[8 ]->pos;
            float motor11_pos_rad = motors_[10]->pos;
            float motor4_vel_rad =  motors_[9 ]->rad_vel;
            float motor5_vel_rad =  motors_[11]->rad_vel;
            float motor10_vel_rad = motors_[8 ]->rad_vel;
            float motor11_vel_rad = motors_[10]->rad_vel;
            float ankle_pitch_r =  motors_[9 ]->filtered_torque;
            float ankle_roll_r =   motors_[11]->filtered_torque;
            float ankle_pitch_l =  motors_[8 ]->filtered_torque;
            float ankle_roll_l =   motors_[10]->filtered_torque;

            motor2_r << motor4_pos_rad, -motor5_pos_rad;
            tie(ankle_rp_r, ankle_Jac_r) = ankle_fk(motor2_r(0), motor2_r(1), 'r');
            motor2_l << motor10_pos_rad, -motor11_pos_rad;
            tie(ankle_rp_l, ankle_Jac_l) = ankle_fk(motor2_l(0), motor2_l(1), 'l');

            motor2_vel_r << motor4_vel_rad, -motor5_vel_rad;
            ankle_rp_vel_r = (ankle_Jac_r * motor2_vel_r);
            motor2_vel_l << motor10_vel_rad, -motor11_vel_rad;
            ankle_rp_vel_l = (ankle_Jac_l * motor2_vel_l);

            motor2_tor_r << ankle_roll_r, ankle_pitch_r;
            ankle_rp_tor_r = (ankle_Jac_r.transpose() * motor2_tor_r);
            ankle_rp_tor_r(1) = -ankle_rp_tor_r(1);
            motor2_tor_l << ankle_roll_l, ankle_pitch_l;
            ankle_rp_tor_l = (ankle_Jac_l.transpose() * motor2_tor_l);
            ankle_rp_tor_l(1) = -ankle_rp_tor_l(1);
            

            for (size_t i = 0; i < motors_.size(); ++i) {
                if(i == 9){
                    joint_pos[i] = ankle_rp_r(1);
                    joint_vel[i] = ankle_rp_vel_r(1);
                    motors_[i]->motor_torque = ankle_rp_tor_r(0)/motors_[i]->tc;
                }
                else if(i == 11){
                    joint_pos[i] = ankle_rp_r(0);
                    joint_vel[i] = ankle_rp_vel_r(0);
                    motors_[i]->motor_torque = ankle_rp_tor_r(1)/motors_[i]->tc;
                }
                else if(i == 8){
                    joint_pos[i] = ankle_rp_l(1);
                    joint_vel[i] = ankle_rp_vel_l(1);
                    motors_[i]->motor_torque = ankle_rp_tor_l(0)/motors_[i]->tc;
                }
                else if(i == 10){
                    joint_pos[i] = ankle_rp_l(0);
                    joint_vel[i] = ankle_rp_vel_l(0);
                    motors_[i]->motor_torque = ankle_rp_tor_l(1)/motors_[i]->tc;
                }
                else{
                    joint_pos[i] = motors_[i]->pos;
                    joint_vel[i] = motors_[i]->rad_vel;
                    if(motors_[i]->can_id == 0x142 || motors_[i]->can_id == 0x143 || motors_[i]->can_id == 0x152 || motors_[i]->can_id == 0x154){
                        motors_[i]->motor_torque = -motors_[i]->filtered_torque/motors_[i]->tc;
                    }
                    else{
                        motors_[i]->motor_torque = motors_[i]->filtered_torque/motors_[i]->tc;
                    }
                }
            }

            publish_observations();
            publish_test();
        
            // --- 콘솔 출력 ---
            if (loop_counter % (CONTROL_PUB_FREQ_HZ / 10) == 0) { // 10Hz 
                auto now_sec = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now().time_since_epoch()).count();

                std::cout << "\r\n--- Motor Status (" << std::fixed << std::setprecision(2) << now_sec << "s) ---" 
                          << "ori: " <<  origin_flag_.load() 
                          << "   rl: " << state_flag_.load() 
                          << "   tor: " << torque_flag_.load() 
                          << "   stop: " << stop_flag_.load() << "   " << stop_count<< std::endl;
                
                std::cout << "joint : ";
                for (size_t i = 0; i < motors_.size(); ++i) {
                    std::cout << joint_pos[i] << "   ";
                }
                std::cout << std::endl;

                for (const auto& motor_ptr : motors_) {
                    std::cout << "  [ID 0x" << std::hex << motor_ptr->can_id << std::dec << "]"
                              << " 위치: " << std::fixed << std::setw(6) << std::setprecision(1) << (motor_ptr->pos_deg) << "°"
                              << " | rad: " << std::setw(5) << motor_ptr->pos << " rad"
                              << " | 속도: " << std::setw(5) << motor_ptr->vel << " rpm"
                              << " | 전류: " << std::setw(5) << motor_ptr->cur / 100.0 << " A"
                              << " | 토크: " << std::setw(5) << motor_ptr->cur / 100.0 * motor_ptr->tc << " Nm"
                              << " | 온도: " << std::setw(3) << static_cast<int>(motor_ptr->temp) << "°C"
                              << " | 목표: " << std::setw(5) << std::setprecision(1) << motor_ptr->target_pos_deg.load() << "°"
                              << " | 목표: " << std::setw(5) << std::setprecision(1) << motor_ptr->filtered_torque << "Nm"
                              << " | 목표: " << std::setw(5) << std::setprecision(1) << motor_ptr->motor_torque << "A"
                              << " | TX(S/P/C): " << motor_ptr->tx_scount.load() << "/" << motor_ptr->tx_pcount.load() << "/" << motor_ptr->tx_cmd_count.load()
                              << " | RX(S/P): " << motor_ptr->rx_scount.load() << "/" << motor_ptr->rx_pcount.load() << "        " << motor_ptr->reset_flag
                              <<"\n" << std::flush;
                }
            }

            next_cycle += CONTROL_PUB_PERIOD;
            std::this_thread::sleep_until(next_cycle);
        }
        publish_observations();
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

// --- main 함수 ---
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotorControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    system("sudo ip link set can0 down");
    // system("sudo ip link set can1 down");
    system("sudo ip link set can2 down");
    return 0;
}




// 권한 부여 필요
// sudo setcap cap_net_raw,cap_sys_nice+eip $(find ~/mujoco_ws/install -type f -name motor_test4)


// pos 명령 주는 명령어
// ros2 topic pub --once /motor_action msg_interfaces/msg/Action "{action: [90.0, 90.0, 90.0, 90.0, 90.0, 90.0, 90.0, 90.0, 90.0, 90.0, 90.0, 90.0]}"

// sudo ./set_active_cores.sh 1 2 3 4 5 6 7 8