// #include <iostream>
// #include <cstring>
// #include <cmath>
// #include <chrono>
// #include <thread>
// #include <csignal>
// #include <iomanip>
// #include <string>
// #include <cstdint>
// #include <mutex>
// #include <atomic>
// #include <vector>
// #include <memory>
// #include <tuple> 

// // 시스템 헤더 파일
// #include <cstdlib>
// #include <fcntl.h>
// #include <sys/socket.h>
// #include <sys/ioctl.h>
// #include <net/if.h>
// #include <unistd.h>
// #include <linux/can.h>
// #include <linux/can/raw.h>
// #include <sys/select.h>
// #include <Eigen/Eigen>
// #include <Eigen/Dense>

// // 리얼타임 관련 헤더
// #include <sched.h>
// #include <sys/mman.h>
// #include <pthread.h>

// // ROS 2 헤더
// #include "rclcpp/rclcpp.hpp"
// #include "msg_interfaces/msg/joymsg.hpp"
// #include "msg_interfaces/msg/observation.hpp"
// #include "msg_interfaces/msg/action.hpp"
// #include "msg_interfaces/msg/imudata2.hpp"

// // --- 설정 변수 ---
// const char* CAN0_INTERFACE = "can0";
// const char* CAN2_INTERFACE = "can2";
// #define PI 3.1415926
// using namespace std;
// using namespace Eigen;

// // 주기 설정 (Hz)
// constexpr int TX_FREQ_HZ = 500;
// constexpr int RX_FREQ_HZ = 1000;
// constexpr int CONTROL_PUB_FREQ_HZ = 200; // 제어 및 발행 주기

// // 시간 변환
// constexpr auto TX_PERIOD = std::chrono::microseconds(1000000 / TX_FREQ_HZ);
// constexpr auto RX_PERIOD_US = 1000000 / RX_FREQ_HZ;
// constexpr auto CONTROL_PUB_PERIOD = std::chrono::microseconds(1000000 / CONTROL_PUB_FREQ_HZ);


// // --- Motor 구조체 정의 ---
// struct Motor {
//     canid_t can_id;
//     uint16_t max_speed;
//     int32_t zero_point;
//     float max_torque;
//     float tc;
//     bool reset_flag;

//     // 모터의 현재 상태 (CAN RX 스레드에서 업데이트)
//     std::atomic<float> pos{0.0f};
//     std::atomic<int8_t> temp{0};
//     std::atomic<int16_t> cur{0};
//     std::atomic<int16_t> vel{0};
//     std::atomic<float> rad_vel{0.0f};
    
//     // 모터의 목표 위치 (Action Subscriber에서 업데이트)
//     std::atomic<double> target_pos_deg{0.0};
    
//     // 통신 카운트
//     std::atomic<int> tx_scount{0};
//     std::atomic<int> rx_scount{0};
//     std::atomic<int> tx_pcount{0};
//     std::atomic<int> rx_pcount{0};
//     std::atomic<int> tx_cmd_count{0};

//     Motor(canid_t id, uint16_t speed, int32_t zp, float torque, float torque_const, bool reset_flag)
//         : can_id(id), max_speed(speed), zero_point(zp), max_torque(torque), tc(torque_const), reset_flag(reset_flag) {}
// };

// class MotorControllerNode : public rclcpp::Node
// {
// public:
//     MotorControllerNode() : Node("motor_controller_node")
//     {
//         // can 모듈 활성화 --------------------------------------------------------------------------------
//         system("sudo ip link set can0 up type can bitrate 1000000");
//         // system("sudo ip link set can1 up type can bitrate 1000000");
//         system("sudo ip link set can2 up type can bitrate 1000000");
//         // ---------------------------------------------------------------------------------------

//         // 시작 시간 초기화 (clock 함수용)
//         start_time_ = std::chrono::steady_clock::now();
        
//         // --- ROS 2 인터페이스 초기화 ---
//         obs_pub_ = this->create_publisher<msg_interfaces::msg::Observation>("mujoco_obs", 10);
        
//         joy_sub_ = this->create_subscription<msg_interfaces::msg::Joymsg>(
//             "joystick_state", 10, std::bind(&MotorControllerNode::joy_callback, this, std::placeholders::_1));
            
//         action_sub_ = this->create_subscription<msg_interfaces::msg::Action>(
//             "action", 10, std::bind(&MotorControllerNode::action_callback, this, std::placeholders::_1));
        
//         imu_sub_ = this->create_subscription<msg_interfaces::msg::Imudata2>(
//             "imu_data", 10, std::bind(&MotorControllerNode::imu_callback, this, std::placeholders::_1));

//         RCLCPP_INFO(this->get_logger(), "ROS 2 Publisher & Subscribers initialized.");
        
//         // 변수 설정
//         joint_pos.assign(num_joints, 0.0f);
//         joint_vel.assign(num_joints, 0.0f);
//         motor_torque.assign(num_joints, 0.0f);
//         target_pos.assign(num_joints, 0.0f);
//         ankle_Jac_r << 0,0,
//                        0,0;
//         ankle_Jac_l << 0,0,
//                        0,0;
        
//         // --- 실시간 메모리 락 ---
//         if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
//             RCLCPP_ERROR(this->get_logger(), "mlockall failed: %s", strerror(errno));
//         }

//         // --- 모터 객체 생성 ---
//         initialize_motors();
//         RCLCPP_INFO(this->get_logger(), "%zu motors initialized.", motors_.size());

//         // --- 통신 및 제어 스레드 시작 ---
//         run_flag_.store(true);
//         threads_.emplace_back(&MotorControllerNode::can_tx_task, this, 1, CAN0_INTERFACE, 0x141, 0x146);
//         threads_.emplace_back(&MotorControllerNode::can_tx_task, this, 1, CAN2_INTERFACE, 0x151, 0x156);
//         threads_.emplace_back(&MotorControllerNode::can_rx_task, this, 2, CAN0_INTERFACE, 0x141, 0x146);
//         threads_.emplace_back(&MotorControllerNode::can_rx_task, this, 3, CAN2_INTERFACE, 0x151, 0x156);
//         threads_.emplace_back(&MotorControllerNode::control_and_publish_thread, this, 0);
        
//         RCLCPP_INFO(this->get_logger(), "RMD-X Multi-motor controller node started.");
//         RCLCPP_INFO(this->get_logger(), "Waiting for Action commands on topic 'motor_action'...");
//     }

//     ~MotorControllerNode()
//     {
//         run_flag_.store(false);
//         RCLCPP_INFO(this->get_logger(), "Stopping threads...");

//         for (auto& t : threads_) {
//             if (t.joinable()) {
//                 t.join();
//             }
//         }
        
//         send_stop_command_to_all();
//         munlockall();
//         RCLCPP_INFO(this->get_logger(), "All threads and motors stopped. Node shutting down.");
//     }

// Matrix2f ankle_J(float R, float P, float t1, float t2, char rl){
//     float l1 = 46.25;
//     float l2_l = 298.457;
//     float l2_s = 210.997;
//     float l3 = 300;
//     float w1 = 64.75;
//     float Cx = 37;
//     float Cy = 55.5;
//     float Cz = -22.5;
//     float origin_t = (31.29*PI/180);

//     Matrix3f ROF;
//     Vector3f OB1;
//     Vector3f OB2;
//     Vector3f FC1;
//     Vector3f FC2;
//     Vector3f OF;
//     Vector3f BC1;
//     Vector3f BC2;
//     Vector3f AB1;
//     Vector3f AB2;
//     Vector2f A;
//     Vector3f B1;
//     Vector3f B2;
//     Matrix2f C;

//     ROF << cos(P), sin(P)*sin(R), cos(R)*sin(P),
//             0,        cos(R),       -sin(R),
//             -sin(P), cos(P)*sin(R), cos(P)*cos(R);

//     if (rl == 'r'){
//         OB1 << l1*cos(t1), w1, -l1*sin(t1);
//         OB2 << l1*cos(t2), -w1, -87.5-l1*sin(t2);
//     }
//     else if (rl == 'l'){
//         OB1 << l1*cos(t1), w1, -87.5-l1*sin(t1);
//         OB2 << l1*cos(t2), -w1, -l1*sin(t2);
//     }

//     FC1 << Cx, Cy, Cz;
//     FC2 << Cx, -Cy, Cz;

//     OF << 0,0,-l3;

//     BC1 = OF + ROF*FC1 - OB1;
//     BC2 = OF + ROF*FC2 - OB2;

//     AB1 << l1*cos(t1+origin_t), 0, -l1*sin(t1+origin_t);
//     AB2 << l1*cos(t2+origin_t), 0, -l1*sin(t2+origin_t);
//     // AB1 << l1*cos(t1), 0, -l1*sin(t1);
//     // AB2 << l1*cos(t2), 0, -l1*sin(t2);

//     Vector3f X(1,0,0);
//     Vector3f Y(0,1,0);

//     A << Y.dot(AB1.cross(BC1)),
//          Y.dot(AB2.cross(BC2));

//     B1 << FC1.cross(BC1);
//     B2 << FC2.cross(BC2);

//     C << X.dot(B1), Y.dot(B1),
//          X.dot(B2), Y.dot(B2);

//     Matrix2f J_1;
//     J_1.row(0) = C.row(0) / A(0);
//     J_1.row(1) = C.row(1) / A(1);
//     Matrix2f J = J_1.inverse();

//     return J;
// }

// Vector2f ankle_ik(float R, float P, char rl){
//     float l1 = 46.25;
//     float l2_l = 298.457;
//     float l2_s = 210.997;
//     float l3 = 300;
//     float w1 = 64.75;
//     float Cx = 37;
//     float Cy = 55.5;
//     float Cz = -22.5;
//     float origin_t = (31.29*PI/180);
    
//     double A1;
//     double B1;
//     double C1;
//     double A2;
//     double B2;
//     double C2;

//     if(rl == 'r'){
//         A1 = Cx*Cx + Cy*Cy + Cz*Cz + l1*l1 - l2_l*l2_l + l3*l3 + w1*w1 - Cz*l3*cos(P + R) - Cy*l3*sin(P + R) - 2*Cy*w1*cos(R) + 2*Cx*l3*sin(P) - Cz*l3*cos(P - R) + 2*Cz*w1*sin(R) + Cy*l3*sin(P - R);
//         B1 = - 2*l1*(l3*sin((1043*PI)/6000) + Cx*cos((1043*PI)/6000)*cos(P) + Cx*sin((1043*PI)/6000)*sin(P) + Cz*cos((1043*PI)/6000)*cos(R)*sin(P) - Cz*sin((1043*PI)/6000)*cos(P)*cos(R) + Cy*cos((1043*PI)/6000)*sin(P)*sin(R) - Cy*sin((1043*PI)/6000)*cos(P)*sin(R));
//         C1 = 2*l1*(Cx*sin((1043*PI)/6000)*cos(P) - Cx*cos((1043*PI)/6000)*sin(P) - l3*cos((1043*PI)/6000) + Cz*cos((1043*PI)/6000)*cos(P)*cos(R) + Cy*cos((1043*PI)/6000)*cos(P)*sin(R) + Cz*sin((1043*PI)/6000)*cos(R)*sin(P) + Cy*sin((1043*PI)/6000)*sin(P)*sin(R));

//         A2 = Cx*Cx + Cy*Cy + Cz*Cz + l1*l1 - l2_s*l2_s + l3*l3 + w1*w1 + (175*Cz*cos(P + R))/2 - 175*l3 - (175*Cy*sin(P + R))/2 - 175*Cx*sin(P) + (175*Cz*cos(P - R))/2 + (175*Cy*sin(P - R))/2  - Cz*l3*cos(P + R) + Cy*l3*sin(P + R) - 2*Cy*w1*cos(R) + 2*Cx*l3*sin(P) - Cz*l3*cos(P - R) - 2*Cz*w1*sin(R) - Cy*l3*sin(P - R) + 30625/4;
//         B2 = -l1*(2*l3*sin((1043*PI)/6000) - 175*sin((1043*PI)/6000) + 2*Cx*cos((1043*PI)/6000)*cos(P) + 2*Cx*sin((1043*PI)/6000)*sin(P) + 2*Cz*cos((1043*PI)/6000)*cos(R)*sin(P) - 2*Cz*sin((1043*PI)/6000)*cos(P)*cos(R) - 2*Cy*cos((1043*PI)/6000)*sin(P)*sin(R) + 2*Cy*sin((1043*PI)/6000)*cos(P)*sin(R));
//         C2 = l1*(175*cos((1043*PI)/6000) - 2*l3*cos((1043*PI)/6000) - 2*Cx*cos((1043*PI)/6000)*sin(P) + 2*Cx*sin((1043*PI)/6000)*cos(P) + 2*Cz*cos((1043*PI)/6000)*cos(P)*cos(R) - 2*Cy*cos((1043*PI)/6000)*cos(P)*sin(R) + 2*Cz*sin((1043*PI)/6000)*cos(R)*sin(P) - 2*Cy*sin((1043*PI)/6000)*sin(P)*sin(R));
//     }
//     else if(rl == 'l'){
//         A1 = Cx*Cx + Cy*Cy + Cz*Cz + l1*l1 - l2_s*l2_s + l3*l3 + w1*w1 + (175*Cz*cos(P + R))/2- 175*l3+ (175*Cy*sin(P + R))/2- 175*Cx*sin(P)+ (175*Cz*cos(P - R))/2- (175*Cy*sin(P - R))/2- Cz*l3*cos(P + R)- Cy*l3*sin(P + R)- 2*Cy*w1*cos(R)+ 2*Cx*l3*sin(P)- Cz*l3*cos(P - R)+ 2*Cz*w1*sin(R)+ Cy*l3*sin(P - R)+ 30625/4;
//         B1 = -l1*(2*l3*sin((1043*PI)/6000) - 175*sin((1043*PI)/6000) + 2*Cx*cos((1043*PI)/6000)*cos(P) + 2*Cx*sin((1043*PI)/6000)*sin(P) + 2*Cz*cos((1043*PI)/6000)*cos(R)*sin(P) - 2*Cz*sin((1043*PI)/6000)*cos(P)*cos(R) + 2*Cy*cos((1043*PI)/6000)*sin(P)*sin(R) - 2*Cy*sin((1043*PI)/6000)*cos(P)*sin(R));
//         C1 = l1*(175*cos((1043*PI)/6000) - 2*l3*cos((1043*PI)/6000) - 2*Cx*cos((1043*PI)/6000)*sin(P) + 2*Cx*sin((1043*PI)/6000)*cos(P) + 2*Cz*cos((1043*PI)/6000)*cos(P)*cos(R) + 2*Cy*cos((1043*PI)/6000)*cos(P)*sin(R) + 2*Cz*sin((1043*PI)/6000)*cos(R)*sin(P) + 2*Cy*sin((1043*PI)/6000)*sin(P)*sin(R));
        
//         A2 = Cx*Cx + Cy*Cy + Cz*Cz + l1*l1 - l2_l*l2_l + l3*l3 + w1*w1 - Cz*l3*cos(P + R) + Cy*l3*sin(P + R) - 2*Cy*w1*cos(R) + 2*Cx*l3*sin(P) - Cz*l3*cos(P - R) - 2*Cz*w1*sin(R) - Cy*l3*sin(P - R);
//         B2 = (2*Cz*l1*sin((1043*PI)/6000)*cos(P)*cos(R) - 2*Cx*l1*cos((1043*PI)/6000)*cos(P) - 2*Cx*l1*sin((1043*PI)/6000)*sin(P) - 2*Cz*l1*cos((1043*PI)/6000)*cos(R)*sin(P) - 2*l1*l3*sin((1043*PI)/6000) + 2*Cy*l1*cos((1043*PI)/6000)*sin(P)*sin(R) - 2*Cy*l1*sin((1043*PI)/6000)*cos(P)*sin(R));
//         C2 = (2*Cx*l1*sin((1043*PI)/6000)*cos(P) - 2*Cx*l1*cos((1043*PI)/6000)*sin(P) - 2*l1*l3*cos((1043*PI)/6000) + 2*Cz*l1*cos((1043*PI)/6000)*cos(P)*cos(R) - 2*Cy*l1*cos((1043*PI)/6000)*cos(P)*sin(R) + 2*Cz*l1*sin((1043*PI)/6000)*cos(R)*sin(P) - 2*Cy*l1*sin((1043*PI)/6000)*sin(P)*sin(R));
//     }

//     Vector2f t1;
//     t1 << (-2*C1 + sqrt(4*C1*C1 - 4*(A1-B1)*(A1+B1)))/(2*(A1-B1)), 
//           (-2*C1 - sqrt(4*C1*C1 - 4*(A1-B1)*(A1+B1)))/(2*(A1-B1));
    
//     Vector2f t2;
//     t2 << (-2*C2 + sqrt(4*C2*C2 - 4*(A2-B2)*(A2+B2)))/(2*(A2-B2)), 
//           (-2*C2 - sqrt(4*C2*C2 - 4*(A2-B2)*(A2+B2)))/(2*(A2-B2));

//     Vector2f theta;
//     if(abs(2*atan(t1(0))*180/PI) < 80){
//         theta(0) = 2*atan(t1(0));
//     }
//     else if(abs(2*atan(t1(1))*180/PI) < 80){
//         theta(0) = 2*atan(t1(1));
//     }
//     if(abs(2*atan(t2(0))*180/PI) < 80){
//         theta(1) = 2*atan(t2(0));
//     }
//     else if(2*atan(t2(1)*180/PI) < 80){
//         theta(1) = 2*atan(t2(1));
//     }

//     return theta;
// }

// tuple<Vector2f,Matrix2f> ankle_fk(float t1, float t2, char rl){
//     Vector2f theta;
//     float alpha = 1;
//     Matrix2f jac;
//     Vector2f error;
//     Vector2f ankle_rp(0,0); 
//     for(int i = 0; i < 12; i++){
//         theta = ankle_ik(ankle_rp(0), ankle_rp(1), rl);
//         error << t1-theta(0), t2-theta(1);
//         jac = ankle_J(ankle_rp(0), ankle_rp(1), theta(0), theta(1), rl);
//         ankle_rp += alpha*jac*error;
//         if(sqrt(pow(error(0),2) + pow(error(1),2)) < 1e-4){
//             break;
//         }
//     }
//     return make_tuple(ankle_rp, jac);
// }

// private:
//     // --- ROS 2 멤버 ---
//     rclcpp::Publisher<msg_interfaces::msg::Observation>::SharedPtr obs_pub_;
//     rclcpp::Subscription<msg_interfaces::msg::Joymsg>::SharedPtr joy_sub_;
//     rclcpp::Subscription<msg_interfaces::msg::Action>::SharedPtr action_sub_;
//     rclcpp::Subscription<msg_interfaces::msg::Imudata2>::SharedPtr imu_sub_;

//     // --- 클래스 멤버 ---
//     std::vector<std::unique_ptr<Motor>> motors_;
//     std::vector<std::thread> threads_;
//     std::atomic<bool> run_flag_{false};
//     std::atomic<bool> motors_enabled_{false}; // 조이스틱으로 모터 활성화/비활성화
//     std::mutex motor_data_mutex_; // 모터 데이터 접근 동기화를 위한 뮤텍스
//     int prev_tracking_state_ = 0;
//     int tracking_state_ =0;

//     std::chrono::steady_clock::time_point start_time_; // Clock 함수용 시작 시간
    
//     const int num_joints = 12;
//     std::vector<float> ang_vel_{0.0f, 0.0f, 0.0f};
//     std::vector<float> projection_gravity_{0.0f, 0.0f, 0.0f};
//     std::vector<float> commands_{0.0f, 0.0f, 0.0f};
//     std::vector<float> joint_pos;
//     std::vector<float> joint_vel;
//     std::vector<float> motor_torque;
//     std::vector<float> target_pos;
    
//     Vector2f ankle_rp_r{0,0};
//     Vector2f ankle_rp_vel_r{0,0};
//     Vector2f ankle_rp_tor_r{0,0};
//     Vector2f motor2_r{0,0};
//     Vector2f motor2_vel_r{0,0};
//     Vector2f motor2_tor_r{0,0};
//     Matrix2f ankle_Jac_r;
//     Vector2f lpf_ankle_rp_vel_r{0,0};

//     Vector2f ankle_rp_l{0,0};
//     Vector2f ankle_rp_vel_l{0,0};
//     Vector2f ankle_rp_tor_l{0,0};
//     Vector2f motor2_l{0,0};
//     Vector2f motor2_vel_l{0,0};
//     Vector2f motor2_tor_l{0,0};
//     Matrix2f ankle_Jac_l;
//     Vector2f lpf_ankle_rp_vel_l{0,0};
//     float alpha = 0.9;
//     bool origin = false;

//     // --- 모터 초기화 ---
//     void initialize_motors() {
//         // can0
//         motors_.push_back(std::make_unique<Motor>(0x141, 2,  0, 30, 30.0f/4.0f, false));
//         motors_.push_back(std::make_unique<Motor>(0x142, 2,  0, 30, 30.0f/4.0f, false));
//         motors_.push_back(std::make_unique<Motor>(0x143, 5,  0, 50, 50.0f/6.7f, false));
//         motors_.push_back(std::make_unique<Motor>(0x144, 11, 0, 50, 50.0f/6.7f, false));
//         motors_.push_back(std::make_unique<Motor>(0x145, 5,  0, 8 , 4.0f/3.6f , false));
//         motors_.push_back(std::make_unique<Motor>(0x146, 5,  0, 8 , 4.0f/3.6f , false));
//         // can2
//         motors_.push_back(std::make_unique<Motor>(0x151, 2,  0, 30, 30.0f/4.0f, false));
//         motors_.push_back(std::make_unique<Motor>(0x152, 2,  0, 30, 30.0f/4.0f, false));
//         motors_.push_back(std::make_unique<Motor>(0x153, 5,  0, 50, 50.0f/6.7f, false));
//         motors_.push_back(std::make_unique<Motor>(0x154, 11, 0, 50, 50.0f/6.7f, false));
//         motors_.push_back(std::make_unique<Motor>(0x156, 5,  0, 8 , 4.0f/3.6f , false));
//         motors_.push_back(std::make_unique<Motor>(0x155, 5,  0, 8 , 4.0f/3.6f , false));   

//         // 초기 목표 위치를 현재 위치로 설정
//         for(const auto& motor : motors_) {
//             motor->target_pos_deg.store(motor->zero_point * 180.0 / PI);
//         }
//     }

//     double deg2rad(double degree){
//         double radian = degree*PI/180;
//         return radian;
//     }

//     float lpf(float pre_data, float data, float alpha){
//         float lpf_data = pre_data*alpha + data*(1-alpha);
//         return lpf_data;
//     }

//     std::vector<float> _get_clock() {
//         auto now = std::chrono::steady_clock::now();
//         double total_time = std::chrono::duration_cast<std::chrono::duration<double>>(now - start_time_).count();
        
//         double phase = 2 * (0.15 + 0.3);
//         double local_phi = fmod(total_time, phase);
//         double phi = local_phi / phase;
        
//         std::vector<float> clock = {
//             (float)std::sin(2 * M_PI * phi), 
//             (float)std::cos(2 * M_PI * phi)
//         };
//         return clock;
//     }

//     void publish_observations() {
//         auto obs_msg = msg_interfaces::msg::Observation();
//         obs_msg.header.stamp = this->get_clock()->now();
//         obs_msg.joint_pos.resize(motors_.size());
//         obs_msg.joint_vel.resize(motors_.size());
        
//         for (size_t i = 0; i < motors_.size(); ++i) {
//             // obs_msg.joint_pos[i] = motors_[i]->pos.load();
//             // obs_msg.joint_vel[i] = motors_[i]->rad_vel.load();
//             obs_msg.joint_pos[i] = joint_pos[i];
//             obs_msg.joint_vel[i] = joint_vel[i];
//         }
//         obs_msg.base_ang_vel = ang_vel_;
//         obs_msg.gravity = projection_gravity_;
//         obs_msg.commands = commands_;

//         std::vector<float> clock = _get_clock();
//         obs_msg.clock = clock;
//         obs_msg.flag = run_flag_;
        
//         obs_pub_->publish(obs_msg);
//     }

//     // --- ROS 2 콜백 함수 ---
//     void joy_callback(const msg_interfaces::msg::Joymsg::SharedPtr msg) {
//         if (msg->buttons.size() > 4){
//             tracking_state_ = msg->buttons[4];
//             if (msg->buttons[4] == 1 && prev_tracking_state_ == 0) {
//                 motors_enabled_ = !motors_enabled_.load();
//                 if(motors_enabled_.load()){
//                     RCLCPP_INFO(this->get_logger(), "Motors ENABLED by joystick.");
//                 }
//                 else{
//                     RCLCPP_INFO(this->get_logger(), "Motors DISABLED by joystick.");
//                 }
//             }
//             prev_tracking_state_ = tracking_state_;
//         }
        
//         if (msg->cross.size() > 1) {
//             if (msg->cross[0] == -1){
//                 RCLCPP_INFO(this->get_logger(), "Shutdown flag received. Shutting down imudata node...");
//                 run_flag_.store(false);
//                 std::thread([](){ rclcpp::shutdown(); }).detach();
//                 return;
//             }
//         }

//         if (motors_enabled_ == true){
//             if (msg->axes.size() > 1) {
//                 commands_[0] = (std::abs(msg->axes[1]) < 0.1) ? 0.0 : -msg->axes[1] * 1.0;
//             }
//             if (msg->axes.size() > 0) {
//                 commands_[1] = (std::abs(msg->axes[0]) < 0.1) ? 0.0 : -msg->axes[0] * 0.5;
//             }
//             if (msg->axes.size() > 2) {
//                 commands_[2] = (std::abs(msg->axes[2]) < 0.1) ? 0.0 : -msg->axes[2] * 0.5;
//             }
//         }
//         else{
//             commands_ = {0.0f, 0.0f, 0.0f};
//         }
//     }

//     void action_callback(const msg_interfaces::msg::Action::SharedPtr msg) {
//         if (msg->action.size() != motors_.size()) {
//             RCLCPP_WARN(this->get_logger(), "Received action size (%zu) does not match motor count (%zu).", msg->action.size(), motors_.size());
//             return;
//         }

//         std::lock_guard<std::mutex> lock(motor_data_mutex_);
//         for (size_t i = 0; i < motors_.size(); ++i) {
//             // motors_[i]->target_pos_deg.store(static_cast<double>(msg->action[i]));
//             target_pos = msg->action;
//         }
//     }

//     void imu_callback(const msg_interfaces::msg::Imudata2::SharedPtr msg) {
//         if (msg->angvel.size() != ang_vel_.size()) {
//             RCLCPP_WARN(this->get_logger(), "Received angvel size (%zu) does not match motor count (%zu).", msg->angvel.size(), ang_vel_.size());
//             return;
//         }
//         if (msg->projection_gravity.size() != projection_gravity_.size()) {
//             RCLCPP_WARN(this->get_logger(), "Received projection_gravity size (%zu) does not match motor count (%zu).", msg->projection_gravity.size(), projection_gravity_.size());
//             return;
//         }

//         ang_vel_ = msg->angvel;
//         projection_gravity_ = msg->projection_gravity;
//     }
    
//     // --- 유틸리티 함수 ---
//     void set_thread_affinity(int core, int priority, const std::string& thread_name) {
//         cpu_set_t cpuset;
//         CPU_ZERO(&cpuset);
//         CPU_SET(core, &cpuset);
//         if (pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset) != 0) {
//              RCLCPP_ERROR(this->get_logger(), "pthread_setaffinity_np for %s failed: %s", thread_name.c_str(), strerror(errno));
//         }
//         struct sched_param param = { .sched_priority = priority };
//         if (sched_setscheduler(0, SCHED_FIFO, &param) != 0) {
//             RCLCPP_ERROR(this->get_logger(), "sched_setscheduler for %s failed: %s", thread_name.c_str(), strerror(errno));
//         }
//     }

//     int open_can_socket(const char* interface, const std::vector<struct can_filter>& filters) {
//         int s;
//         struct sockaddr_can addr;
//         struct ifreq ifr;

//         s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
//         if (s < 0) { perror("socket"); return -1; }

//         int flags = fcntl(s, F_GETFL, 0);
//         if (flags < 0) { perror("fcntl get"); close(s); return -1; }
//         if (fcntl(s, F_SETFL, flags | O_NONBLOCK) < 0) { perror("fcntl set"); close(s); return -1; }

//         strcpy(ifr.ifr_name, interface);
//         if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) { perror("ioctl"); close(s); return -1; }

//         addr.can_family = AF_CAN;
//         addr.can_ifindex = ifr.ifr_ifindex;
//         if (bind(s, (struct sockaddr*)&addr, sizeof(addr)) < 0) { perror("bind"); close(s); return -1; }
        
//         if (!filters.empty()) {
//             if (setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, filters.data(), filters.size() * sizeof(struct can_filter)) != 0) {
//                 perror("Setting CAN filters failed");
//                 close(s);
//                 return -1;
//             }
//         }
//         return s;
//     }

//     // --- CAN 통신 스레드 ---
//     void can_tx_task(int core, const char* interface_name, canid_t id_min, canid_t id_max) {
//         set_thread_affinity(core, 98, std::string(interface_name) + "_tx");
//         RCLCPP_INFO(this->get_logger(), "%s TX thread started (Core %d)", interface_name, core);
        
//         int sock = open_can_socket(interface_name, {});
//         if (sock < 0) { 
//             RCLCPP_ERROR(this->get_logger(), "%s TX socket creation failed", interface_name); 
//             return; 
//         }

//         auto next_cycle = std::chrono::steady_clock::now();
//         int loop_counter = 0;

//         while (run_flag_.load()) {
//             loop_counter++;

//             for (const auto& motor_ptr : motors_) {
//                 if (motor_ptr->can_id >= id_min && motor_ptr->can_id <= id_max) {
//                     // 상태 요청
//                     if (loop_counter % (TX_FREQ_HZ / 50) == 1) { // 50Hz
//                         struct can_frame frame = { .can_id = motor_ptr->can_id, .can_dlc = 8, .data = {0x92, 0,0,0,0,0,0,0} }; // Multi-motor angle
//                         if(write(sock, &frame, sizeof(frame)) > 0) motor_ptr->tx_scount++;
//                     }
//                     if (loop_counter % (TX_FREQ_HZ / 50) == 4) { // 50Hz
//                         if (motor_ptr->reset_flag == true){
//                             struct can_frame frame;
//                             frame.can_id = motor_ptr->can_id;
//                             frame.can_dlc = 8;
//                             frame.data[0] = 0x76;
//                             frame.data[1] = 0x00;
//                             frame.data[2] = 0x00;
//                             frame.data[3] = 0x00;
//                             frame.data[4] = 0x00;
//                             frame.data[5] = 0x00;
//                             frame.data[6] = 0x00;
//                             frame.data[7] = 0x00;
                            
//                             if(write(sock, &frame, sizeof(frame)) > 0) motor_ptr->tx_cmd_count++;

//                         }
//                         else if(motor_ptr->can_id == 0x141 ||
//                                 motor_ptr->can_id == 0x142 ||
//                                 motor_ptr->can_id == 0x143 ||
//                                 motor_ptr->can_id == 0x144 ||
//                                 motor_ptr->can_id == 0x151 ||
//                                 motor_ptr->can_id == 0x152 ||
//                                 motor_ptr->can_id == 0x153 ||
//                                 motor_ptr->can_id == 0x154)
//                             {
//                             int32_t target_pos_deg_int = static_cast<int32_t>(motor_ptr->target_pos_deg.load() * 100.0);
//                             uint16_t speed_limit = motor_ptr->max_speed;

//                             struct can_frame frame;
//                             frame.can_id = motor_ptr->can_id;
//                             frame.can_dlc = 8;
//                             frame.data[0] = 0xA4;
//                             frame.data[1] = 0x00;
//                             frame.data[2] = static_cast<uint8_t>(speed_limit);
//                             frame.data[3] = static_cast<uint8_t>(speed_limit >> 8);
//                             frame.data[4] = static_cast<uint8_t>(target_pos_deg_int);
//                             frame.data[5] = static_cast<uint8_t>(target_pos_deg_int >> 8);
//                             frame.data[6] = static_cast<uint8_t>(target_pos_deg_int >> 16);
//                             frame.data[7] = static_cast<uint8_t>(target_pos_deg_int >> 24);
                            
//                             if(write(sock, &frame, sizeof(frame)) > 0) motor_ptr->tx_cmd_count++;
//                         }
//                         else{
//                             struct can_frame frame = { .can_id = motor_ptr->can_id, .can_dlc = 8, .data = {0x9C, 0,0,0,0,0,0,0} }; // Motor status 2
//                             if(write(sock, &frame, sizeof(frame)) > 0) motor_ptr->tx_pcount++;
//                         }
//                     }

//                     // // 위치 제어 명령 전송
//                     // if (motors_enabled_.load()) {
//                     //     int16_t target_pos_deg_int = static_cast<int32_t>(motor_ptr->target_pos_deg.load() * 100.0);
//                     //     uint16_t speed_limit = motor_ptr->max_speed;

//                     //     struct can_frame frame;
//                     //     frame.can_id = motor_ptr->can_id;
//                     //     frame.can_dlc = 8;
//                     //     frame.data[0] = 0xA4; // Position control 2 (with speed limit)
//                     //     frame.data[1] = 0x00;
//                     //     frame.data[2] = speed_limit & 0xFF;
//                     //     frame.data[3] = (speed_limit >> 8) & 0xFF;
//                     //     memcpy(&frame.data[4], &target_pos_deg_int, 4);
                        
//                     //     if(write(sock, &frame, sizeof(frame)) > 0) motor_ptr->tx_cmd_count++;
//                     // }
//                 }
//             }
//             next_cycle += TX_PERIOD;
//             std::this_thread::sleep_until(next_cycle);
//         }
//         close(sock);
//         RCLCPP_INFO(this->get_logger(), "%s TX thread finished.", interface_name);
//     }

//     void can_rx_task(int core, const char* interface_name, canid_t id_min, canid_t id_max) {
//         set_thread_affinity(core, 99, std::string(interface_name) + "_rx");
//         RCLCPP_INFO(this->get_logger(), "%s RX thread started (Core %d)", interface_name, core);
        
//         std::vector<struct can_filter> filters;
//         for (const auto& motor_ptr : motors_) {
//             if (motor_ptr->can_id >= id_min && motor_ptr->can_id <= id_max) {
//                 filters.push_back({ .can_id = motor_ptr->can_id + 0x100, .can_mask = CAN_SFF_MASK });
//             }
//         }
//         int sock = open_can_socket(interface_name, filters);
//         if (sock < 0) { 
//             RCLCPP_ERROR(this->get_logger(), "%s RX socket creation failed", interface_name);
//             return;
//         }

//         while (run_flag_.load()) {
//             fd_set read_fds;
//             FD_ZERO(&read_fds);
//             FD_SET(sock, &read_fds);
//             struct timeval timeout = {0, RX_PERIOD_US};

//             if (select(sock + 1, &read_fds, NULL, NULL, &timeout) > 0) {
//                 struct can_frame frame;
//                 while (read(sock, &frame, sizeof(frame)) > 0) { 
//                     if (frame.can_dlc == 8) {
//                         for (const auto& motor_ptr : motors_) {
//                             if (motor_ptr->can_id == frame.can_id - 0x100) {
//                                 if (frame.data[0] == 0x9C || frame.data[0] == 0xA1 || frame.data[0] == 0xA4) {
//                                     motor_ptr->temp.store(static_cast<int8_t>(frame.data[1]));
//                                     motor_ptr->cur.store(static_cast<int16_t>((frame.data[3] << 8) | frame.data[2]));
//                                     motor_ptr->vel.store(static_cast<int16_t>((frame.data[5] << 8) | frame.data[4]));
//                                     motor_ptr->rad_vel.store(deg2rad(motor_ptr->vel));
//                                     motor_ptr->rx_pcount++;
//                                 } else if (frame.data[0] == 0x92) {
//                                     int32_t pos_val;
//                                     memcpy(&pos_val, &frame.data[4], 4);
//                                     motor_ptr->pos.store(deg2rad(pos_val * 0.01f));
                                    
//                                     motor_ptr->rx_scount++;
//                                 }
//                                 break; 
//                             }

//                         }
//                     }
//                 }
//             }
//         }
//         close(sock);
//         RCLCPP_INFO(this->get_logger(), "%s RX thread finished.", interface_name);
//     }


//     // --- 제어 및 발행 스레드 ---
//     void control_and_publish_thread(int core) {
//         set_thread_affinity(core, 95, "control_pub");
//         RCLCPP_INFO(this->get_logger(), "Control & Publish thread started (Core %d)", core);
        
//         auto next_cycle = std::chrono::steady_clock::now();
//         int loop_counter = 0;
        
//         while(run_flag_.load()) {
//             // if (origin == false){
//             //     for (const auto& motor_ptr : motors_) {
//             //         if (motor_ptr->can_id == 0x143 || motor_ptr->can_id == 0x153 ||
//             //             motor_ptr->can_id == 0x144 || motor_ptr->can_id == 0x154) {
//             //             if(motor_ptr->pos <= 120 && motor_ptr->pos > 2){
//             //                 motor_ptr->target_pos_deg.store(0);
//             //             }
//             //             else if(motor_ptr->pos >= -120 && motor_ptr->pos < -2){
//             //                 motor_ptr->target_pos_deg.store(0);
//             //             }
//             //             else if(motor_ptr->pos <= -352 && motor_ptr->pos > -360){
//             //                 TX_MY_ACT_RESET(can_socs[i], motor_ptr->can_id);
//             //                 usleep(500000); //0.5s
//             //                 motor_ptr->target_pos_deg.store(0);
//             //             }
//             //             else if(motor_ptr->pos < -300 && motor_ptr->pos > -352){
//             //                 motor_ptr->target_pos_deg.store(-360);
//             //             } 
//             //             else if(motor_ptr->pos >= 357 && motor_ptr->pos < 360){
//             //                 TX_MY_ACT_RESET(can_socs[i], motor_ptr->can_id);
//             //                 usleep(500000);
//             //                 motor_ptr->target_pos_deg.store(0);
//             //             }
//             //             else if(motor_ptr->pos > 300 && motor_ptr->pos < 355){
//             //                 motor_ptr->target_pos_deg.store(360);
//             //             } 
//             //         }
//             //     }





//             //     for (int i = 0; i < num_motors*2; ++i) {
//             //         if (i == 3 || i == 4 || i == 9 || i == 10){
//             //             if(motors[i].pos <= 120 && motors[i].pos > 2){
//             //                 motors[i].zero_point = 0;
//             //             }
//             //             else if(motors[i].pos >= -120 && motors[i].pos < -2){
//             //                 motors[i].zero_point = 0;
//             //             }
//             //             else if(motors[i].pos <= -352 && motors[i].pos > -360){
//             //                 TX_MY_ACT_RESET(can_socs[i], motors[i].can_id);
//             //                 usleep(500000); //0.5s
//             //                 motors[i].zero_point = 0;
//             //             }
//             //             else if(motors[i].pos < -300 && motors[i].pos > -352){
//             //                 motors[i].zero_point = -36000;
//             //             } 
//             //             else if(motors[i].pos >= 357 && motors[i].pos < 360){
//             //                 TX_MY_ACT_RESET(can_socs[i], motors[i].can_id);
//             //                 usleep(500000);
//             //                 motors[i].zero_point = 0;
//             //             }
//             //             else if(motors[i].pos > 300 && motors[i].pos < 355){
//             //                 motors[i].zero_point = 36000;
//             //             } 
//             //         }
//             //     }
//             // }
//             loop_counter++;

//             float motor4_pos_rad =  motors_[4 ]->pos.load();
//             float motor5_pos_rad =  motors_[5 ]->pos.load();
//             float motor10_pos_rad = motors_[10]->pos.load();
//             float motor11_pos_rad = motors_[11]->pos.load();
//             float motor4_vel_rad =  motors_[4 ]->rad_vel.load();
//             float motor5_vel_rad =  motors_[5 ]->rad_vel.load();
//             float motor10_vel_rad = motors_[10]->rad_vel.load();
//             float motor11_vel_rad = motors_[11]->rad_vel.load();
//             float motor4_tor =  motors_[4 ]->tc * motors_[4 ]->cur.load();
//             float motor5_tor =  motors_[5 ]->tc * motors_[5 ]->cur.load();
//             float motor10_tor = motors_[10]->tc * motors_[10]->cur.load();
//             float motor11_tor = motors_[11]->tc * motors_[11]->cur.load();

//             motor2_r << motor4_pos_rad, -motor5_pos_rad;
//             tie(ankle_rp_r, ankle_Jac_r) = ankle_fk(motor2_r(0), motor2_r(1), 'r');
//             motor2_l << motor10_pos_rad, -motor11_pos_rad;
//             tie(ankle_rp_l, ankle_Jac_l) = ankle_fk(motor2_l(0), motor2_l(1), 'l');

//             motor2_vel_r << motor4_vel_rad, -motor5_vel_rad;
//             ankle_rp_vel_r = (ankle_Jac_r * motor2_vel_r);
//             motor2_vel_l << motor10_vel_rad, -motor11_vel_rad;
//             ankle_rp_vel_l = (ankle_Jac_l * motor2_vel_l);

//             motor2_tor_r << motor4_tor, -motor5_tor;
//             ankle_rp_tor_r = (ankle_Jac_r * motor2_tor_r);
//             motor2_tor_l << motor10_tor, -motor11_tor;
//             ankle_rp_tor_l = (ankle_Jac_r * motor2_tor_l);
            

//             for (size_t i = 0; i < motors_.size(); ++i) {
//                 if(i == 4 || i == 5){
//                     joint_pos[i] = ankle_rp_r(5-i);
//                     joint_vel[i] = ankle_rp_vel_r(5-i);
//                     motor_torque[i] = ankle_rp_tor_r(5-i);
//                 }
//                 else if(i == 10 || i == 11){
//                     joint_pos[i] = ankle_rp_l(11-i);
//                     joint_vel[i] = ankle_rp_vel_l(11-i);
//                     motor_torque[i] = ankle_rp_tor_l(11-i);
//                 }
//                 else{
//                     joint_pos[i] = motors_[i]->pos.load();
//                     joint_vel[i] = motors_[i]->rad_vel.load();
//                     motor_torque[i] = motors_[i]->tc * (motors_[i]->cur.load());
//                 }
//             }

//             publish_observations();

//             // // --- 콘솔 출력 ---
//             if (loop_counter % (CONTROL_PUB_FREQ_HZ / 10) == 0) { // 10Hz 
//                 auto now_sec = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now().time_since_epoch()).count();

//                 std::cout << "\r\n--- Motor Status (" << std::fixed << std::setprecision(2) << now_sec
//                           << "s) | Motors Enabled: " << (motors_enabled_.load() ? "YES" : "NO") << " ---" << std::endl;

//                 for (const auto& motor_ptr : motors_) {
//                     std::cout << "  [ID 0x" << std::hex << motor_ptr->can_id << std::dec << "]"
//                               << " 위치: " << std::fixed << std::setw(6) << std::setprecision(1) << (motor_ptr->pos.load() * 180.0 / PI) << "°"
//                               << " | rad: " << std::setw(5) << motor_ptr->pos.load() << " rad"
//                               << " | 속도: " << std::setw(5) << motor_ptr->vel.load() << " rpm"
//                               << " | 전류: " << std::setw(5) << motor_ptr->cur.load() << " mA"
//                               << " | 온도: " << std::setw(3) << static_cast<int>(motor_ptr->temp.load()) << "°C"
//                               << " | 목표: " << std::setw(5) << std::setprecision(1) << motor_ptr->target_pos_deg.load() << "°"
//                               << " | TX(S/P/C): " << motor_ptr->tx_scount.load() << "/" << motor_ptr->tx_pcount.load() << "/" << motor_ptr->tx_cmd_count.load()
//                               << " | RX(S/P): " << motor_ptr->rx_scount.load() << "/" << motor_ptr->rx_pcount.load()
//                               <<"\n" << std::flush;
//                 }
//             }

//             next_cycle += CONTROL_PUB_PERIOD;
//             std::this_thread::sleep_until(next_cycle);
//         }
//         publish_observations();
//         RCLCPP_INFO(this->get_logger(), "Control & Publish thread finished.");
//     }
    
//     void send_stop_command_to_all() {
//         int sock_can0 = open_can_socket(CAN0_INTERFACE, {});
//         int sock_can2 = open_can_socket(CAN2_INTERFACE, {});
//         if(sock_can0 < 0 && sock_can2 < 0) return;

//         RCLCPP_INFO(this->get_logger(), "Sending STOP command to all motors...");
//         for (const auto& motor_ptr : motors_) {
//             int current_sock = (motor_ptr->can_id < 0x150) ? sock_can0 : sock_can2;
//             if(current_sock < 0) continue;
//             struct can_frame frame = { .can_id = motor_ptr->can_id, .can_dlc = 8, .data = {0x80, 0, 0, 0, 0, 0, 0, 0} };
//             (void)write(current_sock, &frame, sizeof(frame));
//             std::this_thread::sleep_for(std::chrono::milliseconds(5)); 
//         }

//         if(sock_can0 >= 0) close(sock_can0);
//         if(sock_can2 >= 0) close(sock_can2);
//     }
// };

// // --- main 함수 ---
// int main(int argc, char * argv[])
// {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<MotorControllerNode>();
//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     system("sudo ip link set can0 down");
//     // system("sudo ip link set can1 down");
//     system("sudo ip link set can2 down");
//     return 0;
// }

































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
    int32_t zero_point;
    float max_torque;
    float tc;
    bool reset_flag;

    // 모터의 현재 상태 (CAN RX 스레드에서 업데이트)
    std::atomic<float> pos{0.0f};
    std::atomic<int8_t> temp{0};
    std::atomic<int16_t> cur{0};
    std::atomic<int16_t> vel{0};
    std::atomic<float> rad_vel{0.0f};
    
    // 모터의 목표 위치 (Action Subscriber에서 업데이트)
    std::atomic<double> target_pos_deg{0.0};
    
    // 통신 카운트
    std::atomic<int> tx_scount{0};
    std::atomic<int> rx_scount{0};
    std::atomic<int> tx_pcount{0};
    std::atomic<int> rx_pcount{0};
    std::atomic<int> tx_cmd_count{0};

    Motor(canid_t id, uint16_t speed, int32_t zp, float torque, float torque_const, bool reset_flag)
        : can_id(id), max_speed(speed), zero_point(zp), max_torque(torque), tc(torque_const), reset_flag(reset_flag) {}
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
        motor_torque.assign(num_joints, 0.0f);
        target_pos.assign(num_joints, 0.0f);
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
        threads_.emplace_back(&MotorControllerNode::can_tx_task, this, 1, CAN2_INTERFACE, 0x151, 0x156);
        threads_.emplace_back(&MotorControllerNode::can_rx_task, this, 2, CAN0_INTERFACE, 0x141, 0x146);
        threads_.emplace_back(&MotorControllerNode::can_rx_task, this, 3, CAN2_INTERFACE, 0x151, 0x156);
        threads_.emplace_back(&MotorControllerNode::control_and_publish_thread, this, 0);
        
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
    rclcpp::Subscription<msg_interfaces::msg::Joymsg>::SharedPtr joy_sub_;
    rclcpp::Subscription<msg_interfaces::msg::Action>::SharedPtr action_sub_;
    rclcpp::Subscription<msg_interfaces::msg::Imudata2>::SharedPtr imu_sub_;

    // --- 클래스 멤버 ---
    std::vector<std::unique_ptr<Motor>> motors_;
    std::vector<std::thread> threads_;
    std::atomic<bool> run_flag_{false};
    std::atomic<bool> motors_enabled_{false}; // 조이스틱으로 모터 활성화/비활성화
    std::mutex motor_data_mutex_; // 모터 데이터 접근 동기화를 위한 뮤텍스
    int prev_tracking_state_ = 0;
    int tracking_state_ =0;

    std::chrono::steady_clock::time_point start_time_; // Clock 함수용 시작 시간
    
    const int num_joints = 12;
    std::vector<float> ang_vel_{0.0f, 0.0f, 0.0f};
    std::vector<float> projection_gravity_{0.0f, 0.0f, 0.0f};
    std::vector<float> commands_{0.0f, 0.0f, 0.0f};
    std::vector<float> joint_pos;
    std::vector<float> joint_vel;
    std::vector<float> motor_torque;
    std::vector<float> target_pos;
    
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
    bool origin = false;

    // --- 모터 초기화 ---
    void initialize_motors() {
        // can0
        motors_.push_back(std::make_unique<Motor>(0x141, 2,  0, 30, 30.0f/4.0f, false));
        motors_.push_back(std::make_unique<Motor>(0x142, 2,  0, 30, 30.0f/4.0f, false));
        motors_.push_back(std::make_unique<Motor>(0x143, 5,  0, 50, 50.0f/6.7f, false));
        motors_.push_back(std::make_unique<Motor>(0x144, 11, 0, 50, 50.0f/6.7f, false));
        motors_.push_back(std::make_unique<Motor>(0x145, 5,  0, 8 , 4.0f/3.6f , false));
        motors_.push_back(std::make_unique<Motor>(0x146, 5,  0, 8 , 4.0f/3.6f , false));
        // can2
        motors_.push_back(std::make_unique<Motor>(0x151, 2,  0, 30, 30.0f/4.0f, false));
        motors_.push_back(std::make_unique<Motor>(0x152, 2,  0, 30, 30.0f/4.0f, false));
        motors_.push_back(std::make_unique<Motor>(0x153, 5,  0, 50, 50.0f/6.7f, false));
        motors_.push_back(std::make_unique<Motor>(0x154, 11, 0, 50, 50.0f/6.7f, false));
        motors_.push_back(std::make_unique<Motor>(0x156, 5,  0, 8 , 4.0f/3.6f , false));
        motors_.push_back(std::make_unique<Motor>(0x155, 5,  0, 8 , 4.0f/3.6f , false));   

        // 초기 목표 위치를 현재 위치로 설정
        for(const auto& motor : motors_) {
            motor->target_pos_deg.store(motor->zero_point * 180.0 / PI);
        }
    }

    double deg2rad(double degree){
        double radian = degree*PI/180;
        return radian;
    }

    float lpf(float pre_data, float data, float alpha){
        float lpf_data = pre_data*alpha + data*(1-alpha);
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
        auto obs_msg = msg_interfaces::msg::Observation();
        obs_msg.header.stamp = this->get_clock()->now();
        obs_msg.joint_pos.resize(motors_.size());
        obs_msg.joint_vel.resize(motors_.size());
        
        for (size_t i = 0; i < motors_.size(); ++i) {
            // obs_msg.joint_pos[i] = motors_[i]->pos.load();
            // obs_msg.joint_vel[i] = motors_[i]->rad_vel.load();
            obs_msg.joint_pos[i] = joint_pos[i];
            obs_msg.joint_vel[i] = joint_vel[i];
        }
        obs_msg.base_ang_vel = ang_vel_;
        obs_msg.gravity = projection_gravity_;
        obs_msg.commands = commands_;

        std::vector<float> clock = _get_clock();
        obs_msg.clock = clock;
        obs_msg.flag = run_flag_;
        
        obs_pub_->publish(obs_msg);
    }

    // --- ROS 2 콜백 함수 ---
    void joy_callback(const msg_interfaces::msg::Joymsg::SharedPtr msg) {
        if (msg->buttons.size() > 4){
            tracking_state_ = msg->buttons[4];
            if (msg->buttons[4] == 1 && prev_tracking_state_ == 0) {
                motors_enabled_ = !motors_enabled_.load();
                if(motors_enabled_.load()){
                    RCLCPP_INFO(this->get_logger(), "Motors ENABLED by joystick.");
                }
                else{
                    RCLCPP_INFO(this->get_logger(), "Motors DISABLED by joystick.");
                }
            }
            prev_tracking_state_ = tracking_state_;
        }
        
        if (msg->cross.size() > 1) {
            if (msg->cross[0] == -1){
                RCLCPP_INFO(this->get_logger(), "Shutdown flag received. Shutting down imudata node...");
                run_flag_.store(false);
                std::thread([](){ rclcpp::shutdown(); }).detach();
                return;
            }
        }

        if (motors_enabled_ == true){
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
            // motors_[i]->target_pos_deg.store(static_cast<double>(msg->action[i]));
            target_pos = msg->action;
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
        
        if(write(sock, &frame, sizeof(frame)) > 0) motor_ptr->tx_cmd_count++;

        return 0;
    }
    
    int TX_Pos(int sock, const auto& motor_ptr) {
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
                    // 상태 요청
                    if (loop_counter % (TX_FREQ_HZ / 50) == 1) { // 50Hz
                        struct can_frame frame = { .can_id = motor_ptr->can_id, .can_dlc = 8, .data = {0x92, 0,0,0,0,0,0,0} }; // Multi-motor angle
                        if(write(sock, &frame, sizeof(frame)) > 0) motor_ptr->tx_scount++;
                    }
                    if (loop_counter % (TX_FREQ_HZ / 50) == 4) { // 50Hz
                        if(motor_ptr->reset_flag){
                            TX_Reset(sock, motor_ptr);
                        }
                        else if(motor_ptr->can_id == 0x141 ||
                                motor_ptr->can_id == 0x142 ||
                                motor_ptr->can_id == 0x143 ||
                                motor_ptr->can_id == 0x144 ||
                                motor_ptr->can_id == 0x151 ||
                                motor_ptr->can_id == 0x152 ||
                                motor_ptr->can_id == 0x153 ||
                                motor_ptr->can_id == 0x154)
                            {
                            TX_Pos(sock, motor_ptr);
                        }
                        else{
                            struct can_frame frame = { .can_id = motor_ptr->can_id, .can_dlc = 8, .data = {0x9C, 0,0,0,0,0,0,0} }; // Motor status 2
                            if(write(sock, &frame, sizeof(frame)) > 0) motor_ptr->tx_pcount++;
                        }
                    }

                    // // 위치 제어 명령 전송
                    // if (motors_enabled_.load()) {
                    //     int16_t target_pos_deg_int = static_cast<int32_t>(motor_ptr->target_pos_deg.load() * 100.0);
                    //     uint16_t speed_limit = motor_ptr->max_speed;

                    //     struct can_frame frame;
                    //     frame.can_id = motor_ptr->can_id;
                    //     frame.can_dlc = 8;
                    //     frame.data[0] = 0xA4; // Position control 2 (with speed limit)
                    //     frame.data[1] = 0x00;
                    //     frame.data[2] = speed_limit & 0xFF;
                    //     frame.data[3] = (speed_limit >> 8) & 0xFF;
                    //     memcpy(&frame.data[4], &target_pos_deg_int, 4);
                        
                    //     if(write(sock, &frame, sizeof(frame)) > 0) motor_ptr->tx_cmd_count++;
                    // }
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
                                    motor_ptr->temp.store(static_cast<int8_t>(frame.data[1]));
                                    motor_ptr->cur.store(static_cast<int16_t>((frame.data[3] << 8) | frame.data[2]));
                                    motor_ptr->vel.store(static_cast<int16_t>((frame.data[5] << 8) | frame.data[4]));
                                    motor_ptr->rad_vel.store(deg2rad(motor_ptr->vel));
                                    motor_ptr->rx_pcount++;
                                } else if (frame.data[0] == 0x92) {
                                    int32_t pos_val;
                                    memcpy(&pos_val, &frame.data[4], 4);
                                    motor_ptr->pos.store(deg2rad(pos_val * 0.01f));
                                    
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


    // --- 제어 및 발행 스레드 ---
    void control_and_publish_thread(int core) {
        set_thread_affinity(core, 95, "control_pub");
        RCLCPP_INFO(this->get_logger(), "Control & Publish thread started (Core %d)", core);
        
        auto next_cycle = std::chrono::steady_clock::now();
        int loop_counter = 0;
        
        while(run_flag_.load()) {
            // if (origin == false){
            //     for (const auto& motor_ptr : motors_) {
            //         if (motor_ptr->can_id == 0x143 || motor_ptr->can_id == 0x153 ||
            //             motor_ptr->can_id == 0x144 || motor_ptr->can_id == 0x154) {
            //             if(motor_ptr->pos <= 120 && motor_ptr->pos > 2){
            //                 motor_ptr->target_pos_deg.store(0);
            //             }
            //             else if(motor_ptr->pos >= -120 && motor_ptr->pos < -2){
            //                 motor_ptr->target_pos_deg.store(0);
            //             }
            //             else if(motor_ptr->pos <= -352 && motor_ptr->pos > -360){
            //                 TX_MY_ACT_RESET(can_socs[i], motor_ptr->can_id);
            //                 usleep(500000); //0.5s
            //                 motor_ptr->target_pos_deg.store(0);
            //             }
            //             else if(motor_ptr->pos < -300 && motor_ptr->pos > -352){
            //                 motor_ptr->target_pos_deg.store(-360);
            //             } 
            //             else if(motor_ptr->pos >= 357 && motor_ptr->pos < 360){
            //                 TX_MY_ACT_RESET(can_socs[i], motor_ptr->can_id);
            //                 usleep(500000);
            //                 motor_ptr->target_pos_deg.store(0);
            //             }
            //             else if(motor_ptr->pos > 300 && motor_ptr->pos < 355){
            //                 motor_ptr->target_pos_deg.store(360);
            //             } 
            //         }
            //     }





            //     for (int i = 0; i < num_motors*2; ++i) {
            //         if (i == 3 || i == 4 || i == 9 || i == 10){
            //             if(motors[i].pos <= 120 && motors[i].pos > 2){
            //                 motors[i].zero_point = 0;
            //             }
            //             else if(motors[i].pos >= -120 && motors[i].pos < -2){
            //                 motors[i].zero_point = 0;
            //             }
            //             else if(motors[i].pos <= -352 && motors[i].pos > -360){
            //                 TX_MY_ACT_RESET(can_socs[i], motors[i].can_id);
            //                 usleep(500000); //0.5s
            //                 motors[i].zero_point = 0;
            //             }
            //             else if(motors[i].pos < -300 && motors[i].pos > -352){
            //                 motors[i].zero_point = -36000;
            //             } 
            //             else if(motors[i].pos >= 357 && motors[i].pos < 360){
            //                 TX_MY_ACT_RESET(can_socs[i], motors[i].can_id);
            //                 usleep(500000);
            //                 motors[i].zero_point = 0;
            //             }
            //             else if(motors[i].pos > 300 && motors[i].pos < 355){
            //                 motors[i].zero_point = 36000;
            //             } 
            //         }
            //     }
            // }
            loop_counter++;

            float motor4_pos_rad =  motors_[4 ]->pos.load();
            float motor5_pos_rad =  motors_[5 ]->pos.load();
            float motor10_pos_rad = motors_[10]->pos.load();
            float motor11_pos_rad = motors_[11]->pos.load();
            float motor4_vel_rad =  motors_[4 ]->rad_vel.load();
            float motor5_vel_rad =  motors_[5 ]->rad_vel.load();
            float motor10_vel_rad = motors_[10]->rad_vel.load();
            float motor11_vel_rad = motors_[11]->rad_vel.load();
            float motor4_tor =  motors_[4 ]->tc * motors_[4 ]->cur.load();
            float motor5_tor =  motors_[5 ]->tc * motors_[5 ]->cur.load();
            float motor10_tor = motors_[10]->tc * motors_[10]->cur.load();
            float motor11_tor = motors_[11]->tc * motors_[11]->cur.load();

            motor2_r << motor4_pos_rad, -motor5_pos_rad;
            tie(ankle_rp_r, ankle_Jac_r) = ankle_fk(motor2_r(0), motor2_r(1), 'r');
            motor2_l << motor10_pos_rad, -motor11_pos_rad;
            tie(ankle_rp_l, ankle_Jac_l) = ankle_fk(motor2_l(0), motor2_l(1), 'l');

            motor2_vel_r << motor4_vel_rad, -motor5_vel_rad;
            ankle_rp_vel_r = (ankle_Jac_r * motor2_vel_r);
            motor2_vel_l << motor10_vel_rad, -motor11_vel_rad;
            ankle_rp_vel_l = (ankle_Jac_l * motor2_vel_l);

            motor2_tor_r << motor4_tor, -motor5_tor;
            ankle_rp_tor_r = (ankle_Jac_r * motor2_tor_r);
            motor2_tor_l << motor10_tor, -motor11_tor;
            ankle_rp_tor_l = (ankle_Jac_r * motor2_tor_l);
            

            for (size_t i = 0; i < motors_.size(); ++i) {
                if(i == 4 || i == 5){
                    joint_pos[i] = ankle_rp_r(5-i);
                    joint_vel[i] = ankle_rp_vel_r(5-i);
                    motor_torque[i] = ankle_rp_tor_r(5-i);
                }
                else if(i == 10 || i == 11){
                    joint_pos[i] = ankle_rp_l(11-i);
                    joint_vel[i] = ankle_rp_vel_l(11-i);
                    motor_torque[i] = ankle_rp_tor_l(11-i);
                }
                else{
                    joint_pos[i] = motors_[i]->pos.load();
                    joint_vel[i] = motors_[i]->rad_vel.load();
                    motor_torque[i] = motors_[i]->tc * (motors_[i]->cur.load());
                }
            }

            publish_observations();

            // // --- 콘솔 출력 ---
            if (loop_counter % (CONTROL_PUB_FREQ_HZ / 10) == 0) { // 10Hz 
                auto now_sec = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now().time_since_epoch()).count();

                std::cout << "\r\n--- Motor Status (" << std::fixed << std::setprecision(2) << now_sec
                          << "s) | Motors Enabled: " << (motors_enabled_.load() ? "YES" : "NO") << " ---" << std::endl;

                for (const auto& motor_ptr : motors_) {
                    std::cout << "  [ID 0x" << std::hex << motor_ptr->can_id << std::dec << "]"
                              << " 위치: " << std::fixed << std::setw(6) << std::setprecision(1) << (motor_ptr->pos.load() * 180.0 / PI) << "°"
                              << " | rad: " << std::setw(5) << motor_ptr->pos.load() << " rad"
                              << " | 속도: " << std::setw(5) << motor_ptr->vel.load() << " rpm"
                              << " | 전류: " << std::setw(5) << motor_ptr->cur.load() << " mA"
                              << " | 온도: " << std::setw(3) << static_cast<int>(motor_ptr->temp.load()) << "°C"
                              << " | 목표: " << std::setw(5) << std::setprecision(1) << motor_ptr->target_pos_deg.load() << "°"
                              << " | TX(S/P/C): " << motor_ptr->tx_scount.load() << "/" << motor_ptr->tx_pcount.load() << "/" << motor_ptr->tx_cmd_count.load()
                              << " | RX(S/P): " << motor_ptr->rx_scount.load() << "/" << motor_ptr->rx_pcount.load()
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