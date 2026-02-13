// C++ 표준 라이브러리
#include <cstdint>
#include <iostream>
#include <sstream>
#include <string>
#include <iomanip>
#include <cmath>

// STL 컨테이너 및 스마트 포인터
#include <queue>
#include <vector>
#include <memory>

// 멀티스레딩
#include <chrono>
#include <thread>
#include <mutex>
#include <atomic>

// 리얼타임 관련
#include <sched.h>
#include <sys/mman.h>
#include <pthread.h>

// Eigen
#include <Eigen/Eigen>

// VectorNav IMU
#include "vn/sensors.h"

// ROS2
#include <rclcpp/rclcpp.hpp>
#include "msg_interfaces/msg/imudata.hpp"
#include "msg_interfaces/msg/joymsg.hpp"

#define SEC_TO_NSEC 1000000000
#define NSEC_TO_SEC 0.000000001
#define PI 3.1415926

using namespace std;
using namespace Eigen;
using namespace vn::math;
using namespace vn::sensors;
using namespace vn::protocol::uart;
using namespace vn::xplat;

// 주기 설정 (Hz)
constexpr int IMU_FREQ_HZ = 800;
constexpr int PUBLISH_FREQ_HZ = 200; // 발행 주기 (800Hz로 읽고 200Hz로 발행)
// constexpr int PUBLISH_FREQ_HZ = 50; // 발행 주기 (800Hz로 읽고 50Hz로 발행)

// 시간 변환
constexpr auto IMU_PERIOD = std::chrono::microseconds(1000000 / IMU_FREQ_HZ);
constexpr auto PUBLISH_PERIOD = std::chrono::microseconds(1000000 / PUBLISH_FREQ_HZ);

struct IMU {
    uint64_t timeStartup;
    vec3f yawPitchRoll;
    vec4f quaternion;
    vec3f angularRate;
    vec3f acceleration;
    
    // 처리된 데이터
    Vector3f euler_angle;
    Vector3f projection_angularRate;
    Vector3f projection_acceleration;
    Vector3f projection_gravity_array;
    
    std::chrono::steady_clock::time_point timestamp;
};

double deg2rad(double degree){
    double radian = degree*PI/180;
    return radian;
}

Matrix3f quaternionToRotationMatrix(float x, float y, float z, float w) {
    Matrix3f rotationMatrix;

    rotationMatrix(0,0) = 1 - 2 * y * y - 2 * z * z;
    rotationMatrix(0,1) = 2 * x * y - 2 * z * w;
    rotationMatrix(0,2) = 2 * x * z + 2 * y * w;

    rotationMatrix(1,0) = 2 * x * y + 2 * z * w;
    rotationMatrix(1,1) = 1 - 2 * x * x - 2 * z * z;
    rotationMatrix(1,2) = 2 * y * z - 2 * x * w;

    rotationMatrix(2,0) = 2 * x * z - 2 * y * w;
    rotationMatrix(2,1) = 2 * y * z + 2 * x * w;
    rotationMatrix(2,2) = 1 - 2 * x * x - 2 * y * y;

    return rotationMatrix;
}

Vector3f calOri(const Matrix3f& R) {
    Vector3f rpy;
    rpy[0] = atan2(R(2, 1), R(2, 2)); // roll
    rpy[1] = atan2(-R(2, 0), sqrt(pow(R(2, 1), 2) + pow(R(2, 2), 2))); // pitch
    rpy[2] = atan2(R(1, 0), R(0, 0)); // yaw
    return rpy;
}

float lpf(float pre_data, float data, float alpha){
    float lpf_data = pre_data*alpha + data*(1-alpha);
    return lpf_data;
}

Vector3f lpfvec(Vector3f pre_data, Vector3f data, float alpha){
    Vector3f lpf_data = pre_data*alpha + data*(1-alpha);
    return lpf_data;
}

tuple<Vector3f, Vector3f, Vector3f, Vector3f> RX_IMU_data(IMU data, Matrix3f Framefix, Vector3f gravity){
    Matrix3f quatToeuler = quaternionToRotationMatrix(data.quaternion.x, data.quaternion.y, data.quaternion.z, data.quaternion.w);
    Matrix3f euler_Matrix = Framefix * quatToeuler;
    Matrix3f transposed_Euler = euler_Matrix.transpose();
    
    Vector3f acceleration(-data.acceleration.x, -data.acceleration.y, -data.acceleration.z);
    Vector3f angularRate(data.angularRate.x, data.angularRate.y, data.angularRate.z);

    Vector3f Projection_angularRate = euler_Matrix * angularRate;
    Vector3f Projection_Acceleration = transposed_Euler * acceleration;
    Vector3f Projection_gravity_Array = transposed_Euler * gravity;

    return make_tuple(calOri(euler_Matrix), Projection_angularRate, Projection_Acceleration, Projection_gravity_Array);
}

class ImuControllerNode : public rclcpp::Node
{
public:
    ImuControllerNode() : Node("imu_data_node")
    {
        // ROS 2 인터페이스 초기화
        imudata_pub_ = this->create_publisher<msg_interfaces::msg::Imudata>("/imu_data", 10);
        RCLCPP_INFO(this->get_logger(), "IMU Controller Node initialized.");
        joy_sub_ = this->create_subscription<msg_interfaces::msg::Joymsg>(
            "joystick_state", 10, std::bind(&ImuControllerNode::joystick_callback, this, std::placeholders::_1));

        // 실시간 메모리 락
        if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
            RCLCPP_ERROR(this->get_logger(), "mlockall failed: %s", strerror(errno));
        }

        // IMU 센서 초기화
        initialize_imu();
        
        // 변수 초기화
        initialize_variables();

        // 스레드 시작
        run_flag_.store(true);
        threads_.emplace_back(&ImuControllerNode::imu_data_thread, this, 1);
        threads_.emplace_back(&ImuControllerNode::publish_thread, this, 0);
        
        RCLCPP_INFO(this->get_logger(), "IMU real-time controller started at 800Hz.");
    }

    ~ImuControllerNode()
    {
        run_flag_.store(false);
        RCLCPP_INFO(this->get_logger(), "Stopping IMU threads...");

        // VectorNav 센서 정리
        if (vs_initialized_) {
            vs_->unregisterAsyncPacketReceivedHandler();
            vs_->disconnect();
            RCLCPP_INFO(this->get_logger(), "IMU set down.");
        }

        for (auto& t : threads_) {
            if (t.joinable()) {
                t.join();
            }
        }
        
        munlockall();
        RCLCPP_INFO(this->get_logger(), "IMU Controller Node shut down.");
    }

private:
    // ROS 2 멤버
    rclcpp::Publisher<msg_interfaces::msg::Imudata>::SharedPtr imudata_pub_;
    rclcpp::Subscription<msg_interfaces::msg::Joymsg>::SharedPtr joy_sub_;

    // 클래스 멤버
    std::vector<std::thread> threads_;
    std::atomic<bool> run_flag_{false};
    std::mutex imu_data_mutex_;

    // IMU 관련 변수
    std::unique_ptr<VnSensor> vs_;
    bool vs_initialized_ = false;
    std::queue<IMU> imuQueue_;
    std::mutex queueMutex_;
    
    // 필터링 변수
    Vector3f pre_euler_angle_{0,0,0};
    Vector3f pre_projection_angularRate_{0,0,0};
    Vector3f pre_projection_acceleration_{0,0,0};
    Vector3f pre_projection_gravity_array_{0,0,0};
    // float alpha_ = 0.978f;
    float alpha_ = 0.0f;
    
    
    // 좌표 변환 매트릭스
    Matrix3f framefix_;
    Vector3f gravity_;
    
    // 통계 변수
    std::atomic<int> imu_count_{0};
    std::atomic<int> publish_count_{0};

    void joystick_callback(const msg_interfaces::msg::Joymsg::SharedPtr msg) {
        if (msg->cross.size() > 1) {
            if (msg->cross[0] == -1){
                RCLCPP_INFO(this->get_logger(), "Shutdown flag received. Shutting down imudata node...");
                run_flag_.store(false);
                std::thread([](){ rclcpp::shutdown(); }).detach();
                return;
            }
        }
    }

    // IMU 콜백 함수 (정적 함수로 정의하고 this 포인터 전달)
    static void asciiOrBinaryAsyncMessageReceived(void* userData, Packet& p, size_t index) {
        ImuControllerNode* node = static_cast<ImuControllerNode*>(userData);
        node->handleImuCallback(p, index);
    }

    void handleImuCallback(Packet& p, size_t index) {
        if (p.type() == Packet::TYPE_BINARY)
        {
            if (!p.isCompatible(
                COMMONGROUP_TIMESTARTUP | COMMONGROUP_YAWPITCHROLL | COMMONGROUP_QUATERNION | COMMONGROUP_ANGULARRATE | COMMONGROUP_ACCEL,
                TIMEGROUP_NONE,
                IMUGROUP_NONE,
                GPSGROUP_NONE,
                ATTITUDEGROUP_NONE,
                INSGROUP_NONE,
                GPSGROUP_NONE))
                return;

            IMU data;
            data.timeStartup = p.extractUint64();
            data.yawPitchRoll = p.extractVec3f();
            data.quaternion = p.extractVec4f();
            data.angularRate = p.extractVec3f();
            data.acceleration = p.extractVec3f();
            data.timestamp = std::chrono::steady_clock::now();

            {
                std::lock_guard<std::mutex> lock(queueMutex_);

                const size_t MAX_QUEUE_SIZE = 1000;
                if (imuQueue_.size() >= MAX_QUEUE_SIZE) {
                    imuQueue_.pop();
                }

                imuQueue_.push(data);
            }
        }
    }

    void initialize_imu() {
        const string SensorPort = "/dev/ttyUSB0";
        const uint32_t SensorBaudrate = 921600;
        
        vs_ = std::make_unique<VnSensor>();
        
        try {
            vs_->connect(SensorPort, SensorBaudrate);
            vs_->setResponseTimeoutMs(2000);

            string mn = vs_->readModelNumber();
            RCLCPP_INFO(this->get_logger(), "Model Number: %s", mn.c_str());

            vs_->reset(true);
            vs_->changeBaudRate(921600);
            vs_->writeSettings(true);
            uint32_t newBaud = vs_->readSerialBaudRate();
            RCLCPP_INFO(this->get_logger(), "New Baud Rate: %u", newBaud);
            vs_->tare(true);

            // 바이너리 출력 설정 (800Hz)
            BinaryOutputRegister bor(
                ASYNCMODE_PORT1,
                1, // 800Hz를 위한 분주비 (800Hz = 800/1)
                COMMONGROUP_TIMESTARTUP | COMMONGROUP_YAWPITCHROLL | COMMONGROUP_QUATERNION | COMMONGROUP_ANGULARRATE | COMMONGROUP_ACCEL,
                TIMEGROUP_NONE,
                IMUGROUP_NONE,
                GPSGROUP_NONE,
                ATTITUDEGROUP_NONE,
                INSGROUP_NONE,
                GPSGROUP_NONE);
            vs_->writeBinaryOutput1(bor);

            // 콜백 등록
            vs_->registerAsyncPacketReceivedHandler(this, asciiOrBinaryAsyncMessageReceived);
            
            vs_initialized_ = true;
            RCLCPP_INFO(this->get_logger(), "VectorNav IMU initialized successfully.");
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize IMU: %s", e.what());
            vs_initialized_ = false;
        }
    }

    void initialize_variables() {
        framefix_ << 1.0, 0.0, 0.0,
                     0.0, -1.0, 0.0,
                     0.0, 0.0, -1.0;
        
        gravity_ << 0, 0, -1;
    }

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

    void imu_data_thread(int core) {
        set_thread_affinity(core, 99, "imu_data");
        RCLCPP_INFO(this->get_logger(), "IMU data processing thread started (Core %d)", core);

        auto next_cycle = std::chrono::steady_clock::now();
        IMU current_data;
        bool data_available = false;

        while (run_flag_.load()) {
            // IMU 큐에서 최신 데이터 가져오기
            if (!imuQueue_.empty()) {
                std::lock_guard<std::mutex> lock(queueMutex_);
                if (!imuQueue_.empty()) {
                    current_data = imuQueue_.back();
                    // 큐 비우기 (최신 데이터만 사용)
                    while (!imuQueue_.empty()) {
                        imuQueue_.pop();
                    }
                    data_available = true;
                }
            }

            if (data_available) {
                // IMU 데이터 처리
                tie(current_data.euler_angle, 
                    current_data.projection_angularRate, 
                    current_data.projection_acceleration, 
                    current_data.projection_gravity_array) = RX_IMU_data(current_data, framefix_, gravity_);

                // 필터링 적용
                current_data.euler_angle = lpfvec(pre_euler_angle_, current_data.euler_angle, alpha_);
                current_data.projection_angularRate = lpfvec(pre_projection_angularRate_, current_data.projection_angularRate, alpha_);
                current_data.projection_acceleration = lpfvec(pre_projection_acceleration_, current_data.projection_acceleration, alpha_);
                current_data.projection_gravity_array = lpfvec(pre_projection_gravity_array_, current_data.projection_gravity_array, alpha_);

                // 이전 값 업데이트
                pre_euler_angle_ = current_data.euler_angle;
                pre_projection_angularRate_ = current_data.projection_angularRate;
                pre_projection_acceleration_ = current_data.projection_acceleration;
                pre_projection_gravity_array_ = current_data.projection_gravity_array;

                // 처리된 데이터를 발행 스레드에서 사용할 수 있도록 저장
                {
                    std::lock_guard<std::mutex> lock(imu_data_mutex_);
                    latest_processed_data_ = current_data;
                    new_data_available_ = true;
                }

                imu_count_++;
            }

            next_cycle += IMU_PERIOD;
            std::this_thread::sleep_until(next_cycle);
        }
        RCLCPP_INFO(this->get_logger(), "IMU data processing thread finished.");
    }

    void publish_thread(int core) {
        set_thread_affinity(core, 95, "publish");
        RCLCPP_INFO(this->get_logger(), "Publish thread started (Core %d)", core);

        auto next_cycle = std::chrono::steady_clock::now();
        int loop_counter = 0;

        while (run_flag_.load()) {
            loop_counter++;

            // 새로운 데이터가 있으면 발행
            if (new_data_available_.load()) {
                IMU data_to_publish;
                {
                    std::lock_guard<std::mutex> lock(imu_data_mutex_);
                    data_to_publish = latest_processed_data_;
                    new_data_available_ = false;
                }

                // ROS2 메시지 생성 및 발행
                auto imu_msg = msg_interfaces::msg::Imudata();
                imu_msg.header.stamp = this->get_clock()->now();
                imu_msg.header.frame_id = "imu_link";

                imu_msg.orientation_w = data_to_publish.quaternion.w;
                imu_msg.orientation_x = data_to_publish.quaternion.x;
                imu_msg.orientation_y = data_to_publish.quaternion.y;
                imu_msg.orientation_z = data_to_publish.quaternion.z;
                
                imu_msg.angular_velocity_x = data_to_publish.projection_angularRate(0);
                imu_msg.angular_velocity_y = data_to_publish.projection_angularRate(1);
                imu_msg.angular_velocity_z = data_to_publish.projection_angularRate(2);
                
                imu_msg.linear_acceleration_x = data_to_publish.projection_acceleration(0);
                imu_msg.linear_acceleration_y = data_to_publish.projection_acceleration(1);
                imu_msg.linear_acceleration_z = data_to_publish.projection_acceleration(2);
                
                imu_msg.projection_gravity_x = data_to_publish.projection_gravity_array(0);
                imu_msg.projection_gravity_y = data_to_publish.projection_gravity_array(1);
                imu_msg.projection_gravity_z = data_to_publish.projection_gravity_array(2);
                
                imu_msg.roll = data_to_publish.euler_angle(0);
                imu_msg.pitch = data_to_publish.euler_angle(1);
                imu_msg.yaw = data_to_publish.euler_angle(2);

                imudata_pub_->publish(imu_msg);
                publish_count_++;
            }

            // // 상태 출력 (10Hz)
            // if (loop_counter % (PUBLISH_FREQ_HZ / 10) == 0) {
            //     RCLCPP_INFO(this->get_logger(), 
            //         "IMU Status - Data processed: %d, Published: %d, Queue size: %zu", 
            //         imu_count_.load(), publish_count_.load(), imuQueue_.size());
                
            //     if (new_data_available_.load()) {
            //         std::lock_guard<std::mutex> lock(imu_data_mutex_);
            //         RCLCPP_INFO(this->get_logger(), 
            //             "Current Data - Roll: %.2f, Pitch: %.2f, Yaw: %.2f", 
            //             latest_processed_data_.euler_angle(0) * 180.0 / PI,
            //             latest_processed_data_.euler_angle(1) * 180.0 / PI,
            //             latest_processed_data_.euler_angle(2) * 180.0 / PI);
            //     }
            // }

            next_cycle += PUBLISH_PERIOD;
            std::this_thread::sleep_until(next_cycle);
        }
        RCLCPP_INFO(this->get_logger(), "Publish thread finished.");
    }

    // 스레드 간 데이터 공유를 위한 변수
    IMU latest_processed_data_;
    std::atomic<bool> new_data_available_{false};
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImuControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}




// 권한 부여 필요
// sudo setcap cap_net_raw,cap_sys_nice+eip $(find ~/mujoco_ws/install -type f -name imu_test3)