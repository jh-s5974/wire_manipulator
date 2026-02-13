#include <cstdint>
#include <iostream>
#include <stdint.h>     
#include <string.h>     
#include <unistd.h>     
#include <stdio.h>      
#include <net/if.h>     
#include <sys/ioctl.h>  
#include <sys/socket.h> 
#include <linux/can.h>  
#include <linux/can/raw.h>
#include <fcntl.h>      
#include <termios.h>    
#include <time.h>       
#include <math.h>

#include <fstream>
#include <sstream>
#include <string>
#include <Eigen/Eigen>
#include <Eigen/Dense>

#include <chrono>
#include <thread>
#include <mutex>
#include <queue>
#include "vn/sensors.h"
#include "vn/thread.h"

#include <cmath>

#include <iomanip>
#include <cstdlib>

#define SEC_TO_NSEC 1000000000
#define NSEC_TO_SEC 0.000000001
#define PI 3.1415926

using namespace std;
using namespace Eigen;
using namespace vn::math;
using namespace vn::sensors;
using namespace vn::protocol::uart;
using namespace vn::xplat;

#define DT_MIN 1e-6
#define DT_MAX 1


struct IMU {
    uint64_t timeStartup;
    vec3f yawPitchRoll;
    vec4f quaternion;
    vec3f angularRate;
    vec3f acceleration;
};
std::queue<IMU> imuQueue;
std::mutex queueMutex;


double deg2rad(double degree){
    double radian = degree*PI/180;
    return radian;
}

void asciiOrBinaryAsyncMessageReceived(void* userData, Packet& p, size_t index) { //IMU 바이너리 메시지 옵션 설정
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

        {
            std::lock_guard<std::mutex> lock(queueMutex);

            const size_t MAX_QUEUE_SIZE = 1000; // 예시 크기
            if (imuQueue.size() >= MAX_QUEUE_SIZE) {
                imuQueue.pop(); // 가장 오래된 데이터 제거
            }

            imuQueue.push(data);
        }
    }
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

int main(int argc, char *argv[]) {
    const string SensorPort = "/dev/ttyUSB0";
    const uint32_t SensorBaudrate = 921600;
    VnSensor vs;
    vs.connect(SensorPort, SensorBaudrate);
	vs.setResponseTimeoutMs(2000); // 최대응답 기다리는 시간

	string mn = vs.readModelNumber();
	cout << "Model Number: " << mn << endl;

	vs.reset(true);
	vs.changeBaudRate(921600);
    vs.writeSettings(true);  //보드레이트를 영구적으로 세팅
	uint32_t newBaud = vs.readSerialBaudRate();
	cout << "New Baud Rate: " << newBaud << endl;
	vs.tare(true); // 요를 0으로 세팅

    Matrix3f Framefix;
    Framefix << 1.0, 0.0, 0.0,
                0.0, -1.0, 0.0,
                0.0, 0.0, -1.0;

    {
        BinaryOutputRegister bor(
            ASYNCMODE_PORT1,
            1,
            COMMONGROUP_TIMESTARTUP | COMMONGROUP_YAWPITCHROLL | COMMONGROUP_QUATERNION | COMMONGROUP_ANGULARRATE | COMMONGROUP_ACCEL,
            TIMEGROUP_NONE,
            IMUGROUP_NONE,
            GPSGROUP_NONE,
            ATTITUDEGROUP_NONE,
            INSGROUP_NONE,
            GPSGROUP_NONE);
        vs.writeBinaryOutput1(bor);
    }
	vs.registerAsyncPacketReceivedHandler(NULL, asciiOrBinaryAsyncMessageReceived);

    Vector3f gravity(0, 0, -1);

    Vector3f euler_angle;
    Vector3f Projection_angularRate;
    Vector3f Projection_Acceleration;
    Vector3f Projection_gravity_Array;
    Vector3f pre_euler_angle(0,0,0);
    Vector3f pre_Projection_angularRate(0,0,0);
    Vector3f pre_Projection_Acceleration(0,0,0);
    Vector3f pre_Projection_gravity_Array(0,0,0);
    IMU data;

    
    // 변수 설정 -------------------------------------------------------------------------------

    struct timespec start_imu, now_imu;
    double elapsed_imu = 0;
    const double Period_IMU = 0.00125; // 800hz

    float alpha = 0.978;
    float pre_trans_Acceleration = 0;

    clock_gettime(CLOCK_MONOTONIC, &start_imu);

    while (true){
        clock_gettime(CLOCK_MONOTONIC, &now_imu);
        elapsed_imu = (now_imu.tv_sec - start_imu.tv_sec) + (now_imu.tv_nsec - start_imu.tv_nsec) * NSEC_TO_SEC;

        if (!imuQueue.empty()) {
            std::lock_guard<std::mutex> lock(queueMutex); // 이 코드가 포함된 영역{ } 은 전부 lock_guard의 보호를 받음.
            data = imuQueue.back();
        }

        if (elapsed_imu >= Period_IMU){
            tie(euler_angle, Projection_angularRate, Projection_Acceleration, Projection_gravity_Array) = RX_IMU_data(data, Framefix, gravity);
            // euler_angle = lpfvec(pre_euler_angle, euler_angle, alpha);
            // Projection_angularRate = lpfvec(pre_Projection_angularRate, Projection_angularRate, alpha);
            // Projection_Acceleration = lpfvec(pre_Projection_Acceleration, Projection_Acceleration, alpha);
            // Projection_gravity_Array = lpfvec(pre_Projection_gravity_Array, Projection_gravity_Array, alpha);

            // pre_euler_angle = euler_angle;
            // pre_Projection_angularRate = Projection_angularRate;
            // pre_Projection_Acceleration = Projection_Acceleration;
            // pre_Projection_gravity_Array = Projection_gravity_Array;

            cout << "ori: " << euler_angle.transpose();
            cout << "     angular: " << Projection_angularRate.transpose();
            cout << "     acc: " << Projection_Acceleration.transpose();
            cout << "     gravity: " << Projection_gravity_Array.transpose();
            cout << endl;
            start_imu = now_imu;
        }
    }

    return 0;
}

