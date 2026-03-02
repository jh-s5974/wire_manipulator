#pragma once

#include <rtfw/task.h>
// #include "parameters.h"

// #include "dynamixel_sdk/dynamixel_sdk.h"

#include <errno.h>


using namespace rtfw;
using namespace rtfw::rt;


// Control table address for X series (except XL-320)
#define ADDR_OPERATING_MODE 11
#define ADDR_TORQUE_ENABLE 64
#define ADDR_GOAL_VELOCITY 104
#define ADDR_PROFILE_ACCELERATION 108
#define ADDR_PROFILE_VELOCITY 112
#define ADDR_GOAL_POSITION 116
#define ADDR_PRESENT_VELOCITY 128
#define ADDR_PRESENT_POSITION 132

// Protocol version
#define PROTOCOL_VERSION 2.0  // Default Protocol version of DYNAMIXEL X series.

// Default setting
#define BAUDRATE 4600000  // Default Baudrate of DYNAMIXEL X series
#define DEVICE_NAME "/dev/dynamixel_bus"  // [Linux]: "/dev/ttyUSB*", [Windows]: "COM*"



namespace task_pool {

    class WheelIF : public ITask {
    public:
        ~WheelIF() {
            motor_stop();
            // if (can_so > 0) {
            //     close(can_so);
            // }
            // can_so = -1;
        }
        const char* getName() const override { return "Motor_Task"; }
        void setup(TaskRegistry& r) override {
            r.add_dependency(dw_wm_state);
            r.add_dependency(dw_wheel);
            r.add_dependency(dr_wheel);
            r.add_dependency(dw_steer);
            r.add_dependency(dr_steer);

            r.add_dependency(param.port);
            r.add_dependency(param.wheel_radius);
            r.add_dependency(param.rpm_max);
            r.add_dependency(param.gear);
            r.add_dependency(param.steer_max);
            r.add_dependency(param.steer_min);            
        }
        void initialize() override {
            // portHandler = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
            // packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

            // gswPos = new dynamixel::GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, 4);
            // gswVel = new dynamixel::GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_VELOCITY, 4);

            // gsrPos = new dynamixel::GroupSyncRead(portHandler, packetHandler, ADDR_PRESENT_POSITION, 4);
            // gsrVel = new dynamixel::GroupSyncRead(portHandler, packetHandler, ADDR_PRESENT_VELOCITY, 4);

            // // Set the baudrate of the serial port (use DYNAMIXEL Baudrate)
            // dxl_comm_result = portHandler->setBaudRate(BAUDRATE);
            // if (dxl_comm_result == false) {
            //     printf("Failed to set the baudrate!\n");
            // } else {
            //     printf("Succeeded to set the baudrate.\n");
            // }

        }
        void execute() override {            
            dr_wheel.on_update([this](const std::array<double, 4>& data) {
                double factor = 0.2;
                for (auto i=0; i<4; i++) {
                    auto req = data[i];
                    if (i%2 == 1)
                        req = -req;
                    if (req > param.rpm_max.read()) req = param.rpm_max.read();
                    if (req < -param.rpm_max.read()) req = -param.rpm_max.read();
                    rpm_sv[i] = req * (1-factor) + rpm_sv[i] * factor;
                    if (std::isnan(rpm_sv[i]) || std::isinf(rpm_sv[i])) {
                        rpm_sv[0] = 0;
                        rpm_sv[1] = 0;
                        rpm_sv[2] = 0;
                        rpm_sv[3] = 0;
                        break;
                    }
                }

                for (auto i=0; i<4; i++) {
                    if (abs(steer_sv[i] - steer_pv[i]) > 90) {
                        // rpm_sv.fill(0);
                        break;
                    }
                }

            });
              
            dr_steer.on_update([this](const std::array<double, 4>& data) {
                double factor = 0.0;
                for (auto i=0; i<4; i++) {
                    auto req = -data[4+i]+180;
                    if (req > param.steer_max.read()) req = param.steer_max.read();
                    if (req < param.steer_min.read()) req = param.steer_min.read();
                    steer_sv[i] = req * (1-factor) + steer_sv[i] * factor;
                    if (std::isnan(steer_sv[i]) || std::isinf(steer_sv[i])) {
                        steer_sv[0] = 180;
                        steer_sv[1] = 180;
                        steer_sv[2] = 180;
                        steer_sv[3] = 180;
                        break;
                    }
                }

            });




            // prev code
            if (comm_init) {
                while(recv() > 0);
                trns();
            } else {
                auto res = port_open();
                if (!res) {
                    PERIODIC_CALL(
                        getLogger()->info("[{}] setup {} failed({})", getName(), param.port.read().c_str(), res);
                    , 1s);
                } else {
                    PERIODIC_CALL(
                        getLogger()->info("[{}] setup {} succeed", getName(), param.port.read().c_str());
                    , 1s);
                    wheel_setup(true);
                    motor_stop();
                }
            }

            dw_wheel.write(rpm_pv);
            dw_steer.write(steer_pv);
            dw_wm_state.write(comm_init);
        }
    private:
        DataWriter<bool> dw_wm_state{"wm_state", ArchiveOption::Enable};
        DataWriter<std::array<double, 4>> dw_wheel{"wheel_pv", ArchiveOption::Enable};
        DataWriter<std::array<double, 4>> dw_steer{"steer_pv", ArchiveOption::Enable};

        DataReader<std::array<double, 4>> dr_wheel{"wheel_sv", DependencyType::Weak};
        DataReader<std::array<double, 4>> dr_steer{"steer_sv", DependencyType::Weak};


        struct {
            Parameter<std::string> port{"param.wheel.port", "/dev/dynamixel_bus"};
            Parameter<double> wheel_radius{"param.wheel.wheel_radius", 0.033};
            Parameter<int16_t> rpm_max{"param.wheel.rpm_max", 50};
            Parameter<double> gear{"param.wheel.gear", 353.5};
            Parameter<double> steer_max{"param.wheel.steer_max", 359};
            Parameter<double> steer_min{"param.wheel.steer_min", 1};
        } param;
        
        
    private:
        // dynamixel::PortHandler * portHandler;
        // dynamixel::PacketHandler * packetHandler;
        // dynamixel::GroupSyncWrite* gswPos;
        // dynamixel::GroupSyncWrite* gswVel;
        // dynamixel::GroupSyncRead* gsrPos;
        // dynamixel::GroupSyncRead* gsrVel;

        uint8_t dxl_error = 0;
        // int dxl_comm_result = COMM_TX_FAIL;
        
        
        std::array<double, 4> rpm_sv = {0};
        std::array<double, 4> rpm_pv = {0};
        std::array<double, 4> steer_sv = {0};
        std::array<double, 4> steer_pv = {0};

    private:

        bool port_open() {
            // Open Serial Port
            // dxl_comm_result = portHandler->openPort();
            // if (dxl_comm_result == false) {
            //     printf("Failed to open the port!\n");
            // } else {

            // }
            comm_init = true;
            return comm_init;
        }
        bool wheel_setup(bool on) {
            // if (on) {
            //     packetHandler->write4ByteTxRx(
            //     portHandler,
            //     BROADCAST_ID,
            //     ADDR_PROFILE_VELOCITY,
            //     50 / 0.229,
            //     &dxl_error
            //     );

            //     packetHandler->write4ByteTxRx(
            //     portHandler,
            //     BROADCAST_ID,
            //     ADDR_PROFILE_ACCELERATION,
            //     10 * 3600 / 214.577,
            //     &dxl_error
            //     );
            // }

            // // Enable Torque of DYNAMIXEL
            // dxl_comm_result = packetHandler->write1ByteTxRx(
            //     portHandler,
            //     BROADCAST_ID,
            //     ADDR_TORQUE_ENABLE,
            //     on,
            //     &dxl_error
            // );
        

            // if (dxl_comm_result != COMM_SUCCESS) {
            //     printf("Failed to enable/disable torque.\n");
            //     printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
            //     return false;
            // } else if (dxl_error != 0) {
            //     printf("dxl error: ");
            //     printf("%s\n", packetHandler->getTxRxResult(dxl_error));
            //     return false;
            // } else {
            //     printf("Succeeded to enable/disable torque.\n");
            //     return true;                
            // }
            return true;
        }

        void trns() {
            if (!comm_init) return;

            // gswVel->clearParam();
            // int32_t rpm_i32[4];
            // uint8_t rpm[4][4] = {0};
            // for (auto i=0; i<4; i++) {
            //     rpm_i32[i] = rpm_sv[i]/0.229;
            //     rpm[i][0] = DXL_LOBYTE(DXL_LOWORD(rpm_i32[i]));
            //     rpm[i][1] = DXL_HIBYTE(DXL_LOWORD(rpm_i32[i]));
            //     rpm[i][2] = DXL_LOBYTE(DXL_HIWORD(rpm_i32[i]));
            //     rpm[i][3] = DXL_HIBYTE(DXL_HIWORD(rpm_i32[i]));
            //     gswVel->addParam(21+i, rpm[i]);
            // }
            // dxl_comm_result = gswVel->txPacket();

            // if (dxl_comm_result != COMM_SUCCESS) {
            //     printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
            //     return;
            // } else {

            // }
            
            
            // gswPos->clearParam();
            // int32_t steer_i32[4];
            // uint8_t steer[4][4] = {0};
            // for (auto i=0; i<4; i++) {
            //     steer_i32[i] = steer_sv[i]/0.088;
            //     steer[i][0] = DXL_LOBYTE(DXL_LOWORD(steer_i32[i]));
            //     steer[i][1] = DXL_HIBYTE(DXL_LOWORD(steer_i32[i]));
            //     steer[i][2] = DXL_LOBYTE(DXL_HIWORD(steer_i32[i]));
            //     steer[i][3] = DXL_HIBYTE(DXL_HIWORD(steer_i32[i]));
            //     gswPos->addParam(11+i, steer[i]);
            // }
            // dxl_comm_result = gswPos->txPacket();

            // if (dxl_comm_result != COMM_SUCCESS) {
            //     printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
            //     return;
            // } else {
            
            // }


            // // steer pos read
            // gsrPos->clearParam();
            // for (auto i=0; i<4; i++) {
            //     gsrPos->addParam(11+i);
            // }
            // dxl_comm_result = gsrPos->txPacket();
            // if (dxl_comm_result != COMM_SUCCESS) {
            //     printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
            //     return;
            // } else {
            //     dxl_comm_result = gsrPos->rxPacket();
            //     if (dxl_comm_result != COMM_SUCCESS) {
            //         printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
            //         return;
            //     } else {
            //         for (auto i=0; i<4; i++) {
            //             if (gsrPos->isAvailable(11+i, ADDR_PRESENT_POSITION, 4)) {
            //                 int32_t raw = (int32_t)gsrPos->getData(11+i, ADDR_PRESENT_POSITION, 4);
            //                 steer_pv[i] = raw * 0.088;
            //             }
            //         }      
            //     }
            // }

            // // wheel vel read
            // gsrVel->clearParam();
            // for (auto i=0; i<4; i++) {
            //     gsrVel->addParam(11+i);
            // }
            // dxl_comm_result = gsrVel->txPacket();
            // if (dxl_comm_result != COMM_SUCCESS) {
            //     printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
            //     return;
            // } else {
            //     dxl_comm_result = gsrVel->rxPacket();
            //     if (dxl_comm_result != COMM_SUCCESS) {
            //         printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
            //         return;
            //     } else {
            //         for (auto i=0; i<4; i++) {
            //             if (gsrVel->isAvailable(11+i, ADDR_PRESENT_VELOCITY, 4)) {
            //                 int32_t raw = (int32_t)gsrVel->getData(11+i, ADDR_PRESENT_VELOCITY, 4);
            //                 rpm_pv[i] = raw * 0.229;
            //             }
            //         }      
            //     }
            // }            
        }

        int recv() {
            if (!comm_init)
                return -1;
            return 0;
        }

        void motor_stop() {
            rpm_sv.fill(0);
            steer_sv.fill(0);
            trns();
        }
        

    private:
        bool comm_init = false;
        bool enc_init = false;
    };
};