#pragma once

#include <cstdio>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include <thread>

#include <eigen3/Eigen/Dense>
#include "test_4wis/framework/signal.hpp"
#include "test_4wis/framework/util.hpp"

using namespace project;

// 50 rpm -> 0.17m/s

class WheelIF
{
public:
    struct {
      std::string interface = "/dev/dynamixel_bus";
      double wheel_radius = 0.033;
      double gear = 353.5;
      int16_t rpm_max = 50;
      double steer_max = 359;
      double steer_min = 1;
    } param;

    struct {
        struct {
            signal::Tx<bool> state;
            signal::Tx<std::array<double, 8>> wheel;  // rpm[4], deg[4]
        } tx;
        struct {
            signal::Rx<bool>::SharedPtr state;
            signal::Rx<std::array<double, 8>>::SharedPtr wheel; // rpm[4], deg[4]
        } rx;
    } signal;

    std::array<double, 4> rpm_sv = {0};
    std::array<double, 4> rpm_pv = {0};
    std::array<double, 4> steer_sv = {0};
    std::array<double, 4> steer_pv = {0};


  WheelIF();
  virtual ~WheelIF();

private:
  bool wheel_setup(bool on);

  dynamixel::PortHandler * portHandler;
  dynamixel::PacketHandler * packetHandler;
  dynamixel::GroupSyncWrite* gswPos;
  dynamixel::GroupSyncWrite* gswVel;
  dynamixel::GroupSyncRead* gsrPos;
  dynamixel::GroupSyncRead* gsrVel;

  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  bool active =  false;
  // rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_twist;
  // rclcpp::Service<GetPosition>::SharedPtr get_position_server_;

};


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
#define BAUDRATE 1000000  // Default Baudrate of DYNAMIXEL X series
#define DEVICE_NAME "/dev/dynamixel_bus"  // [Linux]: "/dev/ttyUSB*", [Windows]: "COM*"



WheelIF::WheelIF(): active(false)
{
  portHandler = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
  packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  gswPos = new dynamixel::GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, 4);
  gswVel = new dynamixel::GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_VELOCITY, 4);

  gsrPos = new dynamixel::GroupSyncRead(portHandler, packetHandler, ADDR_PRESENT_POSITION, 4);
  gsrVel = new dynamixel::GroupSyncRead(portHandler, packetHandler, ADDR_PRESENT_VELOCITY, 4);

  // Set the baudrate of the serial port (use DYNAMIXEL Baudrate)
  dxl_comm_result = portHandler->setBaudRate(BAUDRATE);
  if (dxl_comm_result == false) {
    printf("Failed to set the baudrate!\n");
  } else {
    printf("Succeeded to set the baudrate.\n");
  }

  // Open Serial Port
  dxl_comm_result = portHandler->openPort();
  if (dxl_comm_result == false) {
    printf("Failed to open the port!\n");
  } else {

  }


  signal.rx.state = std::make_shared<signal::Rx<bool>>([&](const bool& data) {
    if (data && !active) {
      if (wheel_setup(true)) {
        active = data;
        rpm_sv.fill(0);
        steer_sv.fill(180);
        printf("WheelIF: state=on\n");
      }
    } 
    if (!data && active) {
      if(wheel_setup(false)) {
        active = data;
        printf("WheelIF: state=off\n");
      }
    }
  });
  
  signal.rx.wheel = std::make_shared<signal::Rx<std::array<double, 8>>>([&](const std::array<double, 8>& data) {
    if (!active)
      return;
    // printf("WheelIF: req rpm=[%.0f, %.0f, %.0f, %.0f]\n", data[0], data[1], data[2], data[3]);

    double factor;

    gsrPos->clearParam();
    for (auto i=0; i<4; i++) {
      gsrPos->addParam(11+i);
    }
    dxl_comm_result = gsrPos->txRxPacket();
    if (dxl_comm_result != COMM_SUCCESS) {
      printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
      return;
    } else {
      for (auto i=0; i<4; i++) {
        if (gsrPos->isAvailable(11+i, ADDR_PRESENT_POSITION, 4)) {
          int32_t raw = (int32_t)gsrPos->getData(11+i, ADDR_PRESENT_POSITION, 4);
          steer_pv[i] = raw * 0.088;
        }
      }      
    }

    gsrVel->clearParam();
    for (auto i=0; i<4; i++) {
      gsrVel->addParam(11+i);
    }
    dxl_comm_result = gsrVel->txRxPacket();
    if (dxl_comm_result != COMM_SUCCESS) {
      printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
      return;
    } else {
      for (auto i=0; i<4; i++) {
        if (gsrVel->isAvailable(11+i, ADDR_PRESENT_VELOCITY, 4)) {
          int32_t raw = (int32_t)gsrVel->getData(11+i, ADDR_PRESENT_VELOCITY, 4);
          rpm_pv[i] = raw * 0.229;
        }
      }      
    }
    
    std::array<double, 8> wheel_pv;
    for (auto i=0; i<4; i++) {
      wheel_pv[i] = rpm_pv[i];
      wheel_pv[4+i] = -(steer_pv[i]-180);
    }
    signal.tx.wheel.send(wheel_pv);
    
    
    factor = 0.0;
    for (auto i=0; i<4; i++) {
      auto req = -data[4+i]+180;
      if (req > param.steer_max) req = param.steer_max;
      if (req < param.steer_min) req = param.steer_min;
      steer_sv[i] = req * (1-factor) + steer_sv[i] * factor;
      if (std::isnan(steer_sv[i]) || std::isinf(steer_sv[i])) {
        steer_sv[0] = 180;
        steer_sv[1] = 180;
        steer_sv[2] = 180;
        steer_sv[3] = 180;
        break;
      }
    }

    gswPos->clearParam();
    int32_t steer_i32[4];
    uint8_t steer[4][4] = {0};
    for (auto i=0; i<4; i++) {
      steer_i32[i] = steer_sv[i]/0.088;
      steer[i][0] = DXL_LOBYTE(DXL_LOWORD(steer_i32[i]));
      steer[i][1] = DXL_HIBYTE(DXL_LOWORD(steer_i32[i]));
      steer[i][2] = DXL_LOBYTE(DXL_HIWORD(steer_i32[i]));
      steer[i][3] = DXL_HIBYTE(DXL_HIWORD(steer_i32[i]));
      gswPos->addParam(11+i, steer[i]);
    }
    dxl_comm_result = gswPos->txPacket();

    if (dxl_comm_result != COMM_SUCCESS) {
      printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
      return;
    } else {
      
    }

    factor = 0.2;
    for (auto i=0; i<4; i++) {
      auto req = data[i];
      if (i%2 == 1)
        req = -req;
      if (req > param.rpm_max) req = param.rpm_max;
      if (req < -param.rpm_max) req = -param.rpm_max;
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

    gswVel->clearParam();
    int32_t rpm_i32[4];
    uint8_t rpm[4][4] = {0};
    for (auto i=0; i<4; i++) {
      rpm_i32[i] = rpm_sv[i]/0.229;
      rpm[i][0] = DXL_LOBYTE(DXL_LOWORD(rpm_i32[i]));
      rpm[i][1] = DXL_HIBYTE(DXL_LOWORD(rpm_i32[i]));
      rpm[i][2] = DXL_LOBYTE(DXL_HIWORD(rpm_i32[i]));
      rpm[i][3] = DXL_HIBYTE(DXL_HIWORD(rpm_i32[i]));
      gswVel->addParam(21+i, rpm[i]);
    }
    dxl_comm_result = gswVel->txPacket();

    if (dxl_comm_result != COMM_SUCCESS) {
      printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
      return;
    } else {

    }

  //   PERIODIC_CALL(
  //     printf("rpm = %.0lf, %.0lf, %.0lf, %.0lf\n", rpm_pv[0], rpm_pv[1], rpm_pv[2], rpm_pv[3]);
  //     printf("dir = %.0lf, %.0lf, %.0lf, %.0lf\n", steer_pv[0], steer_pv[1], steer_pv[2], steer_pv[3]);
  //  , 1s);
  });

    // std::thread([&](){
    //     printf("wheel if: thread started\n");
    //     while (run) {
    //         recv();
    //     }
    //     printf("wheel if: thread finished\n");
    // }).detach();
        
  // auto get_present_position =
  //   [this](
  //   const std::shared_ptr<GetPosition::Request> request,
  //   std::shared_ptr<GetPosition::Response> response) -> void
  //   {
  //     // Read Present Position (length : 4 bytes) and Convert uint32 -> int32
  //     // When reading 2 byte data from AX / MX(1.0), use read2ByteTxRx() instead.
  //     dxl_comm_result = packetHandler->read4ByteTxRx(
  //       portHandler,
  //       (uint8_t) request->id,
  //       ADDR_PRESENT_POSITION,
  //       reinterpret_cast<uint32_t *>(&present_position),
  //       &dxl_error
  //     );

  //     RCLCPP_INFO(
  //       this->get_logger(),
  //       "Get [ID: %d] [Present Position: %d]",
  //       request->id,
  //       present_position
  //     );

  //     response->position = present_position;
  //   };
}

WheelIF::~WheelIF()
{
  // Disable Torque of DYNAMIXEL
  wheel_setup(false);
  portHandler->closePort();
}

bool WheelIF::wheel_setup(bool on) {

  if (on) {
    packetHandler->write4ByteTxRx(
      portHandler,
      BROADCAST_ID,
      ADDR_PROFILE_VELOCITY,
      50 / 0.229,
      &dxl_error
    );

    packetHandler->write4ByteTxRx(
      portHandler,
      BROADCAST_ID,
      ADDR_PROFILE_ACCELERATION,
      10 * 3600 / 214.577,
      &dxl_error
    );


  }

  // Enable Torque of DYNAMIXEL
  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler,
    BROADCAST_ID,
    ADDR_TORQUE_ENABLE,
    on,
    &dxl_error
  );
  

    if (dxl_comm_result != COMM_SUCCESS) {
    printf("Failed to enable/disable torque.\n");
      printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    return false;
    } else if (dxl_error != 0) {
    printf("dxl error: ");
      printf("%s\n", packetHandler->getTxRxResult(dxl_error));
    return false;
    } else {
    printf("Succeeded to enable/disable torque.\n");
    return true;
      
    }

  // if (dxl_comm_result != COMM_SUCCESS) {
  //   printf("Failed to enable/disable torque.\n");
  //   return false;
  // } else {
  //   printf("Succeeded to enable/disable torque.\n");
  //   return true;
  // }
}

