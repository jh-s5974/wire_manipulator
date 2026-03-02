#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <thread>

#include <eigen3/Eigen/Dense>
#include "test_4wis/4wis.hpp"
#include "test_4wis/frame.hpp"
#include "test_4wis/framework/signal.hpp"
#include "test_4wis/framework/util.hpp"

using namespace std::chrono_literals;
using namespace project;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class KinFourwis {
  public:
    struct {
        struct {
            signal::Tx<bool> state;
            signal::Tx<se3> velocity;
            signal::Tx<std::array<double, 8>> wheel;

        } tx;
        struct {
            signal::Rx<bool>::SharedPtr state;
            signal::Rx<se3>::SharedPtr velocity;
            signal::Rx<std::array<double, 8>>::SharedPtr wheel;
        } rx;
    } signal;

    double width = 0.58/2;      // m
    double length = 0.48/2;  // m
    double radius = 0.033;       // m
    double rpm_max = 50;        // rpm
    se3 vel_max;

    se3 vel_sv;
    se3 vel_pv;

    Eigen::Vector4d rpm_sv;
    Eigen::Vector4d dir_sv;
    Eigen::Vector4d rpm_pv;
    Eigen::Vector4d dir_pv;

    KinFourwis(double max_rpm=50): rpm_max(max_rpm) {
      {
        Eigen::Matrix<double, 3, 8> fk;
        fourwis::fk(fk, width, length, radius);

        Eigen::Vector<double, 8> rpm;
        rpm << rpm_max, 0, rpm_max, 0, rpm_max, 0 ,rpm_max, 0;
        rpm *= 2*M_PI/60;    // rpm -> rad/s
        Eigen::Vector3d velx = fk * rpm;

        rpm << 0, rpm_max, 0, rpm_max, 0, rpm_max, 0, rpm_max;
        rpm *= 2*M_PI/60;    // rpm -> rad/s
        Eigen::Vector3d vely = fk * rpm;

        auto s_ = sqrt(width/(width*width + length*length));
        auto c_ = sqrt(length/(width*width + length*length));
        rpm << -c_*rpm_max, s_*rpm_max, c_*rpm_max, s_*rpm_max, -c_*rpm_max, -s_*rpm_max, c_*rpm_max, -s_*rpm_max;
        rpm *= 2*M_PI/60;    // rpm -> rad/s
        Eigen::Vector3d rot = fk * rpm;

        vel_max.linear.x() = velx[0];
        vel_max.linear.y() = vely[1];
        vel_max.angular.z() = rot[2];

        rpm_sv.setZero();
        dir_sv.setZero();
        printf("fourwis: max rpm(%0.lf) -> max velocity=(%.2lf, %.2lf, %.2lf)\n", rpm_max, velx[0], vely[1], rot[2]);
      }

      signal.rx.state = std::make_shared<signal::Rx<bool>>([&](const bool& data) {
        if (data) {
            if (!active) {
                active = true;
                std::cout << "fourwis: state=on" << std::endl;
            }
        } else {
            if (active) {
                active = false;
                rpm_sv.setZero();
                dir_sv.setZero();
                std::cout << "fourwis: state=off" << std::endl;
            }
        }
      });

      signal.rx.velocity = std::make_shared<signal::Rx<se3>>([&](const se3& data) {
        vel_sv = data;
        Eigen::Vector3d vel;
        vel << vel_sv.linear.x(), vel_sv.linear.y(), vel_sv.angular.z();
        Eigen::Matrix<double, 8, 3> ik;
        fourwis::ik(ik, width, length, radius);

        Eigen::Vector<double, 8> wheel_vel = ik * vel;
        wheel_vel *= 60/2/M_PI;    // rad/s -> rpm

        Eigen::Vector4d rpm;
        Eigen::Vector4d dir;
        if (vel.norm() > 0.01) {

          rpm <<
          sqrt(wheel_vel[0]*wheel_vel[0]+wheel_vel[1]*wheel_vel[1]),
          sqrt(wheel_vel[2]*wheel_vel[2]+wheel_vel[3]*wheel_vel[3]),
          sqrt(wheel_vel[4]*wheel_vel[4]+wheel_vel[5]*wheel_vel[5]),
          sqrt(wheel_vel[6]*wheel_vel[6]+wheel_vel[7]*wheel_vel[7]);
          dir << 
            atan2(wheel_vel[1], wheel_vel[0]),
            atan2(wheel_vel[3], wheel_vel[2]),
            atan2(wheel_vel[5], wheel_vel[4]),
            atan2(wheel_vel[7], wheel_vel[6]);
          dir = dir * 180 / M_PI;
          // double max_coeff = rpm.cwiseAbs().maxCoeff();
          // if (max_coeff > rpm_max) {
          //   // rpm scaling
          //   rpm = rpm * rpm_max/max_coeff;
          // }

          for (auto i=0; i<4; i++) {
            if (dir[i] - dir_pv[i] > 90) {
              dir[i] -= 180;
              rpm[i] = -rpm[i];
            }
            if (dir[i] - dir_pv[i] < -90) {
              dir[i] += 180;
              rpm[i] = -rpm[i];
            }


            if (dir[i] > 135) {
              dir[i] -= 180;
              rpm[i] = -rpm[i];
            }

            if (dir[i] <= -135) {
              dir[i] += 180;
              rpm[i] = -rpm[i];
            }


            if (dir[i] < -180) dir[i] += 360;
            if (dir[i] > 180) dir[i] -= 360;

          }
        } else {
          dir.fill(0);
          rpm.fill(0);
        }

        for (auto i=0; i<4; i++) {
          dir_sv[i] = dir[i];
          rpm_sv[i] = rpm[i];
        }
        std::array<double, 8> msg_wheel;
        for (auto i=0; i<4; i++)
          msg_wheel[i] = rpm_sv[i];

        for (auto i=0; i<4; i++)
          msg_wheel[4+i] = dir_sv[i];
        signal.tx.wheel.send(msg_wheel);
      //   PERIODIC_CALL(
      //     printf("rpm_sv = %.0lf, %.0lf, %.0lf, %.0lf\n", rpm_sv[0], rpm_sv[1], rpm_sv[2], rpm_sv[3]);
      //     printf("dir_sv = %.0lf, %.0lf, %.0lf, %.0lf\n", dir_sv[0], dir_sv[1], dir_sv[2], dir_sv[3]);
      //  , 1s);
      });

      signal.rx.wheel = std::make_shared<signal::Rx<std::array<double, 8>>>([&](const std::array<double, 8>& data) {

        for(auto i=0; i<4; i++) {
          rpm_pv[i] = data[i];
          dir_pv[i] = data[4+i];
        }

        Eigen::Vector4d rpm;
        Eigen::Vector4d dir;
        rpm << data[0], data[1], data[2], data[3];
        rpm *= 2*M_PI/60;    // rpm -> rad/s
        dir << data[4], data[5], data[6], data[7];
        dir = dir * M_PI / 180;

        Eigen::Vector<double, 8> wheel_vel;
        wheel_vel << rpm[0] * std::cos(dir[0]), rpm[0] * std::sin(dir[0]),
                      rpm[1] * std::cos(dir[1]), rpm[1] * std::sin(dir[1]),
                      rpm[2] * std::cos(dir[2]), rpm[2] * std::sin(dir[2]),
                      rpm[3] * std::cos(dir[3]), rpm[3] * std::sin(dir[3]);

        Eigen::Matrix<double, 3, 8> fk;
        fourwis::fk_min(fk, width, length, radius);

        Eigen::Vector3d vel = fk * wheel_vel;
        
        vel_pv.linear.x() = vel[0];
        vel_pv.linear.y() = vel[1];
        vel_pv.angular.z() = vel[2];
        signal.tx.velocity.send(vel_pv);
      });
      std::thread([&](){
        while (run) {
            std::this_thread::sleep_for(500ms);
            signal.tx.state.send(active);
        }
      }).detach();
    }

  private:
    bool active = false;
    bool run = true;

    double L[2];
    double R;
};