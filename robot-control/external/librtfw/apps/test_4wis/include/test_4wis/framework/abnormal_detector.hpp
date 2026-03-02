#pragma once

#include <eigen3/Eigen/Dense>
#include <deque>
#include <thread>
#include "test_4wis/frame.hpp"
#include "test_4wis/framework/signal.hpp"
#include "test_4wis/framework/util.hpp"


using namespace std::chrono_literals;
using namespace project;

// TODO: Compare with PID

class AbnormalDetectorNode {
  public:
    double pitch_limit = 20;
    double roll_limit = 20;

    struct {
        struct {
            signal::Tx<bool> state;
            signal::Tx<bool> abnormal;
        } tx;
        struct {
            signal::Rx<bool>::SharedPtr state;
            signal::Rx<SE3>::SharedPtr pose;
            signal::Rx<se3>::SharedPtr vel;
        } rx;
    } signal;

    struct {
      bool abnormal;
    } result;

    AbnormalDetectorNode() {
      signal.rx.state = std::make_shared<signal::Rx<bool>>([&](const bool& data) {
        if (data && !active) {
          active = data;
          printf("abnormal detector: state=on\n");
          orient_ref = orient;
          result.abnormal = false;
        }
        if (!data && active) {
          active = data;
          printf("abnormal detector: state=off\n");

        }
      });

      signal.rx.pose = std::make_shared<signal::Rx<SE3>>([&](const SE3& data) {        
        Eigen::Quaterniond ori(data.R);
        Eigen::AngleAxisd aa(ori);
        orient = aa.axis() * aa.angle();
        if (!active)
            return;
        auto error = (orient - orient_ref);
        result.abnormal = abs(error.x()) > roll_limit*M_PI/180 || abs(error.y()) > pitch_limit*M_PI/180;
        if (result.abnormal) {
          printf("abnormal detector: abnormal state - pitch=%.0lf deg, roll=%.0lf deg\n", error.y()*180/M_PI, error.x()*180/M_PI);
        } else {
          PERIODIC_CALL(printf("abnormal detector: pitch=%.0lf deg, roll=%.0lf deg\n", error.y()*180/M_PI, error.x()*180/M_PI), 1s);
        }

      });

      signal.rx.vel = std::make_shared<signal::Rx<se3>>([&](const se3& data) {
        vel = data;
        if (!active)
            return;

        PERIODIC_CALL(printf("abnormal detector: velocity max=%.1lf, %.1lf\n", vel.linear.cwiseAbs().maxCoeff(), vel.angular.cwiseAbs().maxCoeff()), 1s);

        if (vel.linear.cwiseAbs().maxCoeff() > 2.0 || vel.angular.cwiseAbs().maxCoeff() > 1.0) {
          result.abnormal = true;
          printf("abnormal detector: abnormal state - velocity max=%.1lf, %.1lf\n", vel.linear.cwiseAbs().maxCoeff(), vel.angular.cwiseAbs().maxCoeff());
        }

      });

      std::thread([&](){
        while(run) {
            std::this_thread::sleep_for(100ms);
            signal.tx.state.send(active);
            if (active) {
              signal.tx.abnormal.send(result.abnormal);
            }
        }

      }).detach();
    }
    ~AbnormalDetectorNode() {
        run = false;
    }
  private:
    Eigen::Vector3d orient;
    se3 vel;
    Eigen::Vector3d orient_ref;

  private:
    bool active = false;
    bool run = true;
};