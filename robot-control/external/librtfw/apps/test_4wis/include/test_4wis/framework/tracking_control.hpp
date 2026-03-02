#pragma once

#include <eigen3/Eigen/Dense>
#include "test_4wis/frame.hpp"
#include "test_4wis/framework/signal.hpp"
#include "test_4wis/framework/util.hpp"

#include <deque>
#include <thread>

using namespace std::chrono_literals;
using namespace project;


// TODO: Compare with LQR

class TrackingNode {
  public:
    std::array<double, 3> kp;
    std::array<double, 3> kv;
    std::array<double, 3> dead_zone;

    double v_max = 0.63*0.2;
    double w_max = 0.75*0.2;

    struct {
        struct {
          signal::Tx<bool> state; 
          signal::Tx<se3> mobile_vel;
          signal::Tx<se3> error;
        } tx;
        struct {
            signal::Rx<bool>::SharedPtr state;
            signal::Rx<SE3>::SharedPtr pose;
            signal::Rx<se3>::SharedPtr mobile_vel;
            signal::Rx<se3>::SharedPtr robot_vel;
        } rx;
    } signal;

    se3 pose_error;
    Eigen::Matrix<double, 6, 6> Ad;
    Eigen::Matrix<double, 6, 3> Bd;
    Eigen::Matrix<double, 6, 6> Q;
    Eigen::Matrix<double, 3, 3> R;
    Eigen::Matrix<double, 3, 6> K;

    TrackingNode(double dt=0.02) {
      kp = {1.0, 1.0, 1.0};
      kv = {0.01, 0.01, 0.01};
      dead_zone = {0.005, 0.005, 0.05};         // (m, m, rad)

      
      robot_vel_pv.setZero();
      pose2d_pv.setZero();

      signal.rx.state = std::make_shared<signal::Rx<bool>>([&](const bool& data) {
        if (data) {
            if (!active) {
                active = true;
                pose_ref = pose_pv;
                pose2d_ref = pose2d_pv;
                vel_sv = vel_pv;
                printf("tracking control: state=on\n");
            }
        } else {
            if (active) {
                active = false;
                vel_sv.setZero();
                se3 msg_vel;
                msg_vel.linear.x() = 0;
                msg_vel.linear.y() = 0;
                msg_vel.angular.z() = 0;
                signal.tx.mobile_vel.send(msg_vel);                
                printf("tracking control: state=off\n");
            }
        }
      });

      signal.rx.mobile_vel = std::make_shared<signal::Rx<se3>>([&](const se3& data) {
        vel_pv[0] = data.linear.x();
        vel_pv[1] = data.linear.y();
        vel_pv[2] = data.angular.z();

        odom[0] += (cos(odom[2])*vel_pv[0]-sin(odom[2])*vel_pv[1])*0.01;
        odom[1] += (sin(odom[2])*vel_pv[0]+cos(odom[2])*vel_pv[1])*0.01;
        odom[2] += vel_pv[2]*0.01;

      });
      signal.rx.robot_vel = std::make_shared<signal::Rx<se3>>([&](const se3& data) {
        robot_vel_pv.x() = data.linear.x();
        robot_vel_pv.y() = data.linear.y();
        robot_vel_pv.z() = data.angular.z();
      });
      signal.rx.pose = std::make_shared<signal::Rx<SE3>>([&](const SE3& data) {        
        Eigen::Quaterniond ori(data.R);
        Eigen::AngleAxisd aa(ori);

        odom.setZero();

        pose_pv = data;
        pose2d_pv.x() = data.T.x();
        pose2d_pv.y() = data.T.y();
        pose2d_pv.z() = aa.axis().z() * aa.angle();

        if (!active)
            return;

        pose_error = diff(pose_pv, pose_ref);
        signal.tx.error.send(pose_error);
        // Eigen::Vector3d error;
        // error << pose_error.linear.x(), pose_error.linear.y(), pose_error.angular.z();

        // for (auto i=0; i<3; i++) {
        //     if (abs(error(i)) < dead_zone[i])
        //         error(i) = 0;
        // }

        // if (active)
        //   update_controller(error, robot_vel_pv);
        
      });
      std::thread([&](){
        auto now = std::chrono::steady_clock::now();
        auto last = now;
        while(run) {
          std::this_thread::sleep_for(1ms);
          now = std::chrono::steady_clock::now();
          if (now - last >= 10ms) {
            if (active) {
              Eigen::Vector3d error;
              error << pose_error.linear.x(), pose_error.linear.y(), pose_error.angular.z();
              error -= odom;
      
              for (auto i=0; i<3; i++) {
                  if (abs(error(i)) < dead_zone[i])
                      error(i) = 0;
              }

              update_controller(error, robot_vel_pv);
            }
            
          }

        // Eigen::Vector3d error;
        // error << pose_error.linear.x(), pose_error.linear.y(), pose_error.angular.z();

        // for (auto i=0; i<3; i++) {
        //     if (abs(error(i)) < dead_zone[i])
        //         error(i) = 0;
        // }

        // if (active)
        //   update_controller(error, robot_vel_pv);

            signal.tx.state.send(active);
        }
      }).detach();

      Ad.setZero();
      Bd.setZero();
      Q.setZero();
      R.setZero();
      Ad.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
      Ad.block<3, 3>(0, 3) = dt*Eigen::Matrix3d::Identity();
      Bd.block<3, 3>(3, 0) = Eigen::Matrix3d::Identity();
      Q.diagonal() << 3, 3, 3, 0, 0, 0;
      R.diagonal() << 1, 1, 1;
      Eigen::Matrix<double, 6, 6> P;
      P.setZero();
      if(solve_dare_static<6, 3>(Ad, Bd, Q, R, P)) {
        K = R.inverse()*Bd.transpose()*P;
      } else {
        printf("Warning! discrete ARE solver failed\n");
        K.setZero();
      }
    }
    ~TrackingNode() {
        run = false;
    }
  private:

    bool update_controller(Eigen::Vector3d pos, Eigen::Vector3d vel) {
        if (!active)
            return false;

        se3 msg_vel;
        
          // PD Control
        {
          // Eigen::Vector3d eig_kp = Eigen::Map<Eigen::Vector3d>(&kp[0]);
          // Eigen::Vector3d eig_kv = Eigen::Map<Eigen::Vector3d>(&kv[0]);

          // Eigen::Vector3d mv = pos.cwiseProduct(eig_kp) + vel.cwiseProduct(eig_kv);
          
          // if (mv[0] > v_max) mv[0] = v_max;
          // if (mv[0] < -v_max) mv[0] = -v_max;
          // if (mv[1] > v_max) mv[1] = v_max;
          // if (mv[1] < -v_max) mv[1] = -v_max;
          // if (mv[2] > w_max) mv[2] = w_max;
          // if (mv[2] < -w_max) mv[2] = -w_max;

          // msg_vel.linear.x() = mv[0];
          // msg_vel.linear.y() = mv[1];
          // msg_vel.angular.z() = mv[2];

        }

        // LQR Control
        {
          Eigen::Vector<double, 6> state;
          state.head(3) = -pos;
          state.tail(3) = vel_pv;
          vel_sv = -this->K*state;
          msg_vel.linear.x() = vel_sv[0];
          msg_vel.linear.y() = vel_sv[1];
          msg_vel.angular.z() = vel_sv[2];


        // PERIODIC_CALL(
        //   printf("state=%lf, %lf, %lf, %lf, %lf, %lf\n", state[0], state[1], state[2], state[3], state[4], state[5]);
        //   printf("vx=%lf, vy=%lf, wx=%lf\n", msg_vel.linear.x(), msg_vel.linear.y(), msg_vel.angular.z())
        // , 1s);
        }

        signal.tx.mobile_vel.send(msg_vel);

        return true;
    }

  private:
    bool active = false;
    bool run=true;

    SE3 pose_ref;
    SE3 pose_pv;
    Eigen::Vector3d pose2d_ref;
    Eigen::Vector3d pose2d_pv;
    Eigen::Vector3d robot_vel_pv;
    Eigen::Vector<double, 3> vel_pv;
    Eigen::Vector<double, 3> vel_sv;

    Eigen::Vector3d odom;

};