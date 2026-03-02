#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <thread>

#include <eigen3/Eigen/Dense>
#include "test_4wis/framework/signal.hpp"
#include "test_4wis/framework/util.hpp"
#include "test_4wis/frame.hpp"

using namespace std::chrono_literals;
using namespace project;


//TODO: LB (reverse direction)
//TODO: CRANE POS CONTROL fix (origin reset -> crane_if)

/******************************************
 * State Machine
 * 
 * 1. State
 *   1) Manual: 
 *      - Joystick available      
 *   2) Tracking
 *      - Crane lock
 *      - Joystick lock
 *   3) Crane  
 *      - Wheel slow
 *   4) Recover
 *      - Wheel lock
 *      - Crane lock
 * 
 *  2. State-Transition:
 * 
 *        Crane <---> Manual <---> Tracking
 *          ^                          |
 *          ㄴ------ Recover <----------
 * 
 *  3. Safety
 *   1) slow move during Crane up
 *   2) while moving, Crane locked
 * 
*********************************************/

template<typename T>
class StateMachine {
  T cur;
  T prev;
  public:
    struct {
      std::function<void(T&)> before;
      std::function<void(T&)> current;
      std::function<void(T&)> after;
    } branch;

    void update() {
      prev = cur;
    }

    void operator =(T new_state) {
      cur = new_state;
    }

    T previous() {
      return prev;
    }

    T curent() {
      return cur;
    }

    bool is_updated() {
      return cur != prev;
    }
};

class CraneSM
{
  enum STATE {
    Idle,
    Manual,
    Crane,
    Tracking,
    Recover,
    Emergency,
  } state = STATE::Idle;


  struct {
    int ad; // abnormal detector
    int tc; // tracking control
    int cc; // crane control
    int cm; // crane motor
    int wm; // wheel motor
    int cv; // computer vision
    int mk; // mecanum kinematics
    int dq; // daq
    int js; // joystick
  } wdt, states;

  public:
    double slow_factor = 0.2;
    double v_max = 0.63*0.1;
    double w_max = 0.75*0.1;
    double vc_max = 0.05;
    double elevation = 0.01;

    struct {
        struct {
            signal::Tx<bool> cmd_ad_state;
            signal::Tx<bool> cmd_tc_state;
            signal::Tx<bool> cmd_cc_state;
            signal::Tx<bool> cmd_cm_state;
            signal::Tx<bool> cmd_wm_state;
            signal::Tx<bool> cmd_cv_state;
            signal::Tx<bool> cmd_mk_state;
            signal::Tx<bool> cmd_dq_state;
            signal::Tx<bool> cmd_js_state;
            signal::Tx<void> origin_reset;
            signal::Tx<double> height_sv;
            signal::Tx<se3> vel_sv;
            signal::Tx<double> crane_spd_sv;
        } tx;
        struct {
            signal::Rx<bool>::SharedPtr ad_state;
            signal::Rx<bool>::SharedPtr tc_state;
            signal::Rx<bool>::SharedPtr cc_state;
            signal::Rx<bool>::SharedPtr cm_state;
            signal::Rx<bool>::SharedPtr wm_state;
            signal::Rx<bool>::SharedPtr cv_state;
            signal::Rx<bool>::SharedPtr mk_state;
            signal::Rx<bool>::SharedPtr dq_state;
            signal::Rx<bool>::SharedPtr js_state;
            signal::Rx<double>::SharedPtr cmd_vx;
            signal::Rx<double>::SharedPtr cmd_vy;
            signal::Rx<double>::SharedPtr cmd_wz;
            signal::Rx<double>::SharedPtr cmd_vc_up;
            signal::Rx<double>::SharedPtr cmd_vc_down;
            signal::Rx<bool>::SharedPtr cmd_tracking_on;
            signal::Rx<bool>::SharedPtr cmd_tracking_off;
            signal::Rx<bool>::SharedPtr cmd_origin_reset;
            signal::Rx<bool>::SharedPtr cmd_soft_emg;
            signal::Rx<bool>::SharedPtr cmd_crane_teach;
            signal::Rx<bool>::SharedPtr cmd_vxy_reverse;
            signal::Rx<bool>::SharedPtr marker_detection;
            signal::Rx<double>::SharedPtr height_pv;
            signal::Rx<bool>::SharedPtr abnormal;
        } rx;
    } signal;

    CraneSM()
    {

    //   this->declare_parameter<double>("velocity_min", 0.01); // m/s
    //   this->declare_parameter<double>("rotation_min", 0.1); // rad/s
    //   this->declare_parameter<double>("crane_speed_min", 0.1); // m/s


      // last = std::chrono::steady_clock::now();


      signal.rx.ad_state = std::make_shared<signal::Rx<bool>>([&](const bool& data){
        wdt.ad = 0;
        if (states.ad != data) printf("state machine: ad state changed(%s)\n", data? "true": "false");
        states.ad = data;
      });
      signal.rx.tc_state = std::make_shared<signal::Rx<bool>>([&](const bool& data){
        wdt.tc = 0;
        if (states.tc != data) printf("state machine: tc state changed(%s)\n", data? "true": "false");
        states.tc = data;
      });
      signal.rx.cc_state = std::make_shared<signal::Rx<bool>>([&](const bool& data){
        wdt.cc = 0;
        if (states.cc != data) printf("state machine: cc state changed(%s)\n", data? "true": "false");
        states.cc = data;
      });
      signal.rx.cm_state = std::make_shared<signal::Rx<bool>>([&](const bool& data){
        wdt.cm = 0;
        if (states.cm != data) printf("state machine: cm state changed(%s)\n", data? "true": "false");
        states.cm = data;
      });
      signal.rx.wm_state = std::make_shared<signal::Rx<bool>>([&](const bool& data){
        wdt.wm = 0;
        if (states.wm != data) printf("state machine: wm state changed(%s)\n", data? "true": "false");
        states.wm = data;
      });
      signal.rx.cv_state = std::make_shared<signal::Rx<bool>>([&](const bool& data){
        wdt.cv = 0;
        if (states.cv != data) printf("state machine: cv state changed(%s)\n", data? "true": "false");
        states.cv = data;
      });
      signal.rx.mk_state = std::make_shared<signal::Rx<bool>>([&](const bool& data){
        wdt.mk = 0;
        if (states.mk != data) printf("state machine: mk state changed(%s)\n", data? "true": "false");
        states.mk = data;
      });
      signal.rx.dq_state = std::make_shared<signal::Rx<bool>>([&](const bool& data){
        wdt.dq = 0;
        if (states.dq != data) printf("state machine: dq state changed(%s)\n", data? "true": "false");
        states.dq = data;
      });
      signal.rx.js_state = std::make_shared<signal::Rx<bool>>([&](const bool& data){
        wdt.js = 0;
        if (states.js != data) printf("state machine: js state changed(%s)\n", data? "true": "false");
        states.js = data;
      });

      wdt = {0};
      states = {0};

      
      signal.rx.cmd_vx = std::make_shared<signal::Rx<double>>([&](const double& data) {
        req.vx = -data;
        if (abs(req.vx) < 0.02) req.vx = 0;
        if (is_vxy_reverse)
          req.vx = -req.vx;
      });
      signal.rx.cmd_vy = std::make_shared<signal::Rx<double>>([&](const double& data) {
        req.vy = -data;
        if (abs(req.vy) < 0.02) req.vy = 0;
        if (is_vxy_reverse)
          req.vy = -req.vy;
      });
      signal.rx.cmd_wz = std::make_shared<signal::Rx<double>>([&](const double& data) {
        req.wz = -data;
        if (abs(req.wz) < 0.02) req.wz = 0;
      });
      signal.rx.cmd_vc_up = std::make_shared<signal::Rx<double>>([&](const double& data) {
        req.vc = (1 + data) * 0.5;
        if (abs(req.vc) < 0.02) req.vc = 0;
      });
      signal.rx.cmd_vc_down = std::make_shared<signal::Rx<double>>([&](const double& data) {
        req.vc = -(1 + data) * 0.5;
        if (abs(req.vc) < 0.02) req.vc = 0;
      });

      signal.rx.cmd_tracking_on = std::make_shared<signal::Rx<bool>>([&](const bool& data) {
        if (!data)
          return;

        if (state != STATE::Manual)
          return;

        if (!states.cv) {
          std::cout << "can't enter tracking mode(cv offline)" << std::endl;
          return;
        }
        if (marker_lost > 0) {
          std::cout << "can't enter tracking mode(marker lost)" << std::endl;
          return;
        }
        // if (crane_pos >= 0) {
        if (crane_pos > 0) {
          std::cout << "can't enter tracking mode(crane not aligned)" << std::endl;
          return;
        }
        std::cout << "system enter tracking mode" << std::endl;
        state = STATE::Tracking;
        sig_ad = false;
        signal.tx.cmd_tc_state.send(true);
        signal.tx.cmd_ad_state.send(true);
      });

      signal.rx.cmd_tracking_off = std::make_shared<signal::Rx<bool>>([&](const bool& data) {
        if (!data)
          return;

        signal.tx.cmd_tc_state.send(false);
        signal.tx.cmd_ad_state.send(false);
        if (state != STATE::Tracking) {
          return;
        }

        state = STATE::Manual;
        std::cout << "system enter manual mode" << std::endl;
      });
      signal.rx.cmd_origin_reset = std::make_shared<signal::Rx<bool>>([&](const bool& data) {
        if (states.cc == true) return;
        if (state == STATE::Recover) return;
        if (!data) return;

        signal.tx.origin_reset.send();
      });

      signal.rx.cmd_soft_emg = std::make_shared<signal::Rx<bool>>([&](const bool& data) {
        if (data) {
          std::cout << "***emergency***" << std::endl;
          state = STATE::Emergency;
          signal.tx.cmd_ad_state.send(false);
          signal.tx.cmd_tc_state.send(false);
          signal.tx.cmd_cc_state.send(false);
          // signal.tx.cmd_cm_state.send(false);
          signal.tx.cmd_wm_state.send(false);
          signal.tx.cmd_cv_state.send(false);
          signal.tx.cmd_mk_state.send(false);
          signal.tx.cmd_dq_state.send(false);

          cmd.vx = req.vx = 0;
          cmd.vy = req.vy = 0;
          cmd.wz = req.wz = 0;
          cmd.vc = req.vc = 0;
          // pub_js_on->publish(std_msgs::msg::Bool().set__data(false));
        } else {
          std::cout << "***system start***" << std::endl;
          state = STATE::Manual;

          // pub_ad_on->publish(std_msgs::msg::Bool().set__data(true));
          // pub_tc_on->publish(std_msgs::msg::Bool().set__data(true));
          // pub_cc_on->publish(std_msgs::msg::Bool().set__data(true));
          // signal.tx.cmd_cm_state.send(true);
          signal.tx.cmd_wm_state.send(true);
          signal.tx.cmd_cv_state.send(true);
          signal.tx.cmd_mk_state.send(true);
          signal.tx.cmd_dq_state.send(true);
          // pub_js_on->publish(std_msgs::msg::Bool().set__data(true));
        }
      });
      signal.rx.cmd_crane_teach = std::make_shared<signal::Rx<bool>>([&](const bool& data) {
        if (data) {
          elevation = crane_pos;
          printf("set elevation height=%.3lf\n", elevation);
        } else {

        } 
      });
      signal.rx.cmd_vxy_reverse = std::make_shared<signal::Rx<bool>>([&](const bool& data) {
        if (data) {
          is_vxy_reverse = true;
          std::cout << "vxy reverse on" << std::endl;
        } else {
          is_vxy_reverse = false;
          std::cout << "vxy reverse off" << std::endl;
        } 
      });
      signal.rx.marker_detection = std::make_shared<signal::Rx<bool>>([&](const bool& data) {
        wdt.cv = 0;
        if (data) {
          if (marker_lost > 0)
            printf("marker tracking restore\n");
          marker_lost = 0;
        } else {
          if (marker_lost == 0)
            printf("marker tracking lost\n");

          if (marker_lost < marker_lost_max) {
            marker_lost++;
            if (state == STATE::Tracking)
              printf("marker lost=%d\n", marker_lost);
          }
        }
      });
      signal.rx.height_pv = std::make_shared<signal::Rx<double>>([&](const double& data) {
        crane_pos = data;
        wdt.dq = 0;
      });
      signal.rx.abnormal = std::make_shared<signal::Rx<bool>>([&](const bool& data){
        if (data) {
          if (state == STATE::Tracking) {
            std::cout << "abnormal detected" << std::endl;
            sig_ad = true;
          }
        } else {

        }
      });

      
      signal.tx.cmd_ad_state.send(false);
      signal.tx.cmd_tc_state.send(false);
      signal.tx.cmd_cc_state.send(false);
      signal.tx.cmd_cm_state.send(true);
      signal.tx.cmd_wm_state.send(true);
      signal.tx.cmd_cv_state.send(true);
      signal.tx.cmd_mk_state.send(true);
      signal.tx.cmd_dq_state.send(true);
      signal.tx.cmd_js_state.send(true);

      std::thread([&](){
        auto stamp = std::chrono::steady_clock::now();
        bool sync = false;
        while (run) {
          stamp += 100ms;
          auto now = std::chrono::steady_clock::now();
          if (now > stamp) {
            if (sync)
              printf("state machine: sync failed\n");
            sync = false;
          } else {
            if (!sync)
              printf("state machine: sync succeed\n");
            sync = true;
            std::this_thread::sleep_until(stamp);
          }

          watchdog_task();

          

          if (!states.js) {
            PERIODIC_CALL(std::cout << "Joystick not connected" << std::endl, 3s);
            cmd.vx = req.vx = 0;
            cmd.vy = req.vy = 0;
            cmd.wz = req.wz = 0;
            cmd.vc = req.vc = 0;
            state = STATE::Idle;
            signal.tx.cmd_ad_state.send(false);
            signal.tx.cmd_tc_state.send(false);
            signal.tx.cmd_cc_state.send(false);
            signal.tx.cmd_cm_state.send(true);
            signal.tx.cmd_wm_state.send(true);
            signal.tx.cmd_cv_state.send(false);
            signal.tx.cmd_mk_state.send(false);
            signal.tx.cmd_dq_state.send(false);
            signal.tx.cmd_js_state.send(true);

          } else {
            if (state == STATE::Idle) {
              printf("state_machine: state initialize\n");
              state = STATE::Manual;

              signal.tx.cmd_cm_state.send(true);
              signal.tx.cmd_wm_state.send(true);
              signal.tx.cmd_cv_state.send(true);
              signal.tx.cmd_mk_state.send(true);
              signal.tx.cmd_dq_state.send(true);

            }
          }

          double factor = 0.5;
          cmd.vx = req.vx * (1-factor) + cmd.vx * factor;
          cmd.vy = req.vy * (1-factor) + cmd.vy * factor;
          cmd.wz = req.wz * (1-factor) + cmd.wz * factor;
          cmd.vc = req.vc * (1-factor) + cmd.vc * factor;

          if (abs(cmd.vx) < 0.01 && abs(req.vx) < 0.01) cmd.vx = 0;
          if (abs(cmd.vy) < 0.01 && abs(req.vy) < 0.01) cmd.vy = 0;
          if (abs(cmd.wz) < 0.01 && abs(req.wz) < 0.01) cmd.wz = 0;
          if (abs(cmd.vc) < 0.01 && abs(req.vc) < 0.01) cmd.vc = 0;

          switch (state) {
            case STATE::Manual: {
              se3 msg_mvel;
              msg_mvel.linear.x() = v_max * cmd.vx;
              msg_mvel.linear.y() = v_max * cmd.vy;
              msg_mvel.angular.z() = w_max * cmd.wz;
              signal.tx.vel_sv.send(msg_mvel);

              double msg_cvel;
              msg_cvel = vc_max * cmd.vc;
              signal.tx.crane_spd_sv.send(msg_cvel);

              static std::chrono::steady_clock::time_point l_stamp = std::chrono::steady_clock::now();
              if (std::chrono::steady_clock::now() >= l_stamp + 1s) {
                printf("MANUAL: vx=%lf, vy=%lf, wz=%lf, vc=%f, h=%lf\n", msg_mvel.linear.x(), msg_mvel.linear.y(), msg_mvel.angular.z(), msg_cvel, crane_pos);
                l_stamp = std::chrono::steady_clock::now();
              }

              if (crane_pos > 0) {
                state = STATE::Crane;
                std::cout << "system enter crane mode" << std::endl;
              }


  // // debug!!!
  // last = this->get_clock()->now();
  // crane_pos = -1;

              break;
            }
            case STATE::Crane: {
              se3 msg_mvel;
              msg_mvel.linear.x() = slow_factor * v_max * cmd.vx;
              msg_mvel.linear.y() = slow_factor * v_max * cmd.vy;
              msg_mvel.angular.z() = slow_factor * w_max * cmd.wz;
              signal.tx.vel_sv.send(msg_mvel);

              double msg_cvel;
              msg_cvel = vc_max * cmd.vc;
              signal.tx.crane_spd_sv.send(msg_cvel);

              static std::chrono::steady_clock::time_point l_stamp = std::chrono::steady_clock::now();
              if (std::chrono::steady_clock::now() >= l_stamp + 1s) {
                printf("CRANE: vx=%lf, vy=%lf, wz=%lf, vc=%f, h=%lf\n", 
                msg_mvel.linear.x(), msg_mvel.linear.y(), msg_mvel.angular.z(), msg_cvel, crane_pos);
                l_stamp = std::chrono::steady_clock::now();
              }

              if (crane_pos <= 0) {
                state = STATE::Manual;
                std::cout << "system enter manual mode" << std::endl;
              }
              
              break;
            }
            case STATE::Tracking: {
              bool recover = false;
              recover |= sig_ad;
              recover |= marker_lost >= marker_lost_max;
              recover |= !states.cv;
              // recover = false;
              if (recover) {
                signal.tx.cmd_tc_state.send(false);
                signal.tx.cmd_ad_state.send(false);
                if (sig_ad) printf("state machine: abnormal detected\n");
                if (marker_lost) printf("state machine: marker lost\n");

                double msg = elevation;
                printf("state machine: crane on\n");
                signal.tx.cmd_cc_state.send(true);
                states.cc = true;
                printf("state machine: crane up\n");
                signal.tx.height_sv.send(msg);
                printf("system enter recovery mode (to %lf m)\n", elevation);
                state = STATE::Recover;
                // signal.tx.cmd_wm_state.send(false);
              }
              
              break;
            }
            case STATE::Recover: {
  // // debug!!!
  // crane_pos += 0.001;
              if (!states.cc) {
                state = STATE::Crane;
                std::cout << "system enter crane mode" << std::endl;
                signal.tx.cmd_cc_state.send(false);
              } else {

                static std::chrono::steady_clock::time_point l_stamp = std::chrono::steady_clock::now();
                if (std::chrono::steady_clock::now() >= l_stamp + 1s) {
                  printf("RECOVER: height=%lf -> %lf\n", crane_pos, elevation);
                  l_stamp = std::chrono::steady_clock::now();
                }
              }
              break;
            }
            default: {
              se3 msg_mvel;
              msg_mvel.linear.x() = 0;
              msg_mvel.linear.y() = 0;
              msg_mvel.angular.z() = 0;
              signal.tx.vel_sv.send(msg_mvel);

              double msg_cvel;
              msg_cvel = 0;
              signal.tx.crane_spd_sv.send(msg_cvel);

              break;
            }
          }
        }
        printf("state machine: thread finishied\n");
      }).detach();
    }

  void watchdog_task() {
    if (++wdt.ad > 100) {
      PERIODIC_CALL(std::cout << "abnormal detection no signal" << std::endl, 10s);
      wdt.ad = 0;
      states.ad = false;
    }
    if (++wdt.cc > 100) {
      PERIODIC_CALL(std::cout << "crane control no signal" << std::endl, 10s);
      wdt.cc = 0;
      states.cc = false;
    }
    if (++wdt.cm > 100) {
      PERIODIC_CALL(std::cout << "crane motor no signal" << std::endl, 10s);
      wdt.cm = 0;
      states.cm = false;
    }
    if (++wdt.tc > 100) {
      PERIODIC_CALL(std::cout << "tracking control no signal" << std::endl, 10s);
      wdt.tc = 0;
      states.tc = false;
    }
    if (++wdt.wm > 100) {
      PERIODIC_CALL(std::cout << "wheel motor no signal" << std::endl, 10s);
      wdt.wm = 0;
      states.wm = false;
    }
    if (++wdt.cv > 100) {
      PERIODIC_CALL(std::cout << "vision no signal" << std::endl, 10s);
      wdt.cv = 0;
      states.cv = false;
    }
    if (++wdt.mk > 100) {
      PERIODIC_CALL(std::cout << "mecanum no signal" << std::endl, 10s);
      wdt.mk = 0;
      states.mk = false;
    }
    if (++wdt.dq > 100) {
      PERIODIC_CALL(std::cout << "DAQ no signal" << std::endl, 10s);
      wdt.dq = 0;
      states.dq = false;
    }
    if (++wdt.js > 100) {
      PERIODIC_CALL(std::cout << "joystick no signal" << std::endl, 10s);
      wdt.js = 0;
      states.js = false;
    }
  }

  ~CraneSM() {
    run = false;
  }

  private:
    bool active;
    bool run = true;
    std::chrono::steady_clock::time_point last;

    double crane_pos = 0;
    int marker_lost = 0;
    const int marker_lost_max = 20;
    bool sig_ad = false;
    bool is_vxy_reverse = false;
    
    struct {
      double vx;
      double vy;
      double wz;
      double vc;
    } req, cmd;

};