#include "test_4wis/framework/state_machine.hpp"
#include "test_4wis/framework/joystick.hpp"
#include "test_4wis/framework/kin_4wis.hpp"
#include "test_4wis/framework/wheel_if.hpp"
#include "test_4wis/framework/tracking_control.hpp"
#include "test_4wis/framework/vision.hpp"
#include "test_4wis/framework/abnormal_detector.hpp"
#include "test_4wis/frame.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "json.hpp"
#include <fstream>
#include <chrono>


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("test_4wis");
    node->declare_parameter("gui", false);
    bool is_gui = false;
    node->get_parameter("gui", is_gui);
    node->declare_parameter("inv", false);
    bool is_inv = false;
    node->get_parameter("inv", is_inv);
    
    CraneSM sm;
    JoystickNode js;
    PoseEstimator vision(5, is_gui, is_inv);
    KinFourwis mk(50);
    WheelIF wm;
    TrackingNode tc;
    AbnormalDetectorNode ad;

    // wm.param.rpm_max = 30;
    sm.v_max = mk.vel_max.linear[0];// * 0.5;
    sm.w_max = mk.vel_max.angular[2] * 0.5;
    tc.v_max = mk.vel_max.linear[0];// * 0.5;
    tc.w_max = mk.vel_max.angular[2] * 0.5;

#pragma region SIGNAL_CONNECTION

    sm.signal.tx.cmd_cv_state += (vision.signal.rx.state);
    sm.signal.tx.cmd_js_state += (js.signal.rx.state);
    sm.signal.tx.cmd_wm_state += (wm.signal.rx.state);
    sm.signal.tx.cmd_mk_state += (mk.signal.rx.state);
    sm.signal.tx.cmd_tc_state += (tc.signal.rx.state);
    sm.signal.tx.cmd_ad_state += (ad.signal.rx.state);

    sm.signal.tx.vel_sv += (mk.signal.rx.velocity);

    vision.signal.tx.state += (sm.signal.rx.cv_state);
    vision.signal.tx.detection += (sm.signal.rx.marker_detection);
    vision.signal.tx.pose += (tc.signal.rx.pose);
    vision.signal.tx.twist += (tc.signal.rx.robot_vel);
    vision.signal.tx.pose += (ad.signal.rx.pose);
    vision.signal.tx.twist += (ad.signal.rx.vel);

    js.signal.tx.state += (sm.signal.rx.js_state);
    // // Logitech F710
    // js.signal.tx.axis[0] += (sm.signal.rx.cmd_vy);
    // js.signal.tx.axis[1] += (sm.signal.rx.cmd_vx);
    // js.signal.tx.axis[2] += (sm.signal.rx.cmd_vc_down);
    // js.signal.tx.axis[3] += (sm.signal.rx.cmd_wz);
    // js.signal.tx.axis[5] += (sm.signal.rx.cmd_vc_up);
    // js.signal.tx.btn[0] += (sm.signal.rx.cmd_tracking_on);
    // js.signal.tx.btn[1] += (sm.signal.rx.cmd_tracking_off);
    // js.signal.tx.btn[2] += (sm.signal.rx.cmd_origin_reset);
    // js.signal.tx.btn[3] += (sm.signal.rx.cmd_soft_emg);
    // js.signal.tx.btn[4] += (sm.signal.rx.cmd_crane_teach);
    // js.signal.tx.btn[5] += (sm.signal.rx.cmd_vxy_reverse);

    // Xbox controller
    js.signal.tx.axis[0] += (sm.signal.rx.cmd_vy);
    js.signal.tx.axis[1] += (sm.signal.rx.cmd_vx);
    js.signal.tx.axis[5] += (sm.signal.rx.cmd_vc_down);
    js.signal.tx.axis[2] += (sm.signal.rx.cmd_wz);
    js.signal.tx.axis[4] += (sm.signal.rx.cmd_vc_up);
    js.signal.tx.btn[0] += (sm.signal.rx.cmd_tracking_on);
    js.signal.tx.btn[1] += (sm.signal.rx.cmd_tracking_off);
    js.signal.tx.btn[3] += (sm.signal.rx.cmd_origin_reset);
    js.signal.tx.btn[4] += (sm.signal.rx.cmd_soft_emg);
    js.signal.tx.btn[6] += (sm.signal.rx.cmd_crane_teach);
    js.signal.tx.btn[7] += (sm.signal.rx.cmd_vxy_reverse);



    mk.signal.tx.state += (sm.signal.rx.mk_state);
    mk.signal.tx.wheel += (wm.signal.rx.wheel);
    mk.signal.tx.velocity += (tc.signal.rx.mobile_vel);

    wm.signal.tx.state += (sm.signal.rx.wm_state);
    wm.signal.tx.wheel += (mk.signal.rx.wheel);

    tc.signal.tx.state += (sm.signal.rx.tc_state);
    tc.signal.tx.mobile_vel += (mk.signal.rx.velocity);

    ad.signal.tx.state += (sm.signal.rx.ad_state);
    ad.signal.tx.abnormal += (sm.signal.rx.abnormal);

#pragma endregion

    bool rec = false;
    bool req_rec = false;
    signal::Tx<bool> rec_state;
    rec_state += vision.signal.rx.rec;
    

    auto db_ctrl = std::make_shared<signal::Rx<bool>>([&](const bool& data) {
        req_rec = data;
    });

    auto db_on = std::make_shared<signal::Rx<bool>>([&](const bool& data) {
        if (data && !req_rec) req_rec = true;
    });
    auto db_off = std::make_shared<signal::Rx<bool>>([&](const bool& data) {
        if (data && req_rec) req_rec = false;
    });

    sm.signal.tx.cmd_tc_state += db_ctrl;
    js.signal.tx.btn[10] += db_on;
    js.signal.tx.btn[11] += db_off;

    

    auto pub_rpm_sv = node->create_publisher<std_msgs::msg::Float32MultiArray>("rpm_sv", 1);
    auto pub_vel_sv = node->create_publisher<geometry_msgs::msg::Twist>("vel_sv", 1);
    auto pub_robot_pose = node->create_publisher<geometry_msgs::msg::Pose>("robot_pose", 1);
    auto pub_robot_vel = node->create_publisher<geometry_msgs::msg::Twist>("robot_vel", 1);
    auto pub_rpm_pv = node->create_publisher<std_msgs::msg::Float32MultiArray>("rpm_pv", 1);
    auto pub_vel_pv = node->create_publisher<geometry_msgs::msg::Twist>("vel_pv", 1);
    auto pub_pose_error = node->create_publisher<geometry_msgs::msg::Twist>("pose_error", 1);

    std_msgs::msg::Float32MultiArray msg_rpm_sv;
    geometry_msgs::msg::Twist msg_vel_sv;
    geometry_msgs::msg::Pose msg_robot_pose;
    geometry_msgs::msg::Twist msg_robot_vel;
    std_msgs::msg::Float32MultiArray msg_rpm_pv;
    geometry_msgs::msg::Twist msg_vel_pv;
    geometry_msgs::msg::Twist msg_pose_error;
  
    std::vector<std::array<double, 4>> db_rpm_sv;
    std::vector<std::array<double, 4>> db_rpm_pv;
    std::vector<std::array<double, 4>> db_steer_sv;
    std::vector<std::array<double, 4>> db_steer_pv;
    std::vector<std::array<double, 3>> db_vel_sv;
    std::vector<std::array<double, 3>> db_vel_pv;
    std::vector<std::array<double, 6>> db_robot_pose;
    std::vector<std::array<double, 6>> db_robot_vel;
    std::vector<std::array<double, 6>> db_pose_error;
    std::vector<uint8_t> db_abnormal;

    rclcpp::Rate rate(30);
    while (rclcpp::ok()) {

        if (req_rec && !rec) {
            std::cout << "db record start" << std::endl;
            db_rpm_sv.clear();
            db_rpm_pv.clear();
            db_steer_sv.clear();
            db_steer_pv.clear();
            db_vel_sv.clear();
            db_vel_pv.clear();
            db_robot_pose.clear();
            db_robot_vel.clear();
            db_pose_error.clear();
            db_abnormal.clear();

            rec = req_rec;
            rec_state.send(rec);
        }

        if (!req_rec && rec) {
            std::cout << "db record stop" << std::endl;
            
            auto t = std::time(nullptr);
            auto tm = *std::localtime(&t);
            std::ostringstream oss;
            oss << std::put_time(&tm, "%Y-%m-%d_%H:%M:%S");

            std::cout << "writing log 4wis_" << oss.str() << ".json" << std::endl;
            nlohmann::json j;
            j["program"] = "test_4wis/standalone";
            j["date"] = oss.str();
            j["sampling"] = "30hz";
            j["length"] = db_rpm_sv.size();
            j["data"] = {};
            j["data"]["rpm_sv"] = db_rpm_sv;
            j["data"]["rpm_pv"] = db_rpm_pv;
            j["data"]["steer_sv"] = db_steer_sv;
            j["data"]["steer_pv"] = db_steer_pv;
            j["data"]["vel_sv"] = db_vel_sv;
            j["data"]["vel_pv"] = db_vel_pv;
            j["data"]["robot_pose"] = db_robot_pose;
            j["data"]["robot_vel"] = db_robot_vel;
            j["data"]["pose_error"] = db_pose_error;
            j["data"]["abnormal"] = db_abnormal;

            std::string json_string = j.dump(2, ' ', true);
            std::ofstream ofs("4wis_" + oss.str() + ".json");
            if (ofs.is_open()) {
                ofs << json_string;
                ofs.close();
                std::cout << "export log file succeed" << std::endl;
            } else {
                std::cout << "export log file failed" << std::endl;
            }

            rec = req_rec;
            rec_state.send(rec);
        }

        if (rec) {
            db_rpm_sv.push_back(wm.rpm_sv);
            db_rpm_pv.push_back(wm.rpm_pv);
            db_steer_sv.push_back(wm.steer_sv);
            db_steer_pv.push_back(wm.steer_pv);
            db_vel_sv.push_back({mk.vel_sv.linear.x(), mk.vel_sv.linear.y(), mk.vel_sv.angular.z()});
            db_vel_pv.push_back({mk.vel_pv.linear.x(), mk.vel_pv.linear.y(), mk.vel_pv.angular.z()});
            SE3 identity;
            se3 pose = diff(vision.result.pose, identity);
            db_robot_pose.push_back({pose.linear.x(), pose.linear.y(), pose.linear.z(), 
                                    pose.angular.x(), pose.angular.y(), pose.angular.z()});
            db_robot_vel.push_back({vision.result.twist.linear.x(), vision.result.twist.linear.y(), vision.result.twist.linear.z(), 
                                    vision.result.twist.angular.x(), vision.result.twist.angular.y(), vision.result.twist.angular.z()});
            db_pose_error.push_back({tc.pose_error.linear.x(), tc.pose_error.linear.y(), tc.pose_error.linear.z(),
                                    tc.pose_error.angular.x(), tc.pose_error.angular.y(), tc.pose_error.angular.z()});
            db_abnormal.push_back(ad.result.abnormal);
        }

        msg_rpm_sv.data.resize(4);
        for (auto i=0; i<4; i++)
            msg_rpm_sv.data[i] = wm.rpm_sv[i];
        msg_vel_sv.linear.x = mk.vel_sv.linear.x();
        msg_vel_sv.linear.y = mk.vel_sv.linear.y();
        msg_vel_sv.angular.x = mk.vel_sv.angular.z();
        msg_robot_pose.position.x = vision.result.pose.T.x();
        msg_robot_pose.position.y = vision.result.pose.T.y();
        msg_robot_pose.position.z = vision.result.pose.T.z();
        Eigen::Quaterniond quat(vision.result.pose.R);
        msg_robot_pose.orientation.w = quat.w();
        msg_robot_pose.orientation.x = quat.x();
        msg_robot_pose.orientation.y = quat.y();
        msg_robot_pose.orientation.z = quat.z();
        msg_robot_vel.linear.x = vision.result.twist.linear.x();
        msg_robot_vel.linear.y = vision.result.twist.linear.y();
        msg_robot_vel.linear.z = vision.result.twist.linear.z();
        msg_robot_vel.angular.x = vision.result.twist.angular.x();
        msg_robot_vel.angular.y = vision.result.twist.angular.y();
        msg_robot_vel.angular.z = vision.result.twist.angular.z();
        msg_rpm_pv.data.resize(4);
        for (auto i=0; i<4; i++)
            msg_rpm_pv.data[i] = wm.rpm_pv[i];
        msg_vel_pv.linear.x = mk.vel_pv.linear.x();
        msg_vel_pv.linear.y = mk.vel_pv.linear.y();
        msg_vel_pv.angular.x = mk.vel_pv.angular.z();
        msg_pose_error.linear.x = tc.pose_error.linear.x();
        msg_pose_error.linear.y = tc.pose_error.linear.y();
        msg_pose_error.linear.z = tc.pose_error.linear.z();
        msg_pose_error.angular.x = tc.pose_error.angular.x();
        msg_pose_error.angular.y = tc.pose_error.angular.y();
        msg_pose_error.angular.z = tc.pose_error.angular.z();

        pub_rpm_sv->publish(msg_rpm_sv);
        pub_vel_sv->publish(msg_vel_sv);
        pub_robot_pose->publish(msg_robot_pose);
        pub_robot_vel->publish(msg_robot_vel);
        pub_rpm_pv->publish(msg_rpm_pv);
        pub_vel_pv->publish(msg_vel_pv);
        pub_pose_error->publish(msg_pose_error);

        rate.sleep();
        if (rclcpp::ok())
            rclcpp::spin_some(node);
    }

    node = nullptr;
    rclcpp::shutdown();
    std::cout << "standalone terminated" << std::endl;
    return 0;
}