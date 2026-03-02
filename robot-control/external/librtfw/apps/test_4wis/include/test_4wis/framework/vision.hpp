#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <deque>
#include <thread>

#include <eigen3/Eigen/Dense>
#include "Spinnaker.h"
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#include "test_4wis/maf.hpp"
#include "test_4wis/frame.hpp"
#include "test_4wis/framework/signal.hpp"
#include "test_4wis/framework/util.hpp"
#include "test_4wis/framework/safe_queue.hpp"


using namespace std::chrono_literals;
using namespace project;


/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class PoseEstimator
{
  public:

    struct {
        SE3 pose;
        se3 twist;
        bool marker;
        std::chrono::steady_clock::time_point stamp;
    } result;
    
    PoseEstimator(int filter=5, bool is_gui=false, bool is_inv=false): fsize(filter), gui(is_gui), inv(is_inv) {
      // pub_state = this->create_publisher<std_msgs::msg::Bool>("state", 1);      
        
      fPos[0].resize(fsize);
      fPos[1].resize(fsize);
      fPos[2].resize(fsize);
      // fOrient[0].resize(2*fsize);
      // fOrient[1].resize(2*fsize);
      // fOrient[2].resize(2*fsize);
      // fOrient[3].resize(2*fsize);

      fVel[0].resize(30);
      fVel[1].resize(30);
      fVel[2].resize(30);
      fVel[3].resize(30);
      fVel[4].resize(30);
      fVel[5].resize(30);

      result.pose.R.setIdentity();
      result.pose.T.setZero();
      result.twist.linear.setZero();
      result.twist.angular.setZero();
      result.marker = false;
      result.stamp = std::chrono::steady_clock::now();

      signal.rx.state = std::make_shared<signal::Rx<bool>>([&](const bool& data) {
        if (data && !active) {
            set_active();
        }
        if (!data && active) {
            set_deactive();
        }
      });

      signal.rx.rec = std::make_shared<signal::Rx<bool>>([&](const bool& data) {
        if (data && !on_rec) {
            auto t = std::time(nullptr);
            auto tm = *std::localtime(&t);
            std::ostringstream oss;
            oss << std::put_time(&tm, "4wis_%Y-%m-%d_%H:%M:%S");
            oss << ".avi";
            writer.open(oss.str(), cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), video_fps, cv::Size(video_width, video_height), true);
            on_rec = true;
            std::cout << "image record start" << std::endl;
        }
        if (!data && on_rec) {
            on_rec = false;
        }
      });

        std::thread(std::bind(&PoseEstimator::task, this)).detach();
        std::thread(std::bind(&PoseEstimator::task2, this)).detach();
    }
    ~PoseEstimator() {
        if (active)
            set_deactive();
        run = false;
        printf("vision: terminated\n");
    }

    void set_active() {
        if (active) return;
        std::cout << "vision state=on" << std::endl;
        active = true;
        last_frame = 0;
        result.pose.T.setZero();
        result.pose.R.setIdentity();
        result.twist.linear.setZero();
        result.twist.angular.setZero();
        result.marker = false;

        maf_rot.clear();
        fPos[0].reset();
        fPos[1].reset();
        fPos[2].reset();
        fVel[0].reset();
        fVel[1].reset();
        fVel[2].reset();
        fVel[3].reset();
        fVel[4].reset();
        fVel[5].reset();
        lpf[0] = 0;
        lpf[1] = 0;
        lpf[2] = 0;
        lpf[3] = 0;
        lpf[4] = 0;
        lpf[5] = 0;
        signal.tx.state.send(active);
    }

    void set_deactive() {
        if (!active) return;
        std::cout << "vision state=off" << std::endl;
        active = false;
    }

    bool get_state() {
        return active;
    }

    bool is_outdate(std::chrono::steady_clock::time_point last) {
        return last < result.stamp;
    }

    struct {
        struct {
            signal::Tx<bool> state;
            signal::Tx<SE3> pose;
            signal::Tx<se3> twist;
            signal::Tx<bool> detection;
        } tx;
        struct {
            signal::Rx<bool>::SharedPtr state;
            signal::Rx<bool>::SharedPtr rec;
        } rx;
    } signal;

  private:

    void publish_pose(Eigen::Vector3d& position, Eigen::Quaterniond& orient, uint64_t frame_id, double fps) {
      Eigen::Vector3d pos;
      pos << fPos[0].update(position.x()), fPos[1].update(position.y()), fPos[2].update(position.z());
      
      // pos << (position.x()), (position.y()), (position.z());

      Eigen::AngleAxisd aa(orient);
      maf_rot.emplace_back(Eigen::AngleAxisd(aa.angle()/(4*fsize), aa.axis()));
      if (maf_rot.size() > (4*fsize)) {
        maf_rot.pop_front();
      }
      aa = Eigen::AngleAxisd::Identity();
      for (auto& rot: maf_rot)
        aa = rot * aa;
      // Eigen::Quaterniond ori(fOrient[0].update(orient.w()), fOrient[1].update(orient.x()), fOrient[2].update(orient.y()), fOrient[3].update(orient.z()));
      // aa = (ori);
   
      if (last_frame > 0) {
        double dt = (frame_id - last_frame)/fps;
        SE3 pose(aa.matrix(), pos);        
        auto vel = diff(pose, result.pose) / dt;
        if (vel.linear.hasNaN()) vel.linear.setZero();
        if (vel.angular.hasNaN()) vel.angular.setZero();


        if (marker_state == 4) {

            printf("linear error %.3lf\n", vel.linear.norm());
            printf("angular error %.3lf\n", vel.angular.norm());

            if (vel.linear.norm()*dt < 1e-1 && vel.angular.norm()*dt < 1e-1) {
            // if (true) {
                if (ready[1] && ready[2])
                    marker_state = 1;
                if (ready[1] && !ready[2])
                    marker_state = 2;
                if (!ready[1] && ready[2])
                    marker_state = 3;
            } else {
                vel.linear = vel.linear * dt*0.01;
                vel.angular = vel.angular * dt*0.01;
                // for (auto i=0; i<3; i++) {
                //     if (vel.linear[i] < -1e-3) vel.linear[i] = -1e-3;
                //     if (vel.linear[i] > 1e-3) vel.linear[i] = 1e-3;

                //     if (vel.angular[i] < -1e-2) vel.angular[i] = -1e-2;
                //     if (vel.angular[i] > 1e-2) vel.angular[i] = 1e-2;
                // }

                auto norm = vel.angular.norm();
                auto vect = vel.angular / norm;        
                pose = SE3(Eigen::AngleAxisd(norm, vect).matrix(), vel.linear) * result.pose;
                pos = pose.T;
                aa = pose.R;
            }
        }

        // angular velocity has too big variance
        // vel.linear = (pose.T - result.pose.T) / dt;
        // auto old_aa = Eigen::AngleAxisd(Eigen::Quaterniond(pose.R));
        // vel.angular = (aa.angle()*aa.axis() - old_aa.angle()*old_aa.axis()) / dt;

        auto vx = fVel[0].update(vel.linear[0]);
        auto vy = fVel[1].update(vel.linear[1]);
        auto vz = fVel[2].update(vel.linear[2]);
        auto wx = fVel[3].update(vel.angular[0]);
        auto wy = fVel[4].update(vel.angular[1]);
        auto wz = fVel[5].update(vel.angular[2]);

        double factor = 0.2;//0.95;
        lpf[0] = lpf[0] * factor + vel.linear[0] * (1-factor);
        lpf[1] = lpf[1] * factor + vel.linear[1] * (1-factor);
        lpf[2] = lpf[2] * factor + vel.linear[2] * (1-factor);
        factor = 0.5;//0.98;
        lpf[3] = lpf[3] * factor + vel.angular[0] * (1-factor);
        lpf[4] = lpf[4] * factor + vel.angular[1] * (1-factor);
        lpf[5] = lpf[5] * factor + vel.angular[2] * (1-factor);
        
        // result.twist.linear << vx, vy, vz;
        result.twist.linear << lpf[0], lpf[1], lpf[2];
        // result.twist.angular << wx, wy, wz;
        result.twist.angular << lpf[3], lpf[4], lpf[5];
      } else {
        result.twist.linear.setZero();
        result.twist.angular.setZero();
      }


      result.pose.T << pos;
      result.pose.R = aa;
      result.marker = true;
      result.stamp = std::chrono::steady_clock::now();

      signal.tx.pose.send(result.pose);
      signal.tx.twist.send(result.twist);
      last_frame = frame_id;
    }
public:
    void task() {
        double fx, fy, cx, cy, k1, k2, p1, p2, k3, k4;

        printf("try get spinnaker system instance\n");
        Spinnaker::SystemPtr system = Spinnaker::System::GetInstance();
        Spinnaker::CameraList camList = system->GetCameras();
        unsigned int numCameras = camList.GetSize();

        while(numCameras < 1) {
            std::this_thread::sleep_for(1s);
            camList = system->GetCameras();
            numCameras = camList.GetSize();
            printf("wait camera...(%d)\n", numCameras);
        }
        
        printf("get camera succeed(%d)\n", numCameras);

        Spinnaker::CameraPtr pCam;
        pCam = camList.GetByIndex(0);
        // Spinnaker::GenApi::INodeMap& nodeMapTLDevice = pCam->GetTLDeviceNodeMap();
        pCam->Init();

        pCam->DecimationVertical.SetValue(1);
        // pCam->AcquisoitionFrameRate.SetValue(pCam->AcquisitionFrameRate.GetMax());
        pCam->AcquisitionFrameRate.SetValue(30);
        pCam->ReverseX.SetValue(inv);
        pCam->ReverseY.SetValue(inv);
        pCam->BeginAcquisition();
        std::cout << "Camera " << 0 << std::endl;
        std::cout << "  Resolution=" << pCam->Width.GetValue() << "x" << pCam->Height.GetValue() << std::endl;
        std::cout << "  FPS=" << pCam->AcquisitionFrameRate.GetValue() << std::endl;


        // Spinnaker::GenApi::INodeMap& nodeMapTLDevice = pCam->GetTLDeviceNodeMap();
        // Spinnaker::GenApi::INodeMap& nodeMap = pCam->GetNodeMap();


        // Spinnaker::GenApi::CEnumerationPtr ptrFrameRateAuto =
        //     pCam->GetNodeMap().GetNode("AcquisitionFrameRateAuto");
        // if (!Spinnaker::GenApi::IsAvailable(ptrFrameRateAuto) || !Spinnaker::GenApi::IsWritable(ptrFrameRateAuto)) {
        //   std::cout << "Unable to set AcquisitionFrameRateAuto..." << std::endl;
        // } else {

        //   auto ptrFrameRateAutoMode = ptrFrameRateAuto->GetEntryByName("Off");
        //   if (!Spinnaker::GenApi::IsAvailable(ptrFrameRateAutoMode) ||
        //       !Spinnaker::GenApi::IsReadable(ptrFrameRateAutoMode)) {
        //     std::cout << "Unable to set AcquisitionFrameRateAuto to OFF. Aborting..." << std::endl;
        //   }
        //   // Set value
        //   auto value = ptrFrameRateAutoMode->GetValue();
        //   ptrFrameRateAuto->SetIntValue(value);
        // }

        

        // Spinnaker::GenApi::CEnumerationPtr ptrAcquisitionMode = nodeMap.GetNode("AcquisitionMode");
        // if (!Spinnaker::GenApi::IsAvailable(ptrAcquisitionMode) || !Spinnaker::GenApi::IsWritable(ptrAcquisitionMode))
        // {
        //     std::cout << "Unable to set acquisition mode to continuous (enum retrieval). Aborting..." << std::endl;
        //     return -1;
        // }

        // // Retrieve entry node from enumeration node
        // Spinnaker::GenApi::CEnumEntryPtr ptrAcquisitionModeContinuous = ptrAcquisitionMode->GetEntryByName("Continuous");
        // if (!IsAvailable(ptrAcquisitionModeContinuous) || !IsReadable(ptrAcquisitionModeContinuous))
        // {
        //     std::cout << "Unable to set acquisition mode to continuous (entry retrieval). Aborting..." << std::endl;
        //     return -1;
        // }

        // Retrieve integer value from entry node
        // const int64_t acquisitionModeContinuous = ptrAcquisitionModeContinuous->GetValue();

        // Set integer value from entry node as new value of enumeration node
        // ptrAcquisitionMode->SetIntValue(acquisitionModeContinuous);

        //
        // Begin acquiring images
        //
        // *** NOTES ***
        // What happens when the camera begins acquiring images depends on the
        // acquisition mode. Single frame captures only a single image, multi
        // frame captures a set number of images, and continuous captures a
        // continuous stream of images. Because the example calls for the
        // retrieval of 50 images, continuous mode has been set.
        //
        // *** LATER ***
        // Image acquisition must be ended when no more images are needed.
        //

            // // fisheye
        int width = pCam->Width.GetValue();
        int height = pCam->Height.GetValue();
        double fps = pCam->AcquisitionFrameRate.GetValue();
        fx = 217.0699094099272;
        fy = 216.7747232661944;
        cx = 513.838218417913;
        cy = 377.2949596710763;
        k1 = 0.00862554;
        k2 = -0.0280815;
        k3 = 0.0146489;
        k4 = -0.00225029;

        cv::Mat cameraMatrix = cv::Mat(3,3, cv::DataType<double>::type);
        cv::Mat distortionCoeffs = cv::Mat(4,1, cv::DataType<double>::type);

        cameraMatrix.at<double>(0, 0) = fx*2;
        cameraMatrix.at<double>(0, 1) = 0;
        cameraMatrix.at<double>(0, 2) = cx*2;
        cameraMatrix.at<double>(1, 0) = 0;
        cameraMatrix.at<double>(1, 1) = fy*2;
        cameraMatrix.at<double>(1, 2) = cy*2;
        cameraMatrix.at<double>(2, 0) = 0;
        cameraMatrix.at<double>(2, 1) = 0;
        cameraMatrix.at<double>(2, 2) = 1;

        distortionCoeffs.at<double>(0,0) = k1;
        distortionCoeffs.at<double>(1,0) = k2;
        distortionCoeffs.at<double>(2,0) = k3;
        distortionCoeffs.at<double>(3,0) = k4;

        cv::Mat E = cv::Mat::eye(3, 3, cv::DataType<double>::type);  

        printf("width=%d, height=%d, fps=%.0lf\n", width, height, fps);
        video_fps = fps;
        video_width = width;
        video_height = height;

        printf("pose estimator start\n");

        cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
        // parameters->cornerRefinementMaxIterations = 10;
        cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);

        float markerLength = 0.1;	// marker size(Unit: m)

        // set coordinate system
        cv::Mat objPoints(4, 1, CV_32FC3);
        objPoints.ptr<cv::Vec3f>(0)[0] = cv::Vec3f(-markerLength/2.f, markerLength/2.f, 0);
        objPoints.ptr<cv::Vec3f>(0)[1] = cv::Vec3f(markerLength/2.f, markerLength/2.f, 0);
        objPoints.ptr<cv::Vec3f>(0)[2] = cv::Vec3f(markerLength/2.f, -markerLength/2.f, 0);
        objPoints.ptr<cv::Vec3f>(0)[3] = cv::Vec3f(-markerLength/2.f, -markerLength/2.f, 0);
    
        double totalTime = 0;
        auto totalIterations = 0;
        auto cal_fps = fps;
        auto cnt = 0;

        auto last = std::chrono::steady_clock::now();
        uint64_t frame_id = 0;
        cv::Mat image;//, undist;

        cv::Vec3d rvec[3];
        cv::Vec3d tvec[3];
        while(run) {
            // std::this_thread::sleep_for(1s/fps);
            // rate.sleep();

            if (!active) {
                std::this_thread::sleep_for(100ms);
                continue;
            }


            Spinnaker::ImagePtr pResultImage;
            try {
                pResultImage = pCam->GetNextImage(1000);


                if (pResultImage->IsIncomplete())
                {
                    // Retrieve and print the image status description
                    std::cout << "Image incomplete: "
                        << Spinnaker::Image::GetImageStatusDescription(pResultImage->GetImageStatus())
                        << "..." << std::endl;
                    continue;
                } else {
                    int width = pResultImage->GetWidth();
                    int height = pResultImage->GetHeight();
                    cv::Mat img(height, width, CV_8UC1, pResultImage->GetData());
                    cv::cvtColor(img, image, cv::COLOR_BayerRG2RGB);

                    // cv::Size size = { image.cols, image.rows };
                    // cv::Mat map1;
                    // cv::Mat map2;
                    // cv::fisheye::initUndistortRectifyMap(cameraMatrix, distortionCoeffs, E, cameraMatrix, size, CV_16SC2, map1, map2);
                    // cv::remap(image, undist, map1, map2, cv::INTER_LINEAR, CV_HAL_BORDER_CONSTANT);

                }
            } catch (...) {
                active = false;
                std::cout << "GetNextImage Failed" << std::endl;
                break;
            }

            auto tick = std::chrono::steady_clock::now();

            //
            // Ensure image completion
            //
            // *** NOTES ***
            // Images can easily be checked for completion. This should be
            // done whenever a complete image is expected or required.
            // Further, check image status for a little more insight into
            // why an image is incomplete.
            //
            ready[0] = false;
            ready[1] = false;
            ready[2] = false;

            std::vector<int> ids;
            std::vector<std::vector<cv::Point2f> > corners, rejected;

            if (!image.empty())
                cv::aruco::detectMarkers(image, dictionary, corners, ids, parameters, rejected, cameraMatrix, distortionCoeffs);
            else
                printf("image %d empty!\n", 0);

            for (auto idx=0; idx<2; idx++) {
                // double tick = (double)cv::getTickCount();

                // cv::aruco::detectMarkers(image, dictionary, corners, ids, parameters, rejected, cameraMatrix, distortionCoeffs);

                if(!ids.empty()) {
                    // Calculate pose for each marker
                    int nMarkers = corners.size();
                    for (int i = 0; i < nMarkers; i++) {
                        if (ids[i] != 1 && ids[i] != 2 && ids[i] != 3)
                            continue;
                        // if (solvePnP(objPoints, corners.at(i), camMatrix, distCoeffs, rvec, tvec, false, cv::SOLVEPNP_IPPE_SQUARE)) {
                        // if (solvePnP(objPoints, corners.at(i), camMatrix, distCoeffs, rvec, tvec, false, cv::SOLVEPNP_EPNP)) {
                        if (solvePnP(objPoints, corners[i], cameraMatrix, distortionCoeffs, rvec[ids[i]-1], tvec[ids[i]-1], false, cv::SOLVEPNP_ITERATIVE)) {
                            ready[ids[i]-1] = true;
                            // printf("marker(id=%d) detected\n", ids[i]);
                        } else {
                            PERIODIC_CALL(std::cout << "pose estimation failed" << std::endl, 1s);
                        }
                    }
                    // if (gui)
                        cv::aruco::drawDetectedMarkers(image, corners, ids);
                }
                else {
                    // PERIODIC_CALL(std::cout << "marker detection failed" << std::endl, 1s);
                // RCLCPP_WARN(node->get_logger(), "marker detection failed");
                }

            }
            
            SE3 c_X_mi[2];
            for (auto i=0; i<2; i++) {
                if (!ready[i+1])
                    continue;
                Eigen::Vector3d position;
                Eigen::AngleAxisd aa;

                position << tvec[i+1][0], tvec[i+1][1], tvec[i+1][2];

                auto angle = sqrt(rvec[i+1].dot(rvec[i+1]));
                Eigen::Vector3d axis;
                axis << rvec[i+1][0], rvec[i+1][1], rvec[i+1][2];
                // if (axis[2] )
                axis = axis/angle;
                aa = Eigen::AngleAxisd(angle, axis);
                c_X_mi[i] = SE3(aa.matrix(), position);
            }
            SE3 c_X_m;
            
            if (ready[1] || ready[2]) {

                // 0: idle, 1: both, 2: marker[0], 3: marker[1], 4: transient
                if (ready[1] && ready[2]) 
                    switch (marker_state) {
                        case 0:
                            marker_state = 1;
                            printf("marker_state: both\n");
                            break;
                        case 2:
                        case 3:
                            marker_state = 4;
                            printf("marker_state: transient\n");
                            break;
                    }
                if (ready[1] && !ready[2])
                    switch (marker_state) {
                        case 0:
                            marker_state = 2;
                            printf("marker_state: marker[0]\n");
                            break;
                        case 1:
                        case 3:
                            marker_state = 4;
                            printf("marker_state: transient\n");
                            break;
                    }
                if (!ready[1] && ready[2])
                    switch (marker_state) {
                        case 0:
                            marker_state = 3;
                            printf("marker_state: marker[1]\n");
                            break;
                        case 1:
                        case 2:
                            marker_state = 4;
                            printf("marker_state: transient\n");
                            break;
                    }

                if (ready[1] && ready[2]) {
                    m0_X_m1 = c_X_mi[0].inverse() * c_X_mi[1];
                }
                
                if (ready[2]) {
                    SE3 E = SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());
                    se3 m0_x_m1 = diff(m0_X_m1, E);
                    se3 m_x_m1(m0_x_m1.linear*0.5, m0_x_m1.angular*0.5);
                    auto norm = m_x_m1.angular.norm();
                    auto vect = m_x_m1.angular / norm;
    
                    SE3 m_X_m1(Eigen::AngleAxisd(norm, vect).matrix(), m_x_m1.linear);
    
                    c_X_m = c_X_mi[1] * m_X_m1.inverse();
                } else {
                    SE3 E = SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());
                    se3 m1_x_m0 = diff(m0_X_m1.inverse(), E);
                    se3 m_x_m0(m1_x_m0.linear*0.5, m1_x_m0.angular*0.5);
                    auto norm = m_x_m0.angular.norm();
                    auto vect = m_x_m0.angular / norm;
    
                    SE3 m_X_m0(Eigen::AngleAxisd(norm, vect).matrix(), m_x_m0.linear);
    
                    c_X_m = c_X_mi[0] * m_X_m0.inverse();
                }

                Eigen::Matrix3d r_R_m;
                SE3 n_X_c;
                r_R_m << 0, 0, -1,
                            -1, 0, 0,
                            0, 1, 0;

                n_X_c.R << 0, 0, 1,
                            -1, 0, 0,
                            0, -1, 0;
                n_X_c.T << 0.3, 0, 0;
                // 0.50714 w2w
                // 0.1064 w2f
                // ~-0.3
                SE3 n_X_r = n_X_c* c_X_m * r_R_m.transpose();

                Eigen::Vector3d pos(n_X_r.T);
                Eigen::Quaterniond orient(n_X_r.R);
                publish_pose(pos, orient, frame_id, fps);
               
                {
                    SE3 coord = n_X_c.inverse() * result.pose * r_R_m;
                    cv::Vec3d m_rvec, m_tvec;
                    Eigen::Quaterniond quat(coord.R);
                    Eigen::AngleAxisd aa(quat);
                    Eigen::Vector3d rvec = aa.angle() * aa.axis();

                    m_rvec[0] = rvec.x();
                    m_rvec[1] = rvec.y();
                    m_rvec[2] = rvec.z();

                    m_tvec[0] = coord.T.x();
                    m_tvec[1] = coord.T.y();
                    m_tvec[2] = coord.T.z();
                    cv::aruco::drawAxis(image, cameraMatrix, distortionCoeffs, m_rvec, m_tvec, 0.1);

                }
                result.marker = true;
            } else {
                marker_state = 0;
                // printf("marker_state: idle\n");
                result.marker = false;
            }

            signal.tx.detection.send(result.marker);

            if (on_rec) {
                img_buf.Produce(std::make_shared<cv::Mat>(image.clone()));
            }

            if (gui) {
                cv::resize(image, image, cv::Size(1024, 768));
                cv::imshow("image", image);
                cv::waitKey(1);
            }
            frame_id++;



            auto now = std::chrono::steady_clock::now();
            cnt++;
            if (now - last >= 1s) {
                cal_fps = cnt;
                cnt = 0;
                last = now;
                auto currentTime = std::chrono::duration_cast<std::chrono::milliseconds>(now - tick).count();
                printf("fps=%.0lf, detection_time=%d ms, detection=%s\n", cal_fps, currentTime, result.marker? "true": "false");
                // RCLCPP_INFO(node->get_logger(), "markers=%d", ids.size());
            }
            // std::this_thread::sleep_for(10ms);
        }

        printf("vision: thread finished\n");
        for (auto idx=0; idx<2; idx++) {
            pCam->AcquisitionStop();
            pCam->DeInit();
        }
        system->ReleaseInstance();
    }

    void task2() {
        while (run) {
            // if (!writer.isOpened() || img_buf.empty()) {
            if (!writer.isOpened() || img_buf.Size() == 0) {
                std::this_thread::sleep_for(1s);
                continue;
            }

            // if (!on_rec && writer.isOpened() && img_buf.empty()) {
            if (!on_rec && writer.isOpened() && img_buf.Size() == 0) {
                writer.release();
                std::cout << "image record end" << std::endl;
            }

            // auto img_ptr = img_buf.front();
            // img_buf.pop();
            std::shared_ptr<cv::Mat> img_ptr;
            if (img_buf.Consume(img_ptr)) {
                cv::Mat& img = *img_ptr;
                writer << img;
            }

            std::this_thread::sleep_for(10ms);
        }
        printf("vision: thread2 finished\n");
    }

    uint64_t last_frame = 0;
    bool active = false;
    bool gui = false;
    bool inv = false;
    bool run = true;
    
    MAF<double> fPos[3];
    MAF<double> fVel[6];
    double lpf[6];
    // MAF<double> fOrient[4];
    std::deque<Eigen::AngleAxisd> maf_rot;
    int fsize;
    bool on_rec = false;
    cv::VideoWriter writer;
    int video_fps;
    int video_width;
    int video_height;
    // std::queue<std::shared_ptr<cv::Mat>> img_buf;
    SafeQueue<std::shared_ptr<cv::Mat>> img_buf;


    bool ready[3] = {false};
    SE3 m0_X_m1;
    int marker_state = 0; // 0: idle, 1: both, 2: marker[0], 3: marker[1], 4: transient
};


// fuser /dev/video0
// kill -9 <pid>