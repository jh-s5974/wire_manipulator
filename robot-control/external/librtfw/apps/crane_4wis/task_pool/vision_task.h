#pragma once

#include "rtfw/task.h"
#include "interfaces/CvResultData.h"
#include <manif/manif.h>
#include "util.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <deque>
#include <thread>
#include <future>
#include <fstream>

#include <eigen3/Eigen/Dense>
#include "Spinnaker.h"
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include "json.hpp"


enum class SM_State {
    IDLE,
    CONF,
    ACTIVE,
    INACTIVE,
    UNCONF,
};

using namespace rtfw;

namespace task_pool {

    class VisionTask : public rt::ITask {
    public:
        ~VisionTask() {
            if (active)
                set_deactive();
            run = false;

            if (state != SM_State::IDLE)
                SM_UNCONF();

            try {
                if (system.IsValid())
                    system->ReleaseInstance();
            } catch(Spinnaker::Exception e) {
                getLogger()->error("[{}] spinnaker system ReleaseInstance failed : {}", getName(), e.GetErrorMessage());
            }
        }
        const char* getName() const override { return "Vision_Task"; }
        void setup(rt::TaskRegistry& r) override {
            r.add_dependency(dr_cmd_state);
            r.add_dependency(dr_cmd_rec_start);
            r.add_dependency(dr_cmd_rec_stop);
            r.add_dependency(dr_cmd_setup);
            r.add_dependency(dr_target_robot);
            r.add_dependency(dr_target_cur);
            r.add_dependency(dr_target_ref);
            r.add_dependency(dw_state);
            r.add_dependency(dw_detect);
            r.add_dependency(dw_data);
        }
        void initialize() override {
            reset_result();

            std::thread(std::bind(&VisionTask::task2, this)).detach();
        }
        void execute() override {
            dr_cmd_state.on_update([this](const bool& data) {
                if (data && !active) {
                    set_active();
                }
                if (!data && active) {
                    set_deactive();
                }
            });

            dr_cmd_rec_start.on_update([this](const bool& data) {
                if (data && active && !on_rec) {          
                    auto t = std::time(nullptr);
                    auto tm = *std::localtime(&t);
                    std::ostringstream oss;
                    oss << std::put_time(&tm, "crane_%Y-%m-%d_%H%M%S");
                    oss << ".mp4";

                    // --- GStreamer 파이프라인 생성 ---
                    std::ostringstream gst_pipeline;
                     gst_pipeline << "appsrc ! "
                     << "videoconvert ! "
                     << "video/x-raw, format=I420, width=" << video_width
                     << ", height=" << video_height << ", framerate=" << (int)video_fps << "/1 ! "
                     << "videoconvert ! "
                     << "nvvidconv ! "
                     << "video/x-raw(memory:NVMM), format=NV12 ! "
                     << "nvv4l2h264enc bitrate=8000000 ! "
                     << "h264parse ! "
                     << "qtmux ! "
                     << "filesink location=" << oss.str() << " ";

                    // gst_pipeline << "appsrc ! "
                    // // 1. appsrc는 BGR, 2048x1536을 받는다고 선언
                    // << "video/x-raw, format=BGR, width=" << video_width
                    // << ", height=" << video_height << ", framerate=" << (int)video_fps << "/1 ! "
                    // // 2. [핵심] videoconvert가 이 데이터를 받아 GStreamer의 표준 메모리 레이아웃으로 변환
                    // << "videoconvert ! "
                    // // 3. 변환된 데이터를 nvvidconv가 받아 하드웨어 메모리로 다시 변환
                    // << "nvvidconv ! "
                    // << "video/x-raw(memory:NVMM), format=NV12 ! "
                    // << "nvv4l2h264enc bitrate=8000000 ! "
                    // << "h264parse ! "
                    // << "qtmux ! "
                    // << "filesink location=" << oss.str() << " ";

                    if (writer.open(gst_pipeline.str(), cv::CAP_GSTREAMER, 0, video_fps, cv::Size(video_width, video_height), true)) {
                        on_rec = true;
                        getLogger()->info("[{}] video record start", getName());
                        // std::cout << "video record start" << std::endl;
                    }
                    // writer.open(oss.str(), cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), video_fps, cv::Size(video_width, video_height), true);
                }
            });

            dr_cmd_rec_stop.on_update([this](const bool& data) {
                if (data && on_rec) {
                    on_rec = false;
                    getLogger()->info("[{}] video record stop", getName());
                    // std::cout << "video record stop" << std::endl;
                }
            });

            dr_cmd_setup.on_update([&](const double& data) {
                getLogger()->info("[{}] cmd_setup={:.02f}", getName(), data);
                // printf("vision: data=%lf\n", data);
                if (data < 0 && !active) {
                    set_active();
                }
                if (data > 0 && active) {
                    set_deactive();
                }
            });

            dr_target_robot.on_update([this](const manif::SE3d& data) {
                target_robot = data;
            });

            dr_target_cur.on_update([this](const manif::SE3d& data) {
                target_cur = data;
            });

            dr_target_ref.on_update([this](const manif::SE3d& data) {
                target_ref = data;
            });
            
            switch (state) {
                case SM_State::CONF:        SM_CONF();      break;
                case SM_State::INACTIVE:    SM_INACTIVE();  break;
                case SM_State::ACTIVE:      SM_ACTIVE();    break;
                case SM_State::UNCONF:      SM_UNCONF();    break;
            }

            dw_state.write(active);
        }
    private:
        rt::DataReader<bool> dr_cmd_state{"cmd_cv_state", DependencyType::Weak};
        rt::DataReader<bool> dr_cmd_rec_start{"js/btn_10", DependencyType::Weak};
        rt::DataReader<bool> dr_cmd_rec_stop{"js/btn_11", DependencyType::Weak};
        rt::DataReader<double> dr_cmd_setup{"js/axis_7", DependencyType::Weak};
        rt::DataReader<manif::SE3d> dr_target_cur{"mt/target", DependencyType::Weak};
        rt::DataReader<manif::SE3d> dr_target_ref{"mt/reference", DependencyType::Weak};
        rt::DataReader<manif::SE3d> dr_target_robot{"pp_lpf/target", DependencyType::Weak};
        rt::DataWriter<bool> dw_state{"cv_state", ArchiveOption::Enable};
        rt::DataWriter<bool> dw_detect{"marker_detection", ArchiveOption::Enable};
        rt::DataWriter<custom_types::CvResultData> dw_data{"cv_data", ArchiveOption::Enable};

    private:
        
        void set_active() {
            if (active) return;
            getLogger()->info("[{}] state=on", getName());
            // std::cout << "vision state=on" << std::endl;
            active = true;
            reset_result();
        }

        void set_deactive() {
            if (!active) return;
            getLogger()->info("[{}] state=off", getName());
            // std::cout << "vision state=off" << std::endl;
            active = false;
            on_rec = false;
            cv::destroyAllWindows();
        }

        bool get_state() {
            return active;
        }

        bool is_outdate(std::chrono::steady_clock::time_point last) {
            return last < result.stamp;
        }

        void reset_result() {
            last_frame = -1;

            for (auto& marker : result.marker) {
                marker.quat(Eigen::Quaterniond::Identity());
                marker.translation(Eigen::Vector3d::Zero());
            }
            for (auto& detect : result.detect)
                detect = false;
            result.stamp = std::chrono::steady_clock::now();
            result.frame_id = -1;
            result.fps = -1;

            Eigen::Vector3d def_trs(0, 0, 1);

            target_cur.quat(Eigen::Quaterniond::Identity());
            target_cur.translation(def_trs);

            target_robot.quat(Eigen::Quaterniond::Identity());
            target_robot.translation(def_trs);

            target_ref.quat(Eigen::Quaterniond::Identity());
            target_ref.translation(def_trs);
        }

        void aruco_start() {
            evt[0].outdate = false;
            evt[1].outdate = false;

            const int roi_padding = 50;

            aruco_run = true;
            auto aruco_task = [this](int cam_id) {
                cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
                clahe->setClipLimit(2.0);
                clahe->setTilesGridSize(cv::Size(8, 8));

                cv::Mat objPoints = cv::Mat(4, 1, CV_32FC3);
                {
                    float markerLength;
                    switch (cam_id) {
                        case 0: markerLength = 80.0f; break;
                        case 1: markerLength = 100.0f; break;
                    }
                    objPoints.ptr<cv::Vec3f>(0)[0] = cv::Vec3f(-markerLength/2.f, markerLength/2.f, 0);
                    objPoints.ptr<cv::Vec3f>(0)[1] = cv::Vec3f(markerLength/2.f, markerLength/2.f, 0);
                    objPoints.ptr<cv::Vec3f>(0)[2] = cv::Vec3f(markerLength/2.f, -markerLength/2.f, 0);
                    objPoints.ptr<cv::Vec3f>(0)[3] = cv::Vec3f(-markerLength/2.f, -markerLength/2.f, 0);
                }
            #if CV_VERSION_MAJOR > 4 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR > 6)
                cv::aruco::DetectorParameters detectorParams = cv::aruco::DetectorParameters();
                cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
                cv::aruco::ArucoDetector detector(dictionary, detectorParams);
            #else
                cv::Ptr<cv::aruco::DetectorParameters> detectorParams = cv::aruco::DetectorParameters::create();
                cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
            #endif

                auto& roi_rect = _roi_rect[cam_id];
                roi_rect = cv::Rect(img_width/4, img_height/4, img_width/2, img_height/2);
                while(aruco_run) {
                    std::unique_lock<std::mutex> lock(evt[cam_id].mtx);
                    evt[cam_id].cv_request.wait(lock, [&]{return !aruco_run || evt[cam_id].outdate;});
                    if (!aruco_run) break;

                    std::vector<int> ids;
                    std::vector<std::vector<cv::Point2f>> corners, rejected;

                    
                    cv::Mat idealK = (cv::Mat1d(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
                    cv::Mat idealD = (cv::Mat1d(1, 4) << 0, 0, 0, 0);

                    int xmin, xmax, ymin, ymax;

                    {
                        auto roi_image = image[cam_id](roi_rect);

                    #if CV_VERSION_MAJOR > 4 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR > 6)

                    #else
                        clahe->apply(roi_image, roi_image);
                    #endif

                        xmin = img_width; xmax = 0;
                        ymin = img_height; ymax = 0;

                    #if CV_VERSION_MAJOR > 4 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR > 6)
                        detector.detectMarkers(roi_image, corners, ids, rejected);
                    #else
                        cv::aruco::detectMarkers(roi_image, dictionary, corners, ids, detectorParams, rejected);
                    #endif

                        auto nMarkers = corners.size();
                        if(!ids.empty()) {
                            // 각 마커에 대해 자세를 계산합니다.
                            for (size_t i = 0; i < nMarkers; i++) {
                                int idx = -1;
                                switch (ids[i]) {
                                    case 1: if (cam_id == 1) idx = 0; break;
                                    case 2: if (cam_id == 0) idx = 1; break;
                                    case 3: if (cam_id == 0) idx = 2; break;
                                }
                                if (idx < 0 || idx >= result.detect.size()) continue;
                                
                                for (auto& corner: corners[i]) {
                                    corner.x += roi_rect.x;
                                    corner.y += roi_rect.y;
                                    xmin = std::min(xmin, (int)corner.x);
                                    xmax = std::max(xmax, (int)corner.x);
                                    ymin = std::min(ymin, (int)corner.y);
                                    ymax = std::max(ymax, (int)corner.y);
                                }
                                std::vector<cv::Point2f> undist;
                                cv::fisheye::undistortPoints(corners[i], undist, camMatrix[cam_id], distCoeffs[cam_id]);

                                if (cv::solvePnP(objPoints, undist, idealK, idealD, rvec[idx], tvec[idx])) {
                                    result.detect[idx] = true;
                                } else {
                                    continue;
                                }
                            }
                        }
                    }

                    // ... (ROI 업데이트 로직은 기존과 동일)
                    switch (cam_id) {
                        case 0: {
                            if (!result.detect[1] || !result.detect[2]) {
                                roi_rect = cv::Rect(img_width/4, img_height/4, img_width/2, img_height/2);
                            } else {
                                roi_rect.x = std::max(0, xmin - roi_padding);
                                roi_rect.y = std::max(0, ymin - roi_padding);
                                roi_rect.width = std::min(img_width - roi_rect.x, xmax - xmin + 2*roi_padding);
                                roi_rect.height = std::min(img_height - roi_rect.y, ymax - ymin + 2*roi_padding);
                            }
                        }
                        break;
                        case 1: {
                            if (!result.detect[0]) {
                                roi_rect = cv::Rect(img_width/4, img_height/4, img_width/2, img_height/2);
                            } else {
                                roi_rect.x = std::max(0, xmin - roi_padding);
                                roi_rect.y = std::max(0, ymin - roi_padding);
                                roi_rect.width = std::min(img_width - roi_rect.x, xmax - xmin + 2*roi_padding);
                                roi_rect.height = std::min(img_height - roi_rect.y, ymax - ymin + 2*roi_padding);
                            }
                        }
                        break;
                    }
                    
                    cv::cvtColor(image[cam_id], image[cam_id], cv::COLOR_GRAY2BGR);
                    cv::rectangle(image[cam_id], roi_rect, cv::Scalar(255, 0, 0), 1);
                    cv::aruco::drawDetectedMarkers(image[cam_id], corners, ids);

                    evt[cam_id].outdate = false;
                    lock.unlock();
                    evt[cam_id].cv_complete.notify_one();
                }
            };

            std::thread(aruco_task, 0).detach();
            std::thread(aruco_task, 1).detach();
        }

        void aruco_stop() {
            aruco_run = false;

            // event based flow control 
            {
                for (auto cam_id=0; cam_id<2; cam_id++) {
                    std::unique_lock<std::mutex> lock(evt[cam_id].mtx);
                    evt[cam_id].outdate = true;                
                }
                for (auto cam_id=0; cam_id<2; cam_id++) {
                    evt[cam_id].cv_request.notify_one();
                }
            }
        }

        void SM_CONF() {
            {
                system = Spinnaker::System::GetInstance();
                camList = system->GetCameras();
                unsigned int numCameras = camList.GetSize();

                if (numCameras < 2) {
                    getLogger()->error("[{}] camera not available({})", getName(), numCameras);
                    PERIODIC_CALL(
                        getLogger()->info("[{}] wait camera...({})", getName(), numCameras);
                        // printf("wait camera...(%d)\n", numCameras);
                    , 3s);
                    camList.Clear();
                    system->ReleaseInstance();
                    system = nullptr;
                    return;
                }
                
                getLogger()->info("[{}] get camera succeed({})", getName(), numCameras);
                // printf("get camera succeed(%d)\n", numCameras);

                for (auto i=0; i<2; i++) {
                    pCam[i] = camList.GetByIndex(i);
                    // Spinnaker::GenApi::INodeMap& nodeMapTLDevice = pCam[i]->GetTLDeviceNodeMap();
                    pCam[i]->Init();

                    pCam[i]->PixelFormat.SetValue(Spinnaker::PixelFormat_Mono8);
                    pCam[i]->DecimationVertical.SetValue(1);
                    // pCam[i]->AcquisoitionFrameRate.SetValue(pCam[i]->AcquisitionFrameRate.GetMax());
                    // pCam[i]->AcquisitionFrameRate.SetValue(50);
                    pCam[i]->AcquisitionFrameRate.SetValue(25);
                    pCam[i]->ReverseX.SetValue(inv);
                    pCam[i]->ReverseY.SetValue(inv);
                    pCam[i]->ExposureAuto.SetValue(Spinnaker::ExposureAuto_Off);
                    pCam[i]->ExposureTime.SetValue(15000);
                    // pCam[i]->ExposureTime.SetValue(10000);
                    // pCam[i]->BalanceWhiteAuto.SetValue(Spinnaker::BalanceWhiteAuto_Continuous);
                    Spinnaker::GenApi::INodeMap & sNodeMap = pCam[i]->GetTLStreamNodeMap();
                    Spinnaker::GenApi::CIntegerPtr streamBufferCountManual = sNodeMap.GetNode("StreamBufferCountManual");
                    streamBufferCountManual->SetValue(2);
                    getLogger()->info("[{}] Camera {}", getName(), i);
                    getLogger()->info("[{}]   Resolution={}x{}", getName(), pCam[i]->Width.GetValue(), pCam[i]->Height.GetValue());
                    getLogger()->info("[{}]   FPS={}", getName(), pCam[i]->AcquisitionFrameRate.GetValue());

                    // std::cout << "Camera " << i << std::endl;
                    // std::cout << "  Resolution=" << pCam[i]->Width.GetValue() << "x" << pCam[i]->Height.GetValue() << std::endl;
                    // std::cout << "  FPS=" << pCam[i]->AcquisitionFrameRate.GetValue() << std::endl;


                }
            }
                // // fisheye
            int width = pCam[0]->Width.GetValue();
            int height = pCam[0]->Height.GetValue();
            double fps = pCam[0]->AcquisitionFrameRate.GetValue();
            double fx = 440.67405361535384;
            double fy = 440.96804417077976;
            double cx = 1019.4932684278764;
            double cy = 782.4038315850923;
            double k1 = -0.010365497099125128;
            double k2 = 0.002987804198049699;
            double k3 = -0.0038762412758009696;
            double k4 = 0.000547737791230329;

            {
                std::ifstream cam_data("stereo.json");
                if (cam_data.is_open()) {
                    auto data = nlohmann::json::parse(cam_data);
                    getLogger()->info("[{}] camera info loaded", getName());
                    // std::cout << "vision: camera info loaded" << std::endl;
                    for (auto idx=0; idx<2; idx++) {
                        std::ostringstream oss;
                        oss << "stereo_" << idx;
                        auto& cam_info = data[oss.str()];
                        fx = cam_info["fx"];
                        cx = cam_info["cx"];
                        fy = cam_info["fy"];
                        cy = cam_info["cy"];

                        k1 = cam_info["k1"];
                        k2 = cam_info["k2"];
                        k3 = cam_info["k3"];
                        k4 = cam_info["k4"];


                        if (width != data["width"]) {
                            getLogger()->warn("[{}] image width not matched with camera info", getName());
                            // std::cout << "vision: Warning! image width not matched with camera info" << std::endl;
                            double w = data["width"];
                            double factor = width / w;
                            fx *= factor;
                            cx *= factor;
                        }
                        if (height != data["height"]) {
                            getLogger()->warn("[{}] image height not matched with camera info", getName());
                            // std::cout << "vision: Warning! image height not matched with camera info" << std::endl;
                            double h = data["height"];
                            double factor = height / h;
                            fy *= factor;
                            cy *= factor;
                        }

                        camMatrix[idx] = (cv::Mat1d(3, 3) << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0);
                        distCoeffs[idx] = (cv::Mat1d(1, 4) << k1, k2, k3, k4);
                    }
                    Eigen::Matrix3d R;
                    R(0, 0) = data["R_xx"];
                    R(0, 1) = data["R_xy"];
                    R(0, 2) = data["R_xz"];
                    R(1, 0) = data["R_yx"];
                    R(1, 1) = data["R_yy"];
                    R(1, 2) = data["R_yz"];
                    R(2, 0) = data["R_zx"];
                    R(2, 1) = data["R_zy"];
                    R(2, 2) = data["R_zz"];
                    Rt.quat(Eigen::Quaterniond(R));
                    Rt.translation(Eigen::Vector3d(data["T_x"], data["T_y"], data["T_z"]));
                    
                } else {
                    getLogger()->warn("[{}] cam data not available", getName());
                    // printf("[Warning] cam data not available\n");
                    for (auto idx=0; idx<2; idx++) {
                        camMatrix[idx] = (cv::Mat1d(3, 3) << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0);
                        distCoeffs[idx] = (cv::Mat1d(1, 4) << k1, k2, k3, k4);
                        Rt.quat(Eigen::Quaterniond::Identity());
                        Rt.translation(Eigen::Vector3d(-34.58983128587395, 668.5643462042916, 104.4357162177983));
                    }
                }
                Rt.quat(Eigen::Quaterniond::Identity());
                Rt.translation(Rt.translation()*1e-3);
            }

            cv::Mat E = cv::Mat::eye(3, 3, cv::DataType<double>::type);  

            getLogger()->info("[{}] width={}, height={}, fps={:.0f}", getName(), width, height, fps);
            // printf("width=%d, height=%d, fps=%.0lf\n", width, height, fps);
            video_fps = fps;
            video_width = width;
            video_height = height;
            img_width = width;
            img_height = height;

            // float markerLength = 100;	// marker size(Unit: mm)
            aruco_start();
            
        
            double totalTime = 0;
            auto totalIterations = 0;
            auto cal_fps = fps;
            auto cnt = 0;
            auto last = std::chrono::steady_clock::now();
            
            result.fps = fps;


            state = SM_State::INACTIVE;
        }

        void SM_ACTIVE() {
            if (!active) {
                pCam[0]->EndAcquisition();
                pCam[1]->EndAcquisition();
                state = SM_State::INACTIVE;
            }


            Spinnaker::ImagePtr pResultImage[2];

            try {
                for (auto i=0; i<2; i++)
                    pResultImage[i] = pCam[i]->GetNextImage(1);
            } catch (...) {
                return;
            }
            
            for (auto i=0; i<2; i++)
            {
                cv::Mat img(img_height, img_width, CV_8UC1, pResultImage[i]->GetData());
                img.copyTo(image[i]);
                // image[i] = img;


                // cv::equalizeHist(img, image);
                // cv::cvtColor(img, image, cv::COLOR_BayerBG2BGR);
                // cv::cvtColor(img, image, cv::COLOR_BayerBG2GRAY);
            }

            auto tick = std::chrono::steady_clock::now();
            result.stamp = tick;
            result.fps = video_fps;
            for (auto& detect: result.detect)
                detect = false;


            // event based flow control 
            {
                for (auto cam_id=0; cam_id<2; cam_id++) {
                    std::unique_lock<std::mutex> lock(evt[cam_id].mtx);
                    evt[cam_id].outdate = true;                
                }
                for (auto cam_id=0; cam_id<2; cam_id++) {
                    evt[cam_id].cv_request.notify_one();
                    // std::cout << "aruco task " << cam_id << " work calling" << std::endl;
                }
                for (auto cam_id=0; cam_id<2; cam_id++) {
                    std::unique_lock<std::mutex> lock(evt[cam_id].mtx);
                    evt[cam_id].cv_complete.wait(lock, [&]{return !evt[cam_id].outdate;});
                }
            }


            for (auto i=0; i<result.detect.size(); i++) {
                if (!result.detect[i])
                    continue;

                Eigen::Vector3d position;
                position << tvec[i][0], tvec[i][1], tvec[i][2];
                position = position * 1e-3;
        
                auto angle = sqrt(rvec[i].dot(rvec[i]));
                Eigen::Vector3d axis;
                axis << rvec[i][0], rvec[i][1], rvec[i][2];
                axis = axis/angle;
                Eigen::AngleAxisd aa(angle, axis);
        
                result.marker[i] = manif::SE3d(position, Eigen::Quaterniond(aa));
            }
            if (result.detect[0]) {
                result.marker[0] = Rt.inverse() * result.marker[0];
            }

            dw_data.write(result);
            dw_detect.write(result.detect[0] && result.detect[1] && result.detect[2]);
            
            {
                cv::Vec3d m_rvec, m_tvec;
                Eigen::AngleAxisd aa(target_robot.quat());
                Eigen::Vector3d _rvec = aa.angle() * aa.axis();

                m_rvec[0] = _rvec.x();
                m_rvec[1] = _rvec.y();
                m_rvec[2] = _rvec.z();

                m_tvec[0] = target_robot.translation().x()*1e3;
                m_tvec[1] = target_robot.translation().y()*1e3;
                m_tvec[2] = target_robot.translation().z()*1e3;
                                                    
                cv::drawFrameAxes(image[0], camMatrix[0], distCoeffs[0], m_rvec, m_tvec, 100);
            }

            {
                cv::Vec3d m_rvec, m_tvec;
                Eigen::AngleAxisd aa(target_cur.quat());
                Eigen::Vector3d _rvec = aa.angle() * aa.axis();

                m_rvec[0] = _rvec.x();
                m_rvec[1] = _rvec.y();
                m_rvec[2] = _rvec.z();

                m_tvec[0] = target_cur.translation().x()*1e3;
                m_tvec[1] = target_cur.translation().y()*1e3;
                m_tvec[2] = target_cur.translation().z()*1e3;
                                                    
                cv::drawFrameAxes(image[0], camMatrix[0], distCoeffs[0], m_rvec, m_tvec, 80);
            }

            {
                cv::Vec3d m_rvec, m_tvec;
                Eigen::AngleAxisd aa(target_ref.quat());
                Eigen::Vector3d _rvec = aa.angle() * aa.axis();

                m_rvec[0] = _rvec.x();
                m_rvec[1] = _rvec.y();
                m_rvec[2] = _rvec.z();

                m_tvec[0] = target_ref.translation().x()*1e3;
                m_tvec[1] = target_ref.translation().y()*1e3;
                m_tvec[2] = target_ref.translation().z()*1e3;
                                                    
                cv::drawFrameAxes(image[0], camMatrix[0], distCoeffs[0], m_rvec, m_tvec, 60);
            }

            cv::Rect rect(img_width/4, 0, img_width/2, img_height);
            cv::Mat left = image[0](rect);
            cv::Mat right = image[1](rect);
            cv::Mat res;
            cv::hconcat(left, right, res);

            if (on_rec) {
                // clone()으로 이미지 데이터를 복사하여 다른 스레드로 안전하게 전달
                auto img_ptr = std::make_shared<cv::Mat>(res.clone());
                // push가 실패하면 (큐가 꽉 찼으면) 해당 프레임은 그냥 버려집니다.
                if (!img_buf.push(img_ptr)) {
                    // 선택: 큐가 꽉 찼다는 경고 메시지를 출력할 수 있습니다.
                    getLogger()->warn("[{}] Recording queue is full. Dropping a frame.", getName());
                    // std::cerr << "Warning: Recording queue is full. Dropping a frame." << std::endl;
                }
            }
                
            if (true) {
                cv::resize(res, res, cv::Size(img_width/2, img_height/2));
                cv::imshow("Camera", res);
                cv::waitKey(1);
            }
            result.frame_id++;


            frame_cnt++;
            PERIODIC_CALL(
                auto now = std::chrono::steady_clock::now();
                auto currentTime = std::chrono::duration_cast<std::chrono::milliseconds>(now - tick).count();
                getLogger()->info("[{}] fps={}, detection_time={} ms, detection={} {} {}", getName(), 
                    frame_cnt, currentTime, result.detect[0], result.detect[1], result.detect[2]);
                // printf("fps=%d, detection_time=%ld ms, detection=%d %d %d\n", frame_cnt, currentTime, result.detect[0], result.detect[1], result.detect[2]);
                frame_cnt = 0;
            , 1s);

            pResultImage[0]->Release();
            pResultImage[1]->Release();

        }

        void SM_INACTIVE() {
            if (active) {
                pCam[0]->BeginAcquisition();
                pCam[1]->BeginAcquisition();
                // 상태전이
                state = SM_State::ACTIVE;
            }
        }

        void SM_UNCONF() {
            aruco_stop();
            if (system) {
                for (auto i=0; i<2; i++) {
                    try { pCam[i]->EndAcquisition(); } catch (...) {}
                    try { pCam[i]->DeInit(); } catch (...) {}
                    pCam[i] = nullptr;
                }
                camList.Clear();            
                system->ReleaseInstance();
                system = nullptr;
            }
            // try{
            //     pCam[0]->DeInit();
            //     pCam[1]->DeInit();
            // } catch (Spinnaker::Exception e) {
            //     getLogger()->error("[{}] spinnaker DeInit failed", getName());
            //     getLogger()->error("[{}] {}", getName(), e.GetErrorMessage());

            //     // std::cout << "spinnaker DeInit failed" << std::endl;
            //     // std::cout << e.GetErrorMessage() << std::endl;
            // }
            // printf("vision: thread finished\n");

            state = SM_State::IDLE;
        }

        void task2() {
            while (run) {
                std::shared_ptr<cv::Mat> img_ptr;
                
                // 큐에서 이미지를 가져옵니다.
                if (img_buf.pop(img_ptr)) {
                    // writer가 열려있을 때만 파일에 씁니다.
                    // on_rec 플래그는 writer 열림/닫힘을 제어하는 용도로만 사용합니다.
                    if (writer.isOpened()) {
                        writer << *img_ptr;
                    }
                } else {
                    // 큐가 비어있으면 CPU 낭비를 막기 위해 잠시 대기합니다.
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                }
                
                // 녹화 중지 명령을 받았고, writer가 열려있으며, 큐가 비었다면 안전하게 닫습니다.
                if (!on_rec && writer.isOpened() && img_buf.empty()) {
                    writer.release();
                    getLogger()->info("[{}] Video file closed.", getName());
                    // std::cout << "Video file closed." << std::endl;
                }
            }

            // 스레드 종료 전, writer가 여전히 열려있다면 마지막으로 남은 프레임을 비우고 닫습니다.
            if (writer.isOpened()) {
                std::shared_ptr<cv::Mat> img_ptr;
                while(img_buf.pop(img_ptr)) {
                    writer << *img_ptr;
                    getLogger()->info("[{}] {} image remained", getName(), img_buf.read_available());
                    // std::cout << img_buf.read_available() << " image remained" << std::endl;
                }
                writer.release();
                getLogger()->info("[{}] Finalizing video file.", getName());
                // std::cout << "Finalizing video file." << std::endl;
            }
            // getLogger()->info("[{}] recording thread finished", getName());
            // printf("vision: recording thread finished\n");
        }
    private:

        SM_State state = SM_State::CONF;
        
        uint64_t last_frame = 0;
        bool active = false;
        bool gui = false;
        bool inv = false;
        bool run = true;
        bool aruco_run;
        
        int img_width;
        int img_height;
        int fsize;
        bool on_rec = false;
        cv::VideoWriter writer;
        int video_fps;
        int video_width;
        int video_height;
        int frame_cnt = 0;
        // SafeQueue<std::shared_ptr<cv::Mat>> img_buf;
        boost::lockfree::spsc_queue<std::shared_ptr<cv::Mat>, boost::lockfree::capacity<256>> img_buf;


        Spinnaker::SystemPtr system;
        Spinnaker::CameraList camList;
        Spinnaker::CameraPtr pCam[2];
        cv::Mat camMatrix[2];
        cv::Mat distCoeffs[2];
        cv::Mat image[2];
        manif::SE3d Rt;
        cv::Vec3d rvec[3];
        cv::Vec3d tvec[3];
        cv::Rect _roi_rect[2];

        struct {
            bool outdate;
            std::condition_variable cv_request;
            std::condition_variable cv_complete;
            std::mutex mtx;
        } evt[2];

        custom_types::CvResultData result;
        manif::SE3d target_robot;
        manif::SE3d target_cur;
        manif::SE3d target_ref;
    };
};