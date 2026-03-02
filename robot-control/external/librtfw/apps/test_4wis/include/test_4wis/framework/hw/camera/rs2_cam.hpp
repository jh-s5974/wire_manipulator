#pragma once

#pragma once


#include <librealsense2/rs.h>
#include <librealsense2/h/rs_pipeline.h>
#include <librealsense2/h/rs_frame.h>
#include <opencv2/opencv.hpp>
#include "cam.hpp"


namespace framework {
    namespace hw {
        class Camera_rs2 : public Camera {
        public:
            virtual Camera(int fps) {

            };
            virtual int get_fps() {

            }
            virtual std::shared_ptr<cv::Mat> get_image() {
                
            }
        };
    };
};