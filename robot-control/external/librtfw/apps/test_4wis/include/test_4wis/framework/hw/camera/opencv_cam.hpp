#pragma once

#include <opencv2/opencv.hpp>
#include "cam.hpp"


namespace framework {
    namespace hw {
        class Camera_opencv : public Camera {
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