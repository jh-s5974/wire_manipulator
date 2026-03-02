#pragma once

#include <opencv2/opencv.hpp>

namespace framework {
    namespace hw {
        class Camera {
        public:
            struct _intrinsic {
                const double fx;
                const double fy;
                const double cx;
                const double cy;
            } intrinsic;

            struct _distortion {
                const double k1;
                const double k2;
                const double k3;
                const double k4;
                const double p1;
                const double p2;
            } distortion;

            virtual Camera(int fps) = 0;
            virtual int get_fps() = 0;
            virtual std::shared_ptr<cv::Mat> get_image() = 0;
        };
    };
};
