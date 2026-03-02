#pragma once

#include <eigen3/Eigen/Dense>

namespace custom_types {
    struct ImuData {
        Eigen::Quaterniond orient;
        Eigen::Vector3d gyro;
        Eigen::Vector3d accel;
    };
};