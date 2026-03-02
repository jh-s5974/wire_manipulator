#pragma once
#include <cinttypes>
#include "eigen3/Eigen/Dense"

namespace interface {
    struct Pose {
        Eigen::Vector3d position;
        Eigen::Quaterniond orient;
    };
};