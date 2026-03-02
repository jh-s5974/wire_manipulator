#pragma once
#include <cinttypes>

namespace interface {
    struct IMU {
        uint64_t timeStartup;
        double yawPitchRoll[3];
        double quaternion[4];
        double angularRate[3];
        double acceleration[3];
    };
};