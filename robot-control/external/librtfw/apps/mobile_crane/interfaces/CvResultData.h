#pragma once

#include <chrono>
#include <array>
#include <manif/manif.h>


namespace custom_types {
    struct CvResultData {
        int frame_id;
        double fps;
        std::chrono::steady_clock::time_point stamp;
        std::array<bool, 3> detect;
        std::array<manif::SE3d, 3> marker;
    };
}
