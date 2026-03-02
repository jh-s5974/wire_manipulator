#pragma once
#include <cinttypes>

namespace interface {
    struct MOTOR {
        int can_id;
        int max_speed;
        int32_t zero_point;
        int16_t pos;
        float pre_pos;
        int8_t temp;
        int16_t cur;
        int16_t vel;
        float max_torque;
        float tc;
    };
};