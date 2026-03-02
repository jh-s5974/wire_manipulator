#pragma once
#include <cinttypes>

namespace interface {
    struct FT {
        int can_tx_id;
        int can_rx_id1;
        int can_rx_id2;
        float Fx;
        float Fy;
        float Fz;
        float Tx;
        float Ty;
        float Tz;
    };
};