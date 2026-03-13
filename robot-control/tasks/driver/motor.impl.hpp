#pragma once
#include <cstdint>
#include <cstring>
#include <unistd.h>
#include <linux/can.h>
#include <arpa/inet.h> // ntohs 사용을 위해 추가
#include <cmath>



// --- 공통 데이터 구조체 ---
struct MotorCommand {
    double pos = 0.0, vel = 0.0, torque = 0.0, kp = 0.0, kd = 0.0;
};

struct MotorState {
    double pos = 0.0, vel = 0.0, torque = 0.0;
    int status = 0;
    bool online = false;
};

// --- 유틸리티 함수 ---
uint16_t float_to_uint(float x, float x_min, float x_max, int bits) {
    float val = x > x_max? x_max: x < x_min? x_min: x;
    float span = x_max - x_min;
    return (uint16_t)((val - x_min) * ((float)((1 << bits) - 1)) / span);
}

float uint_to_float(int x_int, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + x_min;
}

// ==========================================================
// [Motor 추상 인터페이스]
// ==========================================================
class Motor {
public:
    uint32_t id;
    MotorState state;
    virtual ~Motor() {}
    
    virtual bool packStart(struct can_frame* frame) = 0;
    virtual bool packControl(struct can_frame* frame, const MotorCommand& cmd) = 0;
    virtual bool packStop(struct can_frame* frame) = 0;
    virtual bool parseFeedback(const struct can_frame* frame) = 0;
    virtual bool isMyFrame(const struct can_frame* frame) = 0;
    void Start(int so, bool dummy=false) {
        struct can_frame frame;
        if (packStart(&frame))
            auto res = write(so, &frame, sizeof(frame));
        if (dummy) {
            MotorCommand cmd;
            memset(&cmd, 0, sizeof(MotorCommand));
            Control(so, cmd);
        }
    }
    void Control(int so, const MotorCommand& cmd) {
        struct can_frame frame;
        if (packControl(&frame, cmd))
            auto res = write(so, &frame, sizeof(frame));
    }
    void Stop(int so) {
        struct can_frame frame;
        if (packStop(&frame))
            auto res = write(so, &frame, sizeof(frame));
    }
};

// ==========================================================
// [RobStrideMotor 구현체]
// ==========================================================
class RobStride03 : public Motor {
private:
    struct __attribute__((packed)) motor_fault {
        uint8_t uncalibrated : 1;
        uint8_t overload     : 1;
        uint8_t magnet_fault : 1;
        uint8_t overtemp     : 1;
        uint8_t overcurrent  : 1;
        uint8_t undervoltage : 1;
        uint8_t mode         : 2;
    };

    // --- 1. CAN ID Header Union ---
    union RobStrideHeader {
        uint32_t raw;
        
        struct { uint32_t reserved: 24; uint32_t typecode: 5; uint32_t unused: 3; } common;
        
        struct { uint32_t target_id: 8; uint32_t host_id: 16; uint32_t typecode: 5; uint32_t unused: 3; } type0_tx;
        struct { uint32_t reserved: 8;  uint32_t target_id: 16; uint32_t typecode: 5; uint32_t unused: 3; } type0_rx;
        struct { uint32_t target_id: 8; uint32_t torque: 16;    uint32_t typecode: 5; uint32_t unused: 3; } type1;
        struct { uint32_t host_id: 8;   uint32_t target_id: 8;  struct motor_fault fault; uint32_t typecode: 5; uint32_t unused: 3; } type2;
        struct { uint32_t target_id: 8; uint32_t main_id: 8;    uint32_t reserved: 8; uint32_t typecode: 5; uint32_t unused: 3; } type3;
        struct { uint32_t target_id: 8; uint32_t main_id: 8;    uint32_t reserved: 8; uint32_t typecode: 5; uint32_t unused: 3; } type4;
        struct { uint32_t target_id: 8; uint32_t main_id: 8;    uint32_t reserved: 8; uint32_t typecode: 5; uint32_t unused: 3; } type6;
        struct { uint32_t target_id: 8; uint32_t main_id: 8;    uint32_t can_id: 8;   uint32_t typecode: 5; uint32_t unused: 3; } type7;
        struct { uint32_t target_id: 8; uint32_t main_id: 16;   uint32_t typecode: 5; uint32_t unused: 3; } type17_tx;
        struct { uint32_t target_id: 8; uint32_t main_id: 8;    uint32_t result: 8;   uint32_t typecode: 5; uint32_t unused: 3; } type17_rx;
        struct { uint32_t target_id: 8; uint32_t main_id: 16;   uint32_t typecode: 5; uint32_t unused: 3; } type18;
        struct { uint32_t target_id: 8; uint32_t host_id: 16;   uint32_t typecode: 5; uint32_t unused: 3; } type22;
    };

    // --- 2. CAN Data Union ---
    union RobStrideData {
        uint8_t raw[8];
        uint64_t raw64;

        struct { uint8_t  unused[8]; } type0_tx;
        struct { union { uint64_t value; uint8_t bytes[8]; } mcu_id; } type0_rx;
        struct { uint16_t pos; uint16_t vel; uint16_t kp; uint16_t kd; } type1;
        struct { uint16_t pos; uint16_t vel; uint16_t torque; uint16_t temp; } type2;
        struct { uint64_t unused; } type3;
        struct { uint8_t  fault_clear; uint64_t _unused: 56; } type4;
        struct { uint8_t  set_zero;    uint64_t _unused: 56; } type6;
        struct { uint16_t index; uint64_t unused: 48; } type17_tx;
        struct { uint16_t index; uint16_t unused; union { float f32; uint8_t u8; uint16_t u16; uint32_t u32; } param; } type17_rx;
        struct { uint16_t index; uint16_t unused; union { float f32; uint8_t u8; uint16_t u16; uint32_t u32; } param; } type18;
        struct { uint64_t unused; } type7;
    };
public:
    RobStride03(uint32_t _id) { id = _id; }

    bool packStart(struct can_frame* frame) override {
        memset(frame, 0, sizeof(struct can_frame));
        RobStrideHeader h = {0};
        h.type3.typecode = 0x03;
        h.type3.main_id = 0xFD;
        h.type3.target_id = id;

        frame->can_id = h.raw | CAN_EFF_FLAG;
        frame->can_dlc = 8;
        memset(frame->data, 0, 8);
        return true;
    }

    bool packControl(struct can_frame* frame, const MotorCommand& cmd) override {
        memset(frame, 0, sizeof(struct can_frame));
        RobStrideHeader h = {0};
        RobStrideData d = {0};

        h.raw = 0;
        h.type1.typecode = 0x01;
        h.type1.target_id = id;
        
        h.type1.torque = float_to_uint(cmd.torque, -60, 60, 16);
        d.type1.pos = htons(float_to_uint(cmd.pos, -4*M_PI, 4*M_PI, 16));
        d.type1.vel = htons(float_to_uint(cmd.vel, -20.0, 20.0, 16));
        d.type1.kp  = htons(float_to_uint(cmd.kp, 0.0, 5000.0, 16));
        d.type1.kd  = htons(float_to_uint(cmd.kd, 0.0, 100.0, 16));

        frame->can_id = h.raw | CAN_EFF_FLAG;
        frame->can_dlc = 8;
        *(uint64_t*)frame->data = d.raw64;
        return true;
    }

    bool packStop(struct can_frame* frame) override {
        const bool clear = false;

        memset(frame, 0, sizeof(struct can_frame));
        RobStrideHeader h = {0};
        RobStrideData d = {0};

        h.type4.typecode = 0x04;
        h.type4.main_id = 0xFD;
        h.type4.target_id = id;
        
        d.type4.fault_clear = clear ? 1 : 0;

        frame->can_id = h.raw | CAN_EFF_FLAG;
        frame->can_dlc = 8;
        *(uint64_t*)frame->data = d.raw64; // 8바이트 즉시 쓰기    printf("packet_motor_stop\n");
        return true;
    }

    bool parseFeedback(const struct can_frame* frame) override {        
        RobStrideHeader h = {0};
        RobStrideData d = {0};
        
        h.raw = frame->can_id & CAN_EFF_MASK;
        if (h.type2.target_id != id) return false;
        if (h.type2.typecode != 0x02) return false;

        memcpy(d.raw, frame->data, 8);

        motor_fault fault = h.type2.fault;
        
        // 데이터 Union을 통해 편리하게 접근
        state.pos = uint_to_float(htons(d.type2.pos), -4 * M_PI, 4 * M_PI, 16);
        state.vel = uint_to_float(htons(d.type2.vel), -20.0, 20.0, 16);
        state.torque = uint_to_float(htons(d.type2.torque), -60.0, 60.0, 16);
        double temp = ntohs(d.type2.temp) * 0.1;
        state.online = true;
        memcpy(&state.status, &fault, 1);

        return true;
        // printf("\r[ID=0x%02X] T: %6.3f | Pos: %6.3f | Vel: %6.3f\n", 
        //     id, state.torque, state.pos, state.vel);
    }

    bool isMyFrame(const struct can_frame* frame) override {
        if (!(frame->can_id & CAN_EFF_FLAG)) return false;
        uint32_t m_id = (frame->can_id >> 8) & 0xFF;
        return m_id == id;
    }
};

// ==========================================================
// [RmdMotor 구현체 (V3 매뉴얼 반영)]
// ==========================================================
class RmdX6P36 : public Motor {
private:
    struct __attribute__((packed)) motor_error {
        uint8_t reserved1 : 1;
        uint8_t motor_stall     : 1;
        uint8_t low_voltage : 1;
        uint8_t over_voltage     : 1;
        uint8_t over_current     : 1;
        uint8_t reserved2     : 1;
        uint8_t power_overrun  : 1;
        uint8_t calib_error : 1;
        uint8_t speeding : 1;
        uint8_t reserved3     : 1;
        uint8_t reserved4     : 1;    
        uint8_t comp_over_temp     : 1;
        uint8_t motor_over_temp     : 1;
        uint8_t enc_calib_error     : 1;
        uint8_t enc_data_error     : 1;
        uint8_t reserved5     : 1;    
    };

    // --- 2. CAN Data Union ---
    union RmdCANData {
        uint8_t raw[8];
        uint64_t raw64;

        struct { uint8_t cmd; uint8_t temp; uint8_t MOS_temp; uint8_t brake_release; uint16_t voltage_low; motor_error error_status; } motor_status_1; // 0x9A
        struct { uint8_t cmd; uint8_t temp; uint16_t current; uint16_t vel; uint16_t pos; } motor_status_2; // 0x9C
        struct { uint8_t cmd; uint8_t temp; uint16_t iA; uint16_t iB; uint16_t iC; } motor_status_3; // 0x9D
        struct { uint8_t cmd; } motor_shutdown; // 0x80, torque off
        struct { uint8_t cmd; } motor_stop; // 0x81 zero vel
        struct { uint8_t cmd; uint8_t unused[6]; uint8_t mode; } op_mode; // 0x70, read_op_mode
        struct { uint8_t cmd; } reset; // 0x76, system reset
        struct { uint8_t cmd; } brake_release; // 0x77, system brake release
        struct { uint8_t cmd; } brake_lock; // 0x78, system brake lock
        struct { uint8_t cmd; uint8_t index; uint16_t unused; union { uint8_t u8[4]; uint16_t u16[2]; uint32_t u32; } param; } func_ctrl; // 0x78, function control
        struct { uint16_t p_des; uint16_t v_des: 12; uint16_t kp: 12; uint16_t kd: 12; uint16_t t_ff: 12; } motion_tx; // 0x400+ID, motion mode control
        struct { uint8_t can_id; uint16_t p_des; uint16_t v_des: 12; uint16_t t_ff: 12; uint8_t unused[2]; } motion_rx; // 0x400+ID, motion mode control
    };
public:
    RmdX6P36(uint32_t _id) { id = _id; }

    bool packStart(struct can_frame* frame) override {
        return false;
    }

    bool packControl(struct can_frame* frame, const MotorCommand& cmd) override {
        memset(frame, 0, sizeof(struct can_frame));
        frame->can_id = 0x400 + id;
        frame->can_dlc = 8;
        
        uint16_t p_des = float_to_uint(cmd.pos, -12.5f, 12.5f, 16);
        uint16_t v_des = float_to_uint(cmd.vel, -45.0f, 45.0f, 12);
        uint16_t kp = float_to_uint(cmd.kp, 0.0f, 500.0f, 12);
        uint16_t kd = float_to_uint(cmd.kd, 0.0f, 5.0f, 12);
        uint16_t t_ff = float_to_uint(cmd.torque, -24.0f, 24.0f, 12);

        frame->data[0] = p_des >> 8;
        frame->data[1] = p_des & 0xFF;
        frame->data[2] = v_des >> 4;
        frame->data[3] = ((v_des & 0xF) << 4) | (kp >> 8);
        frame->data[4] = kp & 0xFF;
        frame->data[5] = kd >> 4;
        frame->data[6] = ((kd & 0xF) << 4) | (t_ff >> 8);
        frame->data[7] = t_ff & 0xFF;
        return true;
    }

    bool packStop(struct can_frame* frame) override {
        memset(frame, 0, sizeof(struct can_frame));
        frame->can_id = 0x140 + id; // Command Identifier
        frame->can_dlc = 8;
        frame->data[0] = 0x80; // Shutdown command
        return true;
    }

    bool parseFeedback(const struct can_frame* frame) override {    
        uint16_t p_int = (frame->data[1] << 8) | frame->data[2];
        uint16_t v_int = (frame->data[3] << 4) | (frame->data[4] >> 4);
        uint16_t t_int = ((frame->data[4] & 0xF) << 8) | frame->data[5];

        state.pos = uint_to_float(p_int, -12.5f, 12.5f, 16);
        state.vel = uint_to_float(v_int, -45.0f, 45.0f, 12);
        state.torque = uint_to_float(t_int, -24.0f, 24.0f, 12);
        state.online = true;
        return true;
    }

    bool isMyFrame(const struct can_frame* frame) override {
        if (frame->can_id & CAN_EFF_FLAG) return false;
        // RMD MIT Feedback ID는 0x500+ID 또는 0x400+ID
        // return (frame->can_id == (0x400 + id) || frame->can_id == (0x500 + id));
        return frame->can_id == (0x500 + id);
    }
};