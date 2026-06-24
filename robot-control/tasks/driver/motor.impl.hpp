#pragma once
#include <cstdint>
#include <cstring>
#include <unistd.h>
#include <linux/can.h>
#include <arpa/inet.h> // ntohs 사용을 위해 추가
#include <cmath>
#include <algorithm>



// --- 공통 데이터 구조체 ---
struct MotorCommand {
    double pos = 0.0, vel = 0.0, torque = 0.0, kp = 0.0, kd = 0.0;
    // true: 위치제어(절대위치+속도제한 명령, 멀티턴 가능) / false: MIT 토크·위치혼합 모드(±한정 범위)
    // MyActuatorX6에서만 의미가 있음 — kp>0 이면 자동으로 true (send_motor_cmd에서 설정)
    bool position_mode = false;
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
    // Control() 직후 추가로 보낼 조회 프레임이 있는 모터(예: 위치모드 정밀 각도 조회)를 위한 훅. 기본은 no-op.
    // 반환값: 보낼 프레임이 없으면(no-op) true, 보냈다면 write() 성공 여부
    virtual bool RequestExtra(int /*so*/) { return true; }
    // state.pos가 신뢰 가능한 절대위치인지 — 기본은 항상 true(RobStride 등 단일 응답으로 충분한 모터),
    // MyActuatorX6는 0x92 절대위치 응답을 받기 전까지 false (windowed MIT 값으로 잘못 동기화 방지)
    virtual bool hasValidPosition() const { return true; }
    // 현재 위치를 기계 영점으로 저장하는 프레임. 지원하지 않으면 false(no-op).
    //   RobStride03: type6(즉시 적용) / MyActuatorX6: 0x64(ROM 기록 → packReset 후 적용)
    virtual bool packSetZero(struct can_frame* /*frame*/) { return false; }
    // 영점 저장 후 적용을 위해 모터를 재시작하는 프레임. 필요 없으면 false(no-op).
    //   MyActuatorX6: 0x76(시스템 리셋). RobStride03 는 즉시 적용이라 불필요.
    virtual bool packReset(struct can_frame* /*frame*/) { return false; }
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
    // 반환값: 실제로 8바이트 전부 write() 성공했는지 — CAN TX 버퍼 과부하로 드랍되면 false
    bool Control(int so, const MotorCommand& cmd) {
        struct can_frame frame;
        if (!packControl(&frame, cmd)) return false;
        return write(so, &frame, sizeof(frame)) == (ssize_t)sizeof(frame);
    }
    void Stop(int so) {
        struct can_frame frame;
        if (packStop(&frame))
            auto res = write(so, &frame, sizeof(frame));
    }
    // 현재 위치를 영점으로 저장 — packSetZero 가 false(미지원)면 아무것도 안 하고 false 반환
    bool SetZero(int so) {
        struct can_frame frame;
        if (!packSetZero(&frame)) return false;
        return write(so, &frame, sizeof(frame)) == (ssize_t)sizeof(frame);
    }
    // 모터 재시작(영점 적용용) — packReset 이 false(불필요)면 아무것도 안 하고 false 반환
    bool Reset(int so) {
        struct can_frame frame;
        if (!packReset(&frame)) return false;
        return write(so, &frame, sizeof(frame)) == (ssize_t)sizeof(frame);
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

    // 현재 위치를 기계 영점으로 저장 (통신 타입 6) — reset/stop(비활성) 상태에서 호출, 즉시 적용됨.
    // 확장 ID 0x0600FD0X, data[0]=1. (cansend 0600FD0X#01.. 과 동일)
    bool packSetZero(struct can_frame* frame) override {
        memset(frame, 0, sizeof(struct can_frame));
        RobStrideHeader h = {0};
        RobStrideData d = {0};

        h.type6.typecode  = 0x06;
        h.type6.main_id   = 0xFD;
        h.type6.target_id = id;

        d.type6.set_zero = 1; // Byte[0]=1 : 현재 위치를 0점으로 설정

        frame->can_id = h.raw | CAN_EFF_FLAG;
        frame->can_dlc = 8;
        *(uint64_t*)frame->data = d.raw64;
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
// [MyActuator X6 구현체] — 전부 Private 프로토콜(0x140+ID TX / 0x240+ID RX), MIT 모션모드는 미사용
// X6-8 데이터시트 실측: 8:1 reduction, 정격토크 4.5 Nm, 피크토크 8 Nm, 정격전류 3.6A, 정격속도 310rpm
//
// 명령의 kp 값으로 모드를 자동 전환한다 (MotorCommand::position_mode, send_motor_cmd()에서
// kp>0이면 true로 설정):
//
//   [position_mode=true]  절대위치 폐루프 제어 (0xA4) — angleControl int32(0.01°/LSB)라
//                          멀티턴 전체 범위를 그대로 표현 가능. maxSpeed로 속도제한.
//   [position_mode=false] 토크(전류) 폐루프 제어 (0xA1) — iqControl int16(0.01A/LSB).
//                          MotorCommand::torque 값을 단위 변환 없이 그대로 전류[A]로 사용한다
//                          (Nm↔A 추정 변환은 실측 전이라 부정확하므로 사용하지 않음 — 호출부에서
//                          torque 필드에 원하는 전류[A]를 직접 넣을 것).
//
// 0xA1/0xA4 응답은 완전히 같은 구조(DATA[0]=echo, D1=temp, D2-3=iq(0.01A), D4-5=speed(1dps),
// D6-7=angle(1°/LSB)) — 같은 분기로 처리한다.
// 위치(state.pos)는 0x92 응답(Read Multi-Turn Angle, int32 0.01°/LSB)에서만 갱신한다.
// A1/A4 응답의 D6-7(1°/LSB)은 속도·전류 확인에는 충분하나 위치 정밀도가 낮아 사용하지 않는다.
// 0x92는 4모터를 라운드로빈으로 1대씩 조회(can_bus1/can_bus0에서 처리) — 동시 충돌 방지.
// ==========================================================
class MyActuatorX6 : public Motor {
public:
    MyActuatorX6(uint32_t _id) { id = _id; }

    bool packStart(struct can_frame* frame) override {
        // 활성화용 — 토크 0인 0xA1 프레임을 보냄 (전용 enable 명령이 프로토콜에 없음)
        return packControl(frame, MotorCommand{});
    }

    bool packControl(struct can_frame* frame, const MotorCommand& cmd) override {
        return cmd.position_mode ? packPositionControl(frame, cmd) : packTorqueControl(frame, cmd);
    }

    bool packStop(struct can_frame* frame) override {
        memset(frame, 0, sizeof(struct can_frame));
        frame->can_id  = 0x140 + id;
        frame->can_dlc = 8;
        frame->data[0] = 0x80; // shutdown
        return true;
    }

    // 현재 멀티턴 위치를 모터 영점으로 ROM 에 기록 (0x64). ROM 기록이라 적용은 재시작(packReset 0x76)
    // 또는 전원 재투입 후에 이뤄진다. (cansend 140+ID#64.. 과 동일)
    bool packSetZero(struct can_frame* frame) override {
        memset(frame, 0, sizeof(struct can_frame));
        frame->can_id  = 0x140 + id;
        frame->can_dlc = 8;
        frame->data[0] = 0x64; // Write current multi-turn position to ROM as motor zero
        return true;
    }

    // 시스템 리셋 (0x76) — 0x64 로 기록한 영점을 적용하기 위해 모터를 재시작한다(~1s 오프라인).
    bool packReset(struct can_frame* frame) override {
        memset(frame, 0, sizeof(struct can_frame));
        frame->can_id  = 0x140 + id;
        frame->can_dlc = 8;
        frame->data[0] = 0x76; // System reset
        return true;
    }

    bool parseFeedback(const struct can_frame* frame) override {
        if (frame->can_id != (0x240 + id)) return false;

        if (frame->data[0] == 0xA1 || frame->data[0] == 0xA4) {
            // 프로토콜 V4.4: D1=temp, D2-3=iq(0.01A/LSB), D4-5=speed(1dps/LSB), D6-7=angle(1°/LSB)
            // 위치(state.pos)는 0x92 응답에서만 갱신 — A1/A4는 속도·전류만 사용
            int16_t iq_raw    = (int16_t)(((uint16_t)frame->data[3] << 8) | frame->data[2]);
            int16_t speed_dps = (int16_t)(((uint16_t)frame->data[5] << 8) | frame->data[4]);
            state.torque = iq_raw * 0.01;
            state.vel    = speed_dps * (M_PI / 180.0);
            state.online = true;
            return true;
        }
        if (frame->data[0] == 0x92) {
            // Read Multi-Turn Angle 응답: D4-7=int32, 0.01°/LSB (멀티턴 절대 위치, 유일한 위치 소스)
            int32_t raw = (int32_t)((uint32_t)frame->data[4] | ((uint32_t)frame->data[5] << 8) |
                                    ((uint32_t)frame->data[6] << 16) | ((uint32_t)frame->data[7] << 24));
            state.pos    = (raw * 0.01) * (M_PI / 180.0);
            state.online = true;
            pos_valid_   = true;
            return true;
        }
        return false;
    }

    bool isMyFrame(const struct can_frame* frame) override {
        if (frame->can_id & CAN_EFF_FLAG) return false;
        return frame->can_id == (0x240 + id);
    }

    // 0x92 절대위치 응답을 한 번이라도 받아야 sync 허용
    // 0x92는 모터별 라운드로빈(1모터/사이클)으로 전송 → 4모터 동시 응답 충돌 방지
    bool hasValidPosition() const override { return pos_valid_; }

    // Control() 직후 항상 0.01° 정밀 위치를 별도로 조회 (모드 무관)
    bool RequestExtra(int so) override {
        struct can_frame frame{};
        if (!packReadAngle(&frame)) return false;
        return write(so, &frame, sizeof(frame)) == (ssize_t)sizeof(frame);
    }

private:
    bool pos_valid_ = false; // 0x92 절대위치 응답을 한 번이라도 받았는지

    // 토크(전류) 폐루프 제어 (0xA1)
    bool packTorqueControl(struct can_frame* frame, const MotorCommand& cmd) {
        memset(frame, 0, sizeof(struct can_frame));
        frame->can_id  = 0x140 + id;
        frame->can_dlc = 8;

        const int16_t iq_control = (int16_t)std::lround(cmd.torque * 100.0); // torque[A] 그대로 0.01A/LSB

        frame->data[0] = 0xA1;
        frame->data[1] = 0x00;
        frame->data[2] = 0x00;
        frame->data[3] = 0x00;
        frame->data[4] = (uint8_t)(iq_control & 0xFF);
        frame->data[5] = (uint8_t)((iq_control >> 8) & 0xFF);
        frame->data[6] = 0x00;
        frame->data[7] = 0x00;
        return true;
    }

    // 절대위치 폐루프 제어 (0xA4) — 멀티턴 위치 + 속도제한, kp/kd 없음 (펌웨어 내장 PI)
    bool packPositionControl(struct can_frame* frame, const MotorCommand& cmd) {
        memset(frame, 0, sizeof(struct can_frame));
        frame->can_id  = 0x140 + id;
        frame->can_dlc = 8;

        const int32_t angle_control = (int32_t)std::lround(cmd.pos * (180.0 / M_PI) * 100.0); // 0.01°/LSB
        const double  max_speed_dps = std::min(std::abs(cmd.vel) * (180.0 / M_PI), 65535.0);
        const uint16_t max_speed    = (uint16_t)max_speed_dps; // 1dps/LSB, 0 = 속도제한 없음(PI 결과 그대로)

        frame->data[0] = 0xA4;
        frame->data[1] = 0x00;
        frame->data[2] = (uint8_t)(max_speed & 0xFF);
        frame->data[3] = (uint8_t)((max_speed >> 8) & 0xFF);
        frame->data[4] = (uint8_t)(angle_control & 0xFF);
        frame->data[5] = (uint8_t)((angle_control >> 8) & 0xFF);
        frame->data[6] = (uint8_t)((angle_control >> 16) & 0xFF);
        frame->data[7] = (uint8_t)((angle_control >> 24) & 0xFF);
        return true;
    }

    // Read Multi-Turn Angle (0x92) — 조회 전용, 응답은 parseFeedback에서 처리
    bool packReadAngle(struct can_frame* frame) {
        memset(frame, 0, sizeof(struct can_frame));
        frame->can_id  = 0x140 + id;
        frame->can_dlc = 8;
        frame->data[0] = 0x92;
        return true;
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