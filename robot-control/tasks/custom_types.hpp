#pragma once


namespace custom_types {
    struct Imu {
        struct {
            double x;
            double y;
            double z;
        } acc;
        struct {
            double x;
            double y;
            double z;
        } gyro;
        struct {        
            double w;
            double x;
            double y;
            double z;
        } orientation;
    };

    struct MotorState {
        char name[32];
        double pos;
        double vel;
        double torque;
        double temp;
        int status;
    };

    struct MotorCmd {
        char name[32];
        double pos;
        double vel;
        double torque;
        double kp;
        double kd;
        double duration_ms = 0.0;  // interpolation duration (ms), 0 = immediate
    };


}