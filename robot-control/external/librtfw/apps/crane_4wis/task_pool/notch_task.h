#include <cmath>


namespace Filters {

// struct BiquadNotch {
//     double b0=1, b1=0, b2=1, a1=0, a2=0;
//     double x1=0, x2=0, y1=0, y2=0;

//     void set(double fs, double f0, double Q){
//         const double w0 = 2*M_PI*f0/fs;
//         const double cw = std::cos(w0), sw = std::sin(w0);
//         const double alpha = sw/(2.0*Q);

//         double a0 = 1 + alpha;
//         b0 =  1;        b1 = -2*cw;    b2 =  1;
//         a1 = -2*cw;     a2 =  1 - alpha;

//         // normalize
//         b0/=a0; b1/=a0; b2/=a0; a1/=a0; a2/=a0;
//         reset();
//     }

//     void reset(){ x1=x2=y1=y2=0; }

//     double step(double x){
//         const double y = b0*x + b1*x1 + b2*x2 - a1*y1 - a2*y2;
//         x2=x1; x1=x; y2=y1; y1=y;
//         return y;
//     }
// };
struct BiquadNotch {
    // (동일 API 유지를 위해) 계수 멤버는 그대로 둡니다.
    double b0=1, b1=0, b2=1, a1=0, a2=0;

    // 상태공간용 내부 상태 x = [x1, x2]^T
    double x1=0, x2=0;

    // y = Cx + D u 의 C, D (전처리해 둡니다)
    double c1=0, c2=0, d0=1;

    void set(double fs, double f0, double Q){
        const double w0 = 2*M_PI*f0/fs;
        const double cw = std::cos(w0), sw = std::sin(w0);
        const double alpha = sw/(2.0*Q);

        const double a0 = 1.0 + alpha;

        // (정규화 전) 표준 biquad 노치 계수
        double _b0 =  1.0;
        double _b1 = -2.0*cw;
        double _b2 =  1.0;
        double _a1 = -2.0*cw;
        double _a2 =  1.0 - alpha;

        // 정규화 (a0 = 1로)
        b0 = _b0/a0;  b1 = _b1/a0;  b2 = _b2/a0;
        a1 = _a1/a0;  a2 = _a2/a0;

        // 상태공간 실현 (제어가능 표준형)
        // x[k+1] = A x[k] + B u[k],  y[k] = C x[k] + D u[k]
        // A = [[-a1, -a2],
        //      [  1,   0]]
        // B = [1, 0]^T
        // D = b0
        // C = [b1 - a1*b0,  b2 - a2*b0]
        d0 = b0;
        c1 = b1 - a1*b0;
        c2 = b2 - a2*b0;

        reset();
    }

    void reset(){ x1 = 0.0; x2 = 0.0; }

    double step(double u){
        // 출력: y = Cx + D u
        const double y = c1*x1 + c2*x2 + d0*u;

        // 다음 상태: x+ = A x + B u
        const double nx1 = -a1*x1 - a2*x2 + u;
        const double nx2 =  x1;

        x1 = nx1;
        x2 = nx2;
        return y;
    }
};

}

#include "rtfw/task.h"
#include "util.hpp"

#include <eigen3/Eigen/Dense>
#include <manif/manif.h>


using namespace rtfw::rt;

namespace task_pool {

    class NotchFilterTask : public ITask {
    public:
        const char* getName() const override { return "NotchFilter"; }
        void setup(TaskRegistry& r) override {
            r.add_dependency(dr_pose);
            r.add_dependency(dr_twist);
            r.add_dependency(dw_pose);
            r.add_dependency(dw_twist);
            r.add_dependency(p_freq);
            r.add_dependency(p_zeta);
            r.add_dependency(p_gain);
            r.add_dependency(p_mask);
        }
        void initialize() override {
            const double fs = getFrequency();
            for (auto& filter: filters) {
                filter.set(fs, p_freq.read(), p_gain.read());
            }
            for (auto& filter: vel_filters) {
                filter.set(fs, p_freq.read(), p_gain.read());
            }
        }
        void execute() override {   
            auto mask = p_mask.read();
            dr_pose.on_update([&](const manif::SE3Tangentd& E_in) {
                auto v = E_in.coeffs();
                for (auto i=0; i<6; i++) {
                    if (!mask[i]) continue;
                    v(i) = filters[i].step(v(i));                    
                }
                dw_pose.write(manif::SE3Tangentd(v));
            });


            dr_twist.on_update([&](const manif::SE3Tangentd& V_in) {
                auto v = V_in.coeffs();
                for (auto i=0; i<6; i++) {
                    if (!mask[i]) continue;
                    v(i) = vel_filters[i].step(v(i));                    
                }
                dw_twist.write(manif::SE3Tangentd(v));
            });
        }
    private:
        DataReader<manif::SE3Tangentd> dr_pose{"pose_error"};
        DataReader<manif::SE3Tangentd> dr_twist{"marker_iekf/vel_target"};
        DataWriter<manif::SE3Tangentd> dw_pose{"notch/pose_error"};
        DataWriter<manif::SE3Tangentd> dw_twist{"notch/vel_target"};

        Parameter<double> p_freq{"param.notch.freq", 0.55};
        Parameter<double> p_zeta{"param.notch.zeta", 0.3};
        Parameter<double> p_gain{"param.notch.gain", 5};
        Parameter<std::array<bool, 6>> p_mask{"param.notch.mask", {false, false, false, false, false, false}};

    private:
        Filters::BiquadNotch filters[6];
        Filters::BiquadNotch vel_filters[6];
    };
};