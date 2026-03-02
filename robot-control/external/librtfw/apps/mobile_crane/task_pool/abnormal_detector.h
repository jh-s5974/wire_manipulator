#pragma once

#include <rtfw/task.h>
#include "eigen3/Eigen/Dense"
// #include "frame.hpp"
#include <manif/manif.h>
#include "util.hpp"

using namespace rtfw;
using namespace rtfw::rt;
const static double EPS = 1e-9;

namespace task_pool {

    class AbnormalDetector : public ITask {
    public:
        const char* getName() const override { return "Abnormal_Detector"; }
        void setup(TaskRegistry& r) override {
            r.add_dependency(dw_state);
            r.add_dependency(dw_abnormal);
            r.add_dependency(dr_start);
            r.add_dependency(dr_stop);
            r.add_dependency(dr_error);
            r.add_dependency(dr_body);
            r.add_dependency(dr_right_foot);
            r.add_dependency(dr_left_foot);
            r.add_dependency(dr_body_vel);
        }
        void execute() override {
            dr_start.on_update([this]() {
                if (!active) {
                    active = true;
                    perr.lin().setZero();
                    perr.ang().setZero();
                    getLogger()->info("[{}] state=on", getName());
                    result.abnormal = false;
                    // result.dcm_stable = true;
                }
            });
            dr_stop.on_update([this]() {
                if (active) {
                    active = false;
                    getLogger()->info("[{}] state=off", getName());
                }
            });

            dr_body.on_update([this](const manif::SE3d& data) {
                body = data;
            });
            dr_left_foot.on_update([this](const manif::SE3d& data) {
                left_foot = data;
            });
            dr_right_foot.on_update([this](const manif::SE3d& data) {
                right_foot = data;
            });
            dr_body_vel.on_update([this](const manif::SE3Tangentd& data) {
                body_vel = data;

                auto freq = std::sqrt(com_adjust*(body.translation().z() - std::min(left_foot.translation().z(), right_foot.translation().z()))/9.81);
                auto dcm = body.translation().head(2) + freq*body_vel.lin().head(2);
                auto origin = (left_foot.translation().head(2) + right_foot.translation().head(2))*0.5;
                auto local_com = body.translation().head(2) - origin;
                auto local_dcm = dcm - origin;
                auto local_left_foot = left_foot.translation().head(2) - origin;
                auto local_right_foot = right_foot.translation().head(2) - origin;
                
                const double sp_gain = 1.2;
                const double flx = 0.240/2*sp_gain;
                const double fly = 0.160/2*sp_gain;
                std::vector<Eigen::Vector2d> foot = {
                    {flx, fly},
                    {-flx, fly},
                    {-flx, -fly},
                    {flx, -fly}
                };

                std::vector<Eigen::Vector2d> sp;
                auto left_foot_yaw = left_foot.rotation().block<2, 2>(0, 0);
                auto right_foot_yaw = right_foot.rotation().block<2, 2>(0, 0);
                sp.push_back(local_left_foot + left_foot_yaw*foot[0]);
                sp.push_back(local_left_foot + left_foot_yaw*foot[1]);
                sp.push_back(local_left_foot + left_foot_yaw*foot[2]);
                sp.push_back(local_left_foot + left_foot_yaw*foot[3]);
                sp.push_back(local_right_foot + right_foot_yaw*foot[0]);
                sp.push_back(local_right_foot + right_foot_yaw*foot[1]);
                sp.push_back(local_right_foot + right_foot_yaw*foot[2]);
                sp.push_back(local_right_foot + right_foot_yaw*foot[3]);
                result.supportpolygon = convexHull(sp);
                result.dcm_stable = isInsideConvex(result.supportpolygon, local_dcm);
                result.global_dcm = dcm;
                result.local_dcm = local_dcm;

                // PERIODIC_CALL(printf("abnormal detector: dcm=(%.0lf mm, %.0lf mm)\n", local_dcm.x()*1e3, local_dcm.y()*1e3), 1s);
            });

            dr_error.on_update([this](const manif::SE3Tangentd& data) {
                if (!active)
                    return;
                auto& error = data;
                result.abnormal = std::abs(error.ang().x()) > roll_limit*M_PI/180 || std::abs(error.ang().y()) > pitch_limit*M_PI/180;
                if (result.abnormal) {
                    getLogger()->info("[{}] pitch={:.0f} deg, roll={:.0f} deg", getName(), error.ang().y()*180/M_PI, error.ang().x()*180/M_PI);
                }

                // if (!result.dcm_stable) {
                //     result.abnormal = true;
                //     printf("abnormal detector: dcm not stable\n");
                // }

                manif::SE3Tangentd derr;
                derr.lin() = (error.lin() - perr.lin())*30.0;
                derr.ang() = (error.ang() - perr.ang())*30.0;
                
                // if (derr.lin().cwiseAbs().maxCoeff() > 10.0) { // m/s 
                //   result.abnormal = true;
                //   printf("abnnormal detector: len_vel=%.2lf > 10.0 m/s", derr.lin().cwiseAbs().maxCoeff());
                // }
            
                // if (derr.ang().cwiseAbs().maxCoeff() > 5.0) { // rad/s
                //   result.abnormal = true;
                //   printf("abnnormal detector: ang_vel=%.2lf > 5.0 rad/s", derr.ang().cwiseAbs().maxCoeff());
                // }

                PERIODIC_CALL(
                    getLogger()->info("[{}] pitch={:.0f} deg, roll={:.0f} deg", getName(), error.ang().y()*180/M_PI, error.ang().x()*180/M_PI);
                , 1s);

                perr = error;
                dw_abnormal.write(result.abnormal);
            });

            dw_state.write(active);
        }
    private:
        DataWriter<bool> dw_state{"ad_state"};
        DataWriter<bool> dw_abnormal{"abnormal"};
        DataReader<Signal> dr_start{"tracking_start"};
        DataReader<Signal> dr_stop{"tracking_stop"};
        DataReader<manif::SE3Tangentd> dr_error{"pose_error"};
        DataReader<manif::SE3d> dr_body{"pose_g_body"};
        DataReader<manif::SE3d> dr_right_foot{"pose_g_right_foot"};
        DataReader<manif::SE3d> dr_left_foot{"pose_g_left_foot"};
        DataReader<manif::SE3Tangentd> dr_body_vel{"g_body_vel"};

    private:
        double pitch_limit = 20;
        double roll_limit = 20;
        double com_adjust = 0.5;

        struct {
            bool abnormal;
            bool dcm_stable;
            Eigen::Vector2d global_dcm;
            Eigen::Vector2d local_dcm;
            std::vector<Eigen::Vector2d> supportpolygon;
        } result;
    
    private:
        bool active = false;
        Eigen::Vector3d orient;
        manif::SE3Tangentd perr;
        Eigen::Vector3d orient_ref;
        
        manif::SE3d body;
        manif::SE3Tangentd body_vel;
        manif::SE3d right_foot;
        manif::SE3d left_foot;


        // cross product for 2D vectors
        double cross2D(const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
            return a.x() * b.y() - a.y() * b.x();
        }

        double cross(const Eigen::Vector2d& O, const Eigen::Vector2d& A, const Eigen::Vector2d& B) {
            return cross2D(A - O, B - O);
        }

        // Convex hull with Eigen::Vector2d
        std::vector<Eigen::Vector2d> convexHull(std::vector<Eigen::Vector2d>& P) {
            int n = P.size(), k = 0;
            std::sort(P.begin(), P.end(), [](const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
                return a.x() < b.x() || (std::abs(a.x() - b.x()) < EPS && a.y() < b.y());
            });

            std::vector<Eigen::Vector2d> H(2 * n);
            for (int i = 0; i < n; ++i) {
                while (k >= 2 && cross(H[k - 2], H[k - 1], P[i]) <= 0) k--;
                H[k++] = P[i];
            }
            for (int i = n - 2, t = k + 1; i >= 0; --i) {
                while (k >= t && cross(H[k - 2], H[k - 1], P[i]) <= 0) k--;
                H[k++] = P[i];
            }
            H.resize(k - 1);
            return H;
        }

        // 내부 판단
        bool isInsideConvex(const std::vector<Eigen::Vector2d>& poly, const Eigen::Vector2d& p) {
            for (int i = 0; i < poly.size(); ++i) {
                const Eigen::Vector2d& a = poly[i];
                const Eigen::Vector2d& b = poly[(i + 1) % poly.size()];
                if (cross(a, b, p) < -EPS)
                    return false;
            }
            return true;
        }
    };
};